"""Record the gripper closing/opening under Newton. Real RTX frames -> PNG -> MP4."""
import math
import os

USD = "/tmp/rebot_pkg/00-arm-rs_asm-v3/00-arm-rs_asm-v3.usda"
OUTDIR = "/tmp/armtrack/gripper_frames"
FPS = 30
DT = 1.0 / 60.0

from isaacsim import SimulationApp

_release = os.environ["ISAACSIM_PATH"]
_exp = os.path.join(_release, "apps", "isaacsim.exp.full.newton.kit")
app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 1280, "height": 720}, experience=_exp)

import numpy as np
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager

SimulationManager.switch_physics_engine("newton")
stage_utils.open_stage(USD)
while stage_utils.is_stage_loading():
    app.update()
for _ in range(30):
    app.update()
SimulationManager.setup_simulation(dt=DT, device="cpu")

import omni.usd
from pxr import Gf, UsdGeom, UsdLux

stage = omni.usd.get_context().get_stage()

# lights
dome = UsdLux.DomeLight.Define(stage, "/World_Lights/Dome")
dome.CreateIntensityAttr(1000.0)
sun = UsdLux.DistantLight.Define(stage, "/World_Lights/Sun")
sun.CreateIntensityAttr(3000.0)
UsdGeom.XformCommonAPI(sun.GetPrim()).SetRotate(Gf.Vec3f(-45, 30, 0))

from isaacsim.core.experimental.prims import Articulation

art = Articulation("/tn__00armrs_asmv3_hJ6D/Geometry/base_link")
app_utils.play(commit=True)
for _ in range(10):
    app.update()
assert str(SimulationManager.get_active_physics_engine()).lower() == "newton"
assert art.num_dofs == 8, art.num_dofs

names = list(art.dof_names)
gl, gr = names.index("joint_left"), names.index("joint_right")
lower, upper = [x.numpy()[0].astype(float) for x in art.get_dof_limits()]
q0 = art.get_dof_positions().numpy()[0].astype(float)

# find gripper world position for camera aim (end of chain prim)
cache = UsdGeom.XformCache()
tip = stage.GetPrimAtPath(
    "/tn__00armrs_asmv3_hJ6D/Geometry/base_link/link1/link2/link3/link4/link5/link6/gripper_end"
)
tip_pos = Gf.Vec3d(cache.GetLocalToWorldTransform(tip).ExtractTranslation()) if tip else Gf.Vec3d(0, 0, 0.5)

cam = UsdGeom.Camera.Define(stage, "/World_Cams/grip")
cam.CreateFocalLengthAttr(24.0)
eye = Gf.Vec3d(0.9, -0.9, 0.55)
m = Gf.Matrix4d()
m.SetLookAt(eye, Gf.Vec3d(0.15, 0.0, 0.22), Gf.Vec3d(0, 0, 1))
minv = m.GetInverse()
xf = UsdGeom.XformCommonAPI(cam.GetPrim())
xf.SetTranslate(Gf.Vec3d(minv.ExtractTranslation()))
rot = minv.ExtractRotation().Decompose(Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
xf.SetRotate(Gf.Vec3f(rot[0], rot[1], rot[2]))

from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file

vp = get_active_viewport()
vp.camera_path = "/World_Cams/grip"
for _ in range(60):  # RTX warmup
    app.update()

os.makedirs(OUTDIR, exist_ok=True)

# motion plan: hold(0.5s) -> close(1.5s) -> hold closed(0.5s) -> open(1.5s) -> hold(0.5s)
# gripper: lower = closed=0? URDF lower=0 upper=0.05/0.0715 -> assume upper=open, lower=closed?
# initial q0 is 0 (= lower). We drive to UPPER (apart) then back to LOWER (together).
phases = [
    ("hold", 0.5, None),
    ("open", 1.5, "upper"),
    ("hold", 0.5, None),
    ("close", 1.5, "lower"),
    ("hold", 0.7, None),
]

tgt = q0.copy()
frame_i = 0
log = []
for phase, dur, dest in phases:
    steps = int(dur / DT)
    start_l, start_r = tgt[gl], tgt[gr]
    for i in range(steps):
        a = (i + 1) / steps
        if dest == "upper":
            tgt[gl] = start_l + a * (upper[gl] - start_l)
            tgt[gr] = start_r + a * (upper[gr] - start_r)
        elif dest == "lower":
            tgt[gl] = start_l + a * (lower[gl] - start_l)
            tgt[gr] = start_r + a * (lower[gr] - start_r)
        art.set_dof_position_targets(tgt.reshape(1, -1).astype(np.float32))
        app.update()
        if frame_i % 2 == 0:  # 60Hz sim -> 30fps video
            capture_viewport_to_file(vp, f"{OUTDIR}/f_{frame_i//2:05d}.png")
            app.update()
        frame_i += 1
    q = art.get_dof_positions().numpy()[0]
    log.append((phase, float(q[gl]), float(q[gr]), float(tgt[gl]), float(tgt[gr])))
    print(f"PHASE {phase}: left={q[gl]:.4f}/{tgt[gl]:.4f} right={q[gr]:.4f}/{tgt[gr]:.4f}", flush=True)

# let captures flush
for _ in range(30):
    app.update()
print("FRAMES", frame_i // 2, flush=True)
app_utils.stop()
app.close()
