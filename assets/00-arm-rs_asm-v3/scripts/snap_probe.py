"""Replicate issue #681 condition: command joint1 to its lower limit, watch settle.
Usage: python.sh snap_probe.py <usd> <engine> <out_json>
"""
import json, math, os, sys
USD, ENGINE, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
DT = 0.002
from isaacsim import SimulationApp
_release = os.environ["ISAACSIM_PATH"]
_exp = os.path.join(_release, "apps", "isaacsim.exp.full.newton.kit") if ENGINE == "newton" else ""
app = SimulationApp({"headless": True}, experience=_exp)
import numpy as np
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
if ENGINE == "newton":
    # convexDecomposition colliders generate >200 contacts; raise MuJoCo-Warp caps
    from isaacsim.physics.newton.impl import extension as _next
    _ns = getattr(_next, "_newton_stage", None)
    if _ns is not None:
        _ns.cfg.solver_cfg.nconmax = 4000
        _ns.cfg.solver_cfg.njmax = 12000
        print(f"[CFG] nconmax={_ns.cfg.solver_cfg.nconmax} njmax={_ns.cfg.solver_cfg.njmax}", flush=True)

SimulationManager.switch_physics_engine(ENGINE)
stage_utils.open_stage(USD)
while stage_utils.is_stage_loading():
    app.update()
for _ in range(30):
    app.update()  # settle: payload composition can lag is_stage_loading()
SimulationManager.setup_simulation(dt=DT, device="cpu")
import omni.usd
from pxr import UsdPhysics
stage = omni.usd.get_context().get_stage()

# optional: disable ALL collisions (self-collision + anything else)
if os.environ.get("NOCOL") == "1":
    ncol = 0
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr().Set(False)
            ncol += 1
    print(f"[NOCOL] disabled {ncol} colliders", flush=True)
root = None
for _ in range(50):
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.ArticulationRootAPI) or any("ArticulationRootAPI" in str(s) for s in p.GetAppliedSchemas()):
            root = str(p.GetPath()); break
    if root: break
    app.update()
print("DBG stage:", stage.GetRootLayer().identifier, "prims:", sum(1 for _ in stage.Traverse()), flush=True)
assert root, "no articulation root found"
from isaacsim.core.experimental.prims import Articulation
art = Articulation(root)
app_utils.play(commit=True)
for _ in range(10):
    app.update()
assert str(SimulationManager.get_active_physics_engine()).lower() == ENGINE
lower, upper = [l.numpy()[0].astype(float) for l in art.get_dof_limits()]
q0 = art.get_dof_positions().numpy()[0].astype(float)
tgt = q0.copy()
tgt[0] = lower[0]  # joint1 to lower limit (~ -2.8 rad), others hold
traj, vels = [], []
for i in range(int(10.0 / DT)):
    art.set_dof_position_targets(tgt.reshape(1, -1).astype(np.float32))
    app.update()
    q = art.get_dof_positions().numpy()[0]
    v = art.get_dof_velocities().numpy()[0]
    traj.append(float(q[0])); vels.append(float(v[0]))
traj = np.array(traj); vels = np.array(vels)
# settle: |q-tgt|<0.02 and |v|<0.03 sustained to end
settled = None
for i in range(len(traj)):
    if np.all(np.abs(traj[i:] - tgt[0]) < 0.02) and np.all(np.abs(vels[i:]) < 0.03):
        settled = i * DT
        break
out = {
    "engine": ENGINE, "target": float(tgt[0]), "q0": float(q0[0]),
    "min_pos": float(traj.min()), "max_overshoot_past_limit": float(max(0.0, tgt[0] - traj.min())),
    "final_pos": float(traj[-1]), "final_vel": float(vels[-1]),
    "settle_time_s": settled,
    "traj_sampled_every_0p25s": [round(float(x), 3) for x in traj[:: int(0.25 / DT)]],
}
json.dump(out, open(OUT, "w"), indent=1)
print("WROTE", OUT)
app_utils.stop(); app.close()
