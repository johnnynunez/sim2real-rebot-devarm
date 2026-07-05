# Isaac Sim code snippets for this asset

## A. Move the arm live in the GUI (Script Editor)

Load the asset, press **Play FIRST**, then run:

```python
import numpy as np, math, omni.kit.app, omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager

print("engine:", SimulationManager.get_active_physics_engine())  # expect 'newton' or 'physx'

tl = omni.timeline.get_timeline_interface()
if not tl.is_playing():
    tl.play(); print("Play started — wait 1 s and re-run this cell")

art = Articulation("/tn__00armrs_asmv3_hJ6D/Geometry/base_link")
assert art.num_dofs == 8, "0 DOFs: press Play first, then re-run (recreate after every Stop)"

lower, upper = [x.numpy()[0] for x in art.get_dof_limits()]
center = (lower + upper) / 2
amp = np.minimum(0.3, 0.4 * (upper - lower) / 2)

t = 0.0
def on_update(e):
    global t
    t += 1/60
    tgt = center + amp * np.sin(2 * math.pi * 0.2 * t)   # gentle 0.2 Hz sweep
    art.set_dof_position_targets(tgt.reshape(1, -1).astype(np.float32))

sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
# stop with: sub.unsubscribe()
```

## B. Close / open the gripper (Script Editor, Play active)

```python
import numpy as np, omni.kit.app
from isaacsim.core.experimental.prims import Articulation

art = Articulation("/tn__00armrs_asmv3_hJ6D/Geometry/base_link")
assert art.num_dofs == 8
names = list(art.dof_names)
gl, gr = names.index("joint_left"), names.index("joint_right")
lower, upper = [x.numpy()[0] for x in art.get_dof_limits()]

tgt = art.get_dof_positions().numpy()[0].copy()
state = {"phase": "open", "i": 0, "steps": 90}   # 1.5 s per phase @60 Hz

def on_update(e):
    s = state
    a = min(1.0, (s["i"] + 1) / s["steps"])
    dest = upper if s["phase"] == "open" else lower
    for j in (gl, gr):
        tgt[j] = (1 - a) * tgt[j] + a * dest[j]
    art.set_dof_position_targets(tgt.reshape(1, -1).astype(np.float32))
    s["i"] += 1
    if s["i"] >= s["steps"] + 30:                 # 0.5 s hold, then flip
        s["phase"] = "close" if s["phase"] == "open" else "open"
        s["i"] = 0

sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
# stop with: sub.unsubscribe()
```

## C. Record the gripper cycle headless (real RTX frames -> MP4)

`scripts/record_gripper.py` in this package. Run:

```bash
export ISAACSIM_PATH=/path/to/isaac-sim/_build/linux-x86_64/release
$ISAACSIM_PATH/python.sh scripts/record_gripper.py
ffmpeg -y -framerate 30 -i /tmp/armtrack/gripper_frames/f_%05d.png \
       -c:v libx264 -pix_fmt yuv420p -crf 20 gripper_newton.mp4
```

(Edit the USD constant at the top to point at this package's .usda; engine is Newton
via the newton kit experience.)

## Gotchas (hard-won)

- Create `Articulation` AFTER `timeline.play()` or you get a 0-DOF view and
  `broadcast ... (1,0) vs (1,8)` errors. Recreate it after every Stop->Play.
- Newton headless needs `experience=isaacsim.exp.full.newton.kit`; enabling the
  extension after boot does NOT create the Newton tensor backend.
- Gain Tuner Snap-to-Limits: run ONE joint per sequence (GUI default). Commanding
  all joints to limits simultaneously folds the arm into itself.
- This package ships hybrid colliders: convexHull on body/links (engine-agreement
  validated), convexDecomposition on gripper_end/left/right (hulls filled the
  finger gap and caused false premature contact). Default MuJoCo-Warp contact caps
  are sufficient for this hybrid; do NOT raise nconmax above the allocated
  rigid_contact_max or every step errors.
