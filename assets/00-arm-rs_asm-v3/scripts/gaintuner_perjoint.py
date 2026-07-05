"""Run the REAL Gain Tuner SnapToLimitsTest headless.

Usage: python.sh gaintuner_run.py <usd> <engine: newton|physx> <out_json>
"""
import json
import os
import sys

USD, ENGINE, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
DT = 0.002

from isaacsim import SimulationApp

_release = os.environ["ISAACSIM_PATH"]
_exp = (
    os.path.join(_release, "apps", "isaacsim.exp.full.newton.kit")
    if ENGINE == "newton"
    else ""
)
app = SimulationApp({"headless": True}, experience=_exp)

import numpy as np
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager

app_utils.enable_extension("isaacsim.robot_setup.gain_tuner")
app.update()

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
    app.update()

SimulationManager.setup_simulation(dt=DT, device="cpu")

import omni.usd
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
root = None
for p in stage.Traverse():
    if any("ArticulationRootAPI" in str(s) for s in p.GetAppliedSchemas()):
        root = str(p.GetPath())
        break
assert root, "no articulation root"

from isaacsim.core.experimental.prims import Articulation

art = Articulation(root)
app_utils.play(commit=True)
for _ in range(10):
    app.update()
assert str(SimulationManager.get_active_physics_engine()).lower() == ENGINE

from isaacsim.robot_setup.gain_tuner.gains_tuner import JointMode
from isaacsim.robot_setup.gain_tuner.snap_to_limits import SnapToLimitsTest

num = art.num_dofs
names = list(art.dof_names)
indices = list(range(num))
modes = {i: int(JointMode.POSITION) for i in indices}

# PER-JOINT sequences: one joint per test run (official GUI methodology).
# Other joints hold their home pose via their own drives.
all_metrics = {}
total_steps = 0
MAX_STEPS = int(120.0 / DT)  # per-joint cap
_only = os.environ.get("ONLY_JOINTS")
if _only:
    _sel = [i for i, n in enumerate(names) if n in _only.split(",")]
    indices = _sel
    print("[ONLY]", [names[i] for i in indices], flush=True)
for j in indices:
    test = SnapToLimitsTest()
    test.setup(art, [j], {j: modes[j]}, {"hold_duration": 1.0, "tolerance": 0.01})
    gen = test.run()
    result = None
    steps = 0
    try:
        while steps < MAX_STEPS:
            test._step = DT
            next(gen)
            app.update()
            steps += 1
            if steps % 2000 == 0:
                import time as _t
                print(f"PROGRESS joint={names[j]} steps={steps} sim_t={steps*DT:.1f}s wall={_t.time():.0f}", flush=True)
    except StopIteration as si:
        result = si.value
    assert result is not None, f"joint {names[j]} did not finish within {MAX_STEPS} steps"
    for idx, m in (result.joint_metrics or {}).items():
        all_metrics[int(idx)] = m
    total_steps += steps
    print(f"JOINT_DONE {names[j]} status={result.joint_metrics.get(j, {}).get('status')}", flush=True)

steps = total_steps

class _R:  # adapter for the serialization block below
    joint_metrics = all_metrics
result = _R()

metrics = {}
for idx, m in (result.joint_metrics or {}).items():
    clean = {}
    for k, v in m.items():
        if isinstance(v, (np.floating, np.integer)):
            v = v.item()
        elif isinstance(v, np.ndarray):
            v = v.tolist()
        clean[k] = v
    metrics[names[int(idx)]] = clean

out = {
    "engine": ENGINE,
    "usd": USD,
    "dt": DT,
    "sim_steps": steps,
    "sim_time_s": steps * DT,
    "joint_metrics": metrics,
}
json.dump(out, open(OUT, "w"), indent=1, default=str)
print("WROTE", OUT)
app_utils.stop()
app.close()
