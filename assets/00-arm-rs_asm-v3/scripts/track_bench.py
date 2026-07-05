"""Dynamic trajectory-tracking benchmark for the 8-DOF arm.

Usage: python.sh track_bench.py <usd_path> <engine: newton|physx> <out_json>

Protocol (identical for both engines, dt=0.002):
  1. ramp from initial pose to a safe mid-range center pose (2 s)
  2. hold center (1 s)  -> gravity-hold drift metric
  3. sinusoidal tracking, all joints simultaneously (10 s):
       q_t(t) = center + A*sin(2*pi*f*t), f=0.5 Hz
       A = 0.3 rad revolute / 40% half-range prismatic
  4. step response on joint1: +0.5 rad step, 4 s -> overshoot + settle time
Metrics: RMS/max tracking error during sinusoid (steady portion, t>1 period),
         phase lag, hold drift, step overshoot/settling.
"""
import json
import math
import sys

USD_PATH, ENGINE, OUT_JSON = sys.argv[1], sys.argv[2], sys.argv[3]
DT = 0.002

from isaacsim import SimulationApp

import os
_release = os.environ["ISAACSIM_PATH"]
_experience = (
    os.path.join(_release, "apps", "isaacsim.exp.full.newton.kit")
    if ENGINE == "newton"
    else ""
)
simulation_app = SimulationApp({"headless": True}, experience=_experience)

import numpy as np
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager

SimulationManager.switch_physics_engine(ENGINE)

stage_utils.open_stage(USD_PATH)
while stage_utils.is_stage_loading():
    simulation_app.update()
for _ in range(30):
    simulation_app.update()  # settle: payload composition can lag is_stage_loading()

SimulationManager.setup_simulation(dt=DT, device="cpu")

# find articulation root
from pxr import Usd, UsdPhysics
import omni.usd

stage = omni.usd.get_context().get_stage()

# optional: disable ALL collisions (self-collision + anything else)
if os.environ.get("NOCOL") == "1":
    ncol = 0
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr().Set(False)
            ncol += 1
    print(f"[NOCOL] disabled {ncol} colliders", flush=True)

root_path = None
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI) or "ArticulationRootAPI" in str(
        prim.GetAppliedSchemas()
    ):
        root_path = str(prim.GetPath())
        break
print("DBG stage:", stage.GetRootLayer().identifier, "prims:", sum(1 for _ in stage.Traverse()), flush=True)
assert root_path, "no articulation root found"

from isaacsim.core.experimental.prims import Articulation

art = Articulation(root_path)
app_utils.play(commit=True)
for _ in range(10):
    simulation_app.update()

active_engine = SimulationManager.get_active_physics_engine()
assert str(active_engine).lower() == ENGINE.lower(), f"engine mismatch: {active_engine}"
num_dofs = art.num_dofs
dof_names = list(art.dof_names)
q0 = art.get_dof_positions().numpy()[0].astype(float)

# limits in solver units (rad / m)
lower, upper = [l.numpy()[0].astype(float) for l in art.get_dof_limits()]
is_prismatic = np.array([("joint_left" in n) or ("joint_right" in n) for n in dof_names])

center = (lower + upper) / 2.0
amp = np.where(is_prismatic, 0.4 * (upper - lower) / 2.0, 0.3)
# keep sinusoid inside limits
amp = np.minimum(amp, 0.9 * (upper - lower) / 2.0)
freq = 0.5  # Hz

kg, dg = [g.numpy()[0].astype(float) for g in art.get_dof_gains()]

log = {
    "engine_requested": ENGINE,
    "engine_active": str(active_engine),
    "usd": USD_PATH,
    "dt": DT,
    "root": root_path,
    "dof_names": dof_names,
    "limits_lower": lower.tolist(),
    "limits_upper": upper.tolist(),
    "gains_stiffness_solver": kg.tolist(),
    "gains_damping_solver": dg.tolist(),
    "center": center.tolist(),
    "amp": amp.tolist(),
    "freq_hz": freq,
}

def step_sim(target):
    art.set_dof_position_targets(target.reshape(1, -1).astype(np.float32))
    simulation_app.update()
    q = art.get_dof_positions().numpy()[0].astype(float)
    v = art.get_dof_velocities().numpy()[0].astype(float)
    return q, v

# ---- phase 1: ramp to center (2 s)
RAMP_STEPS = int(2.0 / DT)
for i in range(RAMP_STEPS):
    a = (i + 1) / RAMP_STEPS
    tgt = (1 - a) * q0 + a * center
    q, v = step_sim(tgt)

# ---- phase 2: hold center (1 s)
HOLD_STEPS = int(1.0 / DT)
hold_qs = []
for _ in range(HOLD_STEPS):
    q, v = step_sim(center)
    hold_qs.append(q)
hold_qs = np.array(hold_qs)
hold_drift = np.abs(hold_qs - center).max(axis=0)
hold_final_err = np.abs(hold_qs[-1] - center)
log["hold_drift_max"] = hold_drift.tolist()
log["hold_final_err"] = hold_final_err.tolist()

# ---- phase 3: sinusoid tracking (10 s), all joints
SIN_T = 10.0
SIN_STEPS = int(SIN_T / DT)
ts, targets, qs, vs = [], [], [], []
diverged = False
for i in range(SIN_STEPS):
    t = (i + 1) * DT
    tgt = center + amp * np.sin(2 * math.pi * freq * t)
    q, v = step_sim(tgt)
    ts.append(t)
    targets.append(tgt.copy())
    qs.append(q)
    vs.append(v)
    if not np.isfinite(q).all() or np.abs(q - center).max() > 50:
        diverged = True
        break
ts = np.array(ts); targets = np.array(targets); qs = np.array(qs)
log["sin_diverged"] = bool(diverged)
if not diverged:
    steady = ts > (1.0 / freq)  # skip first period
    err = qs[steady] - targets[steady]
    log["sin_rms_err"] = np.sqrt((err ** 2).mean(axis=0)).tolist()
    log["sin_max_abs_err"] = np.abs(err).max(axis=0).tolist()
    # amplitude ratio + phase lag via correlation with target sinusoid
    tt = ts[steady]
    ratios, lags = [], []
    for j in range(num_dofs):
        meas = qs[steady, j] - center[j]
        ref_sin = np.sin(2 * math.pi * freq * tt)
        ref_cos = np.cos(2 * math.pi * freq * tt)
        a_s = 2 * (meas * ref_sin).mean()
        a_c = 2 * (meas * ref_cos).mean()
        meas_amp = math.hypot(a_s, a_c)
        ratios.append(meas_amp / amp[j] if amp[j] > 0 else float("nan"))
        lags.append(math.degrees(math.atan2(-a_c, a_s)))
    log["sin_amp_ratio"] = ratios
    log["sin_phase_lag_deg"] = lags

# ---- phase 4: step response on joint1 (+0.5 rad), 4 s
j = 0
step_tgt = center.copy()
step_tgt[j] += 0.5
STEP_STEPS = int(4.0 / DT)
sq = []
for _ in range(STEP_STEPS):
    q, v = step_sim(step_tgt)
    sq.append(q[j])
sq = np.array(sq)
final = step_tgt[j]
overshoot = (sq.max() - final) / 0.5 * 100.0
band = 0.02 * 0.5  # 2% band
settled_idx = None
for i in range(len(sq)):
    if np.all(np.abs(sq[i:] - final) < band):
        settled_idx = i
        break
log["step_joint"] = dof_names[j]
log["step_overshoot_pct"] = float(overshoot)
log["step_settle_time_s"] = None if settled_idx is None else float(settled_idx * DT)
log["step_final_err"] = float(abs(sq[-1] - final))

# full time series for plotting/statistics
log["ts"] = ts.tolist()
log["targets"] = targets.tolist()
log["qs"] = qs.tolist()
log["step_traj"] = sq.tolist()
log["hold_traj"] = hold_qs.tolist()
with open(OUT_JSON, "w") as f:
    json.dump(log, f)
print("WROTE", OUT_JSON)

app_utils.stop()
simulation_app.close()
