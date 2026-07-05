# reBot DevArm (00-arm-rs_asm-v3) — Newton/PhysX validated USD package

**Date:** 2026-07-03
**Source URDF:** https://github.com/vectorBH6/reBotArm_control_py (urdf/00-arm-rs_asm-v3)
**Converter:** urdf-usd-converter v0.2.0 (newton-physics, newton-usd-schemas 0.3.1) — exact URDF inertia round-trip
**Validated on:** Isaac Sim 6.0 develop (Kit 110), Newton (MuJoCo-Warp solver) + PhysX, dt=0.002, RTX PRO 6000 Blackwell

## Contents

- `00-arm-rs_asm-v3.usda` — asset root (Atomic Component, payloaded)
- `Payload/Physics.usda` — joints, collisions (convexHull), drives (PhysX-original gains), exact inertia tensors
- `Payload/MDLMaterials.usda` + bindings — original Blender/SolidWorks material assignment
  (Aluminum_Anodized, Aluminum_Anodized_Black, ABS_Hard_Leather, ABS_Hard_Leather_Pop_Lime)

## Drive gains (authored, validated on BOTH engines)

| joint | stiffness | damping |
|---|---|---|
| joint1 | 500 | 60 |
| joint2 | 1500 | 96 |
| joint3 | 1000 | 76 |
| joint4 | 150 | 18 |
| joint5 | 80 | 10 |
| joint6 | 50 | 7 |
| joint_left/right | 100 | 4 |

Do NOT use the June "Newton-tuned" soft gains (fn/zeta recipe) — they degrade dynamic
tracking (joint2 resonates x1.56 @0.5Hz) and under-hold gravity. Retired.

## Validation summary (evidence JSONs in /tmp/armtrack/, scripts alongside)

| Test | Newton | PhysX |
|---|---|---|
| Gain Tuner Snap-to-Limits per-joint, NO collision | 8/8 pass, err 0.000 | 8/8 pass, err 0.000 |
| Gain Tuner per-joint, WITH collision (convexHull) | 4 pass / 4 blocked | identical pattern (8/8 agreement) |
| Snap joint1 to both limits | 0.00 overshoot, settle 0.11 s | 0.00 overshoot, settle 0.10 s |
| Sinusoid tracking 0.5 Hz (6 arm joints) | rms 0.006–0.013 rad | rms 0.005–0.011 rad |
| Step 0.5 rad joint1 | 0.0 % overshoot | 0.0 % overshoot |

The `blocked` verdicts (joint2 upper, joint4 lower, gripper finger extremes) are
conservative convexHull artifacts and/or genuine self-contact near URDF limit extremes;
both engines agree, and without collision all limits are exactly reachable.

## Hybrid collider configuration (shipped)

- **Body + links (7 prims): convexHull** — engine-agreement validated (8/8).
- **gripper_end / gripper_left / gripper_right: convexDecomposition** — plain hulls
  fill the finger gap and cause false premature contact when closing. Decomposition
  restores a realistic close. VALIDATED: with decomposed fingers, joint_left/right
  Snap-to-Limits = pass on BOTH engines (collision enabled), and the full close/open
  cycle reaches the exact limits (0.0500->0.0000, 0.0715->0.0002 m) under Newton —
  see evidence/gripper_newton_hybrid.mp4 (real RTX capture) and gt_grip_decomp_*.json.
  Arm regression after the switch: joint3/joint5 still pass (gt_arm_decomp_newton.json).

Full-asset convexDecomposition was evaluated and REJECTED: it fragments engine
agreement on the arm (spurious grazing contacts differ per engine; PhysX arm rms
0.011 -> 0.166 rad) and on Newton overflows default MuJoCo-Warp contact caps.
The hybrid keeps arm parity while fixing the gripper. Note: do not raise nconmax
beyond the allocated rigid_contact_max (sized from the scene) or every step errors.

## Statistical parity analysis (evidence/analysis/)

Sinusoid 0.5 Hz steady-state, ~4000 samples/joint, arm joints 1-6, hull colliders,
identical targets both engines: correlation >= 0.99994 per joint, |Newton-PhysX|
max < 5 mrad, bias ~0. Step response 0 % overshoot on both. Figures fig1-fig5 +
stats.json (gripper rows in stats.json carry a non-comparability caveat; see note).

## Newton runtime note (only if you switch to convexDecomposition)

Default MuJoCo-Warp caps overflow with decomposed colliders (silent contact drops):

```python
from isaacsim.physics.newton.impl import extension as _next
_next._newton_stage.cfg.solver_cfg.nconmax = 4000   # default 200
_next._newton_stage.cfg.solver_cfg.njmax = 12000    # default 1200
```

With the shipped convexHull colliders the defaults are fine.

## History / why this asset replaces the old one

The legacy multi-backend USD (SolidWorks export + physx/physics/mujoco variants) showed
unstable limit-approach transients under Newton (1.65 rad overshoot; with self-collision
feedback this produced the sustained oscillation reported in isaac-sim/IsaacSim#681).
This converter-generated asset shows Newton/PhysX parity on every measured metric with
the same PhysX gains. The June gain-retuning recipe treated the symptom, not the cause.
