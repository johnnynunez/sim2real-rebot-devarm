# sim2real-rebot-devarm

Sim2Real bridge for the Seeed Studio **reBot DevArm B601-DM** (6-DOF + gripper,
Damiao motors) and **NVIDIA Isaac Sim** (Newton physics).

Verified on: Jetson/GB10 (aarch64), Ubuntu, Isaac Sim 6.0 (develop, Newton engine),
motorbridge 0.4.8, USD asset converted from the official URDF
([vectorBH6/reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py)).

```
 physical arm  --CAN/dm-serial-->  real_to_sim_bridge.py  --UDP:5801-->  Isaac Sim mirror
 (motors passive, move by hand)         (50 Hz JSON)                (set_dof_position_targets)
```

## What works today

- **Real -> Sim live mirror**: move the physical arm by hand and the simulated
  arm follows in real time (motors stay disabled — completely safe).
- **Supervised single-joint moves** (Sim -> Real building block): enable one
  motor in POS_VEL mode with a low velocity limit, move, verify via feedback,
  disable. Verified on joint 1: commanded -0.300 rad, reached -0.270 rad
  under low-speed limit, then cleanly disabled.

## Hardware

| Item | Value |
|---|---|
| Arm | reBot Arm B601-DM (7x Damiao: motors 1-3 DM4340, 4-7 DM4310) |
| Adapter | Damiao USB2CAN serial bridge (`/dev/ttyACM0`, 921600 baud) |
| Motor IDs | CAN ID 1..7, feedback/master ID = motor_id + 0x10 (0x11..0x17) |
| Power | 24 V DC |

Motor IDs and zero calibration must be done first — follow the
[official getting-started guide](https://wiki.seeedstudio.com/rebot_b601_dm_getting_started/)
(factory pre-assembled arms ship with IDs already written).

### RobStride build

The scripts also support arms built from RobStride RS-series motors
(`--vendor robstride`, or set `REBOT_VENDOR=robstride`). Differences vs the
Damiao table above:

| Item | Value |
|---|---|
| Arm | 7x RobStride RS-series (motor models per joint in `scripts/rebot_vendor.py` — defaults are `rs-00` placeholders, set your actual `rs-00`..`rs-06` models) |
| Adapter | any SocketCAN adapter (e.g. PCAN, candleLight/gs_usb), classic CAN at 1 Mbps |
| Motor IDs | CAN ID 1..7, feedback goes to the fixed host ID 0xFD (RobStride convention) |

Bring up the CAN interface (no serial-port permissions needed — the
`dialout`/`chmod` step above is dm-serial only; SocketCAN just needs the
link up):

```bash
sudo ip link set can0 up type can bitrate 1000000
```

Then pass `--vendor robstride --channel can0` to any of the vendor-aware
scripts (`--serial-port`/`--baud` are ignored for robstride):

```bash
python scripts/read_joints.py        --vendor robstride --channel can0
python scripts/real_to_sim_bridge.py --vendor robstride --channel can0 --rate 50
python scripts/move_joint_test.py    --vendor robstride --channel can0 --motor 1 --step -0.3 --vlim 0.5
python scripts/rebot_daemon.py       --vendor robstride --channel can0
```

Mode.POS_VEL maps to the RobStride Position-PP mode, so the daemon's
enable/hold/move/servo paths work unchanged. The daemon's torque-abort
thresholds for robstride are conservative placeholders — tune them in
`scripts/rebot_daemon.py` (`TORQUE_ABORT["robstride"]`) for the actual
RS-series models fitted. The gripper wrap/rehome logic was characterized on
Damiao hardware; re-verify open/close calibration on an RS build.

## Setup

```bash
# 1. Python env (any venv manager works)
uv venv .venv --python 3.12
source .venv/bin/activate
uv pip install -r requirements.txt

# 2. Serial port permissions
sudo usermod -aG dialout $USER   # then re-login (or: sudo chmod 666 /dev/ttyACM0)

# 3. Isaac Sim with the remote code-execution server
cd <IsaacSim>/_build/linux-$(uname -m)/release
./isaac-sim.newton.sh --enable isaacsim.code_editor.python_server
# wait for the app to finish loading; the server listens on 127.0.0.1:8226
```

> The arm USD asset ships in this repo: `assets/00-arm-rs_asm-v3/00-arm-rs_asm-v3.usda`
> (URDF converted with urdf-usd-converter, exact inertia round-trip, validated on
> both Newton and PhysX — see `assets/00-arm-rs_asm-v3/VALIDATION.md`).

## Run the Real -> Sim mirror

```bash
# 1. Load the arm USD into Isaac Sim (shared remote context)
python scripts/isaacsim_client.py --timeout 180 \
    --arg usd_path=$PWD/assets/00-arm-rs_asm-v3/00-arm-rs_asm-v3.usda \
    --file isaac/01_load_arm_stage.py

# 2. Start playback + UDP mirror listener inside Isaac Sim
python scripts/isaacsim_client.py --timeout 150 --file isaac/02_start_mirror.py

# 3. Stream the real arm joints (motors passive — move the arm by hand!)
python scripts/real_to_sim_bridge.py --rate 50

# 4. (optional) check mirror status / packet count
python scripts/isaacsim_client.py --file isaac/03_mirror_status.py
```

The first `play` in Newton compiles CUDA kernels — expect ~1-2 min on the
first run; later runs are fast.

## Joint mapping (verified)

Sim DOF order: `[joint1..joint6, joint_left, joint_right]`.

| Real motor | Sim DOF | Sign |
|---|---|---|
| 1..5 | joint1..joint5 | **-1** (USD/URDF convention flip) |
| 6 | joint6 | +1 |
| 7 (gripper) | joint_left + joint_right | +1, clipped to each finger's limits |

The sign convention `q_sim = q_real * [-1,-1,-1,-1,-1,+1]` comes from the
URDF->USD conversion and was validated by FK round-trip (zero error) and by
the live mirror (base turned the same direction as the physical arm).

## Safety notes

- The bridge **never enables motors**: it only calls `request_feedback`.
- `scripts/move_joint_test.py` enables exactly ONE motor, uses POS_VEL with a
  low velocity limit (0.5 rad/s), and always disables in a `finally` block.
- Keep >= 1 m distance when any motor is enabled. Kill power if in doubt.
- Do not run two processes against `/dev/ttyACM0` at once (the port is
  exclusive; the bridge and motorbridge-gateway/Studio cannot run together).

## Roadmap

- [x] Sim -> Real trajectory execution (with per-joint velocity limits + e-stop) — `scripts/rebot_daemon.py`
- [x] Gripper span calibration (motor 7 rad -> finger range) — calibrated defaults in the daemon (open −6.8 rad / close 0.0)
- [x] XR teleoperation (Quest 3 controllers -> real arm) — `scripts/xr_teleop_rebot.py`
- [ ] Eye-in-hand RGB-D (RealSense D455) + GraspNet pipeline on top

## Real -> Sim vs teleoperation stacks

Two independent ways to drive/read the arm live in `scripts/`; both speak the same
UDP mirror format so the Isaac Sim twin works with either:

- `real_to_sim_bridge.py` — passive reader (never enables motors), minimal deps.
- `rebot_daemon.py` — full HTTP control daemon (single owner of the serial port):
  state/FK/IK, blocking moves, **`/servo` streaming teleop**, gripper, e-stop,
  torque/temperature watchdogs, and the same UDP mirror on 127.0.0.1:5801.
  Kinematics need `pin` (Pinocchio) plus the URDF from
  [reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py)
  (override the path with `REBOT_URDF` or `--urdf`).

```bash
python scripts/rebot_daemon.py                  # owns /dev/ttyACM0
python scripts/rebot_client.py health           # joints_seen 1-7, kinematics ok
python scripts/rebot_client.py enable           # holds current pose (no jump)
python scripts/rebot_client.py move-pose 0.30 0.0 0.25 --vlim 0.3
python scripts/rebot_client.py gripper close
python scripts/rebot_client.py estop            # e-stop (disable_all)
```

## XR teleoperation (Quest 3 -> real arm)

`scripts/xr_teleop_rebot.py` drives the physical arm from Quest 3 controllers over
NVIDIA Isaac Teleop / CloudXR:

```
Quest 3 controllers --CloudXR--> XRController (lerobot isaac_teleop)
    -> PoseGate (occlusion ghost / snap-back guard)
    -> Clutch (squeeze-to-engage + anti-windup leash)
    -> rebot_daemon /servo (warm-seeded IK + step clamp + watchdogs)
    -> Damiao motors (POS_VEL)
```

Controls: **squeeze** (hold) engages the clutch — hand deltas drive the EE 1:1;
release and the arm holds. **Trigger** drives the gripper proportionally. All
operator feedback is in-headset haptics (engage/release buzz, workspace-edge buzz,
pose-lost pulse); if the XR session drops (headset sleep / Wi-Fi), the arm holds
and the script relaunches CloudXR until the headset returns.

Prerequisites (beyond this repo): the `isaac_teleop` teleoperator stack from
[huggingface/lerobot#3927](https://github.com/huggingface/lerobot/pull/3927)
(or the [johnnynunez/lerobot](https://github.com/johnnynunez/lerobot) fork, which
adds `PoseGate`/`Clutch.limit_lead`) with the `isaac-teleop` extra installed, and
a CloudXR-connected Quest 3.

```bash
python scripts/rebot_daemon.py                          # terminal 1 (arm venv)
# terminal 2 (lerobot venv):
python scripts/xr_teleop_rebot.py                       # 20 Hz, 6 cm leash
python scripts/xr_teleop_rebot.py --hz 25 --max-lead-m 0.08 --hand left
```

Verified on hardware: live tracking, clutch engage/release, workspace-edge leash,
proportional gripper, IK-unreachable frames held (arm never lurches), XR-session
loss recovery.

## Repository layout

```
assets/
  00-arm-rs_asm-v3/       # arm USD package (Newton+PhysX validated) + evidence
scripts/
  rebot_vendor.py         # vendor selection (damiao/robstride): models, transport
  real_to_sim_bridge.py   # real arm -> UDP joint stream (passive, 50 Hz)
  read_joints.py          # print joint positions (sanity check)
  move_joint_test.py      # supervised single-joint move (POS_VEL, slow)
  isaacsim_client.py      # minimal TCP client for Isaac Sim python_server
  rebot_daemon.py         # HTTP control daemon (FK/IK, moves, /servo, e-stop)
  rebot_client.py         # CLI client for the daemon
  xr_teleop_rebot.py      # Quest 3 XR controller teleop of the real arm
isaac/
  01_load_arm_stage.py    # open the arm USD stage (runs inside Isaac Sim)
  02_start_mirror.py      # play + articulation + UDP mirror listener
  03_mirror_status.py     # packet counter / current sim q
  04_fix_newton_contacts.py  # fix MJWarp "exceeded limit (200)" contact overflow
```

## Troubleshooting

**`Number of Newton contacts (N) exceeded MJWarp limit (200). Increase nconmax.`**

The MuJoCo-Warp solver pre-allocates a fixed contact buffer. Scenes with a
ground plane / table under the gripper's convex-decomposition colliders can
overflow the 200-contact default. Fix at runtime (raises the caps and rebuilds
the solver model):

```bash
python scripts/isaacsim_client.py --timeout 300 --file isaac/04_fix_newton_contacts.py
```

Do not raise `nconmax` beyond Newton's allocated `rigid_contact_max` — the
script clamps automatically (values above it make every step error out).

> **Update (2026-07-07):** the shipped asset now persists the caps in USD as
> custom attrs on the articulation root (`newton:solver:nconmax = 8192`,
> `newton:solver:njmax = 32768`), so patched Isaac builds pick them up on
> stage open with no runtime script. Recent Newton builds also take
> `max(user value, geometry estimate)` for `nconmax`, so over-raising is safe
> there (verify the real allocation with `solver.mjw_data.naconmax`). On stock
> develop the attrs are ignored harmlessly and `04_fix_newton_contacts.py`
> remains the fix. The asset also ships `newton:selfCollisionEnabled = 0` and
> the `IsaacRobotAPI` schema (Gain Tuner dropdown + 8/8 snap-to-limits pass);
> full data in `assets/00-arm-rs_asm-v3/evidence/analysis_2026-07-07/`.

