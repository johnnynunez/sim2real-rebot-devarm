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

> The USD asset of the arm is expected at the path passed in step 1 below.
> Ours lives in `00-arm-rs_asm-v3` (URDF converted with urdf-usd-converter,
> validated on both Newton and PhysX).

## Run the Real -> Sim mirror

```bash
# 1. Load the arm USD into Isaac Sim (shared remote context)
python scripts/isaacsim_client.py --timeout 180 \
    --arg usd_path=/absolute/path/to/00-arm-rs_asm-v3.usda \
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

- [ ] Sim -> Real trajectory execution (with per-joint velocity limits + e-stop)
- [ ] Gripper span calibration (motor 7 rad -> finger prismatic/revolute range)
- [ ] Eye-in-hand RGB-D (RealSense D455) + GraspNet pipeline on top

## Repository layout

```
scripts/
  real_to_sim_bridge.py   # real arm -> UDP joint stream (passive, 50 Hz)
  read_joints.py          # print joint positions (sanity check)
  move_joint_test.py      # supervised single-joint move (POS_VEL, slow)
  isaacsim_client.py      # minimal TCP client for Isaac Sim python_server
isaac/
  01_load_arm_stage.py    # open the arm USD stage (runs inside Isaac Sim)
  02_start_mirror.py      # play + articulation + UDP mirror listener
  03_mirror_status.py     # packet counter / current sim q
```
