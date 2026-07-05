# SPDX-License-Identifier: Apache-2.0
"""Quest 3 XR-controller teleoperation of the physical reBot Arm B601-DM.

Data flow (all pieces individually validated before this script):

    Quest 3 controllers --CloudXR--> XRController (lerobot isaac_teleop fork)
        -> PoseGate (occlusion ghost / snap-back guard)
        -> Clutch (squeeze-to-engage, anti-windup leash)
        -> rebot_daemon /servo (Pinocchio IK + step clamp + torque/temp watchdogs)
        -> Damiao motors (POS_VEL)

Operator feedback is IN-HEADSET (haptics): engage/release buzzes, workspace-edge
buzz, pose-lost pulse. Terminal output is a mirror/log only.

Controls:
    squeeze (hold)  engage clutch -- hand deltas drive the EE 1:1
    release         disengage -- arm holds pose
    trigger         gripper (proportional: 0 = open, 1 = closed)
    Ctrl+C          clean exit (arm holds, then daemon keeps it enabled)

Prerequisites:
    - rebot_daemon.py running and /enable'd (this script checks /health)
    - CloudXR runtime NOT already running (the device auto-launches it)
    - Quest 3 connected to this host's CloudXR (same as SO-101 sessions)

Usage:
    source ~/Projects/demo/.env/bin/activate
    python xr_teleop_rebot.py            # defaults: 20 Hz, 6 cm leash
    python xr_teleop_rebot.py --hz 25 --max-lead-m 0.08
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request

import numpy as np

DAEMON = "http://127.0.0.1:5810"

# Haptic grammar (matches the SO-101 example loop the operator already knows).
HAPTIC_ENGAGE = {"amplitude": 0.9, "duration_s": 0.08}
HAPTIC_DISENGAGE = {"amplitude": 0.4, "duration_s": 0.05}
HAPTIC_EDGE_BUZZ = {"amplitude": 0.25, "duration_s": 0.05}
HAPTIC_POSE_LOST = {"amplitude": 0.7, "duration_s": 0.25}
HAPTIC_READY = {"amplitude": 0.8, "duration_s": 0.15}

SQUEEZE_ENGAGE = 0.5


def api(path: str, payload: dict | None = None, timeout: float = 5.0) -> dict:
    req = urllib.request.Request(
        DAEMON + path,
        data=None if payload is None else json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="GET" if payload is None else "POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def rpy_from_quat(quat_xyzw: np.ndarray) -> list[float]:
    """Quaternion [xyzw] -> [roll, pitch, yaw], Pinocchio convention (R = Rz*Ry*Rx).

    Matches pin.rpy.rpyToMatrix on the daemon side, so the pose round-trips.
    lerobot's Rotation helper has no as_euler; extract from the matrix directly.
    """
    from lerobot.utils.rotation import Rotation

    m = Rotation.from_quat(np.asarray(quat_xyzw, dtype=float)).as_matrix()
    roll = float(np.arctan2(m[2, 1], m[2, 2]))
    pitch = float(np.arcsin(np.clip(-m[2, 0], -1.0, 1.0)))
    yaw = float(np.arctan2(m[1, 0], m[0, 0]))
    return [roll, pitch, yaw]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hz", type=float, default=20.0, help="Servo stream rate")
    parser.add_argument(
        "--max-lead-m",
        type=float,
        default=0.06,
        help="Clutch anti-windup leash: max commanded-vs-measured EE lead [m]",
    )
    parser.add_argument("--vlim", type=float, default=0.8, help="Joint velocity limit [rad/s]")
    parser.add_argument(
        "--hand", choices=["left", "right"], default="right", help="Controller hand"
    )
    args = parser.parse_args()

    # --- 1. Daemon must be up, streaming, with kinematics ---------------------
    try:
        health = api("/health")
    except (urllib.error.URLError, OSError):
        sys.exit(
            "FAILED: rebot_daemon not reachable on 127.0.0.1:5810.\n"
            "Start it first:  cd ~/Projects/rebotdev && "
            ".venv-rebot/bin/python rebot_daemon.py"
        )
    if health.get("kinematics") != "ok" or len(health.get("joints_seen", [])) < 7:
        sys.exit(f"FAILED: daemon unhealthy: {health}")

    pose = api("/pose")
    print(f"[teleop] arm EE at xyz={np.round(pose['xyz'], 3).tolist()} rpy="
          f"{np.round(pose['rpy'], 3).tolist()} (frame {pose['frame']})")

    # --- 2. XR device (auto-launches CloudXR; blocks until runtime ready) -----
    from lerobot.teleoperators.isaac_teleop import Clutch, XRController, XRControllerConfig
    from lerobot.teleoperators.isaac_teleop.pose_gate import PoseGate

    teleop = XRController(XRControllerConfig(hand_side=args.hand, app_name="RebotXRTeleop"))
    print("[teleop] launching CloudXR + OpenXR session (put the headset on)...")
    teleop.connect()

    # --- 3. Enable the arm (holds current pose; no jump) ----------------------
    enable = api("/enable", {})
    print(f"[teleop] motors enabled, held at {json.dumps(enable['held_at'])}")

    # Seed the clutch home from the arm's MEASURED EE pose (FK), so the first
    # engage cannot teleport the arm.
    from lerobot.utils.rotation import Rotation

    home = np.eye(4)
    home[:3, 3] = pose["xyz"]
    home[:3, :3] = Rotation.from_quat(np.asarray(pose["quat_xyzw"])).as_matrix()
    clutch = Clutch(home)
    gate = PoseGate()

    gripper_open, gripper_close = -6.8, 0.0  # daemon-calibrated endpoints [rad]

    dt = 1.0 / args.hz
    engaged = False
    was_tracking = False
    last_log = 0.0
    print(f"[teleop] streaming at {args.hz:.0f} Hz -- squeeze to engage. Ctrl+C to stop.")
    try:
        while True:
            t0 = time.monotonic()
            try:
                action = teleop.get_action()
            except RuntimeError as e:
                # XR session died under us (headset sleep / Wi-Fi drop / runtime exit:
                # xrSyncActions -13 INSTANCE_LOST). Hold the arm and relaunch.
                engaged = False
                was_tracking = False
                print(f"[teleop] XR session lost ({e}); arm held. Reconnecting "
                      "(wake the headset / reopen the CloudXR client)...")
                try:
                    teleop.disconnect()
                except Exception:
                    # Ensure a failed teardown can't wedge us in "Already connected".
                    teleop._session = None  # noqa: SLF001
                while True:
                    try:
                        teleop.connect()
                        break
                    except Exception as retry_err:
                        print(f"[teleop] reconnect failed ({retry_err}); retrying in 5 s")
                        time.sleep(5.0)
                gate = PoseGate()  # fresh distrust window after any reconnect
                print("[teleop] XR session re-established -- squeeze to engage")
                continue
            tracking = teleop.is_tracking

            if tracking and not was_tracking:
                teleop.send_feedback(HAPTIC_READY)
                print("[teleop] controller tracked -- ready")
            was_tracking = tracking

            grip_pos = np.asarray(action["grip_pos"], dtype=float)
            verdict = gate.check(grip_pos, tracking and teleop.pose_valid)

            if verdict != "ok":
                if engaged:
                    engaged = False
                    teleop.send_feedback(HAPTIC_POSE_LOST)
                    print(f"[teleop] pose {verdict} -- clutch force-released, arm held")
                time.sleep(max(0.0, dt - (time.monotonic() - t0)))
                continue

            squeeze = float(action["squeeze"])
            if squeeze >= SQUEEZE_ENGAGE and not engaged:
                clutch.engage(grip_pos, np.asarray(action["grip_quat"]))
                engaged = True
                teleop.send_feedback(HAPTIC_ENGAGE)
                print("[teleop] clutch ENGAGED")
            elif squeeze < SQUEEZE_ENGAGE and engaged:
                engaged = False
                teleop.send_feedback(HAPTIC_DISENGAGE)
                print("[teleop] clutch released -- arm holds")

            if engaged:
                target_pos, target_quat = clutch.rebase(
                    grip_pos, np.asarray(action["grip_quat"])
                )
                # Anti-windup leash vs the arm's MEASURED EE (fresh FK).
                measured = np.asarray(api("/pose")["xyz"], dtype=float)
                target_pos, excess = clutch.limit_lead(measured, args.max_lead_m)
                if excess > 0.0:
                    teleop.send_feedback(HAPTIC_EDGE_BUZZ)

                trigger = float(action["trigger"])
                grip_cmd = gripper_open + trigger * (gripper_close - gripper_open)
                res = api(
                    "/servo",
                    {
                        "xyz": target_pos.tolist(),
                        "rpy": rpy_from_quat(target_quat),
                        "vlim": args.vlim,
                        "gripper": grip_cmd,
                    },
                )
                now = time.monotonic()
                if now - last_log >= 1.0:
                    state = "sent" if res.get("sent") else f"held ({res.get('note')})"
                    print(
                        f"[teleop] target xyz={np.round(target_pos, 3).tolist()} "
                        f"trigger={trigger:.2f} servo={state} "
                        f"lead_excess={excess * 100:.1f}cm"
                    )
                    last_log = now
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\n[teleop] stopping (arm stays enabled and held; use "
              "rebot_client.py disable to go limp)")
    finally:
        teleop.disconnect()
        print("[teleop] XR session closed")


if __name__ == "__main__":
    main()
