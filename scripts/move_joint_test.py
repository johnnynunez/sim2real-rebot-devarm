#!/usr/bin/env python3
"""Supervised single-joint move on the real reBot Arm B601-DM.

Safety-first single-motor test (the Sim -> Real building block):
- Enables ONLY the selected motor, all others stay passive.
- POS_VEL mode with a low velocity limit.
- Moves a relative step from the current position, verifies via feedback,
  then ALWAYS disables the motor (finally block).

Usage:
    python scripts/move_joint_test.py --motor 1 --step -0.3 --vlim 0.5

WARNING: the arm WILL move. Keep >= 1 m distance and clear the workspace.
"""
import argparse
import time

from motorbridge import Controller, Mode

MODELS = {1: "4340", 2: "4340", 3: "4340", 4: "4310", 5: "4310", 6: "4310", 7: "4310"}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--serial-port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--motor", type=int, default=1, choices=sorted(MODELS))
    ap.add_argument("--step", type=float, default=-0.30, help="relative move in rad")
    ap.add_argument("--vlim", type=float, default=0.5, help="velocity limit in rad/s")
    ap.add_argument("--timeout", type=float, default=8.0)
    ap.add_argument("--tol", type=float, default=0.03, help="position tolerance in rad")
    args = ap.parse_args()

    ctrl = Controller.from_dm_serial(serial_port=args.serial_port, baud=args.baud)
    m = ctrl.add_damiao_motor(
        motor_id=args.motor, feedback_id=args.motor + 0x10, model=MODELS[args.motor]
    )
    try:
        m.request_feedback()
        time.sleep(0.01)
        st = m.get_state()
        assert st is not None, f"no feedback from motor {args.motor}"
        q0 = st.pos
        target = q0 + args.step
        print(f"motor {args.motor}: {q0:+.4f} rad -> {target:+.4f} rad (vlim {args.vlim} rad/s)")

        m.ensure_mode(Mode.POS_VEL)
        m.enable()
        print("enabled (POS_VEL)")

        t0 = time.monotonic()
        pos = q0
        while time.monotonic() - t0 < args.timeout:
            m.send_pos_vel(target, args.vlim)
            time.sleep(0.02)
            st = m.get_state()
            if st is not None:
                pos = st.pos
                if abs(pos - target) < args.tol:
                    break
        print(f"final pos: {pos:+.4f} rad (err {abs(pos - target) * 1000:.1f} mrad)")
    finally:
        try:
            m.disable()
            print("disabled")
        finally:
            ctrl.shutdown()


if __name__ == "__main__":
    main()
