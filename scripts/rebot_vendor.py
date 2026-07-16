#!/usr/bin/env python3
"""Motor-vendor selection shared by the reBot scripts (single source of truth).

Two supported arm builds:
  - damiao (default): reBot Arm B601-DM, Damiao DM4340/DM4310 over the Damiao
    USB2CAN serial bridge (dm-serial, --serial-port/--baud).
  - robstride: RobStride RS-series over classic SocketCAN (--channel), any
    SocketCAN adapter at 1 Mbps.

Scripts pick the vendor with --vendor (default: env REBOT_VENDOR, else damiao).
"""
import os
import time

from motorbridge import Controller

# Damiao model per joint (verified via register dump: motors 1-3 tau_max=28
# -> DM4340, motors 4-7 vel_max=30 / tau_max=10 -> DM4310). Motor 7 = gripper.
DAMIAO_MODELS = {1: "4340", 2: "4340", 3: "4340", 4: "4310", 5: "4310", 6: "4310", 7: "4310"}
# RobStride model per joint: "rs-00" is a placeholder — set the actual
# RS-series model ("rs-00".."rs-06") fitted at each joint of your build.
ROBSTRIDE_MODELS = {mid: "rs-00" for mid in range(1, 8)}

MODELS = {"damiao": DAMIAO_MODELS, "robstride": ROBSTRIDE_MODELS}
VENDORS = sorted(MODELS)
MOTOR_IDS = tuple(sorted(DAMIAO_MODELS))  # same joint layout for both vendors
DEFAULT_VENDOR = os.environ.get("REBOT_VENDOR", "damiao")

# Feedback-id rule: Damiao replies on feedback/master id = motor_id + 0x10;
# RobStride replies to the constant HOST id (convention: 0xFD = 253).
ROBSTRIDE_HOST_ID = 0xFD


def feedback_id(vendor: str, mid: int) -> int:
    return mid + 0x10 if vendor == "damiao" else ROBSTRIDE_HOST_ID


def add_vendor_args(ap) -> None:
    """Add the shared vendor/transport options to an argparse parser."""
    ap.add_argument("--vendor", choices=VENDORS, default=DEFAULT_VENDOR,
                    help="motor vendor (default: env REBOT_VENDOR, else damiao)")
    ap.add_argument("--serial-port", default="/dev/ttyACM0",
                    help="dm-serial bridge port (vendor damiao)")
    ap.add_argument("--baud", type=int, default=921600,
                    help="dm-serial baud rate (vendor damiao)")
    ap.add_argument("--channel", default="can0",
                    help="SocketCAN channel (vendor robstride)")


def make_controller_and_motors(args, ids=None):
    """Build the vendor transport and motors from parsed args.

    Returns (ctrl, {motor_id: Motor}). ids defaults to all 7 joints.
    RobStride motors cannot use the dm-serial transport (Damiao-only), so the
    robstride path always opens classic SocketCAN on args.channel.
    """
    models = MODELS[args.vendor]
    ids = sorted(models) if ids is None else list(ids)
    if args.vendor == "damiao":
        ctrl = Controller.from_dm_serial(serial_port=args.serial_port, baud=args.baud)
        add = ctrl.add_damiao_motor
    else:
        ctrl = Controller(args.channel)
        add = ctrl.add_robstride_motor
    motors = {
        mid: add(motor_id=mid, feedback_id=feedback_id(args.vendor, mid),
                 model=models[mid])
        for mid in ids
    }
    if args.vendor == "robstride":
        # Nudge the firmware toward the classic type-2 feedback frames. On the
        # RS firmware tested this yields one feedback frame per motor (enough to
        # seed get_state()) but NOT a periodic stream — its idle streaming uses
        # compact type-0x18 report frames that motorbridge's state decoder does
        # not consume. Position reads must therefore go through read_positions()
        # below, which uses parameter reads (verified live on RS hardware).
        for m in motors.values():
            m.robstride_set_active_report(True)
    return ctrl, motors


# RobStride parameter indices (read as exact f32, already rad / rad/s).
ROBSTRIDE_PARAM_MECH_POS = 0x7019
ROBSTRIDE_PARAM_MECH_VEL = 0x701A


def read_positions(vendor, ctrl, motors, timeout_ms=20):
    """Read joint positions [rad] for all motors; returns {motor_id: pos}.

    Motors that fail to reply this cycle are simply absent from the result
    (callers already treat an incomplete snapshot as "skip this cycle").

    - damiao: request_feedback + poll + get_state (the dm-serial pattern).
    - robstride: single-parameter read of mechPos per motor. get_state() is NOT
      live on RS firmware (see make_controller_and_motors); param-read replies
      are, and carry exact f32 radians.
    """
    q = {}
    if vendor == "robstride":
        for mid, m in motors.items():
            try:
                q[mid] = m.robstride_get_param_f32(ROBSTRIDE_PARAM_MECH_POS,
                                                   timeout_ms=timeout_ms)
            except Exception:
                pass  # missed reply; caller holds/skips this cycle
        return q
    for m in motors.values():
        m.request_feedback()
    time.sleep(0.001)
    ctrl.poll_feedback_once()
    for mid, m in motors.items():
        st = m.get_state()
        if st is not None:
            q[mid] = st.pos
    return q
