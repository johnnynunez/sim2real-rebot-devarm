#!/usr/bin/env python3
"""HTTP control daemon for the reBot Arm (Damiao dm-serial or RobStride SocketCAN).

Single owner of the motor transport. Exposes a small JSON API for agent tools:

    GET  /health         daemon + serial + kinematics status
    GET  /state          per-joint pos/vel/torque/temps + timestamp
    GET  /pose           FK pose of the gripper_end frame (m / rad)
    POST /enable         enable arm motors and hold current pose
    POST /disable        disable all motors (arm goes limp)
    POST /estop          immediate disable_all (also aborts active moves)
    POST /move_joints    {"q": [6 floats rad], "vlim": 0.5}
    POST /move_pose      {"xyz": [3], "rpy": [3], "vlim": 0.5}  (IK + move)
    POST /gripper        {"action": "open"|"close"} or {"pos": rad}
    POST /rehome_gripper close-until-stall re-zero (multi-turn wrap recovery)

Safety model:
  - Motors are NEVER enabled at startup (passive feedback polling only).
  - move_* / gripper refuse unless POST /enable was called first.
  - Joint targets are clamped to URDF limits; velocity limit is capped.
  - Torque / MOSFET-temperature watchdog aborts motion and disables motors.
  - Real->URDF sign/offset mapping is identity until verified; check FK
    against the physical arm before trusting Cartesian moves.

Optionally mirrors joint state as JSON over UDP (same wire format as
s2r_real_reader.py) so an Isaac Sim digital twin can keep following the arm.
"""
import argparse
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
from motorbridge import Mode
from rebot_vendor import add_vendor_args, make_controller_and_motors

# Motor models per joint live in rebot_vendor.py (single source of truth).
ARM_IDS = [1, 2, 3, 4, 5, 6]
GRIPPER_ID = 7

# Sustained torque abort thresholds (Nm), per vendor.
TORQUE_ABORT = {
    # Damiao: conservative vs URDF effort limits (DM4340 / DM4310).
    "damiao": {1: 22.0, 2: 22.0, 3: 22.0, 4: 9.0, 5: 9.0, 6: 9.0, 7: 9.0},
    # RobStride placeholder: RS-series per-model torque limits vary — these
    # are conservative defaults, tune per the rs-* model fitted at each joint.
    "robstride": {1: 8.0, 2: 8.0, 3: 8.0, 4: 4.0, 5: 4.0, 6: 4.0, 7: 4.0},
}
TEMP_ABORT_C = 65.0
MAX_VLIM = 1.0          # hard cap on commanded velocity limit (rad/s)
DEFAULT_VLIM = 0.5
MOVE_TOL = 0.02         # rad, per joint
MOVE_TIMEOUT = 25.0     # s
SERVO_MAX_STEP_RAD = 0.08   # per-servo-call joint step clamp (teleop streaming)
# Zero-calibration slack on URDF position limits: the measured rest pose sits
# ~0.03 rad outside j2/j3's [0, pi] window, which pins the CLIK at the boundary
# and blocks whole motion directions. Small, verified-safe margin.
LIMIT_MARGIN_RAD = 0.05
GRIPPER_CLOSE_TORQUE = 1.5   # Nm plateau => object grasped
# Gripper protection: streaming teleop once drove the jaws to the mechanical
# open stop and cracked the plastic fingers, so /servo gets its own (tighter)
# opening bound and a slower velocity cap, and any blocking /gripper move
# aborts on a sustained stall instead of grinding for the whole timeout.
SERVO_GRIPPER_VLIM = 2.0     # rad/s cap for gripper targets streamed via /servo
GRIPPER_STALL_TORQUE = 1.0   # Nm sustained short of target => stalled, stop pushing
# Multi-turn wrap protection: the Damiao turn counter is volatile across power
# cycles, so the gripper can wake up reading physical + 2*pi*k. /enable refuses
# when the reading falls outside the calibrated travel plus this margin, and
# POST /rehome_gripper restores the close==0 frame by stalling against the stop.
GRIPPER_WRAP_MARGIN = 0.5          # rad of tolerance around [open, close]
GRIPPER_REHOME_STALL_TORQUE = 0.8  # Nm sustained => mechanical stop reached
GRIPPER_REHOME_MAX_TRAVEL = 8.0    # rad sweep guard
# URDF for FK/IK; override with REBOT_URDF or --urdf for other checkouts.
URDF = os.environ.get(
    "REBOT_URDF",
    str(Path.home() / "Projects/rebotdev/reBotArm_control_py/urdf/"
                      "00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"))
EE_FRAME = "gripper_end"


class Kinematics:
    """Pinocchio FK/IK on the 6 arm joints (gripper joints locked)."""

    def __init__(self, urdf_path: str, signs=None):
        import pinocchio as pin
        self.pin = pin
        full = pin.buildModelFromUrdf(urdf_path)
        lock = [full.getJointId(n) for n in ("joint_left", "joint_right")]
        self.model = pin.buildReducedModel(full, lock, pin.neutral(full))
        self.data = self.model.createData()
        self.fid = self.model.getFrameId(EE_FRAME)
        self.lo = self.model.lowerPositionLimit.copy()
        self.hi = self.model.upperPositionLimit.copy()
        # Real-motor -> URDF sign mapping (UNVERIFIED: identity by default).
        self.signs = np.array(signs if signs else [1.0] * 6)

    def to_urdf(self, q_real):
        return np.asarray(q_real) * self.signs

    def to_real(self, q_urdf):
        return np.asarray(q_urdf) * self.signs  # signs are +-1: involution

    def clamp(self, q_urdf):
        return np.clip(q_urdf, self.lo - LIMIT_MARGIN_RAD, self.hi + LIMIT_MARGIN_RAD)

    def fk(self, q_real):
        pin = self.pin
        q = self.to_urdf(q_real)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        M = self.data.oMf[self.fid]
        rpy = pin.rpy.matrixToRpy(M.rotation)
        quat = pin.Quaternion(M.rotation).coeffs()  # x y z w
        return {
            "xyz": M.translation.tolist(),
            "rpy": rpy.tolist(),
            "quat_xyzw": quat.tolist(),
        }

    def ik(self, xyz, rpy, q_seed_real, iters=300, damp=1e-6, tol=1e-4):
        """CLIK with damped least squares. Returns (q_real, err_norm)."""
        pin = self.pin
        target = pin.SE3(pin.rpy.rpyToMatrix(np.asarray(rpy, dtype=float)),
                         np.asarray(xyz, dtype=float))
        q = self.to_urdf(q_seed_real).copy()
        err = None
        for _ in range(iters):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            err = pin.log(self.data.oMf[self.fid].actInv(target)).vector
            if np.linalg.norm(err) < tol:
                break
            J = pin.computeFrameJacobian(self.model, self.data, q,
                                         self.fid, pin.LOCAL)
            dq = J.T @ np.linalg.solve(J @ J.T + damp * np.eye(6), err)
            q = self.clamp(pin.integrate(self.model, q, dq * 0.5))
        return self.to_real(q), float(np.linalg.norm(err))


class ArmDaemon:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()          # serializes all serial access
        self.state = {}                        # mid -> dict(pos, vel, torq, ...)
        self.state_t = 0.0
        self.enabled = False
        self.moving = False
        self.abort_flag = False
        self.servo_until = 0.0                 # torque watchdog armed while servoing
        self._servo_strikes = {}
        self.last_error = None
        self.kin = None
        self.kin_error = None
        try:
            self.kin = Kinematics(args.urdf, signs=args.joint_signs)
        except Exception as e:  # daemon still useful without kinematics
            self.kin_error = f"{type(e).__name__}: {e}"

        self.torque_abort = TORQUE_ABORT[args.vendor]
        self.ctrl, self.motors = make_controller_and_motors(args)
        self.mirror = None
        if args.mirror_port:
            self.mirror = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.mirror_dst = (args.mirror_host, args.mirror_port)

        t = threading.Thread(target=self._poll_loop, daemon=True)
        t.start()

    # ---------------- polling ----------------
    def _poll_loop(self):
        dt = 1.0 / self.args.rate
        while True:
            t0 = time.monotonic()
            with self.lock:
                for m in self.motors.values():
                    m.request_feedback()
                time.sleep(0.001)
                self.ctrl.poll_feedback_once()  # pump RX (required on SocketCAN; harmless on dm-serial)
                snap = {}
                for mid, m in self.motors.items():
                    st = m.get_state()
                    if st is not None:
                        snap[mid] = {"pos": st.pos, "vel": st.vel,
                                     "torq": st.torq, "t_mos": st.t_mos,
                                     "t_rotor": st.t_rotor,
                                     "status": st.status_code}
            if snap:
                self.state = snap
                self.state_t = time.time()
                if self.mirror and len(snap) == len(self.motors):
                    payload = {"t": self.state_t,
                               "q": {str(k): v["pos"] for k, v in snap.items()}}
                    try:
                        self.mirror.sendto(json.dumps(payload).encode(),
                                           self.mirror_dst)
                    except OSError:
                        pass
            # temperature watchdog runs even when idle
            for mid, s in self.state.items():
                if s["t_mos"] > TEMP_ABORT_C and self.enabled:
                    self._estop(f"motor {mid} MOSFET temp {s['t_mos']:.0f}C")
            # torque watchdog while servoing (blocking moves run their own)
            if self.enabled and time.monotonic() < self.servo_until:
                for mid in ARM_IDS:
                    s = self.state.get(mid)
                    if s is None:
                        continue
                    if abs(s["torq"]) > self.torque_abort[mid]:
                        self._servo_strikes[mid] = self._servo_strikes.get(mid, 0) + 1
                        if self._servo_strikes[mid] >= 3:
                            self._estop(f"motor {mid} torque {s['torq']:.1f} Nm (servo)")
                    else:
                        self._servo_strikes[mid] = 0
            sleep = dt - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

    # ---------------- helpers ----------------
    def _require_fresh_state(self):
        if time.time() - self.state_t > 0.5 or len(self.state) < len(self.motors):
            raise RuntimeError("joint state stale or incomplete")

    def q_real(self):
        self._require_fresh_state()
        return [self.state[mid]["pos"] for mid in ARM_IDS]

    def _estop(self, reason):
        self.abort_flag = True
        with self.lock:
            try:
                self.ctrl.disable_all()
            finally:
                self.enabled = False
        self.last_error = f"ESTOP: {reason}"
        return self.last_error

    # ---------------- commands ----------------
    def _gripper_wrap_check(self):
        """Detect a 2*pi-wrapped multi-turn gripper reading before enabling motors.

        The Damiao multi-turn counter is volatile across power cycles: the
        single-turn absolute zero survives, the turn count does not. The gripper
        travel (~6.8 rad) exceeds one turn, so after a repower it can report
        physical + 2*pi*k (verified: physically closed gripper reading +6.227 rad
        = -0.056 + 2*pi in the close=0/open=-6.8 frame). Absolute open/close
        targets are then wrong by whole turns and drive the mechanism into its
        stop through the gear reduction -- the prime suspect for silently latched
        coil over-temp faults. Returns the offending reading, or None if sane.

        Characterized on Damiao hardware; positions are multi-turn absolute for
        both vendors, so the guard stays active on RobStride builds as well.
        """
        s = self.state.get(GRIPPER_ID)
        if s is None or self.args.gripper_open is None or self.args.gripper_close is None:
            return None
        lo = min(self.args.gripper_open, self.args.gripper_close) - GRIPPER_WRAP_MARGIN
        hi = max(self.args.gripper_open, self.args.gripper_close) + GRIPPER_WRAP_MARGIN
        pos = s["pos"]
        return None if lo <= pos <= hi else pos

    def rehome_gripper(self):
        """Close the gripper until stall against the mechanical stop, re-zero there.

        Restores the close==0 convention regardless of the 2*pi branch the encoder
        woke up on. Only the gripper motor is enabled during the sweep; it is
        disabled again afterwards (arm state is untouched).
        """
        if self.enabled:
            raise RuntimeError("disable motors before re-homing (POST /disable first)")
        self._require_fresh_state()
        m = self.motors[GRIPPER_ID]
        step, vlim, settle = 0.05, 0.5, 0.15
        with self.lock:
            m.clear_error()
            m.enable()
            m.ensure_mode(Mode.POS_VEL)
        try:
            start = self.state[GRIPPER_ID]["pos"]
            target = start
            strikes = 0
            stalled = False
            # Closing direction is positive (open=-6.8 < close=0.0).
            while abs(target - start) < GRIPPER_REHOME_MAX_TRAVEL:
                target += step
                with self.lock:
                    m.send_pos_vel(pos=target, vlim=vlim)
                time.sleep(settle)
                s = self.state.get(GRIPPER_ID)
                if s is None:
                    continue
                if abs(s["torq"]) > GRIPPER_REHOME_STALL_TORQUE or abs(s["pos"] - target) > 0.3:
                    strikes += 1
                    if strikes >= 3:
                        stalled = True
                        break
                else:
                    strikes = 0
            if not stalled:
                raise RuntimeError(
                    f"gripper re-homing failed: no stall within "
                    f"{GRIPPER_REHOME_MAX_TRAVEL:.1f} rad of travel")
        finally:
            with self.lock:
                m.disable()
        time.sleep(0.3)  # torque off: let the gears relax onto the stop
        with self.lock:
            m.set_zero_position()
        time.sleep(0.3)
        pos = None
        for _ in range(20):  # wait for the poll loop to pick up the re-zeroed value
            time.sleep(0.1)
            s = self.state.get(GRIPPER_ID)
            if s is not None and self.state_t > time.time() - 0.5:
                pos = s["pos"]
                break
        if pos is None or abs(pos) > 0.2:
            raise RuntimeError(
                f"gripper re-homing verification failed: expected ~0 rad at the "
                f"closed stop, read {pos}")
        return {"rehomed": True, "closed_stop_pos": pos}

    def enable(self):
        self._require_fresh_state()
        wrapped = self._gripper_wrap_check()
        if wrapped is not None:
            raise RuntimeError(
                f"gripper reads {wrapped:+.3f} rad, outside calibrated travel "
                f"[{min(self.args.gripper_open, self.args.gripper_close):.1f}, "
                f"{max(self.args.gripper_open, self.args.gripper_close):.1f}] rad "
                f"(+/- {GRIPPER_WRAP_MARGIN:.1f} margin): multi-turn encoder wrapped "
                f"after a power cycle. POST /rehome_gripper first (motors disabled).")
        with self.lock:
            for mid in ARM_IDS + [GRIPPER_ID]:
                m = self.motors[mid]
                # Damiao motors latch protection faults (e.g. coil over-temp
                # status 0xC) and silently produce zero torque until cleared;
                # clear_error is also supported (and harmless) on RobStride.
                m.clear_error()
                m.enable()
                m.ensure_mode(Mode.POS_VEL)
                # hold current pose so the arm does not jump on enable
                m.send_pos_vel(pos=self.state[mid]["pos"], vlim=0.2)
            self.enabled = True
        self.abort_flag = False
        self.last_error = None
        return {"enabled": True, "held_at": {m: self.state[m]["pos"]
                                             for m in ARM_IDS + [GRIPPER_ID]}}

    def disable(self):
        with self.lock:
            self.ctrl.disable_all()
            self.enabled = False
        return {"enabled": False}

    def move_joints(self, q_target_real, vlim):
        if not self.enabled:
            raise RuntimeError("motors not enabled (POST /enable first)")
        if self.moving:
            raise RuntimeError("another move is in progress")
        vlim = min(float(vlim), MAX_VLIM)
        q_target_real = [float(v) for v in q_target_real]
        if len(q_target_real) != 6:
            raise ValueError("q must have 6 values (arm joints 1-6)")
        if self.kin:
            q_urdf = self.kin.clamp(self.kin.to_urdf(q_target_real))
            q_target_real = list(self.kin.to_real(q_urdf))

        self.moving = True
        self.abort_flag = False
        torque_strikes = {mid: 0 for mid in ARM_IDS}
        try:
            with self.lock:
                for i, mid in enumerate(ARM_IDS):
                    self.motors[mid].send_pos_vel(pos=q_target_real[i], vlim=vlim)
            t0 = time.time()
            while time.time() - t0 < MOVE_TIMEOUT:
                if self.abort_flag:
                    raise RuntimeError(self.last_error or "aborted")
                errs = [abs(self.state[mid]["pos"] - q_target_real[i])
                        for i, mid in enumerate(ARM_IDS)]
                for mid in ARM_IDS:
                    if abs(self.state[mid]["torq"]) > self.torque_abort[mid]:
                        torque_strikes[mid] += 1
                        if torque_strikes[mid] >= 3:
                            raise RuntimeError(self._estop(
                                f"motor {mid} torque {self.state[mid]['torq']:.1f} Nm"))
                    else:
                        torque_strikes[mid] = 0
                if max(errs) < MOVE_TOL:
                    return {"reached": True, "max_err_rad": max(errs),
                            "elapsed_s": round(time.time() - t0, 2)}
                time.sleep(0.05)
            return {"reached": False, "max_err_rad": max(errs),
                    "elapsed_s": MOVE_TIMEOUT,
                    "note": "timeout: target not reached"}
        finally:
            self.moving = False

    def move_pose(self, xyz, rpy, vlim):
        if not self.kin:
            raise RuntimeError(f"kinematics unavailable: {self.kin_error}")
        q_seed = self.q_real()
        q_sol, err = self.kin.ik(xyz, rpy, q_seed)
        if err > 5e-3:
            raise RuntimeError(f"IK did not converge (err={err:.4f}); "
                               "target may be unreachable")
        res = self.move_joints(list(q_sol), vlim)
        res["ik_err"] = err
        res["q_solution"] = list(map(float, q_sol))
        return res

    def servo(self, xyz, rpy, vlim=0.8, gripper=None):
        """Non-blocking streaming pose command for teleoperation.

        Unlike move_pose (blocking, settles), servo solves IK from the current
        pose (warm seed), clamps the per-call joint step to SERVO_MAX_STEP_RAD,
        sends pos_vel targets, and returns immediately. Call it at 10-30 Hz.
        Safety: URDF limit clamp + step clamp + vlim cap; the poll-loop torque
        watchdog (armed while servoing) and temp watchdog stay active.
        """
        if not self.enabled:
            raise RuntimeError("motors not enabled (POST /enable first)")
        if self.moving:
            raise RuntimeError("a blocking move is in progress")
        if not self.kin:
            raise RuntimeError(f"kinematics unavailable: {self.kin_error}")
        if self.abort_flag:
            raise RuntimeError(self.last_error or "estopped: POST /enable to re-arm")
        vlim = min(float(vlim), MAX_VLIM)
        q_now = self.q_real()
        # Warm-seeded IK converges in a few iters for the small deltas of a
        # 20 Hz stream; a momentary non-convergence just holds this frame.
        q_sol, err = self.kin.ik(xyz, rpy, q_now, iters=60)
        if err > 5e-3:
            return {"sent": False, "ik_err": err, "note": "IK not converged; frame held"}
        step = np.clip(np.asarray(q_sol) - np.asarray(q_now),
                       -SERVO_MAX_STEP_RAD, SERVO_MAX_STEP_RAD)
        q_cmd = [float(v) for v in np.asarray(q_now) + step]
        with self.lock:
            for i, mid in enumerate(ARM_IDS):
                self.motors[mid].send_pos_vel(pos=q_cmd[i], vlim=vlim)
            if gripper is not None:
                g = float(np.clip(gripper,
                                  min(self.args.gripper_open, self.args.gripper_close),
                                  max(self.args.gripper_open, self.args.gripper_close)))
                self.motors[GRIPPER_ID].send_pos_vel(pos=g, vlim=2.5)
        self.servo_until = time.monotonic() + 0.5   # arms the torque watchdog
        return {"sent": True, "ik_err": err, "q_cmd": q_cmd}

    def gripper(self, action=None, pos=None, vlim=2.0):
        if not self.enabled:
            raise RuntimeError("motors not enabled (POST /enable first)")
        cfg = self.args
        if action == "open":
            target = cfg.gripper_open
        elif action == "close":
            target = cfg.gripper_close
        elif pos is not None:
            target = float(pos)
        else:
            raise ValueError("need action=open|close or pos=<rad>")
        if target is None:
            raise RuntimeError(
                "gripper open/close positions not calibrated: pass pos=<rad> "
                "or start daemon with --gripper-open/--gripper-close")

        m = self.motors[GRIPPER_ID]
        with self.lock:
            m.send_pos_vel(pos=target, vlim=min(vlim, 4.0))
        # multi-turn geared mechanism: full travel ~7 rad
        t0 = time.time()
        grasped = False
        strikes = 0
        while time.time() - t0 < 20.0:
            s = self.state.get(GRIPPER_ID)
            if s is None:
                time.sleep(0.05)
                continue
            if action == "close" and abs(s["torq"]) > GRIPPER_CLOSE_TORQUE \
                    and abs(s["pos"] - target) > 0.05:
                strikes += 1
                if strikes >= 3:
                    grasped = True  # sustained torque before full close = object
                    break
            else:
                strikes = 0
            if abs(s["pos"] - target) < 0.02:
                break
            time.sleep(0.05)
        s = self.state.get(GRIPPER_ID, {})
        return {"target": target, "pos": s.get("pos"),
                "torque": s.get("torq"), "object_grasped": grasped}

    # ---------------- readouts ----------------
    def get_state(self):
        return {"t": self.state_t, "enabled": self.enabled,
                "moving": self.moving, "last_error": self.last_error,
                "joints": {str(k): v for k, v in self.state.items()}}

    def get_pose(self):
        if not self.kin:
            raise RuntimeError(f"kinematics unavailable: {self.kin_error}")
        pose = self.kin.fk(self.q_real())
        pose["q_real"] = self.q_real()
        pose["frame"] = EE_FRAME
        return pose

    def health(self):
        transport = (self.args.serial_port if self.args.vendor == "damiao"
                     else self.args.channel)
        return {"ok": True, "vendor": self.args.vendor, "serial": transport,
                "state_age_s": round(time.time() - self.state_t, 3),
                "joints_seen": sorted(self.state.keys()),
                "enabled": self.enabled,
                "kinematics": "ok" if self.kin else self.kin_error,
                "mirror": f"udp://{self.args.mirror_host}:{self.args.mirror_port}"
                          if self.mirror else None}


def make_handler(daemon: ArmDaemon):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # quiet
            pass

        def _send(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _dispatch(self, fn, *a, **kw):
            try:
                self._send(200, fn(*a, **kw))
            except Exception as e:
                self._send(400, {"error": f"{type(e).__name__}: {e}"})

        def do_GET(self):
            if self.path == "/health":
                self._dispatch(daemon.health)
            elif self.path == "/state":
                self._dispatch(daemon.get_state)
            elif self.path == "/pose":
                self._dispatch(daemon.get_pose)
            else:
                self._send(404, {"error": "unknown endpoint"})

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                return self._send(400, {"error": "invalid JSON body"})
            if self.path == "/enable":
                self._dispatch(daemon.enable)
            elif self.path == "/disable":
                self._dispatch(daemon.disable)
            elif self.path == "/estop":
                self._send(200, {"estop": daemon._estop("user request")})
            elif self.path == "/move_joints":
                self._dispatch(daemon.move_joints, body.get("q"),
                               body.get("vlim", DEFAULT_VLIM))
            elif self.path == "/move_pose":
                self._dispatch(daemon.move_pose, body.get("xyz"),
                               body.get("rpy", [0, 0, 0]),
                               body.get("vlim", DEFAULT_VLIM))
            elif self.path == "/servo":
                self._dispatch(daemon.servo, body.get("xyz"),
                               body.get("rpy", [0, 0, 0]),
                               body.get("vlim", 0.8), body.get("gripper"))
            elif self.path == "/gripper":
                self._dispatch(daemon.gripper, body.get("action"),
                               body.get("pos"), body.get("vlim", 2.0))
            elif self.path == "/rehome_gripper":
                self._dispatch(daemon.rehome_gripper)
            else:
                self._send(404, {"error": "unknown endpoint"})
    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_vendor_args(ap)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--http-port", type=int, default=5810)
    ap.add_argument("--mirror-host", default="127.0.0.1")
    ap.add_argument("--mirror-port", type=int, default=5801,
                    help="UDP mirror for Isaac Sim twin (0 to disable)")
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--joint-signs", type=float, nargs=6,
                    default=[-1, -1, -1, -1, -1, 1],
                    help="real->URDF sign mapping for joints 1-6 "
                         "(default verified 2026-07-05 via gravity-torque "
                         "match: [-1,-1,-1,-1,-1,1])")
    ap.add_argument("--gripper-open", type=float, default=-6.8,
                    help="motor-7 pos for fingers fully open (calibrated "
                         "2026-07-05: stop at -7.02, margin 0.2)")
    ap.add_argument("--gripper-close", type=float, default=0.0,
                    help="motor-7 pos for fingers closed (calibrated "
                         "2026-07-05: stop at +0.10, jaws touch ~0.0)")
    args = ap.parse_args()
    if args.mirror_port == 0:
        args.mirror_port = None

    daemon = ArmDaemon(args)
    srv = ThreadingHTTPServer(("127.0.0.1", args.http_port),
                              make_handler(daemon))
    transport = (f"serial {args.serial_port}" if args.vendor == "damiao"
                 else f"socketcan {args.channel}")
    print(f"reBot Arm daemon on http://127.0.0.1:{args.http_port}  "
          f"({args.vendor}: {transport}, motors passive until /enable)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        daemon.ctrl.shutdown()


if __name__ == "__main__":
    main()
