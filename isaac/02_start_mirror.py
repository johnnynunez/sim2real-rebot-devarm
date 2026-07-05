# Runs INSIDE Isaac Sim via scripts/isaacsim_client.py.
# Starts the timeline, wraps the arm articulation, and installs a per-frame
# UDP listener (port 5801) that mirrors the real arm onto the sim.
#
#   python scripts/isaacsim_client.py --timeout 150 --file isaac/02_start_mirror.py
#
# First run on Newton compiles CUDA kernels (~1-2 min); later runs are fast.
# Stop the mirror later with:  mirror_sub.unsubscribe()

import json
import socket

import numpy as np
import omni.kit.app
import omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
import isaacsim.core.experimental.utils.app as app_utils

ARTICULATION_PATH = "/tn__00armrs_asmv3_hJ6D/Geometry/base_link"
UDP_PORT = 5801

# USD<->URDF sign convention for this asset (validated by FK round-trip and live mirror)
SIGN = np.array([-1.0, -1.0, -1.0, -1.0, -1.0, 1.0], dtype=np.float32)
GRIP_SCALE = 1.0  # motor 7 rad -> finger joint value (calibrate if needed)

tl = omni.timeline.get_timeline_interface()
if not tl.is_playing():
    tl.play()
await app_utils.update_app_async(steps=60)  # noqa: F704  (python_server supports top-level await)

art = Articulation(ARTICULATION_PATH)
assert art.num_dofs == 8, "0 DOFs: articulation must be created AFTER play"
print("engine:", SimulationManager.get_active_physics_engine())
print("dof_names:", list(art.dof_names))

cur = art.get_dof_positions().numpy()[0].astype(np.float32)
# Anti-sway: latch targets to the current pose before streaming begins
art.set_dof_position_targets(cur.reshape(1, -1))

lower, upper = [x.numpy()[0].astype(np.float32) for x in art.get_dof_limits()]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("127.0.0.1", UDP_PORT))
sock.setblocking(False)

state = {"pkts": 0, "last_q": None, "tgt": cur.copy()}


def on_update(e):
    latest = None
    while True:  # drain the socket, keep only the newest packet
        try:
            data, _ = sock.recvfrom(4096)
            latest = data
        except BlockingIOError:
            break
    if latest is None:
        return
    try:
        msg = json.loads(latest.decode())
        q = msg["q"]
        tgt = state["tgt"]
        for i in range(6):
            v = SIGN[i] * float(q[str(i + 1)])
            tgt[i] = np.clip(v, lower[i], upper[i])
        g = GRIP_SCALE * float(q["7"])
        tgt[6] = np.clip(g, lower[6], upper[6])
        tgt[7] = np.clip(g, lower[7], upper[7])
        art.set_dof_position_targets(tgt.reshape(1, -1))
        state["pkts"] += 1
        state["last_q"] = q
    except Exception as ex:  # keep the update loop alive; surface via 03_mirror_status
        state["err"] = repr(ex)


mirror_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
print(f"mirror listener active on udp 127.0.0.1:{UDP_PORT}")
