# Runs INSIDE Isaac Sim via scripts/isaacsim_client.py.
# Fixes "Number of Newton contacts (N) exceeded MJWarp limit (200). Increase nconmax."
#
# The MuJoCo-Warp solver allocates a fixed contact buffer (nconmax, default 200)
# and constraint buffer (njmax, default 1200). Scenes where the gripper's
# convex-decomposition colliders touch a ground plane / table can exceed that.
# This script raises both caps and rebuilds the MuJoCo model (stop -> invalidate
# -> play), which is required for the new limits to take effect.
#
#   python scripts/isaacsim_client.py --timeout 300 --file isaac/04_fix_newton_contacts.py
#
# Optional: pass --arg nconmax=8192 --arg njmax=32768 to override the defaults.

import omni.timeline
import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.physics.newton.impl import extension as _next

if "nconmax" not in dir():
    nconmax = 8192
if "njmax" not in dir():
    njmax = 4 * nconmax

ns = _next.acquire_stage()
assert ns is not None, "Newton stage not initialized — is the Newton engine active?"

cfg = ns.cfg.solver_cfg
print(f"current: nconmax={cfg.nconmax} njmax={cfg.njmax}")

# Respect Newton's own rigid-contact buffer if a model already exists: MJWarp
# cannot consume more contacts than Newton allocates, and raising nconmax past
# it makes every step error out (hard-won pitfall on this asset).
model = getattr(ns, "model", None)
rigid_cap = getattr(model, "rigid_contact_max", None) if model is not None else None
if rigid_cap:
    print(f"newton model rigid_contact_max={rigid_cap}")
    nconmax = min(int(nconmax), int(rigid_cap))

cfg.nconmax = int(nconmax)
cfg.njmax = int(njmax)
print(f"new: nconmax={cfg.nconmax} njmax={cfg.njmax}")

# Rebuild: the MuJoCo model bakes the caps at construction time.
tl = omni.timeline.get_timeline_interface()
was_playing = tl.is_playing()
if was_playing:
    tl.stop()
    await app_utils.update_app_async(steps=10)  # noqa: F704
ns.initialized = False  # force model reconstruction on next play
if was_playing:
    tl.play()
    await app_utils.update_app_async(steps=30)  # noqa: F704
    print("timeline restarted — MuJoCo model rebuilt with new caps")
else:
    print("timeline was stopped — caps take effect on next play")
