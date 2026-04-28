"""
forklift_launcher.py — Isaac Sim entry point for the forklift warehouse simulation.

Opens scene_assembly.usd, starts the timeline, then hands off to
forklift_controller.py which drives the forklift in an infinite patrol loop.

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
"""
from __future__ import annotations

import asyncio
import runpy
import sys
from pathlib import Path

import carb
import omni.kit.app
import omni.timeline
import omni.usd

# ── Paths ──────────────────────────────────────────────────────────────────────
# __file__ is unreliable when run via "Isaac Sim: Run File Remotely" — it
# resolves to the VS Code extension dir, not this file.  Hardcode the
# container-side paths directly (host volume is mounted at the prefix below).

_CONTAINER_ROOT = "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot"
SCRIPT_DIR    = Path(_CONTAINER_ROOT) / "simulations/forklift-warehouse/02_core_scripts"
WORKSPACE_DIR = Path(_CONTAINER_ROOT) / "simulations/forklift-warehouse"

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

SCENE_FILE  = WORKSPACE_DIR / "01_scenes" / "scene_assembly.usd"
MAIN_SCRIPT = SCRIPT_DIR   / "forklift_controller.py"

STAGE_READY_TIMEOUT_S = 90.0
SETTLE_UPDATES = 5


# ── Helpers ───────────────────────────────────────────────────────────────────

def log(msg: str) -> None:
    tagged = f"[launcher] {msg}"
    carb.log_info(tagged)
    print(tagged)


def err(msg: str) -> None:
    tagged = f"[launcher] ERROR: {msg}"
    carb.log_error(tagged)
    print(tagged)


async def _next_updates(n: int) -> None:
    app = omni.kit.app.get_app()
    for _ in range(max(0, n)):
        await app.next_update_async()


async def _wait_for_stage(timeout: float) -> bool:
    import time
    ctx = omni.usd.get_context()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if ctx.get_stage() is not None:
            return True
        await _next_updates(1)
    return False


# ── Main ──────────────────────────────────────────────────────────────────────

async def main() -> None:
    log(f"Opening scene: {SCENE_FILE}")

    if not SCENE_FILE.exists():
        err(f"Scene not found: {SCENE_FILE}")
        return

    ctx = omni.usd.get_context()
    await ctx.open_stage_async(str(SCENE_FILE).replace("\\", "/"))
    await _next_updates(SETTLE_UPDATES)

    if not await _wait_for_stage(STAGE_READY_TIMEOUT_S):
        err("Timed out waiting for stage to load.")
        return

    log("Stage loaded — starting timeline.")
    omni.timeline.get_timeline_interface().play()
    await _next_updates(2)

    log(f"Running forklift controller: {MAIN_SCRIPT}")
    try:
        runpy.run_path(str(MAIN_SCRIPT), run_name="__main__")
    except Exception as exc:
        err(f"Controller failed to load: {exc}")
        import traceback
        traceback.print_exc()


asyncio.ensure_future(main())
