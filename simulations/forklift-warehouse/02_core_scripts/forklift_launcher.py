from __future__ import annotations

"""
forklift_launcher.py — Isaac Sim entry point for the forklift warehouse simulation.

Opens scene_assembly.usd, starts the timeline, then hands off to
forklift_controller.py which drives the forklift in an infinite patrol loop.

Run via:
  ./run_sim.sh
  or: VS Code → Isaac Sim: Run File Remotely
"""

import asyncio
import runpy
from pathlib import Path

import carb
import omni.kit.app
import omni.timeline
import omni.usd

# ── Configuration ─────────────────────────────────────────────────────────────

_THIS_FILE = Path(__file__).resolve()
SCRIPT_DIR = _THIS_FILE.parent
WORKSPACE_DIR = SCRIPT_DIR.parent

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
    runpy.run_path(str(MAIN_SCRIPT), run_name="__main__")
        err("Timed out waiting for stage to load.")
        return

    log("Stage loaded — starting timeline.")
    omni.timeline.get_timeline_interface().play()
    await _next_updates(2)

    log(f"Running main script: {MAIN_SCRIPT}")
    # Replace the line below with your actual simulation logic or
    # use runpy.run_path(str(MAIN_SCRIPT)) to execute an external script.
    log("TODO: implement simulation logic in this launcher or in MAIN_SCRIPT.")


asyncio.ensure_future(main())
