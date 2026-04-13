from __future__ import annotations

"""
launcher.py — Isaac Sim simulation entry point template.

Rename this file to match your simulation (e.g. my_sim_launcher_v1.py).
Update SCENE_FILE and MAIN_SCRIPT to point to your actual assets and logic.

Run via:
  ./run_sim.sh
  or: VS Code → Isaac Sim: Run Remotely
"""

import asyncio
from pathlib import Path

import carb
import omni.kit.app
import omni.timeline
import omni.usd

# ── Configuration ─────────────────────────────────────────────────────────────

_THIS_FILE = Path(__file__).resolve()
SCRIPT_DIR = _THIS_FILE.parent
WORKSPACE_DIR = SCRIPT_DIR.parent

# Update these paths to match your scene and main script filenames.
SCENE_FILE = WORKSPACE_DIR / "01_scenes" / "your_scene.usdc"
MAIN_SCRIPT = SCRIPT_DIR / "your_sim_script.py"

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

    log(f"Running main script: {MAIN_SCRIPT}")
    # Replace the line below with your actual simulation logic or
    # use runpy.run_path(str(MAIN_SCRIPT)) to execute an external script.
    log("TODO: implement simulation logic in this launcher or in MAIN_SCRIPT.")


asyncio.ensure_future(main())
