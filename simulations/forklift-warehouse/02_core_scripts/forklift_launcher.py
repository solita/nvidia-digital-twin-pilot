from __future__ import annotations

"""
forklift_launcher.py — Isaac Sim entry point for the forklift warehouse simulation.

Opens scene_assembly.usd (or reuses the already-loaded stage), starts the
timeline, then hands off to forklift_controller.py which drives the fleet
in an infinite patrol loop.

Run via VS Code → Isaac Sim: Run File Remotely
"""

import asyncio
import runpy
from pathlib import Path

import carb
import omni.kit.app
import omni.timeline
import omni.usd

# ── Configuration ─────────────────────────────────────────────────────────────

# Container mount paths (from brev-compose.yml volumes)
_CONTAINER_SCENE = "/sim-scripts/scenes/scene_assembly.usd"
_CONTAINER_SCRIPTS = "/sim-scripts/core_scripts"

# Fallback: resolve from __file__ when running outside the container
try:
    _THIS_FILE = Path(__file__).resolve()
    _SCRIPT_DIR = _THIS_FILE.parent
    _WORKSPACE_DIR = _SCRIPT_DIR.parent
    _LOCAL_SCENE = _WORKSPACE_DIR / "01_scenes" / "scene_assembly.usd"
    _LOCAL_CONTROLLER = _SCRIPT_DIR / "forklift_controller.py"
except NameError:
    # __file__ is not defined in Isaac Sim remote-execution context
    _LOCAL_SCENE = None
    _LOCAL_CONTROLLER = None

# Pick whichever path actually exists
def _find_file(*candidates):
    for p in candidates:
        if p is not None and Path(p).exists():
            return Path(p)
    return None

SCENE_FILE  = _find_file(_CONTAINER_SCENE, _LOCAL_SCENE)
MAIN_SCRIPT = _find_file(
    f"{_CONTAINER_SCRIPTS}/forklift_controller.py",
    _LOCAL_CONTROLLER,
)

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
    ctx = omni.usd.get_context()
    stage = ctx.get_stage()

    # ── Load scene if not already open ────────────────────────────────────────
    if stage is not None and stage.GetPrimAtPath("/World").IsValid():
        log("Stage already loaded — skipping scene open.")
    elif SCENE_FILE is not None:
        log(f"Opening scene: {SCENE_FILE}")
        await ctx.open_stage_async(str(SCENE_FILE).replace("\\", "/"))
        await _next_updates(SETTLE_UPDATES)

        if not await _wait_for_stage(STAGE_READY_TIMEOUT_S):
            err("Timed out waiting for stage to load.")
            return
        log("Stage loaded.")
    else:
        err("No scene file found and no stage loaded. "
            "Checked: /sim-scripts/scenes/scene_assembly.usd and __file__-relative path.")
        return

    # ── Start timeline ────────────────────────────────────────────────────────
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        log("Starting timeline.")
        timeline.play()
        await _next_updates(2)

    # ── Run forklift controller ───────────────────────────────────────────────
    if MAIN_SCRIPT is not None:
        log(f"Running forklift controller: {MAIN_SCRIPT}")
        runpy.run_path(str(MAIN_SCRIPT), run_name="__main__")
    else:
        err("forklift_controller.py not found on disk. "
            "Run it separately via 'Isaac Sim: Run File Remotely'.")


asyncio.ensure_future(main())
