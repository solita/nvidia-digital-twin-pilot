from __future__ import annotations

import asyncio
import builtins
import runpy
import traceback
from pathlib import Path

import carb
import omni.kit.app
import omni.timeline
import omni.usd

_THIS_FILE = Path(__file__).resolve()
SCRIPT_DIR = (
    _THIS_FILE.parent
    if _THIS_FILE.parent.name == "02_core_scripts"
    else Path("/isaac-sim/.local/share/ov/data/concrete_spray/02_core_scripts")
)
LAUNCHER_SCRIPT = SCRIPT_DIR / "shotcrete_direct_cave_preview_launcher_v1.py"
STATE_KEY = "_SHOTCRETE_DIRECT_CAVE_PREVIEW_STATE"
VALIDATION_TIMEOUT_S = 180.0


def log(message: str) -> None:
    tagged = f"[shotcrete_direct_cave_preview_validate] {message}"
    carb.log_info(tagged)
    print(tagged)


def err(message: str) -> None:
    tagged = f"[shotcrete_direct_cave_preview_validate] {message}"
    carb.log_error(tagged)
    print(tagged)


async def _next_updates(count: int = 1) -> None:
    app = omni.kit.app.get_app()
    for _ in range(max(0, count)):
        await app.next_update_async()


async def _shutdown() -> None:
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing() or not timeline.is_stopped():
        timeline.stop()
        await _next_updates(2)
    usd_context = omni.usd.get_context()
    try:
        if usd_context.get_stage() is not None:
            usd_context.close_stage()
            await _next_updates(5)
    except Exception:
        pass
    omni.kit.app.get_app().post_quit()


async def _wait_for_preview(launcher_task) -> None:
    app = omni.kit.app.get_app()
    deadline = app.get_time_since_start_s() + VALIDATION_TIMEOUT_S

    while app.get_time_since_start_s() < deadline:
        if launcher_task is not None and launcher_task.done():
            exc = launcher_task.exception()
            if exc is not None:
                raise exc

        state = getattr(builtins, STATE_KEY, None)
        if state is not None and state.get("finalized"):
            log(
                f"Validation finalized run_id={state.get('run_id')} "
                f"captured={state.get('captured_particles')} deposits={len(state.get('deposit_positions', []))}"
            )
            return

        await app.next_update_async()

    state = getattr(builtins, STATE_KEY, None)
    if state is None:
        raise TimeoutError("Timed out waiting for the preview state to appear.")
    raise TimeoutError(
        f"Timed out waiting for preview finalization. run_id={state.get('run_id')} "
        f"last_frame={state.get('last_frame')} finalized={state.get('finalized')}"
    )


async def _main() -> None:
    if not LAUNCHER_SCRIPT.exists():
        raise FileNotFoundError(f"Missing launcher script: {LAUNCHER_SCRIPT}")

    log(f"Starting validation with launcher: {LAUNCHER_SCRIPT.name}")
    launcher_globals = runpy.run_path(str(LAUNCHER_SCRIPT), run_name="shotcrete_direct_cave_preview_launcher_validation")
    launcher_task = launcher_globals.get("_SHOTCRETE_DIRECT_CAVE_PREVIEW_LAUNCHER_TASK")
    await _next_updates(2)
    await _wait_for_preview(launcher_task)
    await _shutdown()


async def _guarded_main() -> None:
    try:
        await _main()
    except Exception as exc:
        err(f"Validation failed: {exc}")
        traceback.print_exc()
        carb.log_error(traceback.format_exc())
        await _shutdown()


globals()["_SHOTCRETE_DIRECT_CAVE_PREVIEW_VALIDATE_TASK"] = asyncio.ensure_future(_guarded_main())
