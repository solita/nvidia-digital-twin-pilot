from __future__ import annotations

import asyncio
import os
import runpy
import sys
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
    else Path("/isaac-sim/.local/share/ov/data/workshop2/actual-files/02_core_scripts")
)
WORKSPACE_DIR = SCRIPT_DIR.parent

PREVIEW_SCENE = WORKSPACE_DIR / "01_scenes" / "boolean Applied ver3_preview_particles.usdc"
PREVIEW_SCRIPT = SCRIPT_DIR / "shotcrete_direct_cave_preview_v1.py"

STAGE_READY_TIMEOUT_S = 90.0
SETTLE_UPDATES_AFTER_STAGE_OPEN = 5


def log(message: str) -> None:
    tagged = f"[shotcrete_direct_cave_preview_launcher] {message}"
    carb.log_info(tagged)
    print(tagged)


def err(message: str) -> None:
    tagged = f"[shotcrete_direct_cave_preview_launcher] {message}"
    carb.log_error(tagged)
    print(tagged)


def _to_stage_url(path: Path) -> str:
    return str(path).replace("\\", "/")


async def _next_updates(count: int) -> None:
    app = omni.kit.app.get_app()
    for _ in range(max(0, count)):
        await app.next_update_async()


async def _set_timeline_state(*, play: bool) -> None:
    timeline = omni.timeline.get_timeline_interface()
    if play:
        timeline.play()
        await _next_updates(2)
        return

    if timeline.is_playing() or not timeline.is_stopped():
        timeline.stop()
        await _next_updates(2)


async def _wait_for_stage_openable(timeout_s: float = 30.0) -> None:
    usd_context = omni.usd.get_context()
    app = omni.kit.app.get_app()
    deadline = app.get_time_since_start_s() + timeout_s
    while app.get_time_since_start_s() < deadline:
        if usd_context.can_open_stage():
            return
        await app.next_update_async()
    raise TimeoutError("Timed out waiting for the USD context to become ready for stage open.")


async def _wait_for_stage_ready(label: str, timeout_s: float = STAGE_READY_TIMEOUT_S) -> None:
    usd_context = omni.usd.get_context()
    app = omni.kit.app.get_app()
    deadline = app.get_time_since_start_s() + timeout_s
    stable_ready_frames = 0

    while app.get_time_since_start_s() < deadline:
        stage = usd_context.get_stage()
        loading_message, loaded_count, total_count = usd_context.get_stage_loading_status()
        streaming_busy = usd_context.get_stage_streaming_status()
        ready = stage is not None and not streaming_busy and (total_count == 0 or loaded_count >= total_count)

        if ready:
            stable_ready_frames += 1
            if stable_ready_frames >= 3:
                await _next_updates(SETTLE_UPDATES_AFTER_STAGE_OPEN)
                log(f"Stage ready: {label}")
                return
        else:
            stable_ready_frames = 0

        await app.next_update_async()

    current_url = usd_context.get_stage_url()
    raise TimeoutError(
        f"Timed out waiting for stage readiness: {label}. Current stage URL: {current_url!r}; "
        f"loading={loading_message!r} loaded={loaded_count}/{total_count} streaming_busy={streaming_busy}."
    )


async def _open_stage(stage_path: Path, label: str) -> None:
    if not stage_path.exists():
        raise FileNotFoundError(f"Missing stage file: {stage_path}")

    usd_context = omni.usd.get_context()
    await _set_timeline_state(play=False)
    await _wait_for_stage_openable()

    stage_url = _to_stage_url(stage_path)
    log(f"Opening stage: {stage_url}")
    ok, error_message = await usd_context.open_stage_async(stage_url)
    if not ok:
        raise RuntimeError(f"Failed to open stage {stage_path}: {error_message}")

    await _wait_for_stage_ready(label)


def _run_script_file(script_path: Path, label: str) -> None:
    if not script_path.exists():
        raise FileNotFoundError(f"Missing script file: {script_path}")

    cwd_before = Path.cwd()
    parent_str = str(script_path.parent)
    inserted_sys_path = False

    try:
        os.chdir(parent_str)
        if parent_str not in sys.path:
            sys.path.insert(0, parent_str)
            inserted_sys_path = True
        log(f"Running {label} script: {script_path.name}")
        runpy.run_path(str(script_path), run_name="__main__")
    finally:
        if inserted_sys_path and parent_str in sys.path:
            sys.path.remove(parent_str)
        os.chdir(str(cwd_before))


async def _launcher_main() -> None:
    log("Direct cave preview launcher started.")
    log(f"Preview scene: {PREVIEW_SCENE}")
    log(f"Preview script: {PREVIEW_SCRIPT}")

    if not PREVIEW_SCENE.exists():
        raise FileNotFoundError(f"Missing preview scene: {PREVIEW_SCENE}")
    if not PREVIEW_SCRIPT.exists():
        raise FileNotFoundError(f"Missing preview script: {PREVIEW_SCRIPT}")

    await _open_stage(PREVIEW_SCENE, "direct cave preview scene")
    _run_script_file(PREVIEW_SCRIPT, "direct cave preview")
    await _next_updates(2)
    await _set_timeline_state(play=True)
    log("Preview is running. Watch the viewport for spray flight, cave hits, and sticking deposits.")


async def _guarded_launcher_main() -> None:
    try:
        await _launcher_main()
    except Exception as exc:
        await _set_timeline_state(play=False)
        err(f"Launcher failed: {exc}")
        traceback.print_exc()
        carb.log_error(traceback.format_exc())


_existing_task = globals().get("_SHOTCRETE_DIRECT_CAVE_PREVIEW_LAUNCHER_TASK")
if _existing_task is not None and not _existing_task.done():
    log("Launcher is already running; refusing to start a second concurrent copy.")
else:
    globals()["_SHOTCRETE_DIRECT_CAVE_PREVIEW_LAUNCHER_TASK"] = asyncio.ensure_future(_guarded_launcher_main())
