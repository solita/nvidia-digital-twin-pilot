from __future__ import annotations

"""
Shotcrete one-click pipeline for Isaac Sim.

What it does:
1. Opens the clean export scene.
2. Runs the exporter script.
3. Auto-plays the timeline for the export phase.
4. Waits until the latest proxy-field file updates.
5. Switches to the cave import scene without saving either USD scene.
6. Runs the importer script.
7. Waits until the latest cave-map output updates.
8. Prints a small verification summary.

Important workflow rule encoded here:
- Save edited Python scripts.
- Do NOT save either USD working scene during the loop.
"""

import asyncio
import os
import re
import runpy
import sys
import traceback
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

import carb
import omni.kit.app
import omni.timeline
import omni.usd

SCRIPTS_DIR = Path(__file__).resolve().parent
EXPORT_SCENE = SCRIPTS_DIR.parent / "01_scenes" / "CAVE_clean_master.usdc"
IMPORT_SCENE = SCRIPTS_DIR.parent / "01_scenes" / "boolean Applied ver3_calibration.usdc"

EXPORTER_SCRIPT = SCRIPTS_DIR / "shotcrete_microburst_v16_2_proxy_receiver_exact_field_export.py"
IMPORTER_SCRIPT = SCRIPTS_DIR / "shotcrete_proxy_to_cave_map_v17_8b_exact_field_import_new_cave_autodetect.py"

EXPORT_LATEST_FIELD_JSON = SCRIPTS_DIR / "shotcrete_proxy_field_v16_2_latest.json"
EXPORT_LATEST_FIELD_TXT = SCRIPTS_DIR / "shotcrete_proxy_field_v16_2_latest.txt"
EXPORT_LATEST_METRICS_JSON = SCRIPTS_DIR / "shotcrete_metrics_broad_splat_v16_2_latest.json"
EXPORT_LATEST_METRICS_TXT = SCRIPTS_DIR / "shotcrete_metrics_broad_splat_v16_2_latest.txt"

IMPORT_LATEST_TXT = SCRIPTS_DIR / "shotcrete_proxy_to_cave_map_v17_8b_latest.txt"
IMPORT_LATEST_JSON = SCRIPTS_DIR / "shotcrete_proxy_to_cave_map_v17_8b_latest.json"

# Tuning knobs for orchestration only.
EXPORT_TIMEOUT_S = 240.0
IMPORT_TIMEOUT_S = 120.0
STAGE_READY_TIMEOUT_S = 90.0
SETTLE_UPDATES_AFTER_STAGE_OPEN = 5
SETTLE_UPDATES_AFTER_FILE_CHANGE = 10
CLEAR_DIRTY_FLAG_BEFORE_STAGE_SWITCH = True
CLEAR_DIRTY_FLAG_AT_END = True
AUTO_PLAY_EXPORT = True
AUTO_PLAY_IMPORT = False  # Keep False unless the importer later proves it needs Play.

RUN_ID_RE = re.compile(r"\b(?:source_run_id|run_id)\s*[:=]\s*[\"']?([0-9]{8}_[0-9]{6})", re.IGNORECASE)
SOURCE_RUN_ID_RE = re.compile(r"\bsource_run_id\s*[:=]\s*[\"']?([0-9]{8}_[0-9]{6})", re.IGNORECASE)
GENERIC_RUN_ID_RE = re.compile(r"\b(20[0-9]{6}_[0-9]{6})\b")
COVERAGE_RATIO_RE = re.compile(r"\bcoverage(?:\s+ratio)?\s*[:=]\s*([0-9.]+)", re.IGNORECASE)
ACCEPT_RATIO_RE = re.compile(r"\baccept(?:\s+fraction|/coverage ratio|\s+ratio)?\s*[:=]\s*([0-9.]+)", re.IGNORECASE)
VOLUME_RATIO_RE = re.compile(r"\bvolume(?:\s+ratio)?\s*[:=]\s*([0-9.]+)", re.IGNORECASE)
ACCEPTED_RE = re.compile(r"\baccepted(?:\s+cells)?\s*[:=]\s*([0-9]+)", re.IGNORECASE)
SOURCE_CELLS_RE = re.compile(r"\bsource(?:\s+cells)?\s*[:=]\s*([0-9]+)", re.IGNORECASE)
REJECTED_RE = re.compile(r"\brejected(?:\s+cells)?\s*[:=]\s*([0-9]+)", re.IGNORECASE)


@dataclass
class Verification:
    gate_path: Path
    before_mtime_ns: int | None
    after_mtime_ns: int | None
    run_id: str | None = None
    source_run_id: str | None = None
    coverage_ratio: str | None = None
    volume_ratio: str | None = None
    accepted: str | None = None
    source_cells: str | None = None
    rejected: str | None = None


def log(message: str) -> None:
    tagged = f"[shotcrete_pipeline] {message}"
    carb.log_info(tagged)
    print(tagged)


def _err(message: str) -> None:
    tagged = f"[shotcrete_pipeline] {message}"
    carb.log_error(tagged)
    print(tagged)


def _to_stage_url(path: Path) -> str:
    return str(path).replace("\\", "/")


def _mtime_ns(path: Path) -> int | None:
    try:
        return path.stat().st_mtime_ns
    except FileNotFoundError:
        return None


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return ""
    except UnicodeDecodeError:
        return path.read_text(encoding="utf-8-sig", errors="replace")


def _extract_first(regex: re.Pattern[str], text: str) -> str | None:
    match = regex.search(text)
    return match.group(1) if match else None


def _extract_run_id(text: str) -> str | None:
    match = RUN_ID_RE.search(text)
    if match:
        return match.group(1)
    match = GENERIC_RUN_ID_RE.search(text)
    return match.group(1) if match else None


def _collect_verification(gate_path: Path, before_mtime_ns: int | None, parse_paths: Iterable[Path]) -> Verification:
    verification = Verification(
        gate_path=gate_path,
        before_mtime_ns=before_mtime_ns,
        after_mtime_ns=_mtime_ns(gate_path),
    )

    for path in parse_paths:
        text = _read_text(path)
        if not text:
            continue

        verification.run_id = verification.run_id or _extract_run_id(text)
        verification.source_run_id = verification.source_run_id or _extract_first(SOURCE_RUN_ID_RE, text)
        verification.coverage_ratio = verification.coverage_ratio or _extract_first(COVERAGE_RATIO_RE, text)
        verification.coverage_ratio = verification.coverage_ratio or _extract_first(ACCEPT_RATIO_RE, text)
        verification.volume_ratio = verification.volume_ratio or _extract_first(VOLUME_RATIO_RE, text)
        verification.accepted = verification.accepted or _extract_first(ACCEPTED_RE, text)
        verification.source_cells = verification.source_cells or _extract_first(SOURCE_CELLS_RE, text)
        verification.rejected = verification.rejected or _extract_first(REJECTED_RE, text)

    return verification


async def _next_updates(count: int) -> None:
    app = omni.kit.app.get_app()
    for _ in range(max(0, count)):
        await app.next_update_async()


async def _set_timeline_state(*, play: bool) -> None:
    timeline = omni.timeline.get_timeline_interface()
    if play:
        log("Starting timeline for export phase.")
        timeline.play()
        await _next_updates(2)
        return

    if timeline.is_playing() or not timeline.is_stopped():
        log("Stopping timeline.")
        timeline.stop()
        await _next_updates(2)


async def _clear_dirty_flag_if_needed(reason: str) -> None:
    usd_context = omni.usd.get_context()
    if usd_context.has_pending_edit():
        log(f"Clearing dirty flag ({reason}) to enforce DO NOT SAVE for the USD scenes.")
        usd_context.set_pending_edit(False)
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

    if CLEAR_DIRTY_FLAG_BEFORE_STAGE_SWITCH:
        await _clear_dirty_flag_if_needed(f"before opening {stage_path.name}")

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


async def _wait_for_file_update(gate_path: Path, before_mtime_ns: int | None, timeout_s: float, label: str) -> None:
    app = omni.kit.app.get_app()
    deadline = app.get_time_since_start_s() + timeout_s

    while app.get_time_since_start_s() < deadline:
        current_mtime_ns = _mtime_ns(gate_path)
        changed = current_mtime_ns is not None and (before_mtime_ns is None or current_mtime_ns > before_mtime_ns)
        if changed:
            await _next_updates(SETTLE_UPDATES_AFTER_FILE_CHANGE)
            log(f"Detected updated output for {label}: {gate_path.name}")
            return
        await app.next_update_async()

    raise TimeoutError(f"Timed out waiting for updated output: {gate_path}")


def _format_verification(name: str, result: Verification) -> str:
    parts = [f"{name}: {result.gate_path.name}"]
    if result.run_id:
        parts.append(f"run_id={result.run_id}")
    if result.source_run_id:
        parts.append(f"source_run_id={result.source_run_id}")
    if result.source_cells:
        parts.append(f"source_cells={result.source_cells}")
    if result.accepted:
        parts.append(f"accepted={result.accepted}")
    if result.rejected:
        parts.append(f"rejected={result.rejected}")
    if result.coverage_ratio:
        parts.append(f"coverage_ratio={result.coverage_ratio}")
    if result.volume_ratio:
        parts.append(f"volume_ratio={result.volume_ratio}")
    if result.after_mtime_ns is not None:
        parts.append(f"mtime_ns={result.after_mtime_ns}")
    return " | ".join(parts)


async def _run_export_phase() -> Verification:
    await _open_stage(EXPORT_SCENE, "export scene")

    export_before = _mtime_ns(EXPORT_LATEST_FIELD_JSON)
    _run_script_file(EXPORTER_SCRIPT, "exporter")

    if AUTO_PLAY_EXPORT:
        await _set_timeline_state(play=True)

    await _wait_for_file_update(
        gate_path=EXPORT_LATEST_FIELD_JSON,
        before_mtime_ns=export_before,
        timeout_s=EXPORT_TIMEOUT_S,
        label="export phase",
    )

    if AUTO_PLAY_EXPORT:
        await _set_timeline_state(play=False)

    return _collect_verification(
        gate_path=EXPORT_LATEST_FIELD_JSON,
        before_mtime_ns=export_before,
        parse_paths=(
            EXPORT_LATEST_FIELD_JSON,
            EXPORT_LATEST_FIELD_TXT,
            EXPORT_LATEST_METRICS_JSON,
            EXPORT_LATEST_METRICS_TXT,
        ),
    )


async def _run_import_phase() -> Verification:
    await _open_stage(IMPORT_SCENE, "import scene")

    import_before = _mtime_ns(IMPORT_LATEST_TXT)
    _run_script_file(IMPORTER_SCRIPT, "importer")

    if AUTO_PLAY_IMPORT:
        await _set_timeline_state(play=True)

    await _wait_for_file_update(
        gate_path=IMPORT_LATEST_TXT,
        before_mtime_ns=import_before,
        timeout_s=IMPORT_TIMEOUT_S,
        label="import phase",
    )

    if AUTO_PLAY_IMPORT:
        await _set_timeline_state(play=False)

    return _collect_verification(
        gate_path=IMPORT_LATEST_TXT,
        before_mtime_ns=import_before,
        parse_paths=(IMPORT_LATEST_TXT, IMPORT_LATEST_JSON),
    )


async def _pipeline_main() -> None:
    log("One-click shotcrete pipeline started.")
    log("Save policy: save edited Python scripts; DO NOT SAVE the two USD working scenes.")

    for required_path in (EXPORT_SCENE, IMPORT_SCENE, EXPORTER_SCRIPT, IMPORTER_SCRIPT):
        if not required_path.exists():
            raise FileNotFoundError(f"Required path is missing: {required_path}")

    export_result = await _run_export_phase()
    log(_format_verification("Export verification", export_result))

    if CLEAR_DIRTY_FLAG_BEFORE_STAGE_SWITCH:
        await _clear_dirty_flag_if_needed("after export phase and before import scene switch")

    import_result = await _run_import_phase()
    log(_format_verification("Import verification", import_result))

    await _set_timeline_state(play=False)

    if CLEAR_DIRTY_FLAG_AT_END:
        await _clear_dirty_flag_if_needed("after import phase completion")

    log("Pipeline finished. USD scenes were not saved by this runner.")


async def _guarded_pipeline_main() -> None:
    try:
        await _pipeline_main()
    except Exception as exc:
        await _set_timeline_state(play=False)
        _err(f"Pipeline failed: {exc}")
        traceback.print_exc()
        carb.log_error(traceback.format_exc())


_existing_task = globals().get("_SHOTCRETE_ONE_CLICK_PIPELINE_TASK")
if _existing_task is not None and not _existing_task.done():
    log("Pipeline is already running; refusing to start a second concurrent copy.")
else:
    globals()["_SHOTCRETE_ONE_CLICK_PIPELINE_TASK"] = asyncio.ensure_future(_guarded_pipeline_main())
