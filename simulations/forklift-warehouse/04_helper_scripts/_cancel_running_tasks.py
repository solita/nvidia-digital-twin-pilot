"""
_cancel_running_tasks.py — Cancel only forklift/scan tasks (safe emergency stop).
Run via: Ctrl+Shift+P → Isaac Sim: Run File Remotely
"""
import asyncio
import carb

# Only cancel tasks whose coroutine name matches our scripts.
# Never cancels Isaac Sim internal tasks (omni.*, uvicorn, starlette, etc.)
CANCEL_PREFIXES = ("run_forklift", "scan", "scan_scene")

loop = asyncio.get_event_loop()
cancelled = 0
for task in asyncio.all_tasks(loop):
    if task.done():
        continue
    coro_name = task.get_coro().__qualname__ if hasattr(task.get_coro(), "__qualname__") else ""
    task_name = task.get_name()
    if any(p in coro_name or p in task_name for p in CANCEL_PREFIXES):
        task.cancel()
        cancelled += 1
        carb.log_warn(f"[cancel] Cancelled: {coro_name or task_name}")

carb.log_warn(f"[cancel] Done — cancelled {cancelled} task(s).")
