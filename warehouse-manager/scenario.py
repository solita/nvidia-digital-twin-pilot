"""Scenario recording and replay for warehouse simulation runs."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path


@dataclass
class ScenarioEvent:
    t_offset_ms: int
    type: str  # "order"
    data: dict


@dataclass
class Scenario:
    scenario_id: str
    start_time: str  # ISO 8601
    events: list[ScenarioEvent] = field(default_factory=list)


class ScenarioRecorder:
    """Records order events during a simulation run."""

    def __init__(self, scenario_id: str):
        self.scenario_id = scenario_id
        self._start_ts: float | None = None
        self._events: list[ScenarioEvent] = []
        self._recording = False

    def start(self):
        self._start_ts = time.time()
        self._events.clear()
        self._recording = True

    def stop(self) -> Scenario:
        self._recording = False
        from datetime import datetime, timezone

        start_iso = datetime.fromtimestamp(self._start_ts or time.time(), tz=timezone.utc).isoformat()
        return Scenario(
            scenario_id=self.scenario_id,
            start_time=start_iso,
            events=list(self._events),
        )

    def record_order(self, order_data: dict):
        if not self._recording or self._start_ts is None:
            return
        offset = int((time.time() - self._start_ts) * 1000)
        self._events.append(ScenarioEvent(t_offset_ms=offset, type="order", data=order_data))

    @property
    def is_recording(self) -> bool:
        return self._recording


def save_scenario(scenario: Scenario, directory: str = "scenarios") -> Path:
    path = Path(directory)
    path.mkdir(parents=True, exist_ok=True)
    filepath = path / f"{scenario.scenario_id}.json"
    filepath.write_text(json.dumps(asdict(scenario), indent=2))
    return filepath


def load_scenario(scenario_id: str, directory: str = "scenarios") -> Scenario:
    filepath = Path(directory) / f"{scenario_id}.json"
    raw = json.loads(filepath.read_text())
    events = [ScenarioEvent(**e) for e in raw["events"]]
    return Scenario(scenario_id=raw["scenario_id"], start_time=raw["start_time"], events=events)
