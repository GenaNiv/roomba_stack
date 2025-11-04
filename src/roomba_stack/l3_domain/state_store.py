from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from roomba_stack.l0_core import EventBus
from roomba_stack.l0_core.events import SensorUpdate


@dataclass(frozen=True, slots=True)
class RobotSnapshot:
    """Immutable snapshot of the latest known robot state."""
    mode: int | None = None
    voltage_mv: int | None = None
    current_ma: int | None = None
    charge_state: str | None = None
    left_encoder_ticks: int | None = None
    right_encoder_ticks: int | None = None
    last_update_millis: int | None = None



class RobotStateStore:
    """
    Subscribes to 'sensors' events and maintains the latest RobotSnapshot.
    Runs entirely on the Event Bus dispatcher thread, so no locks are needed.
    """

    def __init__(self, eventbus: EventBus) -> None:
        self._bus = eventbus
        self._snapshot: RobotSnapshot = RobotSnapshot()
        self._bus.subscribe("sensors", self._on_sensor_update)

    # ---- subscriber callback (runs on Event Bus dispatcher thread) ----
    def _on_sensor_update(self, evt: SensorUpdate) -> None:
        fields: Mapping[str, object] = evt.fields

        left_ticks = self._pick_first_int(fields, ["encoder_counts_left", "left_encoder_ticks", "left_encoder"])
        right_ticks = self._pick_first_int(fields, ["encoder_counts_right", "right_encoder_ticks", "right_encoder"])

        self._snapshot = RobotSnapshot(
            mode=self._pick_int(fields, "oi_mode", self._snapshot.mode),
            voltage_mv=self._pick_int(fields, "voltage_mv", self._snapshot.voltage_mv),
            current_ma=self._pick_int(fields, "current_ma", self._snapshot.current_ma),
            charge_state=self._pick_str(fields, "charge_state", self._snapshot.charge_state),
            left_encoder_ticks=left_ticks if left_ticks is not None else self._snapshot.left_encoder_ticks,
            right_encoder_ticks=right_ticks if right_ticks is not None else self._snapshot.right_encoder_ticks,
            last_update_millis=evt.timestamp_millis,
        )
        
    @staticmethod
    def _pick_first_int(m: Mapping[str, object], keys: list[str]) -> int | None:
        for k in keys:
            v = m.get(k)
            if isinstance(v, (int, bool)):
                return int(v)
        return None

    # ---- read API for other modules (returns immutable value object) ----
    def get(self) -> RobotSnapshot:
        return self._snapshot

    # ---- tiny helpers (avoid KeyError and type noise) ----
    @staticmethod
    def _pick_int(m: Mapping[str, object], k: str, default: int | None) -> int | None:
        v = m.get(k, default)
        return int(v) if isinstance(v, (int, bool)) else default

    @staticmethod
    def _pick_str(m: Mapping[str, object], k: str, default: str | None) -> str | None:
        v = m.get(k, default)
        return str(v) if isinstance(v, str) else default
