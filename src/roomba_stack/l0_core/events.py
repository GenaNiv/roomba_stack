from __future__ import annotations

from enum import Enum
import time
from dataclasses import dataclass
from typing import Mapping, Union
from dataclasses import dataclass

Number = Union[int, float]
Value = Union[Number, bool, str]


class Severity(str, Enum):
    INFO = "info"
    WARN = "warn"
    ERROR = "error"
    CRITICAL = "critical"
    
def now_ms() -> int:
    """Gives a steady (monotonic) clock for event timing in milliseconds 
        for timestamps in logs/events (steady, not wall-clock).
    """
    return time.monotonic_ns() // 1_000_000

@dataclass(frozen=True, slots=True)
class SensorUpdate:
    """
    Immutable sensor event produced by the RX/decoder path.

    Fields:
      - timestamp_millis: monotonic timestamp in milliseconds (use now_ms())
      - packet_id: Open Interface packet id if known; None for ASCII/other sources
      - source: e.g., "oi", "ascii", "sim"
      - fields: parsed key→value pairs (typed scalars only for simplicity)
    """
    timestamp_millis: int
    packet_id: int | None
    source: str
    fields: Mapping[str, Value]
    
class TaskState(str, Enum):
    STARTED = "started"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True, slots=True)
class TaskStatus:
    """
    Immutable notification about the lifecycle of a task/job.
    Examples: "charging_cycle", "firmware_probe", "voice_capture"
    """
    timestamp_millis: int
    task_name: str
    state: TaskState
    details: Mapping[str, Value]  # small, scalar-only payload


@dataclass(frozen=True, slots=True)
class Fault:
    """
    Immutable fault record used for reporting errors with a severity level.
    Example:
      Fault(timestamp_millis=..., severity=Severity.ERROR, code="SERIAL_WRITE_TIMEOUT",
            message="serial write exceeded 50ms", context={"attempts": 2, "bytes": 24})
    """
    timestamp_millis: int
    severity: Severity
    code: str               # stable programmatic code (e.g., "SERIAL_WRITE_TIMEOUT")
    message: str            # human-readable explanation
    context: Mapping[str, Value]  # small scalar extras (never mutate)
    
@dataclass(frozen=True, slots=True)
class DriveCmd:
    """
    Request to drive the robot.
    - linear_mm_s: forward/backward speed (+forward, -backward)
    - angular_deg_s: rotation speed (+CCW, -CW)
    - duration_ms: optional max duration; handler may clamp/ignore if 0 or None
    """
    linear_mm_s: int
    angular_deg_s: int
    duration_ms: int | None = None


@dataclass(frozen=True, slots=True)
class StopCmd:
    """
    Request to stop motion (and optionally provide a reason for logging).
    """
    reason: str = "user"
    