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
class DriveDirectCmd:
    """
    Request to drive by direct wheel speeds (mm/s).
    Positive = forward; negative = backward.
    """
    left_mm_s: int
    right_mm_s: int
    duration_ms: int | None = None
    
@dataclass(frozen=True, slots=True)
class StopCmd:
    """
    Request to stop motion (and optionally provide a reason for logging).
    """
    reason: str = "user"
    
@dataclass(frozen=True, slots=True)
class AudioTranscript:
    timestamp_millis: int
    text: str
    confidence: float | None = None
    source: str | None = None
    
@dataclass(frozen=True, slots=True)
class SpeakerIdentity:
    """
    Who spoke (from a speaker-ID service).

    Fields:
      timestamp_millis: Wall-clock ms when the utterance was observed.
      speaker: Canonical, lowercase speaker name (e.g., "gena").
      confidence: Optional confidence score from the recognizer (0.0–1.0).
      source: Optional source tag (e.g., "gmm").
    """
    timestamp_millis: int
    speaker: str
    confidence: float | None = None
    source: str | None = None

@dataclass(frozen=True, slots=True)
class TtsRequest:
    """
    Text-to-Speech (TTS) request published on the EventBus for a TTS adapter to speak.

    Purpose
    -------
    Represent a human-facing spoken message as an immutable value object so any part
    of the system (policy, router, safety checks, CLI) can request audible feedback
    without knowing how audio is produced.

    Fields
    ------
    timestamp_millis : int
        Wall-clock time in milliseconds when the request was created (use now_ms()).
        Useful for ordering, telemetry, and time-based coalescing.
    text : str
        The exact phrase to speak. Keep short and self-contained. Do not include SSML
        unless your TTS adapter explicitly supports it.
    priority : int, default 0
        Higher numbers indicate higher urgency. Adapters may use this to reorder,
        preempt, or drop lower-priority messages under load. The semantics are
        adapter-defined; the contract is that larger means “more important.”

    Threading and Delivery
    ----------------------
    • Created and published from any thread.
    • Delivered to subscribers on the EventBus dispatcher thread (single thread),
      ensuring no listener races.
    • Adapters should avoid blocking the dispatcher; offload audio playback to their
      own worker thread or process.

    Typical Topics
    --------------
    Publish on a dedicated topic such as "voice.tts" (convention), e.g.:
        eventbus.publish("voice.tts", TtsRequest(now_ms(), "Hello Gena, how can I help?", 5))

    Usage Examples
    --------------
    • Greeting a newly authorized speaker:
        TtsRequest(now_ms(), f"Hello {name}, how can I help?", priority=5)
    • Error / clarification:
        TtsRequest(now_ms(), "I did not catch that. Please repeat.", priority=3)
    • Safety notice:
        TtsRequest(now_ms(), "Stopping now.", priority=9)

    Notes
    -----
    This class expresses *what to say*, not *how to say it*. Voice selection,
    playback device, and rate are the responsibility of the TTS adapter.
    """
    timestamp_millis: int
    text: str
    priority: int = 0