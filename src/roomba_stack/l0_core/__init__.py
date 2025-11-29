"""
roomba_stack.l0_core
Foundational Core layer (contracts & buses) for the Roomba stack.

Public API (to be implemented in subsequent steps):
- now_ms, Severity, SensorUpdate, TaskStatus, AudioTranscript, Fault
- DriveCmd, StopCmd
- EventBus, CommandBus
"""

# Re-exports (will resolve after events.py and bus.py are added)
from .events import (  # noqa: F401
    now_ms, Severity,
    SensorUpdate, TaskStatus, AudioTranscript, Fault,
    DriveCmd, DriveDirectCmd, StopCmd, DockCmd, ChangeModeCmd, ResetCmd,
)
from .bus import EventBus, CommandBus  # noqa: F401

__all__ = [
    "now_ms", "Severity",
    "SensorUpdate", "TaskStatus", "AudioTranscript", "Fault",
    "DriveCmd", "DriveDirectCmd", "StopCmd", "DockCmd", "ChangeModeCmd", "ResetCmd",
    "EventBus", "CommandBus",
]
