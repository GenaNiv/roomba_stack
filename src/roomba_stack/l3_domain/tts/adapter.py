from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Protocol

from roomba_stack.l0_core.events import TtsRequest

class TtsPort(Protocol):
    """
    Minimal port used by the rest of the system to perform text-to-speech.

    Program-to-interface:
    - Callers depend on this small surface, not a specific engine.
    - Concrete adapters can be swapped without changing domain code.
    """
    def speak(self, req: TtsRequest) -> None: ...

class TtsAdapter(ABC):
    """
    Base class for TTS adapters.

    Responsibility:
    - Convert a TtsRequest into speech (or enqueue it to do so).
    - Own any internal resources needed by a concrete backend.

    Threading:
    - speak(...) should return quickly. If the backend is slow,
      queue work internally rather than blocking the EventBus dispatcher.
    """

    @abstractmethod
    def speak(self, req: TtsRequest) -> None:
        """Perform (or enqueue) text-to-speech for the given request."""
        raise NotImplementedError
