from __future__ import annotations
from roomba_stack.l0_core.events import TtsRequest
from roomba_stack.l3_domain.tts.adapter import TtsAdapter

class PrintTtsAdapter(TtsAdapter):
    """
    Trivial TTS adapter that prints the text to stdout.
    Useful for development and tests where real audio is not needed.

    Responsibility:
    - Convert a TtsRequest into an immediate, visible side-effect.
    - Return quickly; no blocking.

    Usage:
    - Subscribe its 'on_event' method to the EventBus topic "voice.tts".
    """

    def speak(self, req: TtsRequest) -> None:  # fast, non-blocking
        print(f"[SAY] {req.text} (priority={req.priority})")

    # small EventBus-friendly callback to avoid lambda/layering churn
    def on_event(self, req: TtsRequest) -> None:
        self.speak(req)
