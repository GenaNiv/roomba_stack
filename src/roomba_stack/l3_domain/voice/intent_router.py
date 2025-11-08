from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Callable

from roomba_stack.l0_core.events import AudioTranscript, TtsRequest, now_ms
from roomba_stack.l0_core import CommandBus
from roomba_stack.l3_domain.voice.auth_policy import VoiceAuthPolicy
from roomba_stack.l0_core.events import StopCmd  # MVP intent

@dataclass(frozen=True, slots=True)
class IntentThresholds:
    """Confidence thresholds for routing decisions."""
    execute_min: float = 0.80   # >= execute
    confirm_min: float = 0.50   # [confirm_min, execute_min) => ask confirm
    confirm_window_ms: int = 3000  # time to accept yes/no

class IntentRouter:
    """
    Purpose:
        Convert AudioTranscript events into either commands (via CommandBus)
        or spoken prompts (via TtsRequest), using VoiceAuthPolicy for gating.

    Inputs:
        - AudioTranscript on topic "voice.transcript".

    Collaborators:
        - VoiceAuthPolicy: is_authorized/current_speaker.
        - CommandBus: for StopCmd (MVP).
        - emit_tts: injected callable that will publish TtsRequest to EventBus.

    Threading:
        Intended to be called from the EventBus dispatcher thread.
    """

    def __init__(self,
                 policy: VoiceAuthPolicy,
                 commandbus: CommandBus,
                 thresholds: IntentThresholds,
                 emit_tts: Callable[[TtsRequest], None]) -> None:
        self._policy = policy
        self._cmd = commandbus
        self._th = thresholds
        self._emit_tts = emit_tts
        # pending confirmation state
        self._pending_intent: Optional[str] = None
        self._confirm_expires_ms: int = 0

        # EventBus subscriber
    def on_transcript(self, evt: AudioTranscript) -> None:
        """
        Routing with confirmation:
        - Require an authorized current speaker.
        - If text matches 'stop' and confidence >= execute_min -> execute StopCmd.
        - If 'stop' and confirm_min <= confidence < execute_min -> ask to confirm.
        - If we are waiting for confirmation -> accept 'yes' / 'no' within the window
            measured against the *event* timestamp (more robust than wall-clock).
        - Otherwise -> ask to repeat.
        """
        # 0) authorization gate
        name = self._policy.current_speaker()
        if not (name and self._policy.is_authorized(name)):
            self._emit_tts(TtsRequest(now_ms(), "Sorry, I can't take commands from you.", priority=6))
            return

        text = (evt.text or "").strip().lower()
        conf = evt.confidence if evt.confidence is not None else 0.0

        # Use the event's timestamp when available; fall back to local clock
        ts = evt.timestamp_millis if getattr(evt, "timestamp_millis", None) else now_ms()

        # 1) handle pending confirmation first (compare using event time)
        if self._pending_intent and ts <= self._confirm_expires_ms:
            if text in {"yes", "yeah", "yep", "confirm"}:
                if self._pending_intent == "stop":
                    try:
                        self._cmd.call(StopCmd(reason="voice"))
                        self._emit_tts(TtsRequest(ts, "OK, stopping.", priority=9))
                    except Exception:
                        self._emit_tts(TtsRequest(ts, "Stop failed.", priority=9))
                self._pending_intent = None
                self._confirm_expires_ms = 0
                return
            elif text in {"no", "nope", "cancel"}:
                self._emit_tts(TtsRequest(ts, "Okay, canceled.", priority=4))
                self._pending_intent = None
                self._confirm_expires_ms = 0
                return
            # not yes/no → fall through to generic prompt without clearing window

        # 2) simple intent match for 'stop'
        stop_words = {"stop", "halt", "freeze"}
        if text in stop_words:
            if conf >= self._th.execute_min:
                try:
                    self._cmd.call(StopCmd(reason="voice"))
                    self._emit_tts(TtsRequest(ts, "OK, stopping.", priority=9))
                except Exception:
                    self._emit_tts(TtsRequest(ts, "Stop failed.", priority=9))
                return
            elif conf >= self._th.confirm_min:
                # start confirm window using event time
                self._pending_intent = "stop"
                self._confirm_expires_ms = ts + self._th.confirm_window_ms
                self._emit_tts(TtsRequest(ts, "Did you say stop? Please say yes or no.", priority=6))
                return

        # 3) if we had a pending but it expired, give a clearer message
        if self._pending_intent and ts > self._confirm_expires_ms:
            self._pending_intent = None
            self._confirm_expires_ms = 0
            self._emit_tts(TtsRequest(ts, "Confirmation timed out. Please repeat the command.", priority=4))
            return

        # 4) fallback
        self._emit_tts(TtsRequest(ts, "I did not catch that. Please repeat.", priority=3))


