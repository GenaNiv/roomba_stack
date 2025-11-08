from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable
from typing import Callable, Optional

from roomba_stack.l0_core.events import SpeakerIdentity, TtsRequest, now_ms

@dataclass(frozen=True, slots=True)
class VoiceAuthConfig:
    """
    Immutable configuration for voice authorization.

    Fields
    ------
    allowed_speakers : frozenset[str]
        Canonical, lowercase names that are permitted to control the robot.
    auth_ttl_ms : int
        How long (milliseconds) a positive identification keeps a speaker authorized.
        Defaults to 15 seconds.
    """
    allowed_speakers: frozenset[str]
    auth_ttl_ms: int = 15_000
    min_confidence: float = 0.85        
    greet_cooldown_ms: int = 30_000    

class VoiceAuthPolicy:
    """
    Maintain a short-lived 'authorized speaker' window based on SpeakerIdentity events.

    Responsibilities
    ---------------
    • On allowed speaker identification, mark them authorized until now + ttl.
    • Expose queries for current authorization.
    • (Soon) Emit a greeting TtsRequest on first authorization.

    Threading
    ---------
    Designed to be called from the EventBus dispatcher thread for mutations.
    Reads from other threads should use snapshot-style accessors.
    """

    def __init__(self, config: VoiceAuthConfig,
                 emit_tts: Optional[Callable[[TtsRequest], None]] = None) -> None:
        self._cfg = config
        self._emit_tts = emit_tts  # dependency-injected side-effect for speech
        self._authorized_until_ms: dict[str, int] = {}
        self._greeted_until_ms: dict[str, int] = {}
        self._current_speaker: str | None = None

        # ---- event input (subscribe this to "voice.speaker") ----
    def on_speaker_identified(self, evt: SpeakerIdentity) -> None:
        """
        Apply allowlist and confidence filter, then extend the speaker's
        authorization window using a sliding expiry. Also mark them as the
        most-recent 'current' speaker.

        This method is intended to be called from the EventBus dispatcher thread.
        """
        # 1) Allowlist gate
        name = evt.speaker.strip().lower()
        if name not in self._cfg.allowed_speakers:
            return

        # 2) Confidence gate
        conf = evt.confidence if evt.confidence is not None else 0.0
        if conf < self._cfg.min_confidence:
            return

        # 3) Sliding window: now + ttl
        now = now_ms()
        self._authorized_until_ms[name] = now + self._cfg.auth_ttl_ms

        # 4) Keep notion of most-recent authorized speaker
        self._current_speaker = name
        
        self._maybe_greet(name, now)

        # 5) Optional light pruning to keep the map tidy
        self.prune_expired()


    # ---- queries ----
    def is_authorized(self, speaker: str) -> bool:
        """Return True if 'speaker' is currently within the authorization window."""
        exp = self._authorized_until_ms.get(speaker)
        return bool(exp is not None and exp >= now_ms())

    def authorized_speakers(self) -> list[str]:
        """Return a list of currently authorized speakers (non-expired)."""
        t = now_ms()
        return [s for s, exp in self._authorized_until_ms.items() if exp >= t]

    # ---- maintenance ----
    def prune_expired(self) -> None:
        """Remove expired authorizations; idempotent and O(n) in number of speakers."""
        t = now_ms()
        expired = [s for s, exp in self._authorized_until_ms.items() if exp < t]
        for s in expired:
            del self._authorized_until_ms[s]
            
    def _maybe_greet(self, name: str, now: int) -> None:
        """
        If allowed by cooldown and an emitter is set, publish a greeting TtsRequest
        and move the per-speaker 'greet again after' window forward.
        """
        if self._emit_tts is None:
            return
        next_ok = self._greeted_until_ms.get(name, 0)
        if now < next_ok:
            return  # still within cooldown; skip greeting
        # Compose the greeting (can be customized later)
        req = TtsRequest(timestamp_millis=now, text=f"Hello {name}, how can I help?", priority=5)
        try:
            self._emit_tts(req)
        finally:
            self._greeted_until_ms[name] = now + self._cfg.greet_cooldown_ms


    def current_speaker(self) -> str | None:
        """
        Return the most-recent speaker who passed allowlist + confidence
        and is still within the authorization window; otherwise None.

        Design notes:
        - Single-writer model: this is read by other threads, but updates happen
        on the EventBus dispatcher thread, which keeps invariants simple.
        - We only report a name if their auth has not expired; otherwise None.
        """
        name = self._current_speaker
        if not name:
            return None
        exp = self._authorized_until_ms.get(name)
        return name if exp is not None and exp >= now_ms() else None
