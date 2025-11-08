#!/usr/bin/env python3
from __future__ import annotations

import signal
import sys
import time
from typing import Callable

from roomba_stack.l0_core import EventBus, CommandBus
from roomba_stack.l0_core.events import (
    TtsRequest,
    SpeakerIdentity,   # only used in type hints/printing
    AudioTranscript,   # only used in type hints/printing
    StopCmd,
    now_ms,
)
from roomba_stack.l3_domain.bridges.voice_http_bridge import VoiceHttpBridge
from roomba_stack.l3_domain.voice.auth_policy import VoiceAuthPolicy, VoiceAuthConfig
from roomba_stack.l3_domain.voice.intent_router import IntentRouter, IntentThresholds


# --- tiny helper to pretty-print any events on a topic (debug/observability) ---
def _print_topic(topic: str) -> Callable[[object], None]:
    def _cb(evt: object) -> None:
        print(f"[EVENT {topic}] {evt}")
    return _cb


def main() -> int:
    """
    Voice Gateway runner.

    What this process does
    ----------------------
    1) Creates a single-process EventBus (bounded queue + one dispatcher thread).
    2) Exposes a small HTTP endpoint (VoiceHttpBridge) on localhost that accepts
       JSON posts from external voice services (speaker-ID, STT/KWS) and publishes
       typed events into the EventBus.
    3) Maintains a short-lived "authorized speaker" window via VoiceAuthPolicy
       (allowlist + TTL + greeting-on-first-authorization).
    4) Routes transcripts to intents with IntentRouter (MVP: 'stop'), which either
       issues a StopCmd via CommandBus or requests speech via a TtsRequest event.
    5) Prints TTS requests so you can see feedback without a real TTS adapter.

    Threading model
    ---------------
    - EventBus has ONE dispatcher thread; all subscribers run there serially.
      Keep subscriber handlers O(1) and non-blocking.
    - HTTP bridge uses the stdlib server thread to accept requests; it publishes
      small immutable events to the EventBus with a short enqueue timeout.

    Safety/backpressure
    -------------------
    - EventBus queue is bounded; publish() returns False if full (drop-newest).
    - We never block the dispatcher with long work; heavy work should be offloaded
      to workers and report back via new events (future).
    """
    # 1) Core messaging backbone -------------------------------------------------
    #    Bounded queue prevents unbounded memory growth; single dispatcher thread
    #    serializes all callbacks → easy reasoning, no subscriber-side locks.
    bus = EventBus(capacity=1024, publish_timeout_ms=10)

    # (Optional) print raw inbound voice events for visibility
    bus.subscribe("voice.speaker", _print_topic("voice.speaker"))
    bus.subscribe("voice.transcript", _print_topic("voice.transcript"))

    # 2) TTS sink (for now we just print; a real TTS adapter will subscribe here)
    def _emit_tts(req: TtsRequest) -> None:
        # Any component can call this to "speak"; we publish to a well-known topic.
        bus.publish("voice.tts", req)

    def _on_tts(req: TtsRequest) -> None:
        # Human-facing feedback (what the robot would say)
        print(f"[TTS] {req.text} (priority={req.priority})")

    bus.subscribe("voice.tts", _on_tts)

    # 3) Speaker authorization policy -------------------------------------------
    #    Encapsulates: allowlist, confidence threshold, sliding TTL window,
    #    and a greeting with cooldown when someone first becomes authorized.
    cfg = VoiceAuthConfig(
        allowed_speakers=frozenset({"gena"}),  # <-- edit to your allowlist
        auth_ttl_ms=15_000,                    # authorized for 15s after a good ID
        min_confidence=0.85,                   # ignore IDs below this confidence
        greet_cooldown_ms=30_000,              # avoid greeting spam
    )
    auth = VoiceAuthPolicy(config=cfg, emit_tts=_emit_tts)

    # Subscribe the policy to identity events produced by the HTTP bridge
    bus.subscribe("voice.speaker", auth.on_speaker_identified)

    # 4) Command bus + minimal handler(s) ---------------------------------------
    #    CommandBus is synchronous: it routes a command object to its handler.
    cmd_bus = CommandBus()

    # MVP: Stop command handler (replace print with real call into OIService later)
    cmd_bus.register(StopCmd, lambda cmd: print("[CMD] StopCmd(reason=voice)"))

    # 5) Intent router (MVP: only 'stop') ---------------------------------------
    #    Converts transcripts into either a command or a TTS prompt.
    th = IntentThresholds(
        execute_min=0.80,       # >= execute immediately
        confirm_min=0.50,       # [confirm_min, execute_min) would ask to confirm (future step)
        confirm_window_ms=3000  # window for yes/no (not used yet)
    )
    router = IntentRouter(policy=auth, commandbus=cmd_bus, thresholds=th, emit_tts=_emit_tts)
    bus.subscribe("voice.transcript", router.on_transcript)

    # 6) HTTP ingress (voice services → JSON → typed events) --------------------
    bridge = VoiceHttpBridge(eventbus=bus, host="127.0.0.1", port=8765)
    bridge.start()
    print("[gateway] VoiceHttpBridge on http://127.0.0.1:8765  (Ctrl+C to exit)")
    print("Try POSTs like:")
    print('  curl -i -H "Content-Type: application/json" \\')
    print('    -d \'{"topic":"voice.speaker","ts":1730900000000,"speaker":"gena","confidence":0.94}\' \\')
    print('    http://127.0.0.1:8765/')
    print('  curl -i -H "Content-Type: application/json" \\')
    print('    -d \'{"topic":"voice.transcript","ts":1730900000500,"text":"stop","confidence":0.92}\' \\')
    print('    http://127.0.0.1:8765/')

    # 7) Graceful shutdown -------------------------------------------------------
    def _on_sigint(sig, frame):
        print("\n[gateway] shutting down...")
        try:
            bridge.stop()
        finally:
            bus.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, _on_sigint)

    # Keep the process alive; event handling happens on the dispatcher thread.
    while True:
        time.sleep(1)


if __name__ == "__main__":
    sys.exit(main())
