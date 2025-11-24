from __future__ import annotations
"""
Voice HTTP Bridge
=================
This module defines a small boundary adapter that accepts tiny JSON messages
over HTTP (on localhost) from external voice services (speaker recognition,
keyword spotter, or full speech-to-text) and publishes **typed events** onto
the in-process EventBus.

Design intent:
- Keep all HTTP/transport concerns **outside** the core domain. Inside the app,
  we only deal with typed value objects (AudioTranscript, SpeakerIdentity).
- Protect the system at the trust boundary with explicit validation:
  - Require Content-Length and cap body size (prevents unbounded memory use).
  - Require JSON media type.
  - Validate topic and required fields per topic.
- Preserve responsiveness for multiple producers:
  - Use ThreadingHTTPServer so each request is handled in its own short-lived thread.
  - Publish to the EventBus with a short timeout; if the queue is saturated
    and the publish fails, return HTTP 503 so senders can back off or retry.

Accepted payloads:
  {"topic":"voice.speaker","ts":<ms>,"speaker":"gena","confidence":0.94}
  {"topic":"voice.transcript","ts":<ms>,"text":"roomba stop","confidence":0.91}
  {"topic":"voice.command","ts":<ms>,"word":"stop","confidence":0.99}  # coerced to transcript (source="kws")
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

from roomba_stack.l0_core import EventBus
from roomba_stack.l0_core.events import AudioTranscript, SpeakerIdentity, now_ms

# ---- module-level defaults (explicit contract and safe limits) ----
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
DEFAULT_MAX_BODY_BYTES = 16 * 1024  # 16 KiB is plenty for a single tiny JSON object


class VoiceHttpBridge:
    """
    VoiceHttpBridge
    ----------------
    Purpose:
        Accept tiny JSON posts from external voice processes and convert them
        into typed events published on the in-process EventBus.

    Responsibilities:
        • Run a small HTTP server bound to localhost.
        • Validate and normalize incoming requests (length, content type, schema).
        • Convert known topics to value objects:
            - "voice.speaker"    → SpeakerIdentity on topic "voice.speaker"
            - "voice.transcript" → AudioTranscript on topic "voice.transcript"
            - "voice.command"    → coerced to AudioTranscript(source="kws")
        • Publish events using a short timeout. If the EventBus queue is full,
          return HTTP 503 so producers can retry with backoff.

    Non-goals:
        • Do not leak HTTP concepts into the rest of the system.
        • Do not perform business decisions (authorization, routing, TTS, etc.).
        • Do not block producer threads; keep requests short and deterministic.

    Threading model:
        Each HTTP request is handled on a short-lived server thread created by
        ThreadingHTTPServer. Event delivery to subscribers is serialized by the
        EventBus’s single dispatcher thread (no listener data races).

    Parameters:
        eventbus:          The in-process EventBus to publish events into.
        host:              Interface to bind (default 127.0.0.1).
        port:              TCP port to listen on (default 8765).
        max_body_bytes:    Maximum allowed Content-Length in bytes for POST bodies.

    Lifecycle:
        start() → starts background server thread.
        stop()  → shuts down server and joins the thread.
    """

    def __init__(
        self,
        eventbus: EventBus,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        max_body_bytes: int = DEFAULT_MAX_BODY_BYTES,
    ) -> None:
        self._bus = eventbus
        self._host = host
        self._port = port
        self._max_body = max_body_bytes

        # Create a concurrent server so independent producers do not block each other.
        self._server = ThreadingHTTPServer((host, port), self._make_handler())
        # Attach the configured body cap for access by the handler.
        self._server._max_body = self._max_body  # type: ignore[attr-defined]

        # Run the server in a background daemon thread so start() is non-blocking.
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="VoiceHttpBridge", daemon=True
        )

    def start(self) -> None:
        """
        Start the HTTP server in a background thread.

        Effects:
            • Returns immediately; the server begins accepting POST requests.
        Usage:
            bridge = VoiceHttpBridge(eventbus)
            bridge.start()
        """
        self._thread.start()

    def stop(self) -> None:
        """
        Shutdown the HTTP server and join the background thread.

        Effects:
            • Stops accepting new requests and waits briefly for active ones to finish.
        Notes:
            • Safe to call during application shutdown.
        """
        self._server.shutdown()
        self._thread.join(timeout=1.0)

    # ---- internals (request handling) ----
    def _make_handler(self) -> type[BaseHTTPRequestHandler]:
        """
        Build and return a request handler class bound to this bridge’s configuration.

        The handler enforces:
            • Content-Length presence and maximum size (payload too large → 413).
            • JSON content type (unsupported media type → 415).
            • One JSON object per POST.
            • Known topics and required fields (missing → 422, unknown topic → 404).
            • Backpressure reporting: if EventBus publish returns False, respond 503.
            • On success: respond 202 Accepted (queued for asynchronous handling).

        Returns:
            A subclass of BaseHTTPRequestHandler ready for ThreadingHTTPServer.
        """
        bus = self._bus  # capture for closure

        class Handler(BaseHTTPRequestHandler):
            """Per-request handler bound to the enclosing bridge’s configuration."""

            # -------- small helpers for consistent responses --------
            def _bad(self, code: int = 400, msg: str | None = None) -> None:
                """
                Send an error response with an optional plain-text message.

                Parameters:
                    code: HTTP status code (4xx for client errors, 5xx for server saturation).
                    msg:  Optional human-readable description written to the response body.
                """
                self.send_response(code)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                if msg:
                    try:
                        self.wfile.write(msg.encode("utf-8"))
                    except Exception:
                        pass  # never raise from error path

            def _ok(self, code: int = 200, msg: str | None = None) -> None:
                """
                Send a success response with an optional plain-text message.

                Typical codes:
                    • 202 Accepted — payload validated and accepted into the queue.
                """
                self.send_response(code)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                if msg:
                    try:
                        self.wfile.write(msg.encode("utf-8"))
                    except Exception:
                        pass

            # ------------------- main POST entrypoint -------------------
            def do_POST(self) -> None:  # noqa: N802 (httpserver naming)
                """
                Process exactly one JSON object per request.

                Recognized topics:
                    • "voice.speaker":
                        { "topic":"voice.speaker", "ts":<ms>, "speaker":"gena", "confidence":0.94 }
                        → SpeakerIdentity published on "voice.speaker"
                    • "voice.transcript":
                        { "topic":"voice.transcript", "ts":<ms>, "text":"roomba stop", "confidence":0.91 }
                        → AudioTranscript published on "voice.transcript"
                    • "voice.command":
                        { "topic":"voice.command", "ts":<ms>, "word":"stop", "confidence":0.99 }
                        → coerced to AudioTranscript(source="kws") on "voice.transcript"

                Responses:
                    • 202 Accepted — event queued successfully.
                    • 404 Not Found — unknown topic.
                    • 411 Length Required — missing Content-Length header.
                    • 413 Payload Too Large — body exceeds configured cap.
                    • 415 Unsupported Media Type — Content-Type present but not JSON.
                    • 422 Unprocessable Entity — required field missing or empty.
                    • 503 Service Unavailable — EventBus queue saturated; try later.
                """
                # 1) Enforce Content-Length and a safe body cap.
                try:
                    n = int(self.headers.get("Content-Length", "0"))
                except Exception:
                    return self._bad(411, "length required")
                if n <= 0 or n > self.server._max_body:  # type: ignore[attr-defined]
                    return self._bad(413, "payload too large")

                # 2) Require JSON content type when provided (strict for clarity).
                content_type = (self.headers.get("Content-Type") or "").lower()
                if content_type and "json" not in content_type:
                    return self._bad(415, "unsupported media type")

                # 3) Read and parse the JSON body.
                try:
                    raw = self.rfile.read(n)
                    msg = json.loads(raw.decode("utf-8"))
                except Exception:
                    return self._bad(400, "invalid json")

                # 4) Validate topic and construct the appropriate value object.
                topic = str(msg.get("topic", "")).strip()
                ts = int(msg.get("ts", now_ms()))
                published = True  # track publish outcome to reflect backpressure

                if topic == "voice.speaker":
                    name = str(msg.get("speaker", "")).strip().lower()
                    if not name:
                        return self._bad(422, "missing speaker")
                    evt = SpeakerIdentity(
                        timestamp_millis=ts,
                        speaker=name,
                        confidence=_maybe_float(msg.get("confidence")),
                        source="gmm",
                    )
                    published = bus.publish("voice.speaker", evt)

                elif topic == "voice.transcript":
                    text = str(msg.get("text", "")).strip()
                    if not text:
                        return self._bad(422, "missing text")
                    
                    # honor caller-supplied source if present, else default to "stt"
                    src = str(msg.get("source", "stt")).strip().lower()
                    if src not in ("stt", "kws"):
                        src = "stt"

                    evt = AudioTranscript(
                        timestamp_millis=ts,
                        text=text,
                        confidence=_maybe_float(msg.get("confidence")),
                        source=src,              # was hard-coded "stt"
                    )
                    published = bus.publish("voice.transcript", evt)

                elif topic == "voice.command":
                    word = str(msg.get("word", "")).strip()
                    if not word:
                        return self._bad(422, "missing word")
                    evt = AudioTranscript(
                        timestamp_millis=ts,
                        text=word,
                        confidence=_maybe_float(msg.get("confidence")),
                        source="kws",
                    )
                    published = bus.publish("voice.transcript", evt)

                else:
                    return self._bad(404, "unknown topic")

                # 5) Reflect backpressure truthfully.
                if not published:
                    # EventBus publish timed out (queue saturated). Tell the client to retry later.
                    return self._bad(503, "bus saturated; try later")

                # 6) Accepted into the queue for asynchronous handling by the dispatcher.
                return self._ok(202, "accepted")

            # Keep the default server logs quiet; hook your logger if needed.
            def log_message(self, fmt: str, *args: Any) -> None:  # noqa: D401
                return

        return Handler


def _maybe_float(x: Any) -> float | None:
    """
    Try to convert a value to float; return None if not provided or not coercible.

    Parameters:
        x: Any value (string/number/None).

    Returns:
        float value or None.
    """
    try:
        return float(x) if x is not None else None
    except Exception:
        return None
