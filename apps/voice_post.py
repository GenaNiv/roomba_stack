#!/usr/bin/env python3
"""
voice_post.py — tiny client to send KWS/STT (and speaker) events to VoiceHttpBridge.

Examples:
  # keyword spotter hit (maps to AudioTranscript on topic voice.transcript)
  ./apps/voice_post.py --url http://127.0.0.1:8765/ \
      --transcript "stop" --confidence 0.97 --source kws

  # STT transcript
  ./apps/voice_post.py --transcript "turn left" --confidence 0.88 --source stt

  # speaker identity (your GMM service result)
  ./apps/voice_post.py --speaker "gena" --confidence 0.94
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from urllib import request

DEFAULT_URL = "http://127.0.0.1:8765/"

def _now_ms() -> int:
    return int(time.time() * 1000)

def post_json(url: str, payload: dict) -> None:
    data = json.dumps(payload).encode("utf-8")
    req = request.Request(url, data=data, headers={"Content-Type": "application/json"})
    with request.urlopen(req, timeout=5) as resp:
        body = (resp.read() or b"").decode("utf-8", "replace")
        print(f"[HTTP {resp.status}] {body or 'ok'}")

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Send KWS/STT or speaker events to VoiceHttpBridge")
    ap.add_argument("--url", default=DEFAULT_URL, help=f"Bridge URL (default: {DEFAULT_URL})")
    ap.add_argument("--ts", type=int, default=_now_ms(), help="Timestamp (ms since epoch); default=now")
    ap.add_argument("--confidence", type=float, default=None, help="Confidence score (0..1)")

    # one of these:
    ap.add_argument("--transcript", help="Text from STT/KWS (posted to topic voice.transcript)")
    ap.add_argument("--speaker", help="Speaker identity (posted to topic voice.speaker)")

    # optional source tag for transcript posts
    ap.add_argument("--source", default=None, help="Origin tag for transcript: e.g., 'kws' or 'stt'")

    args = ap.parse_args(argv)

    if args.transcript and args.speaker:
        ap.error("Use either --transcript or --speaker (not both).")
    if not args.transcript and not args.speaker:
        ap.error("Provide one of --transcript or --speaker.")

    if args.transcript:
        payload = {
            "topic": "voice.transcript",
            "ts": int(args.ts),
            "text": args.transcript,
        }
        if args.confidence is not None:
            payload["confidence"] = float(args.confidence)
        if args.source:
            payload["source"] = str(args.source)
        post_json(args.url, payload)
        return 0

    # speaker
    payload = {
        "topic": "voice.speaker",
        "ts": int(args.ts),
        "speaker": args.speaker,
    }
    if args.confidence is not None:
        payload["confidence"] = float(args.confidence)
    post_json(args.url, payload)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
