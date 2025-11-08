from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import json

try:
    import yaml  # optional; if unavailable, stick to .json
except Exception:  # pragma: no cover
    yaml = None  # type: ignore[attr-defined]


@dataclass(frozen=True, slots=True)
class VoiceAllowlist:
    """
    Immutable allowlist + thresholds for speaker authorization.

    Fields
    ------
    allowed_speakers : frozenset[str]
        Canonical, lowercase names permitted to control the robot.
    min_confidence : float
        Minimum confidence to accept a SpeakerIdentity event.
    auth_ttl_ms : int
        Authorization time-to-live in milliseconds.
    greet_cooldown_ms : int
        Min interval between greetings per speaker (milliseconds).
    """
    allowed_speakers: frozenset[str]
    min_confidence: float = 0.85
    auth_ttl_ms: int = 15_000
    greet_cooldown_ms: int = 30_000


def _to_names(xs: Iterable[str]) -> frozenset[str]:
    return frozenset(n.strip().lower() for n in xs if str(n).strip())


def load_allowlist(path: str | Path) -> VoiceAllowlist:
    """
    Load allowlist config from a YAML or JSON file.

    Supported shapes:
      YAML:
        allowed_speakers: [gena, alex]
        min_confidence: 0.85
        auth_ttl_ms: 15000
        greet_cooldown_ms: 30000

      JSON:
        {"allowed_speakers":["gena","alex"], "min_confidence":0.85, ...}

    Raises FileNotFoundError / ValueError on bad input.
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(p)

    text = p.read_text(encoding="utf-8")
    data: dict
    if p.suffix.lower() in {".yml", ".yaml"}:
        if yaml is None:
            raise ValueError("PyYAML not installed; use JSON or install pyyaml")
        data = yaml.safe_load(text) or {}
    else:
        data = json.loads(text or "{}")

    allowed = _to_names(data.get("allowed_speakers", []))
    if not allowed:
        raise ValueError("allowed_speakers must contain at least one name")

    return VoiceAllowlist(
        allowed_speakers=allowed,
        min_confidence=float(data.get("min_confidence", 0.85)),
        auth_ttl_ms=int(data.get("auth_ttl_ms", 15_000)),
        greet_cooldown_ms=int(data.get("greet_cooldown_ms", 30_000)),
    )
