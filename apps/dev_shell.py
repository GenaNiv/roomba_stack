#!/usr/bin/env python3
"""
Developer Shell for Roomba Stack
--------------------------------
Minimal REPL that mirrors the production wiring (EventBus, CommandBus,
OIService). 
"""
from __future__ import annotations

import argparse
import shlex
import signal
import sys
from dataclasses import dataclass

from roomba_stack.l0_core import EventBus, CommandBus
from roomba_stack.l2_oi.oi_service import OIService, PID_ASCII_GENERIC, PID_ASCII_BATSTAT
from roomba_stack.l3_domain.state_store import RobotStateStore
from roomba_stack.l0_core.events import (
    AudioTranscript,
    SpeakerIdentity,
    StopCmd,
    DockCmd,
    ChangeModeCmd,
    ResetCmd,
    DriveCmd,
    DriveDirectCmd,
    now_ms,
)
from roomba_stack.l3_domain.voice.auth_policy import VoiceAuthPolicy, VoiceAuthConfig
from roomba_stack.l3_domain.voice.intent_router import IntentRouter, IntentThresholds
from roomba_stack.l3_domain.tts.print_adapter import PrintTtsAdapter


@dataclass(slots=True)
class ShellContext:
    eventbus: EventBus
    commandbus: CommandBus
    service: OIService

@dataclass(slots=True)
class VoiceStack:
    auth: VoiceAuthPolicy
    router: IntentRouter
    tts: PrintTtsAdapter

def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Roomba Dev Shell")
    parser.add_argument("--device", default="/dev/ttyUSB0", help="Serial device path")
    parser.add_argument("--baud", type=int, default=115_200, help="Baud rate")
    return parser.parse_args(argv)

def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    eventbus = EventBus()
    commandbus = CommandBus()
    service = OIService(device=args.device, eventbus=eventbus, baudrate=args.baud)
    service.open()
    service.set_on_sensor(_rx_logger)

    ctx = ShellContext(eventbus=eventbus, commandbus=commandbus, service=service)
    register_commands(ctx)
    voice = init_voice_stack(ctx)
    state_store = RobotStateStore(eventbus)

    def _cleanup(*_: object) -> None:
        shutdown(ctx, voice)
        sys.exit(0)

    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    print("Roomba Dev Shell ready. Type 'help' or 'quit'.")
    run_repl(ctx, voice, state_store)
    shutdown(ctx, voice)
    return 0


def register_commands(ctx: ShellContext) -> None:
    ctx.commandbus.register(StopCmd, ctx.service.handle_stop)
    ctx.commandbus.register(DockCmd, ctx.service.handle_dock)
    ctx.commandbus.register(ChangeModeCmd, ctx.service.handle_change_mode)
    ctx.commandbus.register(ResetCmd, ctx.service.handle_reset)
    ctx.commandbus.register(DriveCmd, ctx.service.handle_drive)
    ctx.commandbus.register(DriveDirectCmd, ctx.service.handle_drive_direct)


def run_repl(ctx: ShellContext, voice: VoiceStack, state_store: RobotStateStore) -> None:
    while True:
        try:
            raw = input("dev> ")
        except (EOFError, KeyboardInterrupt):
            break
        raw = raw.strip()
        if not raw:
            continue
        try:
            tokens = shlex.split(raw)
        except ValueError as exc:
            print(f"[error] {exc}")
            continue
        if not tokens:
            continue
        if not handle_command(tokens, ctx, state_store):
            break

def handle_command(tokens: list[str], ctx: ShellContext, state_store: RobotStateStore) -> bool:
    cmd = tokens[0].lower()
    args = tokens[1:]
    if cmd in ("quit", "exit"):
        return False
    if cmd in ("help", "?"):
        print(
            "Commands: help, stop, drive, drive_direct, dock, reset, start, safe, full, "
            "speaker, transcript, state, queues, quit"
        )
        return True
    if cmd == "stop":
        send_stop(ctx)
        return True
    if cmd == "drive":
        send_drive(ctx, args)
        return True
    if cmd == "drive_direct":
        send_drive_direct(ctx, args)
        return True
    if cmd in ("reset", "start", "safe", "full"):
        send_mode_command(ctx, cmd)
        return True
    if cmd == "state":
        show_state(state_store)
        return True
    if cmd == "queues":
        show_queues(ctx)
        return True
    if cmd == "speaker":
        send_speaker(ctx, args)
        return True
    if cmd == "transcript":
        send_transcript(ctx, args)
        return True
    if cmd == "dock":
        send_dock(ctx)
        return True
    print(f"Unknown command: {' '.join(tokens)}")
    return True

def send_stop(ctx: ShellContext) -> None:
    try:
        ok = ctx.commandbus.call(StopCmd(reason="dev_shell"))
    except Exception as exc:
        print(f"[error] Stop command failed: {exc}")
        return
    if ok:
        print("[ok] StopCmd dispatched")
    else:
        print("[warn] StopCmd handler rejected request")

def send_dock(ctx: ShellContext) -> None:
    try:
        ok = ctx.commandbus.call(DockCmd(reason="dev_shell"))
    except Exception as exc:
        print(f"[error] Dock command failed: {exc}")
        return
    if ok:
        print("[ok] DockCmd dispatched")
    else:
        print("[warn] DockCmd handler rejected request")
     
def send_drive(ctx: ShellContext, args: list[str]) -> None:
    usage = "usage: drive <vel> <radius> | drive --straight <vel> | drive --spin-left <vel> | drive --spin-right <vel>"
    try:
        vel, radius = _parse_drive_args(args)
    except ValueError as exc:
        print(f"[error] {exc}")
        print(usage)
        return
    try:
        ok = ctx.commandbus.call(DriveCmd(linear_mm_s=vel, angular_deg_s=radius))
    except Exception as exc:
        print(f"[error] Drive command failed: {exc}")
        return
    if ok:
        print(f"[ok] DriveCmd dispatched (vel={vel}, radius={radius})")
    else:
        print("[warn] Drive handler rejected request")

def _parse_drive_args(args: list[str]) -> tuple[int, int]:
    if not args:
        raise ValueError("missing drive arguments")
    if args[0] in ("--straight", "--spin-left", "--spin-right"):
        if len(args) != 2:
            raise ValueError("option form requires exactly one velocity argument")
        vel = _parse_int(args[1], "velocity")
        _validate_velocity(vel)
        if args[0] == "--straight":
            radius = -32768
        elif args[0] == "--spin-left":
            radius = 1
        else:
            radius = -1
        return vel, radius
    if len(args) != 2:
        raise ValueError("drive requires <velocity> <radius>")
    vel = _parse_int(args[0], "velocity")
    radius = _parse_int(args[1], "radius")
    _validate_velocity(vel)
    _validate_radius(radius)
    return vel, radius

def _parse_int(value: str, label: str) -> int:
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError(f"{label} must be an integer") from exc

def _validate_velocity(vel: int) -> None:
    if not (-500 <= vel <= 500):
        raise ValueError("velocity must be within -500..500 mm/s")

def _validate_radius(radius: int) -> None:
    if radius not in (-32768, 32767, -1, 1) and not (-2000 <= radius <= 2000):
        raise ValueError("radius must be in -2000..2000 or a special value {-32768, 32767, -1, 1}")

def send_drive_direct(ctx: ShellContext, args: list[str]) -> None:
    if len(args) not in (2, 3):
        print("usage: drive_direct <left_mm_s> <right_mm_s> [duration_ms]")
        return
    try:
        left = _parse_int(args[0], "left velocity")
        right = _parse_int(args[1], "right velocity")
        _validate_velocity(left)
        _validate_velocity(right)
        duration = _parse_int(args[2], "duration") if len(args) == 3 else None
    except ValueError as exc:
        print(f"[error] {exc}")
        return
    try:
        ok = ctx.commandbus.call(
            DriveDirectCmd(left_mm_s=left, right_mm_s=right, duration_ms=duration)
        )
    except Exception as exc:
        print(f"[error] DriveDirect command failed: {exc}")
        return
    if ok:
        print(f"[ok] DriveDirectCmd dispatched (left={left}, right={right}, duration={duration})")
    else:
        print("[warn] DriveDirect handler rejected request")

def send_mode_command(ctx: ShellContext, mode: str) -> None:
    key = mode.strip().lower()
    if key not in {"reset", "start", "safe", "full"}:
        print(f"[error] unsupported mode command: {mode}")
        return
    try:
        if key == "reset":
            ok = ctx.commandbus.call(ResetCmd(reason="dev_shell"))
        else:
            ok = ctx.commandbus.call(ChangeModeCmd(mode=key))
    except Exception as exc:
        print(f"[error] {mode} command failed: {exc}")
        return
    if ok:
        print(f"[ok] {mode.upper()} command sent")
    else:
        print(f"[warn] {mode} handler rejected request")

def show_state(store: RobotStateStore) -> None:
    snap = store.get()
    print(
        "RobotSnapshot("
        f"mode={snap.mode}, "
        f"voltage_mv={snap.voltage_mv}, "
        f"current_ma={snap.current_ma}, "
        f"charge_state={snap.charge_state}, "
        f"left_ticks={snap.left_encoder_ticks}, "
        f"right_ticks={snap.right_encoder_ticks}, "
        f"last_update_ms={snap.last_update_millis})"
    )


def show_queues(ctx: ShellContext) -> None:
    entries = [
        ("EventBus", getattr(ctx.eventbus, "_queue", None)),
        ("RxByteQueue", getattr(ctx.service, "_rx_bytes", None)),
        ("TxFrameQueue", getattr(ctx.service, "_tx_frames", None)),
    ]
    for name, queue in entries:
        if queue is None:
            print(f"{name}: unavailable")
            continue
        try:
            size = queue.qsize()
            maxsize = queue.maxsize()
        except Exception as exc:
            print(f"{name}: error reading size ({exc})")
            continue
        print(f"{name}: {size}/{maxsize}")

def send_speaker(ctx: ShellContext, args: list[str]) -> None:
    if not args:
        print("usage: speaker <name> [confidence]")
        return
    name = args[0].strip().lower()
    if not name:
        print("[error] speaker name must not be empty")
        return
    confidence = 0.0
    if len(args) >= 2:
        try:
            confidence = float(args[1])
        except ValueError:
            print("[error] confidence must be a float between 0 and 1")
            return
    evt = SpeakerIdentity(
        timestamp_millis=now_ms(),
        speaker=name,
        confidence=confidence,
        source="dev_shell",
    )
    ok = ctx.eventbus.publish("voice.speaker", evt)
    if not ok:
        print("[warn] EventBus saturated; speaker event dropped")
    else:
        print(f"[ok] SpeakerIdentity published ({name}, conf={confidence})")

def send_transcript(ctx: ShellContext, args: list[str]) -> None:
    if not args:
        print('usage: transcript "<text>" [confidence] [source]')
        return
    text = args[0]
    confidence = None
    source = "dev_shell"
    if len(args) >= 2:
        try:
            confidence = float(args[1])
        except ValueError:
            print("[error] confidence must be numeric")
            return
    if len(args) >= 3:
        source = args[2].strip().lower() or "dev_shell"
    evt = AudioTranscript(
        timestamp_millis=now_ms(),
        text=text,
        confidence=confidence,
        source=source,
    )
    ok = ctx.eventbus.publish("voice.transcript", evt)
    if not ok:
        print("[warn] EventBus saturated; transcript event dropped")
    else:
        print(f"[ok] AudioTranscript published ({text!r}, conf={confidence}, source={source})")

def init_voice_stack(ctx: ShellContext) -> VoiceStack:
    def _emit_tts(req):
        ctx.eventbus.publish("voice.tts", req)

    tts = PrintTtsAdapter()
    ctx.eventbus.subscribe("voice.tts", tts.on_event)

    cfg = VoiceAuthConfig(
        allowed_speakers=frozenset({"gena"}),
        auth_ttl_ms=15_000,
        min_confidence=0.85,
        greet_cooldown_ms=30_000,
    )

    auth = VoiceAuthPolicy(config=cfg, emit_tts=_emit_tts)

    ctx.eventbus.subscribe("voice.speaker", auth.on_speaker_identified)

    thresholds = IntentThresholds(execute_min=0.80, confirm_min=0.50, confirm_window_ms=3000)
    router = IntentRouter(policy=auth, commandbus=ctx.commandbus, thresholds=thresholds, emit_tts=_emit_tts)

    ctx.eventbus.subscribe("voice.transcript", router.on_transcript)
    return VoiceStack(auth=auth, router=router, tts=tts)

def shutdown(ctx: ShellContext, voice: VoiceStack | None = None) -> None:
    try:
        ctx.service.close()
    finally:
        ctx.eventbus.close()

def _rx_logger(packet_id: int, parsed: object) -> None:
    if packet_id == PID_ASCII_BATSTAT:
        try:
            mode_n = parsed.get("mode")
            print(
                "[ASCII.BAT] "
                f"min={parsed['min']} sec={parsed['sec']} "
                f"mv={parsed['mv']} ma={parsed['ma']} "
                f"rx={parsed['rx']} mah={parsed['mah']} "
                f"state={parsed['state']} mode={mode_n}"
            )
        except Exception:
            print(f"[ASCII.BAT] {parsed}")
    elif packet_id == PID_ASCII_GENERIC:
        print(f"[ASCII.TEXT] {parsed}")
    else:
        print(f"[RX] Packet {packet_id}: {parsed}")


if __name__ == "__main__":
    raise SystemExit(main())

