#!/usr/bin/env python3
"""
Roomba CLI Tool
===============

Interactive CLI for controlling a Roomba through the Open Interface (OI).

Uses:
- L2 OIService: wraps PySerialPort, oi_codec, and oi_protocol.
- Provides high-level convenience commands (start, safe, dock, sensors, etc.).

Features:
- Logs all received data (HEX + ASCII).
- Interactive prompt for sending common OI commands.
- Supports RTS/BRC pin management (wakeup, rts_low, rts_high).
"""

import argparse
import time
from roomba_stack.l2_oi.oi_service import OIService
from roomba_stack.l2_oi.oi_service import PID_ASCII_GENERIC, PID_ASCII_BATSTAT

MODE_NAMES = {0:"Off", 1:"Passive", 2:"Safe", 3:"Full"}

def rx_logger(packet_id: int, parsed: object) -> None:
    """
    RX callback: logs parsed sensor updates and ASCII events.
    """
    if packet_id == PID_ASCII_BATSTAT:
        # parsed is a dict with keys: min, sec, mv, ma, rx, mah, state, mode
        try:
            mode_n = parsed.get("mode")
            mode_s = MODE_NAMES.get(mode_n, str(mode_n))
            print("[ASCII.BAT] "
                  f"min={parsed['min']} sec={parsed['sec']} "
                  f"mv={parsed['mv']} ma={parsed['ma']} "
                  f"rx={parsed['rx']} mah={parsed['mah']} "
                  f"state={parsed['state']} mode={mode_s}")
        except Exception:
            print(f"[ASCII.BAT] {parsed}")
    elif packet_id == PID_ASCII_GENERIC:
        # parsed is the raw text line
        print(f"[ASCII.TEXT] {parsed}")
    else:
        print(f"[RX] Packet {packet_id}: {parsed}")


def print_help() -> None:
    """
    Show available commands.
    """
    print("""
Available Commands:
  reset        - Soft reset (opcode 7 / 0x07)
  start        - Enter OI Passive mode (opcode 128 / 0x80)
  safe         - Enter SAFE mode (opcode 131 / 0x83)
  full         - Enter FULL mode (opcode 132 / 0x84)
  dock         - Seek Home Base (opcode 143 / 0x8F)
  power_off    - Power down (opcode 133 / 0x85)

  rts_low      - Force RTS LOW (BRC active, no opcode)
  rts_high     - Force RTS HIGH (BRC inactive, no opcode)
  wakeup       - Pulse RTS LOW→HIGH to wake Roomba (BRC action)
  deep_off     - Force deep off: RTS LOW + POWER OFF (133)

  sensors [id] - Poll sensor data once (opcode 142 / 0x8E, default id=0 → all)
                 Example: sensors 7

  stream ids   - Start continuous streaming of packets
                 Example: stream 1 7
  stop_stream  - Stop continuous streaming (opcode 150 / 0x96)
  mode          - Read OI mode (packet 35). Values: 0=OFF, 1=PASSIVE, 2=SAFE, 3=FULL.
                 Note: START puts the robot into PASSIVE. Use SAFE or FULL to enable driving.

  help         - Show this help menu
  quit         - Exit CLI
""")


def main():
    parser = argparse.ArgumentParser(description="Roomba CLI")
    parser.add_argument("--device", default="/dev/ttyUSB0", help="Serial device path")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    # Initialize OIService
    svc = OIService(device=args.device, baudrate=args.baud)
    svc.open()
    svc.set_on_sensor(rx_logger)

    print("Roomba CLI started. Type 'help' for commands.")

    while True:
        try:
            cmd = input("> ").strip().lower()

            if cmd == "reset":
                svc.reset()
                print("Reset in progress...")
                time.sleep(5)
            elif cmd == "start":
                svc.start()
            elif cmd == "safe":
                svc.safe()
            elif cmd == "full":
                svc.full()
            elif cmd == "dock":
                svc.dock()
            elif cmd == "power_off":
                svc.power_off()
            elif cmd == "rts_low":
                svc._port.set_rts_low()
            elif cmd == "rts_high":
                svc._port.set_rts_high()
            elif cmd == "wakeup":
                svc._port.pulse_wakeup()
            elif cmd == "deep_off":
                svc._port.set_rts_low()
                svc.power_off()
                print("[INFO] Deep off: Roomba will stay OFF until wakeup pulse or CLEAN button.")
            elif cmd.startswith("sensors"):
                parts = cmd.split()
                if len(parts) == 2 and parts[1].isdigit():
                    packet_id = int(parts[1])
                else:
                    packet_id = 0  # all sensors
                try:
                    parsed = svc.get_sensor(packet_id)
                except TimeoutError:
                    print(f"[WARN] Sensor packet {packet_id} not received (timeout)")
                else:
                    print(f"[TX] Requested sensor {packet_id} → {parsed}")
            elif cmd.startswith("stream"):
                # Example: stream 1 7
                parts = cmd.split()
                ids = [int(x) for x in parts[1:] if x.isdigit()]
                if not ids:
                    print("Usage: stream <id1> <id2> ...")
                else:
                    svc.start_stream(ids)
                    print(f"[TX] Streaming packets: {ids}")
            elif cmd == "mode":
                try:
                    parsed = svc.get_sensor(35)
                except TimeoutError:
                    print("[WARN] OI mode packet not received (timeout)")
                else:
                    mode_value = None
                    if hasattr(parsed, "get"):
                        mode_value = parsed.get("oi_mode")
                    if mode_value is None:
                        mode_value = getattr(parsed, "oi_mode", None)
                    if mode_value is None:
                        mode_value = parsed
                    try:
                        mode_int = int(mode_value)
                    except (TypeError, ValueError):
                        mode_int = None
                    mode_names = {0: "Off", 1: "Passive", 2: "Safe", 3: "Full"}
                    if mode_int is not None:
                        mode_label = mode_names.get(mode_int, "Unknown")
                        print(f"[TX] OI mode → {mode_int} ({mode_label})")
                    else:
                        print(f"[TX] OI mode → {mode_value}")
            elif cmd == "stop_stream":
                svc.stop_stream()
                print("[TX] Stop sensor stream")
            elif cmd == "help":
                print_help()
            elif cmd in ("quit", "exit"):
                print("Exiting...")
                break
            elif cmd == "":
                continue
            else:
                print("Unknown command. Type 'help' for options.")
        except KeyboardInterrupt:
            print("\nExiting...")
            break

    svc.close()


if __name__ == "__main__":
    main()
