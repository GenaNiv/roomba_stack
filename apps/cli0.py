#!/usr/bin/env python3
"""
Roomba CLI0 Tool
================

Minimal interactive CLI to test Roomba Open Interface (OI) commands.

- Directly sends opcodes to Roomba.
- Logs all received bytes (HEX + ASCII).
- Supports polling (142), query list (149), streaming (148), and pause/resume stream (150).
- Includes basic decoder for common sensor packets.
"""

import argparse
from roomba_stack.l1_drivers.pyserial_port import PySerialPort

# Buffer for accumulating sensor replies
rx_buffer = bytearray()
last_packet_id = None  # track the last requested packet

# Mini decoder table: packet_id → (name, length, parser)
PACKET_INFO = {
    7:  ("Bumps & Wheel Drops", 1, lambda b: b[0]),
    8:  ("Wall", 1, lambda b: b[0]),
    13: ("Virtual Wall", 1, lambda b: b[0]),
    14: ("Wheel Overcurrents", 1, lambda b: b[0]),
    19: ("Distance (mm)", 2, lambda b: int.from_bytes(b, "big", signed=True)),
    20: ("Angle (deg)", 2, lambda b: int.from_bytes(b, "big", signed=True)),
    21: ("Charging State", 1, lambda b: b[0]),
    22: ("Battery Voltage (mV)", 2, lambda b: int.from_bytes(b, "big")),
    23: ("Battery Current (mA)", 2, lambda b: int.from_bytes(b, "big", signed=True)),
    24: ("Battery Temp (°C)", 1, lambda b: int.from_bytes(b, "big", signed=True)),
    25: ("Battery Charge (mAh)", 2, lambda b: int.from_bytes(b, "big")),
    26: ("Battery Capacity (mAh)", 2, lambda b: int.from_bytes(b, "big")),
}


def rx_logger(data: bytes) -> None:
    """Log and parse incoming bytes from Roomba."""
    global rx_buffer, last_packet_id

    # Always dump raw bytes
    hex_repr = data.hex()
    ascii_repr = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)
    print(f"[RX] HEX: {hex_repr}")
    print(f"[RX] ASCII: {ascii_repr}")

    # If a new sensor packet request was just made, reset buffer
    if last_packet_id is not None and not rx_buffer:
        rx_buffer = bytearray()

    # Append new bytes
    rx_buffer.extend(data)

    # Try to parse if we know expected packet
    if last_packet_id in PACKET_INFO:
        name, length, parser = PACKET_INFO[last_packet_id]

        # Only parse when we have the full length
        if len(rx_buffer) >= length:
            raw = bytes(rx_buffer[:length])
            print(f"[RAW] Decimal bytes = {[int(x) for x in raw]}")

            try:
                value = parser(raw)
                print(f"[PARSED] {name} = {value}")
            except Exception as e:
                print(f"[ERR] Failed to parse {name}: {e}")

            # Reset buffer after parsing
            rx_buffer = bytearray()
            last_packet_id = None



def print_help() -> None:
    """Show available commands."""
    print("""
Available Commands:
  reset        - Soft reset (7 / 0x07)
  start        - Enter OI mode (128 / 0x80)
  safe         - Enter SAFE mode (131 / 0x83)
  full         - Enter FULL mode (132 / 0x84)
  stop         - Stop OI (173 / 0xAD)
  power        - Power down (133 / 0x85)
  dock         - Seek Home Base (143 / 0x8F)
  clean        - Start cleaning (135 / 0x87)
  spot         - Spot clean (134 / 0x86)
  max          - Max clean until battery empty (136 / 0x88)

  rts_low      - Force RTS LOW (BRC active)
  rts_high     - Force RTS HIGH (BRC inactive)
  wakeup       - Pulse RTS LOW→HIGH to wake

  sensors [id] - Poll sensor packet once (142 / 0x8E)
                 Examples:
                   sensors 7   → bumps & wheel drops
                   sensors 19  → distance
                   sensors 25  → battery charge

  query n ids  - Query list once (149 / 0x95)
                 Example: query 2 7 19

  stream n ids - Start continuous streaming (148 / 0x94)
                 Example: stream 2 19 20
  pause_stream - Pause stream (150 0)"""  """
  resume_stream- Resume stream (150 1)
  stop_stream  - Stop and clear streaming (150 / 0x96 with 0)

  help         - Show this menu
  quit/exit    - Exit CLI
""")


def main():
    global last_packet_id
    parser = argparse.ArgumentParser(description="Roomba CLI0 (raw tester)")
    parser.add_argument("--device", default="/dev/ttyUSB0", help="Serial device path")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    # Initialize serial port
    port = PySerialPort(device=args.device, baudrate=args.baud)
    port.set_reader(rx_logger)
    port.open()

    print("Roomba CLI0 started. Type 'help' for commands.")

    while True:
        try:
            cmd = input("> ").strip().lower()

            if cmd == "reset":
                port.write(bytes([7]))
                print("[TX] Reset (7)")

            elif cmd == "start":
                port.write(bytes([128]))
                print("[TX] Start (128)")

            elif cmd == "safe":
                port.write(bytes([131]))
                print("[TX] Safe (131)")

            elif cmd == "full":
                port.write(bytes([132]))
                print("[TX] Full (132)")

            elif cmd == "stop":
                port.write(bytes([173]))
                print("[TX] Stop (173)")

            elif cmd == "power":
                port.write(bytes([133]))
                print("[TX] Power (133)")

            elif cmd == "dock":
                port.write(bytes([143]))
                print("[TX] Dock (143)")

            elif cmd == "clean":
                port.write(bytes([135]))
                print("[TX] Clean (135)")

            elif cmd == "spot":
                port.write(bytes([134]))
                print("[TX] Spot (134)")

            elif cmd == "max":
                port.write(bytes([136]))
                print("[TX] Max (136)")

            elif cmd == "rts_low":
                port.set_rts_low()
                print("[TX] RTS LOW")

            elif cmd == "rts_high":
                port.set_rts_high()
                print("[TX] RTS HIGH")

            elif cmd == "wakeup":
                port.pulse_wakeup()
                print("[TX] Wakeup pulse")

            elif cmd.startswith("sensors"):
                parts = cmd.split()
                packet_id = int(parts[1]) if len(parts) == 2 and parts[1].isdigit() else 0
                last_packet_id = packet_id
                port.write(bytes([142, packet_id]))
                print(f"[TX] Requested sensor packet {packet_id}")

            elif cmd.startswith("query"):
                parts = cmd.split()
                if len(parts) >= 3 and parts[1].isdigit():
                    n = int(parts[1])
                    ids = [int(x) for x in parts[2:] if x.isdigit()]
                    if len(ids) != n:
                        print("[ERR] Number of IDs must match n")
                    else:
                        payload = [149, n] + ids
                        port.write(bytes(payload))
                        print(f"[TX] Query packets: {ids}")
                else:
                    print("Usage: query <n> <id1> <id2> ...")

            elif cmd.startswith("stream"):
                parts = cmd.split()
                if len(parts) >= 3 and parts[1].isdigit():
                    n = int(parts[1])
                    ids = [int(x) for x in parts[2:] if x.isdigit()]
                    if len(ids) != n:
                        print("[ERR] Number of IDs must match n")
                    else:
                        payload = [148, n] + ids
                        port.write(bytes(payload))
                        print(f"[TX] Start stream packets: {ids}")
                else:
                    print("Usage: stream <n> <id1> <id2> ...")

            elif cmd == "pause_stream":
                port.write(bytes([150, 0]))
                print("[TX] Pause stream (150 0)")

            elif cmd == "resume_stream":
                port.write(bytes([150, 1]))
                print("[TX] Resume stream (150 1)")

            elif cmd == "stop_stream":
                port.write(bytes([150, 0]))
                print("[TX] Stop stream (150 0) and clear list")

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

    port.close()


if __name__ == "__main__":
    main()
