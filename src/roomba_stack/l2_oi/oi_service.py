"""
oi_service.py
=============
High-level service layer for the Roomba Open Interface (OI).

This class wraps:
- L1 driver (`PySerialPort`) for serial comms.
- L2 codec (`oi_codec`) to build outgoing command frames (TX).
- L2 decode (`oi_decode`) to parse incoming sensor packets (RX).

Design:
- Provides convenience methods: start(), safe(), drive(), dock(), etc.
- Provides sensor queries: get_sensor(id), query_list(ids), stream(ids).
- Handles RX asynchronously via the PySerialPort reader callback.

Usage:
    from roomba_stack.l2_oi.oi_service import OIService

    svc = OIService("/dev/ttyUSB0")
    svc.open()
    svc.start()
    svc.safe()
    svc.drive(200, 0)
    print(svc.get_sensor(7))  # distance in mm
"""

import time
from typing import Callable, Optional

from roomba_stack.l1_drivers.pyserial_port import PySerialPort
from . import oi_codec
from . import oi_decode
from .oi_protocol import Opcode


class OIService:
    """
    High-level service for controlling a Roomba via OI.
    """

    def __init__(self, device: str, baudrate: int = 115200) -> None:
        self._port = PySerialPort(device, baudrate)
        self._latest_packets: dict[int, object] = {}
        self._on_sensor: Optional[Callable[[int, object], None]] = None
        self._rx_buffer = bytearray()  # for handling stream frames

    # ------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------

    def open(self) -> None:
        """Open the serial connection and start RX thread."""
        self._port.set_reader(self._rx_handler)
        self._port.open()

    def close(self) -> None:
        """Close the serial connection."""
        self._port.close()

    # ------------------------------------------------------------
    # Control Commands
    # ------------------------------------------------------------

    def reset(self) -> None:
        self._port.write(bytes([Opcode.RESET]))
        time.sleep(0.1)

    def start(self) -> None:
        self._port.write(bytes([Opcode.START]))
        time.sleep(0.1)

    def safe(self) -> None:
        self._port.write(bytes([Opcode.SAFE]))
        time.sleep(0.1)

    def full(self) -> None:
        self._port.write(bytes([Opcode.FULL]))
        time.sleep(0.1)

    def dock(self) -> None:
        self._port.write(bytes([Opcode.DOCK]))

    def power_off(self) -> None:
        self._port.write(bytes([Opcode.POWER]))

    def drive(self, velocity: int, radius: int) -> None:
        self._port.write(oi_codec.encode_drive(velocity, radius))

    def drive_direct(self, right: int, left: int) -> None:
        self._port.write(oi_codec.encode_drive_direct(right, left))

    # ------------------------------------------------------------
    # Sensor Queries
    # ------------------------------------------------------------

    def get_sensor(self, packet_id: int) -> object:
        """Request one sensor packet (polled mode)."""
        self._port.write(oi_codec.encode_sensors(packet_id))
        time.sleep(0.05)  # allow reply to arrive
        return self._latest_packets.get(packet_id)

    def query_list(self, packet_ids: list[int]) -> dict[int, object]:
        """Request multiple sensor packets once."""
        self._port.write(oi_codec.encode_query_list(packet_ids))
        time.sleep(0.1)
        return {pid: self._latest_packets.get(pid) for pid in packet_ids}

    def start_stream(self, packet_ids: list[int]) -> None:
        """Start continuous streaming of sensor packets."""
        self._port.write(oi_codec.encode_stream(packet_ids))

    def stop_stream(self) -> None:
        """Stop continuous streaming."""
        self._port.write(oi_codec.encode_stop_stream())

    # ------------------------------------------------------------
    # RX Handling
    # ------------------------------------------------------------

    def _rx_handler(self, data: bytes) -> None:
        if not data:
            return

        # Always dump raw bytes (for debugging/logging)
        hex_repr = data.hex()
        ascii_repr = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)
        print(f"[RX RAW] HEX: {hex_repr}")
        print(f"[RX RAW] ASCII: {ascii_repr}")
        
        # Stream frame detection
        if data[0] == 19:
            self._rx_buffer.extend(data)
            try:
                results = oi_decode.decode_stream_frame(bytes(self._rx_buffer))
                for pid, parsed in results.items():
                    self._latest_packets[pid] = parsed
                    if self._on_sensor:
                        self._on_sensor(pid, parsed)
                self._rx_buffer.clear()
            except ValueError:
                # incomplete or bad frame → keep buffering
                pass
            return

        # Sensor packets (IDs 1–58)
        packet_id = data[0]
        if 1 <= packet_id <= 58:
            raw = data[1:]
            try:
                parsed = oi_decode.decode_sensor(packet_id, raw)
                self._latest_packets[packet_id] = parsed
                if self._on_sensor:
                    self._on_sensor(packet_id, parsed)
                return
            except Exception:
                # fall back to log if parsing fails
                pass

        # Otherwise: log output
        try:
            ascii_text = data.decode(errors="ignore")
        except Exception:
            ascii_text = data.hex()
        print(f"[LOG] {ascii_text.strip()}")



    # ------------------------------------------------------------
    # Callback Registration
    # ------------------------------------------------------------

    def set_on_sensor(self, cb: Callable[[int, object], None]) -> None:
        """Register a callback for sensor updates."""
        self._on_sensor = cb
