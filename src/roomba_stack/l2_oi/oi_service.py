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
    print(svc.get_sensor(7))  # bumps & wheel drops
"""

import time
from typing import Callable, Optional

from roomba_stack.l1_drivers.pyserial_port import PySerialPort
from . import oi_codec
from . import oi_decode
from . import oi_protocol
from .oi_protocol import Opcode


class OIService:
    """
    High-level service for controlling a Roomba via OI.
    """

    def __init__(self, device: str, baudrate: int = 115200) -> None:
        self._port = PySerialPort(device, baudrate)
        self._latest_packets: dict[int, object] = {}
        self._on_sensor: Optional[Callable[[int, object], None]] = None

        # Buffers for RX handling
        self._rx_buffer = bytearray()         # for stream frames
        self._rx_buffer_single = bytearray()  # for single packets

        # Track currently pending single packet request
        self._pending_request_id: Optional[int] = None

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

    def get_sensor(self, packet_id: int, timeout: float = 1.0) -> object:
        """
        Request one sensor packet (polled mode).
        Blocking call: sends [142][id] and waits for reply.

        Per OI spec, reply = only the data bytes (no leading packet ID).
        """
        # Clear stale state
        self._latest_packets.pop(packet_id, None)
        self._rx_buffer_single.clear()
        self._pending_request_id = packet_id

        # Determine expected reply length
        expected_len = oi_protocol.packet_length(packet_id)
        print("DEBUG: packet_length(", packet_id, ") =", expected_len)
        if expected_len <= 0:
            raise ValueError(f"Unknown or unsupported packet ID {packet_id}")

        # Send request
        self._port.write(oi_codec.encode_sensors(packet_id))

        # Wait until RX handler parses reply
        start = time.time()
        while time.time() - start < timeout:
            if packet_id in self._latest_packets:
                return self._latest_packets[packet_id]
            time.sleep(0.01)

        raise TimeoutError(f"Sensor packet {packet_id} not received within {timeout}s")

    def query_list(self, packet_ids: list[int], timeout: float = 1.0) -> dict[int, object]:
        """
        Request multiple sensor packets once (opcode 149).
        Returns dict mapping id → parsed value.
        """
        for pid in packet_ids:
            self._latest_packets.pop(pid, None)
        self._port.write(oi_codec.encode_query_list(packet_ids))

        start = time.time()
        while time.time() - start < timeout:
            if all(pid in self._latest_packets for pid in packet_ids):
                return {pid: self._latest_packets[pid] for pid in packet_ids}
            time.sleep(0.01)
        raise TimeoutError(f"Sensor packets {packet_ids} not fully received")

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
        """
        Handle incoming bytes from Roomba.

        - Stream frames (opcode 148 replies): framed with 0x13 + checksum.
        - Single sensor replies (opcode 142): raw data only, length from schema.
        - Query list replies (opcode 149): TODO – parse multiple packets.
        - Boot/log messages: ASCII text, fallback.
        """

        if not data:
            return

        # Always dump raw bytes (debugging)
        hex_repr = data.hex()
        ascii_repr = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)
        print(f"[RX RAW] HEX: {hex_repr}")
        print(f"[RX RAW] ASCII: {ascii_repr}")

        # Case 1: Stream frame
        if data[0] == 19:  # 0x13 header
            self._rx_buffer.extend(data)
            try:
                results = oi_decode.decode_stream_frame(bytes(self._rx_buffer))
                for pid, parsed in results.items():
                    self._latest_packets[pid] = parsed
                    if self._on_sensor:
                        self._on_sensor(pid, parsed)
                self._rx_buffer.clear()
            except ValueError:
                pass
            return

        # Case 2: Pending single packet request
        if self._pending_request_id:
            pid = self._pending_request_id
            expected_len = oi_protocol.packet_length(pid)
            print("DEBUG: packet_length(", pid, ") =", expected_len)

            self._rx_buffer_single.extend(data)
            if len(self._rx_buffer_single) >= expected_len:
                raw = bytes(self._rx_buffer_single[:expected_len])
                parsed = oi_decode.decode_sensor(pid, raw)

                self._latest_packets[pid] = parsed
                if self._on_sensor:
                    self._on_sensor(pid, parsed)

                del self._rx_buffer_single[:expected_len]
                self._pending_request_id = None
            return

        # Case 3: Fallback → log ASCII
        self._rx_buffer_single.extend(data)
        try:
            ascii_text = self._rx_buffer_single.decode(errors="ignore")
        except Exception:
            ascii_text = self._rx_buffer_single.hex()
        print(f"[LOG] {ascii_text.strip()}")
        self._rx_buffer_single.clear()

    # ------------------------------------------------------------
    # Callback Registration
    # ------------------------------------------------------------

    def set_on_sensor(self, cb: Callable[[int, object], None]) -> None:
        """Register a callback for sensor updates."""
        self._on_sensor = cb
