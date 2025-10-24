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
import logging
from typing import Callable, Optional, Mapping
import threading
import re

from roomba_stack.l1_drivers.pyserial_port import PySerialPort
from . import oi_codec
from . import oi_decode
from . import oi_protocol
from .protocol_queue import BoundedQueue
from roomba_stack.l0_core import EventBus
from roomba_stack.l0_core.events import SensorUpdate, now_ms, Value  # Value = int|float|bool|str


# -----------------------------------------------------------------------------
# Feature flags / architecture
# -----------------------------------------------------------------------------
QUEUE_DISPATCHER = True          # use dispatcher thread that decodes from a byte buffer
ENQUEUE_ONLY_RX  = True          # RX callback enqueues only; no parsing on callback thread

# Queue sizes & timeouts
RX_QUEUE_MAX   = 4096
TX_QUEUE_MAX   = 256
Q_PUT_TIMEOUT  = 0.01
Q_GET_TIMEOUT  = 0.10

# TX pacing and shutdown
TX_INTER_CMD_DELAY = 0.02        # 20ms between commands (tunable)
SENTINEL = object()              # sentinel to wake TX writer during shutdown

# Diagnostics (dev)
DEBUG_RAW_UNKNOWN = False        # dump unknown-leading bytes (HEX + ASCII) for diagnosis
RAW_DUMP_HEAD     = 48           # how many bytes of the buffer front to show

# ASCII sniff (boot/status lines)
ASCII_SNIFF = True               # treat printable text as ASCII lines
ASCII_MAX   = 80                 # cap per-line capture

# Virtual PIDs for ASCII lines
PID_ASCII_GENERIC = -100         # plain text line
PID_ASCII_BATSTAT = -101         # parsed battery/status line

_ASCII_BAT_RE = re.compile(
    r"^bat:\s+min\s+(?P<min>\d+)\s+sec\s+(?P<sec>\d+)\s+mV\s+(?P<mv>\d+)\s+mA\s+(?P<ma>-?\d+)\s+rx-byte\s+(?P<rx>\d+)\s+mAH\s+(?P<mah>\d+)\s+state\s+(?P<state>\d+)\s+mode\s+(?P<mode>\d+)",
    re.IGNORECASE,
)

log = logging.getLogger(__name__)


class OIService:
    """
    High-level service for controlling a Roomba via OI.
    """

    def __init__(self, device: str, eventbus: EventBus, baudrate: int = 115200) -> None:
        self._port = PySerialPort(device, baudrate)
        self._latest_packets: dict[int, object] = {}
        self._on_sensor: Optional[Callable[[int, object], None]] = None

        # Legacy buffers for the non-dispatcher path
        self._rx_buffer = bytearray()         # for stream frames
        self._rx_buffer_single = bytearray()  # for single packets

        # Track pending single packet request (legacy path)
        self._pending_request_id: Optional[int] = None

        # Bounded queues for RX bytes and TX frames  (ctor: maxsize, name)
        self._rx_bytes  = BoundedQueue(RX_QUEUE_MAX, "RxByteQueue")
        self._tx_frames = BoundedQueue(TX_QUEUE_MAX, "TxFrameQueue")

        # Threads & lifecycle
        self._dispatcher_thread = None
        self._tx_thread = None
        self._alive = threading.Event()
        self._alive.clear()

        # Dispatcher-side RX assembly buffer
        self._rx_buf = bytearray()

        # ASCII accumulation buffer
        self._ascii_accum = bytearray()
        
        self._eventbus = eventbus

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------
    def open(self) -> None:
        """Open the serial connection and start RX/TX threads."""
        if QUEUE_DISPATCHER:
            self._alive.set()
            self._dispatcher_thread = threading.Thread(
                target=self._dispatcher_loop, name="dispatcher", daemon=True
            )
            self._dispatcher_thread.start()
            self._port.set_reader(self._on_serial_bytes)
        else:
            self._port.set_reader(self._rx_handler)

        # start TX writer thread
        self._tx_thread = threading.Thread(
            target=self._tx_writer_loop, name="tx-writer", daemon=True
        )
        self._tx_thread.start()

        self._port.open()

    def close(self) -> None:
        """
        Gracefully stop threads and close the serial connection.

        Behavior:
        - Signals dispatcher and TX writer to stop.
        - Unregisters serial reader to prevent late callbacks.
        - Joins threads with bounded timeouts.
        - Closes the serial port last.

        Idempotent and exception-safe.
        """
        was_running = self._alive.is_set()
        self._alive.clear()

        # Unregister reader
        try:
            self._port.set_reader(None)
        except Exception:
            log.exception("Error while unregistering serial reader")

        # Wake TX writer (in case it waits on queue.get)
        try:
            self._tx_frames.put(SENTINEL, timeout=0)
        except Exception:
            pass

        # Join dispatcher
        t = self._dispatcher_thread
        if was_running and t and t.is_alive() and t is not threading.current_thread():
            try:
                t.join(timeout=0.5)
            except Exception:
                log.exception("Error while joining dispatcher thread")
        self._dispatcher_thread = None
        log.info("Dispatcher thread stopped")

        # Join TX writer
        t = self._tx_thread
        if t and t.is_alive() and t is not threading.current_thread():
            try:
                t.join(timeout=0.5)
            except Exception:
                log.exception("Error while joining tx-writer thread")
        self._tx_thread = None
        log.info("TX writer thread stopped")

        # Close port last
        self._port.close()

    # -------------------------------------------------------------------------
    # Control Commands
    # -------------------------------------------------------------------------
    def reset(self) -> None:        self._send(oi_codec.encode_reset())
    def start(self) -> None:        self._send(oi_codec.encode_start())
    def safe(self) -> None:         self._send(oi_codec.encode_safe())
    def full(self) -> None:         self._send(oi_codec.encode_full())
    def dock(self) -> None:         self._send(oi_codec.encode_dock())
    def power_off(self) -> None:    self._send(oi_codec.encode_power())
    def drive(self, velocity: int, radius: int) -> None:
        self._send(oi_codec.encode_drive(velocity, radius))
    def drive_direct(self, right: int, left: int) -> None:
        self._send(oi_codec.encode_drive_direct(right, left))

    def stream_pause(self) -> None:
        """Pause sensor streaming (STREAM_CTRL=0)."""
        self._send(oi_codec.encode_pause_stream())

    def stream_resume(self) -> None:
        """Resume sensor streaming (STREAM_CTRL=1)."""
        self._send(oi_codec.encode_resume_stream())

    def motors(
        self,
        *,
        main_on: bool,
        vacuum_on: bool,
        side_on: bool,
        main_reverse: bool = False,
        side_reverse: bool = False,
    ) -> None:
        """Control main, vacuum, and side brushes; optional direction flips."""
        self._send(
            oi_codec.encode_motors(
                main_on=main_on,
                vacuum_on=vacuum_on,
                side_on=side_on,
                main_reverse=main_reverse,
                side_reverse=side_reverse,
            )
        )

    def pwm_motors(self, main_pwm: int, side_pwm: int, vacuum_pwm: int) -> None:
        """Set PWM for main/side (±127) and vacuum (0..127)."""
        self._send(oi_codec.encode_pwm_motors(main_pwm, side_pwm, vacuum_pwm))

    def drive_pwm(self, right_pwm: int, left_pwm: int) -> None:
        """
        Drive wheels using raw PWM values (-255..255), right then left.
        Positive = forward, negative = reverse.
        """
        self._send(oi_codec.encode_drive_pwm(right_pwm, left_pwm))

    # -------------------------------------------------------------------------
    # Sensor Queries
    # -------------------------------------------------------------------------
    def get_sensor(self, packet_id: int, timeout: float = 1.0) -> object:
        """
        Query a single sensor packet in polled mode (OI opcode 142: SENSORS).
        """
        # Clear stale state
        self._latest_packets.pop(packet_id, None)
        self._rx_buffer_single.clear()
        self._pending_request_id = packet_id

        # Validate packet length (ensures supported ID)
        expected_len = oi_protocol.packet_length(packet_id)
        log.debug("packet_length(%d) = %d", packet_id, expected_len)
        if expected_len <= 0:
            raise ValueError(f"Unknown or unsupported packet ID {packet_id}")

        # Send request via TX writer (thread-safe)
        self._send(oi_codec.encode_sensors(packet_id))

        # Wait for the parser to populate the latest value
        start = time.time()
        while time.time() - start < timeout:
            if packet_id in self._latest_packets:
                return self._latest_packets[packet_id]
            time.sleep(0.01)

        raise TimeoutError(f"Sensor packet {packet_id} not received within {timeout}s")

    def query_list(self, packet_ids: list[int], timeout: float = 1.0) -> dict[int, object]:
        """
        Query multiple sensor packets in one request (OI opcode 149).
        """
        if not packet_ids:
            return {}
        for pid in packet_ids:
            self._latest_packets.pop(pid, None)

        # enqueue via TX writer (thread-safe)
        self._send(oi_codec.encode_query_list(packet_ids))

        start = time.time()
        while time.time() - start < timeout:
            if all(pid in self._latest_packets for pid in packet_ids):
                return {pid: self._latest_packets[pid] for pid in packet_ids}
            time.sleep(0.01)
        raise TimeoutError(f"Sensor packets {packet_ids} not fully received")

    def start_stream(self, packet_ids: list[int]) -> None:
        """
        Start continuous sensor streaming (OI opcode 148: STREAM).
        """
        if not packet_ids:
            raise ValueError("packet_ids must not be empty")
        self._send(oi_codec.encode_stream(packet_ids))

    def stop_stream(self) -> None:
        """
        Pause/stop continuous streaming (OI opcode 150: STREAM_CTRL=0).
        """
        self._send(oi_codec.encode_pause_stream())

    # -------------------------------------------------------------------------
    # RX Handling
    # -------------------------------------------------------------------------
    def _rx_handler(self, data: bytes) -> None:
        """
        Legacy RX path (no dispatcher); retained for completeness.
        """
        if not self._alive.is_set() or not data:
            return

        # Raw dump (dev)
        hex_repr = data.hex()
        ascii_repr = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)
        print(f"[RX RAW] HEX: {hex_repr}")
        print(f"[RX RAW] ASCII: {ascii_repr}")

        # Stream frame
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

        # Pending single packet
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

        # Fallback → ASCII
        self._rx_buffer_single.extend(data)
        try:
            ascii_text = self._rx_buffer_single.decode(errors="ignore")
        except Exception:
            ascii_text = self._rx_buffer_single.hex()
        print(f"[LOG] {ascii_text.strip()}")
        self._rx_buffer_single.clear()

    # -------------------------------------------------------------------------
    # Callback Registration
    # -------------------------------------------------------------------------
    def set_on_sensor(self, cb: Callable[[int, object], None]) -> None:
        """Register a callback for sensor updates."""
        self._on_sensor = cb

    def _on_serial_bytes(self, data: bytes) -> None:
        """Reader callback from PySerialPort: enqueue bytes only."""
        if not self._alive.is_set() or not data:
            return
        ok = self._rx_bytes.put(data, timeout=Q_PUT_TIMEOUT)
        if not ok:
            log.warning("[%s] overflow: dropped %d bytes", self._rx_bytes.name(), len(data))

    def _dispatcher_loop(self) -> None:
        """Background thread to dispatch RX bytes."""
        log.info("Dispatcher thread started")
        while self._alive.is_set():
            ok, chunk = self._rx_bytes.get(timeout=Q_GET_TIMEOUT)
            if not ok:
                continue    # timed wait; no busy spin
            self._rx_buf.extend(chunk)
            self._decode_available_frames()

    def _deliver(self, pid: int, parsed: object) -> None:
        self._latest_packets[pid] = parsed
        if self._on_sensor:
            try:
                self._on_sensor(pid, parsed)
            except Exception:
                log.exception("on_sensor callback error")

    def _decode_available_frames(self) -> None:
        """
        Consume and decode as many complete frames as available in `_rx_buf`.
        """
        while True:
            if not self._rx_buf:
                break

            b0 = self._rx_buf[0]

            # A) Stream frame (0x13 header)
            if b0 == 0x13:
                if len(self._rx_buf) < 3:
                    break
                n = self._rx_buf[1]
                if n > 128:
                    log.warning("Stream len insane (%d), resync drop 1", n)
                    del self._rx_buf[0]
                    continue
                total = 2 + n + 1
                if len(self._rx_buf) < total:
                    break
                frame = bytes(self._rx_buf[:total])
                try:
                    results = oi_decode.decode_stream_frame(frame)
                except ValueError:
                    log.warning("RX resync: dropping 1 byte (buf=%d)", len(self._rx_buf))
                    del self._rx_buf[0]
                    continue

                for pid, parsed in results.items():
                    self._deliver(pid, parsed)

                del self._rx_buf[:total]
                continue

            # B) Pending single packet reply (opcode 142: raw payload only)
            if self._pending_request_id is not None:
                pid = self._pending_request_id
                expected_len = oi_protocol.packet_length(pid)
                if expected_len <= 0:
                    log.warning("Unknown length for pid=%d; resync drop 1 (buf=%d)", pid, len(self._rx_buf))
                    del self._rx_buf[0]
                    continue
                if len(self._rx_buf) < expected_len:
                    break

                raw = bytes(self._rx_buf[:expected_len])
                try:
                    parsed = oi_decode.decode_sensor(pid, raw)
                except Exception:
                    log.warning("Decode failed for pid=%d; resync drop 1 (buf=%d)", pid, len(self._rx_buf))
                    del self._rx_buf[0]
                    continue

                self._deliver(pid, parsed)
                del self._rx_buf[:expected_len]
                self._pending_request_id = None
                continue

            # C) ASCII pre-filter (dev)
            if ASCII_SNIFF and self._is_printable(b0):
                eol = self._find_eol(self._rx_buf)
                if eol == -1:
                    take = min(len(self._rx_buf), ASCII_MAX - len(self._ascii_accum))
                    self._ascii_accum.extend(self._rx_buf[:take])
                    del self._rx_buf[:take]
                    if len(self._ascii_accum) >= ASCII_MAX:
                        line = bytes(self._ascii_accum); self._ascii_accum.clear()
                        preview = self._ascii_preview(line)
                        parsed = self._parse_ascii_line(preview)
                        if parsed:
                            pid, payload = parsed; self._deliver(pid, payload)
                        else:
                            self._deliver(PID_ASCII_GENERIC, preview)
                        if DEBUG_RAW_UNKNOWN:
                            log.warning("[ASCII] HEX: %s", line.hex())
                            log.warning("[ASCII] TEXT: %s", preview)
                    continue
                else:
                    self._ascii_accum.extend(self._rx_buf[:eol])
                    del self._rx_buf[:eol]
                    line = bytes(self._ascii_accum); self._ascii_accum.clear()
                    preview = self._ascii_preview(line)
                    parsed = self._parse_ascii_line(preview)
                    if parsed:
                        pid, payload = parsed; self._deliver(pid, payload)
                    else:
                        self._deliver(PID_ASCII_GENERIC, preview)
                    if DEBUG_RAW_UNKNOWN:
                        log.warning("[ASCII] HEX: %s", line.hex())
                        log.warning("[ASCII] TEXT: %s", preview)
                    continue

            # D) Unknown/unexpected leading byte → drop to resync quickly
            log.warning("RX resync: dropping 1 byte (buf=%d)", len(self._rx_buf))
            del self._rx_buf[0]
            continue

    # -------------------------------------------------------------------------
    # ASCII helpers
    # -------------------------------------------------------------------------
    def _is_printable(self, b: int) -> bool:
        return 32 <= b <= 126 or b in (9,)  # tab allowed

    def _find_eol(self, buf: bytearray) -> int:
        # return index after CRLF/LF/CR if present; else -1
        for i, v in enumerate(buf):
            if v in (10, 13):  # LF or CR
                j = i + 1
                if j < len(buf) and buf[j] in (10, 13) and buf[j] != v:
                    return j + 1
                return i + 1
        return -1

    def _ascii_preview(self, data: bytes) -> str:
        """Return a printable preview: ASCII printable as-is, others as '.'"""
        out = []
        for b in data:
            out.append(chr(b) if 32 <= b <= 126 else '.')
        return ''.join(out)

    def _parse_ascii_line(self, text: str):
        """
        Try to parse known ASCII status lines.
        Returns (pid, payload) or None if not recognized.
        """
        m = _ASCII_BAT_RE.match(text.strip())
        if m:
            d = {k: int(v) for k, v in m.groupdict().items()}
            return (PID_ASCII_BATSTAT, d)
        return None

    # -------------------------------------------------------------------------
    # TX helpers (single writer owns serial TX)
    # -------------------------------------------------------------------------
    def _send(self, frame: bytes) -> None:
        """Enqueue a command frame to be written by the TX writer thread."""
        if not frame:
            return
        if not self._alive.is_set():
            log.debug("drop TX while not alive (len=%d)", len(frame))
            return
        ok = self._tx_frames.put(frame, timeout=Q_PUT_TIMEOUT)
        if not ok:
            log.warning("[TxFrameQueue] overflow: dropped frame len=%d", len(frame))

    def _tx_writer_loop(self) -> None:
        """Single writer that owns serial TX to avoid interleaving from many callers."""
        log.info("TX writer started")
        while True:
            ok, item = self._tx_frames.get(timeout=Q_GET_TIMEOUT)
            if not ok:
                if not self._alive.is_set():
                    break
                continue
            if item is SENTINEL or not self._alive.is_set():
                break
            try:
                self._port.write(item)
            except Exception:
                log.exception("TX write failed")
            if TX_INTER_CMD_DELAY:
                time.sleep(TX_INTER_CMD_DELAY)
        log.info("TX writer exiting")


    def _publish_sensor(self, packet_id: int, fields: Mapping[str, Value]) -> None:
        """
        Build and publish a typed SensorUpdate on topic 'sensors'.
        Runs fast; if the EventBus queue is full we drop and continue (no IO stall).
        """
        evt = SensorUpdate(
            timestamp_millis=now_ms(),
            packet_id=packet_id,
            source="oi",
            fields=fields,
        )
        ok = self._eventbus.publish("sensors", evt)
        if not ok:
            # Minimal early behavior: make it visible during bring-up.
            # Later we will publish a Fault event instead of printing.
            print("[WARN] EventBus full; dropped SensorUpdate", packet_id, fields)