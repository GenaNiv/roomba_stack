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
from typing import Callable, Optional
import threading
import re

from roomba_stack.l1_drivers.pyserial_port import PySerialPort
from . import oi_codec
from . import oi_decode
from . import oi_protocol
from .oi_protocol import Opcode
from .protocol_queue import BoundedQueue

QUEUE_DISPATCHER = True

RX_QUEUE_MAX = 4096
TX_QUEUE_MAX = 256
Q_PUT_TIMEOUT = 0.01
Q_GET_TIMEOUT = 0.10

# Guard: RX callback must remain enqueue-only (no parsing here)
ENQUEUE_ONLY_RX = True

# Feature flags / diagnostics (dev-only)
DEBUG_RAW_UNKNOWN = False   # dump unknown-leading bytes (HEX + ASCII) for diagnosis
RAW_DUMP_HEAD     = 48      # how many bytes of the buffer front to show

ASCII_SNIFF       = True    # treat printable text as ASCII lines (no resync spam)
ASCII_MAX         = 80      # cap per-line capture

PID_ASCII_GENERIC = -100    # plain text line
PID_ASCII_BATSTAT = -101    # parsed battery/status line

_ASCII_BAT_RE = re.compile(
    r"^bat:\s+min\s+(?P<min>\d+)\s+sec\s+(?P<sec>\d+)\s+mV\s+(?P<mv>\d+)\s+mA\s+(?P<ma>-?\d+)\s+rx-byte\s+(?P<rx>\d+)\s+mAH\s+(?P<mah>\d+)\s+state\s+(?P<state>\d+)\s+mode\s+(?P<mode>\d+)",
    re.IGNORECASE,
)

# TX pacing and shutdown
TX_INTER_CMD_DELAY = 0.02  # 20ms between commands (tune/disable as needed)
SENTINEL = object()        # sentinel to wake the TX writer during shutdown

log = logging.getLogger(__name__)


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

        # Bounded queues for RX bytes and TX frames
        self._rx_bytes = BoundedQueue(RX_QUEUE_MAX, "RxByteQueue")
        self._tx_frames = BoundedQueue(TX_QUEUE_MAX, "TxFrameQueue")
        
        self._dispatcher_thread = None
        self._alive = threading.Event()   # service-level on/off
        self._alive.clear()
        self._rx_buf = bytearray()
        
        self._ascii_accum = bytearray()

        self._tx_thread = None
        self._ascii_accum = bytearray()

        

    # ------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------

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
        Gracefully stop the dispatcher and close the serial connection.

        Behavior:
        - Signals the dispatcher loop to stop by setting `_running = False`.
        - Unregisters the serial reader to prevent late callbacks after shutdown starts.
        - Joins the dispatcher thread with a bounded timeout (0.5s) if it's alive
            and not the current thread (avoids self-join deadlock).
        - Closes the serial port last.

        Notes:
        - Idempotent: safe to call multiple times.
        - Ordering matters: stop -> unregister reader -> join -> close port.
        - Any exceptions during unregistering or joining are caught and logged.
        """
        # 1) Tell the dispatcher loop to exit on its next iteration.
        was_running = self._alive.is_set()
        self._alive.clear()

        # 2) Best-effort: unregister the reader so no more callbacks arrive.
        try:
            self._port.set_reader(None)
        except Exception:
            # Reader may already be cleared or port may be mid-shutdown.
            log.exception("Error while unregistering serial reader")
            
        # 3) Wake TX writer (in case it's blocked on get)
        try:
            self._tx_frames.put(SENTINEL, timeout=0)
        except Exception:
            pass

        # 4) If we had a running dispatcher, try to join it (but never self-join).
        t = self._dispatcher_thread
        if was_running and t and t.is_alive() and t is not threading.current_thread():
            try:
                t.join(timeout=0.5)
            except Exception:
                log.exception("Error while joining dispatcher thread")

        self._dispatcher_thread = None
        log.info("Dispatcher thread stopped")

        # 5) Join TX writer
        t = self._tx_thread
        if t and t.is_alive() and t is not threading.current_thread():
            try:
                t.join(timeout=0.5)
            except Exception:
                log.exception("Error while joining tx-writer thread")
        self._tx_thread = None
        log.info("TX writer thread stopped")  
    
        # 6) Close the serial port last to release the device and file descriptors.
        self._port.close()


    # ------------------------------------------------------------
    # Control Commands
    # ------------------------------------------------------------

    def reset(self) -> None:
        self._send(oi_codec.encode_reset())

    def start(self) -> None:
        self._send(oi_codec.encode_start())

    def safe(self) -> None:
        self._send(oi_codec.encode_safe())

    def full(self) -> None:
        self._send(oi_codec.encode_full())

    def dock(self) -> None:
        self._send(oi_codec.encode_dock())

    def power_off(self) -> None:
        self._send(oi_codec.encode_power())

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
        """Control main, vacuum, side brushes; optional direction flips for main/side."""
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



    # ------------------------------------------------------------
    # Sensor Queries
    # ------------------------------------------------------------

    def get_sensor(self, packet_id: int, timeout: float = 1.0) -> object:
        """
        Query a single sensor packet in polled mode (OI opcode 142: SENSORS).

        Overview
        --------
        Sends a one-shot `[142][packet_id]` request and blocks until the reply for
        *that* packet is parsed by the RX pipeline, or until `timeout` elapses.
        The decoded value (typed per `oi_protocol.py`) is returned.

        Parameters
        ----------
        packet_id : int
            The OI packet ID to read (e.g., 7, 14, 18, 34, 45, 25, ...).
            Must be supported by `oi_protocol.packet_length(packet_id)`.
        timeout : float
            Maximum seconds to wait for the requested packet to arrive/parse.

        Behavior
        --------
        - Clears any stale cache entry for `packet_id`.
        - Clears the one-shot RX scratch buffer and marks `_pending_request_id`.
        - Validates `packet_id` via `oi_protocol.packet_length(...)`.
        - Enqueues the request via the TX writer (`_send(...)`).
        - Waits until `_latest_packets` contains a fresh value for `packet_id`.

        Returns
        -------
        object
            The decoded value for `packet_id` (e.g., int, dict of bitfields, etc.)
            as defined by the corresponding schema in `oi_protocol.py`.

        Raises
        ------
        ValueError
            If `packet_id` is unknown/unsupported (length <= 0).
        TimeoutError
            If no decoded value for `packet_id` is received within `timeout`.

        Notes
        -----
        • If continuous streaming is active, pause it first via `stream_pause()`,
        or prefer the streaming API to avoid mode conflicts.
        • Decoding is performed by the RX pipeline using `oi_protocol` constructs,
        so return types match your packet adapters (e.g., Packet 7/14/18/34/45).

        Example
        -------
        >>> svc.get_sensor(7)
        {'bump_right': False, 'bump_left': False, 'wheel_drop_right': False, 'wheel_drop_left': False}
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

        Overview
        --------
        Builds and sends a single `QUERY_LIST` frame (`[149][N][id1]...[idN]`) and
        blocks until *all* requested packet IDs have been parsed by the RX path or
        until `timeout` elapses. Parsed values are returned as a mapping
        `{packet_id: parsed_value}`. This is a *polled* (one-shot) read; it does not
        start streaming.

        Parameters
        ----------
        packet_ids : list[int]
            One or more OI packet IDs to query (e.g., [7, 14, 18]).
            - Duplicates are allowed; they don’t change the result (the last value wins).
            - An empty list returns `{}` immediately.
        timeout : float
            Max seconds to wait for all requested packets to arrive and be parsed.
            The wait includes any configured TX pacing.

        Behavior
        --------
        - Clears stale cache entries for the requested IDs.
        - Enqueues a `QUERY_LIST` frame via the TX writer (thread-safe).
        - Waits until every requested ID has a fresh entry in `_latest_packets`.
        - Returns a dict containing exactly the requested IDs (order in the dict
        is not guaranteed).

        Returns
        -------
        dict[int, object]
            Mapping from packet ID to the decoded/typed value as defined in
            `oi_protocol.py` (e.g., booleans/bitfields/adapters/ints).

        Raises
        ------
        TimeoutError
            If not all requested IDs were received/parsed within `timeout`.

        Notes
        -----
        • If sensor streaming is active, pause it first (see `stream_pause()`),
        or prefer the streaming API instead of mixing modes.
        • Values are decoded by the RX pipeline using the `oi_protocol` schemas,
        so types match your packet adapters (e.g., Packet 7/14/18/34/45 bitfields).

        Example
        -------
        >>> svc.query_list([7, 14, 18], timeout=1.0)
        {7: {'bump_right': False, 'bump_left': False, 'wheel_drop_right': False, 'wheel_drop_left': False},
        14: {'side_brush': False, 'reserved_bit1': False, 'main_brush': False, 'right_wheel': False, 'left_wheel': False},
        18: {'wall': False, 'cliff_left': False, 'cliff_front_left': False, 'cliff_front_right': False,
            'cliff_right': False, 'virtual_wall': False, 'overcurrent_left_wheel': False, 'overcurrent_right_wheel': False}}
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

        Overview
        --------
        Builds `[148][N][id1]...[idN]` and enqueues it via the TX writer. After this,
        the robot begins sending those packets periodically until paused/stopped.
        Your RX pipeline will keep parsing them and updating `_latest_packets`, and
        `on_sensor` (if registered) will be invoked per decoded item.

        Parameters
        ----------
        packet_ids : list[int]
            One or more OI packet IDs to stream. An empty list is invalid.

        Notes
        -----
        • Do not mix `get_sensor/query_list` polling while streaming; pause first.
        • See also: `stream_pause()` / `stream_resume()` to control an active stream.
        """
        if not packet_ids:
            raise ValueError("packet_ids must not be empty")
        self._send(oi_codec.encode_stream(packet_ids))


    def stop_stream(self) -> None:
        """
        Pause/stop continuous streaming (OI opcode 150: STREAM_CTRL=0).

        Overview
        --------
        Enqueues `[150][0]` to pause the active stream. Use `stream_resume()` to
        resume without resending the packet list, or `start_stream([...])` to start
        a fresh set of streamed packets.
        """
        # Backward-compatible with older name “stop_stream”; uses the pause control.
        self._send(oi_codec.encode_pause_stream())


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
        if not self._alive.is_set():
            return
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
        
    def _on_serial_bytes(self, data: bytes) -> None:
        """Callback from PySerialPort with incoming bytes."""
        if not self._alive.is_set():
            return
        if not data:
            return
        ok = self._rx_bytes.put(data, timeout=Q_PUT_TIMEOUT)
        if not ok:
            log.warning("[%s] overflow: dropped %d bytes", self._rx_bytes.name(), len(data))

    def _dispatcher_loop(self) -> None:
        """Background thread to dispatch RX bytes"""
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

        Overview
        - Runs only on the dispatcher thread; `_rx_buf` is append-only elsewhere.
        - Incremental parsing: when a complete frame is decoded, its bytes are
          removed from `_rx_buf`. If incomplete, the loop exits to await more RX.
        - Fast resync: on malformed frames or parse errors, drop one byte.

        Supported inputs
        1) Stream frame (opcode 148 reply)
           Bytes: [0x13][N][payload...][checksum]
           - 0x13: stream header
           - N: payload length in bytes (does NOT include header/len/checksum)
           - payload: concatenation of [packet_id][packet_payload] for one or
             more sensor packets
           - checksum: sum(header + N + payload + checksum) & 0xFF == 0

           Example (IDs 7 and 19):
           - Packet 7 (1 byte) = 0x03
           - Packet 19 (2 bytes) = 0x00 0x78 (distance = 120)
           - payload = 07 03 13 00 78  → N = 0x05
           - frame without checksum = 13 05 07 03 13 00 78  (sum = 0xAD)
           - checksum = (-0xAD) & 0xFF = 0x53
           - full frame = 13 05 07 03 13 00 78 53

        2) Single packet reply (opcode 142 request)
           Bytes: [raw payload only], length determined by the schema of the
           `_pending_request_id`. No packet ID byte and no checksum.
           Examples:
           - Request 7 → 1 byte: 05
           - Request 19 → 2 bytes: 00 78

        Not implemented here (future work)
        - Query List (opcode 149) combined replies. If needed, add explicit
          request tracking and parsing for multi-packet responses.
        """

        while True:
            # Nothing buffered: done for now
            if not self._rx_buf:
                break

            b0 = self._rx_buf[0]

            # Case A: Stream frame (0x13 header)
            if b0 == 0x13:
                # Need at least header + len + checksum
                if len(self._rx_buf) < 3:
                    break  # wait for more
                n = self._rx_buf[1]
                if n > 128:  # sanity cap for stream payload length
                    log.warning("Stream len insane (%d), resync drop 1", n)
                    del self._rx_buf[0]
                    continue
                total = 2 + n + 1
                if len(self._rx_buf) < total:
                    break  # incomplete frame
                frame = bytes(self._rx_buf[:total])
                try:
                    results = oi_decode.decode_stream_frame(frame)
                except ValueError:
                    # Bad checksum or malformed
                    log.warning("RX resync: dropping 1 byte (buf=%d)", len(self._rx_buf))
                    del self._rx_buf[0]
                    continue

                # Deliver parsed packets
                for pid, parsed in results.items():
                    self._deliver(pid, parsed)

                # Consume the frame
                del self._rx_buf[:total]
                continue

            # Case B: Pending single packet reply (opcode 142 response is raw payload only)
            if self._pending_request_id is not None:
                pid = self._pending_request_id
                expected_len = oi_protocol.packet_length(pid)
                if expected_len <= 0:
                    # Unknown length → drop one byte to avoid deadlock
                    log.warning("Unknown length for pid=%d; resync drop 1 (buf=%d)", pid, len(self._rx_buf))
                    del self._rx_buf[0]
                    continue
                if len(self._rx_buf) < expected_len:
                    break  # need more bytes

                raw = bytes(self._rx_buf[:expected_len])
                try:
                    parsed = oi_decode.decode_sensor(pid, raw)
                except Exception:
                    # Parsing failed → drop 1 byte and retry/resync
                    log.warning("Decode failed for pid=%d; resync drop 1 (buf=%d)", pid, len(self._rx_buf))
                    del self._rx_buf[0]
                    continue

                self._deliver(pid, parsed)
                del self._rx_buf[:expected_len]
                self._pending_request_id = None
                continue

            # ASCII pre-filter (dev)
            if ASCII_SNIFF and self._is_printable(b0):
                # accumulate printable run until EOL or MAX
                eol = self._find_eol(self._rx_buf)
                if eol == -1:
                    # no full line yet: move a chunk (bounded) to accum and wait
                    take = min(len(self._rx_buf), ASCII_MAX - len(self._ascii_accum))
                    self._ascii_accum.extend(self._rx_buf[:take])
                    del self._rx_buf[:take]
                    # if we hit the cap, flush as one line
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
                    # otherwise, wait for more bytes
                    continue
                else:
                    # we have a full line ending at 'eol'
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


            # Case C: Unknown/unexpected leading byte → drop to resync quickly
            log.warning("RX resync: dropping 1 byte (buf=%d)", len(self._rx_buf))
            del self._rx_buf[0]
            continue
        
    def _is_printable(self, b: int) -> bool:
        return 32 <= b <= 126 or b in (9,)  # tab allowed
    
    def _find_eol(self, buf: bytearray) -> int:
        # return index after CRLF/LF/CR if present; else -1
        for i, v in enumerate(buf):
            if v in (10, 13):  # LF or CR
                # swallow a paired CRLF/LFCR if present
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

    # --- TX helpers (single writer owns serial TX) ---
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


    def _is_printable(self, b: int) -> bool:
        return 32 <= b <= 126 or b in (9,)  # tab allowed
    
    def _find_eol(self, buf: bytearray) -> int:
        # return index after CRLF/LF/CR if present; else -1
        for i, v in enumerate(buf):
            if v in (10, 13):  # LF or CR
                # swallow a paired CRLF/LFCR if present
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

