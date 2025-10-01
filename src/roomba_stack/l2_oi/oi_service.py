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
DEBUG_RAW_UNKNOWN = True   # dump unknown-leading bytes (HEX + ASCII) for diagnosis
RAW_DUMP_HEAD     = 48     # how many bytes of the buffer front to show

ASCII_SNIFF       = True   # treat printable text as ASCII lines (no resync spam)
ASCII_MAX         = 80     # cap per-line capture


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
        self._running = False
        self._rx_buf = bytearray()
        

    # ------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------

    def open(self) -> None:
        """Open the serial connection and start RX thread."""
        if QUEUE_DISPATCHER:
            self._running = True
            self._dispatcher_thread = threading.Thread(
                target=self._dispatcher_loop, name="dispatcher", daemon=True
            )
            self._dispatcher_thread.start()
            self._port.set_reader(self._on_serial_bytes)
        else:
            self._port.set_reader(self._rx_handler)
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
        was_running = self._running
        self._running = False

        # 2) Best-effort: unregister the reader so no more callbacks arrive.
        try:
            self._port.set_reader(None)
        except Exception:
            # Reader may already be cleared or port may be mid-shutdown.
            log.exception("Error while unregistering serial reader")

        # 3) If we had a running dispatcher, try to join it (but never self-join).
        t = self._dispatcher_thread
        if was_running and t and t.is_alive() and t is not threading.current_thread():
            try:
                t.join(timeout=0.5)
            except Exception:
                log.exception("Error while joining dispatcher thread")

        self._dispatcher_thread = None
        log.info("Dispatcher thread stopped")

        # 4) Close the serial port last to release the device and file descriptors.
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
        log.debug("packet_length(%d) = %d", packet_id, expected_len)

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
        
    def _on_serial_bytes(self, data: bytes) -> None:
        """Callback from PySerialPort with incoming bytes."""
        if not data:
            return
        ok = self._rx_bytes.put(data, timeout=Q_PUT_TIMEOUT)
        if not ok:
            log.warning("[%s] overflow: dropped %d bytes", self._rx_bytes.name(), len(data))

    def _dispatcher_loop(self) -> None:
        """Background thread to dispatch RX bytes"""
        log.info("Dispatcher thread started")
        while self._running:
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
                # look for EOL
                eol = self._find_eol(self._rx_buf)
                cut = eol if eol != -1 else min(len(self._rx_buf), ASCII_MAX)
                line = bytes(self._rx_buf[:cut])
                # log both HEX and ASCII for visibility
                if DEBUG_RAW_UNKNOWN:
                    log.warning("[ASCII] HEX: %s", line.hex())
                    try:
                        preview = self._ascii_preview(line)
                    except Exception:
                        preview = "<ascii preview error>"
                    log.warning("[ASCII] TEXT: %s", preview)
                # consume the captured bytes and continue
                del self._rx_buf[:cut]
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
