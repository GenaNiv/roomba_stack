from __future__ import annotations

from typing import Optional
import threading
import logging
import time

import serial  # provided by pyserial
from serial import SerialException
from .serial_port import SerialPort, SerialError, BytesCallback

DEFAULT_READ_CHUNK = 256  # put near top of file if you prefer

log = logging.getLogger(__name__)

class PySerialPort(SerialPort):
    """
    This class is a concrete implementation of the SerialPort interface
    using the pyserial library. It encapsulates serial communication
    through a specified device and provides thread-safe write operations.

    Current capabilities:
    - Open and close a serial connection.
    - Check if the connection is open.
    - Write bytes to the port in a thread-safe manner.
    - Register a reader callback background reading.
    """

    def __init__(self, device: str, baudrate: int = 115200, timeout: float = 0.05) -> None:
        self._device = device
        self._baudrate = baudrate
        self._timeout = timeout

        self._ser: Optional[serial.Serial] = None
        self._on_bytes: Optional[BytesCallback] = None
        self._write_lock = threading.Lock()

        self._reader_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()

    def open(self) -> None:
        """
        Open the serial connection and start the background reader thread.

        This method is responsible for establishing the underlying pyserial
        connection and initializing the asynchronous reader. It performs the
        following steps:

        1. Attempt to open the serial device using the configured device path,
        baud rate, and timeout.
        - If the operation fails, a SerialError is raised to ensure
            higher layers do not need to handle raw pyserial exceptions.

        2. Clear the stop flag to indicate that the reader thread should run.

        3. Create and start a dedicated background thread (`_reader_thread`)
        that continuously monitors the serial port and dispatches incoming
        bytes to the registered callback (if any).

        Raises:
            SerialError: If opening the device fails due to a SerialException.
        """

        try:
            # Establish a pyserial connection
            self._ser = serial.Serial(
                self._device,
                self._baudrate,
                timeout=self._timeout
            )
        except SerialException as e:
            raise SerialError(f"Failed to open {self._device}: {e}") from e

        # Prepare and start the reader thread
        self._stop_flag.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="rx-reader",
            daemon=True
        )
        self._reader_thread.start()


    def close(self) -> None:
        """
        Close the serial connection and stop the background reader thread.

        This method ensures a clean shutdown of the serial driver by:
        1. Signaling the reader thread to stop (via the stop flag).
        2. Waiting for the reader thread to terminate gracefully.
        - If the thread does not stop within the timeout, a SerialError is raised.
        3. Closing the underlying serial port safely.
        4. Resetting all internal references to None.

        Raises:
            SerialError: If the reader thread fails to stop or if closing the
                        serial port raises a SerialException.
        """

        # Signal the reader thread to stop and wait for it to exit
        self._stop_flag.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
            if self._reader_thread.is_alive():
                raise SerialError("Failed to stop reader thread")
        self._reader_thread = None

        # Close the underlying serial port if still open
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except SerialException as e:
                raise SerialError(f"Failed to close {self._device}: {e}") from e
        self._ser = None


    def is_open(self) -> bool:
        """
        Check if the serial connection is currently open.

        This method verifies both that the underlying pyserial object exists
        and that it reports itself as open.

        Returns:
            bool: True if the serial connection is active and ready for
                communication; False otherwise.
        """
        return bool(self._ser and self._ser.is_open)

    def write(self, data: bytes) -> None:
        """
        Write raw bytes to the serial port in a thread-safe manner.

        This method ensures exclusive access to the serial connection during
        the write operation by using a lock. This prevents interleaving of
        bytes when multiple threads attempt to write simultaneously.

        Workflow:
            1. Verify that the serial port is open; raise SerialError if not.
            2. Acquire the write lock to ensure thread-safe access.
            3. Write the provided data bytes to the serial port.
            4. Flush the port to guarantee immediate transmission.
            5. Release the lock automatically upon exit from the 'with' block.

        Args:
            data (bytes): The data to be written to the serial port.

        Raises:
            SerialError: If the port is not open, or if the underlying pyserial
                        write/flush operations fail.
        """
        if not self._ser or not self._ser.is_open:
            raise SerialError("Port not open")
        try:
            with self._write_lock:
                self._ser.write(data)
                self._ser.flush()
        except SerialException as e:
            raise SerialError(f"Write failed on {self._device}: {e}") from e


    def set_reader(self, on_bytes: Optional[BytesCallback]) -> None:
        """
        Register or remove the callback function for incoming bytes.

        The callback, if provided, will be invoked asynchronously from the
        background reader thread whenever data is read from the serial port.

        Args:
            on_bytes (Optional[Callable[[bytes], None]]):
                A function that accepts a 'bytes' object. It will be called
                whenever new data arrives. If None is provided, any previously
                registered callback is removed.

        Notes:
            - The callback should be lightweight and non-blocking. Heavy
            processing should be offloaded to another thread or queue.
            - Exceptions raised in the callback are caught and ignored in order
            to keep the reader thread alive.
        """
        self._on_bytes = on_bytes


    def _reader_loop(self) -> None:
        """
        Background thread loop that continuously reads from the serial port.

        This method runs in its own daemon thread and performs the following:
            1. Continuously attempt to read up to DEFAULT_READ_CHUNK bytes.
            2. If bytes are received and a callback is registered, invoke the
            callback with the received data.
            3. If a SerialException occurs, exit the loop to avoid blocking.
            4. Stop gracefully when the stop flag is set by close().

        Notes:
            - The method catches and discards exceptions raised by the callback
            to ensure the reader thread remains alive.
            - An empty read (b"") typically indicates a timeout and is ignored.

        This is a private helper method and should never be called directly.
        It is started automatically when the port is opened.
        """
        log.info("RX reader started")
        ser = self._ser
        if not ser:
            return
        while not self._stop_flag.is_set():
            try:
                chunk = ser.read(DEFAULT_READ_CHUNK)  # returns b"" on timeout
            except SerialException:
                break  # port error => exit thread
            if chunk and self._on_bytes:
                try:
                    self._on_bytes(chunk)
                except Exception:
                    # swallow callback exceptions so the reader stays alive
                    pass

    def set_rts_low(self) -> None:
        """
        Force RTS (Request To Send) LOW.
        On Roomba, this is wired to the BRC pin:
        - Pulling it LOW engages the BRC signal (used for wake/sleep control).
        - This is a hardware-level action, not an OI command.
        """
        if not self._ser or not self._ser.is_open:
            raise SerialError("Port not open")
        self._ser.rts = False
        print("[INFO] RTS forced LOW (BRC active)")
        
    def set_rts_high(self) -> None:
        """
        Force RTS (Request To Send) HIGH.
        On Roomba, this releases the BRC pin:
        - Allows normal auto-sleep/auto-wake behavior.
        - This is a hardware-level action, not an OI command.
        """
        if not self._ser or not self._ser.is_open:
            raise SerialError("Port not open")
        self._ser.rts = True
        print("[INFO] RTS forced HIGH (BRC inactive)")

    def pulse_wakeup(self, duration: float = 0.5) -> None:
        """
        Send a wakeup pulse via RTS/BRC pin.
        Sequence:
        - Drive RTS LOW for 'duration' seconds (default 0.5s).
        - Return RTS HIGH afterwards.
        
        On Roomba:
        - If the robot is in deep off, this simulates pressing CLEAN.
        - After waking, you must send START (0x80) to enter OI mode.
        """
        if not self._ser or not self._ser.is_open:
            raise SerialError("Port not open")
        print(f"[INFO] Sending wakeup pulse ({duration:.1f}s LOW)...")
        self._ser.rts = False
        time.sleep(duration)
        self._ser.rts = True
        print("[INFO] Wakeup pulse complete. Send START (0x80) next.")
