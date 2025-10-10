"""
oi_codec.py
===========
Encoders for Roomba Open Interface (OI) commands (TX path).

Design philosophy:
- This module ONLY builds outgoing command frames (bytes to send).
- Opcodes and packet schemas are defined centrally in `oi_protocol.py`.
- Decoding/parsing of sensor data is handled separately in `oi_decode.py`.

Usage:
    from roomba_stack.l2_oi.oi_codec import encode_drive, encode_sensors
    port.write(encode_drive(200, 0))       # Drive forward
    port.write(encode_sensors(7))          # Request distance packet
"""

import struct
from .oi_protocol import Opcode


def encode_zero_data(opcode: int) -> bytes:
    """Return a one-byte command frame for zero-data opcodes."""
    return bytes([opcode])

def encode_reset() -> bytes:   return encode_zero_data(Opcode.RESET)
def encode_start() -> bytes:   return encode_zero_data(Opcode.START)
def encode_control() -> bytes: return encode_zero_data(Opcode.CONTROL)
def encode_safe() -> bytes:    return encode_zero_data(Opcode.SAFE)
def encode_full() -> bytes:    return encode_zero_data(Opcode.FULL)
def encode_power() -> bytes:   return encode_zero_data(Opcode.POWER)
def encode_spot() -> bytes:    return encode_zero_data(Opcode.SPOT)
def encode_clean() -> bytes:   return encode_zero_data(Opcode.CLEAN)
def encode_max() -> bytes:     return encode_zero_data(Opcode.MAX)
def encode_dock() -> bytes:    return encode_zero_data(Opcode.DOCK)
def encode_stop() -> bytes:    return encode_zero_data(Opcode.STOP)

def encode_stream_ctrl(state: int) -> bytes:
    """
    STREAM control (opcode 150).
    state: 0 = pause, 1 = resume
    """
    if state not in (0, 1):
        raise ValueError("state must be 0 (pause) or 1 (resume)")
    return bytes([Opcode.STREAM_CTRL, state])

def encode_pause_stream() -> bytes:
    """
    Build a STOP_STREAM command frame (opcode 150 with state=0).

    Format:
        [150][0]

    Returns:
        bytes: Command frame.

    Example:
        >>> encode_pause_stream()
        b'\\x96\\x00'
    """
    return encode_stream_ctrl(0)

def encode_resume_stream() -> bytes:
    """
    Build a RESUME_STREAM command frame (opcode 150 with state=1).

    Format:
        [150][1]

    Returns:
        bytes: Command frame.

    Example:
        >>> encode_pause_stream()
        b'\\x96\\x01'
    """
    return encode_stream_ctrl(1)


# ============================================================
# Helper: signed 16-bit big-endian
# ============================================================

def _i16_be(value: int) -> bytes:
    """
    Pack a signed 16-bit integer as big-endian.

    Args:
        value (int): Signed integer [-32768, 32767].

    Returns:
        bytes: 2-byte big-endian representation.

    Raises:
        ValueError: If value is out of range.
    """
    if not -32768 <= value <= 32767:
        raise ValueError(f"i16 out of range: {value}")
    return struct.pack(">h", value)


# ============================================================
# Movement Commands
# ============================================================

def encode_drive(velocity_mm_s: int, radius_mm: int) -> bytes:
    """
    Build a DRIVE command frame (opcode 137).

    Purpose
    -------
    Command the robot to drive using an **average velocity** and a **turning radius**.
    This is the high-level drive primitive; use `encode_drive_direct` for per-wheel control.

    Frame Format
    ------------
        [137][Velocity hi][Velocity lo][Radius hi][Radius lo]
    Both Velocity and Radius are **signed 16-bit** (two’s complement), **big-endian**.

    Parameters
    ----------
    velocity_mm_s : int
        Average wheel velocity in mm/s (signed int16).
        Spec range: **-500..+500 mm/s** (negative = backward).
    radius_mm : int
        Turning radius in mm (signed int16). Positive radii turn **left**; negative radii turn **right**.
        Typical spec range: **-2000..+2000 mm**.
        **Special radius values:**
          • **Straight**: `-32768` (0x8000) **or** `32767` (0x7FFF)  
          • **Turn in place (CW)**: `-1` (0xFFFF)  
          • **Turn in place (CCW)**: `+1` (0x0001)
        Note: We pack signed 16-bit values, so pass `-32768` to encode 0x8000 for “straight”.

    Returns
    -------
    bytes
        The encoded command frame ready to write to the serial port.

    Behavior Notes
    --------------
    • Positive velocity + positive radius → forward arc **left**; negative radius → forward arc **right**.
    • Negative velocity mirrors behavior in reverse.
    • The robot accepts this in **SAFE** or **FULL** mode.
    • Internal constraints may prevent exact execution at large radii/high speeds.
    • Speed resolution is ~28.5 mm/s; actual motion quantizes to that step size.

    Examples
    --------
    Drive straight forward @ 200 mm/s (use special “straight” radius):
        >>> encode_drive(200, -32768)     # 0x8000
        b'\\x89\\x00\\xc8\\x80\\x00'      # 89 00 C8 80 00
        >>> encode_drive(200, 32767)      # 0x7FFF
        b'\\x89\\x00\\xc8\\x7f\\xff'      # 89 00 C8 7F FF

    Spec example: reverse @ -200 mm/s with radius 500 mm (arc right/left depends on velocity sign):
        >>> encode_drive(-200, 500)
        b'\\x89\\xff\\x38\\x01\\xf4'      # 89 FF 38 01 F4

    Turn in place clockwise @ 100 mm/s:
        >>> encode_drive(100, -1)
        b'\\x89\\x00\\x64\\xff\\xff'      # 89 00 64 FF FF

    Turn in place counter-clockwise @ 100 mm/s:
        >>> encode_drive(100, 1)
        b'\\x89\\x00\\x64\\x00\\x01'      # 89 00 64 00 01

    Gentle arc left (larger positive radius → gentler left turn):
        >>> encode_drive(250, 1000)
        b'\\x89\\x00\\xfa\\x03\\xe8'      # 89 00 FA 03 E8

    Gentle arc right:
        >>> encode_drive(250, -1000)
        b'\\x89\\x00\\xfa\\xfc\\x18'      # 89 00 FA FC 18
    """
    return bytes([Opcode.DRIVE]) + _i16_be(velocity_mm_s) + _i16_be(radius_mm)



def encode_drive_direct(right_mm_s: int, left_mm_s: int) -> bytes:
    """
    Build a DRIVE_DIRECT command frame (opcode 145).

    Purpose
    -------
    Directly set *each wheel's* linear speed in millimeters per second. This gives
    you fine control over turning/curving by commanding different speeds per side.

    Frame Format
    ------------
        [145][Right hi][Right lo][Left hi][Left lo]
    where Right/Left are **signed 16-bit** (two's complement), **big-endian**.

    Parameters
    ----------
    right_mm_s : int
        Right wheel speed in mm/s (signed int16).
        Typical allowed range on Create 2: **-500..+500** (negative = backward).
    left_mm_s : int
        Left wheel speed in mm/s (signed int16).
        Typical allowed range on Create 2: **-500..+500** (negative = backward).

    Returns
    -------
    bytes
        The encoded command frame ready to send over the serial link.

    Behavior Notes
    --------------
    • Equal speeds (R == L) → straight line (sign decides forward/backward).
    • Opposite equal speeds (R == -L) → spin in place (R>0/L<0 spins right; R<0/L>0 spins left).
    • Different magnitudes/signs → arcs/curves/pivots accordingly.
    • No implicit clipping is performed here; pass values within the robot’s allowed range.
    • The robot must be in a mode that accepts drive commands (SAFE or FULL).

    Examples
    --------
    Forward 200 mm/s both wheels:
        >>> encode_drive_direct(200, 200)
        b'\\x91\\x00\\xc8\\x00\\xc8'   # Hex: 91 00 C8 00 C8

    Stop:
        >>> encode_drive_direct(0, 0)
        b'\\x91\\x00\\x00\\x00\\x00'   # Hex: 91 00 00 00 00

    Gentle arc left (right faster than left):
        >>> encode_drive_direct(250, 100)
        b'\\x91\\x00\\xfa\\x00\\x64'   # Hex: 91 00 FA 00 64

    Spin in place to the right (left backward, right forward):
        >>> encode_drive_direct(200, -200)
        b'\\x91\\x00\\xc8\\xff\\x38'   # Hex: 91 00 C8 FF 38

    Spin in place to the left (right backward, left forward):
        >>> encode_drive_direct(-300, 300)
        b'\\x91\\xfe\\xd4\\x01\\x2c'   # Hex: 91 FE D4 01 2C

    One-wheel pivot (left stationary, right forward):
        >>> encode_drive_direct(200, 0)
        b'\\x91\\x00\\xc8\\x00\\x00'   # Hex: 91 00 C8 00 00

    Mixed forward/backward:
        >>> encode_drive_direct(200, -100)
        b'\\x91\\x00\\xc8\\xff\\x9c'   # Hex: 91 00 C8 FF 9C
    """
    return bytes([Opcode.DRIVE_DIRECT]) + _i16_be(right_mm_s) + _i16_be(left_mm_s)



# ============================================================
# LEDs and Audio
# ============================================================

def encode_leds(
    debris: bool,
    spot: bool,
    dock: bool,
    check_robot: bool,
    power_color: int,
    power_intensity: int,
) -> bytes:
    """
    Build a LEDS command frame (opcode 139).

    Format:
        [139][led_bits][power_color][power_intensity]

    Args:
        debris, spot, dock, check_robot (bool): LED states.
        power_color (int): 0..255 (0 = green, 255 = red).
        power_intensity (int): 0..255 (0 = off, 255 = full bright).

    Returns:
        bytes: Command frame.

    Example:
        >>> encode_leds(False, False, True, True, 128, 255)
        b'\\x8b\\x0c\\x80\\xff'
    """
    if not (0 <= power_color <= 255):
        raise ValueError(f"power_color out of range: {power_color}")
    if not (0 <= power_intensity <= 255):
        raise ValueError(f"power_intensity out of range: {power_intensity}")

    bits = (
        (1 if debris else 0)
        | ((1 if spot else 0) << 1)
        | ((1 if dock else 0) << 2)
        | ((1 if check_robot else 0) << 3)
    )

    return bytes([Opcode.LEDS, bits, power_color, power_intensity])


def encode_song(song_number: int, notes: list[tuple[int, int]]) -> bytes:
    """
    Build a SONG definition frame (opcode 140).

    Args:
        song_number (int): Song ID (0–15).
        notes (list[tuple[int, int]]): List of (note, duration) pairs.
            - note: MIDI note [31–127].
            - duration: 1–255 (1 = 1/64s, 64 = 1s).

    Returns:
        bytes: Command frame.

    Example:
        >>> encode_song(0, [(60, 64)])
        b'\\x8c\\x00\\x01\\x3c\\x40'
    """
    if not (0 <= song_number <= 15):
        raise ValueError("song_number must be 0–15")
    if not (1 <= len(notes) <= 16):
        raise ValueError("notes must contain 1–16 items")

    body = []
    for note, dur in notes:
        if not (31 <= note <= 127):
            raise ValueError(f"note {note} out of range (31–127)")
        if not (1 <= dur <= 255):
            raise ValueError(f"duration {dur} out of range (1–255)")
        body.extend([note, dur])

    return bytes([Opcode.SONG, song_number, len(notes)] + body)


def encode_play(song_number: int) -> bytes:
    """
    Build a PLAY command frame (opcode 141).

    Args:
        song_number (int): ID of previously defined song (0–15).

    Returns:
        bytes: Command frame.

    Example:
        >>> encode_play(0)
        b'\\x8d\\x00'
    """
    if not (0 <= song_number <= 15):
        raise ValueError("song_number must be 0–15")
    return bytes([Opcode.PLAY, song_number])


# ============================================================
# Sensor Query Commands
# ============================================================

def encode_sensors(packet_id: int) -> bytes:
    """
    Build a SENSORS command frame (opcode 142).

    Format:
        [142][packet_id]

    Args:
        packet_id (int): Packet ID (0–58).
            - 0 → all packets.
            - 1–58 → specific packet.

    Returns:
        bytes: Command frame.

    Example:
        >>> encode_sensors(7)
        b'\\x8e\\x07'
    """
    if not (0 <= packet_id <= 58):
        raise ValueError("packet_id must be 0–58")
    return bytes([Opcode.SENSORS, packet_id])


def encode_query_list(packet_ids: list[int]) -> bytes:
    """
    Build a QUERY_LIST command frame (opcode 149).

    Format:
        [149][N][id1][id2]...[idN]

    Args:
        packet_ids (list[int]): List of packet IDs to query once.

    Returns:
        bytes: Command frame.

    Example:
        >>> encode_query_list([7, 13])
        b'\\x95\\x02\\x07\\x0d'
    """
    if not (1 <= len(packet_ids) <= 58):
        raise ValueError("must request at least 1 and at most 58 packets")
    return bytes([Opcode.QUERY_LIST, len(packet_ids)] + packet_ids)


def encode_stream(packet_ids: list[int]) -> bytes:
    """
    Build a STREAM command frame (opcode 148).

    Format:
        [148][N][id1][id2]...[idN]

    Args:
        packet_ids (list[int]): List of packet IDs to stream continuously.

    Returns:
        bytes: Command frame.

    Example:
        >>> encode_stream([7, 13])
        b'\\x94\\x02\\x07\\x0d'
    """
    if not (1 <= len(packet_ids) <= 58):
        raise ValueError("must stream at least 1 and at most 58 packets")
    return bytes([Opcode.STREAM, len(packet_ids)] + packet_ids)

def encode_motors(
    main_on: bool,
    vacuum_on: bool,
    side_on: bool,
    *,
    main_reverse: bool = False,
    side_reverse: bool = False,
) -> bytes:
    """
    Build a MOTORS command frame (opcode 138).

    Format:
        [138][motors_bits]

    Bits (per OI spec):
      - 0..2: on/off for main brush, vacuum, side brush (1 = on @ 100% duty).
      - 3: side brush direction (0 = default CCW, 1 = opposite → CW).
      - 4: main brush direction (0 = default inward, 1 = outward).
    Example (spec): [138][13] = 0b00001101 → main on (default inward), side on (reverse=1 → clockwise).  # noqa
    """
    bits = (
        (1 if main_on else 0)
        | ((1 if vacuum_on else 0) << 1)
        | ((1 if side_on else 0) << 2)
        | ((1 if side_reverse else 0) << 3)
        | ((1 if main_reverse else 0) << 4)
    )
    return bytes([Opcode.MOTORS, bits])

def encode_pwm_motors(main_pwm: int, side_pwm: int, vacuum_pwm: int) -> bytes:
    """
    PWM MOTORS (opcode 144)
    Format: [144][main][side][vacuum]

    Ranges (per OI spec):
      - main  : signed 8-bit  (-127..+127), 0 = stop
      - side  : signed 8-bit  (-127..+127), 0 = stop
      - vacuum: unsigned 8-bit (0..127),    0 = off

    Notes:
      - Negative values reverse direction for main/side.
      - Vacuum supports only 0..127 (no reverse).
    """
    if not (-127 <= int(main_pwm) <= 127):
        raise ValueError("main_pwm must be in -127..127")
    if not (-127 <= int(side_pwm) <= 127):
        raise ValueError("side_pwm must be in -127..127")
    if not (0 <= int(vacuum_pwm) <= 127):
        raise ValueError("vacuum_pwm must be in 0..127")

    def _s8(v: int) -> int:
        # convert signed 8-bit to 0..255 two's complement
        return (int(v) + 256) % 256

    return bytes([
        Opcode.PWM_MOTORS,
        _s8(main_pwm),
        _s8(side_pwm),
        int(vacuum_pwm),
    ])


def encode_drive_pwm(right_pwm: int, left_pwm: int) -> bytes:
    """
    Build a DRIVE_PWM command frame (opcode 146).

    Purpose
    -------
    Drive the wheels using **raw PWM** magnitudes instead of mm/s velocity. This
    bypasses the robot’s speed controller and directly commands motor effort,
    which is useful for low-speed maneuvers, torque-heavy moves, or advanced
    control loops. For kinematic (speed) control, prefer `encode_drive_direct()`.

    Frame Format
    ------------
        [146][Right PWM hi][Right PWM lo][Left PWM hi][Left PWM lo]
    Each PWM is a **signed 16-bit** value (two’s complement), **big-endian**.

    Parameters
    ----------
    right_pwm : int
        Motor effort for the **right** wheel. Valid range **-255..+255**.
        Positive → forward; negative → reverse; 0 → stop.
    left_pwm : int
        Motor effort for the **left** wheel. Valid range **-255..+255**.
        Positive → forward; negative → reverse; 0 → stop.

    Returns
    -------
    bytes
        The encoded command frame ready to send over the serial link.

    Behavior Notes
    --------------
    • PWM commands “motor effort,” not closed-loop speed. Actual wheel speed
      depends on load, battery voltage, floor friction, etc.
    • Magnitudes beyond the platform’s effective range will saturate; this
      function does **no** clipping beyond the spec range check.
    • Equal signs/magnitudes → straight motion; opposite equal magnitudes
      → spin in place; one wheel zero → pivot about the stopped wheel.
    • Robot must be in a mode that accepts drive commands (**SAFE** or **FULL**).

    Examples
    --------
    Forward moderate effort on both wheels:
        >>> encode_drive_pwm(200, 200)
        b'\\x92\\x00\\xc8\\x00\\xc8'      # Hex: 92 00 C8 00 C8

    Stop:
        >>> encode_drive_pwm(0, 0)
        b'\\x92\\x00\\x00\\x00\\x00'      # Hex: 92 00 00 00 00

    Spin in place to the right (R forward, L reverse):
        >>> encode_drive_pwm(200, -200)
        b'\\x92\\x00\\xc8\\xff\\x38'      # Hex: 92 00 C8 FF 38

    Spin in place to the left (R reverse, L forward):
        >>> encode_drive_pwm(-200, 200)
        b'\\x92\\xff\\x38\\x00\\xc8'      # Hex: 92 FF 38 00 C8

    Pivot about the left wheel (left stopped, right forward):
        >>> encode_drive_pwm(180, 0)
        b'\\x92\\x00\\xb4\\x00\\x00'      # Hex: 92 00 B4 00 00

    Mixed forward/backward:
        >>> encode_drive_pwm(200, -100)
        b'\\x92\\x00\\xc8\\xff\\x9c'      # Hex: 92 00 C8 FF 9C
    """
    r = int(right_pwm)
    l = int(left_pwm)
    if not (-255 <= r <= 255):
        raise ValueError("right_pwm must be in -255..255")
    if not (-255 <= l <= 255):
        raise ValueError("left_pwm must be in -255..255")
    return bytes([Opcode.DRIVE_PWM]) + _i16_be(r) + _i16_be(l)

