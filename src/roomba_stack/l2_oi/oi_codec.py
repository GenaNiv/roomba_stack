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

    Format:
        [137][vel hi][vel lo][radius hi][radius lo]

    Args:
        velocity_mm_s (int): Signed velocity in mm/s.
            Range: [-500, 500] typical, but only int16 width enforced.
        radius_mm (int): Signed radius in mm. 0 = straight.
            Special values per OI spec (e.g., 32768/32767 for in-place turns).

    Returns:
        bytes: Command frame.

    Example:
        >>> encode_drive(200, 0)
        b'\\x89\\x00\\xc8\\x00\\x00'
    """
    return bytes([Opcode.DRIVE]) + _i16_be(velocity_mm_s) + _i16_be(radius_mm)


def encode_drive_direct(right_mm_s: int, left_mm_s: int) -> bytes:
    """
    Build a DRIVE_DIRECT command frame (opcode 145).

    Format:
        [145][right hi][right lo][left hi][left lo]

    Args:
        right_mm_s (int): Right wheel speed, mm/s (int16).
        left_mm_s (int):  Left wheel speed, mm/s (int16).

    Returns:
        bytes: Command frame.

    Example:
        >>> encode_drive_direct(200, -100)
        b'\\x91\\x00\\xc8\\xff\\x9c'
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


def encode_stop_stream() -> bytes:
    """
    Build a STOP_STREAM command frame (opcode 150 with state=0).

    Format:
        [150][0]

    Returns:
        bytes: Command frame.

    Example:
        >>> encode_stop_stream()
        b'\\x96\\x00'
    """
    return bytes([Opcode.STREAM_CTRL, 0])
