"""
oi_decode.py
============
RX helpers for the Roomba Open Interface (OI).

This module focuses ONLY on the receive path:
- Decoding raw sensor packets into structured data.
- Handling checksums for stream packets.
- Providing safe wrappers around `oi_protocol.parse_sensor`.

Design philosophy:
- Keep parsing logic separate from TX (`oi_codec.py`).
- High-level consumers (oi_service, CLI) can import from here.
"""

from typing import Union
from .oi_protocol import parse_sensor

def decode_sensor(packet_id: int, raw: bytes) -> Union[dict, object]:
    """
    Decode a single sensor packet into structured data.

    Args:
        packet_id (int): Packet ID (1–58).
        raw (bytes): Raw bytes from Roomba.

    Returns:
        dict or Container: Parsed result.

    Example:
        >>> decode_sensor(7, b"\\x00\\x78")
        Container(distance_mm=120)
    """
    return parse_sensor(packet_id, raw)


def decode_stream_frame(frame: bytes) -> dict:
    """
    Decode a streaming frame (opcode 148 reply).

    Format:
        [19][N][packet ID 1][data...][packet ID 2][data...][checksum]

    Args:
        frame (bytes): Full frame including header + checksum.

    Returns:
        dict: {packet_id: parsed_data}

    Raises:
        ValueError: If checksum fails or frame malformed.

    Example:
        raw = b'\\x13\\x05\\x07\\x00\\x78\\x13\\x01\\x00\\xa3'
        decode_stream_frame(raw)
        → {7: Container(distance_mm=120), 19: Container(...)}
    """
    if not frame or frame[0] != 19:
        raise ValueError("Invalid stream frame: missing header 0x13")

    nbytes = frame[1]
    payload = frame[2 : 2 + nbytes]
    checksum = frame[2 + nbytes]

    # Checksum validation
    if ((sum(frame[0 : 2 + nbytes + 1]) & 0xFF) != 0):
        raise ValueError("Checksum mismatch")

    results = {}
    i = 0
    while i < len(payload):
        packet_id = payload[i]
        i += 1
        # TODO: schema lookup for packet length; for now assume 1 or 2 bytes
        if packet_id in (7, 8, 13, 14, 17, 19):  # 2-byte packets
            raw = payload[i : i + 2]
            i += 2
        else:
            raw = payload[i : i + 1]
            i += 1
        results[packet_id] = parse_sensor(packet_id, raw)

    return results
