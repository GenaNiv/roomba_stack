"""
oi_protocol.py
==============
Central definitions of the iRobot Roomba Open Interface (OI) protocol.

This file is the single source of truth for:
- OI command opcodes (TX side).
- OI sensor packet schemas (RX side, using `construct`).

Other modules should import from here:
- oi_codec.py → to build outgoing commands.
- oi_decode.py → to parse incoming sensor packets.

Reference: iRobot Create 2 / Roomba 600 Open Interface Specification
"""

from enum import IntEnum
from construct import Struct, Int8ub, Int16sb, Int16ub, BitStruct, Flag, Padding, Adapter, Enum

# ============================================================
# OI Command Opcodes (TX)
# ============================================================

class Opcode(IntEnum):
    """Enumeration of Roomba Open Interface command opcodes."""

    RESET        = 7      # Reset robot
    START        = 128    # Start OI (Passive mode)
    BAUD         = 129    # Change baud rate
    CONTROL      = 130    # Enter Control mode (same as Safe)
    SAFE         = 131    # Enter Safe mode
    FULL         = 132    # Enter Full mode
    POWER        = 133    # Power down
    SPOT         = 134    # Spot cleaning
    CLEAN        = 135    # Standard clean
    MAX          = 136    # Max clean (until battery empty)
    DRIVE        = 137    # Drive with velocity + radius
    MOTORS       = 138    # Turn main brush, side brush, vacuum on/off
    LEDS         = 139    # Control LEDs
    SONG         = 140    # Define song
    PLAY         = 141    # Play song
    SENSORS      = 142    # Query one sensor packet
    DOCK         = 143    # Seek dock (same as pressing dock button)
    DRIVE_DIRECT = 145    # Drive wheels independently
    DRIVE_PWM    = 146    # Drive wheels with raw PWM values
    STREAM       = 148    # Start continuous streaming of sensor packets
    QUERY_LIST   = 149    # Query multiple packets once
    STREAM_CTRL  = 150    # Pause/resume streaming
    STOP         = 173    # Stop OI and exit to Off

# ============================================================
# Sensor Packet Schemas (RX)
# ============================================================

# Packet 7: Bumps & Wheel Drops (1 byte; bit0..bit3)
class _P7Adapter(Adapter):
    """Adapter that decodes packet 7 bitfields into named booleans."""
    def _decode(self, obj, ctx, path):
        b = obj & 0x0F
        return {
            "bump_right":       bool(b & 0x01),  # bit0
            "bump_left":        bool(b & 0x02),  # bit1
            "wheel_drop_right": bool(b & 0x04),  # bit2
            "wheel_drop_left":  bool(b & 0x08),  # bit3
        }
    def _encode(self, obj, ctx, path):
        v = 0
        if obj.get("bump_right"):       v |= 0x01
        if obj.get("bump_left"):        v |= 0x02
        if obj.get("wheel_drop_right"): v |= 0x04
        if obj.get("wheel_drop_left"):  v |= 0x08
        return v

Packet7_BumpsWheelDrops = _P7Adapter(Int8ub)
"""Construct schema for packet 7 (bump and wheel-drop status)."""



# Basic 1-byte packets
Packet8_Wall             = Struct("wall" / Int8ub)              # 0 = no wall, 1 = wall detected
Packet9_CliffLeft        = Struct("cliff_left" / Int8ub)        # 0 = no cliff, 1 = cliff
Packet10_CliffFrontLeft  = Struct("cliff_front_left" / Int8ub)
Packet11_CliffFrontRight = Struct("cliff_front_right" / Int8ub)
Packet12_CliffRight      = Struct("cliff_right" / Int8ub)
Packet13_VirtualWall     = Struct("virtual_wall" / Int8ub)      # 0 = none, 1 = detected
class _P14Adapter(Adapter):
    """Adapter that decodes packet 14 bitfields into named booleans."""
    def _decode(self, obj, ctx, path):
        b = obj & 0x1F
        return {
            "side_brush":       bool(b & 0x01),  # bit0 
            "reserved_bit1":    bool(b & 0x02),  # bit1 
            "main_brush":       bool(b & 0x04),  # bit2 
            "right_wheel":      bool(b & 0x08),  # bit3
            "left_wheel":       bool(b & 0x10),  # bit4 
        }
    def _encode(self, obj, ctx, path):
        v = 0
        if obj.get("side_brush"):       v |= 0x01
        if obj.get("reserved_bit1"):    v |= 0x02
        if obj.get("main_brush"):       v |= 0x04
        if obj.get("right_wheel"):      v |= 0x08
        if obj.get("left_wheel"):       v |= 0x10
        return v
Packet14_WheelOvercurrents = _P14Adapter(Int8ub)
"""Construct schema for packet 14 (wheel overcurrents status)."""

Packet15_DirtDetect      = Struct("dirt_detect" / Int8ub)       # dirt sensor value
Packet17_IRCharOmni      = Struct("ir_char_omni" / Int8ub)      # IR opcode seen by omni receiver

class _P18Adapter(Adapter):
    """Adapter that decodes packet 18 bitfields into named booleans."""
    def _decode(self, obj, ctx, path):
        b = obj & 0xFF
        return {
            "clean":        bool(b & 0x01),  # bit0 
            "spot":         bool(b & 0x02),  # bit1 
            "dock":         bool(b & 0x04),  # bit2 
            "minute":       bool(b & 0x08),  # bit3
            "hour":         bool(b & 0x10),  # bit4
            "day":          bool(b & 0x20),  # bit5
            "schedule":     bool(b & 0x40),  # bit6
            "clock":        bool(b & 0x80),  # bit7 
        }
    def _encode(self, obj, ctx, path):
        v = 0
        if obj.get("clean"):        v |= 0x01
        if obj.get("spot"):         v |= 0x02
        if obj.get("dock"):         v |= 0x04
        if obj.get("minute"):       v |= 0x08
        if obj.get("hour"):         v |= 0x10
        if obj.get("day"):          v |= 0x20
        if obj.get("schedule"):     v |= 0x40
        if obj.get("clock"):        v |= 0x80
        return v
Packet18_Buttons = _P18Adapter(Int8ub)
"""Construct schema for packet 18 (The state of the Roomba buttons)."""

# Odometry
Packet19_Distance        = Struct("distance_mm" / Int16sb)      # Signed mm since last request
Packet20_Angle           = Struct("angle_deg" / Int16sb)        # Signed degrees since last request

# Battery & charging
Packet21_ChargingState   = Struct("charging_state" / Int8ub)    # 0–5
Packet22_Voltage         = Struct("voltage_mV" / Int16ub)       # Unsigned mV
Packet23_Current         = Struct("current_mA" / Int16sb)       # Signed mA (+in, -out)
Packet24_Temperature     = Struct("temperature_C" / Int8ub)     # Signed °C
Packet25_Charge          = Struct("charge_mAh" / Int16ub)       # Unsigned mAh
Packet26_Capacity        = Struct("capacity_mAh" / Int16ub)     # Unsigned mAh

# Infrared & buttons already above (packets 17, 18)

# Signal strengths
Packet27_WallSignal          = Struct("wall_signal" / Int16ub)        # 0–1023
Packet28_CliffLeftSignal     = Struct("cliff_left_signal" / Int16ub)  # 0–4095
Packet29_CliffFrontLeftSignal= Struct("cliff_front_left_signal" / Int16ub)
Packet30_CliffFrontRightSignal=Struct("cliff_front_right_signal" / Int16ub)
Packet31_CliffRightSignal    = Struct("cliff_right_signal" / Int16ub)

# Charging sources & mode
Packet34_ChargingSources = Struct("charging_sources" / Int8ub)  # bits: home base, internal

# ============================================================
# OI Command Opcodes (TX)
# Modes overview (spec):
# - OFF (0): OI inactive. Most commands ignored. After power/reset, send START.
# - PASSIVE (1): After START, robot accepts most read-only commands (sensors, song),
#   but no actuators (drive, motors). Transition into PASSIVE happens on START.
# - SAFE (2): After SAFE, robot accepts drive/actuator commands with safety constraints.
# - FULL (3): After FULL, robot accepts drive/actuator commands without safety limits.
#
# Transitions (spec highlights):
# - START (128) -> PASSIVE
# - SAFE (131)  -> SAFE (from PASSIVE or FULL)
# - FULL (132)  -> FULL (from PASSIVE or SAFE)
# - POWER/RESET may drop to OFF; send START again to re-enter PASSIVE.
# ============================================================
Packet35_OIMode = Struct(                                       # The current OI mode
    "oi_mode" / Enum(Int8ub,
        OFF=0,        # 0
        PASSIVE=1,    # 1
        SAFE=2,       # 2
        FULL=3,       # 3
    )
)

Packet36_SongNumber      = Struct("song_number" / Int8ub)
Packet37_SongPlaying     = Struct("song_playing" / Int8ub)      # 0 = no, 1 = yes
Packet38_NumStreamPkts   = Struct("num_stream_packets" / Int8ub)

# Requested drive parameters
Packet39_RequestedVelocity = Struct("requested_velocity" / Int16sb)
Packet40_RequestedRadius   = Struct("requested_radius" / Int16sb)
Packet41_RequestedRightVel = Struct("requested_right_velocity" / Int16sb)
Packet42_RequestedLeftVel  = Struct("requested_left_velocity" / Int16sb)

# Encoders
Packet43_LeftEncoderCounts  = Struct("left_encoder_counts" / Int16sb)   # counts, signed rollover
Packet44_RightEncoderCounts = Struct("right_encoder_counts" / Int16sb)

# Light bumper group
Packet45_LightBumper         = Struct("light_bumper" / Int8ub)         # bitmask, 0–127
Packet46_LightBumpLeft       = Struct("light_bump_left" / Int16ub)
Packet47_LightBumpFrontLeft  = Struct("light_bump_front_left" / Int16ub)
Packet48_LightBumpCenterLeft = Struct("light_bump_center_left" / Int16ub)
Packet49_LightBumpCenterRight= Struct("light_bump_center_right" / Int16ub)
Packet50_LightBumpFrontRight = Struct("light_bump_front_right" / Int16ub)
Packet51_LightBumpRight      = Struct("light_bump_right" / Int16ub)

# Infrared per-side receivers
Packet52_IRCharLeft  = Struct("ir_char_left" / Int8ub)
Packet53_IRCharRight = Struct("ir_char_right" / Int8ub)

# Motor currents
Packet54_LeftMotorCurrent   = Struct("left_motor_current" / Int16sb)   # signed mA
Packet55_RightMotorCurrent  = Struct("right_motor_current" / Int16sb)
Packet56_MainBrushCurrent   = Struct("main_brush_current" / Int16sb)
Packet57_SideBrushCurrent   = Struct("side_brush_current" / Int16sb)

# Stasis sensor
Packet58_Stasis             = Struct("stasis" / Int8ub)  # 0–3 (0 = no progress, 1 = forward, 2 = dirty)

# ============================================================
# Registry of all sensor packets
# ============================================================

SENSOR_PACKETS = {
    7:  Packet7_BumpsWheelDrops,
    8:  Packet8_Wall,
    9:  Packet9_CliffLeft,
    10: Packet10_CliffFrontLeft,
    11: Packet11_CliffFrontRight,
    12: Packet12_CliffRight,
    13: Packet13_VirtualWall,
    14: Packet14_WheelOvercurrents,
    15: Packet15_DirtDetect,
    17: Packet17_IRCharOmni,
    18: Packet18_Buttons,
    19: Packet19_Distance,
    20: Packet20_Angle,
    21: Packet21_ChargingState,
    22: Packet22_Voltage,
    23: Packet23_Current,
    24: Packet24_Temperature,
    25: Packet25_Charge,
    26: Packet26_Capacity,
    27: Packet27_WallSignal,
    28: Packet28_CliffLeftSignal,
    29: Packet29_CliffFrontLeftSignal,
    30: Packet30_CliffFrontRightSignal,
    31: Packet31_CliffRightSignal,
    34: Packet34_ChargingSources,
    35: Packet35_OIMode,
    36: Packet36_SongNumber,
    37: Packet37_SongPlaying,
    38: Packet38_NumStreamPkts,
    39: Packet39_RequestedVelocity,
    40: Packet40_RequestedRadius,
    41: Packet41_RequestedRightVel,
    42: Packet42_RequestedLeftVel,
    43: Packet43_LeftEncoderCounts,
    44: Packet44_RightEncoderCounts,
    45: Packet45_LightBumper,
    46: Packet46_LightBumpLeft,
    47: Packet47_LightBumpFrontLeft,
    48: Packet48_LightBumpCenterLeft,
    49: Packet49_LightBumpCenterRight,
    50: Packet50_LightBumpFrontRight,
    51: Packet51_LightBumpRight,
    52: Packet52_IRCharLeft,
    53: Packet53_IRCharRight,
    54: Packet54_LeftMotorCurrent,
    55: Packet55_RightMotorCurrent,
    56: Packet56_MainBrushCurrent,
    57: Packet57_SideBrushCurrent,
    58: Packet58_Stasis,
}
"""Mapping of sensor packet IDs to their Construct schemas."""

# ============================================================
# Helper functions
# ============================================================

def parse_sensor(packet_id: int, raw: bytes):
    """
    Parse a sensor packet by ID into structured data.

    Args:
        packet_id: Numeric packet identifier defined by the OI spec.
        raw: Raw payload bytes (no leading packet ID).

    Returns:
        Parsed value returned by the Construct schema or a hex dump when
        the packet ID is unknown.
    """
    schema = SENSOR_PACKETS.get(packet_id)
    if not schema:
        return {"raw": raw.hex()}
    return schema.parse(raw)

def build_sensor(packet_id: int, payload: dict) -> bytes:
    """
    Build raw payload bytes for a sensor packet.

    Args:
        packet_id: Numeric packet identifier defined by the OI spec.
        payload: Mapping of field names to values compatible with the schema.

    Returns:
        Raw payload bytes (no leading packet ID).
    """
    schema = SENSOR_PACKETS.get(packet_id)
    if not schema:
        raise ValueError(f"Unknown or unsupported packet ID {packet_id}")
    try:
        return schema.build(payload)
    except Exception as exc:  # pragma: no cover - construct raises many types
        raise ValueError(
            f"Failed to build sensor packet {packet_id} with payload {payload}: {exc}"
        ) from exc

def packet_length(packet_id: int) -> int:
    """
    Return expected data length (in bytes) for a given sensor packet ID.

    Args:
        packet_id: Numeric packet identifier defined by the OI spec.

    Returns:
        Length in bytes for the packet's payload or 0 when the packet is
        unknown or variable length.
    """
    schema = SENSOR_PACKETS.get(packet_id)
    if not schema:
        return 0
    try:
        return schema.sizeof()
    except Exception:
        return 0
