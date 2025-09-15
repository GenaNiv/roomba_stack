"""
oi_protocol.py
==============
Central definitions of the iRobot Roomba Open Interface (OI) protocol.

This file is the *single source of truth* for:
- OI command opcodes (TX side).
- OI sensor packet schemas (RX side, using `construct`).

Other modules should import from here:
- oi_codec.py → to build outgoing commands.
- oi_decode.py → to parse incoming sensor packets.

Reference: iRobot Roomba 500/600 OI Spec
"""

from enum import IntEnum
from construct import Struct, Int8ub, Int16sb, Int16ub, BitStruct, Flag, Padding

# ============================================================
# OI Command Opcodes (TX)
# ============================================================

class Opcode(IntEnum):
    RESET       = 7     # Soft reset
    START       = 128   # Start OI (Passive mode)
    SAFE        = 131   # Enter Safe mode
    FULL        = 132   # Enter Full mode
    POWER       = 133   # Power down
    DRIVE       = 137   # Drive with velocity + radius
    DOCK        = 143   # Seek dock
    SENSORS     = 142   # Query one sensor packet
    QUERY_LIST  = 149   # Query multiple packets once
    STREAM      = 148   # Start continuous streaming
    STREAM_CTRL = 150   # Pause/resume streaming
    SONG        = 140   # Define song
    PLAY        = 141   # Play song
    # (add more as needed)

# ============================================================
# Sensor Packet Schemas (RX)
# ============================================================

# Helper: 1-bit flags packed into 1 byte
Packet1_Bumps = BitStruct(
    "right_bump" / Flag,
    "left_bump" / Flag,
    "right_wheel_drop" / Flag,
    "left_wheel_drop" / Flag,
    Padding(4),
)
# Example usage:
#   parse_sensor(1, b"\x05") → right_bump=True, left_bump=False, right_wheel_drop=True, left_wheel_drop=False

# Basic 1-byte packets (0/1 or 0–255)
Packet2_Wall          = Struct("wall" / Int8ub)                # Wall sensor
Packet3_CliffLeft     = Struct("cliff_left" / Int8ub)
Packet4_CliffFrontLeft= Struct("cliff_front_left" / Int8ub)
Packet5_CliffFrontRight= Struct("cliff_front_right" / Int8ub)
Packet6_CliffRight    = Struct("cliff_right" / Int8ub)

# Odometry
Packet7_Distance      = Struct("distance_mm" / Int16sb)        # Signed mm since last query
Packet8_Angle         = Struct("angle_deg" / Int16sb)          # Signed degrees since last query

# Charging state
Packet9_ChargingState = Struct("charging_state" / Int8ub)      # 0=Not charging, 1=Reconditioning, 2=Full, etc.

# Voltage & current
Packet10_Voltage      = Struct("voltage_mV" / Int16ub)         # Unsigned mV
Packet11_Current      = Struct("current_mA" / Int16sb)         # Signed mA
Packet12_Temperature  = Struct("temperature_C" / Int8ub)       # Degrees Celsius
Packet13_Charge       = Struct("charge_mAh" / Int16ub)         # Remaining charge
Packet14_Capacity     = Struct("capacity_mAh" / Int16ub)       # Battery capacity

# Internal state
Packet15_WallSignal   = Struct("wall_signal" / Int16ub)        # 0–1023 strength
Packet16_CliffLeftSig = Struct("cliff_left_signal" / Int16ub)
Packet17_CliffFrontLeftSig = Struct("cliff_front_left_signal" / Int16ub)
Packet18_CliffFrontRightSig= Struct("cliff_front_right_signal" / Int16ub)
Packet19_CliffRightSig = Struct("cliff_right_signal" / Int16ub)

# Buttons
Packet21_Buttons      = BitStruct(
    "clean" / Flag,
    "spot" / Flag,
    "dock" / Flag,
    "minute" / Flag,
    "hour" / Flag,
    "day" / Flag,
    "schedule" / Flag,
    "clock" / Flag,
)
# Example: parse_sensor(21, b"\x08") → dock=True, others=False

# Distance/angle continued
Packet22_Distance     = Packet7_Distance
Packet23_Angle        = Packet8_Angle

# Infrared
Packet24_IRCharOmni   = Struct("ir_char_omni" / Int8ub)        # IR remote/dock signals
Packet25_IRCharLeft   = Struct("ir_char_left" / Int8ub)
Packet26_IRCharRight  = Struct("ir_char_right" / Int8ub)

# Charging
Packet34_ChargingSources = Struct("sources" / Int8ub)          # 0=None, 1=Internal, 2=Home base

# Temperature/battery
Packet35_OIMode       = Struct("oi_mode" / Int8ub)             # 0=Off, 1=Passive, 2=Safe, 3=Full
Packet36_SongNumber   = Struct("song_number" / Int8ub)
Packet37_SongPlaying  = Struct("song_playing" / Int8ub)
Packet38_NumStreamPkts= Struct("num_stream_packets" / Int8ub)

# User I/O (Create/Dev only, not standard Roomba)
Packet38_UserDigital  = BitStruct(
    "input0" / Flag,
    "input1" / Flag,
    "input2" / Flag,
    "input3" / Flag,
    "input4" / Flag,
    Padding(3),
)
Packet39_UserAnalog   = Struct("analog_value" / Int16ub)       # 0–1023

# Motor currents
Packet40_ChargingState= Packet9_ChargingState
Packet41_Voltage      = Packet10_Voltage
Packet42_Current      = Packet11_Current
Packet43_Temperature  = Packet12_Temperature
Packet44_Charge       = Packet13_Charge
Packet45_Capacity     = Packet14_Capacity

# Motor currents (individual motors)
Packet46_Overcurrents = BitStruct(
    "drive_left" / Flag,
    "drive_right" / Flag,
    "main_brush" / Flag,
    "side_brush" / Flag,
    "vacuum" / Flag,
    Padding(3),
)
Packet47_DirtDetect   = Struct("dirt_detect" / Int8ub)
Packet48_IRCharLeft   = Packet25_IRCharLeft
Packet49_IRCharRight  = Packet26_IRCharRight

# Analog signals
Packet50_ChargingState= Packet9_ChargingState
Packet51_Voltage      = Packet10_Voltage
Packet52_Current      = Packet11_Current
Packet53_Temperature  = Packet12_Temperature
Packet54_Charge       = Packet13_Charge
Packet55_Capacity     = Packet14_Capacity

# Extension packets (depending on firmware)
Packet56_DigitalOutputs = Struct("digital_outs" / Int8ub)      # Create only
Packet57_LowSideDrivers = Struct("low_side_drivers" / Int8ub)
Packet58_PWMDrivers     = Struct("pwm_drivers" / Int8ub)

# ============================================================
# Registry of all sensor packets
# ============================================================

SENSOR_PACKETS = {
    1: Packet1_Bumps,
    2: Packet2_Wall,
    3: Packet3_CliffLeft,
    4: Packet4_CliffFrontLeft,
    5: Packet5_CliffFrontRight,
    6: Packet6_CliffRight,
    7: Packet7_Distance,
    8: Packet8_Angle,
    9: Packet9_ChargingState,
    10: Packet10_Voltage,
    11: Packet11_Current,
    12: Packet12_Temperature,
    13: Packet13_Charge,
    14: Packet14_Capacity,
    15: Packet15_WallSignal,
    16: Packet16_CliffLeftSig,
    17: Packet17_CliffFrontLeftSig,
    18: Packet18_CliffFrontRightSig,
    19: Packet19_CliffRightSig,
    21: Packet21_Buttons,
    22: Packet22_Distance,
    23: Packet23_Angle,
    24: Packet24_IRCharOmni,
    25: Packet25_IRCharLeft,
    26: Packet26_IRCharRight,
    34: Packet34_ChargingSources,
    35: Packet35_OIMode,
    36: Packet36_SongNumber,
    37: Packet37_SongPlaying,
    38: Packet38_NumStreamPkts,
    39: Packet39_UserAnalog,
    46: Packet46_Overcurrents,
    47: Packet47_DirtDetect,
    48: Packet48_IRCharLeft,
    49: Packet49_IRCharRight,
    50: Packet50_ChargingState,
    51: Packet51_Voltage,
    52: Packet52_Current,
    53: Packet53_Temperature,
    54: Packet54_Charge,
    55: Packet55_Capacity,
    56: Packet56_DigitalOutputs,
    57: Packet57_LowSideDrivers,
    58: Packet58_PWMDrivers,
}

# ============================================================
# Helper function
# ============================================================

def parse_sensor(packet_id: int, raw: bytes):
    """
    Parse a sensor packet by ID into structured data.

    Args:
        packet_id (int): Packet ID (1-58).
        raw (bytes): Raw bytes received from Roomba.

    Returns:
        dict or Container: Parsed result.

    Example:
        raw = b"\\x00\\x78"  # distance = 120 mm
        parse_sensor(7, raw) → Container(distance_mm=120)
    """
    schema = SENSOR_PACKETS.get(packet_id)
    if not schema:
        return {"raw": raw.hex()}
    return schema.parse(raw)
