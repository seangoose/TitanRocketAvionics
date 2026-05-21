# =============================================================================
#  TRES Titan Ground Station — protocol.py
#  Packet protocol constants — must stay in sync with Teensy_TRES_v7.ino
# =============================================================================

import struct

# -----------------------------------------------------------------------------
#  FRAME HEADER
# -----------------------------------------------------------------------------
MAGIC_0         = 0x54   # 'T'
MAGIC_1         = 0x52   # 'R'
HEADER          = bytes([MAGIC_0, MAGIC_1])

# Packet type bytes (byte [3] of every frame)
PKT_DATA        = 0xDA   # Rocket → Ground  telemetry
PKT_CMD         = 0xC0   # Ground → Rocket  command
PKT_ACK         = 0xAC   # Rocket → Ground  ACK
PKT_STATUS      = 0x57   # Rocket → Ground  full system state

# ACK result codes
ACK_OK          = 0x00
ACK_REJECTED    = 0x01

# -----------------------------------------------------------------------------
#  UPLINK COMMAND BYTES
# -----------------------------------------------------------------------------
CMD_VIDEO_ON        = 0x01
CMD_VIDEO_OFF       = 0x02
CMD_TELEM_HIGH      = 0x03
CMD_TELEM_LOW       = 0x04
CMD_FORCE_PACKET    = 0x05
CMD_FIRE_SOLENOID   = 0x06
CMD_TEST_MODE_ON    = 0x07   # Legacy — maps to CMD_MODE_TEST on Teensy
CMD_TEST_MODE_OFF   = 0x08   # Legacy — maps to CMD_MODE_PAD on Teensy
CMD_SET_LORA_FREQ   = 0x09   # Extended: PLEN=4, float MHz
CMD_SET_VTX_FREQ    = 0x0A   # Extended: PLEN=2, uint16 MHz
CMD_GET_STATUS      = 0x0B
CMD_MODE_TEST       = 0x0C   # → GM_TEST_IDLE
CMD_MODE_PAD        = 0x0D   # → GM_PAD_IDLE
CMD_MODE_LAUNCH     = 0x0E   # → GM_LAUNCH_READY
CMD_SET_VTX_POWER   = 0x0F   # Extended: PLEN=1, uint8 SA index 0–3
CMD_CAM_RECORD_ON   = 0x10   # Start RunCam internal recording
CMD_CAM_RECORD_OFF  = 0x11   # Stop RunCam internal recording
CMD_FULL_SYS_TEST   = 0x12   # Full system test — VTX + camera + telem + solenoid (S2)

CMD_NAMES = {
    CMD_VIDEO_ON:       "VIDEO_ON",
    CMD_VIDEO_OFF:      "VIDEO_OFF",
    CMD_TELEM_HIGH:     "TELEM_HIGH",
    CMD_TELEM_LOW:      "TELEM_LOW",
    CMD_FORCE_PACKET:   "FORCE_PACKET",
    CMD_FIRE_SOLENOID:  "FIRE_SOLENOID",
    CMD_TEST_MODE_ON:   "TEST_MODE_ON (legacy)",
    CMD_TEST_MODE_OFF:  "TEST_MODE_OFF (legacy)",
    CMD_SET_LORA_FREQ:  "SET_LORA_FREQ",
    CMD_SET_VTX_FREQ:   "SET_VTX_FREQ",
    CMD_GET_STATUS:     "GET_STATUS",
    CMD_MODE_TEST:      "MODE_TEST",
    CMD_MODE_PAD:       "MODE_PAD",
    CMD_MODE_LAUNCH:    "MODE_LAUNCH",
    CMD_SET_VTX_POWER:  "SET_VTX_POWER",
    CMD_CAM_RECORD_ON:  "CAM_RECORD_ON",
    CMD_CAM_RECORD_OFF: "CAM_RECORD_OFF",
    CMD_FULL_SYS_TEST:  "FULL_SYS_TEST",
}

# -----------------------------------------------------------------------------
#  GROUND MODE VALUES  (matches GroundMode enum in Teensy)
# -----------------------------------------------------------------------------
GM_TEST_IDLE    = 0
GM_PAD_IDLE     = 1
GM_LAUNCH_READY = 2

GROUND_MODE_NAMES = {
    GM_TEST_IDLE:    "TEST_IDLE",
    GM_PAD_IDLE:     "PAD_IDLE",
    GM_LAUNCH_READY: "LAUNCH_READY",
}

# -----------------------------------------------------------------------------
#  FLIGHT STATE NAMES
# -----------------------------------------------------------------------------
STATE_NAMES_S1 = {
    0: "PAD",        1: "BOOST",      2: "COAST",
    3: "SEPARATION", 4: "DESCENT",    5: "LANDED",
}

STATE_NAMES_S2 = {
    0:  "PAD",             1:  "BOOST_S1",      2:  "COAST_S1",
    3:  "SEPARATION",      4:  "AIRSTART_WAIT",  5:  "NO_AIRSTART",
    6:  "BOOST_S2",        7:  "COAST_S2",       8:  "APOGEE",
    9:  "DESCENT",         10: "LANDED",
}

FAULT_NAMES = {
    0x01: "BMP",  0x02: "LSM",  0x04: "ADXL",
    0x08: "LORA", 0x10: "SD",   0x20: "OSD",  0x40: "CAM",
}

VTX_POWER_LABELS = {0: "200mW", 1: "400mW", 2: "800mW", 3: "1000mW"}

# -----------------------------------------------------------------------------
#  FLIGHT DATA PACKET  (downlink, PKT_DATA)
#  Format '<IffffffffhBBBB' = 42 bytes packed
#  Must exactly match FlightPacket struct in Teensy_TRES_v7.ino.
# -----------------------------------------------------------------------------
FLIGHT_PACKET_FMT  = "<IffffffffhBBBB"
FLIGHT_PACKET_SIZE = struct.calcsize(FLIGHT_PACKET_FMT)   # 42 bytes

FLIGHT_PACKET_FIELDS = [
    "timestamp_ms", "altitude_ft", "velocity_fps",
    "accel_thrust", "accel_lateral",
    "gyro_roll", "gyro_pitch", "gyro_yaw",
    "temperature_c", "rssi_dbm",
    "state", "stage", "fault_flags", "crc8"
]

# -----------------------------------------------------------------------------
#  STATUS PACKET  (downlink, PKT_STATUS)
#  Format '<BBBBBBfHBBBBBBB' = 19 bytes packed
#  Must exactly match StatusPacket struct in Teensy_TRES_v7.ino.
#
#  Fields in order:
#    stage, flight_state, fault_flags, test_mode, video_enabled,
#    telem_rate_hz (5 = high rate / LAUNCH_READY, 1 = low rate / PAD_IDLE), lora_freq_mhz, vtx_freq_mhz,
#    vtx_power_index, vtx_flight_power, ground_mode, cam_recording,
#    last_cmd, last_cmd_result, crc8
# -----------------------------------------------------------------------------
STATUS_PACKET_FMT  = "<BBBBBBfHBBBBBBB"
STATUS_PACKET_SIZE = struct.calcsize(STATUS_PACKET_FMT)   # 19 bytes

STATUS_PACKET_FIELDS = [
    "stage", "flight_state", "fault_flags",
    "test_mode", "video_enabled", "telem_rate_hz",
    "lora_freq_mhz", "vtx_freq_mhz", "vtx_power_index",
    "vtx_flight_power", "ground_mode", "cam_recording",
    "last_cmd", "last_cmd_result", "crc8"
]

# Full frame sizes including 4-byte TRES header
FULL_DATA_FRAME_SIZE   = 4 + FLIGHT_PACKET_SIZE    # 46 bytes
FULL_STATUS_FRAME_SIZE = 4 + STATUS_PACKET_SIZE    # 23 bytes
ACK_FRAME_SIZE         = 7


# -----------------------------------------------------------------------------
#  CRC-8 (XOR)  — matches Teensy calcCRC8()
# -----------------------------------------------------------------------------
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc & 0xFF


# -----------------------------------------------------------------------------
#  FRAME BUILDERS  (ground → rocket uplink)
# -----------------------------------------------------------------------------

def build_standard_frame(stage: int, cmd: int) -> bytes:
    """Build a standard 6-byte uplink command frame."""
    frame = bytearray([MAGIC_0, MAGIC_1, stage, PKT_CMD, cmd, 0x00])
    frame[5] = crc8(frame[:5])
    return bytes(frame)


def build_set_lora_freq_frame(stage: int, freq_mhz: float) -> bytes:
    """Extended CMD_SET_LORA_FREQ. Payload = 4-byte LE float."""
    payload = struct.pack("<f", freq_mhz)
    frame   = bytearray([MAGIC_0, MAGIC_1, stage, PKT_CMD,
                         CMD_SET_LORA_FREQ, len(payload)]) + bytearray(payload)
    frame.append(crc8(frame))
    return bytes(frame)


def build_set_vtx_freq_frame(stage: int, freq_mhz: int) -> bytes:
    """Extended CMD_SET_VTX_FREQ. Payload = 2-byte LE uint16."""
    payload = struct.pack("<H", freq_mhz)
    frame   = bytearray([MAGIC_0, MAGIC_1, stage, PKT_CMD,
                         CMD_SET_VTX_FREQ, len(payload)]) + bytearray(payload)
    frame.append(crc8(frame))
    return bytes(frame)


def build_set_vtx_power_frame(stage: int, level: int) -> bytes:
    """Extended CMD_SET_VTX_POWER. Payload = 1-byte SA index 0-3."""
    if not 0 <= level <= 3:
        raise ValueError(f"VTX power level must be 0-3, got {level}")
    payload = struct.pack("<B", level)
    frame   = bytearray([MAGIC_0, MAGIC_1, stage, PKT_CMD,
                         CMD_SET_VTX_POWER, len(payload)]) + bytearray(payload)
    frame.append(crc8(frame))
    return bytes(frame)


def build_full_sys_test_frame(stage: int) -> bytes:
    """Build a standard CMD_FULL_SYS_TEST frame."""
    return build_standard_frame(stage, CMD_FULL_SYS_TEST)


# M0 retune escape sequence — sent by RadioWorker directly to the M0 serial port
# NOT a LoRa frame — intercepted by M0 firmware before transmission
def build_m0_retune(freq_mhz: float) -> bytes:
    """Build the 6-byte M0 retune escape: [0xFF][0xFE][float_LE_4bytes]"""
    return b'\xff\xfe' + struct.pack("<f", freq_mhz)


# -----------------------------------------------------------------------------
#  FRAME PARSERS  (rocket → ground downlink)
# -----------------------------------------------------------------------------

def parse_flight_packet(payload: bytes) -> dict | None:
    if len(payload) != FLIGHT_PACKET_SIZE:
        return None
    if payload[-1] != crc8(payload[:-1]):
        return None
    return dict(zip(FLIGHT_PACKET_FIELDS,
                    struct.unpack(FLIGHT_PACKET_FMT, payload)))


def parse_status_packet(payload: bytes) -> dict | None:
    if len(payload) != STATUS_PACKET_SIZE:
        return None
    if payload[-1] != crc8(payload[:-1]):
        return None
    return dict(zip(STATUS_PACKET_FIELDS,
                    struct.unpack(STATUS_PACKET_FMT, payload)))


def parse_ack_frame(frame: bytes) -> dict | None:
    if len(frame) != ACK_FRAME_SIZE:
        return None
    if frame[0] != MAGIC_0 or frame[1] != MAGIC_1:
        return None
    if frame[3] != PKT_ACK:
        return None
    if crc8(frame[:6]) != frame[6]:
        return None
    return {"cmd": frame[4], "result": frame[5], "stage": frame[2]}


def fault_string(fault_flags: int) -> str:
    if fault_flags == 0:
        return "None"
    return " | ".join(name for bit, name in FAULT_NAMES.items()
                      if fault_flags & bit)


def state_name(stage: int, state: int) -> str:
    table = STATE_NAMES_S1 if stage == 1 else STATE_NAMES_S2
    return table.get(state, f"UNKNOWN({state})")


def ground_mode_name(mode: int) -> str:
    return GROUND_MODE_NAMES.get(mode, f"UNKNOWN({mode})")


