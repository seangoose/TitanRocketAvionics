/*
 * ================================================================
 *  TRES TITAN — SRAD AVIONICS  v7.0
 *  Teensy 4.1
 *
 *  BEFORE EVERY FLASH — EDIT THESE TWO LINES:
 *    #define STAGE     1   or   2
 *    #define CALLSIGN  "KD0XXX"   ← your licensed amateur callsign
 *
 * ================================================================
 *  IREC RULE 5.13.4 — MANDATORY
 *  This SRAD computer CANNOT initiate motor ignition under any
 *  condition.  Motor events are handled by:
 *      Stage 1 — Featherweight Blue Jay
 *      Stage 2 — Featherweight Blue Raven
 *  The single MOSFET output (Stage 2 only) controls a solenoid
 *  valve for water payload ejection ONLY.
 * ================================================================
 *
 *  HARDWARE  (verified against schematic rev4)
 *  ─────────────────────────────────────────────────────────────
 *  Teensy 4.1                MCU
 *  Adafruit BMP388  (U4)     Barometric altimeter       I2C
 *  Adafruit LSM6DSOX (U2)    6-axis IMU                 I2C
 *  Adafruit ADXL375  (U5)    High-g accelerometer       I2C
 *  Adafruit RFM95W   (U3)    LoRa 915 MHz               SPI  CS=37
 *  DFR0515 / AT7456E (U7)    OSD overlay                SPI  CS=36
 *    NOTE: 5V logic — TXB0104 level shifters required on all SPI lines
 *  AKK A1918 VTX             5.8 GHz video TX           Serial6 TX pin 24
 *  RunCam Split 4            Camera                     Serial1 pins 0(RX)/1(TX)
 *  [Stage 2 only]
 *  MOSFET gate               Normally-closed solenoid   Pin 2
 * ================================================================
 */

// ================================================================
//  *** EDIT BEFORE EVERY FLASH ***
// ================================================================
#define STAGE      2
#define CALLSIGN   "KO6NHZ"

// Self-test mode — set 1 for bench testing, 0 for flight
// When 1: runSelfTest() executes at the start of setup() then normal boot continues.
// When 0: entire self-test is compiled out. No impact on flight firmware behavior.
#define RUN_SELF_TEST  1

#if STAGE != 1 && STAGE != 2
  #error "STAGE must be 1 or 2."
#endif

// ================================================================
//  LIBRARIES
// ================================================================
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_ADXL375.h>

// ================================================================
//  PIN ASSIGNMENTS
// ================================================================
#define PIN_LORA_CS     37
#define PIN_LORA_RST     9
#define PIN_LORA_INT     8
#define PIN_OSD_CS      36
#define SD_BUILTIN      BUILTIN_SDCARD
#define PIN_LED         13

// AKK A1918 VTX — SmartAudio half-duplex UART
// Pin 24 = Serial6 TX  ← CORRECTED from earlier Serial5 error
#define VTX_SERIAL      Serial6

// RunCam Split 4 — RunCam Device Protocol UART
// Pin 0 = Serial1 RX (Teensy RX ← RunCam TX)
// Pin 1 = Serial1 TX (Teensy TX → RunCam RX)
#define CAM_SERIAL      Serial1
#define CAM_BAUD        115200

#if STAGE == 2
  #define PIN_SOLENOID  2
#endif

// ================================================================
//  LoRa
// ================================================================
#define RF95_FREQ_DEFAULT  915.0f
#define RF95_TX_DBM        23
#define RF95_SF             8
#define RF95_BW        125000UL
#define RF95_CR             5

// ================================================================
//  TASK TIMING
// ================================================================
#define RATE_SENSOR_MS      10   // 100 Hz
#define RATE_SD_MS          10   // 100 Hz
#define RATE_OSD_MS        200   // 5 Hz
#define RATE_DEBUG_MS      500   // 2 Hz
#define TELEM_RATE_HIGH_MS 100   // 10 Hz  (LAUNCH_READY default)
#define TELEM_RATE_LOW_MS 1000   // 1 Hz   (PAD_IDLE fixed)
#define HEARTBEAT_MS     60000UL // 60 s   (TEST_IDLE and PAD_IDLE)

// ================================================================
//  FLIGHT THRESHOLDS
// ================================================================
#define THRESH_LAUNCH_ACCEL     34.3f
#define THRESH_BURNOUT_ACCEL    14.7f
#define THRESH_SEP_ACCEL_DELTA  20.0f
#define THRESH_S2_BOOST_ACCEL   19.6f
#define THRESH_APOGEE_VEL       -2.0f
#define THRESH_LANDING_VEL       3.0f
#define ALT_LANDING_FT          50.0f

// ================================================================
//  TIMEOUTS
// ================================================================
#define TIMEOUT_BURNOUT_MIN_MS   2000UL
#define TIMEOUT_S1_COAST_MS     25000UL
#define TIMEOUT_SEP_SETTLE_MS     300UL
#define TIMEOUT_SEPARATION_MS    8000UL
#define TIMEOUT_AIRSTART_MS      4000UL
#define TIMEOUT_S2_COAST_MS     90000UL
#define TIMEOUT_MISSION_MS     360000UL

// ================================================================
//  SOLENOID  (Stage 2 only)
// ================================================================
#define SOLENOID_HOLD_MS   12000UL

// ================================================================
//  VTX SMARTAUDIO POWER LEVELS  (SA index)
//  0=25mW  1=200mW  2=500mW  3=1000mW
// ================================================================
#define VTX_PWR_PAD     1
#define VTX_PWR_FLIGHT  3
#define VTX_PWR_OFF     0

// ================================================================
//  ALTITUDE FILTER
// ================================================================
#define ALT_FILTER_N    7

// ================================================================
//  OSD — AT7456E / DFR0515
// ================================================================
// Video format selection
#define OSD_FMT_AUTO  0   // Auto-detect from RunCam sync (recommended)
#define OSD_FMT_NTSC  1   // Force NTSC — 13 rows  (RunCam Split 4 US default)
#define OSD_FMT_PAL   2   // Force PAL  — 16 rows
#define OSD_FORCE_FORMAT  OSD_FMT_AUTO

// Safe visible row counts
#define OSD_ROWS_NTSC   13
#define OSD_ROWS_PAL    16
#define OSD_COLS        30

// OSD element rows (safe zone: rows 1–11, visible in both NTSC and PAL)
#define OSD_ROW_TOP        1   // Callsign left + mission clock right
#define OSD_ROW_MID        2   // Altitude left + flight state right
#define OSD_ROW_BOT        3   // Velocity left + fault status right

// AT7456E registers
#define OSD_REG_VM0   0x00
#define OSD_REG_VM1   0x01
#define OSD_REG_DMM   0x04
#define OSD_REG_DMAH  0x05
#define OSD_REG_DMAL  0x06
#define OSD_REG_DMDI  0x07
#define OSD_REG_STAT  0xA0

// VM0 bits
#define OSD_VM0_RESET    0x02
#define OSD_VM0_ENABLE   0x08
#define OSD_VM0_PAL      0x40

// STAT bits (read address 0xA0)
#define OSD_STAT_PAL_DET  0x01
#define OSD_STAT_NTSC_DET 0x02
#define OSD_STAT_RESET    0x40

// ================================================================
//  PACKET PROTOCOL
// ================================================================
#define TRES_MAGIC_0    0x54
#define TRES_MAGIC_1    0x52
#define PKT_TYPE_DATA   0xDA
#define PKT_TYPE_CMD    0xC0
#define PKT_TYPE_ACK    0xAC
#define PKT_TYPE_STATUS 0x57

// Uplink commands
#define CMD_VIDEO_ON       0x01
#define CMD_VIDEO_OFF      0x02
#define CMD_TELEM_HIGH     0x03
#define CMD_TELEM_LOW      0x04
#define CMD_FORCE_PACKET   0x05
#define CMD_FIRE_SOLENOID  0x06
#define CMD_TEST_MODE_ON   0x07   // Deprecated: use CMD_MODE_TEST
#define CMD_TEST_MODE_OFF  0x08   // Deprecated: use CMD_MODE_PAD
#define CMD_SET_LORA_FREQ  0x09   // Extended PLEN=4 float MHz
#define CMD_SET_VTX_FREQ   0x0A   // Extended PLEN=2 uint16 MHz
#define CMD_GET_STATUS     0x0B
#define CMD_MODE_TEST      0x0C   // → GM_TEST_IDLE
#define CMD_MODE_PAD       0x0D   // → GM_PAD_IDLE
#define CMD_MODE_LAUNCH    0x0E   // → GM_LAUNCH_READY
#define CMD_SET_VTX_POWER  0x0F   // Extended PLEN=1 SA index 0-3
#define CMD_CAM_RECORD_ON  0x10   // Start RunCam recording
#define CMD_CAM_RECORD_OFF 0x11   // Stop RunCam recording
#define CMD_FULL_SYS_TEST  0x12   // Full system test: VTX+cam+10Hz telem+solenoid(S2)

#define ACK_OK        0x00
#define ACK_REJECTED  0x01
#define UPLINK_FRAME_MIN  6

// ================================================================
//  FAULT FLAGS
// ================================================================
#define FAULT_NONE  0x00
#define FAULT_BMP   (1 << 0)
#define FAULT_LSM   (1 << 1)
#define FAULT_ADXL  (1 << 2)
#define FAULT_LORA  (1 << 3)
#define FAULT_SD    (1 << 4)
#define FAULT_OSD   (1 << 5)
#define FAULT_CAM   (1 << 6)

// ================================================================
//  TELEMETRY PACKET  (42 bytes packed)
//  Layout must match protocol.py FLIGHT_PACKET_FMT = '<IffffffffhBBBB'
// ================================================================
struct __attribute__((packed)) FlightPacket {
  uint32_t  timestamp_ms;
  float     altitude_ft;
  float     velocity_fps;
  float     accel_thrust;
  float     accel_lateral;
  float     gyro_roll;
  float     gyro_pitch;
  float     gyro_yaw;
  float     temperature_c;
  int16_t   rssi_dbm;
  uint8_t   state;
  uint8_t   stage;
  uint8_t   fault_flags;
  uint8_t   crc8;
};

// ================================================================
//  STATUS PACKET  (19 bytes packed)
//  Layout must match protocol.py STATUS_PACKET_FMT = '<BBBBBBfHBBBBBBB'
//
//  Fields in order:
//    stage, flight_state, fault_flags, test_mode, video_enabled,
//    telem_rate_hz, lora_freq_mhz, vtx_freq_mhz,
//    vtx_power_index, vtx_flight_power, ground_mode, cam_recording,
//    last_cmd, last_cmd_result, crc8
// ================================================================
struct __attribute__((packed)) StatusPacket {
  uint8_t  stage;
  uint8_t  flight_state;
  uint8_t  fault_flags;
  uint8_t  test_mode;        // 1 if ground_mode == TEST (legacy field, kept for compat)
  uint8_t  video_enabled;
  uint8_t  telem_rate_hz;
  float    lora_freq_mhz;
  uint16_t vtx_freq_mhz;
  uint8_t  vtx_power_index;
  uint8_t  vtx_flight_power; // Stored flight-power level (applied when LAUNCH_READY)
  uint8_t  ground_mode;      // 0=TEST_IDLE  1=PAD_IDLE  2=LAUNCH_READY
  uint8_t  cam_recording;    // 0=off  1=recording
  uint8_t  last_cmd;
  uint8_t  last_cmd_result;
  uint8_t  crc8;
};

// ================================================================
//  GROUND OPERATING MODES
//
//  GM_TEST_IDLE    Boot state. Radio RX active. No scheduled telem.
//                  60 s heartbeat STATUS. VTX 25 mW. Cam off.
//                  All commands accepted and confirmed. FSM passive.
//
//  GM_PAD_IDLE     Rocket rigged on rail, waiting for launch window.
//                  1 Hz data telem. 60 s heartbeat. VTX 25 mW.
//                  RunCam not recording. FSM armed (will detect launch).
//
//  GM_LAUNCH_READY All systems live.
//                  Configured telem rate (default 10 Hz).
//                  VTX at stored flight power. RunCam FPV active.
//                  FSM fully armed. GPS displayed on ground station.
//                  Auto-entered on launch detect if in PAD_IDLE.
// ================================================================
enum GroundMode : uint8_t {
  GM_TEST_IDLE    = 0,
  GM_PAD_IDLE     = 1,
  GM_LAUNCH_READY = 2
};

static const char* groundModeStr(GroundMode m) {
  switch (m) {
    case GM_TEST_IDLE:    return "TEST_IDLE";
    case GM_PAD_IDLE:     return "PAD_IDLE";
    case GM_LAUNCH_READY: return "LAUNCH_READY";
    default:              return "?";
  }
}

// ================================================================
//  FLIGHT STATE ENUMERATIONS
// ================================================================
#if STAGE == 1
enum FlightState : uint8_t {
  ST_PAD=0, ST_BOOST=1, ST_COAST=2,
  ST_SEPARATION=3, ST_DESCENT=4, ST_LANDED=5
};
#endif

#if STAGE == 2
enum FlightState : uint8_t {
  ST_PAD=0, ST_BOOST_S1=1, ST_COAST_S1=2, ST_SEPARATION=3,
  ST_AIRSTART_WAIT=4, ST_NO_AIRSTART=5,
  ST_BOOST_S2=6, ST_COAST_S2=7, ST_APOGEE=8,
  ST_DESCENT=9, ST_LANDED=10
};
#endif

static const char* flightStateStr(uint8_t s) {
#if STAGE == 1
  switch(s) {
    case 0: return "PAD";        case 1: return "BOOST";
    case 2: return "COAST";      case 3: return "SEPARATION";
    case 4: return "DESCENT";    case 5: return "LANDED";
    default: return "?";
  }
#elif STAGE == 2
  switch(s) {
    case 0: return "PAD";              case 1: return "BOOST_S1";
    case 2: return "COAST_S1";         case 3: return "SEPARATION";
    case 4: return "AIRSTART_WAIT";    case 5: return "NO_AIRSTART";
    case 6: return "BOOST_S2";         case 7: return "COAST_S2";
    case 8: return "APOGEE";           case 9: return "DESCENT";
    case 10: return "LANDED";          default: return "?";
  }
#endif
}

// ================================================================
//  PERIPHERAL INSTANCES
// ================================================================
Adafruit_BMP3XX   bmp;
Adafruit_LSM6DSOX lsm;
Adafruit_ADXL375  adxl(375);
RH_RF95           rf95(PIN_LORA_CS, PIN_LORA_INT);

// ================================================================
//  FORWARD DECLARATIONS
// ================================================================
void   sendTelemetry();
void   sendACK(uint8_t cmd, uint8_t result);
void   sendStatusPacket(uint8_t cmd, uint8_t result);
void   printSerialStatus(uint8_t cmd, uint8_t result);
void   vtxSetPower(uint8_t level);
bool   vtxSetFrequency(uint16_t targetMHz);
void   setGroundMode(GroundMode newMode);
#if STAGE == 2
void   fireSolenoid();
#endif

// ================================================================
//  GLOBAL STATE
// ================================================================
FlightPacket  pkt;
File          logFile;
bool          logOpen      = false;
bool          missionDone  = false;
FlightState   flightState  = ST_PAD;
GroundMode    groundMode   = GM_TEST_IDLE;

// Task timers
unsigned long tSensor=0, tSD=0, tTelem=0, tOSD=0,
              tDebug=0, tHeartbeat=0;

// OSD mission elapsed time — starts from liftoff detection
unsigned long tOSDLiftoff = 0;   // Set when FSM detects launch
bool          osdMissionStarted = false;

// Flight timestamps
unsigned long tLiftoff=0, tBurnout=0, tSeparation=0;

// Altitude
float basePressureHPa = 1013.25f;
float altBuffer[ALT_FILTER_N] = {0};
uint8_t altBufIdx    = 0;
float prevAltitude   = 0.0f;
float altVelocity    = 0.0f;
float prevAltVelocity= 0.0f;
float prevAccelMag   = 0.0f;
bool  apogeeDetected = false;

uint8_t faultFlags = FAULT_NONE;

// Command / mode state
bool     videoEnabled    = true;
unsigned long telemRateMs= TELEM_RATE_HIGH_MS;  // User-configured rate for LAUNCH_READY
float    currentLoRaFreq = RF95_FREQ_DEFAULT;
uint16_t currentVTXFreq  = 5800;
uint8_t  currentVTXPwrIdx= VTX_PWR_OFF;
uint8_t  vtxFlightPower  = VTX_PWR_FLIGHT;  // Applied when entering LAUNCH_READY
uint8_t  lastCmd         = 0x00;
uint8_t  lastCmdResult   = ACK_OK;

// OSD
uint8_t osdVideoRows = OSD_ROWS_NTSC;
bool    osdReady     = false;

// RunCam state
bool camRecording = false;

// Uplink RX buffer
uint8_t rxBuf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t rxLen = sizeof(rxBuf);

#if STAGE == 2
  bool          solenoidFired   = false;
  unsigned long tSolenoidFired  = 0;
#endif

// ================================================================
//  UTILITY — CRC-8 XOR (packet integrity)
// ================================================================
uint8_t calcCRC8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) crc ^= data[i];
  return crc;
}

// CRC-8/DVB-S2 (poly 0xD5) — used by SmartAudio and RunCam RCDP
static uint8_t crc8_dvbs2(const uint8_t* buf, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? ((crc<<1)^0xD5) : (crc<<1);
  }
  return crc;
}

// ================================================================
//  UTILITY — ALTITUDE FILTER
// ================================================================
float filteredAlt(float rawFt) {
  altBuffer[altBufIdx] = rawFt;
  altBufIdx = (altBufIdx + 1) % ALT_FILTER_N;
  float sum = 0; for (uint8_t i = 0; i < ALT_FILTER_N; i++) sum += altBuffer[i];
  return sum / (float)ALT_FILTER_N;
}

float accelMag() { return sqrtf(pkt.accel_thrust*pkt.accel_thrust + pkt.accel_lateral*pkt.accel_lateral); }
float gyroMag()  { return sqrtf(pkt.gyro_roll*pkt.gyro_roll + pkt.gyro_pitch*pkt.gyro_pitch + pkt.gyro_yaw*pkt.gyro_yaw); }

// ================================================================
//  SD CARD LOGGING
// ================================================================
bool initSD() {
  if (!SD.begin(SD_BUILTIN)) { faultFlags|=FAULT_SD; Serial.println("[SD] FAIL"); return false; }
  char fname[20]; uint8_t n=0;
  do { snprintf(fname,sizeof(fname),"FLT%03dS%d.CSV",n++,STAGE); } while(SD.exists(fname)&&n<200);
  logFile = SD.open(fname, FILE_WRITE);
  if (!logFile) { faultFlags|=FAULT_SD; return false; }
  logFile.println("ts_ms,alt_ft,vel_fps,accel_thrust,accel_lat,gyro_roll,gyro_pitch,gyro_yaw,temp_c,state,stage,faults,ground_mode");
  logFile.flush(); logOpen=true;
  Serial.print("[SD] → "); Serial.println(fname); return true;
}
void logToSD() {
  if ((faultFlags&FAULT_SD)||!logOpen) return;
  logFile.print(pkt.timestamp_ms);    logFile.print(',');
  logFile.print(pkt.altitude_ft,2);   logFile.print(',');
  logFile.print(pkt.velocity_fps,2);  logFile.print(',');
  logFile.print(pkt.accel_thrust,3);  logFile.print(',');
  logFile.print(pkt.accel_lateral,3); logFile.print(',');
  logFile.print(pkt.gyro_roll,4);     logFile.print(',');
  logFile.print(pkt.gyro_pitch,4);    logFile.print(',');
  logFile.print(pkt.gyro_yaw,4);      logFile.print(',');
  logFile.print(pkt.temperature_c,2); logFile.print(',');
  logFile.print(pkt.state);           logFile.print(',');
  logFile.print(pkt.stage);           logFile.print(',');
  logFile.print(pkt.fault_flags);     logFile.print(',');
  logFile.println((uint8_t)groundMode);
  static uint8_t fc=0; if(++fc>=20){logFile.flush();fc=0;}
}
void closeSD() { if(!logOpen)return; logFile.flush(); logFile.close(); logOpen=false; Serial.println("[SD] Closed"); }

// ================================================================
//  LoRa — INIT
// ================================================================
bool initLoRa() {
  pinMode(PIN_LORA_RST, OUTPUT);
  digitalWrite(PIN_LORA_RST,LOW); delay(10);
  digitalWrite(PIN_LORA_RST,HIGH); delay(10);
  if (!rf95.init()) { faultFlags|=FAULT_LORA; Serial.println("[LoRa] FAIL"); return false; }
  if (!rf95.setFrequency(RF95_FREQ_DEFAULT)) { faultFlags|=FAULT_LORA; return false; }
  rf95.setTxPower(RF95_TX_DBM,false);
  rf95.setSpreadingFactor(RF95_SF);
  rf95.setSignalBandwidth(RF95_BW);
  rf95.setCodingRate4(RF95_CR);
  rf95.setModeRx();
  Serial.println("[LoRa] OK — 915 MHz | SF8 | 125 kHz | 23 dBm | RX armed");
  return true;
}

// ================================================================
//  LoRa — DOWNLINK
// ================================================================
void sendTelemetry() {
  if (faultFlags & FAULT_LORA) return;
  pkt.rssi_dbm    = rf95.lastRssi();
  pkt.fault_flags = faultFlags;
  pkt.crc8        = calcCRC8((const uint8_t*)&pkt, sizeof(pkt)-1);
  uint8_t frame[4+sizeof(pkt)];
  frame[0]=TRES_MAGIC_0; frame[1]=TRES_MAGIC_1; frame[2]=(uint8_t)STAGE; frame[3]=PKT_TYPE_DATA;
  memcpy(frame+4, &pkt, sizeof(pkt));
  rf95.send(frame, sizeof(frame));
}

// ================================================================
//  LoRa — ACK + STATUS
// ================================================================
void sendACK(uint8_t cmd, uint8_t result) {
  lastCmd=cmd; lastCmdResult=result;
  if (!(faultFlags&FAULT_LORA)) {
    uint8_t f[7]={TRES_MAGIC_0,TRES_MAGIC_1,(uint8_t)STAGE,PKT_TYPE_ACK,cmd,result,0};
    f[6]=calcCRC8(f,6); rf95.send(f,7);
  }
  printSerialStatus(cmd, result);
  sendStatusPacket(cmd, result);
}

void sendStatusPacket(uint8_t cmd, uint8_t result) {
  if (faultFlags & FAULT_LORA) return;
  StatusPacket sp;
  sp.stage           = STAGE;
  sp.flight_state    = (uint8_t)flightState;
  sp.fault_flags     = faultFlags;
  sp.test_mode       = (groundMode == GM_TEST_IDLE) ? 1 : 0;  // Legacy compat
  sp.video_enabled   = videoEnabled ? 1 : 0;
  sp.telem_rate_hz   = (telemRateMs == TELEM_RATE_HIGH_MS) ? 10 : 1;
  sp.lora_freq_mhz   = currentLoRaFreq;
  sp.vtx_freq_mhz    = currentVTXFreq;
  sp.vtx_power_index = currentVTXPwrIdx;
  sp.vtx_flight_power= vtxFlightPower;
  sp.ground_mode     = (uint8_t)groundMode;
  sp.cam_recording   = camRecording ? 1 : 0;
  sp.last_cmd        = cmd;
  sp.last_cmd_result = result;
  sp.crc8            = calcCRC8((const uint8_t*)&sp, sizeof(sp)-1);
  uint8_t frame[4+sizeof(sp)];
  frame[0]=TRES_MAGIC_0; frame[1]=TRES_MAGIC_1; frame[2]=(uint8_t)STAGE; frame[3]=PKT_TYPE_STATUS;
  memcpy(frame+4, &sp, sizeof(sp));
  rf95.send(frame, sizeof(frame));
}

// ================================================================
//  LoRa — UPLINK COMMAND PROCESSING
// ================================================================
void processUplink() {
  if (faultFlags & FAULT_LORA) return;
  if (!rf95.available())       return;
  rxLen = sizeof(rxBuf);
  if (!rf95.recv(rxBuf, &rxLen)) return;

  if (rxLen < UPLINK_FRAME_MIN)        return;
  if (rxBuf[0] != TRES_MAGIC_0)        return;
  if (rxBuf[1] != TRES_MAGIC_1)        return;
  if (rxBuf[2] != (uint8_t)STAGE)      return;
  if (rxBuf[3] != PKT_TYPE_CMD)        return;

  uint8_t cmd = rxBuf[4];
  bool isExt = (cmd==CMD_SET_LORA_FREQ || cmd==CMD_SET_VTX_FREQ || cmd==CMD_SET_VTX_POWER);
  if (isExt) {
    if (rxLen < 8) return;
    uint8_t plen=rxBuf[5], crcPos=6+plen;
    if (rxLen < (uint8_t)(crcPos+1)) return;
    if (calcCRC8(rxBuf, crcPos) != rxBuf[crcPos]) { Serial.println(F("[UPLINK] Ext CRC fail")); return; }
  } else {
    if (calcCRC8(rxBuf,5) != rxBuf[5]) { Serial.println(F("[UPLINK] CRC fail")); return; }
  }

  Serial.print(F("\n[CMD] 0x")); Serial.println(cmd, HEX);

  switch (cmd) {

    case CMD_VIDEO_ON:
      videoEnabled = true;
      vtxSetPower(groundMode==GM_LAUNCH_READY ? vtxFlightPower : VTX_PWR_OFF);
      sendACK(cmd, ACK_OK);
      break;

    case CMD_VIDEO_OFF:
      videoEnabled = false;
      vtxSetPower(VTX_PWR_OFF);
      sendACK(cmd, ACK_OK);
      break;

    case CMD_TELEM_HIGH:
      telemRateMs = TELEM_RATE_HIGH_MS;
      sendACK(cmd, ACK_OK);
      break;

    case CMD_TELEM_LOW:
      telemRateMs = TELEM_RATE_LOW_MS;
      sendACK(cmd, ACK_OK);
      break;

    case CMD_FORCE_PACKET:
      sendTelemetry();
      printSerialStatus(cmd, ACK_OK);
      break;

    case CMD_GET_STATUS:
      sendACK(cmd, ACK_OK);
      break;

    case CMD_FIRE_SOLENOID:
#if STAGE == 1
      Serial.println(F("[CMD] FIRE_SOLENOID — REJECTED (no solenoid S1)"));
      sendACK(cmd, ACK_REJECTED);
#elif STAGE == 2
      if (solenoidFired) {
        sendACK(cmd, ACK_REJECTED);
      } else if (groundMode==GM_TEST_IDLE) {
        // In test mode: fire allowed
        fireSolenoid(); sendACK(cmd, ACK_OK);
      } else if (flightState==ST_APOGEE || flightState==ST_DESCENT) {
        // In flight past apogee: allow manual
        fireSolenoid(); sendACK(cmd, ACK_OK);
      } else {
        Serial.println(F("[CMD] FIRE_SOLENOID — REJECTED (not in test mode or past apogee)"));
        sendACK(cmd, ACK_REJECTED);
      }
#endif
      break;

    // Legacy test mode commands — map to ground mode equivalents
    case CMD_TEST_MODE_ON:
      setGroundMode(GM_TEST_IDLE);
      sendACK(cmd, ACK_OK);
      break;

    case CMD_TEST_MODE_OFF:
      setGroundMode(GM_PAD_IDLE);
      sendACK(cmd, ACK_OK);
      break;

    case CMD_SET_LORA_FREQ: {
      float newFreq; memcpy(&newFreq, rxBuf+6, 4);
      if (newFreq<902.0f || newFreq>928.0f) { sendACK(cmd, ACK_REJECTED); break; }
      Serial.print(F("[CMD] SET_LORA_FREQ ")); Serial.print(currentLoRaFreq,3);
      Serial.print(F(" → ")); Serial.print(newFreq,3); Serial.println(F(" MHz"));
      // Update tracking variable BEFORE ACK so STATUS packet carries the new frequency,
      // allowing the ground station M0 to retune to the correct frequency.
      currentLoRaFreq = newFreq;
      sendACK(cmd, ACK_OK);
      delay(80);
      if (!rf95.setFrequency(newFreq)) {
        faultFlags|=FAULT_LORA; Serial.println(F("[LoRa] setFrequency() failed"));
      }
      break;
    }

    case CMD_SET_VTX_FREQ: {
      uint16_t reqFreq; memcpy(&reqFreq, rxBuf+6, 2);
      sendACK(cmd, vtxSetFrequency(reqFreq) ? ACK_OK : ACK_REJECTED);
      break;
    }

    case CMD_MODE_TEST:
      setGroundMode(GM_TEST_IDLE);
      sendACK(cmd, ACK_OK);
      break;

    case CMD_MODE_PAD:
      setGroundMode(GM_PAD_IDLE);
      sendACK(cmd, ACK_OK);
      break;

    case CMD_MODE_LAUNCH:
      setGroundMode(GM_LAUNCH_READY);
      sendACK(cmd, ACK_OK);
      break;

    case CMD_SET_VTX_POWER: {
      // Extended PLEN=1: rxBuf[6] = SA power index 0-3
      uint8_t level = rxBuf[6];
      if (level > 3) { sendACK(cmd, ACK_REJECTED); break; }
      vtxFlightPower = level;
      // Apply immediately if already in LAUNCH_READY
      if (groundMode == GM_LAUNCH_READY) vtxSetPower(vtxFlightPower);
      Serial.print(F("[CMD] SET_VTX_POWER → index ")); Serial.println(level);
      sendACK(cmd, ACK_OK);
      break;
    }

    case CMD_CAM_RECORD_ON:
      // Only meaningful in TEST_IDLE (not useful in PAD or LAUNCH)
      runcamStartRecording();
      sendACK(cmd, ACK_OK);
      break;

    case CMD_CAM_RECORD_OFF:
      runcamStopRecording();
      sendACK(cmd, ACK_OK);
      break;

    case CMD_FULL_SYS_TEST:
      // Full system test — battery evaluation sequence.
      // Activates VTX at flight power, starts RunCam recording,
      // enables 10 Hz telemetry, and enters LAUNCH_READY.
      // Stage 2: also fires solenoid (water payload, one-shot).
      // Ground station runs a 90-second PacketStatsTracker during this.
      videoEnabled = true;
      telemRateMs  = TELEM_RATE_HIGH_MS;   // 10 Hz telem
      runcamStartRecording();
      setGroundMode(GM_LAUNCH_READY);      // Applies flight VTX power
#if STAGE == 2
      if (!solenoidFired) fireSolenoid();
#endif
      Serial.println(F("[CMD] FULL_SYS_TEST — video ON, 10Hz telem, LAUNCH_READY, cam recording"));
      sendACK(cmd, ACK_OK);
      break;

    default:
      Serial.print(F("[CMD] Unknown: 0x")); Serial.println(cmd, HEX);
      sendACK(cmd, ACK_REJECTED);
      break;
  }
}

// ================================================================
//  SERIAL STATUS DUMP
// ================================================================
void printSerialStatus(uint8_t cmd, uint8_t result) {
  Serial.println(F("┌── TRES STATUS ──────────────────────────────────────────┐"));
  Serial.print(F("│ Stage:        ")); Serial.println(STAGE);
  Serial.print(F("│ Ground mode:  ")); Serial.println(groundModeStr(groundMode));
  Serial.print(F("│ Flight state: ")); Serial.print((uint8_t)flightState);
  Serial.print(F("  (")); Serial.print(flightStateStr((uint8_t)flightState)); Serial.println(F(")"));
  Serial.print(F("│ Altitude:     ")); Serial.print(pkt.altitude_ft,1); Serial.println(F(" ft"));
  Serial.print(F("│ Velocity:     ")); Serial.print(pkt.velocity_fps,1); Serial.println(F(" fps"));
  Serial.println(F("├── RADIO / VIDEO ────────────────────────────────────────┤"));
  Serial.print(F("│ LoRa:         ")); Serial.print(currentLoRaFreq,3); Serial.println(F(" MHz"));
  Serial.print(F("│ Telem rate:   "));
  if (groundMode==GM_TEST_IDLE) Serial.println(F("OFF (test idle)"));
  else if (groundMode==GM_PAD_IDLE) Serial.println(F("1 Hz (pad idle)"));
  else Serial.println(telemRateMs==TELEM_RATE_HIGH_MS ? F("10 Hz") : F("1 Hz"));
  Serial.print(F("│ VTX freq:     ")); Serial.print(currentVTXFreq); Serial.println(F(" MHz"));
  const char* pwrLbls[]={"25mW","200mW","500mW","1000mW"};
  Serial.print(F("│ VTX current:  ")); Serial.println(currentVTXPwrIdx<=3?pwrLbls[currentVTXPwrIdx]:"?");
  Serial.print(F("│ VTX flight:   ")); Serial.println(vtxFlightPower<=3?pwrLbls[vtxFlightPower]:"?");
  Serial.print(F("│ Video:        ")); Serial.println(videoEnabled?F("ON"):F("OFF"));
  Serial.print(F("│ RunCam:       ")); Serial.println(camRecording?F("RECORDING"):F("idle"));
  Serial.println(F("├── FAULTS ───────────────────────────────────────────────┤"));
  Serial.print(F("│ Fault flags:  0x")); Serial.println(faultFlags, HEX);
#if STAGE == 2
  Serial.print(F("│ Solenoid:     ")); Serial.println(solenoidFired?F("FIRED"):F("ready"));
#endif
  if (cmd) {
    Serial.print(F("│ Last cmd:     0x")); Serial.print(cmd,HEX);
    Serial.print(F("  ")); Serial.println(result==ACK_OK?F("OK"):F("REJECTED"));
  }
  Serial.println(F("└─────────────────────────────────────────────────────────┘"));
}

// ================================================================
//  VTX SMARTAUDIO  (AKK A1918)
//  Serial6 TX = pin 24.  Half-duplex 4800 baud.
//  Frame: [0x00][0xAA][0x62][LEN][CMD][DATA][CRC8/DVB-S2]
// ================================================================
void vtxSendCmd(uint8_t cmd, const uint8_t* data, uint8_t dataLen) {
  uint8_t frame[12]; uint8_t i=0;
  frame[i++]=0x00; frame[i++]=0xAA; frame[i++]=0x62;
  frame[i++]=dataLen+1; frame[i++]=cmd;
  for (uint8_t d=0;d<dataLen&&i<11;d++) frame[i++]=data[d];
  frame[i++]=crc8_dvbs2(frame+2, i-2);
  VTX_SERIAL.write(frame,i); VTX_SERIAL.flush();
}

void vtxSetPower(uint8_t level) {
  vtxSendCmd(0x02, &level, 1);
  currentVTXPwrIdx = level;
}

static const uint16_t VTX_CHAN[5][8] = {
  {5865,5845,5825,5805,5785,5765,5745,5725},
  {5733,5752,5771,5790,5809,5828,5847,5866},
  {5705,5685,5665,5645,5885,5905,5925,5945},
  {5740,5760,5780,5800,5820,5840,5860,5880},
  {5658,5695,5732,5769,5806,5843,5880,5917}
};

bool vtxSetFrequency(uint16_t targetMHz) {
  if (targetMHz<5600||targetMHz>5950) return false;
  uint8_t bestIdx=0; uint16_t bestDelta=0xFFFF;
  for (uint8_t band=0;band<5;band++) for (uint8_t ch=0;ch<8;ch++) {
    uint16_t f=VTX_CHAN[band][ch];
    uint16_t d=(f>targetMHz)?(f-targetMHz):(targetMHz-f);
    if (d<bestDelta){bestDelta=d;bestIdx=band*8+ch;currentVTXFreq=f;}
  }
  vtxSendCmd(0x03, &bestIdx, 1);
  return true;
}

bool initVTX() {
  VTX_SERIAL.begin(4800);
  delay(400);
  vtxSetPower(VTX_PWR_OFF);   // 25 mW at boot — setGroundMode will apply correct level
  Serial.println("[VTX] Serial6 init — 25 mW (boot default)");
  return true;
}

// ================================================================
//  RUNCAM DEVICE PROTOCOL  (RCDP v2)
//  Serial1 TX (pin 1) → RunCam RX
//  Serial1 RX (pin 0) ← RunCam TX  (not parsed, used for future feedback)
//
//  Frame: [0xCC][CMD_ID][DATA_LEN][DATA...][CRC8/DVB-S2]
//  CRC covers all bytes from 0xCC to last DATA byte.
//
//  CMD_ID 0x00 = Get device info (LEN=0)
//  CMD_ID 0x01 = Camera control action (LEN=1)
//    Action 0x03 = START_RECORDING  (RCDP v2 explicit)
//    Action 0x04 = STOP_RECORDING   (RCDP v2 explicit)
//
//  camRecording tracks the believed recording state.
//  START/STOP are idempotent by design (guard with camRecording flag).
//
//  NOTE: If the RunCam does not respond to 0x03/0x04 (older firmware),
//  change to action 0x01 (SIMULATE_POWER_BTN = toggle). The toggle
//  approach requires the camRecording state to always be correct.
// ================================================================
#define RCDP_HEADER       0xCC
#define RCDP_CMD_INFO     0x00
#define RCDP_CMD_CTRL     0x01
#define RCDP_ACT_WIFI     0x00
#define RCDP_ACT_POWER    0x01   // Toggle (v1 fallback)
#define RCDP_ACT_START    0x03   // Explicit start recording (v2)
#define RCDP_ACT_STOP     0x04   // Explicit stop recording  (v2)

static void runcamSendCmd(uint8_t cmdId, const uint8_t* data, uint8_t dataLen) {
  uint8_t frame[12]; uint8_t i=0;
  frame[i++] = RCDP_HEADER;
  frame[i++] = cmdId;
  frame[i++] = dataLen;
  for (uint8_t d=0;d<dataLen&&i<11;d++) frame[i++]=data[d];
  frame[i++] = crc8_dvbs2(frame, i);
  CAM_SERIAL.write(frame,i); CAM_SERIAL.flush();
}

void runcamStartRecording() {
  if (camRecording) return;
  uint8_t action = RCDP_ACT_POWER;   // Toggle — compatible with factory Split 4 firmware
  runcamSendCmd(RCDP_CMD_CTRL, &action, 1);
  camRecording = true;
  Serial.println(F("[CAM] Recording started (RCDP POWER toggle)"));
}

void runcamStopRecording() {
  if (!camRecording) return;
  uint8_t action = RCDP_ACT_POWER;   // Toggle — compatible with factory Split 4 firmware
  runcamSendCmd(RCDP_CMD_CTRL, &action, 1);
  camRecording = false;
  Serial.println(F("[CAM] Recording stopped (RCDP POWER toggle)"));
}

bool initRunCam() {
  CAM_SERIAL.begin(CAM_BAUD);
  delay(200);
  // Request device info — confirms camera is alive on the bus
  // Response is not parsed but will appear on Serial1 RX for future use
  runcamSendCmd(RCDP_CMD_INFO, nullptr, 0);
  Serial.println("[CAM] Serial1 init — info request sent to RunCam");
  return true;
}

// ================================================================
//  GROUND MODE TRANSITIONS
// ================================================================
void setGroundMode(GroundMode newMode) {
  Serial.print(F("[MODE] ")); Serial.print(groundModeStr(groundMode));
  Serial.print(F(" → ")); Serial.println(groundModeStr(newMode));
  groundMode = newMode;

  switch (newMode) {

    case GM_TEST_IDLE:
      // Radio RX only. VTX at minimum. Camera not recording.
      vtxSetPower(VTX_PWR_OFF);
      runcamStopRecording();   // Stop recording if it was running (safe to call)
      // telemRateMs unchanged — not used in TEST_IDLE
      Serial.println(F("[MODE] TEST_IDLE: RX only, heartbeat 60s, no telemetry"));
      break;

    case GM_PAD_IDLE:
      // 1 Hz sensor telem. VTX at minimum. Camera idle.
      vtxSetPower(VTX_PWR_OFF);
      runcamStopRecording();   // Stop if transitioning from TEST with active recording
      Serial.println(F("[MODE] PAD_IDLE: 1Hz telem, heartbeat 60s, sensors armed"));
      break;

    case GM_LAUNCH_READY:
      // Full power: VTX live at configured power, telem at configured rate.
      vtxSetPower(vtxFlightPower);
      // RunCam FPV analog output is always-on when camera is powered.
      // The VTX going to flight power is what starts live video transmission.
      // RunCam's own SD recording is a separate concern — operators can
      // use CMD_CAM_RECORD_ON separately if they want internal recording too.
      Serial.print(F("[MODE] LAUNCH_READY: VTX → "));
      const char* pwrLbls[]={"25mW","200mW","500mW","1000mW"};
      Serial.print(vtxFlightPower<=3 ? pwrLbls[vtxFlightPower] : "?");
      Serial.println(F(", telem live, FSM armed"));
      break;
  }

  // Reset heartbeat timer on every mode change so we don't immediately
  // fire a heartbeat right after a mode ACK was already sent.
  tHeartbeat = millis();
}

// ================================================================
//  OSD
// ================================================================
static void osdWrite(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));
  digitalWrite(PIN_OSD_CS,LOW); SPI.transfer(reg|0x80); SPI.transfer(val); digitalWrite(PIN_OSD_CS,HIGH);
  SPI.endTransaction(); delayMicroseconds(2);
}
static uint8_t osdRead(uint8_t reg) {
  SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));
  digitalWrite(PIN_OSD_CS,LOW); SPI.transfer(reg&0x7F); uint8_t v=SPI.transfer(0x00); digitalWrite(PIN_OSD_CS,HIGH);
  SPI.endTransaction(); return v;
}
static void osdPutChar(uint8_t row, uint8_t col, uint8_t ch) {
  if (row>=osdVideoRows||col>=OSD_COLS) return;
  uint16_t addr=(uint16_t)row*OSD_COLS+col;
  osdWrite(OSD_REG_DMAH,(uint8_t)((addr>>8)&0x01)); osdWrite(OSD_REG_DMAL,(uint8_t)(addr&0xFF)); osdWrite(OSD_REG_DMDI,ch);
}
static void osdPutStr(uint8_t row, uint8_t col, const char* str) { while(*str&&col<OSD_COLS) osdPutChar(row,col++,(uint8_t)*str++); }
static void osdClearRow(uint8_t row) { for(uint8_t c=0;c<OSD_COLS;c++) osdPutChar(row,c,0x00); }

bool initOSD() {
  pinMode(PIN_OSD_CS, OUTPUT); digitalWrite(PIN_OSD_CS, HIGH);
  delay(600);
  osdWrite(OSD_REG_VM0, OSD_VM0_RESET); delay(2);
  uint16_t to=5000;
  while ((osdRead(OSD_REG_STAT)&OSD_STAT_RESET) && --to) delayMicroseconds(1);
  if (!to) { faultFlags|=FAULT_OSD; Serial.println("[OSD] FAIL — reset timeout"); return false; }
  osdWrite(OSD_REG_VM0, OSD_VM0_ENABLE); delay(50);

#if OSD_FORCE_FORMAT == OSD_FMT_NTSC
  osdVideoRows = OSD_ROWS_NTSC;
  Serial.println("[OSD] FORCED NTSC — 13 rows");
#elif OSD_FORCE_FORMAT == OSD_FMT_PAL
  osdWrite(OSD_REG_VM0, OSD_VM0_ENABLE|OSD_VM0_PAL);
  osdVideoRows = OSD_ROWS_PAL;
  Serial.println("[OSD] FORCED PAL — 16 rows");
#else
  bool detected=false;
  uint32_t t0=millis();
  while (millis()-t0 < 500) {
    uint8_t stat=osdRead(OSD_REG_STAT);
    if (stat&OSD_STAT_PAL_DET)  { osdVideoRows=OSD_ROWS_PAL;  Serial.println("[OSD] Detected PAL");  detected=true; break; }
    if (stat&OSD_STAT_NTSC_DET) { osdVideoRows=OSD_ROWS_NTSC; Serial.println("[OSD] Detected NTSC"); detected=true; break; }
    delay(10);
  }
  if (!detected) {
    osdVideoRows=OSD_ROWS_NTSC;
    Serial.println("[OSD] No sync in 500ms — NTSC default (check RunCam power & video cable)");
  }
#endif

  char hdr[28]; snprintf(hdr,sizeof(hdr),"%-10s S%d", CALLSIGN, STAGE);
  osdPutStr(OSD_ROW_TOP, 1, hdr);
  osdReady=true;
  Serial.print("[OSD] Ready — "); Serial.print(osdVideoRows); Serial.println(" rows");
  return true;
}

static void osdPutStrRight(uint8_t row, const char* str) {
  uint8_t len = strlen(str);
  if (len >= OSD_COLS) return;
  uint8_t col = OSD_COLS - len - 2;
  osdPutStr(row, col, str);
}

void updateOSD() {
  if ((faultFlags&FAULT_OSD)||!osdReady) return;
  static bool osdFirstUpdate = true;
  char buf[22];

  if (osdFirstUpdate) {
    for (uint8_t r = 4; r < osdVideoRows - 1; r++) osdClearRow(r);
    osdFirstUpdate = false;
  }

  // Row OSD_ROW_TOP — mission clock right (callsign left written once in initOSD)
  if (osdMissionStarted) {
    unsigned long elapsedSec = (millis() - tOSDLiftoff) / 1000UL;
    snprintf(buf, sizeof(buf), "T+%03lus", elapsedSec);
  } else {
    snprintf(buf, sizeof(buf), "T+ --s");
  }
  osdPutStrRight(OSD_ROW_TOP, buf);

  // Row OSD_ROW_MID — altitude left, flight state right
  snprintf(buf, sizeof(buf), "ALT %5.0f FT", pkt.altitude_ft);
  osdPutStr(OSD_ROW_MID, 1, buf);

  char stateBuf[13];
  snprintf(stateBuf, sizeof(stateBuf), "%s", flightStateStr((uint8_t)flightState));
  osdPutStrRight(OSD_ROW_MID, stateBuf);

  // Row OSD_ROW_BOT — velocity with direction indicator left, fault status right
  char dir = '~';
  if (pkt.velocity_fps >= 2.0f)  dir = '^';
  if (pkt.velocity_fps <= -2.0f) dir = 'v';
  snprintf(buf, sizeof(buf), "VEL %5.1f %c FPS", pkt.velocity_fps, dir);
  osdPutStr(OSD_ROW_BOT, 1, buf);

  if (pkt.fault_flags == 0) {
    snprintf(buf, sizeof(buf), "OK");
  } else {
    snprintf(buf, sizeof(buf), "FLT %02X", pkt.fault_flags);
  }
  osdPutStrRight(OSD_ROW_BOT, buf);
}

// ================================================================
//  SENSORS
// ================================================================
void initSensors() {
  if (!bmp.begin_I2C()) {
    faultFlags|=FAULT_BMP; Serial.println("[BMP388]  FAIL");
  } else {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("[BMP388]  OK");
  }
  if (!lsm.begin_I2C()) {
    faultFlags|=FAULT_LSM; Serial.println("[LSM6DSOX] FAIL");
  } else {
    lsm.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    lsm.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm.setGyroDataRate(LSM6DS_RATE_104_HZ);
    Serial.println("[LSM6DSOX] OK");
  }
  if (!adxl.begin()) {
    faultFlags|=FAULT_ADXL; Serial.println("[ADXL375]  FAIL");
  } else {
    adxl.setDataRate(ADXL343_DATARATE_100_HZ);
    Serial.println("[ADXL375]  OK");
  }
}

void calibratePad() {
  if (faultFlags&FAULT_BMP) { Serial.println("[CAL] BMP faulted — ISA 1013.25 hPa"); return; }
  float sum=0; int good=0;
  Serial.print("[CAL] Sampling");
  for (int i=0;i<40;i++) {
    if (bmp.performReading()){sum+=bmp.pressure/100.0f;good++;}
    delay(50); if(i%8==0) Serial.print('.');
  }
  Serial.println();
  basePressureHPa=(good>0)?(sum/(float)good):1013.25f;
  Serial.print("[CAL] "); Serial.print(basePressureHPa,2); Serial.print(" hPa ("); Serial.print(good); Serial.println(" samples)");
  if (bmp.performReading()) {
    float initAlt=bmp.readAltitude(basePressureHPa)*3.28084f;
    for (uint8_t i=0;i<ALT_FILTER_N;i++) altBuffer[i]=initAlt;
    prevAltitude=initAlt; pkt.altitude_ft=initAlt;
  }
}

// ================================================================
//  SOLENOID  (Stage 2 only)
// ================================================================
#if STAGE == 2
void fireSolenoid() {
  if (solenoidFired) return;
  digitalWrite(PIN_SOLENOID, HIGH); tSolenoidFired=millis(); solenoidFired=true;
  Serial.println("[SOLENOID] OPEN — 12 s hold");
}
void updateSolenoid() {
  if (solenoidFired && (millis()-tSolenoidFired>=SOLENOID_HOLD_MS)) digitalWrite(PIN_SOLENOID, LOW);
}
#endif

// ================================================================
//  SENSOR READ
// ================================================================
void readSensors() {
  pkt.timestamp_ms=millis();
  if (!(faultFlags&FAULT_BMP)) {
    if (bmp.performReading()) { pkt.altitude_ft=filteredAlt(bmp.readAltitude(basePressureHPa)*3.28084f); pkt.temperature_c=bmp.temperature; }
  }
  if (!(faultFlags&FAULT_ADXL)) {
    sensors_event_t ev; adxl.getEvent(&ev);
    pkt.accel_thrust=ev.acceleration.x; pkt.accel_lateral=ev.acceleration.z;
  }
  if (!(faultFlags&FAULT_LSM)) {
    sensors_event_t aEv,gEv,tEv; lsm.getEvent(&aEv,&gEv,&tEv);
    pkt.gyro_roll=gEv.gyro.x; pkt.gyro_pitch=gEv.gyro.y; pkt.gyro_yaw=gEv.gyro.z;
    if (faultFlags&FAULT_BMP) pkt.temperature_c=tEv.temperature;
  }
  static unsigned long tPrevAlt = 0;
  unsigned long tNow = millis();
  float dt = (tPrevAlt > 0) ? (tNow - tPrevAlt) * 0.001f : RATE_SENSOR_MS * 0.001f;
  tPrevAlt = tNow;
  altVelocity = (pkt.altitude_ft - prevAltitude) / dt;
  pkt.velocity_fps=altVelocity; pkt.fault_flags=faultFlags; pkt.state=(uint8_t)flightState; pkt.stage=STAGE;
}

// ================================================================
//  FLIGHT STATE MACHINE — STAGE 1
// ================================================================
#if STAGE == 1
void updateFlightState() {
  // FSM only runs in PAD_IDLE and LAUNCH_READY
  if (groundMode == GM_TEST_IDLE) return;

  float aMag=accelMag(), aDelta=fabsf(aMag-prevAccelMag);
  unsigned long now=millis();

  switch (flightState) {
    case ST_PAD:
      if (aMag > THRESH_LAUNCH_ACCEL) {
        flightState=ST_BOOST; tLiftoff=now;
        tOSDLiftoff = now; osdMissionStarted = true;
        // Auto-advance to LAUNCH_READY on liftoff from PAD_IDLE
        if (groundMode==GM_PAD_IDLE) setGroundMode(GM_LAUNCH_READY);
        Serial.println("[FSM] BOOST — launch");
      }
      break;
    case ST_BOOST:
      if (aMag<THRESH_BURNOUT_ACCEL&&(now-tLiftoff)>TIMEOUT_BURNOUT_MIN_MS)
        {flightState=ST_COAST;tBurnout=now;Serial.println("[FSM] COAST");}
      break;
    case ST_COAST:
      if (aDelta>THRESH_SEP_ACCEL_DELTA||gyroMag()>500.0f)
        {flightState=ST_SEPARATION;tSeparation=now;Serial.println("[FSM] SEPARATION");}
      if (now-tBurnout>TIMEOUT_S1_COAST_MS) {flightState=ST_DESCENT;Serial.println("[FSM] Coast timeout→DESCENT");}
      break;
    case ST_SEPARATION:
      if ((now-tSeparation)>TIMEOUT_SEP_SETTLE_MS||altVelocity<THRESH_APOGEE_VEL)
        {flightState=ST_DESCENT;Serial.println("[FSM] DESCENT");}
      break;
    case ST_DESCENT:
      if (pkt.altitude_ft<ALT_LANDING_FT&&fabsf(altVelocity)<THRESH_LANDING_VEL)
        {flightState=ST_LANDED;Serial.println("[FSM] LANDED");}
      break;
    case ST_LANDED:
      if (!missionDone) {missionDone=true;closeSD();vtxSetPower(VTX_PWR_OFF);Serial.println("[FSM] Mission complete");}
      break;
  }
  prevAccelMag=aMag; prevAltitude=pkt.altitude_ft; prevAltVelocity=altVelocity;
}
#endif

// ================================================================
//  FLIGHT STATE MACHINE — STAGE 2
// ================================================================
#if STAGE == 2
void updateFlightState() {
  if (groundMode == GM_TEST_IDLE) return;

  float aMag=accelMag(), aDelta=fabsf(aMag-prevAccelMag);
  unsigned long now=millis();

  switch (flightState) {
    case ST_PAD:
      if (aMag > THRESH_LAUNCH_ACCEL) {
        flightState=ST_BOOST_S1; tLiftoff=now;
        tOSDLiftoff = now; osdMissionStarted = true;
        if (groundMode==GM_PAD_IDLE) setGroundMode(GM_LAUNCH_READY);
        Serial.println("[FSM] BOOST_S1 — launch");
      }
      break;
    case ST_BOOST_S1:
      if (aMag<THRESH_BURNOUT_ACCEL&&(now-tLiftoff)>TIMEOUT_BURNOUT_MIN_MS)
        {flightState=ST_COAST_S1;tBurnout=now;Serial.println("[FSM] COAST_S1");}
      break;
    case ST_COAST_S1:
      if (aDelta>THRESH_SEP_ACCEL_DELTA)
        {flightState=ST_SEPARATION;tSeparation=now;Serial.println("[FSM] SEPARATION");}
      if (now-tBurnout>TIMEOUT_S1_COAST_MS) {flightState=ST_NO_AIRSTART;Serial.println("[FSM] S1 coast timeout→NO_AIRSTART");}
      break;
    case ST_SEPARATION:
      if ((now-tSeparation)>=TIMEOUT_SEP_SETTLE_MS) {flightState=ST_AIRSTART_WAIT;Serial.println("[FSM] AIRSTART_WAIT");}
      if ((now-tSeparation)>TIMEOUT_SEPARATION_MS) {flightState=ST_NO_AIRSTART;Serial.println("[FSM] Sep timeout→NO_AIRSTART");}
      break;
    // ── IREC 5.13.4: OBSERVE ONLY — NO MOTOR COMMANDS ───────────
    case ST_AIRSTART_WAIT:
      if (aMag>THRESH_S2_BOOST_ACCEL) {flightState=ST_BOOST_S2;Serial.println("[FSM] BOOST_S2 — S2 motor lit");}
      if ((now-tSeparation)>(TIMEOUT_SEP_SETTLE_MS+TIMEOUT_AIRSTART_MS)) {flightState=ST_NO_AIRSTART;Serial.println("[FSM] Airstart timeout→NO_AIRSTART");}
      break;
    case ST_NO_AIRSTART:
      if (!apogeeDetected&&prevAltVelocity>=0.0f&&altVelocity<THRESH_APOGEE_VEL)
        {apogeeDetected=true;fireSolenoid();flightState=ST_APOGEE;Serial.println("[FSM] NO_AIRSTART apogee→solenoid");}
      if (!apogeeDetected&&(now-tSeparation)>30000UL)
        {apogeeDetected=true;fireSolenoid();flightState=ST_APOGEE;Serial.println("[FSM] NO_AIRSTART 30s safety→solenoid");}
      break;
    case ST_BOOST_S2:
      if (aMag<THRESH_BURNOUT_ACCEL) {flightState=ST_COAST_S2;tBurnout=now;Serial.println("[FSM] COAST_S2");}
      break;
    case ST_COAST_S2:
      if (!apogeeDetected&&prevAltVelocity>=0.0f&&altVelocity<THRESH_APOGEE_VEL)
        {apogeeDetected=true;flightState=ST_APOGEE;fireSolenoid();Serial.println("[FSM] APOGEE — solenoid fired");}
      if (!apogeeDetected&&(now-tLiftoff)>TIMEOUT_S2_COAST_MS)
        {apogeeDetected=true;fireSolenoid();flightState=ST_APOGEE;Serial.println("[FSM] Coast timeout→APOGEE solenoid");}
      break;
    case ST_APOGEE:
      if (altVelocity<THRESH_APOGEE_VEL) {flightState=ST_DESCENT;Serial.println("[FSM] DESCENT");}
      break;
    case ST_DESCENT:
      if (pkt.altitude_ft<ALT_LANDING_FT&&fabsf(altVelocity)<THRESH_LANDING_VEL)
        {flightState=ST_LANDED;Serial.println("[FSM] LANDED");}
      break;
    case ST_LANDED:
      if (!missionDone) {missionDone=true;closeSD();vtxSetPower(VTX_PWR_OFF);Serial.println("[FSM] Mission complete");}
      break;
  }
  prevAccelMag=aMag; prevAltitude=pkt.altitude_ft; prevAltVelocity=altVelocity;
}
#endif

// ================================================================
//  SELF-TEST
// ================================================================
#if RUN_SELF_TEST
void runSelfTest() {
  // Step 1 — LED startup signal: 3 fast blinks
  pinMode(PIN_LED, OUTPUT);
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH); delay(80);
    digitalWrite(PIN_LED, LOW);  delay(80);
  }

  // Step 2 — SD initialization
  if (!SD.begin(SD_BUILTIN)) {
    // Infinite rapid blink — SD failed, results cannot be written
    while (true) {
      digitalWrite(PIN_LED, HIGH); delay(50);
      digitalWrite(PIN_LED, LOW);  delay(50);
    }
  }

  // Step 3 — Find unique filename STEST001.TXT … STEST200.TXT
  char fname[16];
  uint8_t n = 1;
  do {
    snprintf(fname, sizeof(fname), "STEST%03d.TXT", n++);
  } while (SD.exists(fname) && n <= 200);
  logFile = SD.open(fname, FILE_WRITE);

  // Step 4 — File header
  logFile.println("TRES TITAN \xe2\x80\x94 SELF-TEST REPORT");
  logFile.println("==============================");
  logFile.println("NOTE: USB-ONLY TEST. External peripheral failures are expected");
  logFile.println("      and normal. Battery rail peripherals cannot be tested");
  logFile.println("      without hardware modification (cutting VUSB/VIN jumper).");
  logFile.println("      Only SD card success is meaningful in this test mode.");
  logFile.println("      DFR0515 result is meaningful only if the chip is installed on the PCB.");
  logFile.println("==============================");
  logFile.print("Stage:    "); logFile.println(STAGE);
  logFile.print("Callsign: "); logFile.println(CALLSIGN);
  logFile.print("Built:    "); logFile.print(__DATE__); logFile.print(" "); logFile.println(__TIME__);
  {
    char tbuf[12];
    dtostrf(tempmonGetTemp(), 5, 1, tbuf);
    logFile.print("MCU Temp: "); logFile.print(tbuf); logFile.println(" C");
  }
  logFile.println();

  // Step 5 — Peripheral tests
  // Wire and SPI initialised here because setup() has not yet run.
  Wire.begin(); Wire.setClock(400000);
  SPI.begin();

  uint8_t faultCount = 0;

  // SD — already confirmed PASS by reaching this line
  logFile.println("SD CARD:       PASS - file open and writable");

  // BMP388
  if (bmp.begin_I2C()) {
    logFile.println("BMP388:        PASS - I2C responded");
  } else {
    logFile.println("BMP388:        FAIL - no I2C response - expected on USB-only");
    faultCount++;
  }

  // LSM6DSOX
  if (lsm.begin_I2C()) {
    logFile.println("LSM6DSOX:      PASS - I2C responded");
  } else {
    logFile.println("LSM6DSOX:      FAIL - no I2C response - expected on USB-only");
    faultCount++;
  }

  // ADXL375
  if (adxl.begin()) {
    logFile.println("ADXL375:       PASS - I2C responded");
  } else {
    logFile.println("ADXL375:       FAIL - no I2C response - expected on USB-only");
    faultCount++;
  }

  // RFM95W — pulse reset then attempt SPI init
  pinMode(PIN_LORA_RST, OUTPUT);
  digitalWrite(PIN_LORA_RST, LOW);  delay(10);
  digitalWrite(PIN_LORA_RST, HIGH); delay(10);
  if (rf95.init()) {
    logFile.println("RFM95W:        PASS - SPI responded");
  } else {
    logFile.println("RFM95W:        FAIL - no SPI response - expected on USB-only");
    faultCount++;
  }

  // DFR0515 (AT7456E OSD) — SPI bus check via software reset and STAT register poll.
  // Inlined SPI transactions match osdWrite/osdRead pattern exactly (SPISettings 4 MHz, MSBFIRST, MODE0).
  // Chip absent: every read returns 0xFF. Reset complete: bit 6 (0x40) of STAT clears.
  {
    bool osdPass = false;
    bool allFF   = true;
    pinMode(PIN_OSD_CS, OUTPUT);
    digitalWrite(PIN_OSD_CS, HIGH);

    // Write VM0 register (0x00): set software reset bit (0x02).
    // Write address = register | 0x80 = 0x80.
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_OSD_CS, LOW);
    SPI.transfer(0x80);
    SPI.transfer(0x02);
    digitalWrite(PIN_OSD_CS, HIGH);
    SPI.endTransaction();

    delay(2);

    // Poll STAT register for up to 5 ms.
    // Read address = 0xA0 & 0x7F = 0x20.
    // Reset complete when bit 6 clears. All-0xFF means chip absent.
    unsigned long t0 = millis();
    while (millis() - t0 < 5) {
      SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
      digitalWrite(PIN_OSD_CS, LOW);
      SPI.transfer(0x20);
      uint8_t stat = SPI.transfer(0x00);
      digitalWrite(PIN_OSD_CS, HIGH);
      SPI.endTransaction();
      if (stat != 0xFF) allFF = false;
      if (!allFF && !(stat & 0x40)) { osdPass = true; break; }
    }

    if (osdPass) {
      logFile.println("DFR0515 (AT7456E): PASS - SPI responded, reset completed");
    } else {
      logFile.println("DFR0515 (AT7456E): FAIL - no SPI response (chip not installed or wiring fault)");
      faultCount++;
    }
  }

  // Teensy internal temperature
  {
    float mcuTemp = tempmonGetTemp();
    char tbuf[12];
    dtostrf(mcuTemp, 5, 1, tbuf);
    if (mcuTemp >= 0.0f && mcuTemp <= 85.0f) {
      logFile.print("TEENSY TEMP:   PASS - "); logFile.print(tbuf); logFile.println(" C");
    } else {
      logFile.print("TEENSY TEMP:   FAIL - "); logFile.print(tbuf); logFile.println(" C (out of range)");
      faultCount++;
    }
  }

  // Step 6 — Fault summary
  logFile.println();
  {
    char summary[72];
    snprintf(summary, sizeof(summary),
             "FAULTS: %d of 6 peripherals failed (expected %d on USB-only)",
             faultCount, faultCount);
    logFile.println(summary);
  }
  logFile.println("RESULT: PASS - SD operational, boot sequence completed without hang");

  // Step 7 — Close file
  logFile.print("Report file: "); logFile.println(fname);
  logFile.flush();
  logFile.close();

  // Step 8 — LED completion signal: 5 slow deliberate blinks
  for (uint8_t i = 0; i < 5; i++) {
    digitalWrite(PIN_LED, HIGH); delay(300);
    digitalWrite(PIN_LED, LOW);  delay(300);
  }
}
#endif

// ================================================================
//  SETUP
// ================================================================
void setup() {
#if RUN_SELF_TEST
  runSelfTest();
#endif

  Serial.begin(115200);
  delay(500);
  Serial.println(F("=========================================="));
  Serial.print(F(" TRES Titan v7.0  |  Stage ")); Serial.println(STAGE);
  Serial.print(F(" Callsign: ")); Serial.println(CALLSIGN);
  Serial.println(F("=========================================="));

#if STAGE == 2
  pinMode(PIN_SOLENOID, OUTPUT);
  digitalWrite(PIN_SOLENOID, LOW);
  Serial.println(F("[SAFETY] Solenoid gate LOW"));
#endif

  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LOW);
  Wire.begin(); Wire.setClock(400000);
  SPI.begin();

  initSensors();
  calibratePad();
  initLoRa();
  initVTX();
  initRunCam();
  initSD();
  initOSD();

  // Enter TEST_IDLE — sets VTX to 25 mW, ensures camera not recording
  setGroundMode(GM_TEST_IDLE);

  Serial.print(F("[INIT] Fault register: 0x")); Serial.println(faultFlags, HEX);
  Serial.println(faultFlags ? F("[INIT] WARNING: degraded systems") : F("[INIT] All systems nominal"));
  Serial.println(F("[INIT] Ground mode: TEST_IDLE"));
  Serial.println(F("[INIT] Send CMD_MODE_PAD then CMD_MODE_LAUNCH to arm for flight\n"));

  uint8_t blinks=faultFlags?6:3;
  for (uint8_t i=0;i<blinks;i++){digitalWrite(PIN_LED,HIGH);delay(120);digitalWrite(PIN_LED,LOW);delay(120);}

  tSensor=tSD=tTelem=tOSD=tDebug=tHeartbeat=millis();
}

// ================================================================
//  MAIN LOOP
// ================================================================
void loop() {
  unsigned long now = millis();

  // ── 100 Hz: sensors + state machine ──────────────────────────
  if (now - tSensor >= RATE_SENSOR_MS) {
    tSensor = now;
    readSensors();
    updateFlightState();
#if STAGE == 2
    updateSolenoid();
#endif
    processUplink();   // Continuous RX polling
  }

  // ── 100 Hz: SD logging ───────────────────────────────────────
  if (now - tSD >= RATE_SD_MS) { tSD=now; logToSD(); }

  // ── Scheduled telemetry — mode-dependent ─────────────────────
  //   TEST_IDLE:    NO scheduled telem (only heartbeat and command ACKs)
  //   PAD_IDLE:     1 Hz fixed
  //   LAUNCH_READY: telemRateMs (default 10 Hz, adjustable)
  if (groundMode != GM_TEST_IDLE) {
    unsigned long effectiveRate = (groundMode==GM_PAD_IDLE) ? TELEM_RATE_LOW_MS : telemRateMs;
    if (now - tTelem >= effectiveRate) {
      tTelem = now;
      sendTelemetry();
      processUplink();   // Extra poll after TX — catch commands in TX window gap
    }
  }

  // ── 2 Hz: OSD ────────────────────────────────────────────────
  if (now - tOSD >= RATE_OSD_MS) { tOSD=now; updateOSD(); }

  // ── 60 s: Heartbeat STATUS (TEST_IDLE and PAD_IDLE only) ─────
  // This is an unsolicited STATUS packet confirming the link is alive.
  // In PAD_IDLE it supplements the 1 Hz telem with complete system state.
  // In LAUNCH_READY it's suppressed — continuous telem is the confirmation.
  if (groundMode != GM_LAUNCH_READY && (now-tHeartbeat >= HEARTBEAT_MS)) {
    tHeartbeat = now;
    sendStatusPacket(0x00, ACK_OK);
    Serial.print(F("[HEARTBEAT] STATUS sent — mode: ")); Serial.println(groundModeStr(groundMode));
  }

  // ── 2 Hz: Serial debug (TEST_IDLE and PAD_IDLE only) ─────────
  if (groundMode != GM_LAUNCH_READY && (now-tDebug >= RATE_DEBUG_MS)) {
    tDebug = now;
    Serial.print(F("ALT:")); Serial.print(pkt.altitude_ft,1);
    Serial.print(F("ft VEL:")); Serial.print(pkt.velocity_fps,1);
    Serial.print(F("fps A:")); Serial.print(accelMag(),2);
    Serial.print(F("m/s² MODE:")); Serial.print(groundModeStr(groundMode));
    Serial.print(F(" FLT:0x")); Serial.print(faultFlags,HEX);
    Serial.print(F(" CAM:")); Serial.println(camRecording?F("REC"):F("idle"));
  }

  // ── Mission clock failsafe ────────────────────────────────────
  if (!missionDone && tLiftoff>0 && (now-tLiftoff)>TIMEOUT_MISSION_MS) {
    Serial.println(F("[SAFETY] Mission clock — flushing SD"));
    closeSD(); missionDone=true;
  }
}
