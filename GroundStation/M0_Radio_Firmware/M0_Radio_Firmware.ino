/*
 * ================================================================
 *  TRES Titan — Ground Radio Firmware
 *  Adafruit Feather M0 RFM95W LoRa
 *
 *  SET STAGE BEFORE FLASHING:
 *    #define STAGE  1   ← Stage 1 (Booster) paired radio
 *    #define STAGE  2   ← Stage 2 (Sustainer) paired radio
 *
 *  This firmware makes the Feather M0 a transparent LoRa bridge
 *  between the Raspberry Pi ground station and the rocket Teensy.
 *
 *  Pi → M0 USB Serial → LoRa TX → Rocket Teensy
 *  Rocket Teensy → LoRa RX → M0 USB Serial → Pi
 *
 *  FREQUENCY RETUNE PROTOCOL
 *  ─────────────────────────────────────────────────────────────
 *  When the Pi sends CMD_SET_LORA_FREQ to the rocket, it also
 *  needs to retune this M0. The Pi writes a special escape
 *  sequence to the M0's USB serial port:
 *
 *    [0xFF][0xFE][f0][f1][f2][f3]   — 6 bytes total
 *    f0–f3 = new frequency in MHz as little-endian 32-bit float
 *
 *  The M0 intercepts this escape, calls rf95.setFrequency(), and
 *  does NOT forward it to LoRa. Everything else passes through.
 *
 *  On reconnect, the Pi's RadioWorker re-sends the last known
 *  frequency so the M0 stays in sync after a USB reconnect.
 *
 *  LED STATUS
 *  ─────────────────────────────────────────────────────────────
 *  Solid on boot           → RFM95W init OK
 *  Short blink on TX       → LoRa packet transmitted
 *  Long blink on RX        → LoRa packet received
 *  Rapid triple blink      → RFM95W init failed (check SPI)
 *
 *  HARDWARE PINS (Adafruit Feather M0 RFM95W)
 *  ─────────────────────────────────────────────────────────────
 *  RFM95W CS   → pin 8   (SPI chip select)
 *  RFM95W RST  → pin 4   (hardware reset)
 *  RFM95W INT  → pin 3   (DIO0 interrupt, required by RadioHead)
 *  LED         → pin 13
 * ================================================================
 */

// ── EDIT THIS BEFORE FLASHING ────────────────────────────────
#define STAGE  1    // 1 = Booster radio  |  2 = Sustainer radio

#include <SPI.h>
#include <RH_RF95.h>

// Hardware pins (Adafruit Feather M0 RFM95W)
#define RFM95_CS    8
#define RFM95_RST   4
#define RFM95_INT   3
#define PIN_LED     13

// LoRa RF settings — MUST match Teensy_TRES_v7.ino exactly
#define RF95_FREQ_DEFAULT  915.0f
#define RF95_SF            8
#define RF95_BW            125000UL
#define RF95_CR            5
#define RF95_TX_DBM        23

// Pi ↔ M0 escape protocol
// Pi sends these 6 bytes to retune the M0 (not forwarded to LoRa):
//   [0xFF][0xFE][float_4bytes_little_endian]
// The float is the new LoRa frequency in MHz.
#define ESC_0  0xFF
#define ESC_1  0xFE

// Max TRES uplink frame size:
//   Extended frame = header(4) + cmd(1) + plen(1) + payload(4) + crc(1) = 11 bytes
//   Give generous buffer
#define UPLINK_BUF_SIZE  32
#define DOWNLINK_BUF_SIZE 128

RH_RF95 rf95(RFM95_CS, RFM95_INT);

float   currentFreq   = RF95_FREQ_DEFAULT;

// Uplink parser state machine
enum UplinkState {
  UL_IDLE,       // Waiting for first byte of a TRES frame or escape
  UL_SAW_FF,     // Saw 0xFF — could be escape byte 0
  UL_ESC_FLOAT,  // Reading 4-byte float for retune
  UL_MAGIC1,     // Saw 0x54 (TRES magic 0) — waiting for 0x52
  UL_STAGE,      // Waiting for stage byte
  UL_TYPE,       // Waiting for packet type byte
  UL_CMD,        // Waiting for command byte
  UL_ACCUM       // Accumulating remaining frame bytes
};

UplinkState ulState  = UL_IDLE;
uint8_t     ulBuf[UPLINK_BUF_SIZE];
uint8_t     ulLen    = 0;
uint8_t     ulTarget = 0;   // Expected total frame length
uint8_t     escFloatBuf[4];
uint8_t     escFloatIdx = 0;

// TRES protocol constants (mirrors Teensy)
#define TRES_MAGIC_0   0x54
#define TRES_MAGIC_1   0x52
#define PKT_TYPE_CMD   0xC0
// Extended command bytes (need extra payload bytes)
#define CMD_SET_LORA_FREQ  0x09   // PLEN = 4
#define CMD_SET_VTX_FREQ   0x0A   // PLEN = 2
#define CMD_SET_VTX_POWER  0x0F   // PLEN = 1

// ── LED helpers ─────────────────────────────────────────────────
void ledBlink(uint8_t n, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(PIN_LED, HIGH); delay(onMs);
    digitalWrite(PIN_LED, LOW);  delay(offMs);
  }
}


// ── SETUP ────────────────────────────────────────────────────────
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // USB serial to Pi — 115200 baud
  // Do not wait for serial connection; we run in the field without a host.
  Serial.begin(115200);
  delay(500);   // Allow USB enumeration

  // RFM95W hardware reset
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH); delay(10);
  digitalWrite(RFM95_RST, LOW);  delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  if (!rf95.init()) {
    // Init failed — triple blink loop forever
    Serial.print(F("[M0 S")); Serial.print(STAGE);
    Serial.println(F("] FATAL: RFM95W init failed — check SPI wiring"));
    while (true) { ledBlink(3, 80, 80); delay(500); }
  }

  rf95.setFrequency(RF95_FREQ_DEFAULT);
  rf95.setTxPower(RF95_TX_DBM, false);
  rf95.setSpreadingFactor(RF95_SF);
  rf95.setSignalBandwidth(RF95_BW);
  rf95.setCodingRate4(RF95_CR);
  rf95.setModeRx();   // Start in receive mode

  digitalWrite(PIN_LED, HIGH);  // Solid = init OK
  delay(300);
  digitalWrite(PIN_LED, LOW);

  Serial.print(F("[M0 S")); Serial.print(STAGE);
  Serial.print(F("] Ready — ")); Serial.print(RF95_FREQ_DEFAULT, 3);
  Serial.println(F(" MHz | SF8 | 125 kHz | 23 dBm | RX armed"));
}


// ── UPLINK: determine expected frame length ───────────────────────
//
// Standard frame:  MAGIC0+MAGIC1+STAGE+PKT_TYPE+CMD+CRC = 6 bytes
// Extended frame:  ... + PLEN + payload(PLEN) + CRC = 7+PLEN bytes
// Returns 0 if unknown (discard and rescan).
uint8_t frameLength(uint8_t cmd, uint8_t plen) {
  // All frames have at minimum: magic0, magic1, stage, type, cmd = 5 bytes
  // Standard frames add 1 CRC byte = 6 total
  // Extended frames: cmd known-extended → add plen_byte(1) + plen + crc(1) = 7+plen
  switch (cmd) {
    case CMD_SET_LORA_FREQ:
    case CMD_SET_VTX_FREQ:
    case CMD_SET_VTX_POWER:
      return 7 + plen;   // Extended: header(5) + plen_byte(1) + payload(plen) + crc(1)
    default:
      return 6;           // Standard: header(5) + crc(1)
  }
}


// ── UPLINK BYTE PROCESSING ───────────────────────────────────────
//
// Called for every byte received from Pi USB serial.
// Implements a state machine that:
//   1. Detects the 0xFF 0xFE retune escape and reacts to it locally
//   2. Accumulates complete TRES command frames
//   3. Transmits complete frames via LoRa when ready
//
void processUplinkByte(uint8_t b) {
  switch (ulState) {

    case UL_IDLE:
      if (b == ESC_0) {
        // Possible retune escape
        ulState = UL_SAW_FF;
      } else if (b == TRES_MAGIC_0) {
        // Possible TRES frame start
        ulBuf[0] = b; ulLen = 1;
        ulState  = UL_MAGIC1;
      }
      // Any other byte is discarded — we only forward complete TRES frames
      break;

    case UL_SAW_FF:
      if (b == ESC_1) {
        // Confirmed retune escape — read 4-byte float next
        escFloatIdx = 0;
        ulState     = UL_ESC_FLOAT;
      } else if (b == ESC_0) {
        // Another 0xFF — stay in SAW_FF
      } else if (b == TRES_MAGIC_0) {
        // The 0xFF was not an escape — start a TRES frame
        ulBuf[0] = b; ulLen = 1;
        ulState  = UL_MAGIC1;
      } else {
        ulState = UL_IDLE;
      }
      break;

    case UL_ESC_FLOAT:
      escFloatBuf[escFloatIdx++] = b;
      if (escFloatIdx == 4) {
        // Have all 4 float bytes
        float newFreq;
        memcpy(&newFreq, escFloatBuf, 4);
        if (newFreq >= 902.0f && newFreq <= 928.0f) {
          rf95.setFrequency(newFreq);
          currentFreq = newFreq;
          Serial.print(F("[M0 S")); Serial.print(STAGE);
          Serial.print(F("] Retuned → ")); Serial.print(newFreq, 3);
          Serial.println(F(" MHz"));
        } else {
          Serial.print(F("[M0 S")); Serial.print(STAGE);
          Serial.print(F("] Retune rejected — out of range: ")); Serial.println(newFreq, 3);
        }
        ulState = UL_IDLE;
      }
      break;

    case UL_MAGIC1:
      if (b == TRES_MAGIC_1) {
        ulBuf[ulLen++] = b;
        ulState = UL_STAGE;
      } else {
        // Not a valid TRES frame — restart scan
        ulState = UL_IDLE;
        if (b == TRES_MAGIC_0) {
          ulBuf[0] = b; ulLen = 1;
          ulState  = UL_MAGIC1;
        }
      }
      break;

    case UL_STAGE:
      // Accept any stage byte (we forward whatever the Pi sends)
      ulBuf[ulLen++] = b;
      ulState = UL_TYPE;
      break;

    case UL_TYPE:
      if (b == PKT_TYPE_CMD) {
        ulBuf[ulLen++] = b;
        ulState = UL_CMD;
      } else {
        // Not a command — discard
        ulState = UL_IDLE;
      }
      break;

    case UL_CMD: {
      ulBuf[ulLen++] = b;   // Store CMD byte
      // Determine if this is a standard or extended frame
      uint8_t cmd = b;
      if (cmd == CMD_SET_LORA_FREQ || cmd == CMD_SET_VTX_FREQ || cmd == CMD_SET_VTX_POWER) {
        // Extended: need to read PLEN byte next before we know total length
        ulTarget = 0;   // 0 = don't know yet, waiting for PLEN
        ulState  = UL_ACCUM;
      } else {
        // Standard 6-byte frame: we already have 5 bytes, need 1 more (CRC)
        ulTarget = 6;
        ulState  = UL_ACCUM;
      }
      break;
    }

    case UL_ACCUM:
      ulBuf[ulLen++] = b;

      // If ulTarget is not yet set, this byte is PLEN for an extended frame
      if (ulTarget == 0) {
        uint8_t plen = b;
        ulTarget = 7 + plen;   // header(5) + plen_byte(1) + payload(plen) + crc(1)
      }

      if (ulLen >= UPLINK_BUF_SIZE) {
        // Buffer overflow — discard and reset
        ulState = UL_IDLE; ulLen = 0; ulTarget = 0;
        break;
      }

      if (ulLen == ulTarget) {
        // Complete frame — transmit via LoRa
        // Brief LED blink to indicate TX
        digitalWrite(PIN_LED, HIGH);
        rf95.send(ulBuf, ulLen);
        rf95.waitPacketSent();
        rf95.setModeRx();   // Return to RX after transmit
        digitalWrite(PIN_LED, LOW);

        // Reset for next frame
        ulState = UL_IDLE; ulLen = 0; ulTarget = 0;
      }
      break;
  }
}


// ── MAIN LOOP ────────────────────────────────────────────────────
void loop() {

  // ── Pi → LoRa (uplink) ──────────────────────────────────────
  while (Serial.available()) {
    processUplinkByte((uint8_t)Serial.read());
  }

  // ── LoRa → Pi (downlink) ────────────────────────────────────
  if (rf95.available()) {
    uint8_t buf[DOWNLINK_BUF_SIZE];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len) && len > 0) {
      // Forward received bytes to Pi immediately
      Serial.write(buf, len);
      // Brief long blink to indicate RX
      digitalWrite(PIN_LED, HIGH); delay(50); digitalWrite(PIN_LED, LOW);
    }
    rf95.setModeRx();   // Re-arm receiver
  }
}
