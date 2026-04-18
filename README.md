# TRES Titan Avionics

**Student-Designed Two-Stage High-Power Rocket Avionics System**

Designed for the **Intercollegiate Rocket Engineering Competition (IREC)** hosted by Friends of Amateur Rocketry (FAR), launching June 2026 at the Mojave Desert test site.

## Overview

TRES Titan is a complete telemetry, tracking, and control system for a 2-stage amateur high-power rocket featuring:

- **Dual Teensy 4.1 flight computers** (one per stage) with full sensor suite
- **915 MHz LoRa telemetry** for real-time flight data downlink
- **5.8 GHz FPV video transmission** with on-screen display overlay
- **Python/PyQt5 ground station** with live map, video feeds, and command uplink
- **Featherweight GPS tracking** for both stages
- **Full system test mode** for battery evaluation and pre-flight validation

---

## Repository Structure

```
TresCode/
├── GroundStation/          # Ground control application
│   ├── main.py            # PyQt5 application entry point
│   ├── main_window.py     # Main UI window
│   ├── protocol.py        # TRES packet protocol
│   ├── config.py          # Configuration constants
│   ├── data_recorder.py   # CSV telemetry logging + packet stats
│   ├── radio_worker.py    # LoRa radio QThread
│   ├── gps_worker.py      # Featherweight GPS QThread
│   ├── video_worker.py    # Video capture QThread
│   ├── map_widget.py      # Leaflet.js map integration
│   ├── requirements.txt   # Python dependencies
│   └── M0_Radio_Firmware/ # Adafruit M0 LoRa radio firmware
│       └── M0_Radio_Firmware.ino
└── TeensyCode/
    └── Teensy_TRES_v7/
        └── Teensy_TRES_v7.ino  # Rocket flight computer (1282 lines)
```

---

## GroundStation/

**Python/PyQt5 Ground Control Application**

### Features

- **Real-time Telemetry Display**: Altitude, velocity, acceleration, temperature, RSSI, flight state
- **Dual Video Feeds**: Live capture from Stage 1 and Stage 2 cameras
- **Live Map**: Offline-first Leaflet.js map with GPS tracking
- **Command & Control**: Ground mode switching, VTX/LoRa tuning, solenoid control
- **Full System Test**: 90-second battery evaluation with packet statistics and automated reporting
- **Showcase Mode**: Replaces one video panel with a looping pre-rendered highlight reel for presentations while keeping the other panel live. Configure via `SHOWCASE_VIDEO_PATH` and `SHOWCASE_STAGE` in `config.py`.
- **Multi-threaded Architecture**: Separate QThreads for radio, GPS, and video

### Installation

**System Requirements:**
- Python 3.7+
- PyQt5 with WebEngine support
- OpenCV (cv2)
- USB serial drivers (pyserial)

**Install Python dependencies:**
```bash
cd GroundStation
pip install -r requirements.txt
```

**On Raspberry Pi, you may need:**
```bash
sudo apt-get install python3-pyqt5 python3-pyqt5.qtwebengine
sudo apt-get install libgl1-mesa-glx libglib2.0-0 v4l-utils
```

### Configuration

Edit `config.py` before launch:
- Serial port assignments (`/dev/ttyUSB*`, `/dev/ttyACM*`)
- Video capture device indices (`/dev/video0`, `/dev/video1`)
- LoRa frequencies (902-928 MHz ISM band)
- Launch site coordinates (currently set to FAR site: 35.3462°N, 117.8101°W)

### Usage

**Run the ground station:**
```bash
python3 main.py
```

**Pre-flight offline setup** (run at home on WiFi before going to launch site):
```bash
python3 setup_offline.py    # Download Leaflet.js library
python3 cache_tiles.py      # Cache OpenStreetMap tiles (16km radius)
```

**Identify hardware ports:**
```bash
ls /dev/ttyUSB* /dev/ttyACM*   # List serial devices
v4l2-ctl --list-devices        # List video capture cards
```

### M0 Radio Firmware

The ground station uses two **Adafruit Feather M0 RFM95W** radios (one per stage) as USB-to-LoRa bridges.

**Flash the firmware:**
1. Open `M0_Radio_Firmware/M0_Radio_Firmware.ino` in Arduino IDE
2. Set `STAGE` to 1 or 2 (must match paired rocket stage)
3. Select **Board**: Adafruit Feather M0
4. Select **Port**: COM port for the M0
5. Upload

The M0 firmware supports frequency retuning via escape sequence without reprogramming.

---

## TeensyCode/

**Teensy 4.1 Flight Computer Firmware**

### Features

- **6-DOF IMU + High-G Accelerometer + Barometric Altimeter**
- **Flight State Machine**: PAD → BOOST → COAST → SEPARATION → DESCENT → LANDED (Stage 1)
- **LoRa Telemetry**: 42-byte flight packets @ 10 Hz, 19-byte status packets
- **OSD Overlay**: Real-time flight data on analog video feed (AT7456E chip)
- **VTX SmartAudio Control**: AKK A1918 5.8 GHz video transmitter
- **RunCam UART Control**: Start/stop internal SD recording
- **On-board SD Card Logging**: 100 Hz backup logging independent of RF
- **Ground Mode Operation**: TEST_IDLE → PAD_IDLE → LAUNCH_READY
- **Full System Test Support**: 90-second battery evaluation mode

### Hardware

- **Teensy 4.1** microcontroller
- **BMP388** barometric altimeter (I2C)
- **LSM6DSOX** 6-axis IMU (I2C)
- **ADXL375** high-G accelerometer (±200g, I2C)
- **RFM95W** LoRa radio module (SPI, 915 MHz)
- **AT7456E** OSD chip (SPI)
- **AKK A1918** 5.8 GHz video transmitter (UART SmartAudio)
- **RunCam** FPV camera (UART control)
- **Solenoid valve** (Stage 2 only, water payload ejection)

### Installation

**Required Arduino Libraries:**
- RadioHead (RH_RF95.h)
- Adafruit_BMP3XX
- Adafruit_LSM6DSOX
- Adafruit_ADXL375
- Adafruit_Sensor
- Wire, SPI, SD (built-in)

Install via Arduino IDE Library Manager.

### Configuration

**Before uploading:**
1. Set `STAGE` constant (1 or 2) at top of sketch
2. Configure `MY_CALLSIGN` with your amateur radio callsign (required for legal 915 MHz operation)
3. Verify pin assignments match your PCB layout

**Upload to Teensy:**
1. Open `Teensy_TRES_v7.ino` in Arduino IDE with Teensyduino installed
2. Select **Board**: Teensy 4.1
3. Select **USB Type**: Serial
4. Select **Port**: COM port for Teensy
5. Upload

---

## TRES Protocol

**Binary packet protocol** with magic headers `0x54 0x52` ("TR"), CRC-8 validation, and multiple packet types:

- **PKT_DATA (0xDA)**: 42-byte flight telemetry packets
- **PKT_STATUS (0x57)**: 19-byte system status packets
- **PKT_CMD (0xC0)**: Uplink command frames
- **PKT_ACK (0xAC)**: Command acknowledgment

All communications are stage-aware (Stage 1 / Stage 2 independent).

### Uplink Commands

- **Ground Mode**: `CMD_MODE_TEST`, `CMD_MODE_PAD`, `CMD_MODE_LAUNCH`
- **Video**: `CMD_VIDEO_ON`, `CMD_VIDEO_OFF`, `CMD_SET_VTX_FREQ`, `CMD_SET_VTX_POWER`
- **Telemetry**: `CMD_TELEM_HIGH` (10 Hz), `CMD_TELEM_LOW` (1 Hz), `CMD_SET_LORA_FREQ`
- **Camera**: `CMD_CAM_RECORD_ON`, `CMD_CAM_RECORD_OFF`
- **Payload**: `CMD_FIRE_SOLENOID` (Stage 2 only, TEST_IDLE mode only per IREC Rule 5.13.4)
- **Test**: `CMD_FULL_SYS_TEST` (90-second battery evaluation)

---

## Flight Modes

### Ground Modes

| Mode | Description | Telemetry | VTX Power | Purpose |
|------|-------------|-----------|-----------|---------|
| **TEST_IDLE** | Bench testing | RX only, 60s heartbeat | 25mW | Pre-flight checkout, camera tests |
| **PAD_IDLE** | On launch rail | 1 Hz + heartbeat | 25mW | Final arm, waiting for launch |
| **LAUNCH_READY** | Armed for flight | 10 Hz (configurable) | Flight power (1W default) | Active mission |

### Flight States (Stage 1)

`PAD` → `BOOST` → `COAST` → `SEPARATION` → `DESCENT` → `LANDED`

### Flight States (Stage 2)

`PAD` → `BOOST_S1` → `COAST_S1` → `SEPARATION` → `AIRSTART_WAIT` → `BOOST_S2` → `COAST_S2` → `APOGEE` → `DESCENT` → `LANDED`

---

## Competition Compliance

This system is designed for **IREC 2026** and complies with:

- **IREC Rule 5.13.4**: Solenoid payload cannot initiate motor ignition (motor ignition handled by commercial Featherweight Blue Jay/Raven altimeters)
- **Student-Researched and Developed (SRAD)**: Custom Teensy-based avionics, not commercial flight computers
- **FCC Part 97**: 915 MHz LoRa transmission requires licensed amateur radio operator callsign
- **Offline Operation**: No internet dependency at launch site (pre-cached maps and libraries)

---

## Data Recording

All received telemetry is logged to CSV files in `recordings/`:

- **Flight packets**: Timestamp, altitude, velocity, 3-axis accel/gyro, temperature, RSSI, state, faults
- **Status packets**: Ground mode, LoRa/VTX frequencies, power settings, last command
- **GPS tracking**: Featherweight GPS Ground Station V2 position updates
- **Event markers**: User actions, mode changes, test start/stop

**Full System Test** generates additional detailed text reports with packet statistics, signal quality analysis, and pass/fail evaluation.

---

## License

**Private Repository** — Not for public distribution.

Contains site-specific configuration (FAR launch coordinates) and will contain licensed amateur radio callsigns.

---

## Contact

TRES Titan Rocketry Team
Targeting IREC 2026 — Friends of Amateur Rocketry, Mojave CA

**Built with**: Python, PyQt5, C++ (Arduino), Teensy 4.1, Adafruit M0, RadioHead, OpenCV, Leaflet.js
