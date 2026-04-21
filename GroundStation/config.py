# =============================================================================
#  TRES Titan Ground Station — config.py
#  All configurable constants in one place.
#  Edit this file before every launch / test session.
# =============================================================================

# -----------------------------------------------------------------------------
#  SERIAL PORTS
#  Run `ls /dev/ttyUSB* /dev/ttyACM*` after plugging in each device to find
#  the correct port assignments. Disconnect one device at a time to confirm.
# -----------------------------------------------------------------------------

# Adafruit M0 LoRa radio — Stage 1 uplink/downlink
M0_S1_PORT     = "/dev/tres_m0_s1"
M0_S1_BAUD     = 115200

# Adafruit M0 LoRa radio — Stage 2 uplink/downlink
M0_S2_PORT     = "/dev/tres_m0_s2"
M0_S2_BAUD     = 115200

# Featherweight GPS Ground Station V2 — USB serial
# Baud rate: 57600 per Featherweight documentation (Appendix A).
# Verify against your unit's serial output if packets appear garbled.
FW_GPS_PORT    = "/dev/tres_fw_gps"
FW_GPS_BAUD    = 57600

# -----------------------------------------------------------------------------
#  VIDEO CAPTURE
#  Capture cards appear as /dev/video0, /dev/video1, etc.
#  Run `v4l2-ctl --list-devices` to identify which index is which camera.
# -----------------------------------------------------------------------------

VIDEO_S1_INDEX = 0    # Stage 1 camera capture card device index
VIDEO_S2_INDEX = 2    # Stage 2 camera capture card device index  (/dev/video2)

# Frame size requested from capture card
# Most USB capture cards support 640x480 at 30 fps reliably on Pi 5.
# If your card supports 1280x720, increase these.
VIDEO_WIDTH    = 640
VIDEO_HEIGHT   = 480
VIDEO_FPS      = 30

# -----------------------------------------------------------------------------
#  RADIO FREQUENCIES — INITIAL / DEFAULT VALUES
#  These are the startup defaults. Both can be changed live via the command
#  panel, which will retune the M0 radio and send CMD_SET_LORA_FREQ to the
#  rocket. Frequencies must stay within 902.0–928.0 MHz (USA ISM band).
# -----------------------------------------------------------------------------

LORA_S1_FREQ_MHZ   = 915.0   # Stage 1 LoRa starting frequency
LORA_S2_FREQ_MHZ   = 915.0   # Stage 2 LoRa starting frequency
                               # Use different frequencies if interference observed

# VTX frequencies — display only, not used to command radio hardware directly.
# These are updated by STATUS packets received from the rocket.
VTX_S1_FREQ_MHZ    = 5760   # Band F Ch 2 — Stage 1 Booster default
VTX_S2_FREQ_MHZ    = 5800   # Band F Ch 4 — Stage 2 Sustainer default

# -----------------------------------------------------------------------------
#  MAP — LAUNCH SITE COORDINATES
#  MAP_CENTER_* sets the startup view. Change MAP_ACTIVE_SITE to switch sites.
#  Site selector buttons in the Map panel also switch at runtime.
# -----------------------------------------------------------------------------

MAP_SITES = {
    "FAR":  {
        "name":     "FAR — Mojave",
        "lat":      35.3462467793672,
        "lon":      -117.81008271636995,
        "zoom":     15,
        "margin":   0.15,   # ~16 km radius tile cache
        "max_zoom": 17,
    },
    "CSUF": {
        "name":     "CSUF — Fullerton",
        "lat":      33.8825,
        "lon":      -117.8851,
        "zoom":     15,
        "margin":   0.05,   # ~5 km radius
        "max_zoom": 15,
    },
    "MDARS": {
        "name":     "MDARS",
        "lat":      35.104944,
        "lon":      -117.795194,
        "zoom":     15,
        "margin":   0.05,
        "max_zoom": 15,
    },
}

# Which site to center on at startup
MAP_ACTIVE_SITE = "FAR"

_site = MAP_SITES[MAP_ACTIVE_SITE]
MAP_CENTER_LAT  = _site["lat"]
MAP_CENTER_LON  = _site["lon"]
MAP_CENTER_ZOOM = _site["zoom"]

# Tile cache directory — used by setup_offline.py for offline pre-download
# and referenced in map.html for offline tile fallback.
TILE_CACHE_DIR     = "./tile_cache"

# -----------------------------------------------------------------------------
#  UI SETTINGS
# -----------------------------------------------------------------------------

APP_TITLE          = "TRES Titan Ground Station"
DARK_MODE          = True       # Dark theme (recommended for outdoor use)
VIDEO_REFRESH_MS   = 33         # ~30 fps video widget refresh
TELEM_REFRESH_MS   = 100        # Telemetry panel refresh rate

# -----------------------------------------------------------------------------
#  TEST MODE DEFAULTS
# -----------------------------------------------------------------------------

# Whether to start in test mode (should be False for flight day)
TEST_MODE_DEFAULT  = False

# Full system test duration in seconds
# Fires VTX, RunCam, 10Hz telemetry, and Stage 2 solenoid simultaneously
# for battery evaluation and system validation
FULL_SYS_TEST_DURATION_S = 90

# Stage names for display
STAGE_NAMES = {1: "Stage 1 (Booster)", 2: "Stage 2 (Sustainer)"}

# -----------------------------------------------------------------------------
#  SHOWCASE MODE
#  In showcase mode one video panel plays a pre-rendered looping video file
#  instead of a live capture feed. The other panel keeps the live feed.
#  Set SHOWCASE_VIDEO_PATH to the absolute path of your video file.
#  Set SHOWCASE_STAGE to 1 or 2 — this stage's panel plays the file.
#  The opposite stage keeps its live capture card feed.
# -----------------------------------------------------------------------------

SHOWCASE_MODE_DEFAULT   = False         # Start in showcase mode on launch
SHOWCASE_STAGE          = 1            # Which stage panel plays the file (1 or 2)
SHOWCASE_VIDEO_PATH     = "/home/titanrocket/TitanRocketAvionics/GroundStation/showcase"
                                        # Path to a video file OR a folder — if a folder,
                                        # the first .mp4/.avi/.mkv/.mov file found is played.
SHOWCASE_FRAME_DELAY_MS = 33           # ~30 fps playback (match your video's frame rate)
