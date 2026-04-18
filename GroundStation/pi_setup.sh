#!/usr/bin/env bash
# =============================================================================
#  TRES Titan Ground Station — Raspberry Pi One-Time Setup Script
#
#  Run once at home on WiFi before driving to the launch site.
#  Safe to re-run multiple times (idempotent).
#
#  Usage:
#      cd /path/to/repo/GroundStation
#      bash pi_setup.sh
#
#  Requires: Raspberry Pi OS Bookworm (Debian 12), internet connection
# =============================================================================
set -euo pipefail

# ── Environment ───────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if ! REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"; then
    echo "ERROR: Cannot determine repository root." >&2
    echo "       Make sure this script is run from inside the git repository." >&2
    exit 1
fi
GS_DIR="$SCRIPT_DIR"
CURRENT_USER="$(whoami)"
SERVICE_NAME="tres-groundstation"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
UDEV_RULES="/etc/udev/rules.d/99-tres-titan.rules"

# ── Helpers ───────────────────────────────────────────────────────────────────
print_step() {
    echo ""
    echo "════════════════════════════════════════════════════════"
    printf "  %s\n" "$*"
    echo "════════════════════════════════════════════════════════"
}
print_ok()   { printf "  \xe2\x9c\x93  %s\n" "$*"; }
print_warn() { printf "  \xe2\x9a\xa0  %s\n" "$*"; }
die()        { printf "\nFATAL: %s\n" "$*" >&2; exit 1; }

# ── Step 1: System packages ────────────────────────────────────────────────────
step_apt() {
    print_step "STEP 1/7 — System package installation"
    echo "  Running apt-get update..."
    sudo apt-get update -qq || die "apt-get update failed"
    print_ok "apt-get update complete"

    echo "  Installing packages..."
    sudo apt-get install -y \
        python3-pyqt5 \
        python3-pyqt5.qtwebengine \
        python3-pyqt5.qtmultimedia \
        libgl1-mesa-glx \
        libglib2.0-0 \
        v4l-utils \
        python3-pip \
        git \
        || die "apt-get install failed — check the output above"
    print_ok "All system packages installed"
}

# ── Step 2: Python dependencies ────────────────────────────────────────────────
step_pip() {
    print_step "STEP 2/7 — Python dependency installation"

    local py_major py_minor
    py_major="$(python3 -c 'import sys; print(sys.version_info.major)')"
    py_minor="$(python3 -c 'import sys; print(sys.version_info.minor)')"
    echo "  Python ${py_major}.${py_minor} detected"

    if [ "$py_major" -gt 3 ] || { [ "$py_major" -eq 3 ] && [ "$py_minor" -ge 11 ]; }; then
        echo "  Python 3.11+ — using --break-system-packages"
        pip3 install -r "$GS_DIR/requirements.txt" --break-system-packages \
            || die "pip3 install failed"
    else
        pip3 install -r "$GS_DIR/requirements.txt" \
            || die "pip3 install failed"
    fi
    print_ok "Python dependencies installed"
}

# ── Step 3: Offline map setup ──────────────────────────────────────────────────
step_offline() {
    print_step "STEP 3/7 — Offline map setup (Leaflet + OSM tile cache)"
    echo "  This may take several minutes depending on internet speed."
    echo "  Downloading Leaflet library and caching OSM map tiles for the FAR site..."
    echo ""
    python3 "$GS_DIR/setup_offline.py" || die "setup_offline.py failed"
    print_ok "Offline map setup complete"
}

# ── Step 4: udev rules for persistent USB device names ────────────────────────

DETECTED_DEV=""
DETECTED_SERIAL=""

_identify_device() {
    local label="$1"
    local after count dev serial

    DETECTED_DEV=""
    DETECTED_SERIAL=""

    while true; do
        echo ""
        echo "  ─────────────────────────────────────────────────────"
        printf "  Unplug ALL USB devices, then plug in ONLY the:\n"
        printf "    [ %s ]\n" "$label"
        printf "  and press Enter.\n"
        echo "  ─────────────────────────────────────────────────────"
        read -r -p "  Press Enter when ready... " || true

        echo "  Waiting 2 seconds for USB enumeration..."
        sleep 2

        after="$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sort || true)"

        if [ -z "$after" ]; then
            count=0
        else
            count="$(echo "$after" | wc -l | tr -d ' ')"
        fi

        if [ "$count" -eq 0 ]; then
            echo "  No USB serial device detected. Ensure the device is plugged in and try again."
            continue
        elif [ "$count" -gt 1 ]; then
            printf "  %d devices detected — unplug all extras and try again with ONLY the %s:\n" \
                "$count" "$label"
            echo "$after" | sed 's/^/    /'
            continue
        fi

        dev="$(echo "$after" | head -1)"
        printf "  Device detected: %s\n" "$dev"

        serial="$(udevadm info --name="$dev" --attribute-walk 2>/dev/null \
            | grep -m1 'ATTRS{serial}==' \
            | sed "s/.*ATTRS{serial}==\"\\(.*\\)\".*/\\1/" \
            || true)"

        if [ -z "$serial" ] || [ "$serial" = "ATTRS{serial}" ]; then
            print_warn "No unique serial found for $dev — persistent naming may not work"
            serial="NO_SERIAL_$(basename "$dev")"
        fi

        DETECTED_DEV="$dev"
        DETECTED_SERIAL="$serial"
        print_ok "Serial number: $serial"
        return 0
    done
}

step_udev() {
    print_step "STEP 4/7 — udev rules for persistent USB device names"

    echo "  Without persistent names, serial port assignments shift every time"
    echo "  devices are plugged in a different order, causing the ground station"
    echo "  to silently fail on the wrong ports. udev rules fix each device to a"
    echo "  permanent symlink by matching the device's unique USB serial number."
    echo ""
    echo "  Symlinks that will be created:"
    echo "    /dev/tres_m0_s1   — M0 Stage 1 LoRa radio"
    echo "    /dev/tres_m0_s2   — M0 Stage 2 LoRa radio"
    echo "    /dev/tres_fw_gps  — Featherweight GPS Ground Station"

    if [ -f "$UDEV_RULES" ]; then
        echo ""
        print_warn "udev rules file already exists: $UDEV_RULES"
        echo "  Current contents:"
        cat "$UDEV_RULES" | sed 's/^/    /'
        echo ""
        local resp=""
        read -r -p "  Regenerate (re-identify all devices)? [y/N] " resp || true
        if [[ ! "${resp:-N}" =~ ^[Yy] ]]; then
            print_ok "Keeping existing udev rules"
            sudo udevadm control --reload-rules && sudo udevadm trigger || true
            return 0
        fi
    fi

    echo ""
    echo "  You will be prompted to plug in each device individually."

    local m0_s1_serial m0_s2_serial gps_serial

    _identify_device "M0 Stage 1 LoRa Radio (Adafruit M0 with RFM95W)"
    m0_s1_serial="$DETECTED_SERIAL"

    _identify_device "M0 Stage 2 LoRa Radio (Adafruit M0 with RFM95W)"
    m0_s2_serial="$DETECTED_SERIAL"

    _identify_device "Featherweight GPS Ground Station V2"
    gps_serial="$DETECTED_SERIAL"

    echo ""
    echo "  Writing udev rules to $UDEV_RULES..."

    sudo tee "$UDEV_RULES" > /dev/null << UDEVRULES
# TRES Titan — Persistent USB device names
# Generated by pi_setup.sh
SUBSYSTEM=="tty", ATTRS{serial}=="$m0_s1_serial", SYMLINK+="tres_m0_s1"
SUBSYSTEM=="tty", ATTRS{serial}=="$m0_s2_serial", SYMLINK+="tres_m0_s2"
SUBSYSTEM=="tty", ATTRS{serial}=="$gps_serial", SYMLINK+="tres_fw_gps"
UDEVRULES

    sudo udevadm control --reload-rules && sudo udevadm trigger \
        || die "udevadm reload/trigger failed"

    echo ""
    print_ok "udev rules written to $UDEV_RULES"
    print_ok "M0 S1  serial: $m0_s1_serial  →  /dev/tres_m0_s1"
    print_ok "M0 S2  serial: $m0_s2_serial  →  /dev/tres_m0_s2"
    print_ok "GPS    serial: $gps_serial  →  /dev/tres_fw_gps"
}

# ── Step 5: Update config.py ───────────────────────────────────────────────────
step_config() {
    print_step "STEP 5/7 — Update config.py with persistent device names"

    local config_path="$GS_DIR/config.py"
    [ -f "$config_path" ] || die "config.py not found at $config_path"

    echo "  Before:"
    grep -E 'M0_S1_PORT|M0_S2_PORT|FW_GPS_PORT' "$config_path" | sed 's/^/    /'

    python3 - "$config_path" << 'PYEOF'
import re, sys

config_path = sys.argv[1]
with open(config_path, 'r') as f:
    content = f.read()

content = re.sub(r'(M0_S1_PORT\s*=\s*)"[^"]*"', r'\1"/dev/tres_m0_s1"', content)
content = re.sub(r'(M0_S2_PORT\s*=\s*)"[^"]*"', r'\1"/dev/tres_m0_s2"', content)
content = re.sub(r'(FW_GPS_PORT\s*=\s*)"[^"]*"', r'\1"/dev/tres_fw_gps"', content)

with open(config_path, 'w') as f:
    f.write(content)
PYEOF

    echo "  After:"
    grep -E 'M0_S1_PORT|M0_S2_PORT|FW_GPS_PORT' "$config_path" | sed 's/^/    /'
    print_ok "config.py updated with persistent port names"
}

# ── Step 6: Systemd auto-launch service ───────────────────────────────────────
step_systemd() {
    print_step "STEP 6/7 — Systemd auto-launch service"

    [ -f "$GS_DIR/main.py" ] || die "main.py not found at $GS_DIR/main.py"

    local xauth="/home/${CURRENT_USER}/.Xauthority"

    echo "  Writing service file to $SERVICE_FILE..."
    sudo tee "$SERVICE_FILE" > /dev/null << SVCEOF
[Unit]
Description=TRES Titan Ground Station
After=graphical.target network.target
Wants=graphical.target

[Service]
Type=simple
User=${CURRENT_USER}
Group=${CURRENT_USER}
WorkingDirectory=${GS_DIR}
Environment=DISPLAY=:0
Environment=XAUTHORITY=${xauth}
Environment=QT_QPA_PLATFORM=xcb
Environment=QTWEBENGINE_CHROMIUM_FLAGS=--no-sandbox
ExecStart=/usr/bin/python3 ${GS_DIR}/main.py
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=graphical.target
SVCEOF

    sudo systemctl daemon-reload || die "systemctl daemon-reload failed"
    sudo systemctl enable "${SERVICE_NAME}.service" || die "systemctl enable failed"

    print_ok "Service file written: $SERVICE_FILE"
    print_ok "Service enabled — tres-groundstation will auto-launch on next boot"
    echo ""
    echo "  To start manually without rebooting:"
    echo "    sudo systemctl start ${SERVICE_NAME}"
    echo ""
    echo "  To watch the launch log:"
    echo "    journalctl -u ${SERVICE_NAME} -f"
}

# ── Step 7: Final verification ────────────────────────────────────────────────
step_verify() {
    print_step "STEP 7/7 — Final verification"

    echo "  Repository root:   $REPO_ROOT"
    echo "  Ground station:    $GS_DIR"
    echo "  Running as user:   $CURRENT_USER"
    echo ""

    echo "  USB device symlinks (may need a replug or reboot to appear):"
    for symlink in /dev/tres_m0_s1 /dev/tres_m0_s2 /dev/tres_fw_gps; do
        if [ -L "$symlink" ]; then
            print_ok "$symlink  →  $(readlink "$symlink")"
        else
            print_warn "$symlink not yet present (replug device or reboot)"
        fi
    done

    echo ""
    echo "  Systemd service:"
    local svc_state
    svc_state="$(systemctl is-enabled "${SERVICE_NAME}.service" 2>/dev/null || echo "not-found")"
    if [ "$svc_state" = "enabled" ]; then
        print_ok "${SERVICE_NAME}.service is ENABLED"
    else
        print_warn "${SERVICE_NAME}.service state: $svc_state (expected: enabled)"
    fi

    echo ""
    echo "════════════════════════════════════════════════════════"
    echo "  Setup complete. Reboot the Pi to verify auto-launch."
    echo "  Run 'journalctl -u tres-groundstation -f' to watch the launch log."
    echo "════════════════════════════════════════════════════════"
}

# ── Entry point ───────────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════"
echo "  TRES Titan Ground Station — Pi Setup"
echo "  Repository:  $REPO_ROOT"
echo "  User:        $CURRENT_USER"
echo "════════════════════════════════════════════════════════"

step_apt
step_pip
step_offline
step_udev
step_config
step_systemd
step_verify
