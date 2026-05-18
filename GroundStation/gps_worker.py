# =============================================================================
#  TRES Titan Ground Station — gps_worker.py
#  QThread that reads the Featherweight GPS Ground Station V2 USB serial
#  output and parses ASCII "@"-prefixed position packets.
#
#  IMPORTANT — VERIFY AGAINST APPENDIX A OF YOUR FEATHERWEIGHT MANUAL
#  ────────────────────────────────────────────────────────────────────
#  The exact field order and count for each packet type are documented
#  in "Appendix A: Serial commands and data for version 2 ground stations"
#  in your GPS Tracker User's Manual (featherweightaltimeters.com).
#
#  This parser implements the tracker position packet (type "T") based
#  on available documentation and community reports. If any field below
#  produces garbage data, open a terminal at 57600 baud and compare the
#  raw "@T" lines against the Appendix A column listing.
#
#  Known serial format summary:
#    - Binary "FWT"-prefixed packets: internal Bluetooth bridge data.
#      Silently discarded — do not attempt to parse.
#    - ASCII "@"-prefixed packets: human-readable, CR/LF terminated.
#      Format: @[TYPE],[LEN],[DATE],[TIME],[FIELD1],[FIELD2],...
#    - 14 packet types exist; this parser handles types T (tracker
#      position), G (ground station status), and L (lost rocket).
# =============================================================================

import math
import serial
from PyQt5.QtCore import QThread, pyqtSignal


# Packet type constants (single character after the "@")
FW_TYPE_TRACKER  = "T"    # Tracker position data
FW_TYPE_GS       = "G"    # Ground station status
FW_TYPE_LOST     = "L"    # Lost-rocket relay data


class FWGPSWorker(QThread):
    """
    Reads the Featherweight GPS Ground Station V2 USB serial output.

    Signals:
      position_update(tracker_name, lat, lon, alt_ft, vert_vel, horiz_vel)
        Emitted once per second when a valid tracker position packet arrives.

      lost_rocket(tracker_name, lat, lon, alt_ft)
        Emitted when a lost-rocket relay packet is received.

      gs_status(rssi, battery_v, packet_rate)
        Emitted on ground station status updates.

      connection_changed(bool)
        True when port opens, False when it closes or errors.

      raw_log(str)
        Raw line text for the debug console.
    """

    position_update    = pyqtSignal(str, float, float, float, float, float)
    lost_rocket        = pyqtSignal(str, float, float, float)
    gs_status          = pyqtSignal(float, float, float)
    connection_changed = pyqtSignal(bool)
    raw_log            = pyqtSignal(str)

    def __init__(self, port: str, baud: int, parent=None):
        super().__init__(parent)
        self.port     = port
        self.baud     = baud
        self._running = False
        self._last_valid_lat: float | None = None
        self._last_valid_lon: float | None = None

    def stop(self):
        self._running = False

    def run(self):
        self._running = True
        while self._running:
            try:
                with serial.Serial(self.port, self.baud,
                                   timeout=1.0) as ser:
                    self.connection_changed.emit(True)
                    self.raw_log.emit(
                        f"[FW GPS] Connected on {self.port} @ {self.baud}")
                    self._read_loop(ser)
            except serial.SerialException as e:
                self.connection_changed.emit(False)
                self.raw_log.emit(
                    f"[FW GPS] Port error: {e}. Retrying in 3s…")
                self.msleep(3000)

    # ------------------------------------------------------------------

    def _read_loop(self, ser: serial.Serial):
        while self._running:
            try:
                raw = ser.readline()
            except serial.SerialException:
                break

            if not raw:
                continue

            # Discard binary FWT packets silently
            if raw[:3] == b"FWT":
                continue

            # Only process ASCII "@"-prefixed packets
            try:
                line = raw.decode("ascii", errors="replace").strip()
            except Exception:
                continue

            if not line.startswith("@"):
                continue

            self.raw_log.emit(f"[FW GPS] {line}")
            self._parse_line(line)

    # ------------------------------------------------------------------

    def _haversine_distance_m(self, lat1: float, lon1: float,
                               lat2: float, lon2: float) -> float:
        """Great-circle distance in metres between two lat/lon points."""
        if lat1 is None or lon1 is None or lat2 is None or lon2 is None:
            return 0.0
        R    = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # ------------------------------------------------------------------

    def _parse_line(self, line: str):
        """
        Parse one "@TYPE,LEN,DATE,TIME,...fields..." packet line.

        ── APPENDIX A FIELD MAPPING ────────────────────────────────
        This mapping is based on Featherweight documentation and
        community-reported field order. Verify the column numbers
        below against Appendix A of your manual before flight.

        Tracker position packet "@T":
          [0]  @T         packet type
          [1]  LEN        payload length
          [2]  DATE       YYYY-MM-DD
          [3]  TIME       HH:MM:SS.sss
          [4]  NAME       tracker name string
          [5]  CHAN       channel number
          [6]  LAT        decimal degrees (positive = N)
          [7]  LON        decimal degrees (positive = E)
          [8]  ALT_FT     altitude in feet (MSL)
          [9]  VERT_VEL   vertical velocity ft/s
          [10] HORIZ_VEL  horizontal velocity ft/s
          [11] BEARING    bearing from ground station, degrees
          [12] SATS       satellite count
          [13] RSSI       LoRa RSSI dBm
          [14] BATT_V     tracker battery voltage
          [15] GS_RSSI    ground station RSSI
          ... additional fields may follow per firmware version

        ── HOW TO VERIFY ───────────────────────────────────────────
        Run this in a terminal before the first use:
          python3 -c "
          import serial
          s = serial.Serial('/dev/ttyACM0', 57600, timeout=2)
          for _ in range(30):
              l = s.readline().decode('ascii', errors='replace').strip()
              if l.startswith('@T'):
                  print(l)
          s.close()
          "
        Then compare each comma-separated column to the Appendix A table.
        Update the field indices below to match your firmware version.
        """

        parts = line.split(",")
        ptype = parts[0][1:]   # Strip the "@" prefix

        try:
            if ptype == FW_TYPE_TRACKER:
                self._parse_tracker(parts)
            elif ptype == FW_TYPE_GS:
                self._parse_gs_status(parts)
            elif ptype == FW_TYPE_LOST:
                self._parse_lost_rocket(parts)
            # All other packet types are silently ignored

        except (IndexError, ValueError) as e:
            self.raw_log.emit(
                f"[FW GPS] Parse error on '{line[:60]}': {e}")

    def _parse_tracker(self, parts: list):
        """
        Parse tracker position packet.
        VERIFY FIELD INDICES AGAINST APPENDIX A OF YOUR MANUAL.
        """
        # ── FIELD INDEX CONSTANTS ──────────────────────────────────
        # Change these numbers if your firmware version differs.
        IDX_NAME      = 4
        IDX_LAT       = 6
        IDX_LON       = 7
        IDX_ALT_FT    = 8
        IDX_VERT_VEL  = 9
        IDX_HORIZ_VEL = 10
        # ──────────────────────────────────────────────────────────

        name = parts[IDX_NAME].strip()

        try:
            lat       = float(parts[IDX_LAT])
            lon       = float(parts[IDX_LON])
            alt_ft    = float(parts[IDX_ALT_FT])
            vert_vel  = float(parts[IDX_VERT_VEL])
            horiz_vel = float(parts[IDX_HORIZ_VEL])
        except (ValueError, IndexError):
            self.raw_log.emit(f"[FW GPS] {name}: NO FIX — waiting for satellites")
            return

        if self._last_valid_lat is not None:
            distance_m = self._haversine_distance_m(
                self._last_valid_lat, self._last_valid_lon, lat, lon)
            MAX_JUMP_M = 152.4  # 500 feet
            if distance_m > MAX_JUMP_M:
                self.raw_log.emit(
                    f"[FW GPS] REJECTED noise spike: {name} jumped {distance_m:.0f}m "
                    f"from last valid position ({self._last_valid_lat:.5f},"
                    f"{self._last_valid_lon:.5f}) → ({lat:.5f},{lon:.5f})")
                return

        self._last_valid_lat = lat
        self._last_valid_lon = lon
        self.position_update.emit(name, lat, lon, alt_ft, vert_vel, horiz_vel)

    def _parse_gs_status(self, parts: list):
        """
        Parse ground station status packet.
        Field order: verify against Appendix A.
        """
        IDX_RSSI    = 4
        IDX_BATT    = 5
        IDX_PKT_RT  = 6

        rssi      = float(parts[IDX_RSSI])
        batt_v    = float(parts[IDX_BATT])
        pkt_rate  = float(parts[IDX_PKT_RT])

        self.gs_status.emit(rssi, batt_v, pkt_rate)

    def _parse_lost_rocket(self, parts: list):
        """
        Parse lost-rocket relay packet.
        """
        IDX_NAME   = 4
        IDX_LAT    = 5
        IDX_LON    = 6
        IDX_ALT_FT = 7

        name   = parts[IDX_NAME].strip()
        lat    = float(parts[IDX_LAT])
        lon    = float(parts[IDX_LON])
        alt_ft = float(parts[IDX_ALT_FT])

        self._last_valid_lat = None
        self._last_valid_lon = None
        self.lost_rocket.emit(name, lat, lon, alt_ft)
        self.raw_log.emit(
            f"[FW GPS] LOST ROCKET RELAY: {name} @ {lat:.5f},{lon:.5f} "
            f"{alt_ft:.0f}ft")
