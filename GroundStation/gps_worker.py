# =============================================================================
#  TRES Titan Ground Station — gps_worker.py
#  QThread that reads the Featherweight GPS Ground Station V2 USB serial
#  output and parses ASCII "@"-prefixed space-delimited packets.
#
#  CONFIRMED serial format (captured 2026-04-30 at 115200 baud):
#    Baud rate:  115200
#    Format:     space-delimited,  "@ TYPE  LEN  FLAGS  TIME  ..."
#    Examples seen:
#      @ GS_STAT  189 00 HH:MM:SS.mmm CRC_OK TRACKER_NAME PkRx N PkSnt N
#                 p_RSSI N AckRx N AckSnt N a_RSSI N ... SF N f FREQ_HZ CRC: XXXX
#      @ BATT_BLE 067 0000 00 00 HH:MM:SS.mmm BATT_MV BLE+/- TEMP degC CRC: XXXX
#
#  Tracker position packet type name is unconfirmed — the GS_STAT "PkRx" counter
#  shows 0 meaning no tracker LoRa packets have been received yet.  When PkRx > 0,
#  a new packet type will appear in the debug console.  Search for the line that
#  contains LAT/LON values and update _parse_tracker() accordingly.
# =============================================================================

import serial
from PyQt5.QtCore import QThread, pyqtSignal


class FWGPSWorker(QThread):
    """
    Reads the Featherweight GPS Ground Station V2 USB serial output.

    Signals:
      position_update(tracker_name, lat, lon, alt_ft, vert_vel, horiz_vel)
      lost_rocket(tracker_name, lat, lon, alt_ft)
      gs_status(rssi, battery_v, packet_rate)
      connection_changed(bool)
      raw_log(str)
    """

    position_update    = pyqtSignal(str, float, float, float, float, float)
    lost_rocket        = pyqtSignal(str, float, float, float)
    gs_status          = pyqtSignal(float, float, float)
    connection_changed = pyqtSignal(bool)
    raw_log            = pyqtSignal(str)

    def __init__(self, port: str, baud: int, parent=None):
        super().__init__(parent)
        self.port      = port
        self.baud      = baud
        self._running  = False
        self._last_batt_v = 0.0

    def stop(self):
        self._running = False

    def run(self):
        self._running = True
        while self._running:
            try:
                with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
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
            try:
                line = raw.decode("ascii", errors="replace").strip()
            except Exception:
                continue
            if not line.startswith("@"):
                continue
            self.raw_log.emit(f"[FW GPS] {line}")
            self._parse_line(line)

    # ------------------------------------------------------------------

    def _parse_line(self, line: str):
        """
        Parse one space-delimited "@ TYPE ..." packet.

        Actual format confirmed by serial capture (2026-04-30):
          tokens[0] = '@'
          tokens[1] = packet type string (GS_STAT, BATT_BLE, TRACKER?, ...)
          tokens[2] = payload length
          remaining = type-specific fields
        """
        tokens = line.split()
        if len(tokens) < 3:
            return
        ptype = tokens[1]

        try:
            if ptype == "GS_STAT":
                self._parse_gs_status(tokens)
            elif ptype == "BATT_BLE":
                self._parse_batt_ble(tokens)
            elif ptype == "TRACKER":
                # ── UNCONFIRMED — update if tracker sends a different type name ──
                self._parse_tracker(tokens)
            elif ptype in ("LOST", "FND"):
                self._parse_lost_rocket(tokens)
            else:
                # Log every unknown type so the tracker position packet type
                # can be identified when PkRx > 0 on the GS_STAT line.
                self.raw_log.emit(
                    f"[FW GPS] New packet type '{ptype}' — check for LAT/LON fields: "
                    f"{line[:120]}")
        except (IndexError, ValueError) as e:
            self.raw_log.emit(f"[FW GPS] Parse error '{ptype}': {e}")

    # ------------------------------------------------------------------

    def _parse_gs_status(self, tokens: list):
        """
        @ GS_STAT LEN FLAGS TIME CRC_OK NAME PkRx N PkSnt N p_RSSI N
                  AckRx N AckSnt N a_RSSI N ... SF N f FREQ_HZ CRC: XXXX
        Uses keyword lookup so field order changes don't break parsing.
        """
        try:
            rssi_idx = tokens.index("p_RSSI") + 1
            rssi = float(tokens[rssi_idx])
        except (ValueError, IndexError):
            rssi = 0.0

        try:
            pkt_idx = tokens.index("PkRx") + 1
            pkt_rate = float(tokens[pkt_idx])
        except (ValueError, IndexError):
            pkt_rate = 0.0

        self.gs_status.emit(rssi, self._last_batt_v, pkt_rate)

    def _parse_batt_ble(self, tokens: list):
        """
        @ BATT_BLE LEN FLAGS S1 S2 TIME BATT_MV BLE+/- TEMP degC CRC: XXXX
        tokens[7] = battery millivolts (e.g. 4126 → 4.126 V)
        """
        try:
            self._last_batt_v = float(tokens[7]) / 1000.0
        except (ValueError, IndexError):
            pass

    def _parse_tracker(self, tokens: list):
        """
        Tracker position packet — FIELD INDICES UNCONFIRMED.
        Update these once a live @ TRACKER packet is captured with PkRx > 0.
        The tracker name, lat, lon, alt fields need to be verified.

        To find the right indices: look in the debug console for the line
        logged as "New packet type '...' — check for LAT/LON fields" and
        count the space-delimited tokens until you find decimal degree values.
        """
        # ── UNCONFIRMED indices — verify against a live packet ──────
        IDX_NAME      = 6
        IDX_LAT       = 7
        IDX_LON       = 8
        IDX_ALT_FT    = 9
        IDX_VERT_VEL  = 10
        IDX_HORIZ_VEL = 11
        # ────────────────────────────────────────────────────────────

        name = tokens[IDX_NAME].strip()
        try:
            lat       = float(tokens[IDX_LAT])
            lon       = float(tokens[IDX_LON])
            alt_ft    = float(tokens[IDX_ALT_FT])
            vert_vel  = float(tokens[IDX_VERT_VEL])
            horiz_vel = float(tokens[IDX_HORIZ_VEL])
        except (ValueError, IndexError):
            self.raw_log.emit(f"[FW GPS] {name}: NO FIX or wrong indices")
            return

        self.position_update.emit(name, lat, lon, alt_ft, vert_vel, horiz_vel)

    def _parse_lost_rocket(self, tokens: list):
        """LOST / FND packet — indices unconfirmed, update when seen."""
        IDX_NAME   = 6
        IDX_LAT    = 7
        IDX_LON    = 8
        IDX_ALT_FT = 9

        name   = tokens[IDX_NAME].strip()
        lat    = float(tokens[IDX_LAT])
        lon    = float(tokens[IDX_LON])
        alt_ft = float(tokens[IDX_ALT_FT])

        self.lost_rocket.emit(name, lat, lon, alt_ft)
        self.raw_log.emit(
            f"[FW GPS] LOST/FND: {name} @ {lat:.5f},{lon:.5f} {alt_ft:.0f}ft")
