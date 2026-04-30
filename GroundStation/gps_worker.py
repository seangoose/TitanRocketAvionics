# =============================================================================
#  TRES Titan Ground Station — gps_worker.py
#  QThread that reads the Featherweight GPS Ground Station V2 USB serial
#  output and parses ASCII "@"-prefixed space-delimited packets.
#
#  CONFIRMED serial format (captured 2026-04-30 at 115200 baud):
#    Baud rate:  115200
#    Format:     space-delimited,  "@ TYPE  LEN  FLAGS  TIME  ..."
#
#  Packet types confirmed by live capture with CSUFTRKERS2:
#    GPS_STAT  — tracker position (Alt, lt, ln, Vel keywords)
#    RX_NOMTK  — RF link stats (RSSI, SNR, trk_B_V)
#    GS_STAT   — ground station summary (p_RSSI, PkRx)
#    BATT_BLE  — ground station battery/BLE status
#    TX_STAT   — uplink transmission info (ignored)
#
#  GPS_STAT example:
#    @ GPS_STAT 203 0000 00 00 TIME CRC_OK TRK NAME Alt NNNNNN lt +LAT ln +LON
#               Vel +VVEL +HVEL +SPARE Fix N # SATS ...
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
            if ptype == "GPS_STAT":
                self._parse_gps_stat(tokens)
            elif ptype == "RX_NOMTK":
                self._parse_rx_nomtk(tokens)
            elif ptype == "GS_STAT":
                self._parse_gs_status(tokens)
            elif ptype == "BATT_BLE":
                self._parse_batt_ble(tokens)
            elif ptype in ("LOST", "FND"):
                self._parse_lost_rocket(tokens)
            # TX_STAT and other informational types silently ignored
        except (IndexError, ValueError) as e:
            self.raw_log.emit(f"[FW GPS] Parse error '{ptype}': {e}")

    # ------------------------------------------------------------------

    def _parse_gps_stat(self, tokens: list):
        """
        @ GPS_STAT 203 0000 00 00 TIME CRC_OK TRK NAME Alt NNNNNN
                   lt +LAT ln +LON Vel +VVEL +HVEL +SPARE Fix N # SATS ...

        Confirmed field indices (live capture 2026-04-30):
          [9]  tracker name
          [11] altitude ft  (after keyword 'Alt' at [10])
          [13] latitude     (after keyword 'lt'  at [12])
          [15] longitude    (after keyword 'ln'  at [14])
          [17] vert vel     (after keyword 'Vel' at [16])
          [18] horiz vel
        """
        try:
            name   = tokens[tokens.index("TRK") + 1]
            lat    = float(tokens[tokens.index("lt")  + 1])
            lon    = float(tokens[tokens.index("ln")  + 1])
            alt_ft = float(tokens[tokens.index("Alt") + 1])
            vi     = tokens.index("Vel") + 1
            vert_vel  = float(tokens[vi])
            horiz_vel = float(tokens[vi + 1])
        except (ValueError, IndexError) as e:
            self.raw_log.emit(f"[FW GPS] GPS_STAT parse error: {e}")
            return

        self.raw_log.emit(
            f"[FW GPS] POS {name}  {lat:.5f},{lon:.5f}  {alt_ft:.0f}ft")
        self.position_update.emit(name, lat, lon, alt_ft, vert_vel, horiz_vel)

    def _parse_rx_nomtk(self, tokens: list):
        """
        @ RX_NOMTK ... CRC_OK Rx NomTrk NAME PkRx N PkTx N RSSI N SNR N
                    ... trk_B_V MV ...

        Emits gs_status with RF link RSSI, tracker battery, and packet count.
        """
        try:
            rssi_idx = tokens.index("RSSI") + 1
            rssi = float(tokens[rssi_idx])
        except (ValueError, IndexError):
            rssi = 0.0

        try:
            pkt_idx = tokens.index("PkRx") + 1
            pkt_rate = float(tokens[pkt_idx])
        except (ValueError, IndexError):
            pkt_rate = 0.0

        try:
            batt_idx = tokens.index("trk_B_V") + 1
            self._last_batt_v = float(tokens[batt_idx]) / 1000.0
        except (ValueError, IndexError):
            pass

        self.gs_status.emit(rssi, self._last_batt_v, pkt_rate)

    def _parse_gs_status(self, tokens: list):
        """
        @ GS_STAT ... p_RSSI N ... PkRx N ...
        Secondary status — emits gs_status if RX_NOMTK not present.
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

    def _parse_lost_rocket(self, tokens: list):
        """LOST / FND packet — keyword-based, mirrors GPS_STAT layout."""
        try:
            name   = tokens[tokens.index("TRK") + 1]
            lat    = float(tokens[tokens.index("lt")  + 1])
            lon    = float(tokens[tokens.index("ln")  + 1])
            alt_ft = float(tokens[tokens.index("Alt") + 1])
        except (ValueError, IndexError) as e:
            self.raw_log.emit(f"[FW GPS] LOST parse error: {e}")
            return

        self.lost_rocket.emit(name, lat, lon, alt_ft)
        self.raw_log.emit(
            f"[FW GPS] LOST/FND: {name} @ {lat:.5f},{lon:.5f} {alt_ft:.0f}ft")
