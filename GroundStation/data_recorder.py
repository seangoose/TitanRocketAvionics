# =============================================================================
#  TRES Titan Ground Station — data_recorder.py
#  Thread-safe CSV recorder for all received telemetry data.
#  Records FlightPackets, StatusPackets, and Featherweight GPS positions
#  to a single timestamped CSV file per session.
#
#  Usage:
#      recorder = DataRecorder()
#      recorder.start_recording("session_name")   # opens file
#      recorder.record_flight(stage, data_dict)   # call from any thread
#      recorder.record_gps(name, lat, lon, ...)   # call from any thread
#      recorder.stop_recording()                  # flushes and closes
# =============================================================================

import os
import csv
import threading
from datetime import datetime


# Output directory — relative to the ground station folder
RECORDINGS_DIR = "recordings"


class DataRecorder:
    """
    Thread-safe CSV recorder.  All public methods are safe to call
    from QThread workers.  Uses a threading.Lock (not a QMutex) so it
    works regardless of Qt context.

    CSV format: each row has a 'source' column identifying the data type:
      FLIGHT   — FlightPacket data
      STATUS   — StatusPacket data
      GPS      — Featherweight GPS position
      EVENT    — human-readable event marker (e.g. "recording started")
    All rows share a common prefix: wall_time_iso, elapsed_s, source, stage.
    Unused columns for a given row type are left blank.
    """

    # Column definitions in order
    COLUMNS = [
        # Common
        "wall_time_iso",      # ISO-8601 wall clock at time of receipt
        "elapsed_s",          # Seconds since recording started
        "source",             # FLIGHT | STATUS | GPS | EVENT
        "stage",              # 1 or 2 (blank for GPS/EVENT)
        # FlightPacket fields
        "timestamp_ms",
        "altitude_ft",
        "velocity_fps",
        "accel_thrust_ms2",
        "accel_lateral_ms2",
        "gyro_roll_rads",
        "gyro_pitch_rads",
        "gyro_yaw_rads",
        "temperature_c",
        "rssi_dbm",
        "flight_state",
        "fault_flags_hex",
        # StatusPacket fields (reuse some columns above, add mode fields)
        "lora_freq_mhz",
        "vtx_freq_mhz",
        "vtx_power_index",
        "telem_rate_hz",
        "video_enabled",
        "test_mode",
        "last_cmd_hex",
        "last_cmd_result",
        # GPS fields
        "tracker_name",
        "gps_lat",
        "gps_lon",
        "gps_alt_ft",
        "gps_vert_vel_fps",
        "gps_horiz_vel_fps",
        # Event
        "event_message",
    ]

    def __init__(self, recordings_dir: str = RECORDINGS_DIR):
        self._dir      = recordings_dir
        self._file     = None
        self._writer   = None
        self._lock     = threading.Lock()
        self._active   = False
        self._start_ts = None
        self._filepath = None

    # ------------------------------------------------------------------
    #  Public API
    # ------------------------------------------------------------------

    @property
    def is_recording(self) -> bool:
        return self._active

    @property
    def current_file(self) -> str:
        """Path of the currently open recording file, or empty string."""
        return self._filepath or ""

    def start_recording(self, session_name: str = "") -> str:
        """
        Open a new CSV recording file.
        Returns the full path of the created file.
        Raises RuntimeError if already recording.
        """
        with self._lock:
            if self._active:
                raise RuntimeError("Recording already in progress")

            os.makedirs(self._dir, exist_ok=True)

            ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
            tag = f"_{session_name}" if session_name.strip() else ""
            filename = f"TRES_{ts}{tag}_flight.csv"
            self._filepath = os.path.join(self._dir, filename)

            self._file     = open(self._filepath, "w", newline="",
                                  encoding="utf-8")
            self._writer   = csv.DictWriter(self._file,
                                            fieldnames=self.COLUMNS,
                                            extrasaction="ignore")
            self._writer.writeheader()
            self._start_ts = datetime.now()
            self._active   = True

            self._write_event("Recording started")
            self._file.flush()

        return self._filepath

    def stop_recording(self):
        """Flush, close and finalise the current recording file."""
        with self._lock:
            if not self._active:
                return
            self._write_event("Recording stopped")
            self._file.flush()
            self._file.close()
            self._file    = None
            self._writer  = None
            self._active  = False

    def record_flight(self, stage: int, d: dict):
        """Record one FlightPacket dict."""
        if not self._active:
            return
        row = self._base_row("FLIGHT", stage)
        row.update({
            "timestamp_ms":      d.get("timestamp_ms", ""),
            "altitude_ft":       f"{d.get('altitude_ft', 0):.2f}",
            "velocity_fps":      f"{d.get('velocity_fps', 0):.2f}",
            "accel_thrust_ms2":  f"{d.get('accel_thrust', 0):.4f}",
            "accel_lateral_ms2": f"{d.get('accel_lateral', 0):.4f}",
            "gyro_roll_rads":    f"{d.get('gyro_roll', 0):.5f}",
            "gyro_pitch_rads":   f"{d.get('gyro_pitch', 0):.5f}",
            "gyro_yaw_rads":     f"{d.get('gyro_yaw', 0):.5f}",
            "temperature_c":     f"{d.get('temperature_c', 0):.2f}",
            "rssi_dbm":          d.get("rssi_dbm", ""),
            "flight_state":      d.get("state", ""),
            "fault_flags_hex":   f"0x{d.get('fault_flags', 0):02X}",
        })
        self._write_row(row)

    def record_status(self, stage: int, d: dict):
        """Record one StatusPacket dict."""
        if not self._active:
            return
        row = self._base_row("STATUS", stage)
        row.update({
            "flight_state":    d.get("flight_state", ""),
            "fault_flags_hex": f"0x{d.get('fault_flags', 0):02X}",
            "lora_freq_mhz":   f"{d.get('lora_freq_mhz', 0):.3f}",
            "vtx_freq_mhz":    d.get("vtx_freq_mhz", ""),
            "vtx_power_index": d.get("vtx_power_index", ""),
            "telem_rate_hz":   d.get("telem_rate_hz", ""),
            "video_enabled":   d.get("video_enabled", ""),
            "test_mode":       d.get("test_mode", ""),
            "last_cmd_hex":    f"0x{d.get('last_cmd', 0):02X}",
            "last_cmd_result": d.get("last_cmd_result", ""),
        })
        self._write_row(row)

    def record_gps(self, name: str, lat: float, lon: float,
                   alt_ft: float, vert_vel: float, horiz_vel: float):
        """Record one Featherweight GPS position update."""
        if not self._active:
            return
        row = self._base_row("GPS", stage="")
        row.update({
            "tracker_name":    name,
            "gps_lat":         f"{lat:.7f}",
            "gps_lon":         f"{lon:.7f}",
            "gps_alt_ft":      f"{alt_ft:.1f}",
            "gps_vert_vel_fps":  f"{vert_vel:.2f}",
            "gps_horiz_vel_fps": f"{horiz_vel:.2f}",
        })
        self._write_row(row)

    def record_event(self, message: str):
        """Write a human-readable event marker row."""
        if not self._active:
            return
        row = self._base_row("EVENT", stage="")
        row["event_message"] = message
        self._write_row(row)

    # ------------------------------------------------------------------
    #  Private helpers
    # ------------------------------------------------------------------

    def _base_row(self, source: str, stage) -> dict:
        """Build the common prefix columns for any row type."""
        now     = datetime.now()
        elapsed = (now - self._start_ts).total_seconds() if self._start_ts else 0
        return {
            "wall_time_iso": now.isoformat(timespec="milliseconds"),
            "elapsed_s":     f"{elapsed:.3f}",
            "source":        source,
            "stage":         str(stage),
        }

    def _write_row(self, row: dict):
        """Thread-safe row write with periodic flush."""
        with self._lock:
            if not self._active or not self._writer:
                return
            self._writer.writerow(row)
            # Flush every 50 rows (~500 ms at 100 Hz) to balance
            # write performance against data loss on power cut
            if not hasattr(self, "_row_count"):
                self._row_count = 0
            self._row_count += 1
            if self._row_count >= 50:
                self._file.flush()
                self._row_count = 0

    def _write_event(self, message: str):
        """Write event row — caller must already hold _lock."""
        now     = datetime.now()
        elapsed = (now - self._start_ts).total_seconds() if self._start_ts else 0
        self._writer.writerow({
            "wall_time_iso": now.isoformat(timespec="milliseconds"),
            "elapsed_s":     f"{elapsed:.3f}",
            "source":        "EVENT",
            "stage":         "",
            "event_message": message,
        })


# =============================================================================
#  PACKET STATISTICS TRACKER
#  Used for full system test analysis and battery evaluation
# =============================================================================

class PacketStatsTracker:
    """
    Tracks packet reception statistics during a full system test.

    Usage:
        tracker = PacketStatsTracker(stage=1, expected_rate_hz=10.0)
        tracker.start()
        # For each received packet:
        tracker.record_packet(flight_data_dict)
        # When test completes:
        tracker.stop()
        summary = tracker.summary_dict()
        report_path = tracker.generate_report(output_dir="recordings", session_name="test1")
    """

    def __init__(self, stage: int, expected_rate_hz: float = 10.0):
        self.stage = stage
        self.expected_rate_hz = expected_rate_hz

        self._active = False
        self._start_time = None
        self._stop_time = None

        self._received_count = 0
        self._last_timestamp_ms = None
        self._gaps = 0  # Number of detected packet gaps

        self._rssi_values = []
        self._temp_values = []

    def start(self):
        """Begin tracking statistics."""
        self._active = True
        self._start_time = datetime.now()
        self._received_count = 0
        self._last_timestamp_ms = None
        self._gaps = 0
        self._rssi_values = []
        self._temp_values = []

    def stop(self):
        """Stop tracking statistics."""
        self._active = False
        self._stop_time = datetime.now()

    def record_packet(self, flight_data: dict):
        """
        Record statistics from one received flight packet.

        Args:
            flight_data: Dictionary containing parsed FlightPacket fields
        """
        if not self._active:
            return

        self._received_count += 1

        # Track RSSI
        rssi = flight_data.get("rssi_dbm")
        if rssi is not None:
            self._rssi_values.append(rssi)

        # Track temperature
        temp = flight_data.get("temperature_c")
        if temp is not None:
            self._temp_values.append(temp)

        # Detect gaps in packet reception
        # A gap is when we miss one or more expected packets based on timestamp
        timestamp_ms = flight_data.get("timestamp_ms")
        if timestamp_ms is not None and self._last_timestamp_ms is not None:
            expected_interval_ms = 1000.0 / self.expected_rate_hz
            actual_interval_ms = timestamp_ms - self._last_timestamp_ms

            # If actual interval is more than 1.5x expected, consider it a gap
            if actual_interval_ms > expected_interval_ms * 1.5:
                self._gaps += 1

        self._last_timestamp_ms = timestamp_ms

    def summary_dict(self) -> dict:
        """
        Return current statistics as a dictionary.

        Returns dict with keys:
            stage, received, expected, loss_pct, gaps,
            avg_rssi, min_rssi, temp_min_c, temp_max_c, elapsed_s
        """
        elapsed_s = self._elapsed_seconds()
        expected = int(elapsed_s * self.expected_rate_hz) if elapsed_s > 0 else 0

        loss_pct = 0.0
        if expected > 0:
            loss_pct = max(0.0, (expected - self._received_count) / expected * 100.0)

        avg_rssi = sum(self._rssi_values) / len(self._rssi_values) if self._rssi_values else 0
        min_rssi = min(self._rssi_values) if self._rssi_values else 0

        temp_min_c = min(self._temp_values) if self._temp_values else 0
        temp_max_c = max(self._temp_values) if self._temp_values else 0

        return {
            "stage": self.stage,
            "received": self._received_count,
            "expected": expected,
            "loss_pct": loss_pct,
            "gaps": self._gaps,
            "avg_rssi": avg_rssi,
            "min_rssi": min_rssi,
            "temp_min_c": temp_min_c,
            "temp_max_c": temp_max_c,
            "elapsed_s": elapsed_s,
        }

    def generate_report(self, output_dir: str = "recordings",
                       session_name: str = "") -> str:
        """
        Generate a detailed text report file.

        Returns the path to the created report file.
        """
        os.makedirs(output_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = f"_{session_name}" if session_name.strip() else ""
        filename = f"TRES_{ts}{tag}_S{self.stage}_test_report.txt"
        filepath = os.path.join(output_dir, filename)

        summary = self.summary_dict()

        with open(filepath, "w", encoding="utf-8") as f:
            f.write("=" * 70 + "\n")
            f.write(f"TRES Titan — Stage {self.stage} Full System Test Report\n")
            f.write("=" * 70 + "\n\n")

            f.write(f"Test Duration:     {summary['elapsed_s']:.1f} seconds\n")
            f.write(f"Expected Rate:     {self.expected_rate_hz} Hz\n")
            f.write(f"\n")

            f.write("PACKET STATISTICS:\n")
            f.write(f"  Received:        {summary['received']}\n")
            f.write(f"  Expected:        {summary['expected']}\n")
            f.write(f"  Loss:            {summary['loss_pct']:.2f}%\n")
            f.write(f"  Gaps Detected:   {summary['gaps']}\n")
            f.write(f"\n")

            f.write("SIGNAL QUALITY:\n")
            f.write(f"  Avg RSSI:        {summary['avg_rssi']:.1f} dBm\n")
            f.write(f"  Min RSSI:        {summary['min_rssi']} dBm\n")
            f.write(f"\n")

            f.write("TEMPERATURE RANGE:\n")
            f.write(f"  Min:             {summary['temp_min_c']:.1f} °C\n")
            f.write(f"  Max:             {summary['temp_max_c']:.1f} °C\n")
            f.write(f"\n")

            # Pass/Fail criteria
            f.write("EVALUATION:\n")
            passed = summary['loss_pct'] <= 5.0
            f.write(f"  Status:          {'PASS ✓' if passed else 'FAIL ✗'}\n")
            f.write(f"  Criteria:        <5% packet loss\n")
            f.write(f"\n")

            f.write("=" * 70 + "\n")
            f.write(f"Report generated: {datetime.now().isoformat()}\n")

        return filepath

    def _elapsed_seconds(self) -> float:
        """Return elapsed time in seconds since start."""
        if not self._start_time:
            return 0.0
        end_time = self._stop_time if self._stop_time else datetime.now()
        return (end_time - self._start_time).total_seconds()
