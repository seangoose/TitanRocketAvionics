# =============================================================================
#  TRES Titan Ground Station — radio_worker.py
#  QThread that reads from one Adafruit M0 LoRa radio USB serial port,
#  parses TRES-framed downlink packets, and emits typed Qt signals.
#  One instance is created per stage.
# =============================================================================

import struct
import serial
from PyQt5.QtCore import QThread, pyqtSignal, QMutex, QMutexLocker
from protocol import (
    MAGIC_0, MAGIC_1, PKT_DATA, PKT_ACK, PKT_STATUS,
    FULL_DATA_FRAME_SIZE, FULL_STATUS_FRAME_SIZE, ACK_FRAME_SIZE,
    parse_flight_packet, parse_status_packet, parse_ack_frame,
    crc8, build_m0_retune
)


class RadioWorker(QThread):
    """
    Reads from one M0 serial port in a background thread.

    Signals:
      flight_data(stage, dict)         — parsed FlightPacket
      status_data(stage, dict)         — parsed StatusPacket
      ack_received(stage, dict)        — {cmd, result} from ACK frame
      connection_changed(stage, bool)  — True=connected, False=lost
      raw_log(stage, str)              — raw hex / debug for console

    Public thread-safe methods:
      send_frame(bytes)   — write uplink command frame to radio
      retune(freq_mhz)    — write M0 retune escape to serial port
      set_intended_freq(f)— store intended frequency for reconnect restore
    """

    flight_data        = pyqtSignal(int, dict)
    status_data        = pyqtSignal(int, dict)
    ack_received       = pyqtSignal(int, dict)
    connection_changed = pyqtSignal(int, bool)
    raw_log            = pyqtSignal(int, str)

    def __init__(self, stage: int, port: str, baud: int, parent=None):
        super().__init__(parent)
        self.stage    = stage
        self.port     = port
        self.baud     = baud
        self._running = False
        self._serial  = None
        self._mutex   = QMutex()

        # Frequency tracking for auto-retune
        # _current_lora_freq: last frequency confirmed by STATUS packet
        # _intended_freq: frequency to restore on reconnect
        self._current_lora_freq  = None    # Unknown until first STATUS
        self._intended_freq      = None    # Set by set_intended_freq()

    # ------------------------------------------------------------------
    #  Public API — called from main thread
    # ------------------------------------------------------------------

    def send_frame(self, frame: bytes):
        """Thread-safe: write a TRES uplink command frame to the M0."""
        with QMutexLocker(self._mutex):
            if self._serial and self._serial.is_open:
                try:
                    self._serial.write(frame)
                    self._serial.flush()
                except serial.SerialException as e:
                    self.raw_log.emit(self.stage, f"[TX ERROR S{self.stage}] {e}")

    def retune(self, freq_mhz: float):
        """
        Write the M0 retune escape sequence directly to the serial port.
        The M0 firmware intercepts [0xFF][0xFE][float_LE] and calls
        rf95.setFrequency() on itself — it does NOT forward to LoRa.
        Call this AFTER the rocket has confirmed its frequency change
        via a STATUS packet received on the OLD frequency.
        """
        escape = build_m0_retune(freq_mhz)
        with QMutexLocker(self._mutex):
            if self._serial and self._serial.is_open:
                try:
                    self._serial.write(escape)
                    self._serial.flush()
                    self.raw_log.emit(self.stage,
                        f"[M0 RETUNE S{self.stage}] Escape sent → {freq_mhz:.3f} MHz")
                except serial.SerialException as e:
                    self.raw_log.emit(self.stage, f"[RETUNE ERROR S{self.stage}] {e}")

    def set_intended_freq(self, freq_mhz: float):
        """
        Store the intended LoRa frequency.  On reconnect after a USB
        reset (which resets the M0 to default 915 MHz), the worker
        automatically re-sends the retune escape to restore the correct
        frequency.
        """
        self._intended_freq = freq_mhz

    def stop(self):
        self._running = False

    # ------------------------------------------------------------------
    #  Thread main loop
    # ------------------------------------------------------------------

    def run(self):
        self._running = True
        while self._running:
            try:
                self._serial = serial.Serial(self.port, self.baud, timeout=1.0)
                self.connection_changed.emit(self.stage, True)
                self.raw_log.emit(self.stage,
                    f"[RADIO S{self.stage}] Connected on {self.port} @ {self.baud}")

                # Restore intended frequency after reconnect.
                # USB reconnect resets the M0 to its default frequency (915 MHz).
                # If we previously changed to a different frequency, re-apply it.
                if (self._intended_freq is not None and
                        abs(self._intended_freq - 915.0) > 0.01):
                    import time; time.sleep(1.5)   # Allow M0 boot + USB enum
                    self.retune(self._intended_freq)
                    self.raw_log.emit(self.stage,
                        f"[RADIO S{self.stage}] Reconnect retune → {self._intended_freq:.3f} MHz")

                self._read_loop()

            except serial.SerialException as e:
                self.connection_changed.emit(self.stage, False)
                self.raw_log.emit(self.stage,
                    f"[RADIO S{self.stage}] Port error: {e}. Retry in 3s…")
                self.msleep(3000)
            finally:
                if self._serial:
                    try: self._serial.close()
                    except Exception: pass
                    self._serial = None

    # ------------------------------------------------------------------
    #  Streaming byte parser
    # ------------------------------------------------------------------

    def _read_loop(self):
        """
        Scan the byte stream from the M0 for TRES frames.
        Robust to partial reads, noise, and M0 retune confirmations.
        """
        buf = bytearray()

        while self._running and self._serial and self._serial.is_open:
            try:
                chunk = self._serial.read(128)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= 4:
                idx = self._find_header(buf)
                if idx < 0:
                    buf = buf[-2:]
                    break
                if idx > 0:
                    buf = buf[idx:]
                if len(buf) < 4:
                    break

                pkt_type = buf[3]
                expected = self._expected_size(pkt_type)
                if expected == 0:
                    buf = buf[1:]
                    continue
                if len(buf) < expected:
                    break

                frame = bytes(buf[:expected])
                buf   = buf[expected:]
                self._dispatch(pkt_type, frame)

    def _find_header(self, buf: bytearray) -> int:
        for i in range(len(buf) - 2):
            if (buf[i]   == MAGIC_0 and
                buf[i+1] == MAGIC_1 and
                buf[i+2] == self.stage):
                return i
        return -1

    def _expected_size(self, pkt_type: int) -> int:
        return {
            PKT_DATA:   FULL_DATA_FRAME_SIZE,
            PKT_STATUS: FULL_STATUS_FRAME_SIZE,
            PKT_ACK:    ACK_FRAME_SIZE,
        }.get(pkt_type, 0)

    def _dispatch(self, pkt_type: int, frame: bytes):
        self.raw_log.emit(self.stage,
            f"[RX S{self.stage} 0x{pkt_type:02X}] {frame.hex()}")

        if pkt_type == PKT_DATA:
            data = parse_flight_packet(frame[4:])
            if data:
                self.flight_data.emit(self.stage, data)
            else:
                self.raw_log.emit(self.stage,
                    f"[S{self.stage}] DATA CRC fail / wrong length ({len(frame[4:])} vs {FULL_DATA_FRAME_SIZE-4})")

        elif pkt_type == PKT_STATUS:
            data = parse_status_packet(frame[4:])
            if data:
                # ── AUTO-RETUNE ─────────────────────────────────────────
                # If the STATUS packet contains a LoRa frequency that
                # differs from what we last tracked, the rocket has changed
                # its frequency (via CMD_SET_LORA_FREQ).  Automatically
                # retune this M0 to follow it.
                #
                # The rocket sends ACK and STATUS on the OLD frequency
                # before switching (with an 80ms delay).  By the time we
                # process this STATUS packet, both we and the rocket are
                # ready to switch — the rocket is about to retune, and
                # we retune now so we're listening on the new freq when
                # the next rocket packet arrives.
                new_freq = data["lora_freq_mhz"]
                if (self._current_lora_freq is not None and
                        abs(new_freq - self._current_lora_freq) > 0.01):
                    self.raw_log.emit(self.stage,
                        f"[AUTO-RETUNE S{self.stage}] "
                        f"{self._current_lora_freq:.3f} → {new_freq:.3f} MHz")
                    self._intended_freq = new_freq   # Persist for reconnect
                    self.retune(new_freq)

                self._current_lora_freq = new_freq
                self.status_data.emit(self.stage, data)
            else:
                self.raw_log.emit(self.stage,
                    f"[S{self.stage}] STATUS CRC fail / wrong length ({len(frame[4:])} vs {FULL_STATUS_FRAME_SIZE-4})")

        elif pkt_type == PKT_ACK:
            data = parse_ack_frame(frame)
            if data:
                self.ack_received.emit(self.stage, data)

