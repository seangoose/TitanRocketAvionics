# =============================================================================
#  TRES Titan Ground Station — main_window.py
#  Main application window: four-quadrant layout with controls bar.
# =============================================================================

import os
import subprocess
from datetime import datetime

from PyQt5.QtCore    import Qt, QTimer, pyqtSlot, pyqtSignal
from PyQt5.QtGui     import QPixmap
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QGridLayout,
    QLabel, QPushButton, QLineEdit, QGroupBox, QTextEdit,
    QSplitter, QFrame, QTabWidget, QSizePolicy, QFileDialog,
    QMessageBox, QProgressBar, QComboBox, QScrollArea
)

import config
import protocol as P

# VTX band/channel lookup table — matches Teensy VTX_CHAN[5][8] exactly
# Format: (display_label, freq_mhz)
VTX_CHANNELS = []
_VTX_BAND_NAMES = ["A", "B", "E", "F", "R"]
_VTX_FREQS = [
    [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725],  # Band A
    [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866],  # Band B
    [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945],  # Band E
    [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880],  # Band F
    [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917],  # Raceband
]
for _b, _band in enumerate(_VTX_BAND_NAMES):
    for _ch in range(8):
        _freq = _VTX_FREQS[_b][_ch]
        VTX_CHANNELS.append((f"{_band}{_ch+1} — {_freq} MHz", _freq))
from data_recorder import DataRecorder, PacketStatsTracker
from map_widget    import MapWidget
from radio_worker  import RadioWorker
from gps_worker    import FWGPSWorker
from video_worker      import VideoWorker
from video_file_worker import VideoFileWorker


# =============================================================================
#  STYLESHEET
# =============================================================================
DARK_STYLE = """
QMainWindow, QWidget {
    background-color: #0d1117; color: #c9d1d9;
}
QGroupBox {
    border: 1px solid #30363d; border-radius: 6px;
    margin-top: 8px; padding: 8px;
    font-weight: bold; color: #58a6ff;
}
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QPushButton {
    background-color: #21262d; border: 1px solid #30363d;
    border-radius: 4px; color: #c9d1d9;
    padding: 6px 18px; font-size: 11px;
    min-width: 70px;
}
QPushButton:hover   { background-color: #30363d; }
QPushButton:pressed { background-color: #388bfd; color: #fff; }
QPushButton:disabled { color: #484f58; border-color: #21262d; }
QPushButton#danger  { background-color:#3d0f0f; border-color:#da3633; color:#ff7b72; }
QPushButton#danger:hover { background-color:#6f1313; }
QPushButton#rec_active { background-color:#3d0f0f; border-color:#da3633;
                          color:#ff7b72; font-weight:bold; }
QPushButton#confirm { background-color:#0d1f0d; border-color:#238636; color:#3fb950; }
QPushButton#confirm:hover { background-color:#1a3a1a; }
QPushButton#test_active { background-color:#1b2a1b; border-color:#3fb950;
                           color:#3fb950; font-weight:bold; }
QPushButton#sys_test {
    background-color: #1a1a2e;
    border: 1px solid #f0a500;
    color: #f0a500;
    font-weight: bold;
}
QPushButton#sys_test:hover { background-color: #2a2a3e; }
QLineEdit {
    background-color:#161b22; border:1px solid #30363d;
    border-radius:4px; color:#c9d1d9; padding:4px; font-family:monospace;
}
QTextEdit {
    background-color:#0d1117; border:1px solid #21262d;
    color:#8b949e; font-family:'Courier New',monospace; font-size:10px;
}
QLabel#telem_value { font-size:17px; font-weight:bold; color:#fff; font-family:monospace; }
QLabel#telem_label { font-size:10px; color:#8b949e; }
QLabel#state_label { font-size:13px; font-weight:bold; color:#58a6ff; font-family:monospace; }
QLabel#fault_ok    { font-size:10px; color:#3fb950; }
QLabel#fault_err   { font-size:10px; color:#ff7b72; }
QLabel#status_ok   { color:#3fb950; font-weight:bold; }
QLabel#status_err  { color:#f85149; font-weight:bold; }
QLabel#rec_active  { color:#ff7b72; font-weight:bold; }
QLabel#rec_idle    { color:#484f58; }
QLabel#video_placeholder {
    background-color:#161b22; color:#484f58;
    font-size:12px; border:1px solid #21262d; border-radius:4px;
}
QTabWidget::pane   { border:1px solid #30363d; }
QTabBar::tab       { background:#161b22; color:#8b949e; padding:6px 14px; border:1px solid #21262d; }
QTabBar::tab:selected { background:#21262d; color:#c9d1d9; }
QGroupBox#controls_group {
    border: 1px solid #21262d;
    border-radius: 4px;
    margin-top: 5px;
    padding-top: 4px;
    font-size: 9px;
    font-weight: bold;
    color: #58a6ff;
}
QGroupBox#controls_group::title {
    subcontrol-origin: margin;
    left: 6px;
    padding: 0 3px;
}
QLabel#section_header {
    font-size: 12px;
    font-weight: bold;
    color: #58a6ff;
    padding: 3px 0px;
    border-bottom: 1px solid #21262d;
    margin-bottom: 3px;
}
QSplitter::handle {
    background-color: #21262d;
}
QSplitter::handle:horizontal {
    width: 3px;
}
QSplitter::handle:vertical {
    height: 3px;
}
QSplitter::handle:hover {
    background-color: #388bfd;
}
"""


# =============================================================================
#  TELEMETRY PANEL  (one per stage)
# =============================================================================

class TelemetryPanel(QGroupBox):
    def __init__(self, stage: int, parent=None):
        super().__init__(
            f"Stage {stage} — {'Booster' if stage == 1 else 'Sustainer'}", parent)
        self.stage = stage

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 18, 4, 4)
        outer.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.NoFrame)
        outer.addWidget(scroll)

        inner = QWidget()
        inner.setMinimumWidth(340)
        scroll.setWidget(inner)
        self._setup(inner)

    def _setup(self, parent):
        g = QGridLayout(parent)
        g.setHorizontalSpacing(10)
        g.setVerticalSpacing(5)
        g.setContentsMargins(6, 4, 6, 4)
        # col 0 = label A, col 1 = value A, col 2 = label B, col 3 = value B
        g.setColumnMinimumWidth(0, 70)
        g.setColumnMinimumWidth(2, 80)
        g.setColumnStretch(1, 2)
        g.setColumnStretch(3, 2)

        def lbl(text):
            w = QLabel(text); w.setObjectName("telem_label"); return w

        def val(attr, name="telem_label"):
            w = QLabel("—"); w.setObjectName(name)
            setattr(self, attr, w); return w

        def sep(row):
            f = QFrame(); f.setFrameShape(QFrame.HLine)
            f.setStyleSheet("color:#21262d;"); g.addWidget(f, row, 0, 1, 4)

        # ── Primary flight data ───────────────────────────────────────
        g.addWidget(lbl("ALT"),   0, 0); g.addWidget(val("_alt",   "telem_value"), 0, 1, 1, 3)
        g.addWidget(lbl("VEL"),   1, 0); g.addWidget(val("_vel",   "telem_value"), 1, 1, 1, 3)
        g.addWidget(lbl("ACCEL"), 2, 0); g.addWidget(val("_accel", "telem_value"), 2, 1, 1, 3)
        g.addWidget(lbl("TEMP"),  3, 0); g.addWidget(val("_temp"), 3, 1)
        g.addWidget(lbl("RSSI"),  3, 2); g.addWidget(val("_rssi"), 3, 3)

        sep(4)

        # ── State / faults / mode ─────────────────────────────────────
        g.addWidget(lbl("STATE"), 5, 0)
        self._state = QLabel("—"); self._state.setObjectName("state_label")
        g.addWidget(self._state, 5, 1, 1, 3)

        self._fault = QLabel("Faults: None"); self._fault.setObjectName("fault_ok")
        g.addWidget(self._fault, 6, 0, 1, 4)

        g.addWidget(lbl("MODE"), 7, 0)
        self._gmode = QLabel("—"); self._gmode.setObjectName("state_label")
        g.addWidget(self._gmode, 7, 1, 1, 3)

        sep(8)

        # ── Radio / config ────────────────────────────────────────────
        g.addWidget(lbl("LoRa"),    9, 0); g.addWidget(val("_lora"),   9, 1)
        g.addWidget(lbl("VTX"),     9, 2); g.addWidget(val("_vtx"),    9, 3)

        g.addWidget(lbl("Telem"),  10, 0); g.addWidget(val("_trate"), 10, 1)
        g.addWidget(lbl("VTX pwr"),10, 2); g.addWidget(val("_vpwr"),  10, 3)

        g.addWidget(lbl("Flt pwr"),11, 0); g.addWidget(val("_vfpwr"), 11, 1)
        g.addWidget(lbl("Video"),  11, 2); g.addWidget(val("_video"),  11, 3)

        g.addWidget(lbl("Cam"),    12, 0); g.addWidget(val("_camrec"), 12, 1)
        g.addWidget(lbl("Test"),   12, 2); g.addWidget(val("_testm"),  12, 3)

        sep(13)

        self._conn = QLabel("⚫ Disconnected")
        self._conn.setObjectName("status_err")
        g.addWidget(self._conn, 14, 0, 1, 4)

        self._ts = QLabel("Last: —"); self._ts.setObjectName("telem_label")
        g.addWidget(self._ts, 15, 0, 1, 4)

    def update_flight(self, d: dict):
        stage = d.get("stage", self.stage)
        self._alt.setText(f"{d['altitude_ft']:7.0f} ft")
        self._vel.setText(f"{d['velocity_fps']:6.1f} fps")
        self._accel.setText(f"{d['accel_thrust']/9.81:.2f} G")
        self._temp.setText(f"{d['temperature_c']:.1f} °C")
        self._rssi.setText(f"{d['rssi_dbm']} dBm")
        self._state.setText(P.state_name(stage, d["state"]))
        fstr = P.fault_string(d["fault_flags"])
        self._fault.setText(f"Faults: {fstr}")
        self._fault.setObjectName("fault_err" if fstr != "None" else "fault_ok")
        self._fault.style().unpolish(self._fault)
        self._fault.style().polish(self._fault)
        self._ts.setText(f"T+{d['timestamp_ms']/1000:.1f}s")

    def update_status(self, d: dict):
        gm   = d.get("ground_mode", 0)
        gname = P.ground_mode_name(gm)
        self._gmode.setText(f"MODE: {gname}")
        gm_colors = {0: "#f0a500", 1: "#58a6ff", 2: "#3fb950"}
        self._gmode.setStyleSheet(f"color:{gm_colors.get(gm, '#c9d1d9')};font-weight:bold;")

        self._lora.setText(f"LoRa: {d['lora_freq_mhz']:.3f} MHz")
        self._vtx.setText(f"VTX: {d['vtx_freq_mhz']} MHz")
        self._trate.setText(f"Telem: {d['telem_rate_hz']} Hz")
        self._vpwr.setText(f"VTX cur: {P.VTX_POWER_LABELS.get(d['vtx_power_index'],'?')}")
        self._vfpwr.setText(f"VTX flt: {P.VTX_POWER_LABELS.get(d.get('vtx_flight_power',3),'?')}")
        self._video.setText("Video: ON" if d["video_enabled"] else "Video: OFF")

        cam = d.get("cam_recording", 0)
        self._camrec.setText("Cam: ● REC" if cam else "Cam: idle")
        self._camrec.setStyleSheet("color:#ff7b72;font-weight:bold;" if cam else "color:#8b949e;")

        self._testm.setText("Test: ON ⚠" if d["test_mode"] else "Test: off")
        self._testm.setStyleSheet("color:#f0a500;" if d["test_mode"] else "color:#8b949e;")

    def set_connected(self, connected: bool):
        if connected:
            self._conn.setText(f"🟢 S{self.stage} Connected")
            self._conn.setObjectName("status_ok")
        else:
            self._conn.setText(f"🔴 S{self.stage} Disconnected")
            self._conn.setObjectName("status_err")
        self._conn.style().unpolish(self._conn)
        self._conn.style().polish(self._conn)


# =============================================================================
#  VIDEO PANEL  (one per stage)
# =============================================================================

class VideoPanel(QGroupBox):
    def __init__(self, stage: int, parent=None):
        super().__init__(f"Stage {stage} Video", parent)
        self.stage = stage
        layout = QVBoxLayout(self)
        self._label = QLabel(f"Stage {stage} — No Signal")
        self._label.setObjectName("video_placeholder")
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setMinimumSize(300, 225)
        self._label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self._label)
        self._status = QLabel("Connecting…"); self._status.setObjectName("telem_label")
        layout.addWidget(self._status)
        self._rec_ind = QLabel("● Not recording"); self._rec_ind.setObjectName("rec_idle")
        layout.addWidget(self._rec_ind)

    def update_frame(self, stage: int, img):
        if stage != self.stage: return
        pix = QPixmap.fromImage(img).scaled(
            self._label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self._label.setPixmap(pix)

    def set_status(self, msg: str):
        self._status.setText(msg)

    def set_recording(self, active: bool, path: str = ""):
        if active:
            self._rec_ind.setText(f"● REC  {os.path.basename(path)}")
            self._rec_ind.setObjectName("rec_active")
        else:
            self._rec_ind.setText("● Not recording")
            self._rec_ind.setObjectName("rec_idle")
        self._rec_ind.style().unpolish(self._rec_ind)
        self._rec_ind.style().polish(self._rec_ind)


# =============================================================================
#  RECORDING PANEL
# =============================================================================

class RecordingPanel(QGroupBox):
    def __init__(self, vid_s1: VideoWorker, vid_s2: VideoWorker,
                 data_recorder: DataRecorder, parent=None):
        super().__init__("Recording", parent)
        self._vid_s1 = vid_s1
        self._vid_s2 = vid_s2
        self._data   = data_recorder
        self._setup()

    def _setup(self):
        g = QGridLayout(self)
        g.setSpacing(6)

        g.addWidget(QLabel("Session name:"), 0, 0)
        self._name_edit = QLineEdit("")
        self._name_edit.setPlaceholderText("e.g. IREC_2026_flight1")
        g.addWidget(self._name_edit, 0, 1, 1, 2)

        g.addWidget(QLabel("Output dir:"), 1, 0)
        rec_dir = os.path.abspath("recordings")
        self._dir_lbl = QLabel(rec_dir)
        self._dir_lbl.setObjectName("telem_label")
        self._dir_lbl.setWordWrap(True)
        g.addWidget(self._dir_lbl, 1, 1)
        btn_open = QPushButton("Open")
        btn_open.setToolTip("Open recordings folder in file manager")
        btn_open.clicked.connect(self._open_folder)
        g.addWidget(btn_open, 1, 2)

        self._rec_btn = QPushButton("⏺  Record All")
        self._rec_btn.setObjectName("confirm")
        self._rec_btn.setToolTip(
            "Start recording both video feeds and flight data CSV")
        self._rec_btn.clicked.connect(self._start_all)
        g.addWidget(self._rec_btn, 2, 0, 1, 2)

        self._stop_btn = QPushButton("⏹  Stop All")
        self._stop_btn.setObjectName("danger")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._stop_all)
        g.addWidget(self._stop_btn, 2, 2)

        self._s1v_lbl  = QLabel("S1 Video: idle"); self._s1v_lbl.setObjectName("rec_idle")
        self._s2v_lbl  = QLabel("S2 Video: idle"); self._s2v_lbl.setObjectName("rec_idle")
        self._data_lbl = QLabel("Data:     idle"); self._data_lbl.setObjectName("rec_idle")
        g.addWidget(self._s1v_lbl,  3, 0, 1, 3)
        g.addWidget(self._s2v_lbl,  4, 0, 1, 3)
        g.addWidget(self._data_lbl, 5, 0, 1, 3)

        sep = QFrame(); sep.setFrameShape(QFrame.HLine); sep.setStyleSheet("color:#30363d;")
        g.addWidget(sep, 6, 0, 1, 3)

        self._showcase_btn = QPushButton("🎬  Showcase Mode: OFF")
        self._showcase_btn.setObjectName("confirm")
        self._showcase_btn.setToolTip(
            "Showcase mode: replaces one video panel with a looping highlight reel.\n"
            "Configure SHOWCASE_VIDEO_PATH and SHOWCASE_STAGE in config.py.\n"
            "Live capture continues on the other panel.")
        g.addWidget(self._showcase_btn, 7, 0, 1, 3)

        self._showcase_status = QLabel("Showcase: OFF")
        self._showcase_status.setObjectName("telem_label")
        g.addWidget(self._showcase_status, 8, 0, 1, 3)

    def _start_all(self):
        name = self._name_edit.text().strip()

        p1 = self._vid_s1.start_recording(name)
        p2 = self._vid_s2.start_recording(name)

        try:
            dp = self._data.start_recording(name)
            self._data_lbl.setText(f"Data:  ● {os.path.basename(dp)}")
            self._data_lbl.setObjectName("rec_active")
        except RuntimeError:
            pass

        self._rec_btn.setEnabled(False)
        self._rec_btn.setObjectName("")
        self._stop_btn.setEnabled(True)

        if p1:
            self._s1v_lbl.setText(f"S1 Video: ● {os.path.basename(p1)}")
            self._s1v_lbl.setObjectName("rec_active")
        if p2:
            self._s2v_lbl.setText(f"S2 Video: ● {os.path.basename(p2)}")
            self._s2v_lbl.setObjectName("rec_active")

        self._refresh_styles()

    def _stop_all(self):
        self._vid_s1.stop_recording()
        self._vid_s2.stop_recording()
        self._data.stop_recording()

        self._s1v_lbl.setText("S1 Video: stopped"); self._s1v_lbl.setObjectName("rec_idle")
        self._s2v_lbl.setText("S2 Video: stopped"); self._s2v_lbl.setObjectName("rec_idle")
        self._data_lbl.setText("Data:     stopped"); self._data_lbl.setObjectName("rec_idle")

        self._rec_btn.setEnabled(True)
        self._rec_btn.setObjectName("confirm")
        self._stop_btn.setEnabled(False)
        self._refresh_styles()

    def _open_folder(self):
        rec_dir = os.path.abspath("recordings")
        os.makedirs(rec_dir, exist_ok=True)
        try:
            subprocess.Popen(["xdg-open", rec_dir])
        except FileNotFoundError:
            QMessageBox.information(self, "Recordings",
                f"Recordings are saved to:\n{rec_dir}")

    def _refresh_styles(self):
        for w in [self._s1v_lbl, self._s2v_lbl, self._data_lbl,
                  self._rec_btn, self._stop_btn]:
            w.style().unpolish(w); w.style().polish(w)


# =============================================================================
#  COMMAND PANEL
# =============================================================================

class CommandPanel(QWidget):
    """
    Ground station command panel.

    Ground Mode logic:
    ─────────────────────────────────────────────────────────────
    TEST_IDLE     Boot/bench state. RX only. Heartbeat 60s.
                  Camera recording and video TX commands active.
    PAD_IDLE      On rail. 1Hz telem + heartbeat. VTX at 25mW.
                  Camera not recording.
    LAUNCH_READY  Full power. Configured telem rate. VTX at flight power.
                  GPS panel active. FSM armed.

    Stage-dependency:
    ─────────────────────────────────────────────────────────────
    Ground mode buttons → per-row targeting (S1 / S2 / Both)
    CMD_SET_LORA_FREQ   → per-stage independent inputs
    CMD_SET_VTX_FREQ    → per-stage independent inputs
    CMD_SET_VTX_POWER   → per-stage independent inputs
    CMD_CAM_RECORD_*    → respects stage selector
    CMD_FIRE_SOLENOID   → Stage 2 ONLY, gated by TEST_IDLE mode
    """

    full_sys_test_requested = pyqtSignal(list)

    def __init__(self, radio_s1: RadioWorker, radio_s2: RadioWorker,
                 data_recorder: DataRecorder, parent=None):
        super().__init__(parent)
        self._r1   = radio_s1
        self._r2   = radio_s2
        self._data = data_recorder
        self._s1_ground_mode = P.GM_TEST_IDLE
        self._s2_ground_mode = P.GM_TEST_IDLE
        self._setup()

    def _setup(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(5)

        # ── 1. GROUND MODE ──────────────────────────────────────
        gm = QGroupBox("Ground Mode"); gml = QGridLayout(gm); gml.setSpacing(4)
        gml.addWidget(QLabel("S1:"), 0, 0)
        self._gm_s1_test   = self._mkbtn("Test",      lambda: self._send_mode(P.CMD_MODE_TEST,   [1]))
        self._gm_s1_pad    = self._mkbtn("Pad Idle",  lambda: self._send_mode(P.CMD_MODE_PAD,    [1]))
        self._gm_s1_launch = self._mkbtn("🚀 Launch", lambda: self._send_mode(P.CMD_MODE_LAUNCH, [1]), "confirm")
        gml.addWidget(self._gm_s1_test,   0, 1); gml.addWidget(self._gm_s1_pad, 0, 2); gml.addWidget(self._gm_s1_launch, 0, 3)

        gml.addWidget(QLabel("S2:"), 1, 0)
        self._gm_s2_test   = self._mkbtn("Test",      lambda: self._send_mode(P.CMD_MODE_TEST,   [2]))
        self._gm_s2_pad    = self._mkbtn("Pad Idle",  lambda: self._send_mode(P.CMD_MODE_PAD,    [2]))
        self._gm_s2_launch = self._mkbtn("🚀 Launch", lambda: self._send_mode(P.CMD_MODE_LAUNCH, [2]), "confirm")
        gml.addWidget(self._gm_s2_test,   1, 1); gml.addWidget(self._gm_s2_pad, 1, 2); gml.addWidget(self._gm_s2_launch, 1, 3)

        gml.addWidget(QLabel("Both:"), 2, 0)
        self._gm_both_test   = self._mkbtn("Test",      lambda: self._send_mode(P.CMD_MODE_TEST,   [1, 2]))
        self._gm_both_pad    = self._mkbtn("Pad Idle",  lambda: self._send_mode(P.CMD_MODE_PAD,    [1, 2]))
        self._gm_both_launch = self._mkbtn("🚀 Launch", lambda: self._send_mode(P.CMD_MODE_LAUNCH, [1, 2]), "confirm")
        gml.addWidget(self._gm_both_test, 2, 1); gml.addWidget(self._gm_both_pad, 2, 2); gml.addWidget(self._gm_both_launch, 2, 3)

        self._gm_s1_ind = QLabel("S1 Mode: —"); self._gm_s1_ind.setObjectName("telem_label")
        self._gm_s2_ind = QLabel("S2 Mode: —"); self._gm_s2_ind.setObjectName("telem_label")
        gml.addWidget(self._gm_s1_ind, 3, 0, 1, 2); gml.addWidget(self._gm_s2_ind, 3, 2, 1, 2)
        layout.addWidget(gm)

        # ── 2. STAGE SELECTOR ───────────────────────────────────
        sel = QGroupBox("Target Stage (commands below)"); sl = QHBoxLayout(sel)
        self._btn_s1   = QPushButton("Stage 1"); self._btn_s1.setCheckable(True); self._btn_s1.setChecked(True)
        self._btn_s2   = QPushButton("Stage 2"); self._btn_s2.setCheckable(True)
        self._btn_both = QPushButton("Both");    self._btn_both.setCheckable(True)
        for b in [self._btn_s1, self._btn_s2, self._btn_both]:
            sl.addWidget(b); b.clicked.connect(self._stage_sel)
        layout.addWidget(sel)

        # ── 3. CAMERA CONTROLS ──────────────────────────────────
        cg = QGroupBox("RunCam Recording  [stage selector]"); cgl = QGridLayout(cg)
        self._cam_rec_on_btn  = self._mkbtn("⏺  Record ON",  lambda: self._send(P.CMD_CAM_RECORD_ON),  "confirm")
        self._cam_rec_off_btn = self._mkbtn("⏹  Record OFF", lambda: self._send(P.CMD_CAM_RECORD_OFF), "danger")
        cgl.addWidget(self._cam_rec_on_btn,  0, 0)
        cgl.addWidget(self._cam_rec_off_btn, 0, 1)
        note = QLabel("Writes to RunCam SD. Independent of VTX RF.")
        note.setObjectName("telem_label"); note.setWordWrap(True)
        cgl.addWidget(note, 1, 0, 1, 2)
        layout.addWidget(cg)

        # ── 4. VIDEO / VTX ──────────────────────────────────────
        vg = QGroupBox("Video / VTX"); vgl = QGridLayout(vg); vgl.setSpacing(4)
        vgl.addWidget(QLabel("RF:"), 0, 0)
        self._video_on_btn  = self._mkbtn("Video ON",  lambda: self._send(P.CMD_VIDEO_ON))
        self._video_off_btn = self._mkbtn("Video OFF", lambda: self._send(P.CMD_VIDEO_OFF))
        vgl.addWidget(self._video_on_btn,  0, 1)
        vgl.addWidget(self._video_off_btn, 0, 2)

        vgl.addWidget(QLabel("S1 VTX:"), 1, 0)
        self._vtx_s1_edit = QComboBox()
        for label, freq in VTX_CHANNELS:
            self._vtx_s1_edit.addItem(label, freq)
        _default_s1 = min(range(len(VTX_CHANNELS)),
                          key=lambda i: abs(VTX_CHANNELS[i][1] - config.VTX_S1_FREQ_MHZ))
        self._vtx_s1_edit.setCurrentIndex(_default_s1)
        self._vtx_s1_edit.setMinimumWidth(130)
        vgl.addWidget(self._vtx_s1_edit, 1, 1)
        self._vtx_set_s1_btn = self._mkbtn("Set S1", lambda: self._send_vtx(1), "confirm")
        vgl.addWidget(self._vtx_set_s1_btn, 1, 2)

        vgl.addWidget(QLabel("S2 VTX:"), 2, 0)
        self._vtx_s2_edit = QComboBox()
        for label, freq in VTX_CHANNELS:
            self._vtx_s2_edit.addItem(label, freq)
        _default_s2 = min(range(len(VTX_CHANNELS)),
                          key=lambda i: abs(VTX_CHANNELS[i][1] - config.VTX_S2_FREQ_MHZ))
        self._vtx_s2_edit.setCurrentIndex(_default_s2)
        self._vtx_s2_edit.setMinimumWidth(130)
        vgl.addWidget(self._vtx_s2_edit, 2, 1)
        self._vtx_set_s2_btn = self._mkbtn("Set S2", lambda: self._send_vtx(2), "confirm")
        vgl.addWidget(self._vtx_set_s2_btn, 2, 2)

        pwr_labels = ["25mW", "200mW", "500mW", "1W"]
        vgl.addWidget(QLabel("S1 Flt Pwr:"), 3, 0)
        self._vp_s1_btns = []
        vpg1 = QHBoxLayout()
        for idx in range(4):
            b = self._mkbtn(pwr_labels[idx], lambda i=idx: self._send_vtx_power(1, i))
            b.setToolTip(f"S1 VTX flight power SA index {idx}")
            self._vp_s1_btns.append(b); vpg1.addWidget(b)
        vgl.addLayout(vpg1, 3, 1, 1, 3)

        vgl.addWidget(QLabel("S2 Flt Pwr:"), 4, 0)
        self._vp_s2_btns = []
        vpg2 = QHBoxLayout()
        for idx in range(4):
            b = self._mkbtn(pwr_labels[idx], lambda i=idx: self._send_vtx_power(2, i))
            b.setToolTip(f"S2 VTX flight power SA index {idx}")
            self._vp_s2_btns.append(b); vpg2.addWidget(b)
        vgl.addLayout(vpg2, 4, 1, 1, 3)
        layout.addWidget(vg)

        # ── 5. TELEMETRY ────────────────────────────────────────
        tg = QGroupBox("Telemetry  [stage selector]"); tgl = QGridLayout(tg)
        self._telem_10hz_btn = self._mkbtn("10 Hz",     lambda: self._send(P.CMD_TELEM_HIGH))
        self._telem_1hz_btn  = self._mkbtn(" 1 Hz",     lambda: self._send(P.CMD_TELEM_LOW))
        self._force_pkt_btn  = self._mkbtn("Force Pkt", lambda: self._send(P.CMD_FORCE_PACKET))
        self._get_status_btn = self._mkbtn("Get Status",lambda: self._send(P.CMD_GET_STATUS))
        tgl.addWidget(self._telem_10hz_btn, 0, 0)
        tgl.addWidget(self._telem_1hz_btn,  0, 1)
        tgl.addWidget(self._force_pkt_btn,  1, 0)
        tgl.addWidget(self._get_status_btn, 1, 1)

        tgl.addWidget(QLabel("S1 LoRa MHz:"), 2, 0)
        self._lora_s1_edit = QLineEdit(f"{config.LORA_S1_FREQ_MHZ:.3f}"); self._lora_s1_edit.setMaximumWidth(75)
        tgl.addWidget(self._lora_s1_edit, 2, 1)
        self._lora_set_s1_btn = self._mkbtn("Set S1", lambda: self._send_lora(1), "confirm")
        tgl.addWidget(self._lora_set_s1_btn, 2, 2)

        tgl.addWidget(QLabel("S2 LoRa MHz:"), 3, 0)
        self._lora_s2_edit = QLineEdit(f"{config.LORA_S2_FREQ_MHZ:.3f}"); self._lora_s2_edit.setMaximumWidth(75)
        tgl.addWidget(self._lora_s2_edit, 3, 1)
        self._lora_set_s2_btn = self._mkbtn("Set S2", lambda: self._send_lora(2), "confirm")
        tgl.addWidget(self._lora_set_s2_btn, 3, 2)
        layout.addWidget(tg)

        # ── 6. TEST / PAYLOAD ───────────────────────────────────
        tst = QGroupBox("Test Commands"); tstl = QGridLayout(tst)
        self._sol_btn = self._mkbtn(
            "🔥 Fire Solenoid  [S2 — TEST_IDLE only]",
            self._fire_sol, "danger")
        self._sol_btn.setEnabled(False)
        self._sol_btn.setToolTip(
            "Stage 2 only — no solenoid on Stage 1.\n"
            "S2 must be in TEST_IDLE.  Rejected by Teensy in other modes.")
        tstl.addWidget(self._sol_btn, 0, 0, 1, 2)
        self._sol_stage_lbl = QLabel("⬆ N/A — Stage 1 has no solenoid")
        self._sol_stage_lbl.setObjectName("telem_label")
        self._sol_stage_lbl.setAlignment(Qt.AlignCenter)
        self._sol_stage_lbl.setVisible(True)
        tstl.addWidget(self._sol_stage_lbl, 1, 0, 1, 2)
        tstl.addWidget(self._mkbtn("Force Pkt",  lambda: self._send(P.CMD_FORCE_PACKET)), 2, 0)
        tstl.addWidget(self._mkbtn("Get Status", lambda: self._send(P.CMD_GET_STATUS)),   2, 1)
        layout.addWidget(tst)

        # ── 7. FULL SYSTEM TEST — BATTERY EVALUATION ────────────
        fst = QGroupBox("Full System Test — Battery Evaluation")
        fstl = QGridLayout(fst); fstl.setSpacing(4)

        warn = QLabel(
            "⚡ Fires VTX + RunCam + 10Hz telem simultaneously for 90s.\n"
            "Stage 2: FIRES SOLENOID (one-shot — destructive test).\n"
            "Ground station auto-starts all recording."
        )
        warn.setObjectName("telem_label")
        warn.setWordWrap(True)
        warn.setStyleSheet("color:#f0a500;")
        fstl.addWidget(warn, 0, 0, 1, 2)

        self._fst_s1_btn   = self._mkbtn("▶  Run S1 Only",      lambda: self._send_full_sys_test([1]),    "sys_test")
        self._fst_s2_btn   = self._mkbtn("▶  Run S2 Only",      lambda: self._send_full_sys_test([2]),    "sys_test")
        self._fst_both_btn = self._mkbtn("▶▶  Run Both Stages", lambda: self._send_full_sys_test([1, 2]), "sys_test")

        fstl.addWidget(self._fst_s1_btn,   1, 0)
        fstl.addWidget(self._fst_s2_btn,   1, 1)
        fstl.addWidget(self._fst_both_btn, 2, 0, 1, 2)

        self._fst_countdown = QLabel("Test: idle")
        self._fst_countdown.setObjectName("telem_label")
        fstl.addWidget(self._fst_countdown, 3, 0, 1, 2)

        layout.addWidget(fst)

        # ── 8. MAP ──────────────────────────────────────────────
        mg = QGroupBox("Map"); mgl = QGridLayout(mg)
        self.clear_tracks_btn = QPushButton("Clear Tracks")
        mgl.addWidget(self.clear_tracks_btn, 0, 0, 1, 3)

        mgl.addWidget(QLabel("Jump to site:"), 1, 0)
        self.site_btns = {}
        col = 1
        for key, site in config.MAP_SITES.items():
            b = QPushButton(key)
            b.setToolTip(site["name"])
            self.site_btns[key] = b
            mgl.addWidget(b, 1, col)
            col += 1
        layout.addWidget(mg)

        layout.addStretch()

        # ── 9. COMMAND LOG ──────────────────────────────────────
        lg = QGroupBox("Command Log"); ll = QVBoxLayout(lg)
        self._log = QTextEdit(); self._log.setReadOnly(True); self._log.setMaximumHeight(120)
        ll.addWidget(self._log)
        layout.addWidget(lg)

        self._refresh_stage_ui()

    # ------------------------------------------------------------------
    def _stage_sel(self):
        sender = self.sender()
        for b in [self._btn_s1, self._btn_s2, self._btn_both]: b.setChecked(b is sender)
        self._refresh_stage_ui()

    def _refresh_stage_ui(self):
        s2_in_scope = self._btn_s2.isChecked() or self._btn_both.isChecked()
        sol_ok = s2_in_scope and (self._s2_ground_mode == P.GM_TEST_IDLE)
        self._sol_btn.setEnabled(sol_ok)
        self._sol_stage_lbl.setVisible(not s2_in_scope)
        self._fst_s2_btn.setEnabled(self._s2_ground_mode == P.GM_TEST_IDLE)

    def _targets(self) -> list:
        if self._btn_s1.isChecked():   return [1]
        if self._btn_s2.isChecked():   return [2]
        if self._btn_both.isChecked(): return [1, 2]
        return [1]

    def _radio(self, stage: int) -> RadioWorker:
        return self._r1 if stage == 1 else self._r2

    # ------------------------------------------------------------------
    def _send_mode(self, cmd: int, stages: list):
        names = {P.CMD_MODE_TEST:"TEST_IDLE", P.CMD_MODE_PAD:"PAD_IDLE", P.CMD_MODE_LAUNCH:"LAUNCH_READY"}
        for s in stages:
            self._radio(s).send_frame(P.build_standard_frame(s, cmd))
            self._log_msg("SENT", cmd, s, f"→ {names.get(cmd,'?')}")
            if self._data.is_recording:
                self._data.record_event(f"MODE_{names.get(cmd)} → S{s}")

    def update_ground_modes(self, s1_mode: int, s2_mode: int):
        self._s1_ground_mode = s1_mode
        self._s2_ground_mode = s2_mode
        colors = {0:"#f0a500", 1:"#58a6ff", 2:"#3fb950"}
        def span(m): return f'<span style="color:{colors.get(m,"#c9d1d9")};font-weight:bold">{P.ground_mode_name(m)}</span>'
        self._gm_s1_ind.setText(f"S1: {span(s1_mode)}"); self._gm_s1_ind.setTextFormat(Qt.RichText)
        self._gm_s2_ind.setText(f"S2: {span(s2_mode)}"); self._gm_s2_ind.setTextFormat(Qt.RichText)
        self._refresh_stage_ui()

    # ------------------------------------------------------------------
    def _send(self, cmd: int):
        for s in self._targets():
            self._radio(s).send_frame(P.build_standard_frame(s, cmd))
            self._log_msg("SENT", cmd, s)
            if self._data.is_recording:
                self._data.record_event(f"CMD {P.CMD_NAMES.get(cmd,'?')} → S{s}")

    def _send_lora(self, stage: int):
        edit = self._lora_s1_edit if stage == 1 else self._lora_s2_edit
        try: freq = float(edit.text())
        except ValueError: self._log_msg("ERROR", P.CMD_SET_LORA_FREQ, stage, "Invalid MHz"); return
        if not 902.0 <= freq <= 928.0: self._log_msg("ERROR", P.CMD_SET_LORA_FREQ, stage, f"{freq} outside 902-928"); return
        self._radio(stage).send_frame(P.build_set_lora_freq_frame(stage, freq))
        self._radio(stage).set_intended_freq(freq)
        self._log_msg("SENT", P.CMD_SET_LORA_FREQ, stage, f"{freq:.3f} MHz — M0 auto-retunes on STATUS")
        if self._data.is_recording: self._data.record_event(f"SET_LORA_FREQ {freq:.3f} → S{stage}")

    def _send_vtx(self, stage: int):
        combo = self._vtx_s1_edit if stage == 1 else self._vtx_s2_edit
        freq = combo.currentData()
        if freq is None:
            self._log_msg("ERROR", P.CMD_SET_VTX_FREQ, stage, "No channel selected")
            return
        self._radio(stage).send_frame(P.build_set_vtx_freq_frame(stage, freq))
        label = combo.currentText()
        self._log_msg("SENT", P.CMD_SET_VTX_FREQ, stage, f"{label}")
        if self._data.is_recording:
            self._data.record_event(f"SET_VTX_FREQ {label} → S{stage}")

    def _send_vtx_power(self, stage: int, level: int):
        try: frame = P.build_set_vtx_power_frame(stage, level)
        except ValueError as e: self._log_msg("ERROR", P.CMD_SET_VTX_POWER, stage, str(e)); return
        self._radio(stage).send_frame(frame)
        label = P.VTX_POWER_LABELS.get(level, f"idx{level}")
        self._log_msg("SENT", P.CMD_SET_VTX_POWER, stage, f"Flight pwr → {label}")
        if self._data.is_recording: self._data.record_event(f"SET_VTX_POWER {label} → S{stage}")

    def _fire_sol(self):
        if 2 not in self._targets(): self._log_msg("BLOCKED", P.CMD_FIRE_SOLENOID, 0, "Select S2 or Both"); return
        if self._s2_ground_mode != P.GM_TEST_IDLE:
            self._log_msg("BLOCKED", P.CMD_FIRE_SOLENOID, 2, "S2 must be TEST_IDLE"); return
        self._r2.send_frame(P.build_standard_frame(2, P.CMD_FIRE_SOLENOID))
        self._log_msg("SENT ⚠", P.CMD_FIRE_SOLENOID, 2, "Water payload test")
        if self._data.is_recording: self._data.record_event("CMD FIRE_SOLENOID → S2")

    def _send_full_sys_test(self, stages: list):
        not_ready = []
        for s in stages:
            mode = self._s1_ground_mode if s == 1 else self._s2_ground_mode
            if mode != P.GM_TEST_IDLE:
                not_ready.append(f"S{s} ({P.ground_mode_name(mode)})")

        if not_ready:
            QMessageBox.warning(
                self, "Cannot Start Test",
                f"The following stages are not in TEST_IDLE mode:\n\n" +
                "\n".join(not_ready) + "\n\n" +
                "Set all stages to TEST_IDLE before running full system test."
            )
            return

        if 2 in stages:
            reply = QMessageBox.question(
                self, "Stage 2 Full System Test",
                "Stage 2 full system test will FIRE THE WATER PAYLOAD SOLENOID.\n\n"
                "This is a one-shot event — the solenoid cannot fire again until "
                "the board is power-cycled.\n\n"
                "Proceed?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return

        for s in stages:
            radio = self._r1 if s == 1 else self._r2
            radio.send_frame(P.build_full_sys_test_frame(s))
            self._log_msg("SENT", P.CMD_FULL_SYS_TEST, s, "Full system test started")
            if self._data.is_recording:
                self._data.record_event(f"CMD FULL_SYS_TEST → S{s}")

        self.full_sys_test_requested.emit(stages)

    def set_test_countdown(self, text: str):
        self._fst_countdown.setText(text)

    # ------------------------------------------------------------------
    @pyqtSlot(int, dict)
    def on_ack(self, stage: int, ack: dict):
        result = "✅ OK" if ack["result"] == P.ACK_OK else "❌ REJECTED"
        self._log_msg(result, ack["cmd"], stage, f"← ACK from S{stage}")

    @pyqtSlot(int, str)
    def on_raw_log(self, _stage: int, msg: str):
        if any(kw in msg for kw in ["CMD","ACK","SENT","ERROR","BLOCKED","MODE","RETUNE","RADIO","HEARTBEAT"]):
            self._append(msg)

    def _mkbtn(self, text: str, slot, style: str = "", tip: str = "") -> QPushButton:
        b = QPushButton(text)
        if style: b.setObjectName(style)
        if tip:   b.setToolTip(tip)
        b.clicked.connect(slot)
        return b

    def _log_msg(self, pfx: str, cmd: int, stage: int, extra: str = ""):
        name = P.CMD_NAMES.get(cmd, f"0x{cmd:02X}")
        s    = f"S{stage}" if stage > 0 else "  "
        msg  = f"[{pfx:8s}] {s} {name}"
        if extra: msg += f"  — {extra}"
        self._append(msg)

    def _append(self, msg: str):
        self._log.append(msg)
        self._log.verticalScrollBar().setValue(self._log.verticalScrollBar().maximum())

    @property
    def test_mode(self) -> bool:
        return False


class DebugConsole(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Raw Serial Log", parent)
        l = QVBoxLayout(self)
        self._log = QTextEdit(); self._log.setReadOnly(True)
        l.addWidget(self._log)

    def append(self, msg):
        self._log.append(msg)
        sb = self._log.verticalScrollBar(); sb.setValue(sb.maximum())


# =============================================================================
#  TEST STATS PANEL
# =============================================================================

class TestStatsPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setSpacing(4)

        grid = QGridLayout()
        grid.setSpacing(6)

        s1_hdr = QLabel("Stage 1"); s1_hdr.setObjectName("state_label")
        s2_hdr = QLabel("Stage 2"); s2_hdr.setObjectName("state_label")
        grid.addWidget(s1_hdr, 0, 0)
        grid.addWidget(s2_hdr, 0, 1)

        self._s1_status  = QLabel("Idle"); self._s1_status.setObjectName("telem_label")
        self._s1_packets = QLabel("—");    self._s1_packets.setObjectName("telem_label")
        self._s1_loss    = QLabel("—");    self._s1_loss.setObjectName("telem_label")
        self._s1_gaps    = QLabel("—");    self._s1_gaps.setObjectName("telem_label")
        self._s1_avg_rssi= QLabel("—");    self._s1_avg_rssi.setObjectName("telem_label")
        self._s1_min_rssi= QLabel("—");    self._s1_min_rssi.setObjectName("telem_label")
        self._s1_temp    = QLabel("—");    self._s1_temp.setObjectName("telem_label")
        self._s1_elapsed = QLabel("—");    self._s1_elapsed.setObjectName("telem_label")

        self._s2_status  = QLabel("Idle"); self._s2_status.setObjectName("telem_label")
        self._s2_packets = QLabel("—");    self._s2_packets.setObjectName("telem_label")
        self._s2_loss    = QLabel("—");    self._s2_loss.setObjectName("telem_label")
        self._s2_gaps    = QLabel("—");    self._s2_gaps.setObjectName("telem_label")
        self._s2_avg_rssi= QLabel("—");    self._s2_avg_rssi.setObjectName("telem_label")
        self._s2_min_rssi= QLabel("—");    self._s2_min_rssi.setObjectName("telem_label")
        self._s2_temp    = QLabel("—");    self._s2_temp.setObjectName("telem_label")
        self._s2_elapsed = QLabel("—");    self._s2_elapsed.setObjectName("telem_label")

        row = 1
        grid.addWidget(self._s1_status,  row, 0); grid.addWidget(self._s2_status,  row, 1); row += 1
        grid.addWidget(QLabel("Packets:"),row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_packets, row, 0); grid.addWidget(self._s2_packets, row, 1); row += 1
        grid.addWidget(QLabel("Loss:"),   row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_loss,    row, 0); grid.addWidget(self._s2_loss,    row, 1); row += 1
        grid.addWidget(QLabel("Gaps:"),   row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_gaps,    row, 0); grid.addWidget(self._s2_gaps,    row, 1); row += 1
        grid.addWidget(QLabel("Avg RSSI:"),row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_avg_rssi,row, 0); grid.addWidget(self._s2_avg_rssi,row, 1); row += 1
        grid.addWidget(QLabel("Min RSSI:"),row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_min_rssi,row, 0); grid.addWidget(self._s2_min_rssi,row, 1); row += 1
        grid.addWidget(QLabel("Temp Range:"),row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_temp,    row, 0); grid.addWidget(self._s2_temp,    row, 1); row += 1
        grid.addWidget(QLabel("Elapsed:"),row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_elapsed, row, 0); grid.addWidget(self._s2_elapsed, row, 1); row += 1

        layout.addLayout(grid)

        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setFormat("No test running")
        self._progress.setTextVisible(True)
        layout.addWidget(self._progress)

    def update_stats(self, stage: int, summary: dict):
        widgets = {
            1: {"packets": self._s1_packets, "loss": self._s1_loss,
                "gaps": self._s1_gaps, "avg_rssi": self._s1_avg_rssi,
                "min_rssi": self._s1_min_rssi, "temp": self._s1_temp,
                "elapsed": self._s1_elapsed},
            2: {"packets": self._s2_packets, "loss": self._s2_loss,
                "gaps": self._s2_gaps, "avg_rssi": self._s2_avg_rssi,
                "min_rssi": self._s2_min_rssi, "temp": self._s2_temp,
                "elapsed": self._s2_elapsed},
        }
        w = widgets.get(stage)
        if not w: return

        w["packets"].setText(f"{summary['received']} / {summary['expected']}")
        loss_pct = summary['loss_pct']
        loss_color = "#ff7b72" if loss_pct > 5.0 else "#3fb950"
        w["loss"].setText(f"{loss_pct:.1f}%")
        w["loss"].setStyleSheet(f"color:{loss_color};font-weight:bold;")
        w["gaps"].setText(str(summary['gaps']))
        w["avg_rssi"].setText(f"{summary['avg_rssi']:.1f} dBm")
        w["min_rssi"].setText(f"{summary['min_rssi']} dBm")
        w["temp"].setText(f"{summary['temp_min_c']:.1f} / {summary['temp_max_c']:.1f} °C")
        w["elapsed"].setText(f"{summary['elapsed_s']:.0f}s")

    def set_status(self, stage: int, status_str: str):
        label = self._s1_status if stage == 1 else self._s2_status
        label.setText(status_str)
        if "Running" in status_str:
            label.setStyleSheet("color:#f0a500;font-weight:bold;")
        elif "Complete" in status_str:
            label.setStyleSheet("color:#3fb950;font-weight:bold;")
        else:
            label.setStyleSheet("color:#8b949e;")

    def reset(self):
        for stage in [1, 2]:
            self.set_status(stage, "Idle")
            widgets = {
                1: [self._s1_packets, self._s1_loss, self._s1_gaps,
                    self._s1_avg_rssi, self._s1_min_rssi, self._s1_temp, self._s1_elapsed],
                2: [self._s2_packets, self._s2_loss, self._s2_gaps,
                    self._s2_avg_rssi, self._s2_min_rssi, self._s2_temp, self._s2_elapsed],
            }
            for w in widgets[stage]:
                w.setText("—"); w.setStyleSheet("")
        self._progress.setValue(0)
        self._progress.setFormat("No test running")

    def set_progress(self, pct: int, label_str: str):
        self._progress.setValue(pct)
        self._progress.setFormat(label_str)


# =============================================================================
#  GPS PANEL
# =============================================================================

class GPSPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        g = QGridLayout(self)
        self.conn_lbl = QLabel("⚫ Disconnected"); self.conn_lbl.setObjectName("status_err")
        g.addWidget(QLabel("Status:"),    0, 0); g.addWidget(self.conn_lbl, 0, 1, 1, 3)
        pairs = [
            ("S1 Position:", "gps_s1_pos"), ("S2 Position:", "gps_s2_pos"),
            ("S1 Altitude:", "gps_s1_alt"), ("S2 Altitude:", "gps_s2_alt"),
            ("S1 Vert Vel:", "gps_s1_vel"), ("S2 Vert Vel:", "gps_s2_vel"),
        ]
        for i, (lbl, attr) in enumerate(pairs):
            w = QLabel("—"); w.setObjectName("telem_label")
            setattr(self, attr, w)
            row, col = 1 + i // 2, (i % 2) * 2
            g.addWidget(QLabel(lbl), row, col); g.addWidget(w, row, col + 1)
        self.lost_lbl = QLabel("Lost rocket: none"); self.lost_lbl.setObjectName("fault_err")
        g.addWidget(self.lost_lbl, 4, 0, 1, 4)
        g.setColumnStretch(1, 1); g.setColumnStretch(3, 1)

    def set_connected(self, c):
        if c:
            self.conn_lbl.setText("🟢 Featherweight GPS Connected"); self.conn_lbl.setObjectName("status_ok")
        else:
            self.conn_lbl.setText("🔴 Featherweight GPS Disconnected"); self.conn_lbl.setObjectName("status_err")
        self.conn_lbl.style().unpolish(self.conn_lbl); self.conn_lbl.style().polish(self.conn_lbl)


# =============================================================================
#  MAIN WINDOW
# =============================================================================

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle(config.APP_TITLE)
        if config.DARK_MODE:
            self.setStyleSheet(DARK_STYLE)

        self._data_rec = DataRecorder()

        self._s1_ground_mode = P.GM_TEST_IDLE
        self._s2_ground_mode = P.GM_TEST_IDLE

        self._stats_s1 = PacketStatsTracker(stage=1, expected_rate_hz=10.0)
        self._stats_s2 = PacketStatsTracker(stage=2, expected_rate_hz=10.0)
        self._test_active_stages = []
        self._test_timer = QTimer(self)
        self._test_timer.setInterval(1000)
        self._test_elapsed_s = 0
        self._test_timer_connected = False

        self._radio_s1 = RadioWorker(1, config.M0_S1_PORT, config.M0_S1_BAUD)
        self._radio_s2 = RadioWorker(2, config.M0_S2_PORT, config.M0_S2_BAUD)
        self._gps_wkr  = FWGPSWorker(config.FW_GPS_PORT, config.FW_GPS_BAUD)
        self._vid_s1   = VideoWorker(1, config.VIDEO_S1_INDEX,
                                     config.VIDEO_WIDTH, config.VIDEO_HEIGHT,
                                     config.VIDEO_FPS)
        self._vid_s2   = VideoWorker(2, config.VIDEO_S2_INDEX,
                                     config.VIDEO_WIDTH, config.VIDEO_HEIGHT,
                                     config.VIDEO_FPS)

        self._showcase_active = config.SHOWCASE_MODE_DEFAULT
        self._showcase_worker: "VideoFileWorker | None" = None
        self._sc_status_slot = None
        self._sc_conn_slot   = None

        self._log_window = None

        self._build_ui()
        self._wire_signals()
        self._start_workers()
        self.showMaximized()

    # ------------------------------------------------------------------

    def _build_ui(self):
        # ── Outer vertical scroll area ────────────────────────────────
        outer_scroll = QScrollArea()
        outer_scroll.setWidgetResizable(True)
        outer_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        outer_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        outer_scroll.setFrameShape(QFrame.NoFrame)
        self.setCentralWidget(outer_scroll)

        content = QWidget()
        outer_scroll.setWidget(content)
        root = QVBoxLayout(content)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(2)

        # ── Video panels ─────────────────────────────────────────────
        self._video_s2 = VideoPanel(2)
        self._video_s1 = VideoPanel(1)
        self._video_s2.setMinimumHeight(320)
        self._video_s1.setMinimumHeight(320)

        # ── Map ──────────────────────────────────────────────────────
        self._map = MapWidget()
        self._map.setMinimumHeight(320)

        # ── GPS panel (bottom right) ──────────────────────────────────
        self._gps_panel = GPSPanel()
        gps_container = QWidget()
        gps_container.setMinimumHeight(320)
        gps_vbox = QVBoxLayout(gps_container)
        gps_vbox.setContentsMargins(2, 2, 2, 2)
        gps_vbox.setSpacing(3)
        gps_hdr = QLabel("GPS MAP")
        gps_hdr.setObjectName("section_header")
        gps_vbox.addWidget(gps_hdr)
        gps_vbox.addWidget(self._map, 1)
        gps_vbox.addWidget(self._gps_panel)

        # ── Telemetry (top right) — S1 above S2 ──────────────────────
        telem_container = QWidget()
        telem_container.setMinimumHeight(320)
        tc_layout = QVBoxLayout(telem_container)
        tc_layout.setContentsMargins(2, 2, 2, 2)
        tc_layout.setSpacing(3)
        telem_hdr = QLabel("LIVE TELEMETRY")
        telem_hdr.setObjectName("section_header")
        tc_layout.addWidget(telem_hdr)
        self._telem_s1 = TelemetryPanel(1)
        self._telem_s2 = TelemetryPanel(2)
        tc_layout.addWidget(self._telem_s1, 1)
        tc_layout.addWidget(self._telem_s2, 1)

        # ── Splitter tree ─────────────────────────────────────────────
        self._left_splitter = QSplitter(Qt.Vertical)
        self._left_splitter.addWidget(self._video_s2)
        self._left_splitter.addWidget(self._video_s1)

        self._right_splitter = QSplitter(Qt.Vertical)
        self._right_splitter.addWidget(telem_container)
        self._right_splitter.addWidget(gps_container)

        self._h_splitter = QSplitter(Qt.Horizontal)
        self._h_splitter.addWidget(self._left_splitter)
        self._h_splitter.addWidget(self._right_splitter)
        self._h_splitter.setMinimumHeight(640)
        self._h_splitter.setMinimumWidth(900)

        root.addWidget(self._h_splitter, 1)

        # ── Command + Recording panels (instantiated; widgets extracted) ──
        self._cmd_panel = CommandPanel(self._radio_s1, self._radio_s2, self._data_rec)
        self._cmd_panel.clear_tracks_btn.clicked.connect(self._map.clear_tracks)
        for key, btn in self._cmd_panel.site_btns.items():
            site = config.MAP_SITES[key]
            btn.clicked.connect(
                lambda _=False, s=site: self._map.set_site(s["lat"], s["lon"], s["zoom"])
            )
        self._rec_panel = RecordingPanel(self._vid_s1, self._vid_s2, self._data_rec)

        # ── Debug + test stats (live even when log window is hidden) ──
        self._debug = DebugConsole()
        self._test_stats_panel = TestStatsPanel()

        # ── Controls bar in horizontal scroll area ────────────────────
        ctrl_scroll = QScrollArea()
        ctrl_scroll.setWidgetResizable(True)
        ctrl_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        ctrl_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        ctrl_scroll.setFrameShape(QFrame.NoFrame)
        ctrl_scroll.setFixedHeight(220)
        ctrl_scroll.setWidget(self._build_controls_bar())
        root.addWidget(ctrl_scroll)

        QTimer.singleShot(100, self._set_splitter_sizes)

    # ------------------------------------------------------------------

    def _build_controls_bar(self) -> QWidget:
        bar = QWidget()
        bar_layout = QHBoxLayout(bar)
        bar_layout.setSpacing(4)
        bar_layout.setContentsMargins(6, 6, 6, 6)

        # ── Group 1 — Ground Mode (210px) ────────────────────────────
        g1 = QGroupBox("Ground Mode")
        g1.setObjectName("controls_group")
        g1l = QGridLayout(g1)
        g1l.setSpacing(2)
        g1l.addWidget(QLabel("S1:"),   0, 0)
        g1l.addWidget(self._cmd_panel._gm_s1_test,   0, 1)
        g1l.addWidget(self._cmd_panel._gm_s1_pad,    0, 2)
        g1l.addWidget(self._cmd_panel._gm_s1_launch, 0, 3)
        g1l.addWidget(QLabel("S2:"),   1, 0)
        g1l.addWidget(self._cmd_panel._gm_s2_test,   1, 1)
        g1l.addWidget(self._cmd_panel._gm_s2_pad,    1, 2)
        g1l.addWidget(self._cmd_panel._gm_s2_launch, 1, 3)
        g1l.addWidget(QLabel("Both:"), 2, 0)
        g1l.addWidget(self._cmd_panel._gm_both_test,   2, 1)
        g1l.addWidget(self._cmd_panel._gm_both_pad,    2, 2)
        g1l.addWidget(self._cmd_panel._gm_both_launch, 2, 3)
        g1l.addWidget(self._cmd_panel._gm_s1_ind, 3, 0, 1, 2)
        g1l.addWidget(self._cmd_panel._gm_s2_ind, 3, 2, 1, 2)
        bar_layout.addWidget(g1)

        # ── Group 2 — Stage + Camera (170px) ─────────────────────────
        g2 = QGroupBox("Stage / Camera")
        g2.setObjectName("controls_group")
        g2l = QGridLayout(g2)
        g2l.setSpacing(2)
        g2l.addWidget(self._cmd_panel._btn_s1,   0, 0)
        g2l.addWidget(self._cmd_panel._btn_s2,   0, 1)
        g2l.addWidget(self._cmd_panel._btn_both, 0, 2)
        g2l.addWidget(self._cmd_panel._cam_rec_on_btn,  1, 0, 1, 2)
        g2l.addWidget(self._cmd_panel._cam_rec_off_btn, 1, 2)
        bar_layout.addWidget(g2)

        # ── Group 3 — Video / VTX (260px) ────────────────────────────
        g3 = QGroupBox("Video / VTX")
        g3.setObjectName("controls_group")
        g3l = QGridLayout(g3)
        g3l.setSpacing(2)
        g3l.addWidget(self._cmd_panel._video_on_btn,  0, 0)
        g3l.addWidget(self._cmd_panel._video_off_btn, 0, 1)
        g3l.addWidget(QLabel("S1 VTX:"),              1, 0)
        g3l.addWidget(self._cmd_panel._vtx_s1_edit,   1, 1)
        g3l.addWidget(self._cmd_panel._vtx_set_s1_btn, 1, 2)
        g3l.addWidget(QLabel("S2 VTX:"),              2, 0)
        g3l.addWidget(self._cmd_panel._vtx_s2_edit,   2, 1)
        g3l.addWidget(self._cmd_panel._vtx_set_s2_btn, 2, 2)
        s1_pwr = QHBoxLayout()
        for b in self._cmd_panel._vp_s1_btns:
            s1_pwr.addWidget(b)
        g3l.addLayout(s1_pwr, 3, 0, 1, 3)
        s2_pwr = QHBoxLayout()
        for b in self._cmd_panel._vp_s2_btns:
            s2_pwr.addWidget(b)
        g3l.addLayout(s2_pwr, 4, 0, 1, 3)
        bar_layout.addWidget(g3)

        # ── Group 4 — Telemetry / LoRa (230px) ───────────────────────
        g4 = QGroupBox("Telemetry")
        g4.setObjectName("controls_group")
        g4l = QGridLayout(g4)
        g4l.setSpacing(2)
        g4l.addWidget(self._cmd_panel._telem_10hz_btn, 0, 0)
        g4l.addWidget(self._cmd_panel._telem_1hz_btn,  0, 1)
        g4l.addWidget(self._cmd_panel._force_pkt_btn,  0, 2)
        g4l.addWidget(self._cmd_panel._get_status_btn, 0, 3)
        g4l.addWidget(QLabel("S1 LoRa:"),              1, 0)
        g4l.addWidget(self._cmd_panel._lora_s1_edit,   1, 1)
        g4l.addWidget(self._cmd_panel._lora_set_s1_btn, 1, 2)
        g4l.addWidget(QLabel("S2 LoRa:"),              2, 0)
        g4l.addWidget(self._cmd_panel._lora_s2_edit,   2, 1)
        g4l.addWidget(self._cmd_panel._lora_set_s2_btn, 2, 2)
        bar_layout.addWidget(g4)

        # ── Group 5 — Test / Payload (210px) ─────────────────────────
        g5 = QGroupBox("Test")
        g5.setObjectName("controls_group")
        g5l = QGridLayout(g5)
        g5l.setSpacing(2)
        g5l.addWidget(self._cmd_panel._sol_btn,       0, 0, 1, 2)
        g5l.addWidget(self._cmd_panel._sol_stage_lbl, 1, 0, 1, 2)
        g5l.addWidget(self._cmd_panel._fst_s1_btn,    2, 0)
        g5l.addWidget(self._cmd_panel._fst_s2_btn,    2, 1)
        g5l.addWidget(self._cmd_panel._fst_both_btn,  3, 0, 1, 2)
        g5l.addWidget(self._cmd_panel._fst_countdown, 4, 0, 1, 2)
        bar_layout.addWidget(g5)

        # ── Group 6 — Recording + Showcase (200px) ───────────────────
        g6 = QGroupBox("Recording")
        g6.setObjectName("controls_group")
        g6l = QGridLayout(g6)
        g6l.setSpacing(2)
        g6l.addWidget(self._rec_panel._name_edit, 0, 0, 1, 2)
        g6l.addWidget(self._rec_panel._rec_btn,   1, 0)
        g6l.addWidget(self._rec_panel._stop_btn,  1, 1)
        g6l.addWidget(self._rec_panel._s1v_lbl,   2, 0, 1, 2)
        g6l.addWidget(self._rec_panel._s2v_lbl,   3, 0, 1, 2)
        g6l.addWidget(self._rec_panel._data_lbl,  4, 0, 1, 2)
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color:#30363d;")
        g6l.addWidget(sep, 5, 0, 1, 2)
        g6l.addWidget(self._rec_panel._showcase_btn,    6, 0, 1, 2)
        g6l.addWidget(self._rec_panel._showcase_status, 7, 0, 1, 2)
        bar_layout.addWidget(g6)

        # ── Group 7 — Log (120px) ─────────────────────────────────────
        g7 = QGroupBox("Log")
        g7.setObjectName("controls_group")
        g7l = QVBoxLayout(g7)
        g7l.setSpacing(2)
        open_log_btn = QPushButton("📋  Open Log Window")
        open_log_btn.setObjectName("confirm")
        open_log_btn.clicked.connect(self._show_log_window)
        g7l.addWidget(open_log_btn)
        self._last_cmd_label = QLabel("—")
        self._last_cmd_label.setObjectName("telem_label")
        self._last_cmd_label.setWordWrap(True)
        g7l.addWidget(self._last_cmd_label)
        g7l.addStretch()
        bar_layout.addWidget(g7)

        return bar

    # ------------------------------------------------------------------

    def _set_splitter_sizes(self):
        size = self.size()
        w = size.width()
        h = max(1, size.height() - 200 - 2)
        self._h_splitter.setSizes([w // 2, w // 2])
        self._left_splitter.setSizes([h // 2, h // 2])
        self._right_splitter.setSizes([h // 2, h // 2])

    def _show_log_window(self):
        if self._log_window is not None:
            if self._log_window.isVisible():
                self._log_window.raise_()
                self._log_window.activateWindow()
            else:
                self._log_window.show()
            return

        self._log_window = QMainWindow()
        self._log_window.setWindowTitle("TRES Titan — System Log")
        self._log_window.resize(900, 600)
        if config.DARK_MODE:
            self._log_window.setStyleSheet(DARK_STYLE)

        cw = QWidget()
        self._log_window.setCentralWidget(cw)
        lw_layout = QVBoxLayout(cw)

        tabs = QTabWidget()
        tabs.addTab(self._debug, "Serial Log")
        tabs.addTab(self._test_stats_panel, "Test Stats")
        lw_layout.addWidget(tabs, 1)

        close_btn = QPushButton("Close Log Window")
        close_btn.clicked.connect(self._log_window.hide)
        lw_layout.addWidget(close_btn)

        self._log_window.show()

    # ------------------------------------------------------------------

    def _wire_signals(self):
        self._radio_s1.flight_data.connect(lambda s, d: self._telem_s1.update_flight(d))
        self._radio_s2.flight_data.connect(lambda s, d: self._telem_s2.update_flight(d))
        self._radio_s1.flight_data.connect(self._on_flight_data)
        self._radio_s2.flight_data.connect(self._on_flight_data)

        self._radio_s1.status_data.connect(lambda s, d: self._telem_s1.update_status(d))
        self._radio_s2.status_data.connect(lambda s, d: self._telem_s2.update_status(d))
        self._radio_s1.status_data.connect(self._on_status_data)
        self._radio_s2.status_data.connect(self._on_status_data)

        self._radio_s1.ack_received.connect(self._cmd_panel.on_ack)
        self._radio_s2.ack_received.connect(self._cmd_panel.on_ack)
        self._radio_s1.ack_received.connect(self._on_ack_label)
        self._radio_s2.ack_received.connect(self._on_ack_label)

        self._cmd_panel.full_sys_test_requested.connect(self._on_full_sys_test_requested)

        self._radio_s1.connection_changed.connect(lambda s, c: self._telem_s1.set_connected(c))
        self._radio_s2.connection_changed.connect(lambda s, c: self._telem_s2.set_connected(c))

        for r in [self._radio_s1, self._radio_s2]:
            r.raw_log.connect(lambda s, m: self._debug.append(m))
            r.raw_log.connect(self._cmd_panel.on_raw_log)

        self._gps_wkr.position_update.connect(self._on_gps)
        self._gps_wkr.lost_rocket.connect(self._on_lost_rocket)
        self._gps_wkr.connection_changed.connect(self._gps_panel.set_connected)
        self._gps_wkr.raw_log.connect(self._debug.append)

        self._vid_s1.frame_ready.connect(self._video_s1.update_frame)
        self._vid_s2.frame_ready.connect(self._video_s2.update_frame)
        self._vid_s1.status_message.connect(lambda s, m: self._video_s1.set_status(m))
        self._vid_s2.status_message.connect(lambda s, m: self._video_s2.set_status(m))
        self._vid_s1.recording_changed.connect(
            lambda s, a, p: self._video_s1.set_recording(a, p))
        self._vid_s2.recording_changed.connect(
            lambda s, a, p: self._video_s2.set_recording(a, p))

        self._rec_panel._showcase_btn.clicked.connect(self._toggle_showcase)
        self._update_showcase_btn_state()

    # ------------------------------------------------------------------

    @pyqtSlot(int, dict)
    def _on_ack_label(self, stage: int, ack: dict):
        result = "✅ OK" if ack["result"] == P.ACK_OK else "❌ REJECTED"
        name = P.CMD_NAMES.get(ack["cmd"], f"0x{ack['cmd']:02X}")
        self._last_cmd_label.setText(f"S{stage} {name}: {result}")

    @pyqtSlot(int, dict)
    @pyqtSlot(int, dict)
    def _on_flight_data(self, stage: int, d: dict):
        self._map.update_telemetry(
            stage, d["altitude_ft"], d["velocity_fps"],
            P.state_name(d.get("stage", stage), d["state"]))
        if self._data_rec.is_recording:
            self._data_rec.record_flight(stage, d)

        if stage in self._test_active_stages:
            tracker = self._stats_s1 if stage == 1 else self._stats_s2
            tracker.record_packet(d)

    @pyqtSlot(int, dict)
    def _on_status_data(self, stage: int, d: dict):
        gm = d.get("ground_mode", P.GM_TEST_IDLE)
        if stage == 1:
            self._s1_ground_mode = gm
        else:
            self._s2_ground_mode = gm
        self._cmd_panel.update_ground_modes(
            self._s1_ground_mode, self._s2_ground_mode)
        if self._data_rec.is_recording:
            self._data_rec.record_status(stage, d)

    @pyqtSlot(str, float, float, float, float, float)
    def _on_gps(self, name, lat, lon, alt_ft, vert_vel, horiz_vel):
        launch_ready = (self._s1_ground_mode == P.GM_LAUNCH_READY or
                        self._s2_ground_mode == P.GM_LAUNCH_READY)
        if not launch_ready:
            return

        nl = name.lower()
        stage = 1 if any(k in nl for k in ["s1", "stage1", "booster"]) else 2
        self._map.update_gps(stage, lat, lon, alt_ft, vert_vel, name)

        gp  = self._gps_panel
        pos = f"{lat:.5f}, {lon:.5f}"
        if stage == 1:
            gp.gps_s1_pos.setText(pos); gp.gps_s1_alt.setText(f"{alt_ft:.0f} ft")
            gp.gps_s1_vel.setText(f"{vert_vel:.1f} fps")
        else:
            gp.gps_s2_pos.setText(pos); gp.gps_s2_alt.setText(f"{alt_ft:.0f} ft")
            gp.gps_s2_vel.setText(f"{vert_vel:.1f} fps")

        if self._data_rec.is_recording:
            self._data_rec.record_gps(name, lat, lon, alt_ft, vert_vel, horiz_vel)

    @pyqtSlot(str, float, float, float)
    def _on_lost_rocket(self, name, lat, lon, alt_ft):
        self._map.show_lost_rocket(name, lat, lon, alt_ft)
        self._gps_panel.lost_lbl.setText(
            f"⚠ LOST: {name}  {lat:.5f},{lon:.5f}  {alt_ft:.0f}ft")

    # ------------------------------------------------------------------
    #  Full System Test Lifecycle
    # ------------------------------------------------------------------

    @pyqtSlot(list)
    def _on_full_sys_test_requested(self, stages: list):
        self._test_active_stages = stages

        for stage in stages:
            tracker = self._stats_s1 if stage == 1 else self._stats_s2
            tracker.start()
            self._test_stats_panel.set_status(stage, "Running…")

        if not self._data_rec.is_recording:
            session_name = f"sysTest_{''.join('S'+str(s) for s in stages)}"
            self._rec_panel._name_edit.setText(session_name)
            self._rec_panel._start_all()

        self._test_stats_panel.reset()
        self._show_log_window()
        if self._log_window:
            self._log_window.centralWidget().layout().itemAt(0).widget().setCurrentIndex(1)

        self._test_elapsed_s = 0

        if not self._test_timer_connected:
            self._test_timer.timeout.connect(self._on_test_tick)
            self._test_timer_connected = True

        self._test_timer.start()

        self._debug.append(f"[FULL SYS TEST] Started for stages: {stages}")

    def _on_test_tick(self):
        self._test_elapsed_s += 1
        duration = config.FULL_SYS_TEST_DURATION_S

        pct = min(100, int(self._test_elapsed_s / duration * 100))
        remaining = max(0, duration - self._test_elapsed_s)
        self._test_stats_panel.set_progress(pct, f"Running — {remaining}s remaining")

        self._cmd_panel.set_test_countdown(f"Test: {self._test_elapsed_s}s / {duration}s")

        for stage in self._test_active_stages:
            tracker = self._stats_s1 if stage == 1 else self._stats_s2
            summary = tracker.summary_dict()
            self._test_stats_panel.update_stats(stage, summary)

        if self._test_elapsed_s >= duration:
            self._on_test_complete()

    def _on_test_complete(self):
        self._test_timer.stop()

        for stage in self._test_active_stages:
            tracker = self._stats_s1 if stage == 1 else self._stats_s2
            tracker.stop()
            self._test_stats_panel.set_status(stage, "Complete ✓")

        self._test_stats_panel.set_progress(100, "Test complete")
        self._cmd_panel.set_test_countdown("Test: complete")
        self._rec_panel._stop_all()

        session_name = self._rec_panel._name_edit.text().strip()
        report_paths = []
        for stage in self._test_active_stages:
            tracker = self._stats_s1 if stage == 1 else self._stats_s2
            report_path = tracker.generate_report(
                output_dir="recordings",
                session_name=session_name
            )
            report_paths.append(report_path)
            self._debug.append(f"[FULL SYS TEST] Report saved: {report_path}")

        QMessageBox.information(
            self, "Full System Test Complete",
            "Full System Test Complete\n\n"
            "Test reports saved to recordings/\n"
            "Check the System Test Stats tab for results."
        )

        self._test_active_stages = []

    # ------------------------------------------------------------------
    #  Showcase Mode
    # ------------------------------------------------------------------

    def _resolve_showcase_video(self) -> str:
        """Return the video file path to play. If SHOWCASE_VIDEO_PATH is a
        directory, returns the first .mp4/.avi/.mkv/.mov found inside it."""
        p = config.SHOWCASE_VIDEO_PATH
        if not p:
            return ""
        if os.path.isdir(p):
            for fname in sorted(os.listdir(p)):
                if fname.lower().endswith((".mp4", ".avi", ".mkv", ".mov")):
                    return os.path.join(p, fname)
            return ""
        return p

    def _update_showcase_btn_state(self):
        """Grey out showcase button when no video is available."""
        btn = self._rec_panel._showcase_btn
        lbl = self._rec_panel._showcase_status
        video = self._resolve_showcase_video()
        if not video:
            btn.setEnabled(False)
            if config.SHOWCASE_VIDEO_PATH and os.path.isdir(config.SHOWCASE_VIDEO_PATH):
                btn.setToolTip(
                    f"No video found in {config.SHOWCASE_VIDEO_PATH}\n"
                    "Drop an .mp4/.avi/.mkv/.mov file into that folder.")
                lbl.setText("Showcase: folder empty — add a video file")
            else:
                btn.setToolTip("Set SHOWCASE_VIDEO_PATH in config.py to enable showcase mode.")
                lbl.setText("Showcase: no video path set")
        else:
            btn.setEnabled(True)
            btn.setToolTip(
                f"Play {os.path.basename(video)} on S{config.SHOWCASE_STAGE} panel.\n"
                "Live capture continues on the other panel.")
            lbl.setText(f"Showcase: ready — {os.path.basename(video)}")

    def _toggle_showcase(self):
        if not self._showcase_active:
            if self._data_rec.is_recording:
                QMessageBox.information(self, "Showcase Mode",
                    "Stop recording before enabling showcase mode.")
                return
            self._start_showcase()
        else:
            self._stop_showcase()

    def _start_showcase(self):
        video_path = self._resolve_showcase_video()
        if not video_path:
            return   # button is disabled when no video; guard only
        if config.SHOWCASE_STAGE not in (1, 2):
            QMessageBox.warning(self, "Showcase Mode",
                "SHOWCASE_STAGE must be 1 or 2. Check config.py.")
            return

        if config.SHOWCASE_STAGE == 1:
            old_worker = self._vid_s1
            panel      = self._video_s1
        else:
            old_worker = self._vid_s2
            panel      = self._video_s2

        old_worker.stop()
        old_worker.wait(2000)

        try: old_worker.frame_ready.disconnect(panel.update_frame)
        except RuntimeError: pass
        try: old_worker.status_message.disconnect()
        except RuntimeError: pass
        try: old_worker.recording_changed.disconnect()
        except RuntimeError: pass
        try: old_worker.connection_changed.disconnect()
        except RuntimeError: pass

        self._showcase_worker = VideoFileWorker(
            config.SHOWCASE_STAGE,
            video_path,
            config.SHOWCASE_FRAME_DELAY_MS
        )
        self._showcase_worker.frame_ready.connect(panel.update_frame)
        self._sc_status_slot = lambda s, m: panel.set_status(m)
        self._sc_conn_slot   = lambda s, c: panel.set_status(
            "Showcase: file loaded" if c else "Showcase: file not found — check SHOWCASE_VIDEO_PATH")
        self._showcase_worker.status_message.connect(self._sc_status_slot)
        self._showcase_worker.connection_changed.connect(self._sc_conn_slot)

        if config.SHOWCASE_STAGE == 1:
            self._rec_panel._vid_s1 = self._showcase_worker
        else:
            self._rec_panel._vid_s2 = self._showcase_worker

        self._showcase_worker.start()

        self._showcase_active = True
        btn = self._rec_panel._showcase_btn
        btn.setText("🎬  Showcase Mode: ON — Click to Disable")
        btn.setObjectName("rec_active")
        btn.style().unpolish(btn); btn.style().polish(btn)
        self._rec_panel._showcase_status.setText(
            f"Showcase: S{config.SHOWCASE_STAGE} → {os.path.basename(config.SHOWCASE_VIDEO_PATH)}")
        self._debug.append(
            f"[SHOWCASE] Started — S{config.SHOWCASE_STAGE} playing {config.SHOWCASE_VIDEO_PATH}")

    def _stop_showcase(self):
        if self._showcase_worker is not None:
            self._showcase_worker.stop()
            self._showcase_worker.wait(2000)

            panel = self._video_s1 if config.SHOWCASE_STAGE == 1 else self._video_s2

            try: self._showcase_worker.frame_ready.disconnect(panel.update_frame)
            except RuntimeError: pass
            try:
                if self._sc_status_slot:
                    self._showcase_worker.status_message.disconnect(self._sc_status_slot)
            except RuntimeError: pass
            try:
                if self._sc_conn_slot:
                    self._showcase_worker.connection_changed.disconnect(self._sc_conn_slot)
            except RuntimeError: pass
            try: self._showcase_worker.recording_changed.disconnect()
            except RuntimeError: pass

            if config.SHOWCASE_STAGE == 1:
                live_worker = self._vid_s1
                self._rec_panel._vid_s1 = live_worker
            else:
                live_worker = self._vid_s2
                self._rec_panel._vid_s2 = live_worker

            live_worker.frame_ready.connect(panel.update_frame)
            live_worker.status_message.connect(lambda s, m: panel.set_status(m))
            live_worker.recording_changed.connect(lambda s, a, p: panel.set_recording(a, p))
            live_worker.start()

            self._showcase_worker = None
            self._sc_status_slot  = None
            self._sc_conn_slot    = None

        self._showcase_active = False
        btn = self._rec_panel._showcase_btn
        btn.setText("🎬  Showcase Mode: OFF")
        btn.setObjectName("confirm")
        btn.style().unpolish(btn); btn.style().polish(btn)
        self._rec_panel._showcase_status.setText("Showcase: OFF")
        self._debug.append("[SHOWCASE] Stopped — live capture restored")

    # ------------------------------------------------------------------

    def _start_workers(self):
        for w in [self._radio_s1, self._radio_s2, self._gps_wkr,
                  self._vid_s1, self._vid_s2]:
            w.start()

    def closeEvent(self, event):
        if self._log_window:
            self._log_window.close()
        if self._showcase_active:
            self._stop_showcase()
        if self._showcase_worker:
            self._showcase_worker.stop()
            self._showcase_worker.wait(2000)
        if self._data_rec.is_recording:
            self._data_rec.stop_recording()
        for w in [self._radio_s1, self._radio_s2, self._gps_wkr,
                  self._vid_s1, self._vid_s2]:
            w.stop(); w.wait(2000)
        event.accept()
