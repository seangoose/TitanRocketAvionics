# =============================================================================
#  TRES Titan Ground Station — main_window.py
#  Main application window: video feeds, live map, telemetry panels,
#  command panel with full test mode, and recording controls.
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
    QMessageBox, QProgressBar
)

import config
import protocol as P
from data_recorder import DataRecorder, PacketStatsTracker
from map_widget    import MapWidget
from radio_worker  import RadioWorker
from gps_worker    import FWGPSWorker
from video_worker  import VideoWorker


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
    padding: 6px 12px; font-size: 11px;
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
"""


# =============================================================================
#  TELEMETRY PANEL  (one per stage)
# =============================================================================

class TelemetryPanel(QGroupBox):
    def __init__(self, stage: int, parent=None):
        super().__init__(
            f"Stage {stage} — {'Booster' if stage == 1 else 'Sustainer'}", parent)
        self.stage = stage
        self._setup()

    def _setup(self):
        g = QGridLayout(self)
        g.setSpacing(4)

        def mkrow(label, row, attr, big=False):
            lbl = QLabel(label); lbl.setObjectName("telem_label")
            val = QLabel("—");   val.setObjectName("telem_value" if big else "telem_label")
            setattr(self, attr, val)
            g.addWidget(lbl, row, 0)
            g.addWidget(val, row, 1)

        mkrow("ALT",   0, "_alt")
        mkrow("VEL",   1, "_vel")
        mkrow("ACCEL", 2, "_accel")
        mkrow("TEMP",  3, "_temp")
        mkrow("RSSI",  4, "_rssi")

        self._state = QLabel("—"); self._state.setObjectName("state_label")
        g.addWidget(QLabel("STATE"), 5, 0)
        g.addWidget(self._state,     5, 1, 1, 3)

        self._fault = QLabel("Faults: None"); self._fault.setObjectName("fault_ok")
        g.addWidget(self._fault, 6, 0, 1, 4)

        # Ground mode indicator — large and prominent so it's always visible
        self._gmode = QLabel("MODE: —")
        self._gmode.setObjectName("state_label")
        g.addWidget(QLabel("GROUND MODE"), 7, 0)
        g.addWidget(self._gmode, 7, 1, 1, 3)

        for attr, text, row, col in [
            ("_lora",  "LoRa: — MHz",    8, 0),
            ("_vtx",   "VTX: — MHz",     8, 2),
            ("_trate", "Telem: — Hz",    9, 0),
            ("_vpwr",  "VTX cur: —",     9, 2),
            ("_vfpwr", "VTX flt: —",    10, 0),
            ("_video", "Video: —",       10, 2),
            ("_camrec","Cam: idle",      11, 0),
            ("_testm", "Test: off",      11, 2),
        ]:
            w = QLabel(text); w.setObjectName("telem_label")
            setattr(self, attr, w)
            g.addWidget(w, row, col, 1, 2)

        self._conn = QLabel("⚫ Disconnected")
        self._conn.setObjectName("status_err")
        g.addWidget(self._conn, 12, 0, 1, 4)

        self._ts = QLabel("Last: —"); self._ts.setObjectName("telem_label")
        g.addWidget(self._ts, 13, 0, 1, 4)

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
        # Ground mode — prominent, colour-coded by mode
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
    """
    Controls for starting and stopping all recordings simultaneously:
      - Stage 1 video
      - Stage 2 video
      - Flight data CSV (both stages)
    All recordings share the same session name and timestamp prefix so
    files from one session are easy to identify together.
    """

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

        # Session name
        g.addWidget(QLabel("Session name:"), 0, 0)
        self._name_edit = QLineEdit("")
        self._name_edit.setPlaceholderText("e.g. IREC_2026_flight1")
        g.addWidget(self._name_edit, 0, 1, 1, 2)

        # Output directory display + open button
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

        # Record / Stop buttons
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

        # Per-stream status indicators
        self._s1v_lbl  = QLabel("S1 Video: idle"); self._s1v_lbl.setObjectName("rec_idle")
        self._s2v_lbl  = QLabel("S2 Video: idle"); self._s2v_lbl.setObjectName("rec_idle")
        self._data_lbl = QLabel("Data:     idle"); self._data_lbl.setObjectName("rec_idle")
        g.addWidget(self._s1v_lbl,  3, 0, 1, 3)
        g.addWidget(self._s2v_lbl,  4, 0, 1, 3)
        g.addWidget(self._data_lbl, 5, 0, 1, 3)

    def _start_all(self):
        name = self._name_edit.text().strip()

        # Start video recordings
        p1 = self._vid_s1.start_recording(name)
        p2 = self._vid_s2.start_recording(name)

        # Start data recorder
        try:
            dp = self._data.start_recording(name)
            self._data_lbl.setText(f"Data:  ● {os.path.basename(dp)}")
            self._data_lbl.setObjectName("rec_active")
        except RuntimeError:
            pass   # Already recording

        self._rec_btn.setEnabled(False)
        self._rec_btn.setObjectName("")
        self._stop_btn.setEnabled(True)

        # Update status labels (video labels are also updated via signals)
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

    full_sys_test_requested = pyqtSignal(list)  # emits list of stage ints e.g. [1], [2], [1,2]

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
        # RunCam internal SD recording — independent of VTX RF.
        # Use in TEST_IDLE to verify recording works. Stop on PAD_IDLE transition.
        cg = QGroupBox("RunCam Recording  [stage selector]"); cgl = QGridLayout(cg)
        cgl.addWidget(self._mkbtn("⏺  Record ON",
                                  lambda: self._send(P.CMD_CAM_RECORD_ON),  "confirm"), 0, 0)
        cgl.addWidget(self._mkbtn("⏹  Record OFF",
                                  lambda: self._send(P.CMD_CAM_RECORD_OFF), "danger"),  0, 1)
        note = QLabel("Writes to RunCam SD. Independent of VTX RF.")
        note.setObjectName("telem_label"); note.setWordWrap(True)
        cgl.addWidget(note, 1, 0, 1, 2)
        layout.addWidget(cg)

        # ── 4. VIDEO / VTX ──────────────────────────────────────
        vg = QGroupBox("Video / VTX"); vgl = QGridLayout(vg); vgl.setSpacing(4)
        vgl.addWidget(QLabel("RF:"), 0, 0)
        vgl.addWidget(self._mkbtn("Video ON",  lambda: self._send(P.CMD_VIDEO_ON)),  0, 1)
        vgl.addWidget(self._mkbtn("Video OFF", lambda: self._send(P.CMD_VIDEO_OFF)), 0, 2)

        vgl.addWidget(QLabel("S1 VTX MHz:"), 1, 0)
        self._vtx_s1_edit = QLineEdit(str(config.VTX_S1_FREQ_MHZ)); self._vtx_s1_edit.setMaximumWidth(65)
        vgl.addWidget(self._vtx_s1_edit, 1, 1)
        vgl.addWidget(self._mkbtn("Set S1", lambda: self._send_vtx(1), "confirm"), 1, 2)

        vgl.addWidget(QLabel("S2 VTX MHz:"), 2, 0)
        self._vtx_s2_edit = QLineEdit(str(config.VTX_S2_FREQ_MHZ)); self._vtx_s2_edit.setMaximumWidth(65)
        vgl.addWidget(self._vtx_s2_edit, 2, 1)
        vgl.addWidget(self._mkbtn("Set S2", lambda: self._send_vtx(2), "confirm"), 2, 2)

        # Per-stage VTX flight power — applied on LAUNCH_READY entry
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
        tgl.addWidget(self._mkbtn("10 Hz", lambda: self._send(P.CMD_TELEM_HIGH)), 0, 0)
        tgl.addWidget(self._mkbtn(" 1 Hz", lambda: self._send(P.CMD_TELEM_LOW)),  0, 1)
        tgl.addWidget(self._mkbtn("Force Pkt",  lambda: self._send(P.CMD_FORCE_PACKET)), 1, 0)
        tgl.addWidget(self._mkbtn("Get Status", lambda: self._send(P.CMD_GET_STATUS)),   1, 1)

        tgl.addWidget(QLabel("S1 LoRa MHz:"), 2, 0)
        self._lora_s1_edit = QLineEdit(f"{config.LORA_S1_FREQ_MHZ:.3f}"); self._lora_s1_edit.setMaximumWidth(75)
        tgl.addWidget(self._lora_s1_edit, 2, 1)
        tgl.addWidget(self._mkbtn("Set S1", lambda: self._send_lora(1), "confirm"), 2, 2)

        tgl.addWidget(QLabel("S2 LoRa MHz:"), 3, 0)
        self._lora_s2_edit = QLineEdit(f"{config.LORA_S2_FREQ_MHZ:.3f}"); self._lora_s2_edit.setMaximumWidth(75)
        tgl.addWidget(self._lora_s2_edit, 3, 1)
        tgl.addWidget(self._mkbtn("Set S2", lambda: self._send_lora(2), "confirm"), 3, 2)
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

        # Warning label
        warn = QLabel(
            "⚡ Fires VTX + RunCam + 10Hz telem simultaneously for 90s.\n"
            "Stage 2: FIRES SOLENOID (one-shot — destructive test).\n"
            "Ground station auto-starts all recording."
        )
        warn.setObjectName("telem_label")
        warn.setWordWrap(True)
        warn.setStyleSheet("color:#f0a500;")
        fstl.addWidget(warn, 0, 0, 1, 2)

        # Test buttons
        self._fst_s1_btn = self._mkbtn("▶  Run S1 Only", lambda: self._send_full_sys_test([1]), "sys_test")
        self._fst_s2_btn = self._mkbtn("▶  Run S2 Only", lambda: self._send_full_sys_test([2]), "sys_test")
        self._fst_both_btn = self._mkbtn("▶▶  Run Both Stages", lambda: self._send_full_sys_test([1, 2]), "sys_test")

        fstl.addWidget(self._fst_s1_btn, 1, 0)
        fstl.addWidget(self._fst_s2_btn, 1, 1)
        fstl.addWidget(self._fst_both_btn, 2, 0, 1, 2)

        # Countdown label
        self._fst_countdown = QLabel("Test: idle")
        self._fst_countdown.setObjectName("telem_label")
        fstl.addWidget(self._fst_countdown, 3, 0, 1, 2)

        layout.addWidget(fst)

        # ── 8. MAP ──────────────────────────────────────────────
        mg = QGroupBox("Map"); ml = QHBoxLayout(mg)
        self.clear_tracks_btn = QPushButton("Clear Tracks"); ml.addWidget(self.clear_tracks_btn)
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

        # Disable S2-only full system test button if S2 not in TEST_IDLE
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
        edit = self._vtx_s1_edit if stage == 1 else self._vtx_s2_edit
        try: freq = int(edit.text())
        except ValueError: self._log_msg("ERROR", P.CMD_SET_VTX_FREQ, stage, "Invalid MHz"); return
        if not 5600 <= freq <= 5950: self._log_msg("ERROR", P.CMD_SET_VTX_FREQ, stage, f"{freq} outside 5600-5950"); return
        self._radio(stage).send_frame(P.build_set_vtx_freq_frame(stage, freq))
        self._log_msg("SENT", P.CMD_SET_VTX_FREQ, stage, f"{freq} MHz")
        if self._data.is_recording: self._data.record_event(f"SET_VTX_FREQ {freq} → S{stage}")

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
        """
        Send CMD_FULL_SYS_TEST to specified stages.

        Checks:
        - All stages must be in GM_TEST_IDLE
        - If S2 included, show confirmation dialog (solenoid will fire)
        """
        # Check all stages are in TEST_IDLE
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

        # S2 confirmation (solenoid will fire)
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

        # Send command to each stage
        for s in stages:
            radio = self._r1 if s == 1 else self._r2
            radio.send_frame(P.build_full_sys_test_frame(s))
            self._log_msg("SENT", P.CMD_FULL_SYS_TEST, s, "Full system test started")
            if self._data.is_recording:
                self._data.record_event(f"CMD FULL_SYS_TEST → S{s}")

        # Emit signal to MainWindow
        self.full_sys_test_requested.emit(stages)

    def set_test_countdown(self, text: str):
        """Update the full system test countdown label."""
        self._fst_countdown.setText(text)

    # ------------------------------------------------------------------
    @pyqtSlot(int, dict)
    def on_ack(self, stage: int, ack: dict):
        result = "✅ OK" if ack["result"] == P.ACK_OK else "❌ REJECTED"
        self._log_msg(result, ack["cmd"], stage, f"← ACK from S{stage}")

    @pyqtSlot(str)
    def on_raw_log(self, msg: str):
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
        return False   # Legacy shim


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
    """
    Displays live packet reception statistics during full system test.
    Two columns (S1 | S2) showing received packets, loss %, RSSI, temperature, etc.
    Progress bar at bottom shows test countdown.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setSpacing(4)

        # Two column layout for S1 and S2 stats
        grid = QGridLayout()
        grid.setSpacing(6)

        # Column headers
        s1_hdr = QLabel("Stage 1"); s1_hdr.setObjectName("state_label")
        s2_hdr = QLabel("Stage 2"); s2_hdr.setObjectName("state_label")
        grid.addWidget(s1_hdr, 0, 0)
        grid.addWidget(s2_hdr, 0, 1)

        # S1 column widgets
        self._s1_status = QLabel("Idle"); self._s1_status.setObjectName("telem_label")
        self._s1_packets = QLabel("—"); self._s1_packets.setObjectName("telem_label")
        self._s1_loss = QLabel("—"); self._s1_loss.setObjectName("telem_label")
        self._s1_gaps = QLabel("—"); self._s1_gaps.setObjectName("telem_label")
        self._s1_avg_rssi = QLabel("—"); self._s1_avg_rssi.setObjectName("telem_label")
        self._s1_min_rssi = QLabel("—"); self._s1_min_rssi.setObjectName("telem_label")
        self._s1_temp = QLabel("—"); self._s1_temp.setObjectName("telem_label")
        self._s1_elapsed = QLabel("—"); self._s1_elapsed.setObjectName("telem_label")

        # S2 column widgets
        self._s2_status = QLabel("Idle"); self._s2_status.setObjectName("telem_label")
        self._s2_packets = QLabel("—"); self._s2_packets.setObjectName("telem_label")
        self._s2_loss = QLabel("—"); self._s2_loss.setObjectName("telem_label")
        self._s2_gaps = QLabel("—"); self._s2_gaps.setObjectName("telem_label")
        self._s2_avg_rssi = QLabel("—"); self._s2_avg_rssi.setObjectName("telem_label")
        self._s2_min_rssi = QLabel("—"); self._s2_min_rssi.setObjectName("telem_label")
        self._s2_temp = QLabel("—"); self._s2_temp.setObjectName("telem_label")
        self._s2_elapsed = QLabel("—"); self._s2_elapsed.setObjectName("telem_label")

        # Add rows to grid
        row = 1
        grid.addWidget(self._s1_status, row, 0); grid.addWidget(self._s2_status, row, 1); row += 1
        grid.addWidget(QLabel("Packets:"), row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_packets, row, 0); grid.addWidget(self._s2_packets, row, 1); row += 1
        grid.addWidget(QLabel("Loss:"), row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_loss, row, 0); grid.addWidget(self._s2_loss, row, 1); row += 1
        grid.addWidget(QLabel("Gaps:"), row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_gaps, row, 0); grid.addWidget(self._s2_gaps, row, 1); row += 1
        grid.addWidget(QLabel("Avg RSSI:"), row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_avg_rssi, row, 0); grid.addWidget(self._s2_avg_rssi, row, 1); row += 1
        grid.addWidget(QLabel("Min RSSI:"), row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_min_rssi, row, 0); grid.addWidget(self._s2_min_rssi, row, 1); row += 1
        grid.addWidget(QLabel("Temp Range:"), row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_temp, row, 0); grid.addWidget(self._s2_temp, row, 1); row += 1
        grid.addWidget(QLabel("Elapsed:"), row, 0, 1, 2); row += 1
        grid.addWidget(self._s1_elapsed, row, 0); grid.addWidget(self._s2_elapsed, row, 1); row += 1

        layout.addLayout(grid)

        # Progress bar at bottom
        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setFormat("No test running")
        self._progress.setTextVisible(True)
        layout.addWidget(self._progress)

    def update_stats(self, stage: int, summary: dict):
        """
        Update statistics for one stage.

        Args:
            stage: 1 or 2
            summary: dict from PacketStatsTracker.summary_dict()
                     Keys: received, expected, loss_pct, gaps, avg_rssi,
                           min_rssi, temp_min_c, temp_max_c, elapsed_s
        """
        widgets = {
            1: {
                "packets": self._s1_packets,
                "loss": self._s1_loss,
                "gaps": self._s1_gaps,
                "avg_rssi": self._s1_avg_rssi,
                "min_rssi": self._s1_min_rssi,
                "temp": self._s1_temp,
                "elapsed": self._s1_elapsed,
            },
            2: {
                "packets": self._s2_packets,
                "loss": self._s2_loss,
                "gaps": self._s2_gaps,
                "avg_rssi": self._s2_avg_rssi,
                "min_rssi": self._s2_min_rssi,
                "temp": self._s2_temp,
                "elapsed": self._s2_elapsed,
            },
        }

        w = widgets.get(stage)
        if not w:
            return

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
        """
        Set the status label for a stage.

        Args:
            stage: 1 or 2
            status_str: "Idle" / "Running…" / "Complete ✓"
        """
        label = self._s1_status if stage == 1 else self._s2_status
        label.setText(status_str)

        # Color coding
        if "Running" in status_str:
            label.setStyleSheet("color:#f0a500;font-weight:bold;")
        elif "Complete" in status_str:
            label.setStyleSheet("color:#3fb950;font-weight:bold;")
        else:  # Idle
            label.setStyleSheet("color:#8b949e;")

    def reset(self):
        """Reset both columns to default Idle / — state."""
        for stage in [1, 2]:
            self.set_status(stage, "Idle")
            widgets = {
                1: [self._s1_packets, self._s1_loss, self._s1_gaps,
                    self._s1_avg_rssi, self._s1_min_rssi, self._s1_temp, self._s1_elapsed],
                2: [self._s2_packets, self._s2_loss, self._s2_gaps,
                    self._s2_avg_rssi, self._s2_min_rssi, self._s2_temp, self._s2_elapsed],
            }
            for w in widgets[stage]:
                w.setText("—")
                w.setStyleSheet("")

        self._progress.setValue(0)
        self._progress.setFormat("No test running")

    def set_progress(self, pct: int, label_str: str):
        """
        Update progress bar.

        Args:
            pct: 0–100
            label_str: Text to display on progress bar
        """
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
        self.resize(1650, 950)
        if config.DARK_MODE:
            self.setStyleSheet(DARK_STYLE)

        # Data recorder (shared by command panel and recording panel)
        self._data_rec = DataRecorder()

        # Ground mode tracking — updated by _on_status_data from STATUS packets.
        # Drives GPS gating (map only updates in LAUNCH_READY) and CommandPanel
        # solenoid gate / mode indicator labels.
        self._s1_ground_mode = P.GM_TEST_IDLE
        self._s2_ground_mode = P.GM_TEST_IDLE

        # Full system test tracking
        self._stats_s1 = PacketStatsTracker(stage=1, expected_rate_hz=10.0)
        self._stats_s2 = PacketStatsTracker(stage=2, expected_rate_hz=10.0)
        self._test_active_stages = []
        self._test_timer = QTimer(self)
        self._test_timer.setInterval(1000)  # fires every second
        self._test_elapsed_s = 0
        self._test_timer_connected = False

        # Workers
        self._radio_s1 = RadioWorker(1, config.M0_S1_PORT, config.M0_S1_BAUD)
        self._radio_s2 = RadioWorker(2, config.M0_S2_PORT, config.M0_S2_BAUD)
        self._gps_wkr  = FWGPSWorker(config.FW_GPS_PORT, config.FW_GPS_BAUD)
        self._vid_s1   = VideoWorker(1, config.VIDEO_S1_INDEX,
                                     config.VIDEO_WIDTH, config.VIDEO_HEIGHT,
                                     config.VIDEO_FPS)
        self._vid_s2   = VideoWorker(2, config.VIDEO_S2_INDEX,
                                     config.VIDEO_WIDTH, config.VIDEO_HEIGHT,
                                     config.VIDEO_FPS)

        self._build_ui()
        self._wire_signals()
        self._start_workers()

    # ------------------------------------------------------------------

    def _build_ui(self):
        central = QWidget(); self.setCentralWidget(central)
        root = QHBoxLayout(central); root.setSpacing(3); root.setContentsMargins(3,3,3,3)

        # Left: S1 telem + video
        left = QVBoxLayout()
        self._telem_s1 = TelemetryPanel(1)
        self._video_s1 = VideoPanel(1)
        left.addWidget(self._telem_s1, 2); left.addWidget(self._video_s1, 3)

        # Centre: map
        self._map = MapWidget(); self._map.setMinimumWidth(580)

        # Right: S2 telem + video
        right = QVBoxLayout()
        self._telem_s2 = TelemetryPanel(2)
        self._video_s2 = VideoPanel(2)
        right.addWidget(self._telem_s2, 2); right.addWidget(self._video_s2, 3)

        # Far right: command + recording (scrollable narrow strip)
        side = QVBoxLayout(); side.setSpacing(4)
        self._cmd_panel = CommandPanel(self._radio_s1, self._radio_s2,
                                       self._data_rec)
        self._cmd_panel.clear_tracks_btn.clicked.connect(self._map.clear_tracks)
        self._rec_panel = RecordingPanel(self._vid_s1, self._vid_s2,
                                         self._data_rec)
        side.addWidget(self._cmd_panel, 3)
        side.addWidget(self._rec_panel, 1)

        side_widget = QWidget(); side_widget.setLayout(side)
        side_widget.setFixedWidth(310)

        # Bottom tabs
        self._tabs = QTabWidget(); self._tabs.setMaximumHeight(220)
        self._gps_panel = GPSPanel()
        self._tabs.addTab(self._gps_panel, "Featherweight GPS")
        self._debug = DebugConsole()
        self._tabs.addTab(self._debug, "Raw Serial Log")
        self._test_stats_panel = TestStatsPanel()
        self._tabs.addTab(self._test_stats_panel, "System Test Stats")

        # Assemble body
        body = QHBoxLayout()
        body.addLayout(left,   2)
        body.addWidget(self._map, 5)
        body.addLayout(right,  2)
        body.addWidget(side_widget)

        outer = QVBoxLayout(); outer.addLayout(body, 1); outer.addWidget(self._tabs)
        root.addLayout(outer)

    # ------------------------------------------------------------------

    def _wire_signals(self):
        # Flight data → telem panels + map + data recorder
        self._radio_s1.flight_data.connect(lambda s, d: self._telem_s1.update_flight(d))
        self._radio_s2.flight_data.connect(lambda s, d: self._telem_s2.update_flight(d))
        self._radio_s1.flight_data.connect(self._on_flight_data)
        self._radio_s2.flight_data.connect(self._on_flight_data)

        # Status packets → telem panels
        self._radio_s1.status_data.connect(lambda s, d: self._telem_s1.update_status(d))
        self._radio_s2.status_data.connect(lambda s, d: self._telem_s2.update_status(d))
        self._radio_s1.status_data.connect(self._on_status_data)
        self._radio_s2.status_data.connect(self._on_status_data)

        # ACKs → command panel
        self._radio_s1.ack_received.connect(self._cmd_panel.on_ack)
        self._radio_s2.ack_received.connect(self._cmd_panel.on_ack)

        # Full system test → main window
        self._cmd_panel.full_sys_test_requested.connect(self._on_full_sys_test_requested)

        # Connection status → telem panels
        self._radio_s1.connection_changed.connect(lambda s, c: self._telem_s1.set_connected(c))
        self._radio_s2.connection_changed.connect(lambda s, c: self._telem_s2.set_connected(c))

        # Raw log → debug + command panel
        for r in [self._radio_s1, self._radio_s2]:
            r.raw_log.connect(lambda s, m: self._debug.append(m))
            r.raw_log.connect(self._cmd_panel.on_raw_log)

        # GPS worker
        self._gps_wkr.position_update.connect(self._on_gps)
        self._gps_wkr.lost_rocket.connect(self._on_lost_rocket)
        self._gps_wkr.connection_changed.connect(self._gps_panel.set_connected)
        self._gps_wkr.raw_log.connect(self._debug.append)

        # Video workers → panels + recording indicators
        self._vid_s1.frame_ready.connect(self._video_s1.update_frame)
        self._vid_s2.frame_ready.connect(self._video_s2.update_frame)
        self._vid_s1.status_message.connect(lambda s, m: self._video_s1.set_status(m))
        self._vid_s2.status_message.connect(lambda s, m: self._video_s2.set_status(m))
        self._vid_s1.recording_changed.connect(
            lambda s, a, p: self._video_s1.set_recording(a, p))
        self._vid_s2.recording_changed.connect(
            lambda s, a, p: self._video_s2.set_recording(a, p))

    # ------------------------------------------------------------------

    @pyqtSlot(int, dict)
    @pyqtSlot(int, dict)
    def _on_flight_data(self, stage: int, d: dict):
        self._map.update_telemetry(
            stage, d["altitude_ft"], d["velocity_fps"],
            P.state_name(d.get("stage", stage), d["state"]))
        if self._data_rec.is_recording:
            self._data_rec.record_flight(stage, d)

        # Record packet for system test tracking
        if stage in self._test_active_stages:
            tracker = self._stats_s1 if stage == 1 else self._stats_s2
            tracker.record_packet(d)

    @pyqtSlot(int, dict)
    def _on_status_data(self, stage: int, d: dict):
        # Track per-stage ground modes — drives GPS gating and CommandPanel
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
        # GPS GATING: only update map and panel in LAUNCH_READY.
        # FWGPSWorker always runs but map/panel stay quiet in
        # TEST_IDLE and PAD_IDLE — no value in showing position
        # data until we have a genuine launch commitment.
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
        """
        Called when user initiates a full system test.

        Args:
            stages: List of stage numbers [1], [2], or [1, 2]
        """
        self._test_active_stages = stages

        # Start packet tracking for each stage
        for stage in stages:
            tracker = self._stats_s1 if stage == 1 else self._stats_s2
            tracker.start()
            self._test_stats_panel.set_status(stage, "Running…")

        # Auto-start recording if not already recording
        if not self._data_rec.is_recording:
            session_name = f"sysTest_{''.join('S'+str(s) for s in stages)}"
            self._rec_panel._name_edit.setText(session_name)
            self._rec_panel._start_all()

        # Reset and switch to test stats tab
        self._test_stats_panel.reset()
        # Switch to System Test Stats tab (index 2)
        self._tabs.setCurrentIndex(2)

        # Reset elapsed time
        self._test_elapsed_s = 0

        # Connect timer (only once)
        if not self._test_timer_connected:
            self._test_timer.timeout.connect(self._on_test_tick)
            self._test_timer_connected = True

        # Start timer
        self._test_timer.start()

        self._debug.append(f"[FULL SYS TEST] Started for stages: {stages}")

    def _on_test_tick(self):
        """Called every second during full system test."""
        self._test_elapsed_s += 1
        duration = config.FULL_SYS_TEST_DURATION_S

        # Update progress bar
        pct = min(100, int(self._test_elapsed_s / duration * 100))
        remaining = max(0, duration - self._test_elapsed_s)
        self._test_stats_panel.set_progress(pct, f"Running — {remaining}s remaining")

        # Update countdown label in CommandPanel
        self._cmd_panel.set_test_countdown(f"Test: {self._test_elapsed_s}s / {duration}s")

        # Update stats for each active stage
        for stage in self._test_active_stages:
            tracker = self._stats_s1 if stage == 1 else self._stats_s2
            summary = tracker.summary_dict()
            self._test_stats_panel.update_stats(stage, summary)

        # Check if test is complete
        if self._test_elapsed_s >= duration:
            self._on_test_complete()

    def _on_test_complete(self):
        """Called when full system test completes."""
        # Stop timer
        self._test_timer.stop()

        # Stop packet tracking for each stage
        for stage in self._test_active_stages:
            tracker = self._stats_s1 if stage == 1 else self._stats_s2
            tracker.stop()
            self._test_stats_panel.set_status(stage, "Complete ✓")

        # Set progress to 100%
        self._test_stats_panel.set_progress(100, "Test complete")

        # Update countdown label
        self._cmd_panel.set_test_countdown("Test: complete")

        # Stop recording
        self._rec_panel._stop_all()

        # Generate reports
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

        # Show completion dialog
        QMessageBox.information(
            self, "Full System Test Complete",
            "Full System Test Complete\n\n"
            "Test reports saved to recordings/\n"
            "Check the System Test Stats tab for results."
        )

        # Clear active stages
        self._test_active_stages = []

    # ------------------------------------------------------------------

    def _start_workers(self):
        for w in [self._radio_s1, self._radio_s2, self._gps_wkr,
                  self._vid_s1, self._vid_s2]:
            w.start()

    def closeEvent(self, event):
        # Stop any active recording cleanly before exit
        if self._data_rec.is_recording:
            self._data_rec.stop_recording()
        for w in [self._radio_s1, self._radio_s2, self._gps_wkr,
                  self._vid_s1, self._vid_s2]:
            w.stop(); w.wait(2000)
        event.accept()
