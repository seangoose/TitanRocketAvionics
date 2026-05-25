# =============================================================================
#  TRES Titan Ground Station — video_worker.py
#  QThread that captures video from one USB capture card.
#  Supports start/stop recording to disk via thread-safe methods.
# =============================================================================

import os
import cv2
import threading
from datetime import datetime
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui  import QImage

# Output directory for video recordings
RECORDINGS_DIR = "recordings"

# Video codec — use mp4v for .mp4 files.
# If mp4v isn't available on the Pi, fall back to XVID (.avi).
VIDEO_CODEC   = "mp4v"
VIDEO_EXT     = ".mp4"


class VideoWorker(QThread):
    """
    Captures video from one V4L2 device (USB capture card).

    Signals:
      frame_ready(stage, QImage)       — new display frame available
      connection_changed(stage, bool)  — device open/closed state
      status_message(stage, str)       — human-readable status for UI
      recording_changed(stage, bool, str)
        — emitted when recording starts (True, filepath) or stops (False, "")

    Thread-safe public methods (call from main thread):
      start_recording(session_name)    — begin writing video to disk
      stop_recording()                 — finalise and close video file
    """

    frame_ready        = pyqtSignal(int, QImage)
    connection_changed = pyqtSignal(int, bool)
    status_message     = pyqtSignal(int, str)
    recording_changed  = pyqtSignal(int, bool, str)

    def __init__(self, stage: int, device_index: int,
                 width: int = 640, height: int = 480,
                 fps: int = 30, parent=None):
        super().__init__(parent)
        self.stage        = stage
        self.device_index = device_index
        self.width        = width
        self.height       = height
        self.fps          = fps
        self._running     = False

        # Recording state (protected by _rec_lock)
        self._rec_lock    = threading.Lock()
        self._writer      = None     # cv2.VideoWriter or None
        self._rec_path    = ""
        self._is_recording = False

    # ------------------------------------------------------------------
    #  Recording API — safe to call from main thread at any time
    # ------------------------------------------------------------------

    def start_recording(self, session_name: str = "") -> str:
        """
        Start recording to a new video file.
        Returns the path of the file being written.
        No-op if already recording.
        """
        with self._rec_lock:
            if self._is_recording:
                return self._rec_path

            os.makedirs(RECORDINGS_DIR, exist_ok=True)
            ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
            tag = f"_{session_name}" if session_name.strip() else ""
            fname = f"TRES_{ts}{tag}_S{self.stage}{VIDEO_EXT}"
            path  = os.path.join(RECORDINGS_DIR, fname)

            fourcc = cv2.VideoWriter_fourcc(*VIDEO_CODEC)
            writer = cv2.VideoWriter(path, fourcc, self.fps,
                                     (self.width, self.height))
            if not writer.isOpened():
                # Try XVID fallback
                path   = path.replace(VIDEO_EXT, ".avi")
                fourcc = cv2.VideoWriter_fourcc(*"XVID")
                writer = cv2.VideoWriter(path, fourcc, self.fps,
                                         (self.width, self.height))
            if not writer.isOpened():
                self.status_message.emit(self.stage,
                    f"S{self.stage} Video: could not open VideoWriter — "
                    f"check codec support on this Pi")
                return ""

            self._writer       = writer
            self._rec_path     = path
            self._is_recording = True

        self.recording_changed.emit(self.stage, True, path)
        self.status_message.emit(self.stage,
            f"S{self.stage} Video: recording → {os.path.basename(path)}")
        return path

    def stop_recording(self):
        """Stop and finalise the current video recording."""
        with self._rec_lock:
            if not self._is_recording:
                return
            if self._writer:
                self._writer.release()
                self._writer = None
            path = self._rec_path
            self._rec_path     = ""
            self._is_recording = False

        self.recording_changed.emit(self.stage, False, "")
        self.status_message.emit(self.stage,
            f"S{self.stage} Video: recording saved → {os.path.basename(path)}")

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    # ------------------------------------------------------------------
    #  Thread lifecycle
    # ------------------------------------------------------------------

    def stop(self):
        self._running = False
        self.stop_recording()

    def run(self):
        self._running = True

        while self._running:
            cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)

            if not cap.isOpened():
                self.connection_changed.emit(self.stage, False)
                self.status_message.emit(self.stage,
                    f"S{self.stage} Camera: /dev/video{self.device_index} "
                    f"not found. Retrying in 3 s…")
                self.msleep(3000)
                continue

            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS,          self.fps)

            actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = cap.get(cv2.CAP_PROP_FPS)
            # Many V4L2 capture cards report 0 fps before the first frame is read.
            # Guard here — a VideoWriter opened with fps=0 silently discards every write.
            if actual_fps <= 0:
                actual_fps = float(self.fps)

            # Re-open the writer only if the driver gave us different dimensions than
            # what we opened with.  Previously this ran unconditionally, which caused
            # the writer to be torn down and rebuilt with fps=0 every camera connect,
            # resulting in a 258-byte empty MP4 (the competition recording failure).
            with self._rec_lock:
                if self._is_recording and self._writer:
                    if actual_w != self.width or actual_h != self.height:
                        self._writer.release()
                        fourcc = cv2.VideoWriter_fourcc(*VIDEO_CODEC)
                        new_w = cv2.VideoWriter(
                            self._rec_path, fourcc, actual_fps,
                            (actual_w, actual_h))
                        if new_w.isOpened():
                            self._writer = new_w
                        else:
                            # Failed to reopen — keep the old writer rather than losing frames
                            new_w.release()

            self.connection_changed.emit(self.stage, True)
            self.status_message.emit(self.stage,
                f"S{self.stage} Camera: {actual_w}×{actual_h} "
                f"@ {actual_fps:.0f} fps  (/dev/video{self.device_index})"
                f" — waiting for signal…")

            _no_signal_frames = 0
            while self._running:
                ret, frame = cap.read()
                if not ret:
                    self.connection_changed.emit(self.stage, False)
                    self.status_message.emit(self.stage,
                        f"S{self.stage} Camera: capture failed — reopening…")
                    break

                # Detect black/no-signal frames (mean brightness < 3/255)
                import numpy as np
                is_black = (np.mean(frame) < 3.0)
                if is_black:
                    _no_signal_frames += 1
                    if _no_signal_frames == 30:   # ~0.5s of black frames
                        self.status_message.emit(self.stage,
                            f"S{self.stage} Camera: connected — no signal "
                            f"(/dev/video{self.device_index})")
                else:
                    if _no_signal_frames >= 30:
                        self.status_message.emit(self.stage,
                            f"S{self.stage} Camera: {actual_w}×{actual_h} "
                            f"@ {actual_fps:.0f} fps — live")
                    _no_signal_frames = 0

                # ── Write to disk if recording ────────────────────────
                with self._rec_lock:
                    if self._is_recording and self._writer:
                        self._writer.write(frame)

                # ── Emit QImage for display ───────────────────────────
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                img = QImage(rgb.data, w, h, ch * w,
                             QImage.Format_RGB888).copy()
                self.frame_ready.emit(self.stage, img)

                self.msleep(15)   # ~60 Hz max, display widget throttles further

            cap.release()
