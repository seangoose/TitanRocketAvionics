# =============================================================================
#  TRES Titan Ground Station — video_file_worker.py
#  QThread that reads frames from a pre-rendered video file and emits them
#  as QImages using the same signal interface as VideoWorker.
#  Used in showcase mode to loop a highlight reel on one video panel.
# =============================================================================

import os
import cv2
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui  import QImage


class VideoFileWorker(QThread):
    """
    Reads frames from a video file and emits them using the same signals as
    VideoWorker. Drop-in replacement for one panel in showcase mode.

    Signals match VideoWorker exactly:
      frame_ready(stage, QImage)          — display frame from file
      connection_changed(stage, bool)     — file open/closed state
      status_message(stage, str)          — human-readable status for UI
      recording_changed(stage, bool, str) — never emitted; file playback is not recorded

    recording_changed is never emitted during normal operation.
    start_recording() and stop_recording() are no-ops so RecordingPanel
    does not crash when it calls them on this worker.
    """

    frame_ready        = pyqtSignal(int, QImage)
    connection_changed = pyqtSignal(int, bool)
    status_message     = pyqtSignal(int, str)
    recording_changed  = pyqtSignal(int, bool, str)

    def __init__(self, stage: int, filepath: str,
                 frame_delay_ms: int = 33, parent=None):
        super().__init__(parent)
        self.stage          = stage
        self.filepath       = filepath
        self.frame_delay_ms = frame_delay_ms
        self._running       = False

    # ------------------------------------------------------------------
    #  Recording API stubs — called by RecordingPanel; must not crash
    # ------------------------------------------------------------------

    def start_recording(self, session_name: str = "") -> str:
        """No-op: file playback cannot be recorded. Returns empty path."""
        return ""

    def stop_recording(self) -> None:
        """No-op: nothing to stop."""
        return None

    @property
    def is_recording(self) -> bool:
        return False

    # ------------------------------------------------------------------
    #  Thread lifecycle
    # ------------------------------------------------------------------

    def stop(self):
        self._running = False

    def run(self):
        self._running = True

        while self._running:
            cap = cv2.VideoCapture(self.filepath)

            if not cap.isOpened():
                self.connection_changed.emit(self.stage, False)
                self.status_message.emit(
                    self.stage,
                    "Showcase: file not found — check SHOWCASE_VIDEO_PATH in config.py")
                # Retry every 5 seconds, checking _running every 100 ms
                for _ in range(50):
                    if not self._running:
                        return
                    self.msleep(100)
                continue

            self.connection_changed.emit(self.stage, True)
            self.status_message.emit(
                self.stage,
                f"Showcase: playing {os.path.basename(self.filepath)}")

            while self._running:
                ret, frame = cap.read()

                if not ret:
                    # End of file — loop back to start seamlessly
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                img = QImage(rgb.data, w, h, ch * w,
                             QImage.Format_RGB888).copy()
                self.frame_ready.emit(self.stage, img)

                self.msleep(self.frame_delay_ms)

            cap.release()
