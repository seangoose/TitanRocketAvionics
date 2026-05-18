#!/usr/bin/env python3
# =============================================================================
#  TRES Titan Ground Station — main.py
#  Entry point. Run from the Pi terminal with:
#      python3 main.py
#
#  First-time setup:
#      pip3 install -r requirements.txt
#      sudo apt-get install python3-pyqt5 python3-pyqt5.qtwebengine
#
#  To pre-cache map tiles for offline use at the competition site:
#      python3 cache_tiles.py
#  (Run this at home on WiFi, before driving to the launch site.)
#
#  To identify serial port assignments:
#      ls /dev/ttyUSB* /dev/ttyACM*    (with all devices plugged in)
#      python3 -c "import serial.tools.list_ports; \
#          [print(p) for p in serial.tools.list_ports.comports()]"
#
#  To identify video capture card device indices:
#      v4l2-ctl --list-devices
# =============================================================================

import sys
import os

# Force X11 backend — overrides any Wayland Qt variables set by the Pi 5 OS,
# which would cause QWebEngineView to crash or render blank.
os.environ["QT_QPA_PLATFORM"] = "xcb"

# MapWidget now uses native QPainter tile rendering — no QWebEngineView,
# no Chromium subprocess, no Pi 5 page_allocator mmap crash.

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore    import Qt
from main_window     import MainWindow


def main():
    # Enable Hi-DPI scaling for Pi 5 touchscreen
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    app.setApplicationName("TRES Ground Station")
    app.setOrganizationName("TRES CSUF")

    win = MainWindow()
    win.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
