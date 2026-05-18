# =============================================================================
#  TRES Titan Ground Station — map_widget.py
#  Native Qt tile map — no Chromium/WebEngine required.
#  Renders OpenStreetMap tiles via QPainter; loads from the offline tile cache
#  first (tile_cache/{z}/{x}/{y}.png), falls back to network (OSM tile server)
#  when a cache miss occurs and internet is available.
#
#  Public API is identical to the previous QWebEngineView version so
#  main_window.py requires no changes.
# =============================================================================

import math
import os
import queue
import urllib.request

from PyQt5.QtCore    import Qt, QThread, pyqtSignal, QRectF
from PyQt5.QtGui     import (QPainter, QColor, QPixmap, QImage, QPen,
                              QBrush, QFont, QFontMetrics)
from PyQt5.QtWidgets import QWidget

import config

TILE_SIZE  = 256
OSM_URL    = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT = "TRESGroundStation/1.0"

S1_COLOR   = QColor("#FF6600")   # Stage 1 — orange
S2_COLOR   = QColor("#00AAFF")   # Stage 2 — blue
PAD_COLOR  = QColor("#00CC44")   # Launch pad — green
LOST_COLOR = QColor("#FF2222")   # Lost-rocket alert — red


# ─────────────────────────────────────────────────────────────────────────────
#  Coordinate math
# ─────────────────────────────────────────────────────────────────────────────

def _deg2frac(lat: float, lon: float, zoom: int):
    """Return fractional OSM tile coordinates for (lat, lon) at zoom."""
    lat_r = math.radians(lat)
    n = 1 << zoom
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n
    return x, y


def _frac2latlon(tx: float, ty: float, zoom: int):
    """Inverse: fractional tile coords back to (lat, lon)."""
    n = 1 << zoom
    lon = tx / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * ty / n))))
    return lat, lon


# ─────────────────────────────────────────────────────────────────────────────
#  Background tile loader
# ─────────────────────────────────────────────────────────────────────────────

class _TileLoader(QThread):
    """
    Background thread that loads tile PNGs and emits them as QImage.
    QImage is thread-safe; QPixmap conversion happens on the GUI thread.
    """

    tile_ready = pyqtSignal(int, int, int, QImage)   # z, x, y, image

    def __init__(self, cache_dir: str, parent=None):
        super().__init__(parent)
        self._cache_dir = cache_dir
        self._q         = queue.Queue()
        self._seen      = set()
        self._running   = True

    def request(self, z: int, x: int, y: int):
        key = (z, x, y)
        if key not in self._seen:
            self._seen.add(key)
            self._q.put(key)

    def stop(self):
        self._running = False
        self._q.put(None)   # unblock the blocking get()

    def run(self):
        while self._running:
            item = self._q.get()
            if item is None:
                break
            self._load(*item)

    def _load(self, z: int, x: int, y: int):
        img = QImage()

        # ── Offline cache ─────────────────────────────────────────────────────
        cache_path = os.path.join(self._cache_dir, str(z), str(x), f"{y}.png")
        if os.path.exists(cache_path):
            if img.load(cache_path) and not img.isNull():
                self.tile_ready.emit(z, x, y, img)
                return

        # ── Network fallback ──────────────────────────────────────────────────
        try:
            url = OSM_URL.format(z=z, x=x, y=y)
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=5) as resp:
                data = resp.read()
            if img.loadFromData(data) and not img.isNull():
                os.makedirs(os.path.join(self._cache_dir, str(z), str(x)),
                            exist_ok=True)
                img.save(cache_path)
                self.tile_ready.emit(z, x, y, img)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
#  Map widget
# ─────────────────────────────────────────────────────────────────────────────

class MapWidget(QWidget):
    """
    Native PyQt5 tile map — renders OSM tiles with QPainter.
    Supports pan (mouse drag) and zoom (mouse wheel / touchpad scroll).
    Auto-pans to keep the tracked rocket visible.

    Public API (identical to the previous QWebEngineView version):
        update_gps(stage, lat, lon, alt_ft, vert_vel, name)
        update_telemetry(stage, alt_ft, vel_fps, state_name)
        show_lost_rocket(name, lat, lon, alt_ft)
        set_pad_location(lat, lon)
        set_site(lat, lon, zoom)
        clear_tracks()
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 150)
        self.setMouseTracking(True)
        self.setAttribute(Qt.WA_OpaquePaintEvent, True)

        # View state
        self._zoom       = config.MAP_CENTER_ZOOM
        self._center_lat = config.MAP_CENTER_LAT
        self._center_lon = config.MAP_CENTER_LON

        # Tile cache  (z, x, y) → QPixmap
        self._tiles = {}

        # Data
        self._gps       = {}    # stage → dict(lat, lon, alt_ft, vert_vel, name)
        self._tracks    = {}    # stage → [(lat, lon), …]
        self._telemetry = {}    # stage → dict(alt_ft, vel_fps, state_name)
        self._pad       = (config.MAP_CENTER_LAT, config.MAP_CENTER_LON)
        self._lost      = []    # [(name, lat, lon, alt_ft)]
        self._has_gps   = False

        # Pan drag state
        self._drag_start = None   # QPoint
        self._drag_clat  = None
        self._drag_clon  = None

        # Resolve tile cache dir to an absolute path
        cache_dir = config.TILE_CACHE_DIR
        script_dir = os.path.dirname(os.path.abspath(__file__))
        if not os.path.isabs(cache_dir):
            cache_dir = os.path.normpath(os.path.join(script_dir, cache_dir))
        self._cache_dir = cache_dir

        self._loader = _TileLoader(self._cache_dir, self)
        self._loader.tile_ready.connect(self._on_tile_ready)
        self._loader.start()

        self._request_visible_tiles()

    def closeEvent(self, event):
        self._loader.stop()
        self._loader.wait(2000)
        super().closeEvent(event)

    # ── Tile management ───────────────────────────────────────────────────────

    def _request_visible_tiles(self):
        w = self.width()  or 400
        h = self.height() or 300
        cx, cy = _deg2frac(self._center_lat, self._center_lon, self._zoom)
        n = 1 << self._zoom

        pad = 2
        tx0 = int(cx) - w // (2 * TILE_SIZE) - pad
        ty0 = int(cy) - h // (2 * TILE_SIZE) - pad
        tx1 = int(cx) + w // (2 * TILE_SIZE) + pad + 1
        ty1 = int(cy) + h // (2 * TILE_SIZE) + pad + 1

        for ty in range(ty0, ty1):
            if ty < 0 or ty >= n:
                continue
            for tx_raw in range(tx0, tx1):
                tx = tx_raw % n
                if (self._zoom, tx, ty) not in self._tiles:
                    self._loader.request(self._zoom, tx, ty)

    def _on_tile_ready(self, z: int, x: int, y: int, img: QImage):
        if z == self._zoom:
            self._tiles[(z, x, y)] = QPixmap.fromImage(img)
            self.update()

    # ── Painting ──────────────────────────────────────────────────────────────

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        painter.fillRect(0, 0, w, h, QColor("#1e2a3a"))

        cx, cy = _deg2frac(self._center_lat, self._center_lon, self._zoom)
        n = 1 << self._zoom

        # ── Tiles ─────────────────────────────────────────────────────────────
        tx0 = int(cx) - w // (2 * TILE_SIZE) - 2
        ty0 = int(cy) - h // (2 * TILE_SIZE) - 2
        tx1 = int(cx) + w // (2 * TILE_SIZE) + 3
        ty1 = int(cy) + h // (2 * TILE_SIZE) + 3

        for ty in range(ty0, ty1):
            for tx_raw in range(tx0, tx1):
                if ty < 0 or ty >= n:
                    continue
                tx = tx_raw % n
                px = int((tx_raw - cx) * TILE_SIZE + w / 2)
                py = int((ty      - cy) * TILE_SIZE + h / 2)
                pm = self._tiles.get((self._zoom, tx, ty))
                if pm:
                    painter.drawPixmap(px, py, pm)
                else:
                    # Loading placeholder
                    painter.fillRect(px, py, TILE_SIZE, TILE_SIZE,
                                     QColor("#232f3e"))
                    painter.setPen(QPen(QColor("#2d3f50"), 1))
                    painter.drawRect(px, py, TILE_SIZE - 1, TILE_SIZE - 1)

        # ── Flight tracks ─────────────────────────────────────────────────────
        for stage, pts in self._tracks.items():
            if len(pts) < 2:
                continue
            color = S1_COLOR if stage == 1 else S2_COLOR
            painter.setPen(QPen(color, 2, Qt.SolidLine))
            prev = None
            for lat, lon in pts:
                fx, fy = _deg2frac(lat, lon, self._zoom)
                px = int((fx - cx) * TILE_SIZE + w / 2)
                py = int((fy - cy) * TILE_SIZE + h / 2)
                if prev:
                    painter.drawLine(prev[0], prev[1], px, py)
                prev = (px, py)

        # ── Markers ───────────────────────────────────────────────────────────
        if self._pad:
            self._draw_marker(painter, cx, cy, w, h,
                              self._pad[0], self._pad[1], PAD_COLOR, "PAD", 7)

        for stage, d in self._gps.items():
            color = S1_COLOR if stage == 1 else S2_COLOR
            label = f"S{stage}: {d['alt_ft']:.0f} ft"
            self._draw_marker(painter, cx, cy, w, h,
                              d['lat'], d['lon'], color, label, 9)

        for name, lat, lon, _ in self._lost:
            self._draw_marker(painter, cx, cy, w, h,
                              lat, lon, LOST_COLOR, f"LOST: {name}", 9)

        # ── "Waiting for GPS" overlay ──────────────────────────────────────────
        if not self._has_gps:
            painter.fillRect(0, 0, w, h, QColor(0, 0, 0, 150))
            font = QFont()
            font.setPointSize(11)
            font.setBold(True)
            painter.setFont(font)
            painter.setPen(Qt.white)
            painter.drawText(QRectF(0, 0, w, h), Qt.AlignCenter,
                             "Waiting for Featherweight GPS…\n"
                             "No tracker position received yet.")

        painter.end()

    def _draw_marker(self, painter, cx, cy, w, h,
                     lat, lon, color, label, r):
        fx, fy = _deg2frac(lat, lon, self._zoom)
        px = int((fx - cx) * TILE_SIZE + w / 2)
        py = int((fy - cy) * TILE_SIZE + h / 2)

        # Circle
        painter.setPen(QPen(Qt.white, 2))
        painter.setBrush(QBrush(color))
        painter.drawEllipse(px - r, py - r, r * 2, r * 2)

        # Label with dark background
        font = QFont()
        font.setPointSize(8)
        font.setBold(True)
        painter.setFont(font)
        fm = QFontMetrics(font)
        tw = fm.boundingRect(label).width()
        bg = QRectF(px - tw / 2 - 2, py - r - 16, tw + 4, 13)
        painter.fillRect(bg, QColor(0, 0, 0, 190))
        painter.setPen(Qt.white)
        painter.drawText(bg, Qt.AlignCenter, label)

    # ── Interaction ───────────────────────────────────────────────────────────

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0 and self._zoom < 19:
            self._zoom += 1
        elif delta < 0 and self._zoom > 2:
            self._zoom -= 1
        self._tiles.clear()
        self._request_visible_tiles()
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._drag_start = event.pos()
            self._drag_clat  = self._center_lat
            self._drag_clon  = self._center_lon

    def mouseMoveEvent(self, event):
        if self._drag_start is None:
            return
        dx = event.x() - self._drag_start.x()
        dy = event.y() - self._drag_start.y()
        cx, cy = _deg2frac(self._drag_clat, self._drag_clon, self._zoom)
        new_cx = cx - dx / TILE_SIZE
        new_cy = cy - dy / TILE_SIZE
        lat, lon = _frac2latlon(new_cx, new_cy, self._zoom)
        self._center_lat = max(-85.0, min(85.0, lat))
        self._center_lon = max(-180.0, min(180.0, lon))
        self._request_visible_tiles()
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._drag_start = None

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._request_visible_tiles()

    # ── Public API ────────────────────────────────────────────────────────────

    def update_gps(self, stage: int, lat: float, lon: float,
                   alt_ft: float, vert_vel: float, name: str):
        self._gps[stage] = dict(lat=lat, lon=lon, alt_ft=alt_ft,
                                vert_vel=vert_vel, name=name)
        self._tracks.setdefault(stage, []).append((lat, lon))
        if not self._has_gps:
            self._has_gps = True
            self._center_lat = lat
            self._center_lon = lon
            self._tiles.clear()
            self._request_visible_tiles()
        else:
            self._maybe_recenter(lat, lon)
        self.update()

    def _maybe_recenter(self, lat: float, lon: float):
        """Pan to keep the rocket visible if it drifts near the edge."""
        w, h = self.width(), self.height()
        cx, cy = _deg2frac(self._center_lat, self._center_lon, self._zoom)
        fx, fy = _deg2frac(lat, lon, self._zoom)
        px = (fx - cx) * TILE_SIZE + w / 2
        py = (fy - cy) * TILE_SIZE + h / 2
        margin = 60
        if px < margin or px > w - margin or py < margin or py > h - margin:
            self._center_lat = lat
            self._center_lon = lon
            self._request_visible_tiles()

    def update_telemetry(self, stage: int, alt_ft: float,
                         vel_fps: float, state_name: str):
        self._telemetry[stage] = dict(alt_ft=alt_ft, vel_fps=vel_fps,
                                      state_name=state_name)
        self.update()

    def show_lost_rocket(self, name: str, lat: float, lon: float, alt_ft: float):
        self._lost.append((name, lat, lon, alt_ft))
        self.update()

    def set_pad_location(self, lat: float, lon: float):
        self._pad = (lat, lon)
        self.update()

    def set_site(self, lat: float, lon: float, zoom: int):
        self._center_lat = lat
        self._center_lon = lon
        self._zoom       = zoom
        self._pad        = (lat, lon)
        self._tiles.clear()
        self._request_visible_tiles()
        self.update()

    def clear_tracks(self):
        self._gps.clear()
        self._tracks.clear()
        self._lost.clear()
        self._telemetry.clear()
        self._has_gps = False
        self.update()
