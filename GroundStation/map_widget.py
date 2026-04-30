# =============================================================================
#  TRES Titan Ground Station — map_widget.py
#  PyQt5 widget that embeds Leaflet.js via QWebEngineView.
#  Python calls JavaScript functions to update markers and tracks.
# =============================================================================

import os
from PyQt5.QtCore    import QUrl, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage, QWebEngineSettings

import config


class SilentPage(QWebEnginePage):
    """Log JavaScript errors; suppress noisy tile-404 warnings."""
    def javaScriptConsoleMessage(self, level, msg, line, source):
        # Level 0=Info, 1=Warning, 2=Error — print errors and warnings only
        if level >= 1:
            print(f"[MAP JS] {source}:{line} — {msg}")


class MapWidget(QWidget):
    """
    Embeds a Leaflet map in a QWebEngineView.

    Usage:
        map_widget = MapWidget()
        map_widget.update_gps(stage=1, lat=35.173, lon=-117.815,
                              alt_ft=1200, vert_vel=50, name="TRES-S1")
        map_widget.update_telemetry(stage=2, alt_ft=8000, vel_fps=200,
                                    state_name="COAST_S2")
        map_widget.show_lost_rocket("TRES-S2", 35.180, -117.820, 500)
        map_widget.clear_tracks()
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._view = QWebEngineView()
        self._page = SilentPage(self._view)
        self._view.setPage(self._page)

        # Allow the page to access other local file:// resources (tiles, leaflet).
        # setHtml() assigns a data: origin which blocks local file loads; using
        # load(file://) avoids that.  LocalContentCanAccessLocalUrls is the belt-
        # and-suspenders safety net for any remaining cross-file restrictions.
        self._view.settings().setAttribute(
            QWebEngineSettings.LocalContentCanAccessLocalUrls, True)

        layout.addWidget(self._view)

        # Write the processed HTML (with substituted coords) to a file so we can
        # load it via file:// — setHtml() assigns a data: origin which blocks all
        # relative file:// resources (Leaflet JS/CSS, tile PNGs).
        html = self._build_html()
        live_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "assets", "map_live.html"
        )
        with open(live_path, "w", encoding="utf-8") as f:
            f.write(html)
        self._view.load(QUrl.fromLocalFile(live_path))

    def _build_html(self) -> str:
        """Read the map.html template and substitute Python config values."""
        template_path = os.path.join(
            os.path.dirname(__file__), "assets", "map.html"
        )
        with open(template_path, "r") as f:
            html = f.read()

        # Replace placeholder constants with values from config
        html = html.replace("MAP_CENTER_LAT",  str(config.MAP_CENTER_LAT))
        html = html.replace("MAP_CENTER_LON",  str(config.MAP_CENTER_LON))
        html = html.replace("MAP_CENTER_ZOOM", str(config.MAP_CENTER_ZOOM))
        return html

    # ------------------------------------------------------------------
    #  Public update API — safe to call from main thread at any time
    # ------------------------------------------------------------------

    def _js(self, script: str):
        """Execute JavaScript in the map page."""
        self._page.runJavaScript(script)

    def update_gps(self, stage: int, lat: float, lon: float,
                   alt_ft: float, vert_vel: float, name: str):
        """
        Update position from Featherweight GPS Ground Station.
        This is the primary position source; moves the GPS marker.
        """
        self._js(
            f"updateRocketGPS({stage}, {lat}, {lon}, "
            f"{alt_ft:.1f}, {vert_vel:.1f}, '{name}');"
        )

    def update_telemetry(self, stage: int, alt_ft: float,
                         vel_fps: float, state_name: str):
        """
        Update info overlay from SRAD LoRa telemetry (altitude, velocity).
        Does not move a map marker — no GPS in SRAD system.
        """
        safe_name = state_name.replace("'", "")
        self._js(
            f"updateTelemetry({stage}, {alt_ft:.1f}, "
            f"{vel_fps:.1f}, '{safe_name}');"
        )

    def show_lost_rocket(self, name: str, lat: float,
                         lon: float, alt_ft: float):
        """Add a lost-rocket marker to the map."""
        safe_name = name.replace("'", "")
        self._js(
            f"showLostRocket('{safe_name}', {lat}, {lon}, {alt_ft:.0f});"
        )

    def set_pad_location(self, lat: float, lon: float):
        """Move the pad marker (e.g. after GPS fix on the ground station)."""
        self._js(f"setPadLocation({lat}, {lon});")

    def set_site(self, lat: float, lon: float, zoom: int):
        """Recenter map and move pad marker to a named launch site."""
        self._js(f"setPadLocation({lat}, {lon}, {zoom});")

    def clear_tracks(self):
        """Wipe all flight path polylines and markers."""
        self._js("clearTracks();")
