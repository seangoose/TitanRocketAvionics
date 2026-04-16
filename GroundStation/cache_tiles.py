#!/usr/bin/env python3
# =============================================================================
#  TRES Titan Ground Station — cache_tiles.py
#  Pre-downloads map tiles for a launch site area for offline use.
#
#  Run this at home on WiFi BEFORE driving to the competition site:
#      python3 cache_tiles.py
#
#  Tiles are saved to tile_cache/{z}/{x}/{y}.png and served by the
#  map.html Leaflet "Offline Cache" tile layer.
#
#  Default area: FAR (Friends of Amateur Rocketry), Mojave CA.
#  Edit LAT, LON, and RADIUS_KM in config.py to match your launch site.
#  Zoom levels 10–17 cover region overview through street-level detail.
#  ~500 tiles at zoom 10–15, ~4000 tiles at zoom 10–17.
# =============================================================================

import os
import math
import time
import requests
from config import MAP_CENTER_LAT, MAP_CENTER_LON, TILE_CACHE_DIR

# ── CONFIGURATION ──────────────────────────────────────────────────────────
TILE_URL    = "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
SUBDOMAINS  = ["a", "b", "c"]   # OSM tile servers
MIN_ZOOM    = 10
MAX_ZOOM    = 17
# Bounding box margin around center (degrees)
# ~0.15 degrees ≈ 16 km at Mojave latitude
MARGIN_DEG  = 0.15
DELAY_S     = 0.05              # Rate limit: 20 tiles/second max (OSM policy)
HEADERS     = {
    "User-Agent": "TRES-CSUF-GroundStation/1.0 (tres.csuf@gmail.com)"
}
# ────────────────────────────────────────────────────────────────────────────


def deg2tile(lat_deg: float, lon_deg: float, zoom: int) -> tuple:
    """Convert lat/lon decimal degrees to tile x/y at a given zoom level."""
    lat_rad = math.radians(lat_deg)
    n = 2 ** zoom
    x = int((lon_deg + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return x, y


def tile_bounds(lat: float, lon: float, margin: float, zoom: int) -> tuple:
    """Return (x_min, y_min, x_max, y_max) tile range for the given area."""
    x1, y1 = deg2tile(lat + margin, lon - margin, zoom)
    x2, y2 = deg2tile(lat - margin, lon + margin, zoom)
    return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)


def download_tiles():
    os.makedirs(TILE_CACHE_DIR, exist_ok=True)
    total    = 0
    skipped  = 0
    failed   = 0
    sd_idx   = 0

    for zoom in range(MIN_ZOOM, MAX_ZOOM + 1):
        x_min, y_min, x_max, y_max = tile_bounds(
            MAP_CENTER_LAT, MAP_CENTER_LON, MARGIN_DEG, zoom)
        count = (x_max - x_min + 1) * (y_max - y_min + 1)
        print(f"\nZoom {zoom}: {count} tiles "
              f"(x {x_min}–{x_max}, y {y_min}–{y_max})")

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                out_dir  = os.path.join(TILE_CACHE_DIR, str(zoom), str(x))
                out_path = os.path.join(out_dir, f"{y}.png")
                os.makedirs(out_dir, exist_ok=True)

                if os.path.exists(out_path):
                    skipped += 1
                    continue

                # Round-robin across OSM subdomains to spread load
                sub = SUBDOMAINS[sd_idx % len(SUBDOMAINS)]
                sd_idx += 1
                url = (TILE_URL
                       .replace("{s}", sub)
                       .replace("{z}", str(zoom))
                       .replace("{x}", str(x))
                       .replace("{y}", str(y)))
                try:
                    resp = requests.get(url, headers=HEADERS, timeout=10)
                    if resp.status_code == 200:
                        with open(out_path, "wb") as f:
                            f.write(resp.content)
                        total += 1
                        print(f"  ✓ {zoom}/{x}/{y}", end="\r")
                    else:
                        failed += 1
                        print(f"  ✗ HTTP {resp.status_code}: {url}")
                except requests.RequestException as e:
                    failed += 1
                    print(f"  ✗ {e}")

                time.sleep(DELAY_S)

    print(f"\n\nDone. Downloaded: {total}  Skipped (cached): {skipped}  "
          f"Failed: {failed}")
    print(f"Tile cache: {os.path.abspath(TILE_CACHE_DIR)}")


if __name__ == "__main__":
    print("TRES Ground Station — Tile Cache Download")
    print(f"Center: {MAP_CENTER_LAT}°N, {MAP_CENTER_LON}°E")
    print(f"Zoom:   {MIN_ZOOM}–{MAX_ZOOM}")
    print(f"Cache:  {os.path.abspath(TILE_CACHE_DIR)}")
    print()
    download_tiles()
