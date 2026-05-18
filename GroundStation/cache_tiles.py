#!/usr/bin/env python3
# =============================================================================
#  TRES Titan Ground Station — cache_tiles.py
#  Pre-downloads map tiles for one or more launch sites for offline use.
#
#  Run this at home on WiFi BEFORE driving to the competition site:
#      python3 cache_tiles.py              # downloads all sites in MAP_SITES
#      python3 cache_tiles.py CSUF MDARS  # downloads named sites only
#
#  Tiles are saved to tile_cache/{z}/{x}/{y}.png and served by the
#  map.html Leaflet "Offline Cache" tile layer.
# =============================================================================

import os
import sys
import math
import time
import requests
from config import MAP_SITES, TILE_CACHE_DIR

# ── CONFIGURATION ──────────────────────────────────────────────────────────
TILE_URL    = "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
SUBDOMAINS  = ["a", "b", "c"]
MIN_ZOOM    = 10
DELAY_S     = 0.05              # Rate limit: 20 tiles/second max (OSM policy)
HEADERS     = {
    "User-Agent": "TRES-CSUF-GroundStation/1.0 (tres.csuf@gmail.com)"
}
# ────────────────────────────────────────────────────────────────────────────


def deg2tile(lat_deg: float, lon_deg: float, zoom: int) -> tuple:
    lat_rad = math.radians(lat_deg)
    n = 2 ** zoom
    x = int((lon_deg + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return x, y


def tile_bounds(lat: float, lon: float, margin: float, zoom: int) -> tuple:
    x1, y1 = deg2tile(lat + margin, lon - margin, zoom)
    x2, y2 = deg2tile(lat - margin, lon + margin, zoom)
    return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)


def download_site(key: str, site: dict) -> tuple:
    """Download tiles for one site. Returns (downloaded, skipped, failed)."""
    lat      = site["lat"]
    lon      = site["lon"]
    margin   = site.get("margin", 0.10)
    max_zoom = site.get("max_zoom", 17)

    total   = 0
    skipped = 0
    failed  = 0
    sd_idx  = 0

    os.makedirs(TILE_CACHE_DIR, exist_ok=True)

    for zoom in range(MIN_ZOOM, max_zoom + 1):
        x_min, y_min, x_max, y_max = tile_bounds(lat, lon, margin, zoom)
        count = (x_max - x_min + 1) * (y_max - y_min + 1)
        print(f"  Zoom {zoom:2d}: {count:5d} tiles  "
              f"(x {x_min}–{x_max}, y {y_min}–{y_max})")

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                out_dir  = os.path.join(TILE_CACHE_DIR, str(zoom), str(x))
                out_path = os.path.join(out_dir, f"{y}.png")
                os.makedirs(out_dir, exist_ok=True)

                if os.path.exists(out_path):
                    skipped += 1
                    continue

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
                        print(f"    ✓ {zoom}/{x}/{y}  "
                              f"[+{total} cached]", end="\r")
                    else:
                        failed += 1
                        print(f"\n    ✗ HTTP {resp.status_code}: {url}")
                except requests.RequestException as e:
                    failed += 1
                    print(f"\n    ✗ {e}")

                time.sleep(DELAY_S)

    print()  # newline after final \r
    return total, skipped, failed


def download_bbox(name: str, min_lat: float, max_lat: float,
                  min_lon: float, max_lon: float,
                  min_zoom: int = 10, max_zoom: int = 17,
                  force: bool = False) -> tuple:
    """
    Download all tiles covering an explicit lat/lon bounding box.
    If force=True, re-downloads tiles that already exist in the cache.
    Returns (downloaded, skipped, failed).
    """
    total   = 0
    skipped = 0
    failed  = 0
    sd_idx  = 0

    os.makedirs(TILE_CACHE_DIR, exist_ok=True)

    for zoom in range(min_zoom, max_zoom + 1):
        x1, y1 = deg2tile(max_lat, min_lon, zoom)   # top-left tile
        x2, y2 = deg2tile(min_lat, max_lon, zoom)   # bottom-right tile
        x_min, x_max = min(x1, x2), max(x1, x2)
        y_min, y_max = min(y1, y2), max(y1, y2)
        count = (x_max - x_min + 1) * (y_max - y_min + 1)
        print(f"  Zoom {zoom:2d}: {count:5d} tiles  (x {x_min}-{x_max}, y {y_min}-{y_max})")

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                out_dir  = os.path.join(TILE_CACHE_DIR, str(zoom), str(x))
                out_path = os.path.join(out_dir, f"{y}.png")
                os.makedirs(out_dir, exist_ok=True)

                if not force and os.path.exists(out_path):
                    skipped += 1
                    continue

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
                        print(f"    ✓ {zoom}/{x}/{y}  "
                              f"[+{total} cached]", end="\r")
                    else:
                        failed += 1
                        print(f"\n    ✗ HTTP {resp.status_code}: {url}")
                except requests.RequestException as e:
                    failed += 1
                    print(f"\n    ✗ {e}")

                time.sleep(DELAY_S)

        print()  # newline after final \r

    return total, skipped, failed


if __name__ == "__main__":
    # Determine which sites to download
    requested = [a.upper() for a in sys.argv[1:]]
    if requested:
        sites = {k: v for k, v in MAP_SITES.items() if k in requested}
        unknown = set(requested) - set(MAP_SITES)
        if unknown:
            print(f"Unknown site(s): {', '.join(unknown)}")
            print(f"Available: {', '.join(MAP_SITES)}")
            sys.exit(1)
    else:
        sites = MAP_SITES

    print("TRES Ground Station — Tile Cache Download")
    print(f"Cache dir: {os.path.abspath(TILE_CACHE_DIR)}")
    print(f"Sites:     {', '.join(sites)}")
    print()

    grand_total = grand_skip = grand_fail = 0

    for key, site in sites.items():
        print(f"━━━  {key} — {site['name']}  "
              f"({site['lat']:.4f}, {site['lon']:.4f})  "
              f"margin={site.get('margin',0.10)}°  "
              f"zoom {MIN_ZOOM}–{site.get('max_zoom',17)}")
        dl, sk, fa = download_site(key, site)
        grand_total += dl
        grand_skip  += sk
        grand_fail  += fa
        print(f"  → Downloaded: {dl}  Skipped (cached): {sk}  Failed: {fa}\n")

    print("\n━━━  FAR Southeast patch (force-refresh explicit bbox)  ━━━")
    bbox_dl, bbox_sk, bbox_fa = download_bbox(
        name="FAR_SE_PATCH",
        min_lat=35.31598,
        max_lat=35.38401,
        min_lon=-117.84893,
        max_lon=-117.75624,
        min_zoom=10,
        max_zoom=17,
        force=False   # Set to True only if tiles appear corrupted
    )
    grand_total += bbox_dl
    grand_skip  += bbox_sk
    grand_fail  += bbox_fa
    print(f"  → Downloaded: {bbox_dl}  Skipped (cached): {bbox_sk}  Failed: {bbox_fa}")

    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print(f"All done.  Downloaded: {grand_total}  "
          f"Skipped: {grand_skip}  Failed: {grand_fail}")
    print(f"Cache: {os.path.abspath(TILE_CACHE_DIR)}")
