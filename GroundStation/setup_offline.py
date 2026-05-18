#!/usr/bin/env python3
# =============================================================================
#  TRES Titan Ground Station — setup_offline.py
#
#  Run this ONCE at home on WiFi before driving to the competition site.
#  After this script completes, the ground station runs with ZERO internet
#  dependency — no CDN, no tile servers, nothing external.
#
#  What it does:
#    1. Downloads Leaflet 1.9.4 JS + CSS into assets/
#    2. Downloads OSM map tiles for the launch site area into tile_cache/
#       covering zoom levels 10–17 (~16 km radius around MAP_CENTER)
#
#  Usage:
#    python3 setup_offline.py
#
#  Re-running is safe — already-cached tiles and library files are skipped.
#  To force a re-download of everything: python3 setup_offline.py --clean
# =============================================================================

import os
import sys
import math
import time
import shutil
import requests
from config import MAP_SITES, TILE_CACHE_DIR

# ── Leaflet build to download ─────────────────────────────────────────────
LEAFLET_VERSION = "1.9.4"
LEAFLET_BASE    = f"https://unpkg.com/leaflet@{LEAFLET_VERSION}/dist"
LEAFLET_FILES   = {
    "leaflet.js":  f"{LEAFLET_BASE}/leaflet.js",
    "leaflet.css": f"{LEAFLET_BASE}/leaflet.css",
}
# Leaflet also uses marker icon images referenced in the CSS
LEAFLET_IMAGES  = {
    "marker-icon.png":    f"{LEAFLET_BASE}/images/marker-icon.png",
    "marker-icon-2x.png": f"{LEAFLET_BASE}/images/marker-icon-2x.png",
    "marker-shadow.png":  f"{LEAFLET_BASE}/images/marker-shadow.png",
    "layers.png":         f"{LEAFLET_BASE}/images/layers.png",
    "layers-2x.png":      f"{LEAFLET_BASE}/images/layers-2x.png",
}

# ── Tile download settings ───────────────────────────────────────────────
TILE_URL       = "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
SUBDOMAINS     = ["a", "b", "c"]
MIN_ZOOM       = 10      # minimum zoom cached for all sites
DELAY_S        = 0.05    # 20 tiles/second — respects OSM usage policy
HEADERS        = {
    "User-Agent": (
        "TRES-CSUF-GroundStation/1.0 "
        "(contact: tres@csuf.edu; github: TRES-CSUF)"
    )
}

ASSETS_DIR     = os.path.join(os.path.dirname(__file__), "assets")


# =============================================================================
#  HELPERS
# =============================================================================

def download_file(url: str, dest_path: str, label: str) -> bool:
    """Download url to dest_path. Returns True on success."""
    try:
        resp = requests.get(url, headers=HEADERS, timeout=30)
        resp.raise_for_status()
        os.makedirs(os.path.dirname(dest_path), exist_ok=True)
        with open(dest_path, "wb") as f:
            f.write(resp.content)
        print(f"  ✓  {label}")
        return True
    except requests.RequestException as e:
        print(f"  ✗  {label}  [{e}]")
        return False


def deg2tile(lat: float, lon: float, zoom: int) -> tuple:
    lat_r = math.radians(lat)
    n = 2 ** zoom
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n)
    return x, y


def tile_range(lat: float, lon: float, margin: float, zoom: int) -> tuple:
    x1, y1 = deg2tile(lat + margin, lon - margin, zoom)
    x2, y2 = deg2tile(lat - margin, lon + margin, zoom)
    return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)


# =============================================================================
#  STEP 1 — LEAFLET LIBRARY FILES
# =============================================================================

def download_leaflet() -> bool:
    print("\n── Step 1: Leaflet library files ─────────────────────────────")
    os.makedirs(ASSETS_DIR, exist_ok=True)
    images_dir = os.path.join(ASSETS_DIR, "images")
    os.makedirs(images_dir, exist_ok=True)

    ok = True
    for filename, url in LEAFLET_FILES.items():
        dest = os.path.join(ASSETS_DIR, filename)
        if os.path.exists(dest):
            print(f"  ↩  {filename}  (already cached)")
            continue
        ok = download_file(url, dest, filename) and ok

    for filename, url in LEAFLET_IMAGES.items():
        dest = os.path.join(images_dir, filename)
        if os.path.exists(dest):
            print(f"  ↩  images/{filename}  (already cached)")
            continue
        ok = download_file(url, dest, f"images/{filename}") and ok

    if ok:
        print("\n  Leaflet library ready at assets/")
    else:
        print("\n  WARNING: Some Leaflet files failed — map may not render")
    return ok


# =============================================================================
#  STEP 2 — MAP TILES (all sites)
# =============================================================================

def download_tiles_for_site(site_key: str, site: dict, sd_idx: int, stats: dict) -> int:
    lat      = site["lat"]
    lon      = site["lon"]
    margin   = site["margin"]
    max_zoom = site["max_zoom"]

    total_est = sum(
        (tile_range(lat, lon, margin, z)[2] - tile_range(lat, lon, margin, z)[0] + 1) *
        (tile_range(lat, lon, margin, z)[3] - tile_range(lat, lon, margin, z)[1] + 1)
        for z in range(MIN_ZOOM, max_zoom + 1)
    )
    print(f"\n  Site: {site['name']}  |  ±{margin}° radius  |  zoom {MIN_ZOOM}–{max_zoom}  |  ~{total_est} tiles")

    for zoom in range(MIN_ZOOM, max_zoom + 1):
        x_min, y_min, x_max, y_max = tile_range(lat, lon, margin, zoom)
        count = (x_max - x_min + 1) * (y_max - y_min + 1)
        print(f"    Zoom {zoom:2d}: {count:5d} tiles", end="  ", flush=True)

        zoom_dl = 0
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                out_dir  = os.path.join(TILE_CACHE_DIR, str(zoom), str(x))
                out_path = os.path.join(out_dir, f"{y}.png")

                if os.path.exists(out_path):
                    stats["skipped"] += 1
                    continue

                os.makedirs(out_dir, exist_ok=True)
                sub = SUBDOMAINS[sd_idx % len(SUBDOMAINS)]
                sd_idx += 1
                url = (TILE_URL
                       .replace("{s}", sub)
                       .replace("{z}", str(zoom))
                       .replace("{x}", str(x))
                       .replace("{y}", str(y)))
                try:
                    resp = requests.get(url, headers=HEADERS, timeout=15)
                    if resp.status_code == 200:
                        with open(out_path, "wb") as f:
                            f.write(resp.content)
                        stats["downloaded"] += 1
                        zoom_dl += 1
                    else:
                        stats["failed"] += 1
                except requests.RequestException:
                    stats["failed"] += 1

                time.sleep(DELAY_S)

        print(f"✓ {zoom_dl} new, {count - zoom_dl} cached")

    return sd_idx


def download_tiles() -> dict:
    print("\n── Step 2: OSM map tiles (all sites) ─────────────────────────")
    stats  = {"downloaded": 0, "skipped": 0, "failed": 0}
    sd_idx = 0
    for key, site in MAP_SITES.items():
        sd_idx = download_tiles_for_site(key, site, sd_idx, stats)
    return stats


# =============================================================================
#  STEP 3 — VERIFY OFFLINE READINESS
# =============================================================================

def verify_offline():
    print("\n── Step 3: Offline readiness check ───────────────────────────")
    issues = []

    for fname in ["leaflet.js", "leaflet.css"]:
        path = os.path.join(ASSETS_DIR, fname)
        if not os.path.exists(path):
            issues.append(f"MISSING: assets/{fname}")
        else:
            size = os.path.getsize(path)
            print(f"  ✓  assets/{fname}  ({size:,} bytes)")

    # Check center tile for each site
    for key, site in MAP_SITES.items():
        check_zoom = min(13, site["max_zoom"])
        x, y = deg2tile(site["lat"], site["lon"], check_zoom)
        center_tile = os.path.join(TILE_CACHE_DIR, str(check_zoom), str(x), f"{y}.png")
        if os.path.exists(center_tile):
            print(f"  ✓  {site['name']} center tile at zoom {check_zoom} present")
        else:
            issues.append(f"MISSING: {site['name']} center tile at zoom {check_zoom}")

    # FAR SE patch center tile check
    se_check_zoom = 15
    se_x, se_y = deg2tile(35.35, -117.76, se_check_zoom)
    se_tile = os.path.join(TILE_CACHE_DIR, str(se_check_zoom), str(se_x), f"{se_y}.png")
    if os.path.exists(se_tile):
        print(f"  ✓  FAR SE patch center tile at zoom {se_check_zoom} present")
    else:
        issues.append("MISSING: FAR SE patch center tile — run cache_tiles.py to download")

    # Count total cached tiles
    tile_count = sum(
        len(files) for _, _, files in os.walk(TILE_CACHE_DIR)
    )
    print(f"  ✓  {tile_count} total tiles in cache")

    if issues:
        print("\n  ⚠  Issues found:")
        for i in issues:
            print(f"     {i}")
        print("\n  Re-run setup_offline.py to fix missing files.")
    else:
        print("\n  ✅  System is FULLY OFFLINE CAPABLE")
        print("      No internet connection required at launch site.")

    return len(issues) == 0


# =============================================================================
#  MAIN
# =============================================================================

if __name__ == "__main__":
    clean = "--clean" in sys.argv
    if clean:
        print("-- Clean mode: removing existing cache --")
        if os.path.exists(TILE_CACHE_DIR):
            shutil.rmtree(TILE_CACHE_DIR)
            print(f"  Removed {TILE_CACHE_DIR}")
        for fname in list(LEAFLET_FILES.keys()) + list(LEAFLET_IMAGES.keys()):
            p = os.path.join(ASSETS_DIR, fname)
            if os.path.exists(p):
                os.remove(p)

    print("=" * 60)
    print(" TRES Titan — Offline Setup")
    print(f" Launch site: {MAP_CENTER_LAT}°N, {MAP_CENTER_LON}°E")
    print("=" * 60)

    leaflet_ok = download_leaflet()
    tile_stats  = download_tiles()

    print(f"\n── Tile download summary ──────────────────────────────────────")
    print(f"  Downloaded: {tile_stats['downloaded']}")
    print(f"  Skipped:    {tile_stats['skipped']}  (already cached)")
    print(f"  Failed:     {tile_stats['failed']}")

    all_ok = verify_offline()
    sys.exit(0 if all_ok else 1)
