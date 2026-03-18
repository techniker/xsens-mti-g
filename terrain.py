#!/usr/bin/env python3
"""
Terrain elevation provider using AWS Terrain Tiles (Terrarium format).
Free, no API key, no rate limits.

Tile URL: https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png
Decode:   elevation_m = (R * 256 + G + B / 256) - 32768
"""

import math
import os
import struct
from PyQt6.QtCore import QUrl
from PyQt6.QtGui import QImage, QPixmap, QColor
from PyQt6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply

TILE_SIZE = 256
TERRAIN_ZOOM = 11  # ~30m resolution, good coverage radius (~50km)
CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".tile_cache")

TERRAIN_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"


def lat_lon_to_tile(lat, lon, zoom):
    n = 2 ** zoom
    x = (lon + 180.0) / 360.0 * n
    lat_rad = math.radians(lat)
    y = (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n
    return x, y


def tile_to_lat_lon(tx, ty, zoom):
    n = 2 ** zoom
    lon = tx / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * ty / n)))
    lat = math.degrees(lat_rad)
    return lat, lon


def decode_terrarium(r, g, b):
    """Decode Terrarium RGB to elevation in meters."""
    return (r * 256.0 + g + b / 256.0) - 32768.0


class TerrainProvider:
    """Fetches and caches terrain elevation tiles."""

    def __init__(self):
        self._cache = {}  # (z, tx, ty) -> list[list[float]] (elevation grid)
        self._img_cache = {}  # (z, tx, ty) -> QImage
        self._pending = set()
        self._nam = QNetworkAccessManager()
        self._nam.finished.connect(self._on_loaded)
        self._on_update = None  # callback when new tile arrives
        os.makedirs(CACHE_DIR, exist_ok=True)

    def set_update_callback(self, cb):
        self._on_update = cb

    def get_elevation_at(self, lat, lon, zoom=TERRAIN_ZOOM):
        """Get elevation at a specific lat/lon. Returns None if tile not loaded."""
        tx_f, ty_f = lat_lon_to_tile(lat, lon, zoom)
        tx, ty = int(tx_f), int(ty_f)
        grid = self._get_grid(zoom, tx, ty)
        if grid is None:
            return None
        # Pixel within tile
        px = int((tx_f - tx) * TILE_SIZE)
        py = int((ty_f - ty) * TILE_SIZE)
        px = max(0, min(TILE_SIZE - 1, px))
        py = max(0, min(TILE_SIZE - 1, py))
        return grid[py][px]

    def get_elevation_grid(self, lat, lon, heading, fov_h, fov_v, range_m,
                           grid_w, grid_h, altitude_m, pitch=0.0):
        """Sample a perspective terrain grid for synthetic vision rendering.

        Returns a list of rows, each row is a list of (elev_m, dist_m) tuples,
        sampled along rays from the aircraft position outward.
        """
        zoom = TERRAIN_ZOOM
        result = []
        hdg_rad = math.radians(heading)

        for row in range(grid_h):
            # Row 0 = far, row grid_h-1 = near
            t = 1.0 - row / max(1, grid_h - 1)  # 1.0=far, 0.0=near
            # Exponential spacing: dense near, sparse far — min 30m near
            dist = max(30, range_m * (t ** 2.0))

            row_data = []
            for col in range(grid_w):
                # Spread across FOV
                angle_offset = fov_h * ((col / max(1, grid_w - 1)) - 0.5)
                ray_hdg = hdg_rad + math.radians(angle_offset)

                # Great circle approximation (short distances)
                dlat = dist * math.cos(ray_hdg) / 111320.0
                dlon = dist * math.sin(ray_hdg) / (111320.0 * max(0.01, math.cos(math.radians(lat))))

                sample_lat = lat + dlat
                sample_lon = lon + dlon

                elev = self.get_elevation_at(sample_lat, sample_lon, zoom)
                row_data.append((elev if elev is not None else 0.0, dist))

            result.append(row_data)

        return result

    def prefetch_around(self, lat, lon, radius_tiles=2, zoom=TERRAIN_ZOOM):
        """Prefetch terrain tiles around a position."""
        tx_f, ty_f = lat_lon_to_tile(lat, lon, zoom)
        tx, ty = int(tx_f), int(ty_f)
        n = 2 ** zoom
        for dx in range(-radius_tiles, radius_tiles + 1):
            for dy in range(-radius_tiles, radius_tiles + 1):
                ttx = (tx + dx) % n
                tty = ty + dy
                if 0 <= tty < n:
                    self._get_grid(zoom, ttx, tty)

    def _get_grid(self, z, tx, ty):
        key = (z, tx, ty)
        if key in self._cache:
            return self._cache[key]

        # Try disk cache
        path = self._cache_path(z, tx, ty)
        if os.path.exists(path):
            img = QImage(path)
            if not img.isNull():
                grid = self._decode_image(img)
                self._cache[key] = grid
                return grid

        # Fetch
        if key not in self._pending:
            self._pending.add(key)
            self._fetch(z, tx, ty)
        return None

    def _cache_path(self, z, tx, ty):
        return os.path.join(CACHE_DIR, f"terrain_{z}_{tx}_{ty}.png")

    def _fetch(self, z, tx, ty):
        url = TERRAIN_URL.replace("{z}", str(z)).replace("{x}", str(tx)).replace("{y}", str(ty))
        request = QNetworkRequest(QUrl(url))
        request.setRawHeader(b"User-Agent", b"XsensMTiG-PFD/1.0")
        request.setAttribute(QNetworkRequest.Attribute.User, (z, tx, ty))
        self._nam.get(request)

    def _on_loaded(self, reply: QNetworkReply):
        attr = reply.request().attribute(QNetworkRequest.Attribute.User)
        if attr is None:
            reply.deleteLater()
            return

        z, tx, ty = attr
        key = (z, tx, ty)
        self._pending.discard(key)

        if reply.error() == QNetworkReply.NetworkError.NoError:
            data = reply.readAll().data()
            img = QImage()
            if img.loadFromData(data):
                # Save to disk
                path = self._cache_path(z, tx, ty)
                img.save(path, "PNG")
                # Decode
                grid = self._decode_image(img)
                self._cache[key] = grid
                if self._on_update:
                    self._on_update()

        reply.deleteLater()

    def _decode_image(self, img: QImage):
        """Decode a Terrarium PNG into a 2D elevation grid.
        Uses raw pixel data for performance (vs pixelColor per pixel)."""
        img = img.convertToFormat(QImage.Format.Format_RGB32)
        w, h = img.width(), img.height()
        ptr = img.bits()
        ptr.setsize(h * img.bytesPerLine())
        raw = bytes(ptr)
        bpl = img.bytesPerLine()
        grid = []
        for y in range(h):
            row = []
            offset = y * bpl
            for x in range(w):
                i = offset + x * 4
                # Format_RGB32: BGRA byte order
                b = raw[i]
                g = raw[i + 1]
                r = raw[i + 2]
                row.append((r * 256.0 + g + b / 256.0) - 32768.0)
            grid.append(row)
        return grid
