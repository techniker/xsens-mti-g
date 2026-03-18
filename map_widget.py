#!/usr/bin/env python3
"""
Slippy map widget — live moving map with aircraft position.
Supports OpenStreetMap tiles with OpenAIP overlay capability.
"""

import math
import os
import hashlib
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QPointF, QRectF, QUrl, QTimer
from PyQt6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QPolygonF, QImage, QPixmap,
)
from PyQt6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply

TILE_SIZE = 256
CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".tile_cache")

# Map providers — tile URL templates
PROVIDERS = {
    "OpenStreetMap": {
        "url": "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
        "subdomains": ["a", "b", "c"],
        "attribution": "OpenStreetMap",
        "max_zoom": 19,
        "api_key": False,
    },
    "OpenTopoMap": {
        "url": "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
        "subdomains": ["a", "b", "c"],
        "attribution": "OpenTopoMap",
        "max_zoom": 17,
        "api_key": False,
    },
    "OpenAIP": {
        "url": "https://{s}.api.tiles.openaip.net/api/data/openaip/{z}/{x}/{y}.png?apiKey={key}",
        "subdomains": ["a", "b", "c"],
        "attribution": "OpenAIP",
        "max_zoom": 14,
        "api_key": True,
        "key_name": "openaip_api_key",
    },
}

DEFAULT_PROVIDER = "OpenStreetMap"


def _lat_lon_to_tile(lat, lon, zoom):
    n = 2 ** zoom
    x = (lon + 180.0) / 360.0 * n
    lat_rad = math.radians(lat)
    y = (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n
    return x, y


def _tile_to_lat_lon(tx, ty, zoom):
    n = 2 ** zoom
    lon = tx / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * ty / n)))
    lat = math.degrees(lat_rad)
    return lat, lon


class MapWidget(QWidget):
    """Slippy map centered on aircraft position with heading-up rotation."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent)

        self._lat = 0.0
        self._lon = 0.0
        self._heading = 0.0
        self._zoom = 14
        self._min_zoom = 4
        self._max_zoom = 18
        self._has_position = False
        self._heading_up = False  # False = north up, True = heading up
        self._disp_heading = 0.0  # smoothed display heading for rotation
        self._heading_alpha = 0.80  # smoothing for heading rotation

        self._provider_name = DEFAULT_PROVIDER
        self._overlay_enabled = False  # OpenAIP overlay

        # Pan offset (pixels) from aircraft center
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._centered = True  # auto-follow aircraft
        self._drag_start = None  # mouse drag start point

        # Tile cache (in-memory + disk)
        self._tile_cache = {}  # (z, x, y, provider) -> QPixmap
        self._pending = set()

        # Network
        self._nam = QNetworkAccessManager(self)
        self._nam.finished.connect(self._on_tile_loaded)

        # API keys (loaded from config later)
        self._api_keys = {}

        # Ensure cache dir
        os.makedirs(CACHE_DIR, exist_ok=True)

    @property
    def provider_name(self):
        return self._provider_name

    @provider_name.setter
    def provider_name(self, name):
        if name in PROVIDERS:
            self._provider_name = name
            self._max_zoom = PROVIDERS[name]["max_zoom"]
            self._zoom = min(self._zoom, self._max_zoom)
            self._pending.clear()
            self.update()

    @property
    def heading_up(self):
        return self._heading_up

    @heading_up.setter
    def heading_up(self, val):
        self._heading_up = val
        if val:
            self._disp_heading = self._heading
        self.update()

    def set_api_key(self, provider, key):
        self._api_keys[provider] = key
        self._pending.clear()
        self.update()

    def set_position(self, lat, lon, heading):
        if lat == 0.0 and lon == 0.0:
            return
        self._lat = lat
        self._lon = lon
        self._heading = heading
        self._has_position = True
        if self._centered:
            self._pan_x = 0.0
            self._pan_y = 0.0
        # Smooth heading for map rotation (handle 360/0 wraparound)
        if self._heading_up:
            diff = (heading - self._disp_heading + 180) % 360 - 180
            self._disp_heading = (self._disp_heading + (1 - self._heading_alpha) * diff) % 360
        self.update()

    def center_on_aircraft(self):
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._centered = True
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_start = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._drag_start is not None:
            delta = event.position() - self._drag_start
            self._pan_x += delta.x()
            self._pan_y += delta.y()
            self._centered = False
            self._drag_start = event.position()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_start = None
            self.setCursor(Qt.CursorShape.ArrowCursor)

    def set_zoom(self, z):
        new_z = max(self._min_zoom, min(self._max_zoom, z))
        if new_z != self._zoom:
            self._zoom = new_z
            self._pending.clear()
            self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0:
            self.set_zoom(self._zoom + 1)
        elif delta < 0:
            self.set_zoom(self._zoom - 1)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(0x0A, 0x0C, 0x10))

        if not self._has_position:
            self._draw_no_fix(p)
            p.end()
            return

        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        zoom = self._zoom
        map_rotation = self._disp_heading if (self._heading_up and self._centered) else 0.0

        # Aircraft position in tile coordinates
        ax, ay = _lat_lon_to_tile(self._lat, self._lon, zoom)

        # Calculate which tiles we need (with margin for rotation + pan)
        pan_margin = math.hypot(self._pan_x, self._pan_y)
        diag = math.hypot(w, h) / 2.0 + TILE_SIZE + pan_margin
        tiles_needed = int(math.ceil(diag / TILE_SIZE)) + 2

        tile_cx = int(ax)
        tile_cy = int(ay)
        frac_x = (ax - tile_cx) * TILE_SIZE
        frac_y = (ay - tile_cy) * TILE_SIZE

        provider = PROVIDERS.get(self._provider_name, PROVIDERS[DEFAULT_PROVIDER])

        # Rotate canvas for heading-up mode (around aircraft position)
        ac_sx = cx + self._pan_x
        ac_sy = cy + self._pan_y
        p.save()
        p.translate(ac_sx, ac_sy)
        p.rotate(-map_rotation)
        p.translate(-ac_sx, -ac_sy)

        # Draw tiles
        for dx in range(-tiles_needed, tiles_needed + 1):
            for dy in range(-tiles_needed, tiles_needed + 1):
                tx = tile_cx + dx
                ty = tile_cy + dy
                n = 2 ** zoom
                if ty < 0 or ty >= n:
                    continue
                tx = tx % n

                px = cx + (dx * TILE_SIZE) - frac_x + self._pan_x
                py = cy + (dy * TILE_SIZE) - frac_y + self._pan_y

                pixmap = self._get_tile(zoom, tx, ty)
                if pixmap:
                    p.drawPixmap(int(px), int(py), pixmap)
                else:
                    ipx, ipy = int(px), int(py)
                    p.fillRect(ipx, ipy, TILE_SIZE, TILE_SIZE, QColor(30, 32, 40))
                    p.setPen(QPen(QColor(200, 50, 50), 1))
                    p.setBrush(Qt.BrushStyle.NoBrush)
                    p.drawRect(ipx, ipy, TILE_SIZE, TILE_SIZE)
                    nf = QFont("Monospace", 16)
                    nf.setBold(True)
                    p.setFont(nf)
                    p.drawText(QRectF(ipx, ipy, TILE_SIZE, TILE_SIZE),
                               Qt.AlignmentFlag.AlignCenter, "NO DATA")

        # Draw overlay tiles (OpenAIP)
        if self._overlay_enabled and self._api_keys.get("OpenAIP"):
            for dx in range(-tiles_needed, tiles_needed + 1):
                for dy in range(-tiles_needed, tiles_needed + 1):
                    tx = tile_cx + dx
                    ty = tile_cy + dy
                    n = 2 ** zoom
                    if ty < 0 or ty >= n:
                        continue
                    tx = tx % n
                    px = cx + (dx * TILE_SIZE) - frac_x + self._pan_x
                    py = cy + (dy * TILE_SIZE) - frac_y + self._pan_y
                    overlay = self._get_tile(zoom, tx, ty, provider_override="OpenAIP")
                    if overlay:
                        p.drawPixmap(int(px), int(py), overlay)

        p.restore()  # undo map rotation

        # Prefetch surrounding tiles for smooth movement
        self._prefetch_tiles(ax, ay, zoom, self._provider_name)
        if self._overlay_enabled and self._api_keys.get("OpenAIP"):
            self._prefetch_tiles(ax, ay, zoom, "OpenAIP")

        # Aircraft at screen center + pan (stays at its geographic position)
        ac_x = cx + self._pan_x
        ac_y = cy + self._pan_y
        ac_rotation = 0.0 if (self._heading_up and self._centered) else self._heading
        self._draw_aircraft(p, ac_x, ac_y, ac_rotation)

        # Attribution
        attr = provider.get("attribution", "")
        if attr:
            font = p.font()
            font.setPixelSize(10)
            p.setFont(font)
            p.setPen(QPen(QColor(150, 150, 150, 180)))
            p.drawText(4, h - 4, f"\u00A9 {attr}")

        # Zoom level
        font = p.font()
        font.setPixelSize(11)
        font.setBold(True)
        p.setFont(font)
        p.setPen(QPen(QColor(200, 200, 200)))
        p.drawText(w - 40, h - 4, f"Z{zoom}")

        p.end()

    def _draw_no_fix(self, p: QPainter):
        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0

        # Dark background with crosshatch
        p.fillRect(self.rect(), QColor(0x0A, 0x0C, 0x10))

        # Red X like the PFD fail flags
        margin = min(w, h) * 0.3
        pen = QPen(QColor(200, 50, 50, 120), 3)
        p.setPen(pen)
        p.drawLine(int(cx - margin), int(cy - margin), int(cx + margin), int(cy + margin))
        p.drawLine(int(cx + margin), int(cy - margin), int(cx - margin), int(cy + margin))

        # Message box
        box_w, box_h = 220, 50
        box_x = cx - box_w / 2
        box_y = cy - box_h / 2
        p.setPen(QPen(QColor(200, 50, 50), 2))
        p.setBrush(QColor(10, 12, 16, 220))
        p.drawRoundedRect(int(box_x), int(box_y), box_w, box_h, 6, 6)

        font = p.font()
        font.setPixelSize(16)
        font.setBold(True)
        p.setFont(font)
        p.setPen(QPen(QColor(255, 80, 80)))
        p.drawText(QRectF(box_x, box_y, box_w, box_h),
                   Qt.AlignmentFlag.AlignCenter, "MAP\nWaiting for GPS fix")

    def _prefetch_tiles(self, ax, ay, zoom, provider_name):
        """Prefetch tiles in a ring around current view for smooth panning."""
        tiles_needed = int(math.ceil(math.hypot(self.width(), self.height()) / 2.0 / TILE_SIZE)) + 4
        tile_cx = int(ax)
        tile_cy = int(ay)
        n = 2 ** zoom
        for dx in range(-tiles_needed, tiles_needed + 1):
            for dy in range(-tiles_needed, tiles_needed + 1):
                tx = (tile_cx + dx) % n
                ty = tile_cy + dy
                if ty < 0 or ty >= n:
                    continue
                self._get_tile(zoom, tx, ty, provider_override=provider_name)

    def _draw_aircraft(self, p: QPainter, cx, cy, heading):
        p.save()
        p.translate(cx, cy)
        p.rotate(heading)

        s = 16  # scale factor
        p.setPen(QPen(QColor(0, 0, 0), 2))
        p.setBrush(QColor(255, 255, 255))

        # Airplane top-down silhouette
        fuselage = QPolygonF([
            QPointF(0, -s * 1.4),       # nose
            QPointF(-s * 0.15, -s * 0.8),
            QPointF(-s * 0.15, s * 0.8),
            QPointF(-s * 0.25, s * 1.2), # tail left
            QPointF(s * 0.25, s * 1.2),  # tail right
            QPointF(s * 0.15, s * 0.8),
            QPointF(s * 0.15, -s * 0.8),
        ])
        p.drawPolygon(fuselage)

        # Wings
        wings = QPolygonF([
            QPointF(0, -s * 0.15),
            QPointF(-s * 1.1, s * 0.3),
            QPointF(-s * 1.1, s * 0.5),
            QPointF(0, s * 0.15),
            QPointF(s * 1.1, s * 0.5),
            QPointF(s * 1.1, s * 0.3),
        ])
        p.drawPolygon(wings)

        # Horizontal stabilizer
        tail = QPolygonF([
            QPointF(0, s * 0.8),
            QPointF(-s * 0.5, s * 1.1),
            QPointF(-s * 0.5, s * 1.2),
            QPointF(0, s * 0.95),
            QPointF(s * 0.5, s * 1.2),
            QPointF(s * 0.5, s * 1.1),
        ])
        p.drawPolygon(tail)

        p.restore()

    def _get_tile(self, z, x, y, provider_override=None):
        provider = provider_override or self._provider_name
        key = (z, x, y, provider)

        # In-memory cache
        if key in self._tile_cache:
            return self._tile_cache[key]

        # Disk cache
        cache_path = self._cache_path(z, x, y, provider)
        if os.path.exists(cache_path):
            pixmap = QPixmap(cache_path)
            if not pixmap.isNull():
                self._tile_cache[key] = pixmap
                return pixmap

        # Fetch from network (don't add to pending if it will be skipped)
        if key not in self._pending:
            prov = PROVIDERS.get(provider, PROVIDERS[DEFAULT_PROVIDER])
            if prov.get("api_key") and not self._api_keys.get(provider):
                return None  # no API key — don't block future retries
            self._pending.add(key)
            self._fetch_tile(z, x, y, provider)

        return None

    def _cache_path(self, z, x, y, provider):
        safe_name = provider.replace(" ", "_").lower()
        return os.path.join(CACHE_DIR, f"{safe_name}_{z}_{x}_{y}.png")

    def _fetch_tile(self, z, x, y, provider_name):
        provider = PROVIDERS.get(provider_name, PROVIDERS[DEFAULT_PROVIDER])
        url_template = provider["url"]
        subdomains = provider.get("subdomains", ["a"])
        s = subdomains[(x + y) % len(subdomains)]

        url = url_template.replace("{s}", s).replace("{z}", str(z)).replace("{x}", str(x)).replace("{y}", str(y))

        if provider.get("api_key"):
            key_name = provider.get("key_name", "")
            api_key = self._api_keys.get(provider_name, "")
            url = url.replace("{key}", api_key)
            if not api_key:
                return  # skip if no API key configured

        request = QNetworkRequest(QUrl(url))
        request.setRawHeader(b"User-Agent", b"XsensMTiG-PFD/1.0")
        request.setAttribute(QNetworkRequest.Attribute.User, (z, x, y, provider_name))
        self._nam.get(request)

    def _on_tile_loaded(self, reply: QNetworkReply):
        attr = reply.request().attribute(QNetworkRequest.Attribute.User)
        if attr is None:
            reply.deleteLater()
            return

        z, x, y, provider_name = attr
        key = (z, x, y, provider_name)
        self._pending.discard(key)

        if reply.error() == QNetworkReply.NetworkError.NoError:
            data = reply.readAll().data()
            pixmap = QPixmap()
            if pixmap.loadFromData(data):
                # Evict old entries if memory cache grows too large
                if len(self._tile_cache) > 500:
                    keys = list(self._tile_cache.keys())
                    for k in keys[:200]:
                        del self._tile_cache[k]
                self._tile_cache[key] = pixmap
                # Save to disk cache
                cache_path = self._cache_path(z, x, y, provider_name)
                pixmap.save(cache_path, "PNG")
                self.update()

        reply.deleteLater()
