#!/usr/bin/env python3
"""
Primary Flight Display (PFD) Widget for Xsens MTi AHRS data.
  - Attitude indicator (sky/ground, pitch ladder with 2.5° fine marks, bank arc)
  - Speed tape (scrolling, left side, color bands, rolling pointer)
  - Altitude tape (scrolling, right side, rolling pointer)
  - Heading tape (bottom, scrolling, cardinal/numeric labels)
  - Vertical Speed Indicator (far right, with value readout)
  - Flight Path Vector (FPV / velocity vector)
  - Aircraft reference symbol
  - Fail flags (red X) for invalid data
"""

import math
import time
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QRectF, QPointF, QRect
from PyQt6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QPolygonF,
    QPainterPath, QFontMetrics,
)
from sensors import SensorData


# ─────────── Color Palette ───────────
SKY      = QColor(0x4C, 0xA3, 0xDD)
GROUND   = QColor(0xB9, 0x7A, 0x56)
PANEL_BG = QColor(0x0A, 0x0C, 0x10)
FG       = QColor(255, 255, 255)
FG_DIM   = QColor(180, 180, 180)
YELLOW   = QColor(255, 200, 0)
GREEN    = QColor(0, 200, 0)
RED      = QColor(255, 50, 50)
CYAN     = QColor(0, 220, 220)
TAPE_BG  = QColor(20, 22, 30, 200)
POINTER_BG = QColor(30, 32, 40, 240)

# Speed tape V-speeds (knots) — adjust for your aircraft.
# Set SPD_BANDS_ENABLED = False to hide bands.
SPD_BANDS_ENABLED = True
SPD_VSO = 40    # stall speed, flaps full (bottom of white arc)
SPD_VS1 = 48    # stall speed, clean (bottom of green arc)
SPD_VFE = 85    # max flap extended (top of white arc)
SPD_VNO = 129   # max structural cruise (top of green arc)
SPD_VNE = 163   # never exceed (red line)


class PFDWidget(QWidget):
    """Full PFD rendered via QPainter.
    Displays AHRS data directly — no additional filtering."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(800, 600)
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent)

        # Units: False = US (knots, feet, ft/min), True = metric (km/h, meters, m/s)
        self._metric = False

        # Flight data
        self._roll = 0.0
        self._pitch = 0.0
        self._heading = 0.0
        self._speed = 0.0
        self._altitude = 0.0
        self._vsi = 0.0
        self._fpv_x = 0.0
        self._fpv_y = 0.0
        self._fpv_visible = False

        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._yaw_raw = 0.0

        # Heading bug (selected heading)
        self._hdg_bug = 0.0  # degrees, 0-360

        # Data validity flags — True once first valid sample arrives.
        self._valid_att = False    # roll/pitch (accelerometer)
        self._valid_hdg = False    # heading (EKF yaw)
        self._valid_spd = False    # speed (velocity)
        self._valid_alt = False    # altitude (baro or GPS)
        self._valid_vsi = False    # vertical speed

        # Attitude source: "accel" = raw accelerometer, "ekf" = EKF orientation
        self._att_source = "accel"

        # Roll/pitch from accelerometer (gravity vector) — drift-free.
        # Light low-pass filter (alpha per sample) to smooth accel noise.
        # The Xsens MTi-G EKF drifts ~0.7°/s without GPS, so we bypass it
        # entirely for roll/pitch and use the accelerometer directly.
        self._lp_alpha = 0.85  # per-sample smoothing (0=raw accel, 1=frozen)

        # Speed low-pass + squelch — GPS velocity jitter through hypot()
        # produces a persistently positive Rayleigh-distributed noise floor.
        # Real air data computers filter and squelch below a minimum.
        self._spd_lp_alpha = 0.85  # same smoothing constant as attitude
        self._spd_squelch = 0.5    # m/s — below this, display zero

        # Display-side smoothing for rolling drums — prevents the digits
        # from jumping faster than the eye can track.
        self._disp_speed = 0.0
        self._disp_alt = 0.0
        self._disp_alpha = 0.70  # per-frame smoothing (0=raw, 1=frozen)

        # Slip/skid (lateral acceleration, filtered)
        self._slip = 0.0       # normalized: -1..+1 range
        self._slip_bias = 0.0  # zeroed during AHRS align

        # Z-key bias
        self._bias_roll = 0.0
        self._bias_pitch = 0.0

        # AHRS alignment phase
        self._align_samples = 0
        self._align_target = 90       # ~3 sec at 30 Hz
        self._align_done = False
        self._auto_zero_on_start = True  # zero attitude after alignment
        self._alt_source = "baro"  # "baro" or "gnss"
        self._vsi_source = "gnss"  # "gnss" or "baro"

        # Baro VSI: differentiate LP-filtered barometric altitude.
        # Pre-filter altitude to suppress pressure sensor noise before
        # differentiation (which amplifies noise).
        self._baro_alt_filtered = None  # LP-filtered baro altitude
        self._baro_alt_lp_alpha = 0.95  # altitude pre-filter (suppress noise)
        self._prev_baro_alt_f = None    # previous filtered altitude for derivative
        self._prev_baro_time = None
        self._baro_vsi = 0.0
        self._baro_vsi_alpha = 0.985  # heavy post-filter on VSI (~2s time constant at 30 Hz)

        # GNSS fix status (derived from available data)
        self._gnss_nsv_used = 0   # SVs used in solution (bGPS field = numSV)
        self._gnss_nsv_visible = 0  # SVs with signal (from GPS status channels)
        self._gnss_has_pos = False  # valid position received

    def set_data(self, data: SensorData):
        """Update attitude, heading, speed, altitude from sensor data."""
        if self._att_source == "accel" and data.acc:
            # Roll/pitch from accelerometer gravity vector (drift-free).
            self._valid_att = True
            ax, ay, az = data.acc
            accel_roll = math.degrees(math.atan2(ay, az))
            accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
            a = self._lp_alpha
            self._roll = a * self._roll + (1.0 - a) * (accel_roll - self._bias_roll)
            self._pitch = a * self._pitch + (1.0 - a) * (accel_pitch - self._bias_pitch)
        elif self._att_source == "ekf" and data.roll_deg is not None and data.pitch_deg is not None:
            # Roll/pitch from EKF (fused IMU/GPS, may drift without GPS fix).
            self._valid_att = True
            a = self._lp_alpha
            self._roll = a * self._roll + (1.0 - a) * (data.roll_deg - self._bias_roll)
            self._pitch = a * self._pitch + (1.0 - a) * (data.pitch_deg - self._bias_pitch)

        # Slip/skid always from accelerometer
        if data.acc:
            ax, ay, az = data.acc
            a = self._lp_alpha
            g = math.sqrt(ax * ax + ay * ay + az * az)
            slip_raw = (ay / max(g, 0.1)) - self._slip_bias
            self._slip = a * self._slip + (1.0 - a) * max(-1.0, min(1.0, slip_raw * 5.0))

        # Heading: EKF only (accelerometer can't provide heading)
        if data.yaw_deg is not None:
            self._valid_hdg = True
            self._heading = data.yaw_deg % 360.0
            self._yaw_raw = data.yaw_deg

        if data.vel:
            self._valid_spd = True
            vx, vy, vz = data.vel
            self._vx, self._vy, self._vz = vx, vy, vz
            raw_spd = math.hypot(vx, vy)
            # Squelch before filter — kill noise at the source so it
            # can't accumulate in the integrator.
            if raw_spd < self._spd_squelch:
                raw_spd = 0.0
            a = self._spd_lp_alpha
            self._speed = a * self._speed + (1.0 - a) * raw_spd
            gnss_vsi = -vz
            if self._vsi_source == "gnss":
                self._valid_vsi = True
                self._vsi = gnss_vsi
        elif data.speed_ms is not None:
            self._valid_spd = True
            raw_spd = data.speed_ms
            if raw_spd < self._spd_squelch:
                raw_spd = 0.0
            a = self._spd_lp_alpha
            self._speed = a * self._speed + (1.0 - a) * raw_spd

        if self._alt_source == "gnss":
            if data.pos_alt is not None:
                self._valid_alt = True
                self._altitude = data.pos_alt
            elif data.baro_alt_m is not None:
                self._valid_alt = True
                self._altitude = data.baro_alt_m
        else:  # "baro"
            if data.baro_alt_m is not None:
                self._valid_alt = True
                self._altitude = data.baro_alt_m
            elif data.pos_alt is not None:
                self._valid_alt = True
                self._altitude = data.pos_alt

        # Baro-derived VSI: two-stage filter then differentiate.
        # Stage 1: LP filter raw baro altitude to suppress sensor noise.
        # Stage 2: differentiate filtered altitude, then LP the result.
        if data.baro_alt_m is not None:
            now = time.monotonic()
            # Pre-filter altitude
            if self._baro_alt_filtered is None:
                self._baro_alt_filtered = data.baro_alt_m
            else:
                a1 = self._baro_alt_lp_alpha
                self._baro_alt_filtered = a1 * self._baro_alt_filtered + (1.0 - a1) * data.baro_alt_m
            # Differentiate filtered altitude
            if self._prev_baro_alt_f is not None and self._prev_baro_time is not None:
                dt = now - self._prev_baro_time
                if dt > 0.001:
                    raw_baro_vsi = (self._baro_alt_filtered - self._prev_baro_alt_f) / dt
                    a2 = self._baro_vsi_alpha
                    self._baro_vsi = a2 * self._baro_vsi + (1.0 - a2) * raw_baro_vsi
                    if self._vsi_source == "baro":
                        self._valid_vsi = True
                        self._vsi = self._baro_vsi
            self._prev_baro_alt_f = self._baro_alt_filtered
            self._prev_baro_time = now

        # AHRS alignment: count valid attitude samples
        if not self._align_done and self._valid_att:
            self._align_samples += 1
            if self._align_samples >= self._align_target:
                self._align_done = True
                # Capture slip bias (current filtered slip is the stationary offset)
                self._slip_bias += self._slip / 5.0  # undo the *5.0 gain
                self._slip = 0.0
                if self._auto_zero_on_start:
                    self.zero_attitude()

        # FPV: hidden when stationary
        hs = math.hypot(self._vx, self._vy)
        if hs > 5.0:
            self._fpv_visible = True
            self._fpv_y = math.degrees(math.atan2(-self._vz, hs))
            track = math.degrees(math.atan2(self._vy, self._vx))
            self._fpv_x = ((track - self._yaw_raw + 180.0) % 360.0) - 180.0
        else:
            self._fpv_visible = False
            self._fpv_x = 0.0
            self._fpv_y = 0.0

        # GNSS status: bGPS = numSV (satellites used in solution)
        if data.rawgps and data.rawgps.bGPS is not None:
            self._gnss_nsv_used = data.rawgps.bGPS
        if data.rawgps and data.rawgps.lat_deg is not None:
            # Position is valid if coordinates are non-zero
            self._gnss_has_pos = (data.rawgps.lat_deg != 0.0 or data.rawgps.lon_deg != 0.0)
        if data.gps_status:
            self._gnss_nsv_visible = sum(1 for ch in data.gps_status.channels if ch.cnr > 0)

        self.update()

    def zero_attitude(self):
        """Z key: capture current accel-derived roll/pitch as level reference."""
        self._bias_roll += self._roll
        self._bias_pitch += self._pitch
        self._roll = 0.0
        self._pitch = 0.0

    def reset_zero(self):
        """R key: clear the level bias."""
        self._bias_roll = 0.0
        self._bias_pitch = 0.0

    def toggle_units(self):
        """U key: toggle US (knots/feet) vs metric (km/h/meters)."""
        self._metric = not self._metric

    def set_hdg_bug(self, deg):
        """Set heading bug to a specific value."""
        self._hdg_bug = deg % 360

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Z:
            self.zero_attitude()
        elif event.key() == Qt.Key.Key_U:
            self.toggle_units()
        elif event.key() == Qt.Key.Key_R:
            self.reset_zero()
        elif event.key() == Qt.Key.Key_H:
            # Sync heading bug to current heading
            self._hdg_bug = self._heading
        elif event.key() in (Qt.Key.Key_Plus, Qt.Key.Key_Equal):
            self._hdg_bug = (self._hdg_bug + 1) % 360
        elif event.key() == Qt.Key.Key_Minus:
            self._hdg_bug = (self._hdg_bug - 1) % 360
        else:
            super().keyPressEvent(event)

    # ─────────── Layout geometry ───────────

    def _layout(self):
        """Compute sub-regions based on widget size.
        Attitude and tapes use full height; compass rose overlays the lower third."""
        w, h = self.width(), self.height()
        spd_w = max(80, int(w * 0.085))
        alt_w = max(105, int(w * 0.105))
        vsi_w = max(40, int(w * 0.035))
        att_x = spd_w
        att_w = w - spd_w - alt_w - vsi_w
        # HSI overlays lower third
        hsi_top = int(h * 0.65)
        hsi_h = h - hsi_top
        return {
            'att': QRect(att_x, 0, att_w, h),
            'spd': QRect(0, 0, spd_w, h),
            'alt': QRect(att_x + att_w, 0, alt_w, h),
            'vsi': QRect(att_x + att_w + alt_w, 0, vsi_w, h),
            'hsi': QRect(spd_w, hsi_top, att_w, hsi_h),
        }

    # ─────────── Paint ───────────

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), PANEL_BG)

        layout = self._layout()
        self._draw_attitude(p, layout['att'])
        self._draw_compass_rose(p, layout['hsi'])  # overlays lower attitude
        self._draw_speed_tape(p, layout['spd'])
        self._draw_altitude_tape(p, layout['alt'])
        self._draw_vsi(p, layout['vsi'])

        # AHRS alignment message
        if self._valid_att and not self._align_done:
            self._draw_align_message(p, layout['att'])

        # Fail flags — red X over invalid instruments
        if not self._valid_att:
            self._draw_fail_flag(p, layout['att'], "ATT")
        if not self._valid_spd:
            self._draw_fail_flag(p, layout['spd'], "SPD")
        if not self._valid_alt:
            self._draw_fail_flag(p, layout['alt'], "ALT")
        if not self._valid_vsi:
            self._draw_fail_flag(p, layout['vsi'], "V/S")
        if not self._valid_hdg:
            self._draw_fail_flag(p, layout['hsi'], "HDG")

        self._draw_gnss_status(p, layout['hsi'])

        p.end()

    # ─────────── GNSS fix status ───────────

    def _draw_gnss_status(self, p: QPainter, hsi_rect: QRect):
        """Draw GNSS fix status in the lower-left corner of the HSI area."""
        nsv = self._gnss_nsv_used
        has_pos = self._gnss_has_pos
        nvis = self._gnss_nsv_visible

        if has_pos and nsv >= 4:
            label = "3D FIX"
            color = GREEN
        elif has_pos and nsv > 0:
            label = "2D FIX"
            color = YELLOW
        elif nvis > 0 or nsv > 0:
            label = "SEARCHING"
            color = YELLOW
        else:
            label = "NO FIX"
            color = RED

        p.save()
        font = p.font()
        font.setPixelSize(max(10, int(hsi_rect.height() * 0.08)))
        font.setBold(True)
        p.setFont(font)

        text = f"GNSS: {label}"
        fm = QFontMetrics(font)
        tw = fm.horizontalAdvance(text)
        th = fm.height()
        pad = 4

        x = hsi_rect.left() + 4
        y = hsi_rect.bottom() - th - pad * 2 - 4

        p.setPen(QPen(QColor(160, 160, 160), 1))
        p.setBrush(QColor(20, 22, 28, 160))
        p.drawRect(x, y, tw + pad * 2, th + pad * 2)

        p.setPen(QPen(color))
        p.drawText(x + pad, y + pad + fm.ascent(), text)
        p.restore()

    # ─────────── Fail flag ───────────

    def _draw_fail_flag(self, p: QPainter, r: QRect, label: str):
        """Draw red X with label over an invalid instrument region."""
        p.save()
        p.setClipRect(r)
        # Semi-transparent red overlay
        p.fillRect(r, QColor(80, 0, 0, 120))
        # Red X
        pen = QPen(RED, 3.0)
        p.setPen(pen)
        m = 8  # margin
        p.drawLine(QPointF(r.left() + m, r.top() + m),
                   QPointF(r.right() - m, r.bottom() - m))
        p.drawLine(QPointF(r.right() - m, r.top() + m),
                   QPointF(r.left() + m, r.bottom() - m))
        # Label box in center
        font = QFont("Monospace", max(10, int(min(r.width(), r.height()) * 0.12)))
        font.setBold(True)
        p.setFont(font)
        fm = QFontMetrics(font)
        tw = fm.horizontalAdvance(label)
        th = fm.height()
        bx = r.center().x() - tw / 2 - 6
        by = r.center().y() - th / 2 - 3
        p.fillRect(QRectF(bx, by, tw + 12, th + 6), QColor(0, 0, 0, 200))
        p.setPen(QPen(RED, 1.5))
        p.drawRect(QRectF(bx, by, tw + 12, th + 6))
        p.drawText(QPointF(bx + 6, by + 3 + fm.ascent()), label)
        p.restore()

    def _draw_align_message(self, p: QPainter, r: QRect):
        """Yellow AHRS ALIGN message with progress bar."""
        p.save()
        font = QFont("Monospace", max(12, int(r.height() * 0.028)))
        font.setBold(True)
        p.setFont(font)
        fm = QFontMetrics(font)

        txt = "AHRS ALIGN: Keep Wings Level"
        tw = fm.horizontalAdvance(txt)
        th = fm.height()
        cx, cy = r.center().x(), r.center().y() + r.height() * 0.25

        # Background box
        pad_x, pad_y = 16, 8
        bx = cx - tw / 2 - pad_x
        by = cy - th / 2 - pad_y
        bw = tw + pad_x * 2
        bh = th + pad_y * 2 + 10  # extra for progress bar
        p.fillRect(QRectF(bx, by, bw, bh), QColor(0, 0, 0, 200))
        p.setPen(QPen(YELLOW, 1.5))
        p.drawRect(QRectF(bx, by, bw, bh))

        # Text
        p.drawText(QPointF(cx - tw / 2, cy - th / 2 + fm.ascent()), txt)

        # Progress bar
        progress = min(1.0, self._align_samples / max(1, self._align_target))
        bar_y = cy + th / 2 + 4
        bar_h = 4
        bar_w = bw - pad_x * 2
        bar_x = bx + pad_x
        p.fillRect(QRectF(bar_x, bar_y, bar_w, bar_h), QColor(60, 60, 60))
        p.fillRect(QRectF(bar_x, bar_y, bar_w * progress, bar_h), YELLOW)

        p.restore()

    # ─────────── Attitude indicator ───────────

    def _draw_attitude(self, p: QPainter, r: QRect):
        cx, cy = r.center().x(), r.center().y()
        size = min(r.width(), r.height())
        pitch_px_per_deg = size / 55.0

        p.save()
        p.setClipRect(r)
        p.translate(cx, cy)

        # Roll rotation: negate because right bank → horizon tilts left
        p.rotate(-self._roll)

        # Pitch shift: negate so nose-up (positive pitch) moves horizon down
        pitch_shift = -self._pitch * pitch_px_per_deg

        # Sky & ground
        big = size * 3
        p.fillRect(QRectF(-big, -big + pitch_shift, big * 2, big), SKY)
        p.fillRect(QRectF(-big, pitch_shift, big * 2, big), GROUND)

        # Horizon line
        p.setPen(QPen(FG, 2.5))
        p.drawLine(QPointF(-big, pitch_shift), QPointF(big, pitch_shift))

        # Pitch ladder
        self._draw_pitch_ladder(p, pitch_px_per_deg, pitch_shift, size)

        p.restore()

        # Bank arc & pointer (screen space)
        self._draw_bank_arc(p, r)
        # Aircraft reference symbol (fixed center)
        self._draw_aircraft_symbol(p, cx, cy)
        # Flight Path Vector — only drawn when airborne/moving
        if self._fpv_visible:
            self._draw_fpv(p, r, pitch_px_per_deg)

    def _draw_pitch_ladder(self, p: QPainter, px_per_deg, pitch_shift, size):
        """Pitch ladder: 10° major with labels, 5° minor, 2.5° minor
        between ±20° of horizon."""
        font = QFont("Monospace", max(9, int(size * 0.022)))
        font.setBold(True)
        p.setFont(font)
        fm = QFontMetrics(font)

        # Build list of pitch marks: 10° major, 5° standard minor,
        # 2.5° fine minor between -20° and +20°
        marks = []
        for deg10 in range(-80, 81, 10):
            if deg10 != 0:
                marks.append((deg10, 'major'))
        for deg5 in range(-85, 86, 5):
            if deg5 % 10 != 0 and deg5 != 0:
                marks.append((deg5, 'minor'))
        for q in range(-18, 19):          # ±17.5° in 2.5° steps
            deg25 = q * 2.5
            if deg25 % 5 != 0 and abs(deg25) <= 20:
                marks.append((deg25, 'fine'))

        for deg, kind in marks:
            y = pitch_shift + deg * px_per_deg
            if abs(y) > size * 0.7:
                continue

            if kind == 'major':
                half_w = size * 0.12
                p.setPen(QPen(FG, 1.8))
            elif kind == 'minor':
                half_w = size * 0.06
                p.setPen(QPen(FG, 1.2))
            else:  # fine
                half_w = size * 0.03
                p.setPen(QPen(FG, 1.0))

            if deg < 0:
                pen = p.pen()
                pen.setStyle(Qt.PenStyle.DashLine)
                p.setPen(pen)

            p.drawLine(QPointF(-half_w, y), QPointF(half_w, y))

            if kind == 'major':
                txt = f"{abs(int(deg))}"
                tw = fm.horizontalAdvance(txt)
                p.setPen(QPen(FG, 1))
                p.drawText(QPointF(half_w + 6, y + fm.ascent() / 2.5), txt)
                p.drawText(QPointF(-half_w - 6 - tw, y + fm.ascent() / 2.5), txt)

    def _draw_bank_arc(self, p: QPainter, r: QRect):
        cx = r.center().x()
        # Arc sits near the top of the attitude area, large radius
        radius = r.width() * 0.42
        arc_cy = r.top() + radius + 14  # push arc center down just enough to show the top

        p.save()
        p.setClipRect(r)

        # Arc ±60 degrees from top
        p.setPen(QPen(FG, 1.5))
        arc_rect = QRectF(cx - radius, arc_cy - radius, radius * 2, radius * 2)
        p.drawArc(arc_rect, 30 * 16, 120 * 16)

        # Tick marks at 10, 20, 30, 45, 60°
        for ang in [-60, -45, -30, -20, -10, 10, 20, 30, 45, 60]:
            if abs(ang) in (30, 60):
                tick_len, pen_w = radius * 0.06, 2.0
            elif abs(ang) == 45:
                tick_len, pen_w = radius * 0.05, 1.5
            else:
                tick_len, pen_w = radius * 0.04, 1.5

            rad = math.radians(90 + ang)
            x1 = cx + radius * math.cos(rad)
            y1 = arc_cy - radius * math.sin(rad)
            x2 = cx + (radius - tick_len) * math.cos(rad)
            y2 = arc_cy - (radius - tick_len) * math.sin(rad)
            p.setPen(QPen(FG, pen_w))
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        # Top index triangle (fixed at 0°, small filled white triangle)
        tri_h = radius * 0.04
        tri_w = radius * 0.03
        top_y = arc_cy - radius
        tri = QPolygonF([
            QPointF(cx, top_y),
            QPointF(cx - tri_w, top_y - tri_h),
            QPointF(cx + tri_w, top_y - tri_h),
        ])
        p.setPen(QPen(FG, 1.5))
        p.setBrush(QBrush(FG))
        p.drawPolygon(tri)

        # Moving bank pointer — open triangle riding inside the arc
        bank_ang = max(-60.0, min(60.0, self._roll))
        ptr_h = radius * 0.05
        ptr_w = radius * 0.035
        tip_y = -radius
        base_y = tip_y + ptr_h

        p.save()
        p.translate(cx, arc_cy)
        p.rotate(-bank_ang)

        # Bank pointer — filled white triangle
        tri_pts = QPolygonF([
            QPointF(0, tip_y),
            QPointF(-ptr_w, base_y),
            QPointF(ptr_w, base_y),
        ])
        p.setPen(QPen(FG, 1.5))
        p.setBrush(QBrush(FG))
        p.drawPolygon(tri_pts)

        # Slip/skid indicator — bar wider than the pointer triangle
        bar_w = ptr_w * 3.0
        bar_h = radius * 0.015
        bar_top = base_y + radius * 0.008
        max_disp = ptr_w * 2.5
        slip_offset = max(-1.0, min(1.0, self._slip)) * max_disp
        p.setPen(QPen(FG, 1.0))
        p.setBrush(QBrush(FG))
        p.drawRect(QRectF(-bar_w / 2 + slip_offset, bar_top, bar_w, bar_h))

        p.restore()
        p.restore()

    def _draw_aircraft_symbol(self, p: QPainter, cx, cy):
        p.save()
        wing, gap, tick = 45, 8, 10
        p.setPen(QPen(YELLOW, 3.5))
        p.drawLine(QPointF(cx - gap, cy), QPointF(cx - wing, cy))
        p.drawLine(QPointF(cx - gap, cy), QPointF(cx - gap, cy + tick))
        p.drawLine(QPointF(cx + gap, cy), QPointF(cx + wing, cy))
        p.drawLine(QPointF(cx + gap, cy), QPointF(cx + gap, cy + tick))
        p.setBrush(QBrush(YELLOW))
        p.drawEllipse(QPointF(cx, cy), 3, 3)
        p.restore()

    def _draw_fpv(self, p: QPainter, r: QRect, px_per_deg):
        cx, cy = r.center().x(), r.center().y()
        fpv_x_px = self._fpv_x * px_per_deg
        fpv_y_px = -self._fpv_y * px_per_deg

        fx = cx + max(-r.width() * 0.4, min(r.width() * 0.4, fpv_x_px))
        fy = cy + max(-r.height() * 0.4, min(r.height() * 0.4, fpv_y_px))

        p.save()
        p.setClipRect(r)
        p.setPen(QPen(GREEN, 2.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        rad = 8
        p.drawEllipse(QPointF(fx, fy), rad, rad)
        wing = 22
        p.drawLine(QPointF(fx - rad, fy), QPointF(fx - rad - wing, fy))
        p.drawLine(QPointF(fx + rad, fy), QPointF(fx + rad + wing, fy))
        p.drawLine(QPointF(fx, fy - rad), QPointF(fx, fy - rad - 12))
        p.restore()

    # ─────────── Rolling drum digits ───────────

    def _draw_drum_pointer(self, p: QPainter, value: float, num_digits: int,
                           cy: float, font_size: int, box_rect: QRectF,
                           x_text: float):
        """Rolling drum digit display — right-aligned, no leading zeros.

        Odometer-style: the ones digit scrolls freely; higher digits only
        begin to roll when the digit directly below passes through 9→0,
        so left-hand digits stay crisp and readable at all times.
        """
        font = QFont("Monospace", font_size)
        font.setBold(True)
        fm = QFontMetrics(font)
        char_w = fm.horizontalAdvance("0")
        digit_h = fm.height()
        half_box = box_rect.height() / 2.0
        val = abs(value)

        # Right edge of the digit area
        x_right = x_text + num_digits * char_w

        p.save()
        p.setFont(font)
        p.setPen(QPen(FG, 1.5))
        baseline = cy + (fm.ascent() - fm.descent()) / 2.0

        # Pre-compute per-position digit and cascade fraction.
        # Ones scroll freely; each higher position only scrolls when the
        # position below is at 9 and rolling over (mechanical odometer).
        digits = []
        prev_frac = 0.0
        for position in range(num_digits):
            place = 10 ** position
            digit_val = (val / place) % 10.0
            digit = int(digit_val) % 10

            if position == 0:
                frac = digit_val - int(digit_val)
            else:
                # Only scroll when the digit below is 9 and rolling
                lower_digit = digits[position - 1][0]
                frac = prev_frac if lower_digit == 9 else 0.0

            # Smoothstep easing for fluid motion.
            frac = frac * frac * (3.0 - 2.0 * frac)
            digits.append((digit, frac))
            prev_frac = frac

        for position, (digit, frac) in enumerate(digits):
            place = 10 ** position

            # Suppress leading zeros (never suppress ones digit).
            if position > 0 and val < place:
                continue

            next_d = (digit + 1) % 10
            prev_d = (digit - 1) % 10

            # Right-aligned: ones at x_right - char_w, tens at x_right - 2*char_w, …
            sx = x_right - (position + 1) * char_w

            p.save()
            p.setClipRect(QRectF(sx - 1, cy - half_box, char_w + 2, half_box * 2))

            y_shift = -frac * digit_h
            p.drawText(QPointF(sx, baseline + y_shift), str(digit))
            p.drawText(QPointF(sx, baseline + y_shift + digit_h), str(next_d))
            p.drawText(QPointF(sx, baseline + y_shift - digit_h), str(prev_d))

            p.restore()

        # Negative sign (altitude below sea level)
        if value < 0:
            sign_x = x_right - (num_digits + 1) * char_w
            p.drawText(QPointF(max(box_rect.left(), sign_x), baseline), "-")

        p.restore()

    # ─────────── Speed tape ───────────

    def _draw_speed_tape(self, p: QPainter, r: QRect):
        p.save()

        if self._metric:
            speed_disp = self._speed * 3.6   # m/s → km/h
            major, minor = 10, 5             # tick spacing
            span = 80.0                      # visible range
            gs_unit = "KM/H"
        else:
            speed_disp = self._speed * 1.94384  # m/s → knots
            major, minor = 10, 5
            span = 60.0                      # 60 kt visible
            gs_unit = "KT"

        # Reserve bottom strip for GS readout
        gs_strip_h = max(20, int(r.height() * 0.045))
        tape_r = QRect(r.left(), r.top(), r.width(), r.height() - gs_strip_h)
        gs_r = QRect(r.left(), tape_r.bottom(), r.width(), gs_strip_h)

        p.setClipRect(tape_r)
        p.fillRect(tape_r, TAPE_BG)

        cy = tape_r.center().y()
        tape_w = tape_r.width()
        px_per_unit = tape_r.height() / span

        # ── V-speed color bands (right edge of tape) ──
        # V-speeds are stored in knots — convert to display units
        spd_k = 3.6 / 1.94384 if self._metric else 1.0  # kt → km/h or kt → kt
        v_vso = SPD_VSO * spd_k
        v_vs1 = SPD_VS1 * spd_k
        v_vfe = SPD_VFE * spd_k
        v_vno = SPD_VNO * spd_k
        v_vne = SPD_VNE * spd_k

        if SPD_BANDS_ENABLED:
            bw = max(4, int(tape_w * 0.08))

            def _band(v_lo, v_hi, color):
                yt = cy - (v_hi - speed_disp) * px_per_unit
                yb = cy - (v_lo - speed_disp) * px_per_unit
                yt = max(tape_r.top(), min(tape_r.bottom(), yt))
                yb = max(tape_r.top(), min(tape_r.bottom(), yb))
                if yb > yt:
                    p.fillRect(QRectF(tape_w - bw, yt, bw, yb - yt), color)

            # Draw white first, then green on top (green overwrites overlap)
            _band(v_vso, v_vfe, FG)
            _band(v_vs1, v_vno, GREEN)
            _band(v_vno, v_vne, YELLOW)

            # Vne red line
            y_vne = cy - (v_vne - speed_disp) * px_per_unit
            if tape_r.top() < y_vne < tape_r.bottom():
                p.setPen(QPen(RED, 3.0))
                p.drawLine(QPointF(tape_w - bw - 4, y_vne), QPointF(tape_w, y_vne))

            # 45° red/white barber-pole above Vne (overspeed)
            # Stripes are anchored to the tape so they scroll with speed.
            y_vne_clamp = max(tape_r.top(), min(tape_r.bottom(), y_vne))
            if y_vne_clamp > tape_r.top():
                p.save()
                p.setClipRect(QRectF(tape_w - bw, tape_r.top(), bw, y_vne_clamp - tape_r.top()))
                stripe = bw  # stripe width = band width → 45° diagonals
                # Phase offset: anchor stripes to speed so they move with tape
                phase = (speed_disp * px_per_unit) % (stripe * 2)
                # Draw diagonal stripes by filling angled parallelograms
                y_start = tape_r.top() - bw - stripe * 2 + phase
                while y_start < y_vne_clamp:
                    for color, offset in [(RED, 0), (FG, stripe)]:
                        y0 = y_start + offset
                        poly = QPolygonF([
                            QPointF(tape_w - bw, y0),
                            QPointF(tape_w, y0 - bw),
                            QPointF(tape_w, y0 - bw + stripe),
                            QPointF(tape_w - bw, y0 + stripe),
                        ])
                        p.setPen(Qt.PenStyle.NoPen)
                        p.setBrush(QBrush(color))
                        p.drawPolygon(poly)
                    y_start += stripe * 2
                p.restore()

        # ── Tick marks and labels ──
        tick_font_sz = max(8, int(tape_w * 0.12))
        font = QFont("Monospace", tick_font_sz)
        p.setFont(font)
        fm = QFontMetrics(font)
        tick_l = min(18, int(tape_w * 0.20))
        tick_s = min(8, int(tape_w * 0.09))

        base = int(speed_disp / major) * major
        for i in range(-8, 9):
            for sub in range(0, major, minor):
                v = base + i * major + sub
                if v < 0:
                    continue
                y = cy - (v - speed_disp) * px_per_unit
                if y < tape_r.top() - 10 or y > tape_r.bottom() + 10:
                    continue
                if v % major == 0:
                    p.setPen(QPen(FG, 1.5))
                    p.drawLine(QPointF(tape_w - tick_l, y), QPointF(tape_w, y))
                    txt = f"{v:.0f}"
                    p.drawText(QPointF(tape_w - tick_l - fm.horizontalAdvance(txt) - 3,
                                       y + fm.ascent() / 2.5), txt)
                else:
                    p.setPen(QPen(FG_DIM, 1))
                    p.drawLine(QPointF(tape_w - tick_s, y), QPointF(tape_w, y))

        # ── Pointer box with rolling digits ──
        a = self._disp_alpha
        self._disp_speed = a * self._disp_speed + (1.0 - a) * speed_disp
        drum_spd = self._disp_speed

        arrow_w = 10
        margin = 6
        avail_w = tape_w - arrow_w - margin
        ptr_font_sz = max(9, int(avail_w / 3 * 1.4))
        ptr_font = QFont("Monospace", ptr_font_sz)
        ptr_font.setBold(True)
        ptr_fm = QFontMetrics(ptr_font)
        while ptr_fm.horizontalAdvance("000") > avail_w and ptr_font_sz > 8:
            ptr_font_sz -= 1
            ptr_font = QFont("Monospace", ptr_font_sz)
            ptr_font.setBold(True)
            ptr_fm = QFontMetrics(ptr_font)

        box_h = ptr_fm.height() + 8
        overspeed = SPD_BANDS_ENABLED and speed_disp >= v_vne

        path = QPainterPath()
        path.moveTo(tape_w, cy)
        path.lineTo(tape_w - arrow_w, cy - box_h / 2)
        path.lineTo(2, cy - box_h / 2)
        path.lineTo(2, cy + box_h / 2)
        path.lineTo(tape_w - arrow_w, cy + box_h / 2)
        path.closeSubpath()
        p.fillPath(path, QColor(80, 0, 0) if overspeed else POINTER_BG)
        p.setPen(QPen(RED if overspeed else FG, 1.5))
        p.drawPath(path)

        digits_w = ptr_fm.horizontalAdvance("0") * 3
        x_text = 2 + (avail_w - digits_w) / 2
        box_r = QRectF(2, cy - box_h / 2, tape_w - arrow_w - 2, box_h)
        self._draw_drum_pointer(p, drum_spd, 3, cy, ptr_font_sz, box_r, x_text)

        # GS readout box at the bottom of the speed tape
        p.setClipping(False)

        # Dynamically scale font to fit "GS 000 KM/H" within the box width
        avail_w = gs_r.width() - 6  # padding
        test_text = f"GS 000 {gs_unit}"
        gs_font_sz = max(5, int(gs_strip_h * 0.55))
        while gs_font_sz > 5:
            test_font = QFont("Monospace", gs_font_sz)
            test_font.setBold(True)
            if QFontMetrics(test_font).horizontalAdvance(test_text) <= avail_w:
                break
            gs_font_sz -= 1

        gs_font = QFont("Monospace", gs_font_sz)
        gs_font.setBold(True)
        gs_fm = QFontMetrics(gs_font)

        unit_font_sz = max(4, int(gs_font_sz * 0.7))
        unit_font = QFont("Monospace", unit_font_sz)
        unit_fm = QFontMetrics(unit_font)

        p.setPen(QPen(QColor(160, 160, 160), 1))
        p.setBrush(QColor(20, 22, 28, 160))
        p.drawRect(gs_r)

        baseline_y = gs_r.top() + gs_fm.ascent() + (gs_strip_h - gs_fm.height()) // 2
        pad = 3
        x = gs_r.left() + pad

        # "GS" label
        p.setPen(QPen(FG))
        p.setFont(gs_font)
        p.drawText(x, baseline_y, "GS")
        x += gs_fm.horizontalAdvance("GS") + gs_fm.horizontalAdvance(" ")

        # Fixed-width numeric value
        char_w = gs_fm.horizontalAdvance("0")
        spd_str = f"{int(round(speed_disp)):3d}"
        for ch in spd_str:
            if ch != ' ':
                p.drawText(x, baseline_y, ch)
            x += char_w

        # Unit with a small gap
        x += max(2, char_w // 4)
        p.setFont(unit_font)
        p.setPen(QPen(FG_DIM))
        p.drawText(x, baseline_y, gs_unit)
        p.restore()

    # ─────────── Altitude tape ───────────

    def _draw_altitude_tape(self, p: QPainter, r: QRect):
        p.save()
        p.setClipRect(r)
        p.fillRect(r, TAPE_BG)

        if self._metric:
            alt_disp = self._altitude            # meters
            major, minor = 50, 10                # tick spacing
            span = 300.0                         # visible range
            src = "GNSS" if self._alt_source == "gnss" else "ALT"
            label = f"{src} m"
        else:
            alt_disp = self._altitude * 3.28084  # feet
            major, minor = 100, 20
            span = 600.0
            src = "GNSS" if self._alt_source == "gnss" else "ALT"
            label = f"{src} ft"

        cy = r.center().y()
        tape_w = r.width()
        px_per_unit = r.height() / span
        x0 = r.left()

        tick_font_sz = max(8, int(tape_w * 0.10))
        font = QFont("Monospace", tick_font_sz)
        p.setFont(font)
        fm = QFontMetrics(font)
        p.setPen(QPen(FG, 1))
        tick_l = min(20, int(tape_w * 0.18))
        tick_s = min(10, int(tape_w * 0.09))

        base = int(alt_disp / major) * major
        for i in range(-8, 9):
            for sub in range(0, major, minor):
                v = base + i * major + sub
                y = cy - (v - alt_disp) * px_per_unit
                if y < r.top() - 10 or y > r.bottom() + 10:
                    continue
                if v % major == 0:
                    p.setPen(QPen(FG, 1.5))
                    p.drawLine(QPointF(x0, y), QPointF(x0 + tick_l, y))
                    p.drawText(QPointF(x0 + tick_l + 4, y + fm.ascent() / 2.5),
                               f"{v:.0f}")
                elif v % minor == 0:
                    p.setPen(QPen(FG_DIM, 1))
                    p.drawLine(QPointF(x0, y), QPointF(x0 + tick_s, y))

        # Display-smoothed value for the drum
        a = self._disp_alpha
        self._disp_alt = a * self._disp_alt + (1.0 - a) * alt_disp
        drum_alt = self._disp_alt

        # Pointer font: fit 5 chars within (tape_w - arrow - margins)
        arrow_w = 10
        margin = 8
        avail_w = tape_w - arrow_w - margin
        ptr_font_sz = max(9, int(avail_w / 5 * 1.45))
        ptr_font = QFont("Monospace", ptr_font_sz)
        ptr_font.setBold(True)
        ptr_fm = QFontMetrics(ptr_font)
        while ptr_fm.horizontalAdvance("00000") > avail_w and ptr_font_sz > 8:
            ptr_font_sz -= 1
            ptr_font = QFont("Monospace", ptr_font_sz)
            ptr_font.setBold(True)
            ptr_fm = QFontMetrics(ptr_font)

        box_h = ptr_fm.height() + 8

        path = QPainterPath()
        path.moveTo(x0, cy)
        path.lineTo(x0 + arrow_w, cy - box_h / 2)
        path.lineTo(x0 + tape_w - 2, cy - box_h / 2)
        path.lineTo(x0 + tape_w - 2, cy + box_h / 2)
        path.lineTo(x0 + arrow_w, cy + box_h / 2)
        path.closeSubpath()
        p.fillPath(path, POINTER_BG)
        p.setPen(QPen(FG, 1.5))
        p.drawPath(path)

        # Center 5 digits in the box
        digits_w = ptr_fm.horizontalAdvance("0") * 5
        x_text = x0 + arrow_w + (avail_w - digits_w) / 2
        box_r = QRectF(x0 + arrow_w, cy - box_h / 2, tape_w - arrow_w - 2, box_h)
        self._draw_drum_pointer(p, drum_alt, 5, cy, ptr_font_sz, box_r, x_text)

        small_font = QFont("Monospace", max(7, int(tape_w * 0.10)))
        p.setFont(small_font)
        p.setPen(QPen(FG_DIM, 1))
        p.drawText(QPointF(x0 + 4, r.bottom() - 4), label)
        p.restore()

    # ─────────── Vertical Speed Indicator ───────────

    def _draw_vsi(self, p: QPainter, r: QRect):
        p.save()
        p.setClipRect(r)
        p.fillRect(r, TAPE_BG)

        cy = r.center().y()
        h = r.height()
        vw = r.width()
        x0 = r.left()

        if self._metric:
            vsi_disp = self._vsi              # m/s
            max_val = 10.0
            ticks = [1, 2, 4, 6]
            label_ticks = {2: "2", 4: "4", 6: "6"}
        else:
            vsi_disp = self._vsi * 196.85     # ft/min
            max_val = 2000.0
            ticks = [100, 500, 1000, 1500, 2000]
            label_ticks = {500: ".5", 1000: "1", 1500: "1.5", 2000: "2"}

        # Non-linear scale: compress outer range
        def vsi_to_y(v):
            """Map VSI value to y, with compression above half-scale."""
            half = max_val / 2.0
            if abs(v) <= half:
                frac = v / half * 0.6     # inner 60% of half-height
            else:
                frac = 0.6 + (abs(v) - half) / (max_val - half) * 0.4
                if v < 0:
                    frac = -frac
            return cy - frac * (h * 0.45)

        # Zero line
        p.setPen(QPen(FG_DIM, 1))
        p.drawLine(QPointF(x0, cy), QPointF(x0 + vw, cy))

        # Tick marks and labels
        font = QFont("Monospace", max(6, int(vw * 0.20)))
        p.setFont(font)
        fm = QFontMetrics(font)

        for tv in ticks:
            for sign in (-1, 1):
                y = vsi_to_y(sign * tv)
                if r.top() < y < r.bottom():
                    is_major = tv in label_ticks
                    tw = 8 if is_major else 5
                    p.setPen(QPen(FG_DIM if not is_major else FG, 1))
                    p.drawLine(QPointF(x0, y), QPointF(x0 + tw, y))
                    if is_major:
                        txt = label_ticks[tv]
                        p.drawText(QPointF(x0 + tw + 2, y + fm.ascent() / 3), txt)

        # Pointer with value readout (digits appear when > 100 fpm)
        clamped = max(-max_val, min(max_val, vsi_disp))
        ptr_y = vsi_to_y(clamped)
        show_val = self._metric and abs(vsi_disp) >= 0.5 or \
                   not self._metric and abs(vsi_disp) >= 100

        if show_val:
            # Pointer box with value
            ptr_font = QFont("Monospace", max(6, int(vw * 0.18)))
            ptr_font.setBold(True)
            ptr_fm = QFontMetrics(ptr_font)
            if self._metric:
                vtxt = f"{vsi_disp:+.1f}"
            else:
                vtxt = f"{int(round(vsi_disp / 50) * 50):+d}"
            bx_w = vw - 4
            bx_h = ptr_fm.height() + 2
            bx_y = ptr_y - bx_h / 2
            p.fillRect(QRectF(x0 + 2, bx_y, bx_w, bx_h), POINTER_BG)
            p.setPen(QPen(FG, 1.0))
            p.drawRect(QRectF(x0 + 2, bx_y, bx_w, bx_h))
            p.setFont(ptr_font)
            p.drawText(QRectF(x0 + 2, bx_y, bx_w, bx_h),
                       Qt.AlignmentFlag.AlignCenter, vtxt)
        else:
            # Small triangle pointer
            tri_h = max(4, int(vw * 0.25))
            tri = QPolygonF([
                QPointF(x0, ptr_y),
                QPointF(x0 + tri_h, ptr_y - tri_h / 2),
                QPointF(x0 + tri_h, ptr_y + tri_h / 2),
            ])
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(FG))
            p.drawPolygon(tri)

        p.restore()

    # ─────────── Compass Rose (360° HSI) ───────────

    def _draw_compass_rose(self, p: QPainter, r: QRect):
        p.save()
        # No clip — heading box and HDG readout extend above the HSI rect

        cx = r.center().x()
        cy = r.center().y()
        hdg = self._heading
        radius = min(r.width(), r.height()) * 0.44

        cardinal = {0: "N", 90: "E", 180: "S", 270: "W"}

        # Dark semi-transparent background circle
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(10, 12, 18, 210))
        p.drawEllipse(QPointF(cx, cy), radius + 4, radius + 4)

        # Font for labels
        label_sz = max(9, int(radius * 0.12))
        font = QFont("Monospace", label_sz)
        font.setBold(True)
        fm = QFontMetrics(font)

        # ── Rotating compass card ──
        for deg in range(0, 360, 5):
            # Angle on screen: deg relative to heading, 0° = top
            ang = deg - hdg
            rad = math.radians(ang - 90)
            cos_a = math.cos(rad)
            sin_a = math.sin(rad)

            if deg % 10 == 0:
                # Major tick (inward from rim)
                r1 = radius
                r2 = radius - radius * 0.08
                p.setPen(QPen(FG, 1.5))
                p.drawLine(QPointF(cx + r1 * cos_a, cy + r1 * sin_a),
                           QPointF(cx + r2 * cos_a, cy + r2 * sin_a))

                # Labels every 30°
                if deg in cardinal:
                    txt = cardinal[deg]
                    color = CYAN
                elif deg % 30 == 0:
                    txt = f"{deg // 10}"
                    color = FG
                else:
                    txt = None

                if txt:
                    r_txt = radius - radius * 0.17
                    lx = cx + r_txt * cos_a
                    ly = cy + r_txt * sin_a
                    # Draw text upright: save, translate to label pos, draw centered
                    p.save()
                    p.translate(lx, ly)
                    p.setPen(QPen(color, 1.5))
                    p.setFont(font)
                    tw = fm.horizontalAdvance(txt)
                    p.drawText(QPointF(-tw / 2, fm.ascent() / 2.5), txt)
                    p.restore()
            else:
                # Minor tick every 5°
                r1 = radius
                r2 = radius - radius * 0.04
                p.setPen(QPen(FG_DIM, 1))
                p.drawLine(QPointF(cx + r1 * cos_a, cy + r1 * sin_a),
                           QPointF(cx + r2 * cos_a, cy + r2 * sin_a))

        # Compass circle
        p.setPen(QPen(FG_DIM, 1.0))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(cx, cy), radius, radius)

        # ── Heading bug (cyan rectangle with outward notch, inside rim) ──
        bug_ang = self._hdg_bug - hdg
        bw = radius * 0.04   # half-width
        bh = radius * 0.08   # height (inward from rim)
        notch = bh * 0.45    # depth of the outward notch
        p.save()
        p.translate(cx, cy)
        p.rotate(bug_ang)
        # Bug at top (0°): outer edge on the rim, body extends inward
        # Notch is cut into the OUTER edge (toward rim)
        r_rim = radius
        p.setPen(QPen(CYAN, 1.5))
        p.setBrush(QBrush(CYAN))
        bug_shape = QPolygonF([
            QPointF(-bw, -r_rim),              # outer-left (on rim)
            QPointF(0, -r_rim + notch),        # notch center (pushed inward)
            QPointF(bw, -r_rim),               # outer-right (on rim)
            QPointF(bw, -r_rim + bh),          # inner-right
            QPointF(-bw, -r_rim + bh),         # inner-left
        ])
        p.drawPolygon(bug_shape)
        p.restore()

        # ── Aircraft symbol (fixed, white, top-down airplane) ──
        s = radius * 0.10  # scale unit
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(FG))
        plane = QPolygonF([
            QPointF(cx, cy - s * 1.8),        # nose
            QPointF(cx + s * 0.25, cy - s),    # right fuselage
            QPointF(cx + s * 1.6, cy + s * 0.1),  # right wingtip
            QPointF(cx + s * 1.6, cy + s * 0.4),
            QPointF(cx + s * 0.25, cy + s * 0.1),  # right wing root
            QPointF(cx + s * 0.25, cy + s * 1.0),  # right tail
            QPointF(cx + s * 0.7, cy + s * 1.4),   # right h-stab tip
            QPointF(cx + s * 0.7, cy + s * 1.6),
            QPointF(cx + s * 0.15, cy + s * 1.2),
            QPointF(cx, cy + s * 1.8),         # tail
            QPointF(cx - s * 0.15, cy + s * 1.2),
            QPointF(cx - s * 0.7, cy + s * 1.6),
            QPointF(cx - s * 0.7, cy + s * 1.4),
            QPointF(cx - s * 0.25, cy + s * 1.0),
            QPointF(cx - s * 0.25, cy + s * 0.1),
            QPointF(cx - s * 1.6, cy + s * 0.4),
            QPointF(cx - s * 1.6, cy + s * 0.1),
            QPointF(cx - s * 0.25, cy - s),
        ])
        p.drawPolygon(plane)

        # ── Lubber line (fixed triangle at top of rose) ──
        tri_w = radius * 0.05
        tri_h = radius * 0.07
        top_y = cy - radius
        tri = QPolygonF([
            QPointF(cx, top_y + tri_h),
            QPointF(cx - tri_w, top_y - 2),
            QPointF(cx + tri_w, top_y - 2),
        ])
        p.setPen(QPen(FG, 1.5))
        p.setBrush(QBrush(FG))
        p.drawPolygon(tri)

        # ── Heading readout box (centered above lubber line) ──
        hdg_font = QFont("Monospace", max(12, int(radius * 0.14)))
        hdg_font.setBold(True)
        hdg_fm = QFontMetrics(hdg_font)
        txt = f"{hdg:03.0f}\u00B0"
        fixed_tw = hdg_fm.horizontalAdvance("000\u00B0")
        box_w = fixed_tw + 16
        box_h = hdg_fm.height() + 8
        box_x = cx - box_w / 2
        box_y = top_y - tri_h - box_h - 4
        # Opaque black background
        p.setBrush(QBrush(QColor(10, 12, 18)))
        p.setPen(QPen(FG, 1.0))
        p.drawRect(QRectF(box_x, box_y, box_w, box_h))
        # Green heading text
        p.setFont(hdg_font)
        p.setPen(QPen(GREEN, 1.5))
        tw = hdg_fm.horizontalAdvance(txt)
        p.drawText(QPointF(cx - tw / 2,
                           box_y + (box_h + hdg_fm.ascent() - hdg_fm.descent()) / 2), txt)

        # ── HDG bug readout ("HDG" white + value cyan, to the left) ──
        bug_font = QFont("Monospace", max(8, int(radius * 0.09)))
        bug_font.setBold(True)
        p.setFont(bug_font)
        bug_fm = QFontMetrics(bug_font)
        lbl = "HDG "
        val = f"{self._hdg_bug:03.0f}\u00B0"
        lbl_w = bug_fm.horizontalAdvance(lbl)
        val_w = bug_fm.horizontalAdvance(val)
        total_w = lbl_w + val_w + 10
        hb_h = bug_fm.height() + 4
        hb_x = box_x - total_w - 6
        hb_y = box_y + (box_h - hb_h) / 2
        p.fillRect(QRectF(hb_x, hb_y, total_w, hb_h), QColor(20, 22, 30))
        p.setPen(QPen(FG_DIM, 1.0))
        p.drawRect(QRectF(hb_x, hb_y, total_w, hb_h))
        text_y = hb_y + (hb_h + bug_fm.ascent() - bug_fm.descent()) / 2
        p.setPen(QPen(FG, 1))
        p.drawText(QPointF(hb_x + 5, text_y), lbl)
        p.setPen(QPen(CYAN, 1))
        p.drawText(QPointF(hb_x + 5 + lbl_w, text_y), val)

        p.restore()
