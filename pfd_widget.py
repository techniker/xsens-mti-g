#!/usr/bin/env python3
"""
Airbus-Accurate Primary Flight Display (PFD) Widget
Custom-painted Qt widget with:
  - Attitude indicator (sky/ground, pitch ladder, bank arc)
  - Speed tape (scrolling, left side)
  - Altitude tape (scrolling, right side)
  - Heading tape (bottom, scrolling)
  - Vertical Speed Indicator (far right)
  - Flight Path Vector (FPV / bird)
  - Aircraft reference symbol
  - Airbus color scheme

Display philosophy: trust the Xsens MTi-G onboard EKF.
No additional filtering — the AHRS output is displayed directly,
exactly as a real flight instrument would.
"""

import math
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QRectF, QPointF, QRect
from PyQt6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QPolygonF,
    QPainterPath, QFontMetrics,
)
from sensors import SensorData


# ─────────── Airbus Color Palette ───────────
SKY      = QColor(0x4C, 0xA3, 0xDD)
GROUND   = QColor(0xB9, 0x7A, 0x56)
PANEL_BG = QColor(0x0A, 0x0C, 0x10)
FG       = QColor(255, 255, 255)
FG_DIM   = QColor(180, 180, 180)
YELLOW   = QColor(255, 200, 0)
GREEN    = QColor(0, 200, 0)
RED      = QColor(255, 50, 50)
TAPE_BG  = QColor(20, 22, 30, 200)
POINTER_BG = QColor(30, 32, 40, 240)


class PFDWidget(QWidget):
    """Full Airbus-style PFD rendered via QPainter.
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

        # Z-key bias
        self._bias_roll = 0.0
        self._bias_pitch = 0.0

    def set_data(self, data: SensorData):
        """Update attitude from accelerometer (drift-free), heading from EKF."""
        # Roll/pitch: compute directly from accelerometer gravity vector.
        # Diagnostic proved accel is rock-solid; EKF drifts ~0.7°/s.
        if data.acc:
            ax, ay, az = data.acc
            accel_roll = math.degrees(math.atan2(ay, az))
            accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
            # Low-pass to smooth accel noise
            a = self._lp_alpha
            self._roll = a * self._roll + (1.0 - a) * (accel_roll - self._bias_roll)
            self._pitch = a * self._pitch + (1.0 - a) * (accel_pitch - self._bias_pitch)

        # Heading: EKF only (accelerometer can't provide heading)
        if data.yaw_deg is not None:
            self._heading = data.yaw_deg % 360.0
            self._yaw_raw = data.yaw_deg

        if data.vel:
            vx, vy, vz = data.vel
            self._vx, self._vy, self._vz = vx, vy, vz
            raw_spd = math.hypot(vx, vy)
            # Squelch before filter — kill noise at the source so it
            # can't accumulate in the integrator.
            if raw_spd < self._spd_squelch:
                raw_spd = 0.0
            a = self._spd_lp_alpha
            self._speed = a * self._speed + (1.0 - a) * raw_spd
            self._vsi = -vz
        elif data.speed_ms is not None:
            raw_spd = data.speed_ms
            if raw_spd < self._spd_squelch:
                raw_spd = 0.0
            a = self._spd_lp_alpha
            self._speed = a * self._speed + (1.0 - a) * raw_spd

        if data.baro_alt_m is not None:
            self._altitude = data.baro_alt_m
        elif data.pos_alt is not None:
            self._altitude = data.pos_alt

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

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Z:
            self.zero_attitude()
        elif event.key() == Qt.Key.Key_U:
            self.toggle_units()
        elif event.key() == Qt.Key.Key_R:
            self.reset_zero()
        else:
            super().keyPressEvent(event)

    # ─────────── Layout geometry ───────────

    def _layout(self):
        """Compute sub-regions based on widget size."""
        w, h = self.width(), self.height()
        hdg_h = max(50, int(h * 0.08))
        spd_w = max(90, int(w * 0.10))
        alt_w = max(120, int(w * 0.14))   # wider for 5 rolling digits
        vsi_w = max(45, int(w * 0.04))
        att_x = spd_w
        att_w = w - spd_w - alt_w - vsi_w
        att_h = h - hdg_h
        return {
            'att': QRect(att_x, 0, att_w, att_h),
            'spd': QRect(0, 0, spd_w, att_h),
            'alt': QRect(att_x + att_w, 0, alt_w, att_h),
            'vsi': QRect(att_x + att_w + alt_w, 0, vsi_w, att_h),
            'hdg': QRect(att_x, att_h, att_w, hdg_h),
        }

    # ─────────── Paint ───────────

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), PANEL_BG)

        layout = self._layout()
        self._draw_attitude(p, layout['att'])
        self._draw_speed_tape(p, layout['spd'])
        self._draw_altitude_tape(p, layout['alt'])
        self._draw_vsi(p, layout['vsi'])
        self._draw_heading_tape(p, layout['hdg'])
        p.end()

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
        font = QFont("Monospace", max(9, int(size * 0.022)))
        font.setBold(True)
        p.setFont(font)
        fm = QFontMetrics(font)

        for deg in range(-90, 91, 5):
            if deg == 0:
                continue
            y = pitch_shift + deg * px_per_deg
            if abs(y) > size * 0.7:
                continue

            if deg % 10 == 0:
                half_w = size * 0.12
                p.setPen(QPen(FG, 1.8))
            else:
                half_w = size * 0.06
                p.setPen(QPen(FG, 1.2))

            if deg < 0:
                pen = p.pen()
                pen.setStyle(Qt.PenStyle.DashLine)
                p.setPen(pen)

            p.drawLine(QPointF(-half_w, y), QPointF(half_w, y))

            if deg % 10 == 0:
                txt = f"{abs(deg)}"
                tw = fm.horizontalAdvance(txt)
                p.setPen(QPen(FG, 1))
                p.drawText(QPointF(half_w + 6, y + fm.ascent() / 2.5), txt)
                p.drawText(QPointF(-half_w - 6 - tw, y + fm.ascent() / 2.5), txt)

    def _draw_bank_arc(self, p: QPainter, r: QRect):
        cx, cy = r.center().x(), r.center().y()
        radius = min(r.width(), r.height()) * 0.38
        arc_cy = cy - min(r.width(), r.height()) * 0.12

        p.save()
        p.setClipRect(r)

        # Arc ±60 degrees from top
        p.setPen(QPen(FG, 2))
        arc_rect = QRectF(cx - radius, arc_cy - radius, radius * 2, radius * 2)
        p.drawArc(arc_rect, 30 * 16, 120 * 16)

        # Tick marks
        for ang in [-60, -45, -30, -20, -10, 10, 20, 30, 45, 60]:
            if abs(ang) in (30, 60):
                tick_len, pen_w = radius * 0.12, 2.5
            elif abs(ang) == 45:
                tick_len, pen_w = radius * 0.10, 2.0
            else:
                tick_len, pen_w = radius * 0.08, 1.5

            rad = math.radians(90 + ang)
            x1 = cx + radius * math.cos(rad)
            y1 = arc_cy - radius * math.sin(rad)
            x2 = cx + (radius - tick_len) * math.cos(rad)
            y2 = arc_cy - (radius - tick_len) * math.sin(rad)
            p.setPen(QPen(FG, pen_w))
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        # Bank angle labels
        font = QFont("Monospace", max(8, int(radius * 0.08)))
        p.setFont(font)
        fm = QFontMetrics(font)
        p.setPen(QPen(FG, 1))
        for ang in [-30, -20, -10, 10, 20, 30]:
            rad = math.radians(90 + ang)
            lx = cx + (radius + radius * 0.09) * math.cos(rad)
            ly = arc_cy - (radius + radius * 0.09) * math.sin(rad)
            txt = f"{abs(ang)}"
            p.drawText(QPointF(lx - fm.horizontalAdvance(txt) / 2, ly + fm.ascent() / 3), txt)

        # Top index triangle (fixed at 0°)
        tri_h = radius * 0.07
        tri_w = radius * 0.06
        top_y = arc_cy - radius
        tri = QPolygonF([
            QPointF(cx, top_y - tri_h),
            QPointF(cx - tri_w, top_y),
            QPointF(cx + tri_w, top_y),
        ])
        p.setPen(QPen(FG, 1.5))
        p.setBrush(QBrush(FG))
        p.drawPolygon(tri)

        # Moving bank pointer — triangle that rides on the arc
        # Build at 0° (top), then rotate into position around arc center
        bank_ang = max(-60.0, min(60.0, self._roll))
        ptr_h = radius * 0.09
        ptr_w = radius * 0.06
        # Triangle at top (0°): tip on arc, base inward
        tip_y = -radius
        base_y = -radius + ptr_h
        tri_pts = [
            QPointF(0, tip_y),                # tip on arc
            QPointF(-ptr_w, base_y),           # base left
            QPointF(ptr_w, base_y),            # base right
        ]
        # Rotate around arc center by bank angle
        p.save()
        p.translate(cx, arc_cy)
        p.rotate(-bank_ang)
        p.setPen(QPen(FG, 1.5))
        p.setBrush(QBrush(FG))
        p.drawPolygon(QPolygonF(tri_pts))
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
        fpv_x_px = self._fpv_x * px_per_deg * 0.6
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
        """Rolling drum digit display with properly sized box.

        Every digit scrolls proportionally.  Higher digits move slowly;
        the ones digit scrolls at full speed — like a mechanical counter.
        """
        font = QFont("Monospace", font_size)
        font.setBold(True)
        fm = QFontMetrics(font)
        char_w = fm.horizontalAdvance("0")
        digit_h = fm.height()
        half_box = box_rect.height() / 2.0
        val = abs(value)

        p.save()
        p.setFont(font)
        p.setPen(QPen(FG, 1.5))

        for col in range(num_digits):
            position = num_digits - 1 - col   # 0 = ones, 1 = tens, …
            place = 10 ** position

            digit_val = (val / place) % 10.0
            digit = int(digit_val) % 10
            frac = digit_val - int(digit_val)

            # Smoothstep easing for fluid motion.
            frac = frac * frac * (3.0 - 2.0 * frac)

            # Suppress leading zeros (but always draw the ones digit).
            if position > 0 and digit == 0 and frac < 0.001 and val < place:
                continue

            next_d = (digit + 1) % 10
            prev_d = (digit - 1) % 10
            sx = x_text + col * char_w

            p.save()
            p.setClipRect(QRectF(sx - 1, cy - half_box, char_w + 2, half_box * 2))

            y_shift = -frac * digit_h
            # Vertically center text in box: baseline = cy + ascent/2 - descent/2
            baseline = cy + (fm.ascent() - fm.descent()) / 2.0

            p.drawText(QPointF(sx, baseline + y_shift), str(digit))
            p.drawText(QPointF(sx, baseline + y_shift + digit_h), str(next_d))
            p.drawText(QPointF(sx, baseline + y_shift - digit_h), str(prev_d))

            p.restore()

        if value < 0:
            p.drawText(QPointF(x_text - char_w,
                                cy + (fm.ascent() - fm.descent()) / 2.0), "-")

        p.restore()

    # ─────────── Speed tape ───────────

    def _draw_speed_tape(self, p: QPainter, r: QRect):
        p.save()
        p.setClipRect(r)
        p.fillRect(r, TAPE_BG)

        if self._metric:
            speed_disp = self._speed * 3.6   # m/s → km/h
            major, minor = 10, 2             # tick spacing
            span = 80.0                      # visible range
            label = "SPD km/h"
        else:
            speed_disp = self._speed * 1.94384  # m/s → knots
            major, minor = 10, 2
            span = 60.0
            label = "SPD kt"

        cy = r.center().y()
        tape_w = r.width()
        px_per_unit = r.height() / span

        font = QFont("Monospace", max(9, int(tape_w * 0.13)))
        p.setFont(font)
        fm = QFontMetrics(font)
        p.setPen(QPen(FG, 1))

        base = int(speed_disp / major) * major
        for i in range(-8, 9):
            for sub in range(0, major, minor):
                v = base + i * major + sub
                if v < 0:
                    continue
                y = cy - (v - speed_disp) * px_per_unit
                if y < r.top() - 10 or y > r.bottom() + 10:
                    continue
                if v % major == 0:
                    p.setPen(QPen(FG, 1.5))
                    p.drawLine(QPointF(tape_w - 25, y), QPointF(tape_w, y))
                    txt = f"{v:3.0f}"
                    p.drawText(QPointF(tape_w - 25 - fm.horizontalAdvance(txt) - 4,
                                       y + fm.ascent() / 2.5), txt)
                else:
                    p.setPen(QPen(FG_DIM, 1))
                    p.drawLine(QPointF(tape_w - 12, y), QPointF(tape_w, y))

        # Display-smoothed value for the drum
        a = self._disp_alpha
        self._disp_speed = a * self._disp_speed + (1.0 - a) * speed_disp
        drum_spd = self._disp_speed

        # Font sized to tape height, box sized to font
        ptr_font_sz = max(11, int(r.height() * 0.032))
        ptr_fm = QFontMetrics(QFont("Monospace", ptr_font_sz))
        box_h = ptr_fm.height() + 10
        pad = 6

        path = QPainterPath()
        path.moveTo(tape_w, cy)
        path.lineTo(tape_w - 12, cy - box_h / 2)
        path.lineTo(pad, cy - box_h / 2)
        path.lineTo(pad, cy + box_h / 2)
        path.lineTo(tape_w - 12, cy + box_h / 2)
        path.closeSubpath()
        p.fillPath(path, POINTER_BG)
        p.setPen(QPen(FG, 1.5))
        p.drawPath(path)

        box_r = QRectF(pad, cy - box_h / 2, tape_w - 12 - pad, box_h)
        self._draw_drum_pointer(p, drum_spd, 3, cy, ptr_font_sz, box_r, pad + 4)

        small_font = QFont("Monospace", max(7, int(tape_w * 0.09)))
        p.setFont(small_font)
        p.setPen(QPen(FG_DIM, 1))
        p.drawText(QPointF(4, r.bottom() - 4), label)
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
            label = "ALT m"
        else:
            alt_disp = self._altitude * 3.28084  # feet
            major, minor = 100, 20
            span = 600.0
            label = "ALT ft"

        cy = r.center().y()
        tape_w = r.width()
        px_per_unit = r.height() / span
        x0 = r.left()

        font = QFont("Monospace", max(9, int(tape_w * 0.13)))
        p.setFont(font)
        fm = QFontMetrics(font)
        p.setPen(QPen(FG, 1))

        base = int(alt_disp / major) * major
        for i in range(-8, 9):
            for sub in range(0, major, minor):
                v = base + i * major + sub
                y = cy - (v - alt_disp) * px_per_unit
                if y < r.top() - 10 or y > r.bottom() + 10:
                    continue
                if v % major == 0:
                    p.setPen(QPen(FG, 1.5))
                    p.drawLine(QPointF(x0, y), QPointF(x0 + 25, y))
                    p.drawText(QPointF(x0 + 30, y + fm.ascent() / 2.5), f"{v:5.0f}")
                elif v % minor == 0:
                    p.setPen(QPen(FG_DIM, 1))
                    p.drawLine(QPointF(x0, y), QPointF(x0 + 12, y))

        # Display-smoothed value for the drum
        a = self._disp_alpha
        self._disp_alt = a * self._disp_alt + (1.0 - a) * alt_disp
        drum_alt = self._disp_alt

        # Same font size as speed tape (based on tape height, shared)
        ptr_font_sz = max(11, int(r.height() * 0.032))
        ptr_fm = QFontMetrics(QFont("Monospace", ptr_font_sz))
        box_h = ptr_fm.height() + 10
        pad = 6

        path = QPainterPath()
        path.moveTo(x0, cy)
        path.lineTo(x0 + 12, cy - box_h / 2)
        path.lineTo(x0 + tape_w - pad, cy - box_h / 2)
        path.lineTo(x0 + tape_w - pad, cy + box_h / 2)
        path.lineTo(x0 + 12, cy + box_h / 2)
        path.closeSubpath()
        p.fillPath(path, POINTER_BG)
        p.setPen(QPen(FG, 1.5))
        p.drawPath(path)

        box_r = QRectF(x0 + 12, cy - box_h / 2, tape_w - 12 - pad, box_h)
        self._draw_drum_pointer(p, drum_alt, 5, cy, ptr_font_sz, box_r, x0 + 16)

        small_font = QFont("Monospace", max(7, int(tape_w * 0.09)))
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

        if self._metric:
            vsi_disp = self._vsi              # m/s
            max_val = 10.0                    # ±10 m/s
            ticks = [2, 4, 6, 8, 10]
            label = "m/s"
            fmt = "{:+.1f}"
        else:
            vsi_disp = self._vsi * 196.85     # ft/min
            max_val = 2000.0
            ticks = [500, 1000, 1500, 2000]
            label = "fpm"
            fmt = "{:+.0f}"

        px_per = (h * 0.42) / max_val

        p.setPen(QPen(FG_DIM, 1))
        p.drawLine(QPointF(r.left(), cy), QPointF(r.right(), cy))

        font = QFont("Monospace", max(7, int(r.width() * 0.18)))
        p.setFont(font)
        fm = QFontMetrics(font)

        for tv in ticks:
            for sign in (-1, 1):
                y = cy - sign * tv * px_per
                if r.top() < y < r.bottom():
                    p.setPen(QPen(FG_DIM, 1))
                    p.drawLine(QPointF(r.left(), y), QPointF(r.left() + 8, y))
                    if self._metric:
                        if tv % 2 == 0:
                            p.drawText(QPointF(r.left() + 10, y + fm.ascent() / 3), f"{tv}")
                    else:
                        if tv % 1000 == 0:
                            p.drawText(QPointF(r.left() + 10, y + fm.ascent() / 3), f"{tv // 1000}")

        clamped = max(-max_val, min(max_val, vsi_disp))
        bar_y = cy - clamped * px_per
        thresh1 = 2.5 if self._metric else 500
        thresh2 = 7.5 if self._metric else 1500
        color = GREEN if abs(vsi_disp) < thresh1 else (YELLOW if abs(vsi_disp) < thresh2 else RED)
        p.setPen(QPen(color, 2.5))
        p.drawLine(QPointF(r.left() + 2, cy), QPointF(r.left() + 2, bar_y))

        p.setPen(QPen(FG, 1))
        p.drawText(QPointF(r.left() + 2, r.top() + fm.height() + 2), fmt.format(vsi_disp))

        small_font = QFont("Monospace", max(6, int(r.width() * 0.14)))
        p.setFont(small_font)
        p.setPen(QPen(FG_DIM, 1))
        p.drawText(QPointF(r.left() + 2, r.bottom() - 4), label)
        p.restore()

    # ─────────── Heading tape ───────────

    def _draw_heading_tape(self, p: QPainter, r: QRect):
        p.save()
        p.setClipRect(r)
        p.fillRect(r, TAPE_BG)

        cx = r.center().x()
        tape_h = r.height()
        hdg = self._heading
        px_per_deg = r.width() / 60.0

        font = QFont("Monospace", max(9, int(tape_h * 0.28)))
        font.setBold(True)
        p.setFont(font)
        fm = QFontMetrics(font)

        cardinal = {0: "N", 90: "E", 180: "S", 270: "W"}

        for i in range(-35, 36):
            deg = hdg + i
            norm_deg = int(deg) % 360
            x = cx + i * px_per_deg

            if x < r.left() - 20 or x > r.right() + 20:
                continue

            if norm_deg % 10 == 0:
                p.setPen(QPen(FG, 1.5))
                p.drawLine(QPointF(x, r.top()), QPointF(x, r.top() + tape_h * 0.35))

                txt = cardinal.get(norm_deg, f"{norm_deg:03d}")
                p.setPen(QPen(FG, 1))
                tw = fm.horizontalAdvance(txt)
                p.drawText(QPointF(x - tw / 2, r.top() + tape_h * 0.35 + fm.ascent() + 2), txt)

            elif norm_deg % 5 == 0:
                p.setPen(QPen(FG_DIM, 1))
                p.drawLine(QPointF(x, r.top()), QPointF(x, r.top() + tape_h * 0.2))

        # Center pointer
        tri_w, tri_h = 8, 10
        tri = QPolygonF([
            QPointF(cx, r.top()),
            QPointF(cx - tri_w, r.top() - tri_h + 2),
            QPointF(cx + tri_w, r.top() - tri_h + 2),
        ])
        p.setPen(QPen(YELLOW, 1.5))
        p.setBrush(QBrush(YELLOW))
        p.drawPolygon(tri)

        # Heading readout box
        box_w, box_h = 50, tape_h * 0.5
        box_rect = QRectF(cx - box_w / 2, r.bottom() - box_h - 2, box_w, box_h)
        p.fillRect(box_rect, POINTER_BG)
        p.setPen(QPen(FG, 1.2))
        p.drawRect(box_rect)

        txt = f"{hdg:03.0f}"
        p.drawText(QPointF(cx - fm.horizontalAdvance(txt) / 2, r.bottom() - 6), txt)

        small_font = QFont("Monospace", max(7, int(tape_h * 0.18)))
        p.setFont(small_font)
        p.setPen(QPen(FG_DIM, 1))
        p.drawText(QPointF(r.left() + 4, r.bottom() - 4), "HDG")
        p.restore()
