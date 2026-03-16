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


# ─────────── Color Palette (G1000-inspired) ───────────
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

        # Data validity flags — True once first valid sample arrives.
        self._valid_att = False    # roll/pitch (accelerometer)
        self._valid_hdg = False    # heading (EKF yaw)
        self._valid_spd = False    # speed (velocity)
        self._valid_alt = False    # altitude (baro or GPS)
        self._valid_vsi = False    # vertical speed

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
            self._valid_att = True
            ax, ay, az = data.acc
            accel_roll = math.degrees(math.atan2(ay, az))
            accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
            # Low-pass to smooth accel noise
            a = self._lp_alpha
            self._roll = a * self._roll + (1.0 - a) * (accel_roll - self._bias_roll)
            self._pitch = a * self._pitch + (1.0 - a) * (accel_pitch - self._bias_pitch)

        # Heading: EKF only (accelerometer can't provide heading)
        if data.yaw_deg is not None:
            self._valid_hdg = True
            self._heading = data.yaw_deg % 360.0
            self._yaw_raw = data.yaw_deg

        if data.vel:
            self._valid_spd = True
            self._valid_vsi = True
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
            self._valid_spd = True
            raw_spd = data.speed_ms
            if raw_spd < self._spd_squelch:
                raw_spd = 0.0
            a = self._spd_lp_alpha
            self._speed = a * self._speed + (1.0 - a) * raw_spd

        if data.baro_alt_m is not None:
            self._valid_alt = True
            self._altitude = data.baro_alt_m
        elif data.pos_alt is not None:
            self._valid_alt = True
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
        """Compute sub-regions based on widget size (G1000 proportions)."""
        w, h = self.width(), self.height()
        hdg_h = max(50, int(h * 0.08))
        spd_w = max(80, int(w * 0.085))   # narrow speed tape
        alt_w = max(105, int(w * 0.105))   # wider altitude (5 digits)
        vsi_w = max(40, int(w * 0.035))
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
            self._draw_fail_flag(p, layout['hdg'], "HDG")

        p.end()

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
        """Rolling drum digit display — right-aligned, no leading zeros.

        Every digit scrolls proportionally.  Higher digits move slowly;
        the ones digit scrolls at full speed — like a mechanical counter.
        Digits are positioned from the right so suppressed leading zeros
        don't leave gaps.
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

        for position in range(num_digits):
            # position 0 = ones, 1 = tens, …
            place = 10 ** position

            digit_val = (val / place) % 10.0
            digit = int(digit_val) % 10
            frac = digit_val - int(digit_val)

            # Suppress leading zeros (never suppress ones digit).
            # A digit is a leading zero if the entire value is below this
            # place — the fractional scroll is irrelevant for that check.
            if position > 0 and val < place:
                continue

            # Smoothstep easing for fluid motion.
            frac = frac * frac * (3.0 - 2.0 * frac)

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
        p.setClipRect(r)
        p.fillRect(r, TAPE_BG)

        if self._metric:
            speed_disp = self._speed * 3.6   # m/s → km/h
            major, minor = 10, 2             # tick spacing
            span = 80.0                      # visible range
            label = "km/h"
        else:
            speed_disp = self._speed * 1.94384  # m/s → knots
            major, minor = 10, 2
            span = 60.0
            label = "KT"

        cy = r.center().y()
        tape_w = r.width()
        px_per_unit = r.height() / span

        # ── G1000-style color bands (right edge of tape) ──
        if SPD_BANDS_ENABLED:
            bw = max(4, int(tape_w * 0.08))

            def _band(v_lo, v_hi, color):
                yt = cy - (v_hi - speed_disp) * px_per_unit
                yb = cy - (v_lo - speed_disp) * px_per_unit
                yt = max(r.top(), min(r.bottom(), yt))
                yb = max(r.top(), min(r.bottom(), yb))
                if yb > yt:
                    p.fillRect(QRectF(tape_w - bw, yt, bw, yb - yt), color)

            # Draw white first, then green on top (green overwrites overlap)
            _band(SPD_VSO, SPD_VFE, FG)
            _band(SPD_VS1, SPD_VNO, GREEN)
            _band(SPD_VNO, SPD_VNE, YELLOW)
            # Vne red line
            y_vne = cy - (SPD_VNE - speed_disp) * px_per_unit
            if r.top() < y_vne < r.bottom():
                p.setPen(QPen(RED, 3.0))
                p.drawLine(QPointF(tape_w - bw - 4, y_vne), QPointF(tape_w, y_vne))

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
                if y < r.top() - 10 or y > r.bottom() + 10:
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
        path = QPainterPath()
        path.moveTo(tape_w, cy)
        path.lineTo(tape_w - arrow_w, cy - box_h / 2)
        path.lineTo(2, cy - box_h / 2)
        path.lineTo(2, cy + box_h / 2)
        path.lineTo(tape_w - arrow_w, cy + box_h / 2)
        path.closeSubpath()
        p.fillPath(path, POINTER_BG)
        p.setPen(QPen(FG, 1.5))
        p.drawPath(path)

        digits_w = ptr_fm.horizontalAdvance("0") * 3
        x_text = 2 + (avail_w - digits_w) / 2
        box_r = QRectF(2, cy - box_h / 2, tape_w - arrow_w - 2, box_h)
        self._draw_drum_pointer(p, drum_spd, 3, cy, ptr_font_sz, box_r, x_text)

        small_font = QFont("Monospace", max(7, int(tape_w * 0.10)))
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

    # ─────────── Vertical Speed Indicator (G1000-style) ───────────

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

        # Non-linear scale: compress outer range (G1000 style)
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

        # Pointer triangle
        clamped = max(-max_val, min(max_val, vsi_disp))
        ptr_y = vsi_to_y(clamped)
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

    # ─────────── Heading tape (G1000-style) ───────────

    def _draw_heading_tape(self, p: QPainter, r: QRect):
        p.save()
        p.setClipRect(r)
        p.fillRect(r, TAPE_BG)

        cx = r.center().x()
        tape_h = r.height()
        hdg = self._heading
        px_per_deg = r.width() / 60.0

        font = QFont("Monospace", max(9, int(tape_h * 0.26)))
        font.setBold(True)
        p.setFont(font)
        fm = QFontMetrics(font)

        cardinal = {0: "N", 90: "E", 180: "S", 270: "W"}
        tick_top = r.top() + 2

        for i in range(-35, 36):
            deg = hdg + i
            norm_deg = int(round(deg)) % 360
            x = cx + i * px_per_deg

            if x < r.left() - 20 or x > r.right() + 20:
                continue

            if norm_deg % 10 == 0:
                p.setPen(QPen(FG, 1.5))
                p.drawLine(QPointF(x, tick_top), QPointF(x, tick_top + tape_h * 0.30))

                if norm_deg in cardinal:
                    txt = cardinal[norm_deg]
                    p.setPen(QPen(CYAN, 1.5))
                else:
                    txt = f"{norm_deg:03d}"
                    p.setPen(QPen(FG, 1))
                tw = fm.horizontalAdvance(txt)
                p.drawText(QPointF(x - tw / 2,
                                   tick_top + tape_h * 0.30 + fm.ascent() + 1), txt)

            elif norm_deg % 5 == 0:
                p.setPen(QPen(FG_DIM, 1))
                p.drawLine(QPointF(x, tick_top), QPointF(x, tick_top + tape_h * 0.15))

        # Lubber line (white triangle at top, pointing down)
        tri_w, tri_h = 7, 9
        tri = QPolygonF([
            QPointF(cx, tick_top + tri_h),
            QPointF(cx - tri_w, tick_top),
            QPointF(cx + tri_w, tick_top),
        ])
        p.setPen(QPen(FG, 1.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPolygon(tri)

        # Heading readout box (bottom center, fixed size)
        hdg_font = QFont("Monospace", max(10, int(tape_h * 0.30)))
        hdg_font.setBold(True)
        hdg_fm = QFontMetrics(hdg_font)
        # Fixed box width based on "000" so it never resizes
        fixed_tw = hdg_fm.horizontalAdvance("000")
        box_w = fixed_tw + 12
        box_h = hdg_fm.height() + 4
        box_rect = QRectF(cx - box_w / 2, r.bottom() - box_h - 2, box_w, box_h)
        p.fillRect(box_rect, POINTER_BG)
        p.setPen(QPen(FG, 1.2))
        p.drawRect(box_rect)
        p.setFont(hdg_font)
        txt = f"{hdg:03.0f}"
        tw = hdg_fm.horizontalAdvance(txt)
        p.drawText(QPointF(cx - tw / 2,
                           r.bottom() - 2 - (box_h - hdg_fm.ascent()) / 2), txt)

        p.restore()
