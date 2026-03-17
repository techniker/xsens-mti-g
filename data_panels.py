#!/usr/bin/env python3
"""
Sensor Data Display Panels — flat multi-column layout (no scroll).
Satellite C/N0 bar graph widget. All GNSS fields exposed.
"""

import math
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QFrame, QSizePolicy, QPushButton,
)
from PyQt6.QtCore import Qt, QRectF, QPointF, QTimer, pyqtSignal
from PyQt6.QtGui import QFont, QColor, QPainter, QPen, QBrush, QFontMetrics

from sensors import SensorData, DeviceInfo


# ─────────── Styles ───────────

LABEL_STYLE = "color: #777; font-size: 9px;"
VALUE_STYLE = "color: #ddd; font-size: 10px; font-family: monospace; font-weight: bold;"
GROUP_STYLE = """
QGroupBox {
    border: 1px solid #282a30;
    border-radius: 3px;
    margin-top: 4px;
    padding: 8px 3px 1px 3px;
    background-color: #111318;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 4px;
    padding: 0 2px;
    color: #4CA3DD;
    font-weight: bold;
    font-size: 9px;
}
"""


def _val(text="--"):
    lbl = QLabel(text)
    lbl.setStyleSheet(VALUE_STYLE)
    lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    lbl.setMinimumWidth(90)
    lbl.setMaximumWidth(90)
    return lbl


def _lbl(text):
    lbl = QLabel(text)
    lbl.setStyleSheet(LABEL_STYLE)
    return lbl


# ─────────── Compact group panels ───────────

class _KVGroup(QGroupBox):
    """Key-value group with a grid of label: value rows."""

    def __init__(self, title, items, parent=None):
        super().__init__(title, parent)
        self.setStyleSheet(GROUP_STYLE)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        grid = QGridLayout(self)
        grid.setSpacing(0)
        grid.setContentsMargins(3, 10, 3, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 0)
        self._f = {}
        for row, (label, key) in enumerate(items):
            grid.addWidget(_lbl(label), row, 0)
            v = _val()
            grid.addWidget(v, row, 1)
            self._f[key] = v

    def __getitem__(self, key):
        return self._f[key]


class DeviceInfoPanel(_KVGroup):
    def __init__(self, parent=None):
        super().__init__("DEVICE", [
            ("ID", "did"), ("Product", "product"), ("FW", "fw"),
            ("Mode", "mode"), ("Settings", "settings"),
            ("DataLen", "dlen"), ("RAWGPS", "rawgps"),
        ], parent)

    def set_info(self, info: DeviceInfo):
        self["did"].setText(f"0x{info.device_id:08X}")
        self["product"].setText(info.product_code or "--")
        self["fw"].setText(f"{info.fw_major}.{info.fw_minor}.{info.fw_rev}")
        self["mode"].setText(f"0x{info.output_mode:04X}")
        self["settings"].setText(f"0x{info.output_settings:08X}")
        self["dlen"].setText(str(info.data_length))
        self["rawgps"].setText("ON" if info.has_rawgps else "OFF")
        color = "#0c0" if info.has_rawgps else "#c00"
        self["rawgps"].setStyleSheet(VALUE_STYLE.replace("#ddd", color))


class StatusPanel(_KVGroup):
    def __init__(self, parent=None):
        super().__init__("STATUS", [
            ("Status", "st"), ("Sample#", "sc"), ("AIN1", "a1"),
            ("AIN2", "a2"), ("UTC [ms]", "utc"), ("Rate [Hz]", "rate"),
        ], parent)
        self._last_ts = None
        self._accum = []

    def update_data(self, d: SensorData):
        self["st"].setText(f"0x{d.status_byte:02X}" if d.status_byte is not None else "--")
        self["sc"].setText(f"{d.sample_cnt}" if d.sample_cnt is not None else "--")
        self["a1"].setText(f"{d.ain1}" if d.ain1 is not None else "--")
        self["a2"].setText(f"{d.ain2}" if d.ain2 is not None else "--")
        self["utc"].setText(f"{d.utc_ms}" if d.utc_ms is not None else "--")
        if self._last_ts and d.timestamp > self._last_ts:
            dt = d.timestamp - self._last_ts
            if dt > 0:
                self._accum.append(1.0 / dt)
                if len(self._accum) > 50:
                    self._accum.pop(0)
                self["rate"].setText(f"{sum(self._accum)/len(self._accum):.1f}")
        self._last_ts = d.timestamp


class OrientationPanel(_KVGroup):
    def __init__(self, parent=None):
        super().__init__("ORIENTATION", [
            ("Roll [deg]", "r"), ("Pitch [deg]", "p"), ("Yaw [deg]", "y"),
        ], parent)

    def update_data(self, d: SensorData):
        self["r"].setText(f"{d.roll_deg:+8.2f}" if d.roll_deg is not None else "--")
        self["p"].setText(f"{d.pitch_deg:+8.2f}" if d.pitch_deg is not None else "--")
        self["y"].setText(f"{d.yaw_deg:+8.2f}" if d.yaw_deg is not None else "--")


class AccelPanel(_KVGroup):
    def __init__(self, parent=None):
        super().__init__("ACCEL [m/s2]", [("X","x"),("Y","y"),("Z","z")], parent)

    def update_data(self, d: SensorData):
        if d.acc:
            self["x"].setText(f"{d.acc[0]:+9.4f}")
            self["y"].setText(f"{d.acc[1]:+9.4f}")
            self["z"].setText(f"{d.acc[2]:+9.4f}")
        else:
            for k in "xyz": self[k].setText("--")


class GyroPanel(_KVGroup):
    def __init__(self, parent=None):
        super().__init__("GYRO [rad/s]", [("X","x"),("Y","y"),("Z","z")], parent)

    def update_data(self, d: SensorData):
        if d.gyr:
            self["x"].setText(f"{d.gyr[0]:+9.5f}")
            self["y"].setText(f"{d.gyr[1]:+9.5f}")
            self["z"].setText(f"{d.gyr[2]:+9.5f}")
        else:
            for k in "xyz": self[k].setText("--")


class MagPanel(_KVGroup):
    def __init__(self, parent=None):
        super().__init__("MAG", [("X","x"),("Y","y"),("Z","z")], parent)

    def update_data(self, d: SensorData):
        if d.mag:
            self["x"].setText(f"{d.mag[0]:+8.2f}")
            self["y"].setText(f"{d.mag[1]:+8.2f}")
            self["z"].setText(f"{d.mag[2]:+8.2f}")
        else:
            for k in "xyz": self[k].setText("--")


class VelocityPanel(_KVGroup):
    def __init__(self, parent=None):
        super().__init__("VELOCITY [m/s]", [
            ("Vx","vx"),("Vy","vy"),("Vz","vz"),("GS","gs"),
        ], parent)

    def update_data(self, d: SensorData):
        if d.vel:
            self["vx"].setText(f"{d.vel[0]:+8.3f}")
            self["vy"].setText(f"{d.vel[1]:+8.3f}")
            self["vz"].setText(f"{d.vel[2]:+8.3f}")
        else:
            for k in ("vx","vy","vz"): self[k].setText("--")
        self["gs"].setText(f"{d.speed_ms:8.3f}" if d.speed_ms is not None else "--")


class PositionPanel(_KVGroup):
    def __init__(self, parent=None):
        super().__init__("POSITION", [
            ("Lat [deg]","lat"),("Lon [deg]","lon"),("Alt [m]","alt"),
        ], parent)

    def update_data(self, d: SensorData):
        self["lat"].setText(f"{d.pos_lat:.7f}" if d.pos_lat is not None else "--")
        self["lon"].setText(f"{d.pos_lon:.7f}" if d.pos_lon is not None else "--")
        self["alt"].setText(f"{d.pos_alt:.2f}" if d.pos_alt is not None else "--")


class BaroPanel(_KVGroup):
    def __init__(self, parent=None):
        super().__init__("ENVIRONMENT", [
            ("Press [Pa]","p"),("Baro Alt [m]","ba"),("Temp [C]","t"),
        ], parent)

    def update_data(self, d: SensorData):
        self["p"].setText(f"{d.pressure_pa:.1f}" if d.pressure_pa is not None else "--")
        self["ba"].setText(f"{d.baro_alt_m:.1f}" if d.baro_alt_m is not None else "--")
        self["t"].setText(f"{d.temperature:.2f}" if d.temperature is not None else "--")


class GNSSPanel(_KVGroup):
    def __init__(self, parent=None):
        super().__init__("GNSS (RAWGPS)", [
            ("Lat [deg]","glat"), ("Lon [deg]","glon"), ("Alt [m]","galt"),
            ("Vel N","vn"), ("Vel E","ve"), ("Vel D","vd"),
            ("hAcc [m]","hacc"), ("vAcc [m]","vacc"), ("sAcc","sacc"),
            ("iTOW [ms]","itow"), ("bPrs","bprs"), ("bGPS","bgps"),
        ], parent)

    def update_data(self, d: SensorData):
        rg = d.rawgps
        if rg:
            self["glat"].setText(f"{rg.lat_deg:.7f}" if rg.lat_deg is not None else "--")
            self["glon"].setText(f"{rg.lon_deg:.7f}" if rg.lon_deg is not None else "--")
            self["galt"].setText(f"{rg.alt_m:.2f}" if rg.alt_m is not None else "--")
            self["vn"].setText(f"{rg.vel_n_ms:+.3f}" if rg.vel_n_ms is not None else "--")
            self["ve"].setText(f"{rg.vel_e_ms:+.3f}" if rg.vel_e_ms is not None else "--")
            self["vd"].setText(f"{rg.vel_d_ms:+.3f}" if rg.vel_d_ms is not None else "--")
            self["hacc"].setText(f"{rg.hacc_m:.3f}" if rg.hacc_m is not None else "--")
            self["vacc"].setText(f"{rg.vacc_m:.3f}" if rg.vacc_m is not None else "--")
            self["sacc"].setText(f"{rg.sacc_mps:.3f}" if rg.sacc_mps is not None else "--")
            self["itow"].setText(f"{rg.itow_ms}" if rg.itow_ms is not None else "--")
            self["bprs"].setText(f"{rg.bPrs}" if rg.bPrs is not None else "--")
            self["bgps"].setText(f"{rg.bGPS}" if rg.bGPS is not None else "--")
        else:
            for v in self._f.values():
                v.setText("--")


# ─────────── Satellite Signal Bar Graph ───────────

class SatelliteBarGraph(QWidget):
    """Custom-painted C/N0 bar graph for all GPS satellite channels."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(120)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._channels = []   # list of (svid, cnr, qi)
        self._nch = 0

    def update_data(self, d: SensorData):
        gs = d.gps_status
        if gs and gs.channels:
            self._nch = gs.nch
            self._channels = [(ch.svid, ch.cnr, ch.qi, ch.bitmask) for ch in gs.channels]
        else:
            self._channels = []
            self._nch = 0
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        bg = QColor(0x11, 0x13, 0x18)
        p.fillRect(self.rect(), bg)

        # Title
        title_font = QFont("Monospace", 10)
        title_font.setBold(True)
        p.setFont(title_font)
        p.setPen(QPen(QColor(0x4C, 0xA3, 0xDD), 1))
        p.drawText(6, 14, f"GPS SATELLITES ({self._nch} channels)")

        if not self._channels:
            p.setPen(QPen(QColor(100, 100, 100), 1))
            p.drawText(w // 2 - 40, h // 2, "No satellite data")
            p.end()
            return

        # Layout
        margin_top = 22
        margin_bottom = 28
        margin_left = 8
        margin_right = 8
        bar_area_h = h - margin_top - margin_bottom
        bar_area_w = w - margin_left - margin_right

        n = len(self._channels)
        max_cnr = 55.0  # dB-Hz scale max
        gap = max(1, int(bar_area_w * 0.01))
        bar_w = max(4, (bar_area_w - gap * (n - 1)) // n) if n > 0 else 10

        # Threshold lines
        p.setPen(QPen(QColor(60, 60, 60), 1, Qt.PenStyle.DashLine))
        label_font = QFont("Monospace", 7)
        p.setFont(label_font)
        fm = QFontMetrics(label_font)
        for thresh in (15, 30, 45):
            y = int(margin_top + bar_area_h * (1.0 - thresh / max_cnr))
            p.setPen(QPen(QColor(60, 60, 60), 1, Qt.PenStyle.DashLine))
            p.drawLine(margin_left, y, margin_left + bar_area_w, y)
            p.setPen(QPen(QColor(100, 100, 100), 1))
            p.drawText(margin_left + bar_area_w + 2, y + fm.ascent() // 2, f"{thresh}")

        # Bars
        bar_font = QFont("Monospace", max(6, min(8, bar_w - 1)))
        bar_font.setBold(True)

        for i, (svid, cnr, qi, bitmask) in enumerate(self._channels):
            x = margin_left + i * (bar_w + gap)
            cnr_clamped = max(0, min(cnr, max_cnr))
            bar_h = int(bar_area_h * (cnr_clamped / max_cnr))
            bar_y = margin_top + bar_area_h - bar_h

            # Color by signal quality
            if cnr >= 35:
                color = QColor(0x00, 0xCC, 0x00)      # strong green
            elif cnr >= 25:
                color = QColor(0x66, 0xCC, 0x00)      # yellow-green
            elif cnr >= 15:
                color = QColor(0xCC, 0xCC, 0x00)      # yellow
            elif cnr > 0:
                color = QColor(0xCC, 0x44, 0x00)      # orange
            else:
                color = QColor(0x44, 0x44, 0x44)      # grey (no signal)

            # Bar
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(color))
            if bar_h > 0:
                p.drawRect(x, bar_y, bar_w, bar_h)

            # Bar outline
            p.setPen(QPen(QColor(80, 80, 80), 1))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRect(x, margin_top, bar_w, bar_area_h)

            # C/N0 value on top of bar
            p.setFont(bar_font)
            if cnr > 0:
                p.setPen(QPen(QColor(220, 220, 220), 1))
                txt = f"{cnr}"
                tw = QFontMetrics(bar_font).horizontalAdvance(txt)
                tx = x + (bar_w - tw) // 2
                ty = bar_y - 2
                if ty < margin_top + 10:
                    ty = bar_y + 12  # inside bar if at top
                p.drawText(tx, ty, txt)

            # SV ID at bottom
            sv_font = QFont("Monospace", max(6, min(7, bar_w - 1)))
            p.setFont(sv_font)
            p.setPen(QPen(QColor(180, 180, 180), 1))
            sv_txt = f"{svid}"
            sw = QFontMetrics(sv_font).horizontalAdvance(sv_txt)
            p.drawText(x + (bar_w - sw) // 2, h - margin_bottom + 12, sv_txt)

            # Quality indicator dot
            qi_colors = {
                0: QColor(80, 80, 80),     # no signal
                1: QColor(0xCC, 0x00, 0x00),  # searching
                2: QColor(0xCC, 0xCC, 0x00),  # acquired
                3: QColor(0xCC, 0xCC, 0x00),  # detected
                4: QColor(0x00, 0x88, 0x00),  # code lock
                5: QColor(0x00, 0xCC, 0x00),  # carrier lock
                6: QColor(0x00, 0xCC, 0x00),
                7: QColor(0x00, 0xFF, 0x00),  # nav message
            }
            qc = qi_colors.get(qi, QColor(80, 80, 80))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(qc))
            dot_x = x + bar_w // 2
            dot_y = h - margin_bottom + 20
            p.drawEllipse(QPointF(dot_x, dot_y), 3, 3)

        # Bottom legend
        legend_font = QFont("Monospace", 7)
        p.setFont(legend_font)
        p.setPen(QPen(QColor(120, 120, 120), 1))
        p.drawText(margin_left, h - 2, "SV ID")
        p.drawText(margin_left + bar_area_w - 60, h - 2, "C/N0 [dB-Hz]")

        p.end()


# ─────────── G-Force Display ───────────

G = 9.80665  # m/s² per G

class GForcePanel(QWidget):
    """Large-font G-force display: X, Y, Z in G plus total G-load."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(40)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._gx = 0.0
        self._gy = 0.0
        self._gz = 0.0
        self._total = 0.0

    def update_data(self, d: SensorData):
        if d.acc:
            self._gx = d.acc[0] / G
            self._gy = d.acc[1] / G
            self._gz = d.acc[2] / G
            self._total = math.sqrt(self._gx**2 + self._gy**2 + self._gz**2)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor(0x11, 0x13, 0x18))

        # Border
        p.setPen(QPen(QColor(0x28, 0x2a, 0x30), 1))
        p.drawRect(0, 0, w - 1, h - 1)

        # Layout: split into 4 equal sections
        section_w = w // 4

        label_font = QFont("Monospace", 10)
        value_font = QFont("Monospace", 20)
        value_font.setBold(True)
        unit_font = QFont("Monospace", 9)

        items = [
            ("Gx", self._gx),
            ("Gy", self._gy),
            ("Gz", self._gz),
            ("G TOTAL", self._total),
        ]

        for i, (label, val) in enumerate(items):
            x = i * section_w
            cx = x + section_w // 2

            # Label
            p.setFont(label_font)
            p.setPen(QPen(QColor(0x4C, 0xA3, 0xDD), 1))
            fm = QFontMetrics(label_font)
            p.drawText(cx - fm.horizontalAdvance(label) // 2, 14, label)

            # Value
            p.setFont(value_font)
            fm_v = QFontMetrics(value_font)
            # Color: green near 1G total, yellow >1.5, red >2
            if i == 3:  # total
                if val > 2.0:
                    color = QColor(0xFF, 0x33, 0x33)
                elif val > 1.5:
                    color = QColor(0xFF, 0xCC, 0x00)
                else:
                    color = QColor(0x00, 0xCC, 0x00)
            else:
                color = QColor(0xDD, 0xDD, 0xDD)

            p.setPen(QPen(color, 1))
            txt = f"{val:+.3f}" if i < 3 else f"{val:.3f}"
            p.drawText(cx - fm_v.horizontalAdvance(txt) // 2, h - 10, txt)

            # Separator line
            if i > 0:
                p.setPen(QPen(QColor(0x28, 0x2a, 0x30), 1))
                p.drawLine(x, 4, x, h - 4)

        p.end()


# ─────────── Main data panel (flat grid, no scroll) ───────────

class RepeatButton(QPushButton):
    """Button that fires repeatedly while held, accelerating over time."""

    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._tick_count = 0

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        self._tick_count = 0
        self._timer.start(400)  # initial delay

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        self._timer.stop()

    def _on_tick(self):
        self._tick_count += 1
        if self._tick_count > 15:
            self._timer.setInterval(30)   # fast after ~3s
        elif self._tick_count > 5:
            self._timer.setInterval(80)   # medium after ~1.5s
        else:
            self._timer.setInterval(200)  # slow at start
        self.clicked.emit()


BTN_STYLE = """
QPushButton {
    background-color: #1a1d25; border: 1px solid #333; border-radius: 4px;
    padding: 6px 4px; color: #ddd; font-size: 10px; font-family: monospace;
    min-height: 22px;
}
QPushButton:hover { background-color: #252830; border-color: #4CA3DD; }
QPushButton:pressed { background-color: #4CA3DD; color: black; }
"""


class QuickButtonPanel(QWidget):
    """Quick-access buttons mirroring keyboard shortcuts."""

    # Signals emitted when buttons are clicked
    level_ahrs = pyqtSignal()
    reset_level = pyqtSignal()
    calibrate = pyqtSignal()
    toggle_units = pyqtSignal()
    hdg_sync = pyqtSignal()
    hdg_dec = pyqtSignal()
    hdg_inc = pyqtSignal()
    open_settings = pyqtSignal()
    toggle_fullscreen = pyqtSignal()
    quit_app = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(BTN_STYLE)
        self.setFixedWidth(120)
        layout = QVBoxLayout(self)
        layout.setSpacing(1)
        layout.setContentsMargins(1, 0, 1, 0)

        # AHRS / Calibration
        ahrs_grp = QGroupBox("AHRS")
        ahrs_grp.setStyleSheet(GROUP_STYLE)
        ahrs_lay = QVBoxLayout(ahrs_grp)
        ahrs_lay.setSpacing(1)
        ahrs_lay.setContentsMargins(2, 10, 2, 2)

        btn_level = QPushButton("Level [Z]")
        btn_level.setToolTip("Set current attitude as level reference")
        btn_level.clicked.connect(self.level_ahrs.emit)
        ahrs_lay.addWidget(btn_level)

        btn_reset = QPushButton("Reset Level [R]")
        btn_reset.setToolTip("Clear level bias")
        btn_reset.clicked.connect(self.reset_level.emit)
        ahrs_lay.addWidget(btn_reset)

        btn_cal = QPushButton("Calibrate [C]")
        btn_cal.setToolTip("Orientation reset — keep device stationary")
        btn_cal.clicked.connect(self.calibrate.emit)
        ahrs_lay.addWidget(btn_cal)

        layout.addWidget(ahrs_grp)

        # Heading bug
        hdg_grp = QGroupBox("HDG Bug")
        hdg_grp.setStyleSheet(GROUP_STYLE)
        hdg_lay = QVBoxLayout(hdg_grp)
        hdg_lay.setSpacing(1)
        hdg_lay.setContentsMargins(2, 10, 2, 2)

        btn_sync = QPushButton("Sync [H]")
        btn_sync.setToolTip("Set heading bug to current heading")
        btn_sync.clicked.connect(self.hdg_sync.emit)
        hdg_lay.addWidget(btn_sync)

        hdg_row = QHBoxLayout()
        hdg_row.setSpacing(2)
        btn_dec = RepeatButton("\u2212 1°")
        btn_dec.setToolTip("Heading bug \u22121° (hold to repeat)")
        btn_dec.clicked.connect(self.hdg_dec.emit)
        hdg_row.addWidget(btn_dec)
        btn_inc = RepeatButton("+ 1°")
        btn_inc.setToolTip("Heading bug +1° (hold to repeat)")
        btn_inc.clicked.connect(self.hdg_inc.emit)
        hdg_row.addWidget(btn_inc)
        hdg_lay.addLayout(hdg_row)

        layout.addWidget(hdg_grp)

        # Display / App
        app_grp = QGroupBox("Display")
        app_grp.setStyleSheet(GROUP_STYLE)
        app_lay = QVBoxLayout(app_grp)
        app_lay.setSpacing(1)
        app_lay.setContentsMargins(2, 10, 2, 2)

        btn_units = QPushButton("Units [U]")
        btn_units.setToolTip("Toggle US / Metric")
        btn_units.clicked.connect(self.toggle_units.emit)
        app_lay.addWidget(btn_units)

        btn_settings = QPushButton("Settings [M]")
        btn_settings.setToolTip("Open settings dialog")
        btn_settings.clicked.connect(self.open_settings.emit)
        app_lay.addWidget(btn_settings)

        btn_fs = QPushButton("Fullscreen [F]")
        btn_fs.setToolTip("Toggle fullscreen")
        btn_fs.clicked.connect(self.toggle_fullscreen.emit)
        app_lay.addWidget(btn_fs)

        btn_quit = QPushButton("Quit [Q]")
        btn_quit.setToolTip("Exit application")
        btn_quit.clicked.connect(self.quit_app.emit)
        app_lay.addWidget(btn_quit)

        layout.addWidget(app_grp)
        layout.addStretch()


class DataPanelWidget(QWidget):
    """Horizontal bottom strip: buttons + flight data + sensors + system + GNSS."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background-color: #0A0C10;")

        outer = QVBoxLayout(self)
        outer.setSpacing(1)
        outer.setContentsMargins(1, 1, 1, 1)

        # G-force bar at top of data strip
        self.gforce_panel = GForcePanel()
        outer.addWidget(self.gforce_panel)

        root = QHBoxLayout()
        root.setSpacing(1)
        outer.addLayout(root, 1)

        # Quick-access buttons (leftmost, fixed narrow)
        self.button_panel = QuickButtonPanel()
        root.addWidget(self.button_panel, 0)

        # Flight state: orientation, velocity, position, baro
        col_flight = QVBoxLayout()
        col_flight.setSpacing(0)
        self.orientation_panel = OrientationPanel()
        self.velocity_panel = VelocityPanel()
        self.position_panel = PositionPanel()
        self.baro_panel = BaroPanel()
        col_flight.addWidget(self.orientation_panel)
        col_flight.addWidget(self.velocity_panel)
        col_flight.addWidget(self.position_panel)
        col_flight.addWidget(self.baro_panel)
        col_flight.addStretch()

        # Raw sensors: accel, gyro, mag
        col_raw = QVBoxLayout()
        col_raw.setSpacing(0)
        self.accel_panel = AccelPanel()
        self.gyro_panel = GyroPanel()
        self.mag_panel = MagPanel()
        col_raw.addWidget(self.accel_panel)
        col_raw.addWidget(self.gyro_panel)
        col_raw.addWidget(self.mag_panel)
        col_raw.addStretch()

        # System: device info, status
        col_sys = QVBoxLayout()
        col_sys.setSpacing(0)
        self.device_panel = DeviceInfoPanel()
        self.status_panel = StatusPanel()
        col_sys.addWidget(self.device_panel)
        col_sys.addWidget(self.status_panel)
        col_sys.addStretch()

        # GNSS
        col_gnss = QVBoxLayout()
        col_gnss.setSpacing(0)
        self.gnss_panel = GNSSPanel()
        col_gnss.addWidget(self.gnss_panel)
        col_gnss.addStretch()

        # Satellite bar graph
        self.sat_graph = SatelliteBarGraph()

        root.addLayout(col_flight, 1)
        root.addLayout(col_raw, 1)
        root.addLayout(col_sys, 1)
        root.addLayout(col_gnss, 1)
        root.addWidget(self.sat_graph, 2)

    def set_device_info(self, info: DeviceInfo):
        self.device_panel.set_info(info)

    def update_data(self, d: SensorData):
        self.orientation_panel.update_data(d)
        self.accel_panel.update_data(d)
        self.gyro_panel.update_data(d)
        self.mag_panel.update_data(d)
        self.velocity_panel.update_data(d)
        self.position_panel.update_data(d)
        self.baro_panel.update_data(d)
        self.gnss_panel.update_data(d)
        self.status_panel.update_data(d)
        self.sat_graph.update_data(d)
        self.gforce_panel.update_data(d)
