#!/usr/bin/env python3
"""
AHRS - Primary Flight Display for Xsens MTi
Bjoern Heller <tec@sixtopia.net>

Usage:
    python main.py [--port /dev/tty.usbserial-XSU5ZPZX] [--baud 230400] [--p0 101325.0]

Keys:
    Z   - Level AHRS (set current attitude as level reference)
    R   - Reset level bias
    C   - Calibrate (orientation reset, keep stationary)
    U   - Toggle units: US (knots/feet/fpm) vs Metric (km/h/m/m/s)
    D   - Toggle debug panels (data readouts + status bar)
    H   - Sync heading bug to current heading
    +/= - Heading bug +1°
    -   - Heading bug -1°
    M   - Settings menu
    F   - Toggle fullscreen
    Q   - Quit
    Esc - Quit
"""

import sys
import signal
import argparse
import threading

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QStatusBar, QPushButton, QDialog, QDoubleSpinBox, QGridLayout,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QKeySequence, QShortcut

from sensors import XSensSensor, SensorData, DeviceInfo
from pfd_widget import PFDWidget
import pfd_widget
from data_panels import DataPanelWidget
from settings_dialog import SettingsDialog
import config
from vario_audio import VarioAudio


BAR_STYLE = """
QWidget#pfd_bar { background-color: #1a1d25; border-top: 1px solid #333; }
"""

BAR_BTN_STYLE = """
QPushButton {
    background-color: #252830; border: 1px solid #444; border-radius: 3px;
    padding: 4px 2px; color: #ddd; font-size: 11px; font-family: monospace;
    min-height: 28px;
}
QPushButton:hover { background-color: #2d3040; }
QPushButton:pressed { background-color: #4CA3DD; color: black; }
QPushButton:disabled { background-color: #1a1d25; border-color: #333; color: #333; }
"""

POPUP_STYLE = """
QDialog { background-color: #0e1017; border: 2px solid #4CA3DD; }
QLabel { color: #aaa; font-size: 12px; }
QPushButton {
    background-color: #1a1d25; border: 1px solid #444; border-radius: 4px;
    padding: 10px 16px; color: #ddd; font-size: 13px; min-height: 32px;
}
QPushButton:hover { border-color: #4CA3DD; }
QPushButton:pressed { background-color: #4CA3DD; color: black; }
QPushButton#danger { border-color: #c00; color: #f88; }
QPushButton#danger:hover { background-color: #400; }
QPushButton#close_btn { background-color: #252830; border-color: #666; font-weight: bold; }
QDoubleSpinBox {
    background-color: #1a1d25; border: 1px solid #444; border-radius: 4px;
    padding: 8px; color: #ddd; font-size: 16px; font-family: monospace;
}
"""

NUM_SOFTKEYS = 12  # G1000 has 12 softkeys


class BarButton(QPushButton):
    """A single softkey in the bottom menu bar.
    Supports an optional activity indicator (small colored bar)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(BAR_BTN_STYLE)
        self._indicator = False
        self._indicator_color = "#00CC00"
        self._label = ""
        self.setEnabled(False)

    def configure(self, label: str, callback=None, indicator=False):
        self._label = label
        self._indicator = indicator
        self.setText(label)
        self.setEnabled(label != "")
        try:
            self.clicked.disconnect()
        except TypeError:
            pass
        if callback:
            self.clicked.connect(callback)

    def set_indicator(self, active: bool, color: str = "#00CC00"):
        self._indicator = active
        self._indicator_color = color
        self.update()

    def clear(self):
        self._label = ""
        self._indicator = False
        self.setText("")
        self.setEnabled(False)
        try:
            self.clicked.disconnect()
        except TypeError:
            pass

    def paintEvent(self, event):
        super().paintEvent(event)
        if self._indicator:
            from PyQt6.QtGui import QPainter, QColor
            p = QPainter(self)
            p.setRenderHint(QPainter.RenderHint.Antialiasing)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(self._indicator_color))
            bar_h = 3
            bar_w = int(self.width() * 0.6)
            x = (self.width() - bar_w) // 2
            y = self.height() - bar_h - 2
            p.drawRoundedRect(x, y, bar_w, bar_h, 1, 1)
            p.end()


class _PopupBase(QDialog):
    """Frameless popup base with a close button at the bottom."""

    def __init__(self, title_text: str, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.Dialog)
        self.setStyleSheet(POPUP_STYLE)
        self._outer = QVBoxLayout(self)
        self._outer.setContentsMargins(10, 10, 10, 10)

        title = QLabel(title_text)
        title.setStyleSheet("color: #4CA3DD; font-weight: bold; font-size: 14px;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._outer.addWidget(title)

        self.content = QVBoxLayout()
        self._outer.addLayout(self.content)

        self._outer.addStretch()
        close_btn = QPushButton("Close")
        close_btn.setObjectName("close_btn")
        close_btn.clicked.connect(self.accept)
        self._outer.addWidget(close_btn)


class QNHPopup(_PopupBase):
    def __init__(self, sensor, parent=None):
        super().__init__("QNH", parent)
        self.sensor = sensor
        self.setFixedSize(300, 240)

        self._spin = QDoubleSpinBox()
        self._spin.setRange(900.0, 1100.0)
        self._spin.setDecimals(2)
        self._spin.setSingleStep(1.0)
        self._spin.setSuffix(" hPa")
        self._spin.setValue(sensor.p0_pa / 100.0)
        self._spin.valueChanged.connect(lambda v: setattr(sensor, 'p0_pa', v * 100.0))
        self.content.addWidget(self._spin)

        btn_row = QHBoxLayout()
        for delta in [-1, -0.5, +0.5, +1]:
            sign = "+" if delta > 0 else ""
            btn = QPushButton(f"{sign}{delta}")
            btn.clicked.connect(lambda _, d=delta: self._spin.setValue(self._spin.value() + d))
            btn_row.addWidget(btn)
        self.content.addLayout(btn_row)

        btn_row2 = QHBoxLayout()
        std_btn = QPushButton("STD 1013.25")
        std_btn.clicked.connect(lambda: self._spin.setValue(1013.25))
        btn_row2.addWidget(std_btn)
        sensor_btn = QPushButton("From Sensor")
        sensor_btn.clicked.connect(self._from_sensor)
        btn_row2.addWidget(sensor_btn)
        self.content.addLayout(btn_row2)

    def _from_sensor(self):
        data = self.sensor.get_latest()
        if data and data.pressure_pa and data.pressure_pa > 0:
            self._spin.setValue(data.pressure_pa / 100.0)


class AHRSPopup(_PopupBase):
    def __init__(self, pfd, calibrate_fn, parent=None):
        super().__init__("AHRS", parent)
        self.setFixedSize(280, 240)

        level_btn = QPushButton("Level AHRS  [Z]")
        level_btn.clicked.connect(lambda: (pfd.zero_attitude(), self.accept()))
        self.content.addWidget(level_btn)

        reset_btn = QPushButton("Reset Level  [R]")
        reset_btn.clicked.connect(lambda: (pfd.reset_zero(), self.accept()))
        self.content.addWidget(reset_btn)

        cal_btn = QPushButton("Calibrate  [C]")
        cal_btn.setObjectName("danger")
        cal_btn.clicked.connect(lambda: (calibrate_fn(), self.accept()))
        self.content.addWidget(cal_btn)


class PFDBottomBar(QWidget):
    """G1000-style softkey bar with fixed 12-button layout.
    Buttons are addressed by index (0-11). Unassigned buttons
    are disabled and empty. Each button supports an activity indicator."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("pfd_bar")
        self.setStyleSheet(BAR_STYLE)
        self.setFixedHeight(42)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        self._buttons: list[BarButton] = []
        for i in range(NUM_SOFTKEYS):
            btn = BarButton(self)
            layout.addWidget(btn)
            self._buttons.append(btn)

    def button(self, index: int) -> BarButton:
        return self._buttons[index]

    def configure(self, index: int, label: str, callback=None, indicator=False):
        self._buttons[index].configure(label, callback, indicator)

    def clear_all(self):
        for btn in self._buttons:
            btn.clear()

    def set_indicator(self, index: int, active: bool, color: str = "#00CC00"):
        self._buttons[index].set_indicator(active, color)


class MainWindow(QMainWindow):

    def __init__(self, sensor: XSensSensor):
        super().__init__()
        self.sensor = sensor
        self.setWindowTitle("AHRS - Xsens MTi")
        self.setStyleSheet("background-color: #0A0C10; color: white;")

        # Main layout: PFD on top (wide), data strip below
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)

        # PFD (full width — sole visible widget in normal mode)
        self.pfd = PFDWidget()
        self.pfd.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        layout.addWidget(self.pfd, 6)

        # Bottom bar (always visible)
        self.bottom_bar = PFDBottomBar()
        layout.addWidget(self.bottom_bar, 0)

        # Data panels (bottom strip — hidden by default)
        self.data_panel = DataPanelWidget()
        layout.addWidget(self.data_panel, 3)
        self.data_panel.hide()

        # Connect quick-access button signals
        bp = self.data_panel.button_panel
        bp.level_ahrs.connect(self.pfd.zero_attitude)
        bp.reset_level.connect(self.pfd.reset_zero)
        bp.calibrate.connect(self._calibrate)
        bp.toggle_units.connect(self.pfd.toggle_units)
        bp.hdg_sync.connect(lambda: self.pfd.set_hdg_bug(self.pfd._heading))
        bp.hdg_dec.connect(lambda: self.pfd.set_hdg_bug((self.pfd._hdg_bug - 1) % 360))
        bp.hdg_inc.connect(lambda: self.pfd.set_hdg_bug((self.pfd._hdg_bug + 1) % 360))
        bp.open_settings.connect(self._show_settings)
        bp.toggle_fullscreen.connect(self._toggle_fullscreen)
        bp.quit_app.connect(self.close)

        # Softkey assignments (G1000-style: 12 fixed positions)
        #  0: UNITS   1: QNH   2: VARIO   3: AHRS
        #  4-10: (reserved)   11: MENU
        self.bottom_bar.configure(0, "US", self._toggle_units)
        self._update_units_label()
        self.bottom_bar.configure(1, "QNH", self._show_qnh)
        self.bottom_bar.configure(2, "VARIO", self._toggle_vario)
        self.bottom_bar.configure(3, "AHRS", self._show_ahrs)
        # 4-10 reserved for future use (empty/disabled)
        self.bottom_bar.configure(11, "MENU", self._show_settings)

        # Status bar (hidden by default)
        self.status_bar = QStatusBar()
        self.status_bar.setStyleSheet(
            "background-color: #111318; color: #888; font-size: 11px; font-family: monospace;"
        )
        self.setStatusBar(self.status_bar)
        self._status_label = QLabel("Connecting...")
        self._status_label.setStyleSheet("color: #FFCC00; padding-left: 8px;")
        self.status_bar.addWidget(self._status_label)
        self.status_bar.hide()

        self._debug_mode = False

        # Variometer audio
        self.vario = VarioAudio()

        # Keyboard shortcuts (window-level so they work regardless of focus)
        QShortcut(QKeySequence(Qt.Key.Key_Q), self, self.close)
        QShortcut(QKeySequence(Qt.Key.Key_Escape), self, self.close)
        QShortcut(QKeySequence(Qt.Key.Key_F), self, self._toggle_fullscreen)
        QShortcut(QKeySequence(Qt.Key.Key_C), self, self._calibrate)
        QShortcut(QKeySequence(Qt.Key.Key_U), self, self._toggle_units)
        QShortcut(QKeySequence(Qt.Key.Key_D), self, self._toggle_debug)
        QShortcut(QKeySequence(Qt.Key.Key_M), self, self._show_settings)
        QShortcut(QKeySequence(Qt.Key.Key_Z), self, self.pfd.zero_attitude)
        QShortcut(QKeySequence(Qt.Key.Key_R), self, self.pfd.reset_zero)
        QShortcut(QKeySequence(Qt.Key.Key_H), self, lambda: self.pfd.set_hdg_bug(self.pfd._heading))
        QShortcut(QKeySequence(Qt.Key.Key_Plus), self, lambda: self.pfd.set_hdg_bug((self.pfd._hdg_bug + 1) % 360))
        QShortcut(QKeySequence(Qt.Key.Key_Equal), self, lambda: self.pfd.set_hdg_bug((self.pfd._hdg_bug + 1) % 360))
        QShortcut(QKeySequence(Qt.Key.Key_Minus), self, lambda: self.pfd.set_hdg_bug((self.pfd._hdg_bug - 1) % 360))

        # Calibration result flag (polled from GUI thread)
        self._cal_result = None

        # Update timer (30 Hz UI refresh)
        self._timer = QTimer()
        self._timer.timeout.connect(self._update)
        self._timer.start(33)

    def _show_settings(self):
        if hasattr(self, '_settings_dlg') and self._settings_dlg is not None and self._settings_dlg.isVisible():
            self._settings_dlg.accept()
            self._settings_dlg = None
            return
        self._settings_dlg = SettingsDialog(self.sensor, self.pfd, self.vario, self)
        self._settings_dlg.finished.connect(lambda: setattr(self, '_settings_dlg', None))
        self._settings_dlg.show()

    def _show_qnh(self):
        dlg = QNHPopup(self.sensor, self)
        dlg.exec()

    def _show_ahrs(self):
        dlg = AHRSPopup(self.pfd, self._calibrate, self)
        dlg.exec()

    def _toggle_units(self):
        self.pfd.toggle_units()
        self._update_units_label()

    def _update_units_label(self):
        unit = "METRIC" if self.pfd._metric else "IMPERIAL"
        self.bottom_bar.button(0).setText(f"UNIT [{unit}]")

    def _toggle_vario(self):
        self.vario.enabled = not self.vario.enabled
        self.bottom_bar.set_indicator(2, self.vario.enabled)

    def _toggle_debug(self):
        self._debug_mode = not self._debug_mode
        self.data_panel.setVisible(self._debug_mode)
        self.status_bar.setVisible(self._debug_mode)

    def _toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def _calibrate(self):
        """Send heading reset to the Xsens — device handles it internally."""
        self._status_label.setText("CALIBRATING — keep device stationary...")
        self._status_label.setStyleSheet("color: #FF8800; padding-left: 8px;")
        self._cal_result = None

        def _run():
            self._cal_result = self.sensor.calibrate()

        threading.Thread(target=_run, daemon=True).start()

    def _update(self):
        # Check calibration result (set from background thread)
        if self._cal_result is not None:
            self._status_label.setText(self._cal_result)
            self._status_label.setStyleSheet("color: #00CC00; padding-left: 8px;")
            self._cal_result = None

        # Update connection status
        if self.sensor.connected and "Connecting" in self._status_label.text():
            info = self.sensor.device_info
            self._status_label.setText(
                f"Connected | {info.product_code} | FW {info.fw_major}.{info.fw_minor}.{info.fw_rev}"
            )
            self._status_label.setStyleSheet("color: #00CC00; padding-left: 8px;")
            self.data_panel.set_device_info(info)

        # Show errors (auto-clear once data flows)
        err = self.sensor.last_error
        if err and "Error" not in self._status_label.text():
            self._status_label.setText(f"Error: {err}")
            self._status_label.setStyleSheet("color: #FF3333; padding-left: 8px;")

        data = self.sensor.get_latest()
        if data is None:
            return

        # Clear stale error once data is flowing
        if "Error" in self._status_label.text():
            self.sensor._error = None
            info = self.sensor.device_info
            self._status_label.setText(
                f"Connected | {info.product_code} | FW {info.fw_major}.{info.fw_minor}.{info.fw_rev}"
            )
            self._status_label.setStyleSheet("color: #00CC00; padding-left: 8px;")

        self.pfd.set_data(data)
        self.data_panel.update_data(data)
        self.vario.update_vsi(self.pfd._vsi)

    def closeEvent(self, event):
        # Persist user settings
        config.save({
            "metric": self.pfd._metric,
            "p0_pa": self.sensor.p0_pa,
            "spd_bands_enabled": pfd_widget.SPD_BANDS_ENABLED,
            "spd_vso": pfd_widget.SPD_VSO,
            "spd_vs1": pfd_widget.SPD_VS1,
            "spd_vfe": pfd_widget.SPD_VFE,
            "spd_vno": pfd_widget.SPD_VNO,
            "spd_vne": pfd_widget.SPD_VNE,
            "auto_zero_on_start": self.pfd._auto_zero_on_start,
            "hdg_bug": self.pfd._hdg_bug,
            "att_source": self.pfd._att_source,
            "alt_source": self.pfd._alt_source,
            "vsi_source": self.pfd._vsi_source,
            "vario_enabled": self.vario.enabled,
            "vario_volume": self.vario.volume,
        })
        self.vario.enabled = False
        self.sensor.stop()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(
        description="Airbus PFD - Primary Flight Display for Xsens MTi"
    )
    parser.add_argument("--port", default="/dev/tty.usbserial-XSU5ZPZX",
                        help="Serial port (default: /dev/tty.usbserial-XSU5ZPZX)")
    parser.add_argument("--baud", default=230400, type=int,
                        help="Baud rate (default: 230400)")
    parser.add_argument("--p0", default=101325.0, type=float,
                        help="Sea-level pressure in Pa for baro altitude (default: 101325.0)")
    parser.add_argument("--windowed", action="store_true",
                        help="Start in windowed mode instead of fullscreen")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setApplicationName("Airbus PFD")
    app.setApplicationVersion("2.0")
    app.setStyle("Fusion")

    # Load persisted settings
    cfg = config.load()

    sensor = XSensSensor(port=args.port, baud=args.baud, p0_pa=cfg["p0_pa"])
    window = MainWindow(sensor)

    # Apply saved display settings
    window.pfd._metric = cfg["metric"]
    window.pfd._auto_zero_on_start = cfg["auto_zero_on_start"]
    pfd_widget.SPD_BANDS_ENABLED = cfg["spd_bands_enabled"]
    pfd_widget.SPD_VSO = cfg["spd_vso"]
    pfd_widget.SPD_VS1 = cfg["spd_vs1"]
    pfd_widget.SPD_VFE = cfg["spd_vfe"]
    pfd_widget.SPD_VNO = cfg["spd_vno"]
    pfd_widget.SPD_VNE = cfg["spd_vne"]
    window.pfd._hdg_bug = cfg["hdg_bug"]
    window.pfd._att_source = cfg["att_source"]
    window.pfd._alt_source = cfg["alt_source"]
    window.pfd._vsi_source = cfg["vsi_source"]
    window.vario.enabled = cfg["vario_enabled"]
    window.vario.volume = cfg["vario_volume"]
    window.bottom_bar.set_indicator(2, cfg["vario_enabled"])
    window._update_units_label()

    if args.windowed:
        window.resize(1600, 900)
        window.show()
    else:
        window.showFullScreen()

    sensor.start()

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(200)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
