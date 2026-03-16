#!/usr/bin/env python3
"""
Airbus PFD - Primary Flight Display for Xsens MTi
Bjoern Heller <tec@sixtopia.net>

Usage:
    python main.py [--port /dev/tty.usbserial-XSU5ZPZX] [--baud 230400] [--p0 101325.0]

Keys:
    Z   - Level AHRS (set current attitude as level reference)
    R   - Reset level bias
    C   - Calibrate (orientation reset, keep stationary)
    U   - Toggle units: US (knots/feet/fpm) vs Metric (km/h/m/m/s)
    M   - Settings menu
    F   - Toggle fullscreen
    Q   - Quit
"""

import sys
import signal
import argparse
import threading

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QStatusBar,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QKeySequence, QShortcut

from sensors import XSensSensor, SensorData, DeviceInfo
from pfd_widget import PFDWidget
from data_panels import DataPanelWidget
from settings_dialog import SettingsDialog


class MainWindow(QMainWindow):

    def __init__(self, sensor: XSensSensor):
        super().__init__()
        self.sensor = sensor
        self.setWindowTitle("Airbus PFD - Xsens MTi")
        self.setStyleSheet("background-color: #0A0C10; color: white;")

        # Main layout: PFD on top (wide), data strip below
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)

        # PFD (full width, takes most vertical space)
        self.pfd = PFDWidget()
        self.pfd.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        layout.addWidget(self.pfd, 6)

        # Data panels (bottom strip)
        self.data_panel = DataPanelWidget()
        layout.addWidget(self.data_panel, 3)

        # Status bar
        self.status_bar = QStatusBar()
        self.status_bar.setStyleSheet(
            "background-color: #111318; color: #888; font-size: 11px; font-family: monospace;"
        )
        self.setStatusBar(self.status_bar)
        self._status_label = QLabel("Connecting...")
        self._status_label.setStyleSheet("color: #FFCC00; padding-left: 8px;")
        self.status_bar.addWidget(self._status_label)

        # Keyboard shortcuts
        QShortcut(QKeySequence(Qt.Key.Key_Q), self, self.close)
        QShortcut(QKeySequence(Qt.Key.Key_Escape), self, self.close)
        QShortcut(QKeySequence(Qt.Key.Key_F), self, self._toggle_fullscreen)
        QShortcut(QKeySequence(Qt.Key.Key_C), self, self._calibrate)
        QShortcut(QKeySequence(Qt.Key.Key_U), self, lambda: self.pfd.toggle_units())
        QShortcut(QKeySequence(Qt.Key.Key_M), self, self._show_settings)

        # Calibration result flag (polled from GUI thread)
        self._cal_result = None

        # Update timer (30 Hz UI refresh)
        self._timer = QTimer()
        self._timer.timeout.connect(self._update)
        self._timer.start(33)

    def _show_settings(self):
        dlg = SettingsDialog(self.sensor, self.pfd, self)
        dlg.exec()

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

    def closeEvent(self, event):
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

    sensor = XSensSensor(port=args.port, baud=args.baud, p0_pa=args.p0)
    window = MainWindow(sensor)

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
