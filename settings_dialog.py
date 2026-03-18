#!/usr/bin/env python3
"""
MTi-G Settings Dialog — full Mark III legacy protocol configuration.
All available device parameters exposed.
"""

import struct
from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QPushButton, QComboBox, QSpinBox,
    QDoubleSpinBox, QCheckBox, QTabWidget, QWidget, QSlider,
    QLineEdit, QMessageBox, QScrollArea,
)
from PyQt6.QtCore import Qt, QUrl
from PyQt6.QtGui import QFont
from PyQt6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply

from sensors import MID, Baudrates
import pfd_widget
from map_widget import PROVIDERS as MAP_PROVIDERS

DIALOG_STYLE = """
QDialog { background-color: #0e1017; color: #ddd; }
QGroupBox {
    border: 1px solid #333; border-radius: 4px; margin-top: 10px;
    padding: 12px 6px 6px 6px; background-color: #111318;
    color: #4CA3DD; font-weight: bold; font-size: 12px;
}
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QLabel { color: #aaa; font-size: 11px; }
QPushButton {
    background-color: #1a1d25; border: 1px solid #333; border-radius: 4px;
    padding: 6px 14px; color: #ddd; font-size: 11px;
}
QPushButton:hover { background-color: #252830; border-color: #4CA3DD; }
QPushButton:pressed { background-color: #4CA3DD; color: black; }
QPushButton#danger { border-color: #c00; color: #f88; }
QPushButton#danger:hover { background-color: #400; border-color: #f00; }
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {
    background-color: #1a1d25; border: 1px solid #333; border-radius: 3px;
    padding: 4px 8px; color: #ddd; font-size: 11px; font-family: monospace;
}
QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover { border-color: #4CA3DD; }
QSlider::groove:horizontal { height: 4px; background: #333; border-radius: 2px; }
QSlider::handle:horizontal {
    background: #4CA3DD; width: 14px; height: 14px; margin: -5px 0; border-radius: 7px;
}
QCheckBox { color: #aaa; font-size: 11px; }
QCheckBox::indicator { width: 14px; height: 14px; border: 1px solid #555; border-radius: 3px; background: #1a1d25; }
QCheckBox::indicator:checked { background: #4CA3DD; border-color: #4CA3DD; }
QTabWidget::pane { border: 1px solid #333; background: #0e1017; }
QTabBar::tab { background: #1a1d25; border: 1px solid #333; padding: 6px 14px; color: #888; font-size: 11px; }
QTabBar::tab:selected { background: #111318; color: #4CA3DD; border-bottom: 2px solid #4CA3DD; }
"""


def _ro_label(text):
    """Read-only value label."""
    lbl = QLabel(text)
    lbl.setStyleSheet("color: #ddd; font-family: monospace; font-weight: bold;")
    return lbl


def _section_label(text):
    lbl = QLabel(text)
    lbl.setStyleSheet("color: #666; font-size: 10px;")
    return lbl


class SettingsDialog(QDialog):

    def __init__(self, sensor, pfd, vario=None, map_view=None, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.Dialog)
        self.sensor = sensor
        self.pfd = pfd
        self.vario = vario
        self.map_view = map_view
        self.info = sensor.device_info
        self.setMinimumSize(620, 520)
        self.setStyleSheet(DIALOG_STYLE)

        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        tabs = QTabWidget()
        tabs.addTab(self._build_device_tab(), "Device")
        tabs.addTab(self._build_output_tab(), "Output")
        tabs.addTab(self._build_filter_tab(), "Filter")
        tabs.addTab(self._build_gps_tab(), "GPS / Mag")
        tabs.addTab(self._build_display_tab(), "Display")
        if self.map_view is not None:
            tabs.addTab(self._build_map_tab(), "Map")
        tabs.addTab(self._build_commands_tab(), "Commands")
        layout.addWidget(tabs)

        btn_row = QHBoxLayout()
        btn_row.addStretch()
        close_btn = QPushButton("Close  [M]")
        close_btn.clicked.connect(self.accept)
        btn_row.addWidget(close_btn)
        layout.addLayout(btn_row)

    # ─── Device tab ───
    def _build_device_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        grp = QGroupBox("Device Information (read-only)")
        grid = QGridLayout(grp)
        grid.setSpacing(3)
        i = self.info
        fields = [
            ("Device ID", f"0x{i.device_id:08X}"),
            ("Product Code", i.product_code or "--"),
            ("Firmware", f"{i.fw_major}.{i.fw_minor}.{i.fw_rev}"),
            ("Hardware", f"{i.hw_major}.{i.hw_minor}" if i.hw_major else "--"),
            ("Protocol", "Mark III (Legacy MTData 0x32)"),
            ("Baudrate", f"{Baudrates._map.get(i.baudrate_id, '?')} bps (ID 0x{i.baudrate_id:02X})"),
            ("Location ID", f"{i.location_id}"),
            ("Error Mode", f"0x{i.error_mode:04X}"),
            ("Transmit Delay", f"{i.transmit_delay}"),
        ]
        for row, (label, value) in enumerate(fields):
            grid.addWidget(QLabel(label), row, 0)
            grid.addWidget(_ro_label(value), row, 1)
        layout.addWidget(grp)

        # Configuration block
        cfg_grp = QGroupBox("Current Configuration")
        cfg_grid = QGridLayout(cfg_grp)
        cfg_grid.setSpacing(3)
        period_hz = 115200.0 / i.period if i.period > 0 else 0
        eff_hz = period_hz / (i.skip_factor + 1) if i.skip_factor >= 0 else period_hz
        q = i.alignment_rotation
        oq = i.object_alignment
        cfg_fields = [
            ("Output Mode", f"0x{i.output_mode:04X}"),
            ("Output Settings", f"0x{i.output_settings:08X}"),
            ("Data Length", f"{i.data_length} bytes"),
            ("RAWGPS", "Enabled" if i.has_rawgps else "Disabled"),
            ("Period", f"{i.period} (= {period_hz:.1f} Hz)"),
            ("Skip Factor", f"{i.skip_factor} (effective {eff_hz:.1f} Hz)"),
            ("Processing Flags", f"0x{i.processing_flags:02X}"),
            ("Filter Scenario", f"{i.current_scenario}"),
            ("Gravity Magnitude", f"{i.gravity_magnitude:.5f} m/s²"),
            ("Sensor Alignment", f"({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f})"),
            ("Object Alignment", f"({oq[0]:.4f}, {oq[1]:.4f}, {oq[2]:.4f}, {oq[3]:.4f})"),
            ("GPS Lever Arm", f"({i.lever_arm_gps[0]:.3f}, {i.lever_arm_gps[1]:.3f}, {i.lever_arm_gps[2]:.3f}) m"),
            ("Magnetic Declination", f"{i.magnetic_declination:.3f}°"),
            ("Sync Out Mode", f"0x{i.sync_out_mode:04X}"),
            ("Sync Out Skip", f"{i.sync_out_skip_factor}"),
            ("Sync Out Offset", f"{i.sync_out_offset} ns"),
            ("Sync Out Pulse Width", f"{i.sync_out_pulse_width} ns"),
        ]
        for row, (label, value) in enumerate(cfg_fields):
            cfg_grid.addWidget(QLabel(label), row, 0)
            cfg_grid.addWidget(_ro_label(value), row, 1)
        layout.addWidget(cfg_grp)
        layout.addStretch()
        return w

    # ─── Output tab ───
    def _build_output_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.addWidget(_section_label(
            "Changes require restart. Settings are sent to device and saved in non-volatile memory."
        ))

        # Sample rate
        rate_grp = QGroupBox("Sample Rate")
        rate_layout = QHBoxLayout(rate_grp)
        rate_layout.addWidget(QLabel("Period (1/115200 s):"))
        self._period_spin = QSpinBox()
        self._period_spin.setRange(225, 11520)  # ~512 Hz to ~10 Hz
        self._period_spin.setValue(self.info.period if self.info.period > 0 else 1152)
        rate_layout.addWidget(self._period_spin)
        self._period_hz_label = _ro_label("")
        rate_layout.addWidget(self._period_hz_label)

        rate_layout.addWidget(QLabel("Skip:"))
        self._skip_spin = QSpinBox()
        self._skip_spin.setRange(0, 65535)
        self._skip_spin.setValue(self.info.skip_factor)
        rate_layout.addWidget(self._skip_spin)

        # Now both spins exist, connect signals and update label
        self._period_spin.valueChanged.connect(self._update_period_label)
        self._skip_spin.valueChanged.connect(self._update_period_label)
        self._update_period_label()

        apply_rate_btn = QPushButton("Apply")
        apply_rate_btn.clicked.connect(self._apply_rate)
        rate_layout.addWidget(apply_rate_btn)
        layout.addWidget(rate_grp)

        # Baudrate
        baud_grp = QGroupBox("Baudrate")
        baud_layout = QHBoxLayout(baud_grp)
        baud_layout.addWidget(QLabel("Baudrate:"))
        self._baud_combo = QComboBox()
        sorted_bauds = sorted(Baudrates._map.items(), key=lambda x: x[1])
        for bid, bps in sorted_bauds:
            self._baud_combo.addItem(f"{bps}", bid)
            if bid == self.info.baudrate_id:
                self._baud_combo.setCurrentIndex(self._baud_combo.count() - 1)
        baud_layout.addWidget(self._baud_combo)
        apply_baud_btn = QPushButton("Apply (needs reconnect)")
        apply_baud_btn.clicked.connect(self._apply_baud)
        baud_layout.addWidget(apply_baud_btn)
        layout.addWidget(baud_grp)

        # Transmit Delay
        td_grp = QGroupBox("Transmit Delay")
        td_layout = QHBoxLayout(td_grp)
        td_layout.addWidget(QLabel("Delay (RS232 units):"))
        self._td_spin = QSpinBox()
        self._td_spin.setRange(0, 65535)
        self._td_spin.setValue(self.info.transmit_delay)
        td_layout.addWidget(self._td_spin)
        apply_td = QPushButton("Apply")
        apply_td.clicked.connect(lambda: (
            self.sensor.apply_setting(MID.SetTransmitDelay, struct.pack('!H', self._td_spin.value())),
            self._set_status("Transmit delay sent"),
        ))
        td_layout.addWidget(apply_td)
        layout.addWidget(td_grp)

        # Sync Out Settings
        sync_grp = QGroupBox("Sync Out")
        sync_grid = QGridLayout(sync_grp)
        sync_grid.setSpacing(4)

        sync_grid.addWidget(QLabel("Mode:"), 0, 0)
        self._sync_mode = QSpinBox()
        self._sync_mode.setRange(0, 65535)
        self._sync_mode.setValue(self.info.sync_out_mode)
        sync_grid.addWidget(self._sync_mode, 0, 1)

        sync_grid.addWidget(QLabel("Skip factor:"), 0, 2)
        self._sync_skip = QSpinBox()
        self._sync_skip.setRange(0, 65535)
        self._sync_skip.setValue(self.info.sync_out_skip_factor)
        sync_grid.addWidget(self._sync_skip, 0, 3)

        sync_grid.addWidget(QLabel("Offset (ns):"), 1, 0)
        self._sync_offset = QSpinBox()
        self._sync_offset.setRange(0, 2147483647)
        self._sync_offset.setValue(self.info.sync_out_offset)
        sync_grid.addWidget(self._sync_offset, 1, 1)

        sync_grid.addWidget(QLabel("Pulse width (ns):"), 1, 2)
        self._sync_pw = QSpinBox()
        self._sync_pw.setRange(0, 2147483647)
        self._sync_pw.setValue(self.info.sync_out_pulse_width)
        sync_grid.addWidget(self._sync_pw, 1, 3)

        apply_sync = QPushButton("Apply")
        apply_sync.clicked.connect(self._apply_sync_out)
        sync_grid.addWidget(apply_sync, 2, 0, 1, 4)
        layout.addWidget(sync_grp)

        layout.addStretch()
        return w

    def _update_period_label(self):
        p = self._period_spin.value()
        s = self._skip_spin.value()
        hz = 115200.0 / p
        eff = hz / (s + 1)
        self._period_hz_label.setText(f"= {hz:.1f} Hz, eff {eff:.1f} Hz")

    def _apply_rate(self):
        p = self._period_spin.value()
        s = self._skip_spin.value()
        self.sensor.apply_setting(MID.SetPeriod, struct.pack('!H', p))
        self.sensor.apply_setting(MID.SetOutputSkipFactor, struct.pack('!H', s))
        self._set_status(f"Period={p}, Skip={s} sent (restart required)")

    def _apply_baud(self):
        bid = self._baud_combo.currentData()
        self.sensor.apply_setting(MID.SetBaudrate, struct.pack('!B', bid))
        bps = Baudrates._map.get(bid, '?')
        self._set_status(f"Baudrate {bps} sent — reconnect with new baud rate")

    def _apply_sync_out(self):
        data = struct.pack('!HHII',
                           self._sync_mode.value(),
                           self._sync_skip.value(),
                           self._sync_offset.value(),
                           self._sync_pw.value())
        self.sensor.apply_setting(MID.SetSyncOutSettings, data)
        self._set_status("Sync out settings sent")

    # ─── Filter tab ───
    def _build_filter_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        # XKF Scenario / Filter Profile
        scen_grp = QGroupBox("XKF Filter Profile / Scenario")
        scen_layout = QVBoxLayout(scen_grp)

        cur_label = QLabel(f"Current scenario ID: {self.info.current_scenario}")
        cur_label.setStyleSheet("color: #4CA3DD; font-weight: bold;")
        scen_layout.addWidget(cur_label)

        self._scenario_combo = QComboBox()
        if self.info.available_scenarios:
            for sc in self.info.available_scenarios:
                self._scenario_combo.addItem(
                    f"[{sc.profile_type}] {sc.label}", sc.profile_type
                )
                if sc.profile_type == self.info.current_scenario:
                    self._scenario_combo.setCurrentIndex(self._scenario_combo.count() - 1)
        else:
            self._scenario_combo.addItem("(no scenarios available)")
        scen_layout.addWidget(self._scenario_combo)

        apply_scen_btn = QPushButton("Apply Scenario")
        apply_scen_btn.clicked.connect(self._apply_scenario)
        scen_layout.addWidget(apply_scen_btn)
        layout.addWidget(scen_grp)

        # Gravity Magnitude
        grav_grp = QGroupBox("Gravity Magnitude")
        grav_layout = QHBoxLayout(grav_grp)
        grav_layout.addWidget(QLabel("Local gravity [m/s²]:"))
        self._grav_spin = QDoubleSpinBox()
        self._grav_spin.setRange(9.70, 9.85)
        self._grav_spin.setDecimals(5)
        self._grav_spin.setSingleStep(0.0001)
        self._grav_spin.setValue(self.info.gravity_magnitude)
        grav_layout.addWidget(self._grav_spin)
        apply_grav = QPushButton("Apply")
        apply_grav.clicked.connect(lambda: (
            self.sensor.apply_setting(MID.SetGravityMagnitude, struct.pack('!f', self._grav_spin.value())),
            self._set_status("Gravity magnitude sent"),
        ))
        grav_layout.addWidget(apply_grav)
        layout.addWidget(grav_grp)

        # Alignment Rotation
        align_grp = QGroupBox("Sensor Alignment Rotation (quaternion)")
        align_grid = QGridLayout(align_grp)
        q = self.info.alignment_rotation
        self._align_q = []
        for i, (label, val) in enumerate([("w", q[0]), ("x", q[1]), ("y", q[2]), ("z", q[3])]):
            align_grid.addWidget(QLabel(label), 0, i)
            spin = QDoubleSpinBox()
            spin.setRange(-1.0, 1.0)
            spin.setDecimals(6)
            spin.setSingleStep(0.001)
            spin.setValue(val)
            align_grid.addWidget(spin, 1, i)
            self._align_q.append(spin)
        apply_align = QPushButton("Apply")
        apply_align.clicked.connect(self._apply_alignment)
        align_grid.addWidget(apply_align, 2, 0, 1, 4)
        layout.addWidget(align_grp)

        # Processing Flags
        pf_grp = QGroupBox("Processing Flags")
        pf_layout = QHBoxLayout(pf_grp)
        pf_layout.addWidget(QLabel("Flags (hex):"))
        self._pf_spin = QSpinBox()
        self._pf_spin.setRange(0, 255)
        self._pf_spin.setDisplayIntegerBase(16)
        self._pf_spin.setPrefix("0x")
        self._pf_spin.setValue(self.info.processing_flags)
        pf_layout.addWidget(self._pf_spin)
        apply_pf = QPushButton("Apply")
        apply_pf.clicked.connect(lambda: (
            self.sensor.apply_setting(MID.SetProcessingFlags, struct.pack('!H', self._pf_spin.value())),
            self._set_status("Processing flags sent"),
        ))
        pf_layout.addWidget(apply_pf)
        layout.addWidget(pf_grp)

        # Object Alignment
        obj_grp = QGroupBox("Object Alignment Rotation (quaternion)")
        obj_grid = QGridLayout(obj_grp)
        oq = self.info.object_alignment
        self._obj_q = []
        for i, (label, val) in enumerate([("w", oq[0]), ("x", oq[1]), ("y", oq[2]), ("z", oq[3])]):
            obj_grid.addWidget(QLabel(label), 0, i)
            spin = QDoubleSpinBox()
            spin.setRange(-1.0, 1.0)
            spin.setDecimals(6)
            spin.setSingleStep(0.001)
            spin.setValue(val)
            obj_grid.addWidget(spin, 1, i)
            self._obj_q.append(spin)
        apply_obj = QPushButton("Apply")
        apply_obj.clicked.connect(self._apply_object_alignment)
        obj_grid.addWidget(apply_obj, 2, 0, 1, 4)
        layout.addWidget(obj_grp)

        # Accel smoothing (display-side)
        smooth_grp = QGroupBox("Accelerometer Smoothing (display)")
        smooth_layout = QHBoxLayout(smooth_grp)
        smooth_layout.addWidget(QLabel("Responsive"))
        self._smooth_slider = QSlider(Qt.Orientation.Horizontal)
        self._smooth_slider.setRange(0, 95)
        self._smooth_slider.setValue(int(self.pfd._lp_alpha * 100))
        self._smooth_slider.valueChanged.connect(self._on_smooth_changed)
        smooth_layout.addWidget(self._smooth_slider)
        smooth_layout.addWidget(QLabel("Smooth"))
        self._smooth_val = _ro_label(f"{self.pfd._lp_alpha:.2f}")
        smooth_layout.addWidget(self._smooth_val)
        layout.addWidget(smooth_grp)

        layout.addStretch()
        return w

    def _apply_scenario(self):
        sc_type = self._scenario_combo.currentData()
        if sc_type is not None:
            self.sensor.apply_setting(MID.SetCurrentScenario, struct.pack('!H', sc_type))
            self._set_status(f"Scenario {sc_type} sent")

    def _apply_alignment(self):
        qw, qx, qy, qz = [s.value() for s in self._align_q]
        self.sensor.apply_setting(MID.SetAlignmentRotation, struct.pack('!ffff', qw, qx, qy, qz))
        self._set_status("Alignment rotation sent")

    def _apply_object_alignment(self):
        qw, qx, qy, qz = [s.value() for s in self._obj_q]
        self.sensor.apply_setting(MID.SetObjectAlignment, struct.pack('!ffff', qw, qx, qy, qz))
        self._set_status("Object alignment sent")

    def _on_smooth_changed(self, value):
        alpha = value / 100.0
        self.pfd._lp_alpha = alpha
        self._smooth_val.setText(f"{alpha:.2f}")

    # ─── GPS / Mag tab ───
    def _build_gps_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        # GPS Lever Arm
        la_grp = QGroupBox("GPS Antenna Lever Arm (sensor frame, meters)")
        la_grid = QGridLayout(la_grp)
        la = self.info.lever_arm_gps
        self._la_spins = []
        for i, (label, val) in enumerate([("X", la[0]), ("Y", la[1]), ("Z", la[2])]):
            la_grid.addWidget(QLabel(label), 0, i)
            spin = QDoubleSpinBox()
            spin.setRange(-10.0, 10.0)
            spin.setDecimals(4)
            spin.setSingleStep(0.01)
            spin.setValue(val)
            la_grid.addWidget(spin, 1, i)
            self._la_spins.append(spin)
        apply_la = QPushButton("Apply")
        apply_la.clicked.connect(lambda: (
            self.sensor.apply_setting(MID.SetLeverArmGPS, struct.pack('!fff', *[s.value() for s in self._la_spins])),
            self._set_status("GPS lever arm sent"),
        ))
        la_grid.addWidget(apply_la, 2, 0, 1, 3)
        layout.addWidget(la_grp)

        # Magnetic Declination
        mag_grp = QGroupBox("Magnetic Declination")
        mag_layout = QHBoxLayout(mag_grp)
        mag_layout.addWidget(QLabel("Declination [deg]:"))
        self._decl_spin = QDoubleSpinBox()
        self._decl_spin.setRange(-180.0, 180.0)
        self._decl_spin.setDecimals(3)
        self._decl_spin.setSingleStep(0.1)
        self._decl_spin.setValue(self.info.magnetic_declination)
        mag_layout.addWidget(self._decl_spin)
        apply_decl = QPushButton("Apply")
        apply_decl.clicked.connect(lambda: (
            self.sensor.apply_setting(MID.SetMagneticDeclination, struct.pack('!f', self._decl_spin.value())),
            self._set_status("Magnetic declination sent"),
        ))
        mag_layout.addWidget(apply_decl)
        layout.addWidget(mag_grp)

        # Compass Course Correction
        ccc_grp = QGroupBox("Compass Course Correction")
        ccc_layout = QVBoxLayout(ccc_grp)
        ccc_hint = QLabel(
            "Manual heading offset to correct for residual compass error after "
            "calibration. Applied on top of magnetic declination."
        )
        ccc_hint.setWordWrap(True)
        ccc_hint.setStyleSheet("color: #666; font-size: 10px;")
        ccc_layout.addWidget(ccc_hint)
        ccc_row = QHBoxLayout()
        ccc_row.addWidget(QLabel("Offset [deg]:"))
        self._compass_offset_spin = QDoubleSpinBox()
        self._compass_offset_spin.setRange(-180.0, 180.0)
        self._compass_offset_spin.setDecimals(1)
        self._compass_offset_spin.setSingleStep(0.5)
        self._compass_offset_spin.setValue(self.pfd._compass_offset)
        self._compass_offset_spin.valueChanged.connect(
            lambda v: setattr(self.pfd, '_compass_offset', v))
        ccc_row.addWidget(self._compass_offset_spin)
        zero_btn = QPushButton("Zero")
        zero_btn.clicked.connect(lambda: self._compass_offset_spin.setValue(0.0))
        ccc_row.addWidget(zero_btn)
        ccc_layout.addLayout(ccc_row)
        layout.addWidget(ccc_grp)

        # In-run Compass Calibration
        icc_grp = QGroupBox("In-run Compass Calibration (ICC)")
        icc_layout = QVBoxLayout(icc_grp)
        icc_hint = QLabel(
            "Slowly rotate the device through all orientations (yaw, pitch, roll) "
            "covering as many headings as possible. Continue for at least 30–60 seconds. "
            "Press Start, perform the rotation, then Store & Stop to save the calibration."
        )
        icc_hint.setWordWrap(True)
        icc_layout.addWidget(icc_hint)
        btn_row = QHBoxLayout()
        start_btn = QPushButton("Start ICC")
        start_btn.clicked.connect(lambda: self._set_status(self.sensor.send_icc_command(0)))
        btn_row.addWidget(start_btn)
        stop_btn = QPushButton("Stop (discard)")
        stop_btn.clicked.connect(lambda: self._set_status(self.sensor.send_icc_command(1)))
        btn_row.addWidget(stop_btn)
        store_btn = QPushButton("Store & Stop")
        store_btn.clicked.connect(lambda: self._set_status(self.sensor.send_icc_command(2)))
        btn_row.addWidget(store_btn)
        icc_layout.addLayout(btn_row)
        layout.addWidget(icc_grp)

        # UTC Time
        utc_grp = QGroupBox("UTC Time")
        utc_layout = QVBoxLayout(utc_grp)
        utc_hint = QLabel(
            "Adjust the device's UTC clock offset in nanoseconds. "
            "Positive values advance the clock, negative values retard it."
        )
        utc_hint.setWordWrap(True)
        utc_layout.addWidget(utc_hint)
        utc_row = QHBoxLayout()
        utc_row.addWidget(QLabel("Offset [ns]:"))
        self._utc_offset_spin = QSpinBox()
        self._utc_offset_spin.setRange(-2147483647, 2147483647)
        self._utc_offset_spin.setValue(0)
        utc_row.addWidget(self._utc_offset_spin)
        apply_utc = QPushButton("Adjust UTC")
        apply_utc.clicked.connect(lambda: (
            self.sensor.apply_setting(MID.AdjustUTCTime, struct.pack('!i', self._utc_offset_spin.value())),
            self._set_status("UTC time adjustment sent"),
        ))
        utc_row.addWidget(apply_utc)
        utc_layout.addLayout(utc_row)
        layout.addWidget(utc_grp)

        layout.addStretch()
        return w

    # ─── Display tab ───
    def _build_display_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; }")
        w = QWidget()
        layout = QVBoxLayout(w)

        # Attitude Source
        att_grp = QGroupBox("Attitude Source")
        att_layout = QVBoxLayout(att_grp)
        att_hint = QLabel(
            "Accelerometer: drift-free, noisy under vibration/acceleration. "
            "EKF: fused IMU/GPS, smooth but may drift without GPS fix."
        )
        att_hint.setWordWrap(True)
        att_hint.setStyleSheet("color: #666; font-size: 10px;")
        att_layout.addWidget(att_hint)
        att_row = QHBoxLayout()
        att_row.addWidget(QLabel("Source:"))
        self._att_combo = QComboBox()
        self._att_combo.addItems(["Accelerometer (raw)", "EKF (fused)"])
        self._att_combo.setCurrentIndex(0 if self.pfd._att_source == "accel" else 1)
        self._att_combo.currentIndexChanged.connect(
            lambda idx: setattr(self.pfd, '_att_source', "accel" if idx == 0 else "ekf"))
        att_row.addWidget(self._att_combo)
        att_layout.addLayout(att_row)
        layout.addWidget(att_grp)

        # Units
        units_grp = QGroupBox("Units")
        units_layout = QHBoxLayout(units_grp)
        units_layout.addWidget(QLabel("Unit System:"))
        self._units_combo = QComboBox()
        self._units_combo.addItems(["US (knots / feet / ft/min)", "Metric (km/h / meters / m/s)"])
        self._units_combo.setCurrentIndex(1 if self.pfd._metric else 0)
        self._units_combo.currentIndexChanged.connect(self._on_units_changed)
        units_layout.addWidget(self._units_combo)
        layout.addWidget(units_grp)

        # Barometric
        baro_grp = QGroupBox("Barometric Altitude (QNH)")
        baro_layout = QHBoxLayout(baro_grp)
        baro_layout.addWidget(QLabel("Sea-level pressure [hPa]:"))
        self._p0_spin = QDoubleSpinBox()
        self._p0_spin.setRange(900.0, 1100.0)
        self._p0_spin.setDecimals(2)
        self._p0_spin.setSingleStep(1.0)
        self._p0_spin.setValue(self.sensor.p0_pa / 100.0)
        self._p0_spin.valueChanged.connect(lambda v: setattr(self.sensor, 'p0_pa', v * 100.0))
        baro_layout.addWidget(self._p0_spin)
        std_btn = QPushButton("STD 1013.25")
        std_btn.clicked.connect(lambda: self._p0_spin.setValue(1013.25))
        baro_layout.addWidget(std_btn)
        sensor_btn = QPushButton("Set from sensor")
        sensor_btn.clicked.connect(self._set_qnh_from_sensor)
        baro_layout.addWidget(sensor_btn)
        layout.addWidget(baro_grp)

        # Altitude source
        alt_grp = QGroupBox("Altitude Source")
        alt_layout = QHBoxLayout(alt_grp)
        alt_layout.addWidget(QLabel("Source:"))
        self._alt_combo = QComboBox()
        self._alt_combo.addItems(["Barometric (QNH corrected)", "GNSS (GPS altitude)"])
        self._alt_combo.setCurrentIndex(1 if self.pfd._alt_source == "gnss" else 0)
        self._alt_combo.currentIndexChanged.connect(
            lambda idx: setattr(self.pfd, '_alt_source', "gnss" if idx == 1 else "baro"))
        alt_layout.addWidget(self._alt_combo)
        layout.addWidget(alt_grp)

        # VSI source
        vsi_grp = QGroupBox("Vertical Speed Source")
        vsi_layout = QHBoxLayout(vsi_grp)
        vsi_layout.addWidget(QLabel("Source:"))
        self._vsi_combo = QComboBox()
        self._vsi_combo.addItems(["GNSS (GPS vertical velocity)", "Barometric (pressure-derived)"])
        self._vsi_combo.setCurrentIndex(1 if self.pfd._vsi_source == "baro" else 0)
        self._vsi_combo.currentIndexChanged.connect(
            lambda idx: setattr(self.pfd, '_vsi_source', "baro" if idx == 1 else "gnss"))
        vsi_layout.addWidget(self._vsi_combo)
        layout.addWidget(vsi_grp)

        # Variometer Audio
        if self.vario is not None:
            vario_grp = QGroupBox("Variometer Audio")
            vario_layout = QVBoxLayout(vario_grp)
            vario_hint = QLabel(
                "Glider-style vario sound: beeping tone in climb (faster = stronger lift), "
                "continuous low tone in sink, silent in dead band."
            )
            vario_hint.setWordWrap(True)
            vario_hint.setStyleSheet("color: #666; font-size: 10px;")
            vario_layout.addWidget(vario_hint)
            row1 = QHBoxLayout()
            self._vario_cb = QCheckBox("Enable vario audio")
            self._vario_cb.setChecked(self.vario.enabled)
            self._vario_cb.toggled.connect(self._on_vario_toggled)
            row1.addWidget(self._vario_cb)
            vario_layout.addLayout(row1)
            row2 = QHBoxLayout()
            row2.addWidget(QLabel("Volume:"))
            self._vario_vol = QSlider(Qt.Orientation.Horizontal)
            self._vario_vol.setRange(0, 100)
            self._vario_vol.setValue(int(self.vario.volume * 100))
            self._vario_vol.valueChanged.connect(
                lambda v: setattr(self.vario, 'volume', v / 100.0))
            row2.addWidget(self._vario_vol)
            self._vario_vol_label = _ro_label(f"{int(self.vario.volume * 100)}%")
            self._vario_vol.valueChanged.connect(
                lambda v: self._vario_vol_label.setText(f"{v}%"))
            row2.addWidget(self._vario_vol_label)
            vario_layout.addLayout(row2)
            layout.addWidget(vario_grp)

        # Startup
        startup_grp = QGroupBox("Startup")
        startup_layout = QHBoxLayout(startup_grp)
        self._auto_zero_cb = QCheckBox("Auto-zero attitude after AHRS alignment")
        self._auto_zero_cb.setChecked(self.pfd._auto_zero_on_start)
        self._auto_zero_cb.toggled.connect(
            lambda v: setattr(self.pfd, '_auto_zero_on_start', v))
        startup_layout.addWidget(self._auto_zero_cb)
        layout.addWidget(startup_grp)

        # Speed tape color bands
        # V-speeds stored internally in knots; display in current unit
        self._kt_to_kmh = 3.6 / 1.94384  # ≈ 1.852
        metric = self.pfd._metric
        suffix = " km/h" if metric else " kt"
        k = self._kt_to_kmh if metric else 1.0

        self._spd_grp = QGroupBox(
            f"Speed Tape Color Bands ({'km/h' if metric else 'knots'})")
        spd_grid = QGridLayout(self._spd_grp)
        spd_grid.setSpacing(4)

        self._spd_enable = QCheckBox("Enable speed bands")
        self._spd_enable.setChecked(pfd_widget.SPD_BANDS_ENABLED)
        self._spd_enable.toggled.connect(self._apply_speed_bands)
        spd_grid.addWidget(self._spd_enable, 0, 0, 1, 4)

        spd_fields = [
            ("Vs0 (stall, flaps):", pfd_widget.SPD_VSO),
            ("Vs1 (stall, clean):", pfd_widget.SPD_VS1),
            ("Vfe (max flap):",     pfd_widget.SPD_VFE),
            ("Vno (max cruise):",   pfd_widget.SPD_VNO),
            ("Vne (never exceed):", pfd_widget.SPD_VNE),
        ]
        self._spd_spins = []
        for row, (label, val_kt) in enumerate(spd_fields, start=1):
            spd_grid.addWidget(QLabel(label), row, 0)
            spin = QSpinBox()
            spin.setRange(0, 999)
            spin.setSuffix(suffix)
            spin.setValue(int(round(val_kt * k)))
            spin.valueChanged.connect(self._apply_speed_bands)
            spd_grid.addWidget(spin, row, 1)
            self._spd_spins.append(spin)

        layout.addWidget(self._spd_grp)

        # Keyboard shortcuts
        keys_grp = QGroupBox("Keyboard Shortcuts")
        keys_grid = QGridLayout(keys_grp)
        keys_grid.setSpacing(2)
        for row, (key, desc) in enumerate([
            ("Z", "Level AHRS"), ("R", "Reset level"), ("C", "Reset orientation"),
            ("U", "Toggle units"), ("D", "Toggle debug panels"),
            ("M", "Settings"), ("F", "Fullscreen"), ("Q / Esc", "Quit"),
        ]):
            k = QLabel(key)
            k.setStyleSheet("color: #4CA3DD; font-family: monospace; font-weight: bold;")
            keys_grid.addWidget(k, row, 0)
            keys_grid.addWidget(QLabel(desc), row, 1)
        layout.addWidget(keys_grp)
        layout.addStretch()
        scroll.setWidget(w)
        return scroll

    def _set_qnh_from_sensor(self):
        """Set QNH to the current barometric pressure reading from the IMU."""
        data = self.sensor.get_latest()
        if data and data.pressure_pa and data.pressure_pa > 0:
            self._p0_spin.setValue(data.pressure_pa / 100.0)
        else:
            self._set_status("No pressure reading available")

    # ─── Map tab ───
    def _build_map_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; }")
        w = QWidget()
        layout = QVBoxLayout(w)

        # Base layer
        layer_grp = QGroupBox("Map Layers")
        layer_layout = QVBoxLayout(layer_grp)

        base_row = QHBoxLayout()
        base_row.addWidget(QLabel("Base layer:"))
        self._map_combo = QComboBox()
        for name in MAP_PROVIDERS:
            self._map_combo.addItem(name, name)
            if name == self.map_view.provider_name:
                self._map_combo.setCurrentIndex(self._map_combo.count() - 1)
        self._map_combo.currentIndexChanged.connect(self._on_map_provider_changed)
        base_row.addWidget(self._map_combo)
        layer_layout.addLayout(base_row)

        # Overlay
        self._openaip_overlay_cb = QCheckBox("OpenAIP overlay (airspaces, airports, navaids)")
        self._openaip_overlay_cb.setChecked(self.map_view._overlay_enabled)
        self._openaip_overlay_cb.toggled.connect(
            lambda v: setattr(self.map_view, '_overlay_enabled', v))
        layer_layout.addWidget(self._openaip_overlay_cb)
        layout.addWidget(layer_grp)

        # OpenAIP API Key
        key_grp = QGroupBox("OpenAIP API Key")
        key_layout = QVBoxLayout(key_grp)
        self._openaip_key = QLineEdit()
        self._openaip_key.setPlaceholderText("Enter API key from openaip.net")
        self._openaip_key.setText(self.map_view._api_keys.get("OpenAIP", ""))
        self._openaip_key.textChanged.connect(
            lambda t: self.map_view.set_api_key("OpenAIP", t))
        key_layout.addWidget(self._openaip_key)
        key_hint = QLabel("Get a free API key at openaip.net to enable aviation data.")
        key_hint.setWordWrap(True)
        key_hint.setStyleSheet("color: #666; font-size: 10px;")
        key_layout.addWidget(key_hint)
        layout.addWidget(key_grp)

        # Connection test
        test_grp = QGroupBox("Connection Status")
        test_layout = QVBoxLayout(test_grp)
        test_row = QHBoxLayout()
        test_btn = QPushButton("Test Connection")
        test_btn.clicked.connect(self._test_map_connection)
        test_row.addWidget(test_btn)
        self._map_status = QLabel("")
        self._map_status.setWordWrap(True)
        test_row.addWidget(self._map_status, 1)
        test_layout.addLayout(test_row)
        self._map_nam = QNetworkAccessManager(self)
        self._map_nam.finished.connect(self._on_map_test_reply)
        layout.addWidget(test_grp)

        # Synthetic Vision
        synvis_grp = QGroupBox("Synthetic Vision")
        synvis_layout = QVBoxLayout(synvis_grp)
        synvis_hint = QLabel(
            "3D terrain overlay on the attitude indicator using AWS Terrain Tiles "
            "(free, no API key). Terrain colored by elevation with red warning "
            "for terrain at or above aircraft altitude."
        )
        synvis_hint.setWordWrap(True)
        synvis_hint.setStyleSheet("color: #666; font-size: 10px;")
        synvis_layout.addWidget(synvis_hint)

        self._synvis_cb = QCheckBox("Enable synthetic vision (SYN VIS)")
        self._synvis_cb.setChecked(self.pfd._synvis_enabled)
        self._synvis_cb.toggled.connect(self._on_synvis_toggled)
        synvis_layout.addWidget(self._synvis_cb)

        range_row = QHBoxLayout()
        range_row.addWidget(QLabel("Forward range:"))
        self._synvis_range_spin = QSpinBox()
        self._synvis_range_spin.setRange(5000, 50000)
        self._synvis_range_spin.setSingleStep(1000)
        self._synvis_range_spin.setSuffix(" m")
        self._synvis_range_spin.setValue(self.pfd._synvis_range)
        self._synvis_range_spin.valueChanged.connect(
            lambda v: setattr(self.pfd, '_synvis_range', v))
        range_row.addWidget(self._synvis_range_spin)
        synvis_layout.addLayout(range_row)

        test_btn = QPushButton("Test: Inject Mountains 5 km Ahead")
        test_btn.clicked.connect(self._inject_test_terrain)
        synvis_layout.addWidget(test_btn)
        clear_test_btn = QPushButton("Clear Test Terrain")
        clear_test_btn.clicked.connect(self._clear_test_terrain)
        synvis_layout.addWidget(clear_test_btn)

        layout.addWidget(synvis_grp)

        # Cache
        cache_grp = QGroupBox("Tile Cache")
        cache_layout = QVBoxLayout(cache_grp)
        self._cache_info = QLabel("")
        self._cache_info.setStyleSheet("color: #aaa; font-size: 11px;")
        self._update_cache_info()
        cache_layout.addWidget(self._cache_info)
        clear_btn = QPushButton("Clear Map Cache")
        clear_btn.setObjectName("danger")
        clear_btn.clicked.connect(self._clear_map_cache)
        cache_layout.addWidget(clear_btn)
        layout.addWidget(cache_grp)

        layout.addStretch()
        scroll.setWidget(w)
        return scroll

    def _update_cache_info(self):
        if not hasattr(self, '_cache_info'):
            return
        from map_widget import CACHE_DIR
        import os, shutil
        count = 0
        size = 0
        if os.path.isdir(CACHE_DIR):
            for f in os.listdir(CACHE_DIR):
                fp = os.path.join(CACHE_DIR, f)
                if os.path.isfile(fp):
                    count += 1
                    size += os.path.getsize(fp)
        if size > 1024 * 1024:
            size_str = f"{size / 1024 / 1024:.1f} MB"
        else:
            size_str = f"{size / 1024:.0f} KB"
        try:
            disk = shutil.disk_usage(CACHE_DIR)
            free_gb = disk.free / (1024 ** 3)
            total_gb = disk.total / (1024 ** 3)
            disk_str = f"  |  Disk: {free_gb:.1f} GB free / {total_gb:.1f} GB total"
        except OSError:
            disk_str = ""
        self._cache_info.setText(f"{count} tiles, {size_str}{disk_str}")

    def _clear_map_cache(self):
        from map_widget import CACHE_DIR
        import os, shutil
        if os.path.isdir(CACHE_DIR):
            shutil.rmtree(CACHE_DIR)
            os.makedirs(CACHE_DIR, exist_ok=True)
        if self.map_view:
            self.map_view._tile_cache.clear()
            self.map_view._pending.clear()
            self.map_view.update()
        self._update_cache_info()
        self._set_status("Map cache cleared")

    def _inject_test_terrain(self):
        """Inject synthetic mountain range 5km ahead for SVS testing."""
        import math
        pfd = self.pfd
        terrain = pfd._terrain
        if terrain is None:
            self._set_status("No terrain provider")
            return

        # Enable synvis if not already
        pfd._synvis_enabled = True
        if hasattr(self, '_synvis_cb'):
            self._synvis_cb.setChecked(True)

        # Build a test grid with mountains
        gw = pfd._synvis_grid_w
        gh = pfd._synvis_grid_h
        rng = pfd._synvis_range
        aircraft_alt = pfd._altitude

        test_grid = []
        for row in range(gh):
            t = 1.0 - row / max(1, gh - 1)
            dist = max(30, rng * (t ** 2.0))
            row_data = []
            for col in range(gw):
                h_frac = (col / max(1, gw - 1)) - 0.5

                # Base terrain at 100m MSL
                elev = 100.0

                # Mountain range at 3-7km, peaking at 5km
                if 2000 < dist < 8000:
                    peak_t = 1.0 - abs(dist - 5000) / 3000.0
                    peak_t = max(0, peak_t)
                    # Ridge shape: higher in center, lower at edges
                    ridge = 1.0 - abs(h_frac) * 2.5
                    ridge = max(0, ridge) ** 0.8
                    # Multiple peaks
                    wave = 0.5 + 0.5 * math.sin(h_frac * 15 + dist * 0.001)
                    elev += peak_t * ridge * wave * 1500 + peak_t * ridge * 800

                # Foothills at 1.5-3km
                if 1000 < dist < 4000:
                    foot_t = 1.0 - abs(dist - 2500) / 1500.0
                    foot_t = max(0, foot_t)
                    wave2 = 0.5 + 0.5 * math.sin(h_frac * 25 + dist * 0.002)
                    elev += foot_t * wave2 * 200

                row_data.append((elev, dist))
            test_grid.append(row_data)

        # Inject directly into the display grid
        pfd._synvis_grid_cur = test_grid
        pfd._synvis_grid_tgt = None
        pfd._synvis_blend = 1.0
        # Prevent auto-recompute from overwriting it
        pfd._synvis_last_hdg = pfd._heading
        pfd._synvis_last_lat = pfd._gnss_lat
        pfd._synvis_last_lon = pfd._gnss_lon
        pfd._synvis_last_alt = pfd._altitude
        pfd._synvis_frozen = True
        pfd.update()
        self._set_status("Test terrain injected: mountains at 3-7 km")

    def _clear_test_terrain(self):
        """Clear injected test terrain and resume live data."""
        self.pfd._synvis_frozen = False
        self.pfd._synvis_grid_cur = None
        self.pfd._synvis_grid_tgt = None
        self.pfd._synvis_last_hdg = None
        self.pfd.update()
        self._set_status("Test terrain cleared — live data resumed")

    def _on_synvis_toggled(self, checked):
        self.pfd._synvis_enabled = checked
        self.pfd._synvis_frozen = False
        self.pfd._synvis_grid_cur = None
        self.pfd._synvis_grid_tgt = None
        self.pfd._synvis_last_hdg = None
        if checked and self.pfd._terrain and self.pfd._gnss_has_pos:
            self.pfd._terrain.prefetch_around(
                self.pfd._gnss_lat, self.pfd._gnss_lon, radius_tiles=3)

    def _on_map_provider_changed(self, idx):
        name = self._map_combo.currentData()
        if self.map_view and name:
            self.map_view.provider_name = name
            self._test_map_connection()

    def _test_map_connection(self):
        if not hasattr(self, '_map_status'):
            return
        name = self._map_combo.currentData()
        if not name:
            return
        provider = MAP_PROVIDERS.get(name)
        if not provider:
            return

        self._map_status.setText("Testing...")
        self._map_status.setStyleSheet("color: #FFCC00; font-size: 11px;")

        # Build a test tile URL (zoom 1, tile 0,0 — small, always exists)
        url_template = provider["url"]
        subdomains = provider.get("subdomains", ["a"])
        url = url_template.replace("{s}", subdomains[0]).replace("{z}", "1").replace("{x}", "0").replace("{y}", "0")

        if provider.get("api_key"):
            api_key = self.map_view._api_keys.get(name, "") if self.map_view else ""
            url = url.replace("{key}", api_key)
            if not api_key:
                self._map_status.setText("No API key configured")
                self._map_status.setStyleSheet("color: #FF8800; font-size: 11px;")
                return

        request = QNetworkRequest(QUrl(url))
        request.setRawHeader(b"User-Agent", b"XsensMTiG-PFD/1.0")
        request.setAttribute(QNetworkRequest.Attribute.User, name)
        self._map_nam.get(request)

    def _on_map_test_reply(self, reply: QNetworkReply):
        if not hasattr(self, '_map_status'):
            reply.deleteLater()
            return
        provider = reply.request().attribute(QNetworkRequest.Attribute.User)
        status = reply.attribute(QNetworkRequest.Attribute.HttpStatusCodeAttribute)
        err = reply.error()

        if err == QNetworkReply.NetworkError.NoError and status == 200:
            self._map_status.setText(f"{provider}: OK (HTTP {status})")
            self._map_status.setStyleSheet("color: #00CC00; font-size: 11px;")
        elif status:
            self._map_status.setText(f"{provider}: HTTP {status}")
            self._map_status.setStyleSheet("color: #FF3333; font-size: 11px;")
        else:
            self._map_status.setText(f"{provider}: {reply.errorString()}")
            self._map_status.setStyleSheet("color: #FF3333; font-size: 11px;")
        reply.deleteLater()

    def _on_vario_toggled(self, checked):
        if self.vario:
            self.vario.enabled = checked

    def _on_units_changed(self, idx):
        """Unit combo changed — update PFD and refresh speed band spinboxes."""
        self.pfd._metric = (idx == 1)
        self._refresh_speed_spins()

    def _refresh_speed_spins(self):
        """Re-display speed band spinboxes in the current unit system."""
        metric = self.pfd._metric
        suffix = " km/h" if metric else " kt"
        k = self._kt_to_kmh if metric else 1.0
        self._spd_grp.setTitle(
            f"Speed Tape Color Bands ({'km/h' if metric else 'knots'})")
        kt_vals = [pfd_widget.SPD_VSO, pfd_widget.SPD_VS1,
                    pfd_widget.SPD_VFE, pfd_widget.SPD_VNO, pfd_widget.SPD_VNE]
        for spin, val_kt in zip(self._spd_spins, kt_vals):
            spin.blockSignals(True)
            spin.setSuffix(suffix)
            spin.setValue(int(round(val_kt * k)))
            spin.blockSignals(False)

    def _apply_speed_bands(self):
        """Update the PFD speed tape color bands live.
        Spinbox values are in display units — convert back to knots for storage."""
        pfd_widget.SPD_BANDS_ENABLED = self._spd_enable.isChecked()
        k = 1.0 / self._kt_to_kmh if self.pfd._metric else 1.0
        pfd_widget.SPD_VSO = int(round(self._spd_spins[0].value() * k))
        pfd_widget.SPD_VS1 = int(round(self._spd_spins[1].value() * k))
        pfd_widget.SPD_VFE = int(round(self._spd_spins[2].value() * k))
        pfd_widget.SPD_VNO = int(round(self._spd_spins[3].value() * k))
        pfd_widget.SPD_VNE = int(round(self._spd_spins[4].value() * k))

    # ─── Commands tab ───
    def _build_commands_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        # Orientation commands
        ori_grp = QGroupBox("Orientation")
        ori_layout = QHBoxLayout(ori_grp)
        cal_btn = QPushButton("Object Reset (C)")
        cal_btn.setToolTip("ResetOrientation 0x0003 — reset EKF orientation reference")
        cal_btn.clicked.connect(lambda: self._set_status(self.sensor.calibrate()))
        ori_layout.addWidget(cal_btn)
        zero_btn = QPushButton("Level AHRS (Z)")
        zero_btn.clicked.connect(self.pfd.zero_attitude)
        ori_layout.addWidget(zero_btn)
        reset_btn = QPushButton("Reset Level (R)")
        reset_btn.clicked.connect(self.pfd.reset_zero)
        ori_layout.addWidget(reset_btn)
        norot_btn = QPushButton("Set No Rotation")
        norot_btn.setToolTip("Tell the filter the device is stationary — improves convergence")
        norot_btn.clicked.connect(lambda: (
            self.sensor.apply_setting(MID.SetNoRotation, struct.pack('!H', 0x0000)),
            self._set_status("SetNoRotation sent"),
        ))
        ori_layout.addWidget(norot_btn)
        layout.addWidget(ori_grp)

        # Device commands
        dev_grp = QGroupBox("Device")
        dev_layout = QHBoxLayout(dev_grp)
        reset_dev = QPushButton("Reset Device")
        reset_dev.setObjectName("danger")
        reset_dev.setToolTip("MID 0x40 — full device reset, will disconnect")
        reset_dev.clicked.connect(self._reset_device)
        dev_layout.addWidget(reset_dev)

        factory_btn = QPushButton("Restore Factory Defaults")
        factory_btn.setObjectName("danger")
        factory_btn.setToolTip("MID 0x0E — restore all settings to factory defaults")
        factory_btn.clicked.connect(self._restore_factory)
        dev_layout.addWidget(factory_btn)
        layout.addWidget(dev_grp)

        # Location ID
        loc_grp = QGroupBox("Location ID")
        loc_layout = QHBoxLayout(loc_grp)
        loc_layout.addWidget(QLabel("ID:"))
        self._loc_spin = QSpinBox()
        self._loc_spin.setRange(0, 65535)
        self._loc_spin.setValue(self.info.location_id)
        loc_layout.addWidget(self._loc_spin)
        apply_loc = QPushButton("Apply")
        apply_loc.clicked.connect(lambda: (
            self.sensor.apply_setting(MID.SetLocationID, struct.pack('!H', self._loc_spin.value())),
            self._set_status("Location ID sent"),
        ))
        loc_layout.addWidget(apply_loc)
        layout.addWidget(loc_grp)

        # Error Mode
        err_grp = QGroupBox("Error Mode")
        err_layout = QHBoxLayout(err_grp)
        self._err_combo = QComboBox()
        self._err_combo.addItems([
            "0x0000 — Disable all errors",
            "0x0001 — Default (send error message)",
            "0x0002 — Send last valid data on error",
            "0x0003 — Send error + last valid data",
        ])
        self._err_combo.setCurrentIndex(min(self.info.error_mode, 3))
        err_layout.addWidget(self._err_combo)
        apply_err = QPushButton("Apply")
        apply_err.clicked.connect(lambda: (
            self.sensor.apply_setting(MID.SetErrorMode, struct.pack('!H', self._err_combo.currentIndex())),
            self._set_status("Error mode sent"),
        ))
        err_layout.addWidget(apply_err)
        layout.addWidget(err_grp)

        layout.addStretch()
        return w

    def _reset_device(self):
        ret = QMessageBox.warning(self, "Reset Device",
                                  "This will reset the MTi-G. Connection will be lost.\nContinue?",
                                  QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if ret == QMessageBox.StandardButton.Yes:
            self.sensor.apply_setting(MID.Reset, b'')
            self._set_status("Device reset sent")

    def _restore_factory(self):
        ret = QMessageBox.warning(self, "Restore Factory Defaults",
                                  "This will erase all custom settings!\nContinue?",
                                  QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if ret == QMessageBox.StandardButton.Yes:
            self.sensor.apply_setting(MID.RestoreFactoryDef, b'')
            self._set_status("Factory defaults restored — restart required")

    def _set_status(self, msg):
        if hasattr(self.parent(), '_status_label'):
            self.parent()._status_label.setText(str(msg))
            self.parent()._status_label.setStyleSheet("color: #00CC00; padding-left: 8px;")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_M:
            self.accept()
        else:
            super().keyPressEvent(event)
