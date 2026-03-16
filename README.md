# Xsens MTi-G Primary Flight Display

G1000-style PFD (Primary Flight Display) driven by an Xsens MTi-G IMU/GPS over serial, built with PyQt6. Aligned to the Garmin G1000 Pilot's Guide (190-00498-08 Rev A).

![Python](https://img.shields.io/badge/python-3.10+-blue)
![License](https://img.shields.io/badge/license-AGPL--3.0-blue)

## Features

- **Attitude indicator** — roll/pitch derived directly from accelerometer (bypasses EKF drift), G1000 pitch ladder with 2.5° fine marks
- **Bank arc** — G1000 proportions with slip/skid indicator, auto-calibrated during AHRS alignment
- **Speed tape** — ground speed with rolling drum digits, LP filtering, noise squelch, G1000 color bands (white/green/yellow arcs, red Vne line, barber-pole overspeed)
- **Altitude tape** — barometric altitude with rolling drum digits, GPS altitude fallback
- **Compass rose** — rotating 360° HSI with upright labels, aircraft symbol, heading bug with G1000-style notch marker
- **Vertical speed indicator** — non-linear G1000 scale with value readout above 100 fpm
- **Flight path vector** (FPV / velocity vector)
- **Heading bug** — adjustable with +/- keys, cyan display with HDG readout
- **Fail flags** — red X overlay when data is unavailable (ATT, SPD, ALT, V/S, HDG)
- **AHRS alignment** — startup initialization with progress bar, auto-zero attitude and slip/skid
- **Data panels** — raw sensor readouts (accelerometer, gyroscope, magnetometer, GPS, pressure), toggle with D key
- **Settings dialog** — QNH, units, V-speed bands, auto-zero, GPS lever arm, magnetic declination, sensor alignment, in-run compass calibration
- **Persistent settings** — all user preferences saved to `config.json` and restored on next launch
- **Unit toggle** — knots/feet/fpm or km/h/meters/m/s (V-speed bands convert dynamically)

Communicates using the Xsens Mark III legacy protocol (MTData `0x32`).

## Requirements

- Python 3.10+
- Xsens MTi-G connected via USB-serial

## Install

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage

```bash
python main.py [--port /dev/tty.usbserial-XSU5ZPZX] [--baud 230400] [--windowed]
```

| Argument     | Default                            | Description                              |
|--------------|------------------------------------|------------------------------------------|
| `--port`     | `/dev/tty.usbserial-XSU5ZPZX`     | Serial port                              |
| `--baud`     | `230400`                           | Baud rate                                |
| `--windowed` | off (fullscreen)                   | Start in windowed mode                   |

QNH pressure, units, V-speeds, heading bug, and other settings are persisted in `config.json`.

## Keyboard Shortcuts

| Key   | Action                                          |
|-------|-------------------------------------------------|
| Z     | Level AHRS (set current attitude as zero)       |
| R     | Reset level bias                                |
| C     | Calibrate (orientation reset, keep stationary)  |
| U     | Toggle units (US / Metric)                      |
| D     | Toggle debug data panels                        |
| H     | Sync heading bug to current heading             |
| +/-   | Adjust heading bug by 1°                        |
| M     | Settings menu                                   |
| F     | Toggle fullscreen                               |
| Q/Esc | Quit                                            |

## Architecture

| File                | Purpose                                              |
|---------------------|------------------------------------------------------|
| `main.py`           | Application entry point, main window, keybindings    |
| `sensors.py`        | Xsens MTi-G protocol, serial I/O, data parsing      |
| `pfd_widget.py`     | G1000-style PFD rendering (attitude, tapes, HSI)     |
| `data_panels.py`    | Debug data panel strip                               |
| `settings_dialog.py`| Device and display configuration UI                  |
| `config.py`         | JSON settings persistence                            |

## License

This project is dual-licensed:

- **Open source** — [GNU AGPL-3.0](LICENSE). You may use, modify, and distribute this software freely under AGPL terms. Any networked or distributed derivative work must also be released under AGPL-3.0 with full source code.
- **Commercial** — If you want to use this software in a proprietary/closed-source product without AGPL obligations, contact the author for a commercial license.

## Author

Bjoern Heller <tec@sixtopia.net>
