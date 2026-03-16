# Xsens MTi-G Primary Flight Display

Airbus-style PFD (Primary Flight Display) driven by an Xsens MTi-G IMU/GPS over serial, built with PyQt6.

![Python](https://img.shields.io/badge/python-3.10+-blue)
![License](https://img.shields.io/badge/license-AGPL--3.0-blue)

## Features

- **Attitude indicator** — roll/pitch derived directly from accelerometer (bypasses EKF drift)
- **Speed tape** — ground speed from GPS/EKF velocity with low-pass filtering and noise squelch
- **Altitude tape** — barometric altitude from onboard pressure sensor, GPS altitude fallback
- **Heading tape** — magnetic heading from EKF
- **Vertical speed indicator**
- **Flight path vector** (FPV / velocity vector)
- **Data panels** — raw sensor readouts (accelerometer, gyroscope, magnetometer, GPS, pressure)
- **Settings dialog** — GPS lever arm, magnetic declination, sensor alignment, in-run compass calibration
- **Unit toggle** — knots/feet/fpm or km/h/meters/m/s

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
python main.py [--port /dev/tty.usbserial-XSU5ZPZX] [--baud 230400] [--p0 101325.0] [--windowed]
```

| Argument     | Default                            | Description                              |
|--------------|------------------------------------|------------------------------------------|
| `--port`     | `/dev/tty.usbserial-XSU5ZPZX`     | Serial port                              |
| `--baud`     | `230400`                           | Baud rate                                |
| `--p0`       | `101325.0`                         | Sea-level pressure (Pa) for baro altitude|
| `--windowed` | off (fullscreen)                   | Start in windowed mode                   |

## Keyboard Shortcuts

| Key | Action                                          |
|-----|-------------------------------------------------|
| Z   | Level AHRS (set current attitude as zero)       |
| R   | Reset level bias                                |
| C   | Calibrate (orientation reset, keep stationary)  |
| U   | Toggle units (US / Metric)                      |
| M   | Settings menu                                   |
| F   | Toggle fullscreen                               |
| Q   | Quit                                            |

## Architecture

| File                | Purpose                                      |
|---------------------|----------------------------------------------|
| `main.py`           | Application entry point, main window, keybindings |
| `sensors.py`        | Xsens MTi-G protocol, serial I/O, data parsing   |
| `pfd_widget.py`     | PFD rendering (attitude, tapes, FPV)              |
| `data_panels.py`    | Numeric data panel strip                          |
| `settings_dialog.py`| Device configuration UI                           |

## License

This project is dual-licensed:

- **Open source** — [GNU AGPL-3.0](LICENSE). You may use, modify, and distribute this software freely under AGPL terms. Any networked or distributed derivative work must also be released under AGPL-3.0 with full source code.
- **Commercial** — If you want to use this software in a proprietary/closed-source product without AGPL obligations, contact the author for a commercial license.

## Author

Bjoern Heller <tec@sixtopia.net>
