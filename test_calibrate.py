#!/usr/bin/env python3
"""Test SetNoRotation with write_ack (proper ACK handling among MTData stream)."""
import sys, time, math, struct
sys.path.insert(0, '.')
from sensors import (
    _read_msg, _write_ack, _write_req, _goto_config, _goto_measurement,
    MID, OutputMode, OutputSettings, _parse_mtdata,
)
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-XSU5ZPZX"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 230400

ser = serial.Serial(PORT, BAUD, timeout=0.2, write_timeout=0.2)
time.sleep(0.1)
ser.reset_input_buffer()
ser.reset_output_buffer()

# Configure
_goto_config(ser)
mode = (OutputMode.Temp | OutputMode.Calib | OutputMode.Orient |
        OutputMode.Auxiliary | OutputMode.Position | OutputMode.Velocity |
        OutputMode.Status | OutputMode.RAWGPS)
settings = (OutputSettings.Timestamp_SampleCnt |
            OutputSettings.Timestamp_UTCTime |
            OutputSettings.OrientMode_Euler |
            OutputSettings.CalibMode_AccGyrMag)
try:
    _write_ack(ser, MID.SetOutputMode, struct.pack('!H', mode))
    _write_ack(ser, MID.SetOutputSettings, struct.pack('!I', settings))
    dlen = struct.unpack('!H', _write_ack(ser, MID.ReqDataLength))[0]
except Exception:
    mode &= ~OutputMode.RAWGPS
    _goto_config(ser)
    _write_ack(ser, MID.SetOutputMode, struct.pack('!H', mode))
    _write_ack(ser, MID.SetOutputSettings, struct.pack('!I', settings))
    dlen = struct.unpack('!H', _write_ack(ser, MID.ReqDataLength))[0]

_goto_measurement(ser)
time.sleep(0.05)
ser.reset_input_buffer()

def read_samples(n, label):
    rolls, pitches = [], []
    count = 0
    while count < n:
        try:
            mid, payload = _read_msg(ser, timeout_s=1.0)
        except TimeoutError:
            continue
        if mid == MID.MTData:
            data = _parse_mtdata(payload, mode, settings)
            if data.roll_deg is not None:
                rolls.append(data.roll_deg)
                pitches.append(data.pitch_deg)
            count += 1
    if rolls:
        r_rng = max(rolls) - min(rolls)
        p_rng = max(pitches) - min(pitches)
        r_drift = sum(rolls[-10:])/10 - sum(rolls[:10])/10
        p_drift = sum(pitches[-10:])/10 - sum(pitches[:10])/10
        print(f"  {label}: Roll mean={sum(rolls)/len(rolls):+.3f} range={r_rng:.4f} drift={r_drift:+.4f}  "
              f"Pitch mean={sum(pitches)/len(pitches):+.3f} range={p_rng:.4f} drift={p_drift:+.4f}")

print("=== BEFORE calibration (200 samples) ===")
read_samples(200, "BEFORE")

# SetNoRotation using write_ack (duration=4 seconds)
print("\n=== SetNoRotation (4s) via write_ack... ===")
try:
    ack = _write_ack(ser, MID.SetNoRotation, struct.pack('!H', 4))
    print(f"  ACK received! payload={ack.hex() if ack else 'empty'}")
    print("  Waiting 5 seconds for gyro bias estimation...")
    time.sleep(5)
except IOError as e:
    print(f"  Failed: {e}")
    # Fallback: object reset
    print("\n=== Fallback: ResetOrientation object reset (0x0003) via write_ack... ===")
    try:
        ack = _write_ack(ser, MID.ResetOrientation, struct.pack('!H', 0x0003))
        print(f"  ACK received! payload={ack.hex() if ack else 'empty'}")
    except IOError as e2:
        print(f"  Also failed: {e2}")

ser.reset_input_buffer()

print("\n=== AFTER calibration (200 samples) ===")
read_samples(200, "AFTER ")

_goto_config(ser)
ser.close()
print("\nDone.")
