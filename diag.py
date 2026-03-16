#!/usr/bin/env python3
"""Diagnostic: read 200 samples, print raw Euler angles, check for drift."""
import sys, time, math, struct
sys.path.insert(0, '.')
from sensors import (
    XSensSensor, SensorData, _read_msg, _write_ack, _write_req,
    _goto_config, _goto_measurement, _build_msg, _read_exact,
    MID, OutputMode, OutputSettings,
    _parse_mtdata,
)
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-XSU5ZPZX"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 230400
N = 200

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
    print(f"Mode=0x{mode:04X} Settings=0x{settings:08X} DataLen={dlen}")
except Exception:
    # fallback without RAWGPS
    mode &= ~OutputMode.RAWGPS
    _goto_config(ser)
    _write_ack(ser, MID.SetOutputMode, struct.pack('!H', mode))
    _write_ack(ser, MID.SetOutputSettings, struct.pack('!I', settings))
    dlen = struct.unpack('!H', _write_ack(ser, MID.ReqDataLength))[0]
    print(f"(no RAWGPS) Mode=0x{mode:04X} Settings=0x{settings:08X} DataLen={dlen}")

_goto_measurement(ser)
time.sleep(0.05)
ser.reset_input_buffer()

rolls, pitches, yaws = [], [], []
print(f"\nReading {N} samples... keep IMU stationary.\n")
print(f"{'#':>4}  {'Roll':>9}  {'Pitch':>9}  {'Yaw':>9}  {'AccX':>9}  {'AccY':>9}  {'AccZ':>9}  {'PayloadLen':>10}")
print("-" * 85)

count = 0
while count < N:
    try:
        mid, payload = _read_msg(ser, timeout_s=1.0)
    except TimeoutError:
        continue
    if mid != MID.MTData:
        continue

    data = _parse_mtdata(payload, mode, settings)
    r = data.roll_deg if data.roll_deg is not None else float('nan')
    p = data.pitch_deg if data.pitch_deg is not None else float('nan')
    y = data.yaw_deg if data.yaw_deg is not None else float('nan')
    ax = data.acc[0] if data.acc else float('nan')
    ay = data.acc[1] if data.acc else float('nan')
    az = data.acc[2] if data.acc else float('nan')

    rolls.append(r)
    pitches.append(p)
    yaws.append(y)

    if count % 20 == 0 or count < 5:
        print(f"{count:4d}  {r:+9.4f}  {p:+9.4f}  {y:+9.4f}  {ax:+9.4f}  {ay:+9.4f}  {az:+9.4f}  {len(payload):10d}")
    count += 1

print("-" * 85)

def stats(name, vals):
    vals = [v for v in vals if not math.isnan(v)]
    if not vals:
        print(f"{name}: no data")
        return
    mn, mx = min(vals), max(vals)
    avg = sum(vals) / len(vals)
    std = math.sqrt(sum((v - avg)**2 for v in vals) / len(vals))
    print(f"{name:>8}:  mean={avg:+9.4f}  std={std:7.4f}  min={mn:+9.4f}  max={mx:+9.4f}  range={mx-mn:7.4f}")

print(f"\nStatistics over {N} samples (stationary):")
stats("Roll", rolls)
stats("Pitch", pitches)
stats("Yaw", yaws)

# Check: does it look like drift?
if len(rolls) > 10:
    first10 = sum(rolls[:10]) / 10
    last10 = sum(rolls[-10:]) / 10
    drift = last10 - first10
    print(f"\n  Roll drift (last10 - first10): {drift:+.4f} deg")
    first10 = sum(pitches[:10]) / 10
    last10 = sum(pitches[-10:]) / 10
    drift = last10 - first10
    print(f"  Pitch drift (last10 - first10): {drift:+.4f} deg")
    first10 = sum(yaws[:10]) / 10
    last10 = sum(yaws[-10:]) / 10
    drift = last10 - first10
    print(f"  Yaw drift (last10 - first10): {drift:+.4f} deg")

_goto_config(ser)
ser.close()
print("\nDone.")
