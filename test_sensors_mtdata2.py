#!/usr/bin/env python3
"""Mark IV (MTData2) protocol tests for the MTi-G-710 — no device required.

Packets are built the way the device frames them and fed back through the
parser, so the tests pin the wire layout rather than the parser's own idea
of it.

Run: python test_sensors_mtdata2.py
"""
import math
import struct
import sys
import time
sys.path.insert(0, '.')

from sensors import (
    MID, XDI, PROTOCOL_MK3, PROTOCOL_MK4, SensorData,
    protocol_for_product, mk4_output_config, mk4_rate_for_baud,
    _build_msg, _read_msg, _parse_mtdata2, _carry_forward, _is_inertial,
)

failures = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        failures.append(name)


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def pkt(xdi, data):
    return struct.pack('!HB', xdi, len(data)) + data


def gnss_pvt(lat, lon, height_mm, vn, ve, vd, num_sv, itow=123456):
    return struct.pack(
        '!IHBBBBBBIiBBBBiiiiIIiiiiiIIiHHHHHHH',
        itow, 2026, 9, 24, 12, 0, 0, 0x07, 50, 0,
        3, 0x01, num_sv, 0,
        int(lon * 1e7), int(lat * 1e7), height_mm, height_mm - 47000,
        1500, 2500, vn, ve, vd, int(math.hypot(vn, ve)), 0,
        300, 0, 0, 120, 110, 100, 90, 80, 70, 60)


def satinfo(sats):
    body = struct.pack('!IB3x', 123456, len(sats))
    for svid, cno, flags in sats:
        body += struct.pack('!BBBB', 0, svid, cno, flags)
    return body


class FakeSerial:
    def __init__(self, data=b'', endless=None):
        self._buf = bytearray(data)
        self._endless = endless

    def read(self, n):
        if not self._buf and self._endless:
            self._buf.extend(self._endless * 64)
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out


# ── Product-code detection ──
for code, want in [
    ("MTi-G-710-2A8G4", PROTOCOL_MK4),
    ("MTi-G-700-2A5G4", PROTOCOL_MK4),
    ("MTi-300-2A8G4", PROTOCOL_MK4),
    ("MTi-30-2A8G4", PROTOCOL_MK4),
    ("MTi-G-28A53G35", PROTOCOL_MK3),
    ("MTi-28A53G25", PROTOCOL_MK3),
    ("", PROTOCOL_MK3),
]:
    check(f"protocol for {code or '(empty)'!r}", protocol_for_product(code) == want)

# ── Inertial + position packet round-trip ──
payload = b''.join([
    pkt(XDI.PacketCounter, struct.pack('!H', 4242)),
    pkt(XDI.StatusWord, struct.pack('!I', 0x00A00007)),
    pkt(XDI.EulerAngles | XDI.FrameNWU, struct.pack('!fff', 1.5, -2.25, 93.0)),
    pkt(XDI.Acceleration, struct.pack('!fff', 0.1, -0.2, 9.81)),
    pkt(XDI.RateOfTurn, struct.pack('!fff', 0.01, 0.02, -0.03)),
    pkt(XDI.MagneticField, struct.pack('!fff', 0.4, 0.0, -0.9)),
    pkt(XDI.BaroPressure, struct.pack('!I', 95461)),
    pkt(XDI.Temperature, struct.pack('!f', 31.5)),
    pkt(XDI.LatLon | XDI.Float64, struct.pack('!dd', 50.1234567, 8.7654321)),
    pkt(XDI.AltitudeEllipsoid, struct.pack('!f', 412.5)),
    pkt(XDI.VelocityXYZ | XDI.FrameNWU, struct.pack('!fff', 30.0, -40.0, 1.5)),
    pkt(0x1234, b'\x01\x02\x03'),   # unknown XDI must be skipped, not fatal
    pkt(XDI.UtcTime, struct.pack('!IHBBBBBB', 250000000, 2026, 9, 24, 12, 0, 0, 0x07)),
])
d = _parse_mtdata2(payload)
check("packet counter", d.sample_cnt == 4242)
check("status word", d.status_word == 0x00A00007)
check("status byte keeps legacy low byte", d.status_byte == 0x07)
check("euler NWU", close(d.roll_deg, 1.5) and close(d.pitch_deg, -2.25) and close(d.yaw_deg, 93.0))
check("acc", d.acc is not None and close(d.acc[2], 9.81, 1e-5))
check("gyr", d.gyr is not None and close(d.gyr[2], -0.03, 1e-6))
check("mag", d.mag is not None and close(d.mag[2], -0.9, 1e-6))
check("baro pressure", d.pressure_pa == 95461.0)
check("temperature", close(d.temperature, 31.5))
check("lat/lon float64 keeps precision",
      close(d.pos_lat, 50.1234567, 1e-9) and close(d.pos_lon, 8.7654321, 1e-9))
check("altitude", close(d.pos_alt, 412.5))
check("velocity", d.vel is not None and close(d.vel[1], -40.0))
check("ground speed", close(d.speed_ms, 50.0, 1e-5))
check("unknown XDI skipped, later fields still parsed", d.utc_ms == 250000000)

# ── Fixed-point precisions ──
fp1220 = pkt(XDI.Acceleration | XDI.Fp1220,
             struct.pack('!iii', int(-1.25 * 2**20), 0, int(9.75 * 2**20)))
d = _parse_mtdata2(fp1220)
check("fp12.20 decode", d.acc is not None and close(d.acc[0], -1.25) and close(d.acc[2], 9.75))


def fp1632(v):
    raw = int(round(v * 2**32))
    return struct.pack('!Ih', raw & 0xFFFFFFFF, raw >> 32)


d = _parse_mtdata2(pkt(XDI.LatLon | XDI.Fp1632, fp1632(-33.8688197) + fp1632(151.2092955)))
check("fp16.32 decode (negative)", close(d.pos_lat, -33.8688197, 1e-8) and close(d.pos_lon, 151.2092955, 1e-8))

# ── Truncated payload must not raise ──
try:
    d = _parse_mtdata2(pkt(XDI.EulerAngles, struct.pack('!fff', 1, 2, 3))[:-4])
    check("truncated packet ignored", d.roll_deg is None)
except Exception as e:
    check("truncated packet ignored", False, repr(e))

# ── GNSS PVT maps onto the legacy RAWGPS fields ──
d = _parse_mtdata2(pkt(XDI.GnssPvtData, gnss_pvt(50.1234567, 8.7654321, 460250, 1200, -3400, 250, 11)))
rg = d.rawgps
check("pvt present", rg is not None)
check("pvt lat/lon", close(rg.lat_deg, 50.1234567, 1e-7) and close(rg.lon_deg, 8.7654321, 1e-7))
check("pvt height mm -> m", close(rg.alt_m, 460.25))
check("pvt vel mm/s -> m/s", close(rg.vel_n_ms, 1.2) and close(rg.vel_e_ms, -3.4) and close(rg.vel_d_ms, 0.25))
check("pvt accuracies", close(rg.hacc_m, 1.5) and close(rg.vacc_m, 2.5) and close(rg.sacc_mps, 0.3))
check("pvt numSV in bGPS", rg.bGPS == 11)
check("pvt iTOW", rg.itow_ms == 123456)

# ── GNSS SatInfo maps onto the legacy GPS-status channels ──
d = _parse_mtdata2(pkt(XDI.GnssSatInfo, satinfo([(5, 42, 0x0F), (12, 0, 0x01), (29, 31, 0x04)])))
gs = d.gps_status
check("satinfo count", gs is not None and gs.nch == 3 and len(gs.channels) == 3)
check("satinfo svid/cno", [(c.svid, c.cnr) for c in gs.channels] == [(5, 42), (12, 0), (29, 31)])
check("satinfo quality indicator", [c.qi for c in gs.channels] == [7, 1, 4])
check("satinfo used-in-solution bit", [c.bitmask for c in gs.channels] == [1, 0, 0])

# ── Slow fields carried between packets ──
latch = {}
first = _parse_mtdata2(payload + pkt(XDI.GnssPvtData, gnss_pvt(50.0, 8.0, 1000, 0, 0, 0, 9)))
_carry_forward(first, latch)
second = _parse_mtdata2(pkt(XDI.EulerAngles | XDI.FrameNWU, struct.pack('!fff', 0, 0, 10)))
_carry_forward(second, latch)
check("rawgps carried forward", second.rawgps is first.rawgps)
check("pressure carried forward", second.pressure_pa == 95461.0)
check("temperature carried forward", close(second.temperature, 31.5))
check("fast fields not carried", second.acc is None and second.vel is None)
third = _parse_mtdata2(pkt(XDI.BaroPressure, struct.pack('!I', 95000)))
_carry_forward(third, latch)
check("fresh value replaces latch", third.pressure_pa == 95000.0 and latch['pressure_pa'] == 95000.0)

# ── GNSS-only messages feed the latch but are not samples ──
# The MTi-G-710 sends PVT and SatInfo in their own MTData2 messages,
# alongside only the timestamp and status fields.
latch = {}
gnss_only = _parse_mtdata2(b''.join([
    pkt(XDI.GnssPvtData, gnss_pvt(48.0, 11.0, 520000, 0, 0, 0, 7)),
    pkt(XDI.UtcTime, struct.pack('!IHBBBBBB', 0, 2026, 9, 24, 12, 0, 0, 0x07)),
    pkt(XDI.StatusWord, struct.pack('!I', 0x07)),
    pkt(XDI.PacketCounter, struct.pack('!H', 7)),
]))
_carry_forward(gnss_only, latch)
check("gnss-only message is not a sample", not _is_inertial(gnss_only))
inertial = _parse_mtdata2(pkt(XDI.Acceleration, struct.pack('!fff', 0, 0, 9.8)))
_carry_forward(inertial, latch)
check("inertial message is a sample", _is_inertial(inertial))
check("next sample carries the gnss fix", inertial.rawgps is not None and inertial.rawgps.bGPS == 7)

# ── Output configuration and bandwidth budget ──
SIZES = {
    XDI.PacketCounter: 2, XDI.SampleTimeFine: 4, XDI.StatusWord: 4, XDI.UtcTime: 12,
    XDI.Temperature: 4, XDI.EulerAngles: 12, XDI.Acceleration: 12, XDI.RateOfTurn: 12,
    XDI.MagneticField: 12, XDI.BaroPressure: 4, XDI.LatLon: 16, XDI.AltitudeEllipsoid: 4,
    XDI.VelocityXYZ: 12,
}
for baud in (57600, 115200, 230400, 460800, 921600):
    hz = mk4_rate_for_baud(baud)
    cfg = mk4_output_config(hz)
    # Bytes per second the inertial stream actually needs, with each XDI at
    # its configured rate, plus 5 bytes of framing per message
    per_s = 5 * hz
    for xdi, freq in cfg:
        size = SIZES.get(xdi & XDI.TypeMask)
        if size is None:
            continue   # GNSS: 4 Hz / 1 Hz, accounted for below
        per_s += (3 + size) * (hz if freq == XDI.FreqEvery else min(freq, hz))
    per_s += 4 * (3 + 94) + 1 * (3 + 8 + 4 * 40)
    load = per_s * 10 / baud
    check(f"{baud} bd -> {hz} Hz fits the link", load <= 0.8, f"load {load:.0%}")
cfg = dict(mk4_output_config(100))
check("euler requested in NWU", (XDI.EulerAngles | XDI.FrameNWU) in cfg)
check("velocity requested in NWU", (XDI.VelocityXYZ | XDI.FrameNWU) in cfg)
check("baro capped at 50 Hz", cfg[XDI.BaroPressure] == 50)
check("GNSS PVT at 4 Hz", cfg[XDI.GnssPvtData] == 4)

# ── Framing: MTData2 through the real message reader ──
ser = FakeSerial(b'\x00\x13\x37' + _build_msg(MID.MTData2, payload))
mid, got = _read_msg(ser, timeout_s=0.5)
check("framed MTData2 reads back", mid == MID.MTData2 and got == payload)
big = payload * 3   # > 254 bytes takes the extended length field
mid, got = _read_msg(FakeSerial(_build_msg(MID.MTData2, big)), timeout_s=0.5)
check("extended-length MTData2 reads back", mid == MID.MTData2 and got == big)

# ── Wrong-baud garbage must time out rather than spin ──
t0 = time.time()
try:
    _read_msg(FakeSerial(endless=b'\x55\xAA\x13'), timeout_s=0.2)
    check("garbage stream times out", False, "returned a message")
except TimeoutError:
    elapsed = time.time() - t0
    check("garbage stream times out", elapsed < 1.0, f"{elapsed:.2f}s")

print()
if failures:
    print(f"{len(failures)} FAILED: {', '.join(failures)}")
    sys.exit(1)
print("All MTData2 tests passed")
