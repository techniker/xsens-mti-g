#!/usr/bin/env python3
"""
Xsens MTi-G Sensor Communication & Parsing
Complete Mark III legacy protocol (MTData 0x32).
All MTi-G supported commands implemented.
"""

import math
import serial
import struct
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Callable
from collections import deque


# ─────────────────────── Protocol constants ───────────────────────

class MID:
    # State
    WakeUp            = 0x3E
    WakeUpAck         = 0x3F
    GoToConfig        = 0x30
    GoToMeasurement   = 0x10
    Reset             = 0x40
    # Info
    ReqDID            = 0x00
    DeviceID          = 0x01
    ReqProductCode    = 0x1C
    ProductCode       = 0x1D
    ReqHardwareVersion = 0x1E
    HardwareVersion   = 0x1F
    ReqFWRev          = 0x12
    FirmwareRev       = 0x13
    Error             = 0x42
    # Config
    RestoreFactoryDef = 0x0E
    SetBaudrate       = 0x18
    SetErrorMode      = 0xDA
    SetTransmitDelay  = 0xDC
    SetLocationID     = 0x84
    ReqConfiguration  = 0x0C
    Configuration     = 0x0D
    SetPeriod         = 0x04
    SetOutputMode     = 0xD0
    SetOutputSettings = 0xD2
    SetOutputSkipFactor = 0xD4
    SetAlignmentRotation = 0xEC
    ReqDataLength     = 0x0A
    DataLength        = 0x0B
    # Data
    ReqData           = 0x34
    MTData            = 0x32
    # Filter
    ResetOrientation  = 0xA4
    ResetOrientationAck = 0xA5
    SetNoRotation     = 0x22
    SetNoRotationAck  = 0x23
    SetUTCTime        = 0x60
    UTCTime           = 0x61
    AdjustUTCTime     = 0xA8
    ReqAvailableScenarios = 0x62
    AvailableScenarios = 0x63
    SetCurrentScenario = 0x64
    SetGravityMagnitude = 0x66
    IccCommand        = 0x74
    # MTi-G specific (deprecated but available)
    ReqGPSStatus      = 0xA6
    GPSStatus         = 0xA7
    SetLeverArmGPS    = 0x68
    SetMagneticDeclination = 0x6A
    SetProcessingFlags = 0x20
    SetSyncOutSettings = 0xD8
    SetObjectAlignment = 0xE0


class OutputMode:
    Temp      = 0x0001
    Calib     = 0x0002
    Orient    = 0x0004
    Auxiliary = 0x0008
    Position  = 0x0010
    Velocity  = 0x0020
    Status    = 0x0800
    RAWGPS    = 0x1000
    RAW       = 0x4000


class OutputSettings:
    Timestamp_None        = 0x00000000
    Timestamp_SampleCnt   = 0x00000001
    Timestamp_UTCTime     = 0x00000002
    OrientMode_Quaternion = 0x00000000
    OrientMode_Euler      = 0x00000004
    OrientMode_Matrix     = 0x00000008
    CalibMode_AccGyrMag   = 0x00000000
    CalibMode_GyrMag      = 0x00000010
    CalibMode_AccMag      = 0x00000020
    CalibMode_Mag         = 0x00000030
    CalibMode_AccGyr      = 0x00000040
    CalibMode_Gyr         = 0x00000050
    CalibMode_Acc         = 0x00000060
    CalibMode_Mask        = 0x00000070
    AuxiliaryMode_NoAIN1  = 0x00000400
    AuxiliaryMode_NoAIN2  = 0x00000800
    Coordinates_NED       = 0x80000000


class Baudrates:
    _map = {
        0x0B: 4800, 0x09: 9600, 0x08: 14400, 0x07: 19200,
        0x06: 28800, 0x05: 38400, 0x04: 57600, 0x03: 76800,
        0x02: 115200, 0x01: 230400, 0x00: 460800, 0x80: 921600,
    }
    _rmap = {v: k for k, v in _map.items()}


# ─────────────────────── Wire protocol ───────────────────────

def _checksum(payload_bytes):
    return (-(sum(payload_bytes))) & 0xFF


def _build_msg(mid, data=b''):
    if len(data) > 254:
        length = b'\xFF' + struct.pack('!H', len(data))
    else:
        length = struct.pack('!B', len(data))
    body = b'\xFF' + bytes([mid]) + length + data
    return b'\xFA' + body + bytes([_checksum(body)])


def _read_exact(ser, n, timeout_s=0.5):
    buf = bytearray()
    t0 = time.time()
    while len(buf) < n and (time.time() - t0) < timeout_s:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
        else:
            time.sleep(0.001)
    if len(buf) != n:
        raise TimeoutError(f"read_exact: wanted {n}, got {len(buf)}")
    return bytes(buf)


def _read_msg(ser, timeout_s=2.0):
    t0 = time.time()
    while True:
        b = ser.read(1)
        if not b:
            if time.time() - t0 > timeout_s:
                raise TimeoutError("timeout preamble")
            continue
        if b[0] != 0xFA:
            continue
        if _read_exact(ser, 1, 0.5)[0] != 0xFF:
            continue
        head = _read_exact(ser, 2, 0.5)
        mid = head[0]
        lf = head[1]
        if lf == 0xFF:
            length = struct.unpack('!H', _read_exact(ser, 2, 0.5))[0]
            len_bytes = [0xFF, (length >> 8) & 0xFF, length & 0xFF]
        else:
            length = lf
            len_bytes = [length]
        payload = _read_exact(ser, length, 0.5)
        chk = _read_exact(ser, 1, 0.5)[0]
        total = 0xFF + mid + sum(len_bytes) + sum(payload) + chk
        if (total & 0xFF) != 0:
            continue
        return mid, payload


def _write_ack(ser, mid, data=b'', retries=150):
    ser.write(_build_msg(mid, data))
    for _ in range(retries):
        try:
            mid_ack, ack = _read_msg(ser, timeout_s=2.0)
            if mid_ack == ((mid + 1) & 0xFF):
                return ack
        except TimeoutError:
            continue
    raise IOError(f"No ACK for MID 0x{mid:02X}")


def _write_req(ser, mid, data=b''):
    ser.write(_build_msg(mid, data))


def _goto_config(ser):
    _write_ack(ser, MID.GoToConfig, b'')


def _goto_measurement(ser):
    _write_ack(ser, MID.GoToMeasurement, b'')


# ─────────────────────── Data models ───────────────────────

@dataclass
class RawGPS:
    press_pa: Optional[float] = None
    bPrs: Optional[int] = None
    itow_ms: Optional[int] = None
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    alt_m: Optional[float] = None
    vel_n_ms: Optional[float] = None
    vel_e_ms: Optional[float] = None
    vel_d_ms: Optional[float] = None
    hacc_m: Optional[float] = None
    vacc_m: Optional[float] = None
    sacc_mps: Optional[float] = None
    bGPS: Optional[int] = None


@dataclass
class GPSChannel:
    chn: int
    svid: int
    bitmask: int
    qi: int
    cnr: int


@dataclass
class GPSStatusData:
    nch: int = 0
    channels: List[GPSChannel] = field(default_factory=list)


@dataclass
class SensorData:
    timestamp: float = 0.0
    sample_cnt: Optional[int] = None
    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    yaw_deg: Optional[float] = None
    acc: Optional[Tuple[float, float, float]] = None
    gyr: Optional[Tuple[float, float, float]] = None
    mag: Optional[Tuple[float, float, float]] = None
    pos_lat: Optional[float] = None
    pos_lon: Optional[float] = None
    pos_alt: Optional[float] = None
    vel: Optional[Tuple[float, float, float]] = None
    ain1: Optional[int] = None
    ain2: Optional[int] = None
    temperature: Optional[float] = None
    pressure_pa: Optional[float] = None
    baro_alt_m: Optional[float] = None
    speed_ms: Optional[float] = None
    status_byte: Optional[int] = None
    rawgps: Optional[RawGPS] = None
    gps_status: Optional[GPSStatusData] = None
    utc_ms: Optional[int] = None


@dataclass
class FilterProfile:
    profile_type: int
    version: int
    label: str


@dataclass
class DeviceInfo:
    device_id: int = 0
    product_code: str = ""
    fw_major: int = 0
    fw_minor: int = 0
    fw_rev: int = 0
    hw_major: int = 0
    hw_minor: int = 0
    output_mode: int = 0
    output_settings: int = 0
    data_length: int = 0
    has_rawgps: bool = False
    # Full configuration (from ReqConfiguration 118 bytes)
    period: int = 0             # sampling period (1/115200 s units)
    skip_factor: int = 0        # output skip factor
    baudrate_id: int = 0
    # Filter
    current_scenario: int = 0
    available_scenarios: List[FilterProfile] = field(default_factory=list)
    # MTi-G specific
    lever_arm_gps: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    magnetic_declination: float = 0.0
    gravity_magnitude: float = 9.80665
    processing_flags: int = 0
    error_mode: int = 0
    location_id: int = 0
    alignment_rotation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    object_alignment: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    transmit_delay: int = 0
    sync_out_mode: int = 0
    sync_out_skip_factor: int = 0
    sync_out_offset: int = 0
    sync_out_pulse_width: int = 0


# ─────────────────────── Parser ───────────────────────

def _parse_mtdata(payload: bytes, mode: int, settings: int) -> SensorData:
    s = SensorData(timestamp=time.time())
    data = payload
    n = len(data)
    o = 0

    def remaining():
        return n - o

    def consume(count):
        nonlocal o
        if remaining() < count:
            return None
        chunk = data[o:o + count]
        o += count
        return chunk

    if mode & OutputMode.RAW:
        consume(20)
    if mode & OutputMode.RAWGPS:
        chunk = consume(44)
        if chunk:
            fields = struct.unpack('!H B I i i i i i i I I I B', chunk)
            press_u2, bPrs, itow, lat_i, lon_i, alt_mm, vn, ve, vd, hacc, vacc, sacc, bGPS = fields
            rg = RawGPS(
                press_pa=float(press_u2) * 2.0, bPrs=bPrs, itow_ms=itow,
                lat_deg=lat_i * 1e-7, lon_deg=lon_i * 1e-7, alt_m=alt_mm / 1000.0,
                vel_n_ms=vn / 100.0, vel_e_ms=ve / 100.0, vel_d_ms=vd / 100.0,
                hacc_m=hacc / 1000.0, vacc_m=vacc / 1000.0, sacc_mps=sacc / 1000.0, bGPS=bGPS)
            s.rawgps = rg
            s.pressure_pa = rg.press_pa
    if mode & OutputMode.Temp:
        chunk = consume(4)
        if chunk:
            s.temperature = struct.unpack('!f', chunk)[0]
    if mode & OutputMode.Calib:
        cm = settings & OutputSettings.CalibMode_Mask
        has_acc = cm not in (0x10, 0x50, 0x30)
        has_gyr = cm not in (0x20, 0x60, 0x30)
        has_mag = cm not in (0x40, 0x60, 0x50)
        if has_acc:
            chunk = consume(12)
            if chunk: s.acc = struct.unpack('!fff', chunk)
        if has_gyr:
            chunk = consume(12)
            if chunk: s.gyr = struct.unpack('!fff', chunk)
        if has_mag:
            chunk = consume(12)
            if chunk: s.mag = struct.unpack('!fff', chunk)
    if mode & OutputMode.Orient:
        om = settings & 0x0C
        if om == OutputSettings.OrientMode_Euler:
            chunk = consume(12)
            if chunk:
                r, p, y = struct.unpack('!fff', chunk)
                s.roll_deg, s.pitch_deg, s.yaw_deg = float(r), float(p), float(y)
        elif om == OutputSettings.OrientMode_Matrix:
            consume(36)
        else:
            chunk = consume(16)
            if chunk:
                q0, q1, q2, q3 = struct.unpack('!ffff', chunk)
                sinr = 2.0 * (q0 * q1 + q2 * q3)
                cosr = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
                s.roll_deg = math.degrees(math.atan2(sinr, cosr))
                sinp = max(-1.0, min(1.0, 2.0 * (q0 * q2 - q3 * q1)))
                s.pitch_deg = math.degrees(math.asin(sinp))
                siny = 2.0 * (q0 * q3 + q1 * q2)
                cosy = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
                s.yaw_deg = math.degrees(math.atan2(siny, cosy))
    if mode & OutputMode.Auxiliary:
        if not (settings & OutputSettings.AuxiliaryMode_NoAIN1):
            chunk = consume(2)
            if chunk: s.ain1 = struct.unpack('!H', chunk)[0]
        if not (settings & OutputSettings.AuxiliaryMode_NoAIN2):
            chunk = consume(2)
            if chunk: s.ain2 = struct.unpack('!H', chunk)[0]
    if mode & OutputMode.Position:
        chunk = consume(12)
        if chunk:
            lat, lon, alt = struct.unpack('!fff', chunk)
            s.pos_lat, s.pos_lon, s.pos_alt = float(lat), float(lon), float(alt)
    if mode & OutputMode.Velocity:
        chunk = consume(12)
        if chunk:
            vx, vy, vz = struct.unpack('!fff', chunk)
            s.vel = (float(vx), float(vy), float(vz))
    if mode & OutputMode.Status:
        chunk = consume(1)
        if chunk: s.status_byte = chunk[0]
    if settings & OutputSettings.Timestamp_SampleCnt:
        chunk = consume(2)
        if chunk: s.sample_cnt = struct.unpack('!H', chunk)[0]
    if settings & OutputSettings.Timestamp_UTCTime:
        consume(12)
    if s.pressure_pa and s.pressure_pa > 0:
        s.baro_alt_m = 44330.0 * (1.0 - (s.pressure_pa / 101325.0) ** 0.19029495718363465)
    if s.vel:
        s.speed_ms = math.hypot(s.vel[0], s.vel[1])
    return s


def _parse_gps_status(payload: bytes) -> GPSStatusData:
    if not payload:
        return GPSStatusData()
    nch = payload[0]
    channels = []
    o = 1
    for _ in range(nch):
        if len(payload) - o < 5:
            break
        chn, svid, bitmask, qi, cnr = struct.unpack_from('!BBBbb', payload, o)
        o += 5
        channels.append(GPSChannel(chn=chn, svid=svid, bitmask=bitmask, qi=qi, cnr=cnr))
    return GPSStatusData(nch=nch, channels=channels)


# ─────────────────────── Config read helpers ───────────────────────

def _quick_ack(ser, mid, data=b'', retries=5):
    """Fast write_ack with low retry count for config queries."""
    ser.write(_build_msg(mid, data))
    for _ in range(retries):
        try:
            mid_ack, ack = _read_msg(ser, timeout_s=0.5)
            if mid_ack == ((mid + 1) & 0xFF):
                return ack
        except TimeoutError:
            continue
    return None


def _read_full_config(ser, info: DeviceInfo):
    """Read all available config parameters while in config mode.
    Uses fast timeouts — unsupported commands just return None."""

    # Hardware version
    hw = _quick_ack(ser, MID.ReqHardwareVersion)
    if hw and len(hw) >= 2:
        info.hw_major, info.hw_minor = hw[0], hw[1]

    # Full 118-byte configuration block
    cfg = _quick_ack(ser, MID.ReqConfiguration)
    if cfg and len(cfg) >= 10:
        info.period = struct.unpack_from('!H', cfg, 4)[0]
        info.skip_factor = struct.unpack_from('!H', cfg, 6)[0]

    # Current scenario / filter profile
    sc = _quick_ack(ser, MID.SetCurrentScenario)
    if sc and len(sc) >= 2:
        info.current_scenario = struct.unpack('!H', sc[:2])[0]

    # Available scenarios (22 bytes each: U16 type + 20 char label)
    raw = _quick_ack(ser, MID.ReqAvailableScenarios)
    if raw:
        info.available_scenarios = []
        o = 0
        while o + 22 <= len(raw):
            profile_type = struct.unpack_from('!H', raw, o)[0]
            label = raw[o + 2:o + 22].decode('ascii', errors='ignore').strip('\x00 ')
            info.available_scenarios.append(FilterProfile(profile_type, 0, label))
            o += 22

    # GPS Lever Arm (MTi-G specific)
    la = _quick_ack(ser, MID.SetLeverArmGPS)
    if la and len(la) >= 12:
        info.lever_arm_gps = struct.unpack('!fff', la[:12])

    # Magnetic Declination (MTi-G specific)
    md = _quick_ack(ser, MID.SetMagneticDeclination)
    if md and len(md) >= 4:
        info.magnetic_declination = struct.unpack('!f', md[:4])[0]

    # Gravity Magnitude
    gm = _quick_ack(ser, MID.SetGravityMagnitude)
    if gm and len(gm) >= 4:
        info.gravity_magnitude = struct.unpack('!f', gm[:4])[0]

    # Processing Flags
    pf = _quick_ack(ser, MID.SetProcessingFlags)
    if pf and len(pf) >= 1:
        info.processing_flags = pf[0]

    # Error Mode
    em = _quick_ack(ser, MID.SetErrorMode)
    if em and len(em) >= 2:
        info.error_mode = struct.unpack('!H', em[:2])[0]

    # Location ID
    li = _quick_ack(ser, MID.SetLocationID)
    if li and len(li) >= 2:
        info.location_id = struct.unpack('!H', li[:2])[0]

    # Alignment Rotation (quaternion)
    ar = _quick_ack(ser, MID.SetAlignmentRotation)
    if ar and len(ar) >= 16:
        info.alignment_rotation = struct.unpack('!ffff', ar[:16])

    # Object Alignment (quaternion)
    oa = _quick_ack(ser, MID.SetObjectAlignment)
    if oa and len(oa) >= 16:
        info.object_alignment = struct.unpack('!ffff', oa[:16])

    # Transmit Delay
    td = _quick_ack(ser, MID.SetTransmitDelay)
    if td and len(td) >= 2:
        info.transmit_delay = struct.unpack('!H', td[:2])[0]

    # Sync Out Settings — response format varies by firmware;
    # try block read (12 bytes: mode U16, skip U16, offset U32, pulse U32)
    so = _quick_ack(ser, MID.SetSyncOutSettings)
    if so:
        if len(so) >= 12:
            info.sync_out_mode = struct.unpack_from('!H', so, 0)[0]
            info.sync_out_skip_factor = struct.unpack_from('!H', so, 2)[0]
            info.sync_out_offset = struct.unpack_from('!I', so, 4)[0]
            info.sync_out_pulse_width = struct.unpack_from('!I', so, 8)[0]
        elif len(so) >= 2:
            info.sync_out_mode = struct.unpack_from('!H', so, 0)[0]


# ─────────────────────── Sensor reader thread ───────────────────────

class XSensSensor(threading.Thread):

    def __init__(self, port: str, baud: int = 230400, p0_pa: float = 101325.0):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.p0_pa = p0_pa
        self.stop_event = threading.Event()
        self._data_queue: deque = deque(maxlen=512)
        self._device_info = DeviceInfo()
        self._lock = threading.Lock()
        self._connected = False
        self._error: Optional[str] = None
        self._ser = None
        self._mode = 0
        self._settings = 0
        self.on_data: Optional[Callable[[SensorData], None]] = None
        self.on_device_info: Optional[Callable[[DeviceInfo], None]] = None
        self.on_error: Optional[Callable[[str], None]] = None
        self.on_connected: Optional[Callable[[], None]] = None

    @property
    def device_info(self) -> DeviceInfo:
        with self._lock:
            return self._device_info

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def last_error(self) -> Optional[str]:
        return self._error

    def get_latest(self) -> Optional[SensorData]:
        if self._data_queue:
            return self._data_queue[-1]
        return None

    def stop(self):
        self.stop_event.set()

    def calibrate(self) -> str:
        ser = self._ser
        if not ser or not self._connected:
            return "Not connected"
        self._error = None
        _write_req(ser, MID.ResetOrientation, struct.pack('!H', 0x0003))
        return "Orientation reset sent"

    def send_icc_command(self, cmd: int) -> str:
        """In-run Compass Calibration: 0=start, 1=stop, 2=store+stop."""
        ser = self._ser
        if not ser or not self._connected:
            return "Not connected"
        _write_req(ser, MID.IccCommand, struct.pack('!B', cmd))
        return ["ICC: started", "ICC: stopped (not stored)", "ICC: stored & stopped"][cmd] if cmd < 3 else "ICC: unknown"

    def apply_setting(self, mid: int, data: bytes) -> str:
        """Send a set command (fire-and-forget, read loop owns serial)."""
        ser = self._ser
        if not ser or not self._connected:
            return "Not connected"
        _write_req(ser, mid, data)
        return "Setting sent"

    def run(self):
        try:
            self._connect_and_configure()
            self._read_loop()
        except Exception as e:
            self._error = str(e)
            if self.on_error:
                self.on_error(str(e))
        finally:
            self._cleanup()

    def _connect_and_configure(self):
        ser = serial.Serial(self.port, self.baud, timeout=0.2, write_timeout=0.2)
        self._ser = ser
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        info = DeviceInfo()
        try:
            _goto_config(ser)
            did_raw = _write_ack(ser, MID.ReqDID)
            info.device_id = struct.unpack('!I', did_raw)[0]
            prod_raw = _write_ack(ser, MID.ReqProductCode)
            info.product_code = prod_raw.decode('ascii', errors='ignore').strip('\x00\r\n ')
            fw_raw = _write_ack(ser, MID.ReqFWRev)
            fw = (fw_raw + b'\x00\x00\x00')[:3]
            info.fw_major, info.fw_minor, info.fw_rev = fw[0], fw[1], fw[2]
            # Read all config parameters
            _read_full_config(ser, info)
        except Exception:
            pass

        try:
            mode, settings, length = self._enable_rich(ser, want_rawgps=True)
            info.has_rawgps = True
        except Exception:
            mode, settings, length = self._enable_rich(ser, want_rawgps=False)
            info.has_rawgps = False

        info.output_mode = mode
        info.output_settings = settings
        info.data_length = length
        self._mode = mode
        self._settings = settings

        with self._lock:
            self._device_info = info

        _goto_measurement(ser)
        time.sleep(0.05)
        ser.reset_input_buffer()

        self._connected = True
        if self.on_device_info:
            self.on_device_info(info)
        if self.on_connected:
            self.on_connected()

    def _enable_rich(self, ser, want_rawgps=True):
        _goto_config(ser)
        if want_rawgps:
            mode = (OutputMode.Temp | OutputMode.Calib | OutputMode.Orient |
                    OutputMode.Auxiliary | OutputMode.Position | OutputMode.Velocity |
                    OutputMode.Status | OutputMode.RAWGPS)
        else:
            mode = (OutputMode.Temp | OutputMode.Calib | OutputMode.Orient |
                    OutputMode.Auxiliary | OutputMode.Position | OutputMode.Velocity |
                    OutputMode.Status)
        settings = (OutputSettings.Timestamp_SampleCnt |
                    OutputSettings.Timestamp_UTCTime |
                    OutputSettings.OrientMode_Euler |
                    OutputSettings.CalibMode_AccGyrMag)
        _write_ack(ser, MID.SetOutputMode, struct.pack('!H', mode))
        _write_ack(ser, MID.SetOutputSettings, struct.pack('!I', settings))
        length = struct.unpack('!H', _write_ack(ser, MID.ReqDataLength))[0]
        return mode, settings, length

    def _read_loop(self):
        ser = self._ser
        next_gps_poll = 0.0
        next_utc_poll = 0.0
        last_gps_status = None
        last_utc_ms = None

        while not self.stop_event.is_set():
            try:
                now = time.time()
                if (self._mode & OutputMode.RAWGPS) and now >= next_gps_poll:
                    _write_req(ser, MID.ReqGPSStatus)
                    next_gps_poll = now + 1.0
                if now >= next_utc_poll:
                    _write_req(ser, MID.SetUTCTime)
                    next_utc_poll = now + 1.0

                mid, payload = _read_msg(ser, timeout_s=0.5)

                if mid == MID.MTData:
                    data = _parse_mtdata(payload, self._mode, self._settings)
                    if data.pressure_pa and data.pressure_pa > 0:
                        data.baro_alt_m = 44330.0 * (1.0 - (data.pressure_pa / self.p0_pa) ** 0.19029495718363465)
                    data.gps_status = last_gps_status
                    data.utc_ms = last_utc_ms
                    self._data_queue.append(data)
                    if self.on_data:
                        self.on_data(data)
                elif mid == MID.GPSStatus:
                    last_gps_status = _parse_gps_status(payload)
                elif mid == MID.UTCTime:
                    if len(payload) >= 4:
                        last_utc_ms = struct.unpack_from('!I', payload, 0)[0]
                elif mid == MID.Error:
                    code = payload[0] if payload else 0
                    if code != 0x21:
                        self._error = f"MT Error 0x{code:02X}"
            except TimeoutError:
                continue
            except Exception as e:
                self._error = str(e)
                time.sleep(0.01)

    def _cleanup(self):
        if self._ser:
            try:
                _goto_config(self._ser)
            except Exception:
                pass
            try:
                self._ser.close()
            except Exception:
                pass
        self._connected = False
