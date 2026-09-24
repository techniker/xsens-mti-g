#!/usr/bin/env python3
"""
Native USB transport for Xsens Mark IV devices (MTi-G-710 and siblings).

Over its USB cable the MTi-G-710 enumerates as VID 0x2639 with
vendor-specific interfaces, so no CDC driver binds and no serial port
appears on macOS. Interface 1 is a raw bulk pipe carrying the same Xbus
frames as the serial link. This class exposes the pyserial subset
sensors.py uses, so the protocol code runs over either transport.
"""

import time
from typing import Optional

XSENS_VID = 0x2639
_DATA_INTERFACE = 1
_EP_OUT = 0x02
_EP_IN = 0x83
_READ_CHUNK = 4096


def _usb():
    try:
        import usb.core
        import usb.util
        return usb
    except ImportError:
        return None


def find_device():
    """First attached Xsens USB device, or None (also None without pyusb)."""
    usb = _usb()
    if usb is None:
        return None
    try:
        return usb.core.find(idVendor=XSENS_VID)
    except usb.core.NoBackendError:
        return None


class UsbXbus:
    """Bulk-endpoint Xbus link with pyserial's read/write semantics."""

    def __init__(self, timeout: float = 0.2, write_timeout: float = 0.2):
        usb = _usb()
        if usb is None:
            raise IOError("USB device needs pyusb: pip install pyusb")
        dev = find_device()
        if dev is None:
            raise IOError("No Xsens USB device found")
        self._usb = usb
        self._dev = dev
        self.timeout = timeout
        self.write_timeout = write_timeout
        self._buf = bytearray()
        try:
            # Linux: xsens_mt usb-serial may own the interface
            if dev.is_kernel_driver_active(_DATA_INTERFACE):
                dev.detach_kernel_driver(_DATA_INTERFACE)
        except (NotImplementedError, usb.core.USBError):
            pass
        try:
            dev.set_configuration()
        except usb.core.USBError:
            pass   # already configured
        usb.util.claim_interface(dev, _DATA_INTERFACE)
        self.port = f"usb:{dev.idVendor:04x}:{dev.idProduct:04x}"

    def _fill(self, timeout_ms: int) -> bool:
        try:
            self._buf += self._dev.read(_EP_IN, _READ_CHUNK, timeout=timeout_ms)
            return True
        except self._usb.core.USBTimeoutError:
            return False

    def read(self, n: int = 1) -> bytes:
        if len(self._buf) < n:
            deadline = time.monotonic() + self.timeout
            while len(self._buf) < n:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                self._fill(max(1, int(left * 1000)))
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def write(self, data: bytes) -> int:
        return self._dev.write(_EP_OUT, data, timeout=int(self.write_timeout * 1000))

    def reset_input_buffer(self):
        self._buf.clear()
        for _ in range(32):
            if not self._fill(1):
                break
        self._buf.clear()

    def reset_output_buffer(self):
        pass

    def close(self):
        try:
            self._usb.util.release_interface(self._dev, _DATA_INTERFACE)
            self._usb.util.dispose_resources(self._dev)
        except self._usb.core.USBError:
            pass


def is_usb_port(port: Optional[str]) -> bool:
    return (port or "").lower().startswith("usb")
