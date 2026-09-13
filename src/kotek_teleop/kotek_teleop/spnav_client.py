"""Minimal ctypes wrapper over libspnav, talking to the host's spacenavd.

Why ctypes instead of hidapi (what IsaacLab's Se3SpaceMouse uses): hidapi opens
/dev/hidraw* directly, which is root-only on this host and would need a udev
rule. spacenavd is already running as a systemd service and exposes a
world-accessible AF_UNIX socket (/run/spnav.sock, srwxrwxrwx) -- verified live
against the real SpaceMouse Compact this session (streamed 6-axis motion with
no root, no udev changes). libspnav's spnav_open() talks to exactly that
socket, so this wrapper needs nothing beyond the already-installed libspnav0
package.

Struct layout below is transcribed directly from /usr/include/spnav.h (see
that header for the authoritative definitions); this module intentionally
duplicates only the handful of symbols the teleop node needs, not the whole
API (config/LED/sensitivity calls etc. are all omitted).
"""
from __future__ import annotations

import ctypes
import ctypes.util
from ctypes import POINTER, Structure, Union, c_char, c_int, c_uint
from dataclasses import dataclass

# enum { SPNAV_EVENT_ANY, SPNAV_EVENT_MOTION, SPNAV_EVENT_BUTTON, ... }
SPNAV_EVENT_ANY = 0
SPNAV_EVENT_MOTION = 1
SPNAV_EVENT_BUTTON = 2


class _EventMotion(Structure):
    _fields_ = [
        ('type', c_int),
        ('x', c_int), ('y', c_int), ('z', c_int),
        ('rx', c_int), ('ry', c_int), ('rz', c_int),
        ('period', c_uint),
        ('data', POINTER(c_int)),
    ]


class _EventButton(Structure):
    _fields_ = [('type', c_int), ('press', c_int), ('bnum', c_int)]


class _EventDev(Structure):
    _fields_ = [
        ('type', c_int), ('op', c_int), ('id', c_int), ('devtype', c_int),
        ('usbid', c_int * 2),
    ]


class _EventCfg(Structure):
    _fields_ = [('type', c_int), ('cfg', c_int), ('data', c_int * 6)]


class _EventAxis(Structure):
    _fields_ = [('type', c_int), ('idx', c_int), ('value', c_int)]


class _SpnavEvent(Union):
    _fields_ = [
        ('type', c_int),
        ('motion', _EventMotion),
        ('button', _EventButton),
        ('dev', _EventDev),
        ('cfg', _EventCfg),
        ('axis', _EventAxis),
    ]


@dataclass
class MotionEvent:
    x: int
    y: int
    z: int
    rx: int
    ry: int
    rz: int
    period: int


@dataclass
class ButtonEvent:
    press: bool
    bnum: int


def _load_libspnav():
    """Try the versioned SONAME first (all libspnav0 ships), then the
    unversioned name (only present with libspnav-dev), then a find_library
    fallback. The container image installs libspnav0 only -- .so.0 is what
    must resolve there.
    """
    last_error = None
    for name in ('libspnav.so.0', 'libspnav.so', ctypes.util.find_library('spnav')):
        if not name:
            continue
        try:
            return ctypes.CDLL(name)
        except OSError as exc:
            last_error = exc
    raise OSError(f'could not load libspnav (tried libspnav.so.0, libspnav.so): {last_error}')


class SpnavClient:
    """Thin, synchronous wrapper. Not thread-safe -- use from one thread."""

    def __init__(self):
        self._lib = _load_libspnav()
        self._lib.spnav_open.restype = c_int
        self._lib.spnav_close.restype = c_int
        self._lib.spnav_poll_event.restype = c_int
        self._lib.spnav_poll_event.argtypes = [POINTER(_SpnavEvent)]
        self._lib.spnav_client_name.restype = c_int
        self._lib.spnav_client_name.argtypes = [ctypes.c_char_p]
        self._lib.spnav_dev_name.restype = c_int
        self._lib.spnav_dev_name.argtypes = [POINTER(c_char), c_int]
        self._lib.spnav_dev_buttons.restype = c_int
        self._lib.spnav_dev_axes.restype = c_int
        self._open = False

    def open(self, client_name: str = 'kotek_teleop') -> None:
        if self._lib.spnav_open() == -1:
            raise ConnectionError(
                'spnav_open() failed -- is spacenavd running? '
                '(systemctl status spacenavd on the host; the container needs '
                '/run/spnav.sock bind-mounted)')
        self._open = True
        self._lib.spnav_client_name(client_name.encode('utf-8'))

    def close(self) -> None:
        if self._open:
            self._lib.spnav_close()
            self._open = False

    def device_name(self) -> str:
        buf = ctypes.create_string_buffer(256)
        self._lib.spnav_dev_name(buf, 256)
        return buf.value.decode('utf-8', errors='replace')

    def device_buttons(self) -> int:
        return int(self._lib.spnav_dev_buttons())

    def device_axes(self) -> int:
        return int(self._lib.spnav_dev_axes())

    def poll_event(self):
        """Non-blocking. Returns a MotionEvent, a ButtonEvent, or None."""
        ev = _SpnavEvent()
        evtype = self._lib.spnav_poll_event(ctypes.byref(ev))
        if evtype == 0:
            return None
        if evtype == SPNAV_EVENT_MOTION:
            m = ev.motion
            return MotionEvent(x=m.x, y=m.y, z=m.z, rx=m.rx, ry=m.ry, rz=m.rz, period=m.period)
        if evtype == SPNAV_EVENT_BUTTON:
            b = ev.button
            return ButtonEvent(press=bool(b.press), bnum=b.bnum)
        return None

    def drain_latest_motion(self):
        """Polls every pending event and returns only the most recent
        MotionEvent (or None if none were queued), plus the list of button
        events seen along the way. spacenavd can emit motion events faster
        than a low-rate node polls -- draining rather than reading one event
        per tick keeps latency bounded instead of accumulating a backlog.
        """
        latest_motion = None
        buttons = []
        while True:
            event = self.poll_event()
            if event is None:
                break
            if isinstance(event, MotionEvent):
                latest_motion = event
            elif isinstance(event, ButtonEvent):
                buttons.append(event)
        return latest_motion, buttons
