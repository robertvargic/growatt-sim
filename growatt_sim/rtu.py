"""
Modbus RTU slave on a serial port — standard library only (termios, no pyserial).

The port is one of:

  a tty device   /dev/serial0 (the UART on the GPIO pins, with an RS-485
                 transceiver), /dev/ttyUSB1 (a second USB RS-485 adapter), ...
  "pty"          a virtual serial port. The simulator creates it and links it
                 at a fixed path (default /tmp/ttyGROWATT), so a master on the
                 same machine can open it like a real port. No hardware needed.

RS-485 is half duplex, so the transceiver has to be switched to "send" for
exactly as long as the reply takes:

  de = none   the adapter switches by itself (USB adapters, auto-direction boards)
  de = gpio   a GPIO line drives DE and /RE (MAX485-style boards)
  de = rts    the RTS line of the port drives DE

Frame:  slave address (1) | function code (1) | data ... | CRC-16 (2, low byte first)
"""

from __future__ import annotations

import fcntl
import logging
import os
import pty
import random
import select
import struct
import termios
import threading
import time
from typing import Optional

from . import registers as R
from .model import EXC_ILLEGAL_FUNCTION, EXC_ILLEGAL_VALUE, Inverter, ModbusError, exc_name

log = logging.getLogger("growatt_sim.rtu")

FC_READ_HOLDING, FC_READ_INPUT, FC_WRITE_SINGLE, FC_WRITE_MULTIPLE = 3, 4, 6, 16
SILENCE_S = 0.1            # a gap this long ends a frame; anything incomplete is dropped
REOPEN_S = 2.0             # how often a port that failed to open is tried again


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def add_crc(body: bytes) -> bytes:
    crc = crc16(body)
    return body + bytes([crc & 0xFF, crc >> 8])


def crc_ok(frame: bytes) -> bool:
    return len(frame) >= 4 and crc16(frame[:-2]) == frame[-2] | (frame[-1] << 8)


def request_length(buf: bytes) -> Optional[int]:
    """Length of the request frame starting at buf[0]. None: need more bytes. -1: unknown function."""
    if len(buf) < 2:
        return None
    fc = buf[1]
    if fc in (1, 2, 3, 4, 5, 6):
        return 8
    if fc in (15, 16):
        return 9 + buf[6] if len(buf) >= 7 else None
    return -1


def hexdump(data: bytes) -> str:
    return data.hex(" ")


# ── RS-485 direction control ────────────────────────────────────────────────

class NoDirection:
    """The adapter switches between receive and send by itself."""
    active = False

    def send(self) -> None:
        pass

    def receive(self) -> None:
        pass

    def close(self) -> None:
        pass


class GpioDirection:
    """
    DE (and /RE) on a GPIO line, driven through the kernel's GPIO character
    device (uAPI v2), so no GPIO library is needed. Struct layout checked
    against <linux/gpio.h>: request 592 bytes, config at 288, first attribute
    at 320, num_lines at 560, fd at 588.
    """
    active = True
    _GET_LINE = 0xC250B407         # GPIO_V2_GET_LINE_IOCTL
    _SET_VALUES = 0xC010B40F       # GPIO_V2_LINE_SET_VALUES_IOCTL
    _FLAG_OUTPUT = 1 << 3
    _ATTR_OUTPUT_VALUES = 2

    def __init__(self, line: int, chip: str = "/dev/gpiochip0", active_high: bool = True) -> None:
        self._send_level = 1 if active_high else 0
        chip_fd = os.open(chip, os.O_RDWR | os.O_CLOEXEC)
        try:
            req = bytearray(592)
            struct.pack_into("<I", req, 0, line)
            req[256:256 + 11] = b"growatt-sim"
            struct.pack_into("<QI", req, 288, self._FLAG_OUTPUT, 1)
            # start in "receive": one attribute holding the initial output value
            struct.pack_into("<IIQQ", req, 320, self._ATTR_OUTPUT_VALUES, 0, 1 - self._send_level, 1)
            struct.pack_into("<I", req, 560, 1)
            fcntl.ioctl(chip_fd, self._GET_LINE, req)
            self._fd = struct.unpack_from("<i", req, 588)[0]
        finally:
            os.close(chip_fd)

    def _set(self, level: int) -> None:
        fcntl.ioctl(self._fd, self._SET_VALUES, struct.pack("<QQ", level, 1))

    def send(self) -> None:
        self._set(self._send_level)

    def receive(self) -> None:
        self._set(1 - self._send_level)

    def close(self) -> None:
        try:
            self.receive()
        finally:
            os.close(self._fd)


class RtsDirection:
    """DE wired to the RTS line of the serial port."""
    active = True

    def __init__(self, fd: int, active_high: bool = True) -> None:
        self._fd = fd
        self._active_high = active_high
        self.receive()

    def _rts(self, on: bool) -> None:
        fcntl.ioctl(self._fd, termios.TIOCMBIS if on else termios.TIOCMBIC,
                    struct.pack("I", termios.TIOCM_RTS))

    def send(self) -> None:
        self._rts(self._active_high)

    def receive(self) -> None:
        self._rts(not self._active_high)

    def close(self) -> None:
        pass


# ── Serial port ─────────────────────────────────────────────────────────────

def configure_tty(fd: int, baud: int, parity: str = "N", stopbits: int = 1) -> None:
    """Raw 8-bit mode at the given speed."""
    speed = getattr(termios, f"B{baud}", None)
    if speed is None:
        raise ValueError(f"baud rate {baud} is not supported")
    cflag = termios.CREAD | termios.CLOCAL | termios.CS8
    if parity in ("E", "O"):
        cflag |= termios.PARENB | (termios.PARODD if parity == "O" else 0)
    if stopbits == 2:
        cflag |= termios.CSTOPB
    attrs = termios.tcgetattr(fd)
    cc = list(attrs[6])
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, [0, 0, cflag, 0, speed, speed, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)


class SerialPort:
    """An open serial link: a tty device or a pty."""

    def __init__(self, link: dict) -> None:
        self.baud = int(link["baud"])
        self.char_s = (10 + (link["parity"] != "N") + (int(link["stopbits"]) - 1)) / self.baud
        self._extra_fd: Optional[int] = None
        self._symlink: Optional[str] = None
        self.direction = NoDirection()
        if link["port"] == "pty":
            self.fd, self._extra_fd = pty.openpty()
            configure_tty(self._extra_fd, self.baud, link["parity"], int(link["stopbits"]))
            os.set_blocking(self.fd, False)
            target = os.ttyname(self._extra_fd)
            self._symlink = link["pty_link"]
            try:
                if os.path.islink(self._symlink):
                    os.unlink(self._symlink)
                os.symlink(target, self._symlink)
            except OSError:
                self.close()
                raise
            self.device = f"{self._symlink} → {target}"
            return
        self.fd = os.open(link["port"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            configure_tty(self.fd, self.baud, link["parity"], int(link["stopbits"]))
            if link["de"] == "gpio":
                self.direction = GpioDirection(int(link["de_gpio"]), link["de_chip"], bool(link["de_active_high"]))
            elif link["de"] == "rts":
                self.direction = RtsDirection(self.fd, bool(link["de_active_high"]))
        except Exception:
            os.close(self.fd)
            raise
        self.device = os.path.realpath(link["port"])

    def read(self) -> bytes:
        try:
            return os.read(self.fd, 512)
        except BlockingIOError:
            return b""

    def write(self, data: bytes) -> float:
        """Send a frame, holding the transceiver in "send" for exactly its duration. Returns seconds."""
        duration = len(data) * self.char_s
        self.direction.send()
        started = time.perf_counter()
        view = memoryview(data)
        while view:
            try:
                view = view[os.write(self.fd, view):]
            except BlockingIOError:
                select.select([], [self.fd], [], 0.05)
        if self.direction.active:
            # The UART shifts the bytes out at line speed; release the bus right after the last one.
            # sleep() is coarse, so sleep most of the way and spin for the rest.
            deadline = started + duration + 0.0003
            remaining = deadline - time.perf_counter()
            if remaining > 0.002:
                time.sleep(remaining - 0.0015)
            while time.perf_counter() < deadline:
                pass
            self.direction.receive()
        return duration

    def close(self) -> None:
        try:
            self.direction.close()
        except OSError:
            pass
        for fd in (self.fd, self._extra_fd):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
        if self._symlink and os.path.islink(self._symlink):
            try:
                os.unlink(self._symlink)
            except OSError:
                pass


# ── RTU slave ───────────────────────────────────────────────────────────────

class RtuServer:
    def __init__(self, model: Inverter) -> None:
        self.model = model
        self._port: Optional[SerialPort] = None
        self._gen = -1
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="rtu-slave")
        self._buf = bytearray()
        self._last_rx = 0.0
        self._skip = False            # after a bad frame: ignore everything up to the next silence
        self._echo_left = 0           # bytes of our own reply still to come back (link setting "echo")
        self._echo_until = 0.0
        self._next_open = 0.0

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    # ── Port handling ────────────────────────────────────────────────────

    def _close(self) -> None:
        if self._port is not None:
            self._port.close()
            self._port = None

    def _open(self) -> None:
        link = dict(self.model.link)
        try:
            self._port = SerialPort(link)
        except (OSError, ValueError, termios.error) as exc:
            self._port = None
            self._next_open = time.monotonic() + REOPEN_S
            self.model.set_link_status("error", f"cannot open {link['port']}: {exc}")
            return
        self._buf.clear()
        self._skip = False
        self._echo_left = 0
        frame = f"{link['baud']} 8{link['parity']}{link['stopbits']}"
        self.model.set_link_status("open", f"{self._port.device}, {frame}, slave address {link['unit']}",
                                   self._port.device)

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._gen != self.model.link_gen:
                self._gen = self.model.link_gen
                self._close()
                self._next_open = 0.0
            if self._port is None:
                if time.monotonic() >= self._next_open:
                    self._open()
                if self._port is None:
                    self._stop.wait(0.1)
                    continue
            try:
                readable, _, _ = select.select([self._port.fd], [], [], 0.02)
                data = self._port.read() if readable else b""
            except OSError as exc:
                self._lost(exc)
                continue
            now = time.monotonic()
            if data:
                self._last_rx = now
                self._feed(data)
            elif readable:
                if self.model.link["port"] != "pty":
                    self._lost(OSError("the device returned end-of-file (unplugged?)"))
                else:
                    self._stop.wait(0.05)
            elif (self._buf or self._skip) and now - self._last_rx >= SILENCE_S:
                self._silence()
        self._close()

    def _lost(self, exc: OSError) -> None:
        if self.model.link["port"] == "pty" and exc.errno == 5:
            self._stop.wait(0.05)        # EIO: the master closed the virtual port
            return
        self._close()
        self._next_open = time.monotonic() + REOPEN_S
        self.model.set_link_status("error", f"port lost: {exc}")

    # ── Framing ──────────────────────────────────────────────────────────

    def _feed(self, data: bytes) -> None:
        if self._echo_left:
            # A transceiver with /RE tied low hands back every byte we send. That cannot be told
            # from a request by its content (an FC06 reply equals its request), so it is a link
            # setting, and exactly as many bytes as were sent are dropped.
            if time.monotonic() > self._echo_until:
                self._echo_left = 0
            else:
                n = min(len(data), self._echo_left)
                data, self._echo_left = data[n:], self._echo_left - n
        if self._skip or not data:
            return
        self._buf += data
        while self._buf:
            need = request_length(bytes(self._buf[:8]))
            if need is None or need == -1 or len(self._buf) < need:
                return                   # incomplete, or a frame only the silence can delimit
            frame = bytes(self._buf[:need])
            if not crc_ok(frame):
                self._garbage("CRC error", bytes(self._buf))
                self._buf.clear()
                self._skip = True
                return
            del self._buf[:need]
            self._handle(frame)

    def _silence(self) -> None:
        if self._buf:
            frame = bytes(self._buf)
            self._buf.clear()
            if crc_ok(frame):
                self._handle(frame)      # a complete frame with a function code we do not parse
            else:
                self._garbage("incomplete frame or noise", frame)
        self._skip = False

    def _garbage(self, what: str, data: bytes) -> None:
        self.model.count("crc_errors")
        self.model.log_txn("error", f"{what}: {len(data)} byte(s) dropped — check wiring, baud rate and A/B polarity",
                           "error", req=hexdump(data[:64]))

    # ── One request ──────────────────────────────────────────────────────

    def _handle(self, frame: bytes) -> None:
        model = self.model
        link = model.link
        settings = dict(model.settings)
        started = time.perf_counter()
        unit, pdu = frame[0], frame[1:-2]
        fc = pdu[0]
        info = {"unit": unit, "fc": fc, "req": hexdump(frame)}
        if unit not in (link["unit"], 0):
            model.count("other_unit")
            model.log_txn("error", f"FC{fc:02d} for slave address {unit} ignored — this inverter is "
                                   f"address {link['unit']}", "warn", **info)
            return
        model.note_request()
        what = f"FC{fc:02d}"

        def finish(kind: str, text: str, level: str, body: Optional[bytes], corrupt: bool = False, **extra) -> None:
            reply = None
            if body is not None and unit != 0:           # nobody answers a broadcast
                reply = add_crc(bytes([unit]) + body)
                if corrupt:
                    reply = reply[:-1] + bytes([reply[-1] ^ 0xFF])
                wait = float(link["turnaround_ms"]) / 1000.0 - (time.perf_counter() - started)
                if wait > 0:
                    time.sleep(wait)
                self._send(reply)
            model.log_txn(kind, text, level, resp=hexdump(reply) if reply else None,
                          ms=round((time.perf_counter() - started) * 1000, 1), **info, **extra)

        if settings["fault_mode"] == "silent":
            return finish("error", f"{what} received, no reply sent (fault injection)", "error", None)
        if settings["drop_pct"] and random.random() * 100.0 < float(settings["drop_pct"]):
            model.count("dropped")
            return finish("error", f"{what} received, reply dropped (lost requests {settings['drop_pct']:g} %)",
                          "error", None)
        try:
            if settings["fault_mode"] == "exception":
                raise ModbusError(int(settings["fault_exception_code"]), "fault injection")
            corrupt = settings["fault_mode"] == "bad_crc"
            note = "  (reply sent with a bad CRC: fault injection)" if corrupt else ""
            if fc in (FC_READ_HOLDING, FC_READ_INPUT):
                if len(pdu) != 5:
                    raise ModbusError(EXC_ILLEGAL_VALUE, "malformed read request")
                addr, count = struct.unpack(">HH", pdu[1:5])
                kind = "holding" if fc == FC_READ_HOLDING else "input"
                info.update(addr=addr, count=count, table=kind)
                what = f"read {kind} {addr} ×{count}"
                words = model.modbus_read(kind, addr, count)
                body = bytes([fc, 2 * count]) + struct.pack(f">{count}H", *words)
                return finish("error" if corrupt else "read", R.describe_range(kind, addr, words) + note,
                              "error" if corrupt else "info", body, corrupt=corrupt, words=words[:16])
            if fc in (FC_WRITE_SINGLE, FC_WRITE_MULTIPLE):
                if fc == FC_WRITE_SINGLE:
                    if len(pdu) != 5:
                        raise ModbusError(EXC_ILLEGAL_VALUE, "malformed write request")
                    addr, value = struct.unpack(">HH", pdu[1:5])
                    values = [value]
                else:
                    if len(pdu) < 6:
                        raise ModbusError(EXC_ILLEGAL_VALUE, "malformed write request")
                    addr, count, nbytes = struct.unpack(">HHB", pdu[1:6])
                    if not 1 <= count <= 123 or nbytes != 2 * count or len(pdu) != 6 + nbytes:
                        raise ModbusError(EXC_ILLEGAL_VALUE, "malformed write request")
                    values = list(struct.unpack(f">{count}H", pdu[6:]))
                info.update(addr=addr, count=len(values), table="holding")
                what = f"write holding {addr} ×{len(values)} (FC{fc:02d})"
                model.modbus_write(addr, values, fc)
                text = f"FC{fc:02d}  " + R.describe_range("holding", addr, values) + note
                return finish("error" if corrupt else "write", text, "error" if corrupt else "info",
                              pdu[:5], corrupt=corrupt, words=values[:16])
            raise ModbusError(EXC_ILLEGAL_FUNCTION, "supported: FC03, FC04, FC06 and FC16")
        except ModbusError as exc:
            finish("error", f"{what} → exception {exc.code:#04x} ({exc_name(exc.code)}): {exc.reason}",
                   "error", bytes([fc | 0x80, exc.code]), exc=exc.code)
        except struct.error:
            finish("error", f"{what} → exception 0x03: malformed request", "error",
                   bytes([fc | 0x80, EXC_ILLEGAL_VALUE]), exc=EXC_ILLEGAL_VALUE)

    def _send(self, reply: bytes) -> None:
        try:
            duration = self._port.write(reply)
        except OSError as exc:
            self._lost(exc)
            return
        if self.model.link["echo"]:
            self._echo_left = len(reply)
            self._echo_until = time.monotonic() + duration + 0.25
