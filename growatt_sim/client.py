"""
Minimal Modbus RTU master and command line tool — for checking the wire.

Run it on the side the controller reads from (the USB RS-485 adapter):

    python3 -m growatt_sim.client test                       # 50 reads: is the link clean?
    python3 -m growatt_sim.client dump                       # every register of the simulator
    python3 -m growatt_sim.client read input 3005 2          # FC04
    python3 -m growatt_sim.client read holding 3038 12       # FC03
    python3 -m growatt_sim.client write 3047 25              # FC06
    python3 -m growatt_sim.client write16 3038 0xB501 0x173B # FC16, one TOU slot
    python3 -m growatt_sim.client watch                      # poll like the controller

Defaults: /dev/ttyUSB0, 9600 8N1, slave address 1. Stop the controller first:
two masters on one RS-485 line corrupt each other's frames.
"""

from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import time

from . import registers as R
from .rtu import add_crc, configure_tty, crc_ok


class RtuError(Exception):
    pass


class RtuException(RtuError):
    def __init__(self, fc: int, code: int) -> None:
        super().__init__(f"exception {code:#04x} for FC{fc:02d}")
        self.fc = fc
        self.code = code


class RtuMaster:
    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 9600, parity: str = "N",
                 stopbits: int = 1, unit: int = 1, timeout: float = 1.0) -> None:
        self.unit, self.timeout = unit, timeout
        self._gap = max(0.005, 3.5 * 11 / baud)
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            configure_tty(self.fd, baud, parity, stopbits)
        except Exception:
            os.close(self.fd)
            raise

    def close(self) -> None:
        os.close(self.fd)

    def _read_frame(self) -> bytes:
        buf = bytearray()
        deadline = time.monotonic() + self.timeout
        while True:
            need = 5
            if len(buf) >= 3 and not buf[1] & 0x80:
                need = 5 + buf[2] if buf[1] in (1, 2, 3, 4) else 8
            if len(buf) >= need:
                return bytes(buf[:need])
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RtuError("no reply" if not buf else f"incomplete reply ({bytes(buf).hex(' ')})")
            if select.select([self.fd], [], [], remaining)[0]:
                try:
                    buf += os.read(self.fd, 512)
                except BlockingIOError:
                    pass

    def request(self, pdu: bytes) -> bytes:
        time.sleep(self._gap)                    # bus silence before a frame
        try:
            while os.read(self.fd, 512):         # nothing stale in the buffer
                pass
        except BlockingIOError:
            pass
        os.write(self.fd, add_crc(bytes([self.unit]) + pdu))
        frame = self._read_frame()
        if not crc_ok(frame):
            raise RtuError(f"reply with a bad CRC ({frame.hex(' ')})")
        if frame[0] != self.unit:
            raise RtuError(f"reply from slave address {frame[0]}, expected {self.unit}")
        if frame[1] & 0x80:
            raise RtuException(frame[1] & 0x7F, frame[2])
        return frame[1:-2]

    def read(self, kind: str, addr: int, count: int = 1) -> list[int]:
        body = self.request(struct.pack(">BHH", 3 if kind == "holding" else 4, addr, count))
        return list(struct.unpack(f">{body[1] // 2}H", body[2:2 + body[1]]))

    def write_single(self, addr: int, value: int) -> None:
        self.request(struct.pack(">BHH", 6, addr, value))

    def write_multiple(self, addr: int, values: list[int]) -> None:
        self.request(struct.pack(">BHHB", 16, addr, len(values), 2 * len(values))
                     + struct.pack(f">{len(values)}H", *values))


def _dump(master: RtuMaster) -> None:
    print(f"{'table':<8}{'addr':>5}  {'register':<24}{'raw':<12}value")
    for reg in R.REGISTERS:
        try:
            words = master.read(reg.kind, reg.addr, reg.count)
            raw = " ".join(f"{w:04X}" for w in words)
            print(f"{reg.kind:<8}{reg.addr:>5}  {reg.label:<24}{raw:<12}{R.format_words(reg, words)}")
        except RtuError as exc:
            print(f"{reg.kind:<8}{reg.addr:>5}  {reg.label:<24}{'-':<12}{exc}")


def _test(master: RtuMaster, count: int) -> int:
    ok, times, errors = 0, [], {}
    for _ in range(count):
        started = time.perf_counter()
        try:
            master.read("input", 3171, 1)
            ok += 1
            times.append((time.perf_counter() - started) * 1000)
        except RtuError as exc:
            errors[str(exc).split(" (")[0]] = errors.get(str(exc).split(" (")[0], 0) + 1
    print(f"{ok} of {count} requests answered")
    if times:
        print(f"round trip: min {min(times):.1f} ms, median {sorted(times)[len(times) // 2]:.1f} ms, "
              f"max {max(times):.1f} ms")
    for reason, n in errors.items():
        print(f"  {n} × {reason}")
    if ok == 0:
        print("Nothing came back. Check: A to A and B to B (swap them if unsure), the same baud rate and\n"
              "slave address on both sides, and that the simulator shows the port as open.")
    elif ok < count:
        print("Some requests were lost. With a DE-controlled transceiver on the pins, check the\n"
              "direction settings; with long wires, add termination and a common ground.")
    return 0 if ok == count else 1


def _watch(master: RtuMaster, interval: float) -> None:
    keys = ("pv1_w", "pv2_w", "grid_import_w", "grid_export_w", "soc", "bat_discharge_w", "bat_charge_w")
    while True:
        stamp = time.strftime("%H:%M:%S")
        try:
            parts = []
            for key in keys:
                reg = R.REG_BY_KEY[key]
                parts.append(f"{key}={R.format_words(reg, master.read(reg.kind, reg.addr, reg.count))}")
            print(stamp, "  ".join(parts), flush=True)
        except RtuError as exc:
            print(stamp, exc, flush=True)
        time.sleep(interval)


def main(argv: list[str] | None = None) -> int:
    number = lambda text: int(text, 0)      # noqa: E731  (accepts 0x.. as well)
    ap = argparse.ArgumentParser(prog="growatt_sim.client", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--parity", default="N", choices=["N", "E", "O"])
    ap.add_argument("--unit", type=int, default=1, help="slave address (default 1)")
    ap.add_argument("--timeout", type=float, default=1.0)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("test", help="send a series of reads and report how many were answered")
    p.add_argument("--count", type=int, default=50)
    sub.add_parser("dump", help="read every register the simulator models")
    p = sub.add_parser("read", help="read COUNT registers starting at ADDR")
    p.add_argument("kind", choices=["input", "holding"])
    p.add_argument("addr", type=number)
    p.add_argument("count", type=number, nargs="?", default=1)
    p = sub.add_parser("write", help="write one holding register (FC06)")
    p.add_argument("addr", type=number)
    p.add_argument("value", type=number)
    p = sub.add_parser("write16", help="write holding registers starting at ADDR (FC16)")
    p.add_argument("addr", type=number)
    p.add_argument("values", type=number, nargs="+")
    p = sub.add_parser("watch", help="poll the main registers in a loop")
    p.add_argument("--interval", type=float, default=2.0)
    args = ap.parse_args(argv)

    try:
        master = RtuMaster(args.port, args.baud, args.parity, 1, args.unit, args.timeout)
    except OSError as exc:
        print(f"error: cannot open {args.port}: {exc}", file=sys.stderr)
        return 2
    try:
        if args.cmd == "test":
            return _test(master, args.count)
        if args.cmd == "dump":
            _dump(master)
        elif args.cmd == "read":
            words = master.read(args.kind, args.addr, args.count)
            print(" ".join(str(w) for w in words), " |", R.describe_range(args.kind, args.addr, words, limit=20))
        elif args.cmd == "write":
            master.write_single(args.addr, args.value)
            print("acknowledged |", R.describe_range("holding", args.addr, [args.value]))
        elif args.cmd == "write16":
            master.write_multiple(args.addr, args.values)
            print("acknowledged |", R.describe_range("holding", args.addr, args.values))
        elif args.cmd == "watch":
            _watch(master, args.interval)
    except RtuError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        master.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
