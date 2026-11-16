"""
Growatt inverter simulator.

    python3 -m growatt_sim --port /dev/serial0     # RS-485 transceiver on the GPIO UART
    python3 -m growatt_sim --port /dev/ttyUSB1     # a second USB RS-485 adapter
    python3 -m growatt_sim --port pty              # no hardware: virtual port at /tmp/ttyGROWATT

Open http://<this host>:5031/ for the UI. Everything given here can be changed there.
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import socket
import sys
import threading

from . import __version__
from .model import LINK_DEFAULTS, SETTINGS_META, Inverter, parse_scan
from .rtu import RtuServer
from .web import EvFollower, start_server


def _lan_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("192.0.2.1", 9))        # no packet is sent
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def main(argv: list[str] | None = None) -> int:
    keys = ", ".join(m["key"] for m in SETTINGS_META)
    d = LINK_DEFAULTS
    ap = argparse.ArgumentParser(prog="growatt_sim", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_argument_group("serial link")
    g.add_argument("--port", default=d["port"],
                   help=f'serial device, or "pty" for a virtual port (default {d["port"]})')
    g.add_argument("--pty-link", default=d["pty_link"],
                   help=f'where the virtual port appears (default {d["pty_link"]})')
    g.add_argument("--baud", type=int, default=d["baud"], help="default 9600")
    g.add_argument("--parity", default=d["parity"], choices=["N", "E", "O"], help="default N")
    g.add_argument("--stopbits", type=int, default=d["stopbits"], choices=[1, 2])
    g.add_argument("--unit", type=int, default=d["unit"], help="Modbus slave address (default 1)")
    g.add_argument("--de", default=d["de"], choices=["none", "gpio", "rts"],
                   help="RS-485 direction control: none = the adapter does it (default), "
                        "gpio = a GPIO line drives DE and /RE, rts = the RTS line does")
    g.add_argument("--de-gpio", type=int, default=d["de_gpio"], help="GPIO line for --de gpio (default 18)")
    g.add_argument("--de-chip", default=d["de_chip"], help=f'GPIO chip (default {d["de_chip"]})')
    g.add_argument("--de-active-low", action="store_true", help="DE is active low")
    g.add_argument("--echo", action="store_true",
                   help="the transceiver hands back what is sent (/RE tied low)")
    g.add_argument("--turnaround-ms", type=float, default=d["turnaround_ms"],
                   help="wait this long before a reply (default 10)")
    ap.add_argument("--host", default="0.0.0.0", help="bind address of the web UI (default 0.0.0.0)")
    ap.add_argument("--ui-port", type=int, default=5031, help="web UI and control API port (default 5031)")
    ap.add_argument("--baseline", metavar="FILE",
                    help="register scan of the real inverter (gw_scan.py / scan_diff.py JSON): "
                         "addresses the simulator does not model answer with these values")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help=f"behaviour switch, repeatable. Keys: {keys}")
    ap.add_argument("-v", "--verbose", action="store_true", help="also log every Modbus read")
    ap.add_argument("--version", action="version", version=f"growatt-sim {__version__}")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s", datefmt="%H:%M:%S")

    settings = {}
    for item in args.set:
        key, sep, value = item.partition("=")
        if not sep:
            ap.error(f"--set {item!r}: expected KEY=VALUE")
        settings[key.strip()] = value.strip()
    baseline = None
    if args.baseline:
        try:
            with open(args.baseline) as fh:
                baseline = parse_scan(json.load(fh))
        except (OSError, ValueError) as exc:
            ap.error(f"--baseline {args.baseline}: {exc}")
    link = {"port": args.port, "pty_link": args.pty_link, "baud": args.baud, "parity": args.parity,
            "stopbits": args.stopbits, "unit": args.unit, "de": args.de, "de_gpio": args.de_gpio,
            "de_chip": args.de_chip, "de_active_high": not args.de_active_low, "echo": args.echo,
            "turnaround_ms": args.turnaround_ms}
    try:
        model = Inverter(settings=settings, link=link, baseline=baseline, baseline_name=args.baseline or "")
    except ValueError as exc:
        ap.error(str(exc))

    try:
        ui = start_server(args.host, args.ui_port, model)
    except OSError as exc:
        print(f"error: cannot open port {args.ui_port}: {exc}", file=sys.stderr)
        return 1
    rtu = RtuServer(model)
    rtu.start()
    follower = EvFollower(model)
    follower.start()

    stop = threading.Event()

    def ticker() -> None:
        while not stop.wait(Inverter.TICK_S):
            try:
                model.tick()
            except Exception:       # a bug in one tick must not freeze the simulation
                logging.getLogger("growatt_sim").exception("simulation tick failed")

    threading.Thread(target=ticker, daemon=True, name="sim-tick").start()

    shown = _lan_ip() if args.host == "0.0.0.0" else args.host
    master_port = args.pty_link if args.port == "pty" else "/dev/ttyUSB0"
    wire = ("virtual port, no wiring" if args.port == "pty"
            else f"{args.port}, wired A-A / B-B to the adapter the controller reads")
    print("\n".join([
        "",
        f"  Growatt simulator {__version__}",
        f"    Web UI        http://{shown}:{args.ui_port}/",
        f"    Modbus RTU    {wire}",
        f"                  {args.baud} 8{args.parity}{args.stopbits}, slave address {args.unit}",
        "",
        "  Check the link from the controller's side:",
        f"    python3 -m growatt_sim.client --port {master_port} test",
        "",
        "  Run the controller against it (with em2go-sim for the wallbox):",
        f"    cd ~/solar_charger && GROWATT_PORT={master_port} PC_IP=127.0.0.1 ENV=integration venv/bin/python main.py",
        "",
        "  Ctrl+C stops the simulator.",
        "",
    ]), flush=True)

    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while not stop.wait(0.5):
            pass
    except KeyboardInterrupt:
        pass
    stop.set()
    follower.stop.set()
    rtu.stop()
    ui.stopping.set()
    ui.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
