"""
HTTP side of the simulator (standard library only): the web UI, its JSON API,
the live event stream (Server-Sent Events), and the REST endpoints of the
PC-side growatt-modbus-mock so existing test scripts keep working.
"""

from __future__ import annotations

import json
import logging
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from . import __version__
from . import registers as R
from .model import BAUD_RATES, SETTINGS_META, Inverter, ModbusError

log = logging.getLogger("growatt_sim.web")

STATIC_DIR = Path(__file__).parent / "static"

# PV per roof: MPPT1 = north, 2.7 kWp; MPPT2 = south, 6.75 kWp
PRESETS = [
    {"label": "Night", "plant": {"pv1_w": 0, "pv2_w": 0}},
    {"label": "Overcast", "plant": {"pv1_w": 250, "pv2_w": 700}},
    {"label": "Bright", "plant": {"pv1_w": 900, "pv2_w": 3300}},
    {"label": "Full sun", "plant": {"pv1_w": 1700, "pv2_w": 5600}},
]


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, model: Inverter) -> None:
        super().__init__(address, Handler)
        self.model = model
        self.stopping = threading.Event()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "growatt-sim/" + __version__
    server: Server

    def log_message(self, fmt, *args) -> None:      # keep the terminal for events
        log.debug("%s %s", self.address_string(), fmt % args)

    @property
    def model(self) -> Inverter:
        return self.server.model

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code: int = 200) -> None:
        self._send(code, json.dumps(obj).encode(), "application/json")

    def _json_body(self) -> dict:
        if not self._request_body.strip():
            return {}
        data = json.loads(self._request_body)
        if not isinstance(data, dict):
            raise ValueError("the request body must be a JSON object")
        return data

    def _dispatch(self) -> None:
        url = urlsplit(self.path)
        query = {k: v[-1] for k, v in parse_qs(url.query).items()}
        # Always consume the body, so an early error cannot leave it in the
        # way of the next request on this keep-alive connection.
        length = int(self.headers.get("Content-Length") or 0)
        self._request_body = self.rfile.read(length) if length > 0 else b""
        try:
            handler = ROUTES.get((self.command, url.path))
            if handler is None:
                self._json({"error": f"no such endpoint: {self.command} {url.path}"}, 404)
            else:
                handler(self, query)
        except ModbusError as exc:
            self._json({"error": exc.reason}, 400)
        except (ValueError, KeyError, TypeError) as exc:
            self._json({"error": str(exc) or exc.__class__.__name__}, 400)
        except (BrokenPipeError, ConnectionResetError):
            pass

    do_GET = _dispatch
    do_POST = _dispatch

    # ── Pages and read-outs ──────────────────────────────────────────────

    def page(self, query) -> None:
        self._send(200, (STATIC_DIR / "index.html").read_bytes(), "text/html; charset=utf-8")

    def meta(self, query) -> None:
        self._json({
            "version": __version__,
            "registers": [r.to_json() for r in R.REGISTERS],
            "settings": SETTINGS_META,
            "status_codes": [[k, v] for k, v in R.STATUS_CODES.items()],
            "priorities": [[k, v] for k, v in R.PRIORITY_NAMES.items()],
            "baud_rates": list(BAUD_RATES),
            "presets": PRESETS,
            "ui_port": self.server.server_address[1],
        })

    def state(self, query) -> None:
        self._json(self.model.snapshot())

    def log_entries(self, query) -> None:
        self._json(self.model.log_since(int(query.get("since", 0))))

    def history(self, query) -> None:
        self._json(self.model.history_list())

    def events(self, query) -> None:
        """Server-Sent Events: a `state` snapshot on every change and new `log` entries."""
        model = self.model
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        self.close_connection = True
        last_id = self.headers.get("Last-Event-ID", "")
        since = int(last_id) if last_id.isdigit() else max(0, model.seq - 300)
        if since > model.seq:
            since = 0
        version = -1
        try:
            while not self.server.stopping.is_set():
                with model.changed:
                    if model.version == version:
                        model.changed.wait(timeout=1.0)
                    version = model.version
                out = []
                entries = model.log_since(since)
                if entries:
                    since = entries[-1]["seq"]
                    out.append(f"id: {since}\nevent: log\ndata: {json.dumps(entries)}\n\n")
                out.append(f"event: state\ndata: {json.dumps(model.snapshot())}\n\n")
                self.wfile.write("".join(out).encode())
                self.wfile.flush()
                time.sleep(0.15)        # at most ~6 pushes per second
        except OSError:
            pass

    # ── Controls ─────────────────────────────────────────────────────────

    def _done(self) -> None:
        self._json(self.model.snapshot())

    def post_plant(self, query) -> None:
        self.model.set_plant(**self._json_body())
        self._done()

    def post_register(self, query) -> None:
        body = self._json_body()
        if "rid" not in body or ("value" not in body and "raw" not in body):
            raise ValueError('expected {"rid": "i3171", "value": X} or {"rid": "h3047", "raw": [..]}')
        self.model.set_register(body["rid"], value=body.get("value"), raw=body.get("raw"))
        self._done()

    def post_release(self, query) -> None:
        self.model.release(self._json_body().get("rid"))
        self._done()

    def post_slot(self, query) -> None:
        body = self._json_body()
        if "n" not in body:
            raise ValueError('expected {"n": 1, "enabled": true, "priority": 1, "start": "21:01", "end": "23:59"}')
        self.model.set_slot(body["n"], enabled=body.get("enabled"), priority=body.get("priority"),
                            start=body.get("start"), end=body.get("end"))
        self._done()

    def post_clock(self, query) -> None:
        self.model.set_clock(self._json_body().get("time"))
        self._done()

    def post_settings(self, query) -> None:
        self.model.update_settings(self._json_body())
        self._done()

    def post_link(self, query) -> None:
        self.model.set_link(**self._json_body())
        self._done()

    def post_reset(self, query) -> None:
        self.model.reset(settings=bool(self._json_body().get("settings")))
        self._done()

    def post_log_clear(self, query) -> None:
        self.model.clear_log()
        self._json({"ok": True})

    # ── REST endpoints of the PC-side growatt-modbus-mock ────────────────

    def compat_status(self, query) -> None:
        self._json(self.model.status_compat())

    def compat_set(self, query) -> None:
        self.model.set_compat(self._json_body())
        self._json(self.model.status_compat())

    def compat_scenario(self, query) -> None:
        self.model.apply_scenario(self._json_body())
        self._json(self.model.status_compat())

    def compat_reset(self, query) -> None:
        self.model.reset()
        self._json(self.model.status_compat())


ROUTES = {
    ("GET", "/"): Handler.page,
    ("GET", "/index.html"): Handler.page,
    ("GET", "/api/meta"): Handler.meta,
    ("GET", "/api/state"): Handler.state,
    ("GET", "/api/log"): Handler.log_entries,
    ("GET", "/api/history"): Handler.history,
    ("GET", "/api/events"): Handler.events,
    ("POST", "/api/plant"): Handler.post_plant,
    ("POST", "/api/register"): Handler.post_register,
    ("POST", "/api/release"): Handler.post_release,
    ("POST", "/api/slot"): Handler.post_slot,
    ("POST", "/api/clock"): Handler.post_clock,
    ("POST", "/api/settings"): Handler.post_settings,
    ("POST", "/api/link"): Handler.post_link,
    ("POST", "/api/reset"): Handler.post_reset,
    ("POST", "/api/log/clear"): Handler.post_log_clear,
    ("GET", "/status"): Handler.compat_status,
    ("POST", "/set"): Handler.compat_set,
    ("POST", "/scenario"): Handler.compat_scenario,
    ("POST", "/reset"): Handler.compat_reset,
}


def start_server(host: str, port: int, model: Inverter) -> Server:
    server = Server((host, port), model)
    threading.Thread(target=server.serve_forever, daemon=True, name="ui-http").start()
    return server


class EvFollower(threading.Thread):
    """Reads the wallbox simulator's active power once a second while "ev_follow" is on."""

    def __init__(self, model: Inverter) -> None:
        super().__init__(daemon=True, name="ev-follow")
        self.model = model
        self.stop = threading.Event()

    def poll(self) -> None:
        base = str(self.model.settings["ev_url"]).rstrip("/")
        if not base.startswith(("http://", "https://")):
            self.model.set_ev_follow(False, error="the address must start with http://")
            return
        try:
            with urllib.request.urlopen(base + "/api/state", timeout=1.5) as response:
                state = json.loads(response.read())
            watts = next(reg["value"] for reg in state["registers"] if reg.get("addr") == 12)
            self.model.set_ev_follow(True, float(watts))
        except Exception as exc:      # any failure means "no wallbox data"
            self.model.set_ev_follow(False, error=str(exc)[:120] or exc.__class__.__name__)

    def run(self) -> None:
        while not self.stop.wait(1.0):
            if self.model.settings["ev_follow"]:
                self.poll()
