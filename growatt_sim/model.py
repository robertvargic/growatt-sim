"""
The simulated inverter.

One `Inverter` object holds the plant (PV, house and EV load, battery, grid),
the holding registers the master wrote, the behaviour switches and the traffic
log. The RTU server and the web UI work on it through its public methods;
every one of them takes `self.lock`.

Energy balance, recomputed on every read:

    load first (no TOU slot active)
        PV surplus charges the battery, a deficit discharges it,
        the grid takes or gives the rest.
    battery first (an enabled TOU slot with that priority is active)
        the battery charges at the allowed rate: from PV, and from the grid
        as well when AC charge (3049) is on and SoC is below the stop SoC.
    grid first
        the battery discharges at full power; the surplus is exported.

Seen on the real inverter and reproduced here (see README):

- A write is acknowledged at once but applies ~0.4 s later.
- An FC06 write to a TOU slot start word is acknowledged and ignored.
- Register 3047 limits the charge power: 50 % was 4.8 kW.
- Switching grid charge on inside the window starts charging within seconds;
  outside the window it only arms the inverter.
"""

from __future__ import annotations

import logging
import random
import secrets
import threading
import time
from collections import deque
from datetime import datetime
from typing import Callable, Optional

from . import registers as R
from .registers import HOLDING_DEFAULTS, REGISTERS, REG_BY_RID, TOU_SLOT_REGS, WORD_OWNER, Reg

log = logging.getLogger("growatt_sim")

EXC_ILLEGAL_FUNCTION = 1
EXC_ILLEGAL_ADDRESS = 2
EXC_ILLEGAL_VALUE = 3

EXC_NAMES = {1: "illegal function", 2: "illegal data address", 3: "illegal data value",
             4: "server device failure", 6: "server device busy"}

MPPT_VOLTS = (240.0, 600.0)        # 6 panels on MPPT1 (north), 15 on MPPT2 (south)
# What the controller holds back for the battery with its default settings.
CTRL_RESERVE_W, CTRL_FULL_SOC = 3000.0, 90


class ModbusError(Exception):
    """A request the inverter answers with a Modbus exception response."""

    def __init__(self, code: int, reason: str) -> None:
        super().__init__(reason)
        self.code = code
        self.reason = reason


def exc_name(code: int) -> str:
    return EXC_NAMES.get(code, "exception")


# Behaviour switches. The UI builds its controls from this list.
SETTINGS_META: list[dict] = [
    # ── Plant ────────────────────────────────────────────────────────────
    {"key": "battery_kwh", "group": "plant", "type": "number", "default": 20.0,
     "min": 0.1, "max": 1000, "unit": "kWh",
     "label": "Battery capacity",
     "help": "Sets how fast the SoC moves for a given battery power."},
    {"key": "max_charge_kw", "group": "plant", "type": "number", "default": 9.6,
     "min": 0, "max": 100, "unit": "kW",
     "label": "Charge power at 100 %",
     "help": "Battery charge power when register 3047 is 100. Measured on the real battery: 50 % = 4.8 kW."},
    {"key": "max_discharge_kw", "group": "plant", "type": "number", "default": 10.0,
     "min": 0, "max": 100, "unit": "kW",
     "label": "Max discharge power",
     "help": "The battery covers a deficit up to this power; the grid supplies the rest."},
    {"key": "min_soc", "group": "plant", "type": "number", "default": 10,
     "min": 0, "max": 100, "unit": "%",
     "label": "Discharge stops at",
     "help": "Below this SoC the battery no longer discharges."},
    {"key": "time_speed", "group": "plant", "type": "number", "default": 1.0,
     "min": 0, "max": 3600, "unit": "×",
     "label": "Battery and energy speed",
     "help": "Multiplies how fast the SoC and the energy counters move. 0 freezes them."},
    {"key": "pv_noise_pct", "group": "plant", "type": "number", "default": 0.0,
     "min": 0, "max": 50, "unit": "%",
     "label": "PV flicker",
     "help": "Random variation of the PV power, like passing clouds."},
    # ── Inverter behaviour ───────────────────────────────────────────────
    {"key": "write_apply_delay_ms", "group": "inverter", "type": "number", "default": 400,
     "min": 0, "max": 10000, "unit": "ms",
     "label": "Writes apply after",
     "help": "A write is acknowledged at once, but a read shows the old value for this long. "
             "The real inverter takes 300–500 ms."},
    {"key": "fc06_ignored_on_tou_start", "group": "inverter", "type": "bool", "default": True,
     "label": "FC06 to a TOU start word is ignored",
     "help": "Acknowledged but not applied, as on the real inverter. A slot has to go out as one FC16 pair."},
    {"key": "strict_addresses", "group": "inverter", "type": "bool", "default": False,
     "label": "Reject unmapped addresses",
     "help": "Off: other addresses read 0 (or the loaded scan), and holding registers accept writes."},
    # ── Coupling with the wallbox simulator ──────────────────────────────
    {"key": "ev_follow", "group": "coupling", "type": "bool", "default": False,
     "label": "Take the EV load from em2go-sim",
     "help": "Adds the simulated wallbox's power to the load, so export drops when the car charges."},
    {"key": "ev_url", "group": "coupling", "type": "text", "default": "http://127.0.0.1:5021",
     "label": "em2go-sim address",
     "help": "Base address of the wallbox simulator's web UI."},
    # ── Fault injection ──────────────────────────────────────────────────
    {"key": "fault_mode", "group": "fault", "type": "select", "default": "none",
     "options": [["none", "Normal"],
                 ["silent", "No reply (master times out)"],
                 ["exception", "Exception reply"],
                 ["bad_crc", "Reply with a bad CRC"]],
     "label": "Modbus link",
     "help": "What the master gets back for every request."},
    {"key": "fault_exception_code", "group": "fault", "type": "number", "default": 4,
     "min": 1, "max": 255, "unit": "",
     "label": "Exception code",
     "help": "Sent while the link is set to Exception reply."},
    {"key": "drop_pct", "group": "fault", "type": "number", "default": 0,
     "min": 0, "max": 100, "unit": "%",
     "label": "Lost requests",
     "help": "Share of requests that get no reply, at random: a flaky bus."},
]
_SETTING_BY_KEY = {m["key"]: m for m in SETTINGS_META}

LINK_DEFAULTS = {
    "port": "/dev/serial0",       # tty device, or "pty" for a virtual port
    "pty_link": "/tmp/ttyGROWATT",
    "baud": 9600,
    "parity": "N",                # N | E | O
    "stopbits": 1,
    "unit": 1,                    # Modbus slave address
    "de": "none",                 # none | gpio | rts
    "de_gpio": 18,
    "de_chip": "/dev/gpiochip0",
    "de_active_high": True,
    "echo": False,                # the transceiver hands back what is sent (/RE tied low)
    "turnaround_ms": 10,
}
BAUD_RATES = (1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200)


class Inverter:
    TICK_S = 0.5      # simulation step of the tick thread

    def __init__(self, clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time,
                 settings: Optional[dict] = None, link: Optional[dict] = None,
                 baseline: Optional[dict] = None, baseline_name: str = "") -> None:
        self._clock = clock
        self._wall = wall
        self.lock = threading.RLock()
        self.changed = threading.Condition()   # notified on every visible change
        self.version = 0
        self.boot_id = secrets.token_hex(4)    # lets the UI notice a restarted simulator
        self.settings: dict = {m["key"]: m["default"] for m in SETTINGS_META}
        self.link: dict = dict(LINK_DEFAULTS)
        self.link_gen = 0                      # bumped when the link settings change
        self.link_status = {"state": "closed", "detail": "not opened yet", "device": ""}
        self.log_entries: deque = deque(maxlen=2000)
        self.seq = 0
        self.log_gen = 0
        self.history: deque = deque(maxlen=900)   # one sample per second, 15 min
        self.counters = {"requests": 0, "crc_errors": 0, "other_unit": 0, "dropped": 0}
        self.last_request_t: Optional[float] = None
        self._baseline = baseline or {"input": {}, "holding": {}}
        self.baseline_name = baseline_name
        self._factory()
        if link:
            self.set_link(**link)
        if settings:
            self.update_settings(settings)

    # ── Factory state ────────────────────────────────────────────────────

    def _factory(self) -> None:
        now = self._clock()
        self.plant = {"pv1_w": 0.0, "pv2_w": 0.0, "house_w": 400.0, "ev_w": 0.0}
        self.soc = 50.0
        self.vac = [230.0, 230.0, 230.0]
        self.fac = 50.0
        self.temp_c = 35.0
        self.status = 1
        self.fault_code = 0
        self.warning_code = 0
        self.e_today_kwh = 0.0
        self.e_total_kwh = 2000.0
        self.holding: dict[int, int] = dict(HOLDING_DEFAULTS)
        self.pending: list[dict] = []          # acknowledged writes waiting to apply
        self.clock_offset_s = 0.0
        self.pins: dict[str, list[int]] = {}
        # Values at addresses outside the register map: the loaded scan, then what was set or written.
        self.extras: dict[str, dict[int, int]] = {
            "input": dict(self._baseline.get("input", {})),
            "holding": dict(self._baseline.get("holding", {})),
        }
        for addr in list(self.extras["holding"]):
            if ("holding", addr) in WORD_OWNER:     # a scan also sets the modelled holding registers
                self.holding[addr] = self.extras["holding"].pop(addr)
        for addr in list(self.extras["input"]):
            if ("input", addr) in WORD_OWNER:
                del self.extras["input"][addr]
        self.user_extras: set[tuple[str, int]] = set()
        self.stats = {r.rid: {"r": 0, "w": 0, "tr": 0.0, "tw": 0.0} for r in REGISTERS}
        self.ev_follow = {"ok": None, "w": 0.0, "error": ""}
        self._noise = [0.0, 0.0]
        self._last_tick = now
        self._last_sample = 0.0
        self._last_mode = None
        self._sig = None

    # ── Change notification and log ──────────────────────────────────────

    def _bump(self) -> None:
        with self.changed:
            self.version += 1
            self.changed.notify_all()

    def _append(self, kind: str, text: str, level: str, fields: dict) -> None:
        self.seq += 1
        entry = {"seq": self.seq, "t": time.time(), "kind": kind, "text": text, "level": level}
        entry.update(fields)
        self.log_entries.append(entry)

    def _event(self, text: str, level: str = "info") -> None:
        self._append("event", text, level, {})
        log.log(logging.WARNING if level != "info" else logging.INFO, "%s", text)

    def event(self, text: str, level: str = "info") -> None:
        """Record something that happened outside the model (serial link, EV coupling)."""
        with self.lock:
            self._event(text, level)
        self._bump()

    def log_txn(self, kind: str, text: str, level: str = "info", **fields) -> None:
        """Record one Modbus transaction (called by the RTU server)."""
        with self.lock:
            self._append(kind, text, level, fields)
        if kind == "read":
            log.debug("modbus read: %s", text)
        elif kind == "write":
            log.info("modbus write: %s", text)
        else:
            log.warning("modbus: %s", text)
        self._bump()

    def log_since(self, seq: int) -> list[dict]:
        with self.lock:
            if seq > self.seq:        # the caller remembers a previous run
                seq = 0
            return [e for e in self.log_entries if e["seq"] > seq]

    def clear_log(self) -> None:
        with self.lock:
            self.log_entries.clear()
            self.log_gen += 1
        self._bump()

    # ── Inverter clock and TOU slots ─────────────────────────────────────

    def inverter_time(self) -> datetime:
        return datetime.fromtimestamp(self._wall() + self.clock_offset_s)

    def _slots(self) -> list[dict]:
        now = self.inverter_time()
        t = (now.hour, now.minute)
        slots = []
        for n, reg in enumerate(TOU_SLOT_REGS, start=1):
            start = R.decode_tou_start(self.holding[reg])
            end = R.decode_tou_end(self.holding[reg + 1])
            begin = (start["hour"], start["minute"])
            crosses = end < begin
            slots.append({
                "n": n, "reg": reg, "enabled": start["enabled"], "priority": start["priority"],
                "start": R.hhmm(*begin), "end": R.hhmm(*end),
                "crosses_midnight": crosses,
                # A slot cannot cross midnight on the inverter: such a slot never runs.
                "active": start["enabled"] and not crosses and begin <= t <= end,
            })
        return slots

    def _active_slot(self) -> Optional[dict]:
        for slot in self._slots():
            if slot["active"]:
                return slot
        return None

    # ── Energy balance ───────────────────────────────────────────────────

    def _ev_w(self) -> float:
        if self.settings["ev_follow"]:
            return float(self.ev_follow["w"]) if self.ev_follow["ok"] else 0.0
        return self.plant["ev_w"]

    def _flows(self) -> dict:
        s = self.settings
        pv1 = max(0.0, self.plant["pv1_w"] * (1.0 + self._noise[0]))
        pv2 = max(0.0, self.plant["pv2_w"] * (1.0 + self._noise[1]))
        pv = pv1 + pv2
        house, ev = self.plant["house_w"], self._ev_w()
        load = house + ev
        slot = self._active_slot()
        mode = slot["priority"] if slot else R.PRIO_LOAD
        limit_w = float(s["max_charge_kw"]) * 1000.0 * min(100, self.holding[R.REG_CHARGE_POWER]) / 100.0
        max_dis = float(s["max_discharge_kw"]) * 1000.0
        can_charge = self.soc < 100.0 - 1e-9
        can_discharge = self.soc > float(s["min_soc"]) + 1e-9
        charge = discharge = 0.0
        grid_charge = False
        if mode == R.PRIO_BATTERY:
            grid_charge = (bool(self.holding[R.REG_AC_CHARGE]) and can_charge
                           and self.soc < self.holding[R.REG_STOP_SOC])
            if grid_charge:
                charge = limit_w
            elif can_charge:
                charge = min(limit_w, pv)
        elif mode == R.PRIO_GRID:
            if can_discharge:
                discharge = max_dis
        else:
            surplus = pv - load
            if surplus > 0 and can_charge:
                charge = min(surplus, limit_w)
            elif surplus < 0 and can_discharge:
                discharge = min(-surplus, max_dis)
        grid_net = load + charge - discharge - pv          # > 0 import, < 0 export
        return {
            "pv1": pv1, "pv2": pv2, "pv": pv, "house": house, "ev": ev, "load": load,
            "charge": charge, "discharge": discharge, "grid_net": grid_net,
            "import": max(0.0, grid_net), "export": max(0.0, -grid_net),
            "pac": max(0.0, pv + discharge - charge),
            "mode": mode, "slot": slot["n"] if slot else None,
            "grid_charge": grid_charge, "charge_limit_w": limit_w,
        }

    def _values(self, flows: Optional[dict] = None) -> dict:
        f = flows or self._flows()
        volts = [MPPT_VOLTS[i] if p >= 1.0 else 0.0 for i, p in enumerate((f["pv1"], f["pv2"]))]
        amps = [p / v if v else 0.0 for p, v in zip((f["pv1"], f["pv2"]), volts)]
        values = {
            "status": self.status,
            "mppt1_v": volts[0], "mppt1_a": amps[0], "pv1_w": f["pv1"],
            "mppt2_v": volts[1], "mppt2_a": amps[1], "pv2_w": f["pv2"],
            "pac_w": f["pac"], "fac": self.fac,
            "vac1": self.vac[0], "vac2": self.vac[1], "vac3": self.vac[2],
            "e_today": self.e_today_kwh, "e_total": self.e_total_kwh,
            "temp": self.temp_c, "fault_code": self.fault_code, "warning_code": self.warning_code,
            "grid_import_w": f["import"], "grid_export_w": f["export"],
            "soc": int(self.soc + 1e-6),
            "bat_discharge_w": f["discharge"], "bat_charge_w": f["charge"],
        }
        return values

    def _words(self, reg: Reg, values: dict) -> list[int]:
        pinned = self.pins.get(reg.rid)
        if pinned is not None:
            return pinned
        if reg.kind == "holding":
            return [self.holding[reg.addr]]
        return R.encode(reg, values[reg.value_key], clamp=True)

    def _image(self, kind: str, values: Optional[dict] = None) -> dict[int, int]:
        """Every mapped wire address of one kind -> the word a master reads right now."""
        if values is None:
            values = self._values()
        image: dict[int, int] = {}
        for reg in REGISTERS:
            if reg.kind == kind:
                for i, word in enumerate(self._words(reg, values)):
                    image[reg.addr + i] = word
        return image

    # ── Simulation tick ──────────────────────────────────────────────────

    def _apply_due(self, now: float) -> None:
        due = [p for p in self.pending if p["due"] <= now]
        if not due:
            return
        self.pending = [p for p in self.pending if p["due"] > now]
        applied = []
        for item in due:
            self.holding[item["addr"]] = item["value"]
            reg = WORD_OWNER[("holding", item["addr"])][0]
            applied.append(f"{reg.label} = {R.format_words(reg, [item['value']])}")
        self._event("Applied: " + "; ".join(applied))
        self._note_mode()

    def _note_mode(self) -> None:
        """Log when the battery mode changes (a slot opens, grid charge starts or stops)."""
        f = self._flows()
        mode = (f["mode"], f["slot"], f["grid_charge"])
        if mode == self._last_mode:
            return
        first = self._last_mode is None
        self._last_mode = mode
        if first:
            return
        if f["mode"] == R.PRIO_LOAD:
            self._event("Battery mode: load first (no TOU slot active)")
        elif f["grid_charge"]:
            self._event(f"Battery mode: battery first (TOU slot {f['slot']}) — charging from the grid "
                        f"at up to {f['charge_limit_w'] / 1000:.2f} kW")
        else:
            self._event(f"Battery mode: {R.PRIORITY_NAMES.get(f['mode'], '?')} (TOU slot {f['slot']})")

    def tick(self) -> None:
        """Advance the simulation. Called every TICK_S by the tick thread."""
        with self.lock:
            now = self._clock()
            dt = max(0.0, min(now - self._last_tick, 5.0))
            self._last_tick = now
            self._apply_due(now)
            noise = float(self.settings["pv_noise_pct"]) / 100.0
            self._noise = [random.gauss(0.0, noise) if noise > 0 else 0.0 for _ in range(2)]
            self._note_mode()

            f = self._flows()
            hours = dt / 3600.0 * float(self.settings["time_speed"])
            capacity = float(self.settings["battery_kwh"])
            self.soc = max(0.0, min(100.0, self.soc + (f["charge"] - f["discharge"]) / 1000.0 * hours
                                    / capacity * 100.0))
            self.e_today_kwh += f["pac"] / 1000.0 * hours
            self.e_total_kwh += f["pac"] / 1000.0 * hours

            wall = time.time()
            if wall - self._last_sample >= 1.0:
                self._last_sample = wall
                self.history.append({
                    "t": round(wall, 2), "pv": round(f["pv"]), "load": round(f["load"]),
                    "grid": round(f["grid_net"]), "bat": round(f["discharge"] - f["charge"]),
                })

            values = self._values(f)
            sig = (tuple(sorted(self._image("input", values).items())),
                   tuple(sorted(self.holding.items())), len(self.pending),
                   tuple(sorted(self.link_status.items())), len(self.history),
                   self.ev_follow["ok"], self.inverter_time().strftime("%H:%M"))
            changed = sig != self._sig
            self._sig = sig
        if changed:
            self._bump()

    # ── Modbus side ──────────────────────────────────────────────────────

    def note_request(self) -> None:
        with self.lock:
            self.counters["requests"] += 1
            self.last_request_t = time.time()

    def count(self, name: str) -> None:
        with self.lock:
            self.counters[name] += 1

    def modbus_read(self, kind: str, addr: int, count: int) -> list[int]:
        with self.lock:
            self._apply_due(self._clock())
            if not 1 <= count <= 125:
                raise ModbusError(EXC_ILLEGAL_VALUE, f"a read of {count} registers is not a valid request")
            if addr + count > 0x10000:
                raise ModbusError(EXC_ILLEGAL_ADDRESS, "read past address 65535")
            image = self._image(kind)
            extras = self.extras[kind]
            words = []
            for a in range(addr, addr + count):
                if a in image:
                    words.append(image[a])
                elif a in extras:
                    words.append(extras[a])
                elif self.settings["strict_addresses"]:
                    raise ModbusError(EXC_ILLEGAL_ADDRESS, f"{kind} register {a} is not in the register map")
                else:
                    words.append(0)
            wall = time.time()
            for rid in {WORD_OWNER[(kind, a)][0].rid for a in range(addr, addr + count)
                        if (kind, a) in WORD_OWNER}:
                self.stats[rid]["r"] += 1
                self.stats[rid]["tr"] = wall
            return words

    def modbus_write(self, addr: int, values: list[int], fc: int) -> None:
        """FC06 / FC16 on holding registers: acknowledge now, apply after the delay."""
        with self.lock:
            now = self._clock()
            self._apply_due(now)
            if addr + len(values) > 0x10000:
                raise ModbusError(EXC_ILLEGAL_ADDRESS, "write past address 65535")
            plan = []
            for i, value in enumerate(values):
                a = addr + i
                owner = WORD_OWNER.get(("holding", a))
                if owner is None:
                    if self.settings["strict_addresses"]:
                        raise ModbusError(EXC_ILLEGAL_ADDRESS, f"holding register {a} is not in the register map")
                    plan.append((None, a, value))
                    continue
                self._validate(owner[0], value)
                plan.append((owner[0], a, value))
            delay = float(self.settings["write_apply_delay_ms"]) / 1000.0
            wall = time.time()
            for reg, a, value in plan:
                if reg is None:
                    self.extras["holding"][a] = value
                    self.user_extras.add(("holding", a))
                    self._event(f"Write to holding register {a} = {value}: the simulator does not model "
                                "this register and only stores the value", "warn")
                    continue
                self.stats[reg.rid]["w"] += 1
                self.stats[reg.rid]["tw"] = wall
                if fc == 6 and reg.dtype == "tou_start" and self.settings["fc06_ignored_on_tou_start"]:
                    self._event(f"FC06 write to {reg.label} ({a}) acknowledged but ignored, as on the real "
                                "inverter. Write the start and end word as one FC16 pair.", "warn")
                    continue
                self.pending = [p for p in self.pending if p["addr"] != a]
                self.pending.append({"due": now + delay, "addr": a, "value": value})
            if delay <= 0:
                self._apply_due(now)
        self._bump()

    def _validate(self, reg: Reg, value: int) -> None:
        if not 0 <= value <= 0xFFFF:
            raise ModbusError(EXC_ILLEGAL_VALUE, f"{reg.label}: {value} does not fit a register")
        if reg.dtype == "tou_start":
            d = R.decode_tou_start(value)
            if d["hour"] > 23 or d["minute"] > 59 or d["priority"] > 2:
                raise ModbusError(EXC_ILLEGAL_VALUE, f"{reg.label}: 0x{value:04X} is not a valid slot start")
        elif reg.dtype == "tou_end":
            hour, minute = R.decode_tou_end(value)
            if hour > 23 or minute > 59 or value >> 13:
                raise ModbusError(EXC_ILLEGAL_VALUE, f"{reg.label}: 0x{value:04X} is not a valid time")
        elif reg.key in ("charge_power_pct", "stop_soc") and value > 100:
            raise ModbusError(EXC_ILLEGAL_VALUE, f"{reg.label}: {value} % is more than 100")
        elif reg.enum is not None and value not in reg.enum:
            raise ModbusError(EXC_ILLEGAL_VALUE, f"{reg.label} = {value} is not a valid value")

    # ── Test controls (web UI / REST) ────────────────────────────────────

    def set_plant(self, **changes) -> None:
        with self.lock:
            for key, value in changes.items():
                if value is None:
                    continue
                if key in ("pv1_w", "pv2_w", "house_w", "ev_w"):
                    self.plant[key] = _number(value, 0, 1_000_000, key)
                elif key == "soc":
                    self.soc = _number(value, 0, 100, "SoC")
                elif key == "grid_v":
                    self.vac = [_number(value, 0, 6553, "grid voltage")] * 3
                else:
                    raise ValueError(f"unknown plant field {key!r}")
            self._note_mode()
        self._bump()

    def set_register(self, rid: str, value=None, raw=None) -> None:
        """
        Set a register from the UI. A derived register is pinned to the value,
        a parameter is changed, and a holding register is written at once.
        `rid` is i<addr> or h<addr>; other addresses store a raw value.
        """
        with self.lock:
            rid = str(rid).strip().lower()
            reg = REG_BY_RID.get(rid)
            if reg is None:
                kind = {"i": "input", "h": "holding"}.get(rid[:1])
                if kind is None or not rid[1:].isdigit():
                    raise ValueError(f"{rid!r} is not a register id, use i<address> or h<address>")
                addr = int(rid[1:])
                if (kind, addr) in WORD_OWNER:
                    owner = WORD_OWNER[(kind, addr)][0]
                    raise ValueError(f"{kind} address {addr} is part of register {owner.addr} ({owner.label})")
                if not 0 <= addr <= 0xFFFF:
                    raise ValueError("address must be 0–65535")
                word = raw[0] if isinstance(raw, list) and raw else (raw if raw is not None else value)
                self.extras[kind][addr] = int(_number(word, 0, 0xFFFF, "value"))
                self.user_extras.add((kind, addr))
                self._event(f"Raw {kind} register [{addr}] = {self.extras[kind][addr]} (UI)")
            else:
                if raw is not None:
                    words = [int(_number(w, 0, 0xFFFF, "raw word")) for w in
                             (raw if isinstance(raw, list) else [raw])]
                    if len(words) != reg.count:
                        raise ValueError(f"{reg.label} needs {reg.count} raw word(s)")
                else:
                    words = R.encode(reg, value)
                text = R.format_words(reg, words)
                if reg.role == "derived":
                    self.pins[rid] = words
                    self._event(f"{reg.label} pinned to {text} (UI)")
                elif reg.role == "param":
                    self._set_param(reg.value_key, R.decode(reg, words))
                    self._event(f"{reg.label} = {text} (UI)")
                else:
                    self._validate(reg, words[0])
                    self.pending = [p for p in self.pending if p["addr"] != reg.addr]
                    self.holding[reg.addr] = words[0]
                    self._event(f"{reg.label} = {text} (UI)")
            self._note_mode()
        self._bump()

    def _set_param(self, key: str, value) -> None:
        if key in ("pv1_w", "pv2_w"):
            self.plant[key] = float(value)
        elif key == "soc":
            self.soc = float(_number(value, 0, 100, "SoC"))
        elif key in ("vac1", "vac2", "vac3"):
            self.vac[int(key[-1]) - 1] = float(value)
        elif key == "fac":
            self.fac = float(value)
        elif key == "temp":
            self.temp_c = float(value)
        elif key in ("status", "fault_code", "warning_code"):
            setattr(self, key, int(value))
        elif key == "e_today":
            self.e_today_kwh = float(value)
        elif key == "e_total":
            self.e_total_kwh = float(value)
        else:
            raise ValueError(f"cannot set {key}")

    def release(self, rid: Optional[str] = None) -> None:
        """Hand a pinned register back to the simulation (all of them if rid is None)."""
        with self.lock:
            if rid is None:
                if self.pins:
                    self._event("All pinned registers released")
                self.pins.clear()
            elif self.pins.pop(str(rid), None) is not None:
                self._event(f"{REG_BY_RID[str(rid)].label} released — back to the simulated value")
        self._bump()

    def set_slot(self, n: int, enabled=None, priority=None, start=None, end=None) -> None:
        """Edit one TOU slot from the UI, like changing it in the ShinePhone app."""
        with self.lock:
            n = int(n)
            if not 1 <= n <= len(TOU_SLOT_REGS):
                raise ValueError(f"there is no TOU slot {n}")
            reg = TOU_SLOT_REGS[n - 1]
            cur = R.decode_tou_start(self.holding[reg])
            begin = R.parse_hhmm(start) if start is not None else (cur["hour"], cur["minute"])
            finish = R.parse_hhmm(end) if end is not None else R.decode_tou_end(self.holding[reg + 1])
            prio = int(priority) if priority is not None else cur["priority"]
            if prio not in R.PRIORITY_NAMES:
                raise ValueError(f"priority must be 0, 1 or 2, not {priority!r}")
            on = bool(enabled) if enabled is not None else cur["enabled"]
            self.pending = [p for p in self.pending if p["addr"] not in (reg, reg + 1)]
            self.holding[reg] = R.encode_tou_start(on, prio, *begin)
            self.holding[reg + 1] = R.encode_tou_end(*finish)
            self._event(f"TOU slot {n} = {'on' if on else 'off'}, {R.PRIORITY_NAMES[prio]}, "
                        f"{R.hhmm(*begin)}–{R.hhmm(*finish)} (UI)")
            self._note_mode()
        self._bump()

    def set_clock(self, hhmm: Optional[str] = None) -> None:
        """Set the inverter's clock to HH:MM today, or back to the system time."""
        with self.lock:
            if hhmm in (None, ""):
                self.clock_offset_s = 0.0
                self._event("Inverter clock follows the system time again")
            else:
                hour, minute = R.parse_hhmm(hhmm)
                real = datetime.fromtimestamp(self._wall())
                target = real.replace(hour=hour, minute=minute, second=0, microsecond=0)
                self.clock_offset_s = (target - real).total_seconds()
                self._event(f"Inverter clock set to {R.hhmm(hour, minute)}")
            self._note_mode()
        self._bump()

    def update_settings(self, changes: dict) -> None:
        with self.lock:
            for key, value in changes.items():
                meta = _SETTING_BY_KEY.get(key)
                if meta is None:
                    raise ValueError(f"unknown setting {key!r}")
                if meta["type"] == "bool":
                    if isinstance(value, str):
                        value = value.strip().lower() in ("1", "true", "yes", "on")
                    value = bool(value)
                elif meta["type"] == "number":
                    value = _number(value, meta["min"], meta["max"], meta["label"])
                    if isinstance(meta["default"], int):
                        value = int(value)
                elif meta["type"] == "text":
                    value = str(value).strip()
                    if len(value) > 200:
                        raise ValueError(f"{meta['label']}: too long")
                elif value not in [o[0] for o in meta["options"]]:
                    raise ValueError(f"{meta['label']}: unknown option {value!r}")
                if self.settings[key] != value:
                    self.settings[key] = value
                    self._event(f"Setting: {meta['label']} = {value}")
                    if key in ("ev_follow", "ev_url"):
                        self.ev_follow = {"ok": None, "w": 0.0, "error": ""}
            self._note_mode()
        self._bump()

    def set_ev_follow(self, ok: bool, watts: float = 0.0, error: str = "") -> None:
        """Result of one poll of the wallbox simulator."""
        with self.lock:
            before = self.ev_follow["ok"]
            self.ev_follow = {"ok": ok, "w": float(watts), "error": error}
            if ok != before:
                if ok:
                    self._event("EV load now follows em2go-sim")
                else:
                    self._event(f"em2go-sim is not reachable ({error}) — EV load taken as 0", "warn")

    def set_link(self, **changes) -> None:
        """Change the serial link settings; the RTU server reopens the port."""
        with self.lock:
            link = dict(self.link)
            for key, value in changes.items():
                if value is None:
                    continue
                if key not in LINK_DEFAULTS:
                    raise ValueError(f"unknown link setting {key!r}")
                if key in ("port", "pty_link", "de_chip"):
                    value = str(value).strip()
                    if not value:
                        raise ValueError(f"{key} must not be empty")
                elif key == "baud":
                    value = int(_number(value, 0, 10_000_000, "baud rate"))
                    if value not in BAUD_RATES:
                        raise ValueError(f"baud rate must be one of {', '.join(map(str, BAUD_RATES))}")
                elif key == "parity":
                    value = str(value).upper()[:1]
                    if value not in ("N", "E", "O"):
                        raise ValueError("parity must be N, E or O")
                elif key == "stopbits":
                    value = int(_number(value, 1, 2, "stop bits"))
                elif key == "unit":
                    value = int(_number(value, 1, 247, "slave address"))
                elif key == "de":
                    if value not in ("none", "gpio", "rts"):
                        raise ValueError("direction control must be none, gpio or rts")
                elif key == "de_gpio":
                    value = int(_number(value, 0, 255, "GPIO line"))
                elif key in ("de_active_high", "echo"):
                    value = value.strip().lower() in ("1", "true", "yes", "on") if isinstance(value, str) else bool(value)
                elif key == "turnaround_ms":
                    value = _number(value, 0, 5000, "turnaround delay")
                link[key] = value
            if link != self.link:
                self.link = link
                self.link_gen += 1
                self._event(f"Serial link settings changed: {link['port']} {link['baud']} 8{link['parity']}"
                            f"{link['stopbits']}, slave address {link['unit']}")
        self._bump()

    def set_link_status(self, state: str, detail: str = "", device: str = "") -> None:
        with self.lock:
            status = {"state": state, "detail": detail, "device": device}
            if status == self.link_status:
                return
            self.link_status = status
            self._event(f"Serial link {state}" + (f": {detail}" if detail else ""),
                        "warn" if state == "error" else "info")
        self._bump()

    def reset(self, settings: bool = False) -> None:
        """Back to the resting state of the inverter at night."""
        with self.lock:
            if settings:
                self.settings = {m["key"]: m["default"] for m in SETTINGS_META}
            self._factory()
            self._event("Inverter reset to factory state" + (" and default settings" if settings else ""))
        self._bump()

    # ── Read-outs ────────────────────────────────────────────────────────

    def _controller_view(self, image: dict[int, int], ev_w: float) -> dict:
        """The surplus as power_math.available_for_ev_w() computes it from what the master reads."""
        def u32(addr: int) -> float:
            return ((image[addr] << 16) | image[addr + 1]) * 0.1
        grid_net = u32(3041) - u32(3043) - ev_w           # the controller takes its own EV load out
        bat_net = u32(3178) - u32(3180)                   # > 0 discharge
        surplus = max(0.0, -grid_net) - bat_net
        reserve = 0.0 if image[3171] >= CTRL_FULL_SOC else CTRL_RESERVE_W
        return {"surplus_w": round(surplus), "reserve_w": round(reserve),
                "available_w": round(max(0.0, surplus - reserve))}

    def snapshot(self) -> dict:
        with self.lock:
            now = self._clock()
            wall = time.time()
            self._apply_due(now)
            flows = self._flows()
            values = self._values(flows)
            regs = []
            for reg in REGISTERS:
                words = self._words(reg, values)
                stat = self.stats[reg.rid]
                regs.append({
                    "rid": reg.rid, "words": list(words),
                    "value": R.decode(reg, words), "text": R.format_words(reg, words),
                    "pinned": reg.rid in self.pins,
                    "r": stat["r"], "w": stat["w"], "tr": stat["tr"], "tw": stat["tw"],
                })
            inverter_time = self.inverter_time()
            return {
                "boot_id": self.boot_id, "t": wall, "seq": self.seq, "log_gen": self.log_gen,
                "flows": {k: (round(v, 1) if isinstance(v, float) else v) for k, v in flows.items()},
                "soc": round(self.soc, 2),
                "plant": dict(self.plant),
                "grid_v": self.vac[0],
                "slots": self._slots(),
                "charge_power_pct": self.holding[R.REG_CHARGE_POWER],
                "stop_soc": self.holding[R.REG_STOP_SOC],
                "ac_charge": bool(self.holding[R.REG_AC_CHARGE]),
                "pending": [{"addr": p["addr"], "value": p["value"],
                             "in_ms": max(0, round((p["due"] - now) * 1000))} for p in self.pending],
                "inverter_time": inverter_time.strftime("%Y-%m-%d %H:%M:%S"),
                "clock_offset_s": round(self.clock_offset_s),
                "controller": self._controller_view(self._image("input", values), flows["ev"]),
                "link": dict(self.link), "link_gen": self.link_gen, "link_status": dict(self.link_status),
                "counters": dict(self.counters),
                "last_request_age_s": (round(wall - self.last_request_t, 1)
                                       if self.last_request_t is not None else None),
                "ev_follow": dict(self.ev_follow),
                "registers": regs,
                "extras": [[kind, addr, self.extras[kind][addr]] for kind, addr in sorted(self.user_extras)],
                "baseline": {"name": self.baseline_name,
                             "input": len(self._baseline.get("input", {})),
                             "holding": len(self._baseline.get("holding", {}))},
                "settings": dict(self.settings),
                "sample": self.history[-1] if self.history else None,
            }

    def history_list(self) -> list[dict]:
        with self.lock:
            return list(self.history)

    # ── REST shape of the PC-side growatt-modbus-mock ────────────────────

    def status_compat(self) -> dict:
        with self.lock:
            image = self._image("input")

            def u32(addr: int) -> float:
                return round(((image[addr] << 16) | image[addr + 1]) * 0.1, 1)
            return {
                "pv1_w": u32(3005), "pv2_w": u32(3009),
                "grid_import_w": u32(3041), "grid_export_w": u32(3043),
                "grid_v": round(image[3034] * 0.1, 1), "bat_soc": image[3171],
                "bat_discharge_w": u32(3178), "bat_charge_w": u32(3180),
            }

    def set_compat(self, payload: dict) -> None:
        """POST /set of growatt-modbus-mock: PV, SoC and voltage are inputs here, the rest is pinned."""
        with self.lock:
            self.set_plant(pv1_w=payload.get("pv1_w"), pv2_w=payload.get("pv2_w"),
                           soc=payload.get("bat_soc"), grid_v=payload.get("grid_v"))
            for key, rid in (("grid_import_w", "i3041"), ("grid_export_w", "i3043"),
                             ("bat_discharge_w", "i3178"), ("bat_charge_w", "i3180")):
                if payload.get(key) is not None:
                    self.set_register(rid, value=payload[key])

    def apply_scenario(self, cfg: dict) -> None:
        """POST /scenario of growatt-modbus-mock: raw input registers (u16 / u32_raw / u32_scaled)."""
        with self.lock:
            items: list[tuple[int, list[int]]] = []
            for addr, value in (cfg.get("u16") or {}).items():
                items.append((int(addr), [int(value) & 0xFFFF]))
            for addr, value in (cfg.get("u32_raw") or {}).items():
                raw = int(value)
                items.append((int(addr), [raw >> 16 & 0xFFFF, raw & 0xFFFF]))
            for addr, spec in (cfg.get("u32_scaled") or {}).items():
                raw = int(round(float(spec["value"]) / float(spec.get("scale", 0.1))))
                items.append((int(addr), [raw >> 16 & 0xFFFF, raw & 0xFFFF]))
            for addr, words in items:
                reg = REG_BY_RID.get(f"i{addr}")
                if reg is not None and reg.count == len(words):
                    self.set_register(reg.rid, raw=words)
                else:
                    for i, word in enumerate(words):
                        self.set_register(f"i{addr + i}", raw=[word])


def parse_scan(data: dict) -> dict:
    """
    Read a register scan of the real inverter into {"input": {addr: word}, "holding": {...}}.

    Understands the three formats in use on the Pi:
      gw_scan.py / gw_last.json    {"h3038": 13569, "i3005": 0, ...}
      gw_scan.json                 {"holding 0-124": {"0": 1, ...}, "input 3000-3124": {...}}
      scan_diff.py                 {"HR:3038": 13569, "IN:0": 1, ...}
    """
    out: dict[str, dict[int, int]] = {"input": {}, "holding": {}}
    for key, value in data.items():
        key = str(key)
        if isinstance(value, dict):
            kind = "holding" if key.lower().startswith("h") else "input" if key.lower().startswith("i") else None
            if kind:
                for addr, word in value.items():
                    out[kind][int(addr)] = int(word) & 0xFFFF
        elif key[:3] in ("HR:", "IN:"):
            out["holding" if key[:2] == "HR" else "input"][int(key[3:])] = int(value) & 0xFFFF
        elif key[:1] in ("h", "i") and key[1:].isdigit():
            out["holding" if key[0] == "h" else "input"][int(key[1:])] = int(value) & 0xFFFF
    if not out["input"] and not out["holding"]:
        raise ValueError("no registers found in the scan file")
    return out


def _number(value, lo, hi, what: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{what}: {value!r} is not a number") from None
    if number != number or not lo <= number <= hi:
        raise ValueError(f"{what} must be between {lo:g} and {hi:g}")
    return number
