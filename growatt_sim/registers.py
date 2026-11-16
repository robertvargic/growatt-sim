"""
Growatt register map — the registers the solar charger uses.

Input registers (FC04) are the ones `growatt_reader.py` reads: single values
in the 3000 bank plus one batch read of 0-124. Holding registers (FC03 to
read, FC06 / FC16 to write) are the battery and time-of-use settings that
`night_charge.py` reads and writes. Addresses, scaling and the bit layout of
the TOU words are taken from those two modules, which were verified against
the live inverter.

A register has one of three roles:

  derived   computed by the simulation (grid and battery power, AC output,
            MPPT voltage and current). Setting it from the UI *pins* it.
  param     an input of the simulation (PV power, SoC, voltages, status,
            fault codes, energy counters).
  rw        a holding register the Modbus master can write.

Several registers may show the same model value (`source`): MPPT power is
readable at 5 and at 3005, the voltage of L3 at 46 and at 3034.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

STATUS_CODES = {0: "Waiting", 1: "Normal", 3: "Fault"}
PRIORITY_NAMES = {0: "load first", 1: "battery first", 2: "grid first"}
PRIO_LOAD, PRIO_BATTERY, PRIO_GRID = 0, 1, 2
ONOFF = {0: "Off", 1: "On"}

# Start-word registers of the six TOU slots; each is followed by its end word.
TOU_SLOT_REGS = (3038, 3040, 3042, 3044, 3050, 3052)
REG_CHARGE_POWER = 3047
REG_STOP_SOC = 3048
REG_AC_CHARGE = 3049


@dataclass(frozen=True)
class Reg:
    kind: str                 # input | holding
    addr: int
    key: str                  # unique name
    label: str
    count: int = 1
    dtype: str = "u16"        # u16 | u32 | tou_start | tou_end
    role: str = "derived"     # derived | param | rw
    source: str = ""          # model value it shows (defaults to key)
    unit: str = ""
    scale: float = 1.0        # value in `unit` = raw * scale
    enum: Optional[dict] = None
    note: str = ""
    used_by: str = ""         # controller module that uses it

    @property
    def rid(self) -> str:
        """Short id used by the API: i3005, h3047."""
        return f"{self.kind[0]}{self.addr}"

    @property
    def value_key(self) -> str:
        return self.source or self.key

    @property
    def decimals(self) -> int:
        return 2 if self.scale < 0.1 else 1 if self.scale < 1 else 0

    @property
    def max_raw(self) -> int:
        return 0xFFFFFFFF if self.dtype == "u32" else 0xFFFF

    def to_json(self) -> dict:
        return {
            "rid": self.rid, "kind": self.kind, "addr": self.addr, "key": self.key,
            "label": self.label, "count": self.count, "dtype": self.dtype, "role": self.role,
            "unit": self.unit, "scale": self.scale, "decimals": self.decimals,
            "note": self.note, "used_by": self.used_by,
            "enum": [[k, v] for k, v in self.enum.items()] if self.enum else None,
        }


def _inp(addr, key, label, **kw) -> Reg:
    kw.setdefault("used_by", "reader")
    return Reg("input", addr, key, label, **kw)


def _slot(n: int, start_reg: int) -> list[Reg]:
    return [
        Reg("holding", start_reg, f"tou{n}_start", f"TOU slot {n} start", dtype="tou_start", role="rw",
            used_by="night charge", note="bit 15 on, bits 14-13 priority, hour, minute"),
        Reg("holding", start_reg + 1, f"tou{n}_end", f"TOU slot {n} end", dtype="tou_end", role="rw",
            used_by="night charge", note="hour, minute"),
    ]


REGISTERS: list[Reg] = [
    # ── Input registers 0-124: read in one batch ────────────────────────────
    _inp(0,   "status",        "Inverter status", role="param", enum=STATUS_CODES, note="1 = on grid"),
    _inp(3,   "mppt1_v",       "MPPT1 voltage", unit="V", scale=0.1),
    _inp(4,   "mppt1_a",       "MPPT1 current", unit="A", scale=0.1),
    _inp(5,   "mppt1_w",       "MPPT1 power", count=2, dtype="u32", role="param", source="pv1_w",
         unit="W", scale=0.1, note="north roof"),
    _inp(7,   "mppt2_v",       "MPPT2 voltage", unit="V", scale=0.1),
    _inp(8,   "mppt2_a",       "MPPT2 current", unit="A", scale=0.1),
    _inp(9,   "mppt2_w",       "MPPT2 power", count=2, dtype="u32", role="param", source="pv2_w",
         unit="W", scale=0.1, note="south roof"),
    _inp(35,  "pac_w",         "AC output power", count=2, dtype="u32", unit="W", scale=0.1),
    _inp(37,  "fac",           "Grid frequency", role="param", unit="Hz", scale=0.01),
    _inp(38,  "vac1",          "AC voltage L1", role="param", unit="V", scale=0.1),
    _inp(42,  "vac2",          "AC voltage L2", role="param", unit="V", scale=0.1),
    _inp(46,  "vac3",          "AC voltage L3", role="param", unit="V", scale=0.1),
    _inp(53,  "e_today",       "Energy today", count=2, dtype="u32", role="param", unit="kWh", scale=0.1),
    _inp(91,  "e_total",       "Energy total", count=2, dtype="u32", role="param", unit="kWh", scale=0.1),
    _inp(93,  "temp",          "Heatsink temperature", role="param", unit="°C", scale=0.1),
    _inp(105, "fault_code",    "Fault code", role="param"),
    _inp(106, "warning_code",  "Warning code", role="param"),
    # ── Input registers, 3000 bank: read one value at a time ────────────────
    _inp(3005, "pv1_w",        "PV1 power", count=2, dtype="u32", role="param", unit="W", scale=0.1,
         note="same value as register 5"),
    _inp(3009, "pv2_w",        "PV2 power", count=2, dtype="u32", role="param", unit="W", scale=0.1,
         note="same value as register 9"),
    _inp(3034, "grid_v",       "Grid voltage", role="param", source="vac3", unit="V", scale=0.1,
         note="L3, same value as register 46"),
    _inp(3041, "grid_import_w", "Grid import", count=2, dtype="u32", unit="W", scale=0.1),
    _inp(3043, "grid_export_w", "Grid export", count=2, dtype="u32", unit="W", scale=0.1),
    _inp(3171, "soc",          "Battery SoC", role="param", unit="%"),
    _inp(3178, "bat_discharge_w", "Battery discharge", count=2, dtype="u32", unit="W", scale=0.1),
    _inp(3180, "bat_charge_w", "Battery charge", count=2, dtype="u32", unit="W", scale=0.1),
    # ── Holding registers: battery and time-of-use settings ─────────────────
    *_slot(1, 3038), *_slot(2, 3040), *_slot(3, 3042), *_slot(4, 3044),
    Reg("holding", REG_CHARGE_POWER, "charge_power_pct", "Battery charge power", role="rw", unit="%",
        used_by="night charge", note="FC06 works"),
    Reg("holding", REG_STOP_SOC, "stop_soc", "Charge stop SoC", role="rw", unit="%",
        used_by="night charge", note="read only by the controller"),
    Reg("holding", REG_AC_CHARGE, "ac_charge", "AC (grid) charge", role="rw", enum=ONOFF,
        used_by="night charge", note="FC06 works"),
    *_slot(5, 3050), *_slot(6, 3052),
]

# Resting state of the live inverter (holding-register scan of 2026-11-16): every slot disabled,
# 100 % charge power, grid charge off.
HOLDING_DEFAULTS: dict[int, int] = {
    3038: 0x3501, 3039: 0x173B,     # slot 1  battery first  21:01-23:59
    3040: 0x2000, 3041: 0x063B,     # slot 2  battery first  00:00-06:59
    3042: 0x2101, 3043: 0x063B,     # slot 3  battery first  01:01-06:59
    3044: 0x2700, 3045: 0x081E,     # slot 4  battery first  07:00-08:30
    3047: 100, 3048: 100, 3049: 0,
    3050: 0x1000, 3051: 0x1600,     # slot 5  load first     16:00-22:00
    3052: 0x4C00, 3053: 0x0E00,     # slot 6  grid first     12:00-14:00
}

REG_BY_RID: dict[str, Reg] = {r.rid: r for r in REGISTERS}
REG_BY_KEY: dict[str, Reg] = {r.key: r for r in REGISTERS}
# (kind, wire address) -> (register, index of the word in it)
WORD_OWNER: dict[tuple[str, int], tuple[Reg, int]] = {
    (r.kind, r.addr + i): (r, i) for r in REGISTERS for i in range(r.count)
}


# ── TOU words ───────────────────────────────────────────────────────────────

def encode_tou_start(enabled: bool, priority: int, hour: int, minute: int) -> int:
    return ((1 if enabled else 0) << 15) | ((priority & 0x3) << 13) | ((hour & 0x1F) << 8) | (minute & 0xFF)


def encode_tou_end(hour: int, minute: int) -> int:
    return ((hour & 0x1F) << 8) | (minute & 0xFF)


def decode_tou_start(word: int) -> dict:
    return {"enabled": bool(word >> 15 & 1), "priority": word >> 13 & 0x3,
            "hour": word >> 8 & 0x1F, "minute": word & 0xFF}


def decode_tou_end(word: int) -> tuple[int, int]:
    return word >> 8 & 0x1F, word & 0xFF


def parse_hhmm(text: str) -> tuple[int, int]:
    try:
        hh, mm = str(text).strip().split(":")[:2]
        hour, minute = int(hh), int(mm)
    except ValueError:
        raise ValueError(f"{text!r} is not a time, use HH:MM") from None
    if not (0 <= hour <= 23 and 0 <= minute <= 59):
        raise ValueError(f"{text!r} is not a valid time")
    return hour, minute


def hhmm(hour: int, minute: int) -> str:
    return f"{hour:02d}:{minute:02d}"


# ── Encode / decode ─────────────────────────────────────────────────────────

def encode(reg: Reg, value, clamp: bool = False) -> list[int]:
    """Value in engineering units -> list of 16-bit words (big endian)."""
    try:
        raw = int(round(float(value) / reg.scale))
    except (TypeError, ValueError):
        raise ValueError(f"{reg.label}: {value!r} is not a number") from None
    if clamp:
        raw = max(0, min(raw, reg.max_raw))
    elif not 0 <= raw <= reg.max_raw:
        raise ValueError(f"{reg.label}: {value} is out of range")
    return [raw >> 16, raw & 0xFFFF] if reg.dtype == "u32" else [raw]


def raw_of(words: list[int]) -> int:
    raw = 0
    for word in words:
        raw = (raw << 16) | (word & 0xFFFF)
    return raw


def decode(reg: Reg, words: list[int]):
    raw = raw_of(words)
    if reg.scale == 1.0:
        return raw
    return round(raw * reg.scale, reg.decimals)


def format_words(reg: Reg, words: list[int]) -> str:
    """Human-readable value: '5500.0 W', '1 (Normal)', 'on, battery first, 21:01'."""
    if reg.dtype == "tou_start":
        d = decode_tou_start(words[0])
        return (f"{'on' if d['enabled'] else 'off'}, {PRIORITY_NAMES.get(d['priority'], '?')}, "
                f"{hhmm(d['hour'], d['minute'])}")
    if reg.dtype == "tou_end":
        return hhmm(*decode_tou_end(words[0]))
    value = decode(reg, words)
    if reg.enum is not None:
        return f"{value} ({reg.enum.get(value, 'undocumented')})"
    text = f"{value:.{reg.decimals}f}" if reg.decimals else str(value)
    return f"{text} {reg.unit}".strip()


def describe_range(kind: str, addr: int, words: list[int], limit: int = 6) -> str:
    """Decode a run of words for the traffic log; long batch reads are summarised."""
    parts: list[str] = []
    known = other = 0
    i = 0
    while i < len(words):
        owner = WORD_OWNER.get((kind, addr + i))
        if owner is None:
            other += 1
            if len(words) <= limit:
                parts.append(f"[{addr + i}] = {words[i]}")
            i += 1
            continue
        reg, idx = owner
        if idx == 0 and i + reg.count <= len(words):
            known += 1
            if known <= limit:
                parts.append(f"{reg.label} = {format_words(reg, words[i:i + reg.count])}")
            i += reg.count
        else:
            parts.append(f"{reg.label} [word {idx + 1} of {reg.count}] = 0x{words[i]:04X}")
            i += 1
    if known > limit:
        parts.append(f"and {known - limit} more")
    if other and len(words) > limit:
        parts.append(f"{other} unmapped")
    return "; ".join(parts)
