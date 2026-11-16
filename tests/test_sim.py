"""
Tests for the Growatt simulator. Standard library only:

    python3 -m unittest discover -s tests -v

The RTU tests talk to the simulator over a virtual serial port (a pty), with
the bundled RTU master, so they need no hardware.
"""

from __future__ import annotations

import json
import logging
import os
import struct
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from growatt_sim import registers as R                                   # noqa: E402
from growatt_sim import rtu                                              # noqa: E402
from growatt_sim.client import RtuError, RtuException, RtuMaster         # noqa: E402
from growatt_sim.model import Inverter, ModbusError, parse_scan          # noqa: E402
from growatt_sim.web import EvFollower, start_server                     # noqa: E402

logging.disable(logging.CRITICAL)      # the simulator's own event log is not test output


def at(hour: int, minute: int) -> float:
    """Epoch seconds of today at hour:minute, local time."""
    return datetime.now().replace(hour=hour, minute=minute, second=0, microsecond=0).timestamp()


class ModelCase(unittest.TestCase):
    def setUp(self) -> None:
        self.mono = 1000.0
        self.wall = at(12, 0)
        self.inv = Inverter(clock=lambda: self.mono, wall=lambda: self.wall)

    def run_for(self, seconds: float, step: float = 0.5) -> None:
        for _ in range(int(round(seconds / step))):
            self.mono += step
            self.inv.tick()

    def reg(self, key: str):
        reg = R.REG_BY_KEY[key]
        return R.decode(reg, self.inv.modbus_read(reg.kind, reg.addr, reg.count))

    def flows(self) -> dict:
        return self.inv.snapshot()["flows"]

    def night_charge_on(self, pct: int = 25) -> None:
        """What NightChargeManager._apply(True) writes."""
        self.inv.modbus_write(3047, [pct], fc=6)
        self.inv.modbus_write(3038, [R.encode_tou_start(True, R.PRIO_BATTERY, 21, 1), R.encode_tou_end(23, 59)], fc=16)
        self.inv.modbus_write(3040, [R.encode_tou_start(True, R.PRIO_BATTERY, 0, 0), R.encode_tou_end(6, 59)], fc=16)
        self.inv.modbus_write(3049, [1], fc=6)
        self.mono += 1.0


class RegisterMapTest(unittest.TestCase):
    def test_everything_growatt_reader_reads_is_modelled(self) -> None:
        single = [3005, 3009, 3034, 3041, 3043, 3171, 3178, 3180]
        batch = [0, 3, 4, 5, 7, 8, 9, 35, 37, 38, 42, 46, 53, 91, 93, 105, 106]
        for addr in single + batch:
            self.assertIn(("input", addr), R.WORD_OWNER, addr)

    def test_everything_night_charge_uses_is_modelled(self) -> None:
        for start in (3038, 3040, 3042, 3044, 3050, 3052):
            self.assertEqual(R.WORD_OWNER[("holding", start)][0].dtype, "tou_start")
            self.assertEqual(R.WORD_OWNER[("holding", start + 1)][0].dtype, "tou_end")
        for addr in (3047, 3048, 3049):
            self.assertIn(("holding", addr), R.WORD_OWNER)

    def test_tou_words(self) -> None:
        # values seen on the live inverter
        self.assertEqual(R.decode_tou_start(41507), {"enabled": True, "priority": 1, "hour": 2, "minute": 35})
        self.assertEqual(R.decode_tou_start(13569), {"enabled": False, "priority": 1, "hour": 21, "minute": 1})
        self.assertEqual(R.decode_tou_end(5947), (23, 59))
        self.assertEqual(R.encode_tou_start(True, 1, 21, 1), 0xB501)
        self.assertEqual(R.encode_tou_end(6, 59), 1595)
        self.assertEqual(R.parse_hhmm("06:59"), (6, 59))
        for bad in ("24:00", "12:60", "noon", "12"):
            with self.assertRaises(ValueError):
                R.parse_hhmm(bad)

    def test_encode_decode(self) -> None:
        self.assertEqual(R.encode(R.REG_BY_KEY["pv1_w"], 5500.0), [0, 55000])
        self.assertEqual(R.encode(R.REG_BY_KEY["pv1_w"], 7000.0), [1, 4464])
        self.assertEqual(R.decode(R.REG_BY_KEY["grid_v"], [2300]), 230.0)
        self.assertEqual(R.decode(R.REG_BY_KEY["fac"], [5002]), 50.02)


class BalanceTest(ModelCase):
    def test_night_runs_on_the_battery(self) -> None:
        f = self.flows()
        self.assertEqual((f["pv"], f["load"], f["discharge"], f["charge"], f["grid_net"]), (0, 400, 400, 0, 0))
        self.assertEqual(self.reg("bat_discharge_w"), 400.0)
        self.assertEqual(self.reg("soc"), 50)

    def test_surplus_charges_the_battery_then_exports(self) -> None:
        self.inv.set_plant(pv1_w=1000, pv2_w=4000, house_w=600)
        self.assertEqual(self.reg("bat_charge_w"), 4400.0)
        self.assertEqual(self.reg("grid_export_w"), 0.0)
        self.inv.set_plant(soc=100)
        self.assertEqual(self.reg("bat_charge_w"), 0.0)
        self.assertEqual(self.reg("grid_export_w"), 4400.0)
        self.assertEqual(self.reg("pac_w"), 5000.0)

    def test_empty_battery_imports(self) -> None:
        self.inv.set_plant(house_w=2500, soc=10)
        self.assertEqual(self.reg("bat_discharge_w"), 0.0)
        self.assertEqual(self.reg("grid_import_w"), 2500.0)

    def test_charge_power_register_limits_the_battery(self) -> None:
        self.inv.set_plant(pv2_w=7000, house_w=0)
        self.inv.set_register("h3047", value=50)
        self.assertEqual(self.reg("bat_charge_w"), 4800.0)       # measured on the real battery
        self.assertEqual(self.reg("grid_export_w"), 2200.0)

    def test_registers_are_consistent_with_each_other(self) -> None:
        self.inv.set_plant(pv1_w=1234, pv2_w=4321, house_w=900)
        batch = self.inv.modbus_read("input", 0, 125)
        self.assertEqual(batch[0], 1)
        self.assertEqual((batch[5] << 16) | batch[6], 12340)
        self.assertEqual((batch[9] << 16) | batch[10], 43210)
        self.assertEqual(self.inv.modbus_read("input", 3005, 2), batch[5:7])
        self.assertEqual(self.inv.modbus_read("input", 3009, 2), batch[9:11])
        self.assertEqual(self.inv.modbus_read("input", 3034, 1), [batch[46]])
        self.assertEqual((batch[3], batch[7]), (2400, 6000))
        self.assertEqual(batch[4], round(1234 / 240 * 10))
        self.assertEqual((batch[37], batch[38], batch[93]), (5000, 2300, 350))

    def test_surplus_as_the_controller_computes_it(self) -> None:
        self.inv.set_plant(pv1_w=1000, pv2_w=4000, house_w=600, soc=95)
        self.assertEqual(self.inv.snapshot()["controller"],
                         {"surplus_w": 4400, "reserve_w": 0, "available_w": 4400})
        self.inv.set_plant(soc=60)
        self.assertEqual(self.inv.snapshot()["controller"],
                         {"surplus_w": 4400, "reserve_w": 3000, "available_w": 1400})
        self.inv.set_plant(ev_w=3680)                            # the car's own load is not surplus lost
        self.assertEqual(self.flows()["load"], 4280)
        self.assertEqual(self.inv.snapshot()["controller"]["surplus_w"], 4400)

    def test_soc_and_energy_move_with_time_speed(self) -> None:
        self.inv.update_settings({"time_speed": 3600})
        self.inv.set_plant(pv2_w=2400, house_w=400)              # 2 kW into a 20 kWh battery
        self.run_for(2)                                          # = 2 h of simulated time
        self.assertAlmostEqual(self.inv.soc, 70.0, delta=0.6)
        self.assertEqual(self.reg("soc"), int(self.inv.soc))
        self.assertAlmostEqual(self.reg("e_today"), 0.8, delta=0.1)   # 400 W AC output for 2 h


class TouTest(ModelCase):
    def test_on_inside_the_window_charges_from_the_grid(self) -> None:
        self.wall = at(22, 30)
        self.night_charge_on(25)
        f = self.flows()
        self.assertTrue(f["grid_charge"])
        self.assertEqual((f["mode"], f["slot"], f["charge"]), (R.PRIO_BATTERY, 1, 2400.0))
        self.assertEqual(self.reg("grid_import_w"), 2800.0)      # 2.4 kW battery + 400 W house
        self.assertEqual(self.reg("bat_charge_w"), 2400.0)

    def test_on_outside_the_window_only_arms(self) -> None:
        self.wall = at(20, 5)
        self.night_charge_on(25)
        f = self.flows()
        self.assertFalse(f["grid_charge"])
        self.assertEqual((f["mode"], f["discharge"]), (R.PRIO_LOAD, 400.0))
        self.wall = at(21, 1)                                    # the slot opens
        self.assertTrue(self.flows()["grid_charge"])
        self.wall = at(0, 0)                                     # second slot, after midnight
        self.assertEqual(self.flows()["slot"], 2)
        self.wall = at(7, 0)                                     # window over
        self.assertFalse(self.flows()["grid_charge"])

    def test_slot_without_ac_charge_charges_from_pv_only(self) -> None:
        self.wall = at(22, 0)
        self.inv.set_slot(1, enabled=True)
        self.assertEqual(self.flows()["charge"], 0.0)            # battery first, but no PV at night
        self.assertEqual(self.reg("grid_import_w"), 400.0)       # and the battery is not discharged
        self.inv.set_plant(pv2_w=3000)
        self.assertEqual(self.flows()["charge"], 3000.0)

    def test_charging_stops_at_the_stop_soc(self) -> None:
        self.wall = at(22, 0)
        self.night_charge_on(100)
        self.inv.set_register("h3048", value=80)
        self.inv.set_plant(soc=79)
        self.assertTrue(self.flows()["grid_charge"])
        self.inv.set_plant(soc=80)
        self.assertFalse(self.flows()["grid_charge"])

    def test_a_slot_cannot_cross_midnight(self) -> None:
        self.wall = at(23, 0)
        self.inv.set_slot(3, enabled=True, priority=1, start="21:00", end="06:00")
        slot = self.inv.snapshot()["slots"][2]
        self.assertTrue(slot["crosses_midnight"])
        self.assertFalse(slot["active"])
        self.assertEqual(self.flows()["mode"], R.PRIO_LOAD)

    def test_grid_first_discharges(self) -> None:
        self.wall = at(13, 0)
        self.inv.set_slot(6, enabled=True)                       # 12:00-14:00, grid first
        self.assertEqual(self.flows()["discharge"], 10000.0)
        self.assertEqual(self.reg("grid_export_w"), 9600.0)

    def test_inverter_clock_can_be_set(self) -> None:
        self.night_charge_on()
        self.assertFalse(self.flows()["grid_charge"])            # it is noon
        self.inv.set_clock("03:15")
        self.assertEqual(self.inv.inverter_time().strftime("%H:%M"), "03:15")
        self.assertTrue(self.flows()["grid_charge"])
        self.inv.set_clock(None)
        self.assertFalse(self.flows()["grid_charge"])
        with self.assertRaises(ValueError):
            self.inv.set_clock("25:00")


class WriteTest(ModelCase):
    def test_a_write_applies_after_the_delay(self) -> None:
        self.inv.modbus_write(3047, [25], fc=6)
        self.assertEqual(self.inv.modbus_read("holding", 3047, 1), [100])     # acknowledged, old value
        self.mono += 0.3
        self.assertEqual(self.inv.modbus_read("holding", 3047, 1), [100])
        self.mono += 0.15
        self.assertEqual(self.inv.modbus_read("holding", 3047, 1), [25])

    def test_fc06_to_a_tou_start_word_is_ignored(self) -> None:
        word = R.encode_tou_start(True, 1, 21, 1)
        self.inv.modbus_write(3042, [word], fc=6)
        self.mono += 1
        self.assertEqual(self.inv.modbus_read("holding", 3042, 1), [0x2101])
        self.inv.modbus_write(3043, [R.encode_tou_end(5, 0)], fc=6)           # an end word takes FC06
        self.inv.modbus_write(3042, [word, R.encode_tou_end(6, 59)], fc=16)   # the pair by FC16 works
        self.mono += 1
        self.assertEqual(self.inv.modbus_read("holding", 3042, 2), [word, 1595])
        self.inv.update_settings({"fc06_ignored_on_tou_start": False})
        self.inv.modbus_write(3044, [word], fc=6)
        self.mono += 1
        self.assertEqual(self.inv.modbus_read("holding", 3044, 1), [word])

    def test_invalid_values_are_rejected(self) -> None:
        for addr, values in ((3047, [101]), (3049, [2]), (3038, [0x1800 | 0x8000]), (3039, [0x003C]),
                             (3038, [0xB501, 0x1900])):
            with self.assertRaises(ModbusError) as ctx:
                self.inv.modbus_write(addr, values, fc=16)
            self.assertEqual(ctx.exception.code, 3, (addr, values))
        self.mono += 1
        self.assertEqual(self.inv.modbus_read("holding", 3038, 2), [0x3501, 0x173B])   # nothing applied

    def test_unmapped_addresses(self) -> None:
        self.assertEqual(self.inv.modbus_read("input", 1000, 2), [0, 0])
        self.inv.modbus_write(3046, [7], fc=6)
        self.assertEqual(self.inv.modbus_read("holding", 3046, 1), [7])
        self.inv.update_settings({"strict_addresses": True})
        for call in (lambda: self.inv.modbus_read("input", 1000, 1),
                     lambda: self.inv.modbus_write(3060, [1], fc=6)):
            with self.assertRaises(ModbusError) as ctx:
                call()
            self.assertEqual(ctx.exception.code, 2)

    def test_read_length(self) -> None:
        self.assertEqual(len(self.inv.modbus_read("input", 0, 125)), 125)
        self.assertEqual(len(self.inv.modbus_read("holding", 3038, 12)), 12)   # what night_charge reads
        for count in (0, 126):
            with self.assertRaises(ModbusError) as ctx:
                self.inv.modbus_read("input", 0, count)
            self.assertEqual(ctx.exception.code, 3)


class UiControlTest(ModelCase):
    def test_pin_and_release(self) -> None:
        self.inv.set_register("i3043", value=4000)
        self.inv.set_plant(pv2_w=1000)
        self.assertEqual(self.reg("grid_export_w"), 4000.0)
        self.assertTrue(next(r for r in self.inv.snapshot()["registers"] if r["rid"] == "i3043")["pinned"])
        self.inv.release("i3043")
        self.assertEqual(self.reg("grid_export_w"), 0.0)

    def test_parameters(self) -> None:
        self.inv.set_register("i5", value=1500)                  # MPPT1 power is the PV input
        self.assertEqual(self.inv.plant["pv1_w"], 1500.0)
        self.assertEqual(self.reg("pv1_w"), 1500.0)
        self.inv.set_register("i3034", value=241.5)              # L3 at 3034 and at 46
        self.assertEqual(self.inv.modbus_read("input", 46, 1), [2415])
        self.inv.set_register("i0", value=3)
        self.inv.set_register("i105", value=402)
        batch = self.inv.modbus_read("input", 0, 125)
        self.assertEqual((batch[0], batch[105]), (3, 402))
        self.inv.set_register("h3049", value=1)                  # from the UI: at once
        self.assertEqual(self.inv.modbus_read("holding", 3049, 1), [1])
        self.inv.set_register("i200", raw=[77])                  # any other address
        self.assertEqual(self.inv.modbus_read("input", 200, 1), [77])
        for rid, kw in (("i6", {"value": 1}), ("x1", {"value": 1}), ("h3047", {"value": 150})):
            with self.assertRaises((ValueError, ModbusError)):
                self.inv.set_register(rid, **kw)

    def test_slot_editor(self) -> None:
        self.inv.set_slot(4, enabled=True, priority=2, start="10:15", end="11:45")
        self.assertEqual(self.inv.modbus_read("holding", 3044, 2),
                         [R.encode_tou_start(True, 2, 10, 15), R.encode_tou_end(11, 45)])
        self.inv.set_slot(4, enabled=False)                      # the rest stays
        self.assertEqual(self.inv.modbus_read("holding", 3044, 2),
                         [R.encode_tou_start(False, 2, 10, 15), R.encode_tou_end(11, 45)])
        for bad in ({"n": 7}, {"n": 1, "priority": 3}, {"n": 1, "start": "9"}):
            with self.assertRaises(ValueError):
                self.inv.set_slot(**bad)

    def test_settings_and_link_are_validated(self) -> None:
        self.inv.update_settings({"battery_kwh": "10", "ev_follow": "true", "ev_url": " http://pi:5021 "})
        self.assertEqual(self.inv.settings["battery_kwh"], 10.0)
        self.assertIs(self.inv.settings["ev_follow"], True)
        self.assertEqual(self.inv.settings["ev_url"], "http://pi:5021")
        for bad in ({"battery_kwh": 0}, {"nope": 1}, {"fault_mode": "weird"}, {"drop_pct": 101}):
            with self.assertRaises(ValueError):
                self.inv.update_settings(bad)
        gen = self.inv.link_gen
        self.inv.set_link(port="/dev/ttyUSB1", baud=19200, parity="e", unit=5, de="gpio", de_gpio=23, echo="yes")
        self.assertEqual(self.inv.link_gen, gen + 1)
        link = self.inv.link
        self.assertEqual((link["baud"], link["parity"], link["unit"], link["de"], link["echo"]),
                         (19200, "E", 5, "gpio", True))
        self.inv.set_link(baud=19200)                            # no change, no reopen
        self.assertEqual(self.inv.link_gen, gen + 1)
        for bad in ({"baud": 12345}, {"parity": "X"}, {"unit": 0}, {"de": "magic"}, {"port": ""}, {"colour": 1}):
            with self.assertRaises(ValueError):
                self.inv.set_link(**bad)

    def test_reset(self) -> None:
        self.inv.set_plant(pv1_w=900, soc=99)
        self.inv.set_register("i3043", value=1)
        self.inv.set_slot(1, enabled=True)
        self.inv.set_clock("03:00")
        self.inv.reset()
        snap = self.inv.snapshot()
        self.assertEqual((snap["plant"]["pv1_w"], snap["soc"], snap["clock_offset_s"]), (0.0, 50.0, 0))
        self.assertFalse(any(r["pinned"] for r in snap["registers"]))
        self.assertFalse(snap["slots"][0]["enabled"])
        json.dumps(snap)

    def test_register_scan_as_baseline(self) -> None:
        formats = (
            {"h3038": 41507, "h3047": 50, "h3049": 1, "h30": 1, "i3000": 1, "i3171": 27},
            {"holding 3000-3124": {"3038": 41507, "3047": 50, "3049": 1}, "holding 0-124": {"30": 1},
             "input 3000-3124": {"3000": 1, "3171": 27}},
            {"HR:3038": 41507, "HR:3047": 50, "HR:3049": 1, "HR:30": 1, "IN:3000": 1, "IN:3171": 27},
        )
        for data in formats:
            scan = parse_scan(data)
            self.assertEqual(scan, {"input": {3000: 1, 3171: 27},
                                    "holding": {3038: 41507, 3047: 50, 3049: 1, 30: 1}})
            inv = Inverter(baseline=scan)
            self.assertEqual(inv.modbus_read("holding", 3038, 1), [41507])   # the settings of the scan
            self.assertEqual(inv.modbus_read("holding", 30, 1), [1])         # an unmodelled register
            self.assertEqual(inv.modbus_read("input", 3000, 1), [1])
            self.assertEqual(inv.modbus_read("input", 3171, 1), [50])        # modelled: the simulation wins
            inv.reset()
            self.assertEqual(inv.modbus_read("holding", 3047, 1), [50])
        with self.assertRaises(ValueError):
            parse_scan({"date": "2026-03-03"})


# ── RTU framing ─────────────────────────────────────────────────────────────

class FramingTest(unittest.TestCase):
    def test_crc(self) -> None:
        self.assertEqual(rtu.add_crc(bytes.fromhex("010400000002")), bytes.fromhex("01040000000271cb"))
        self.assertTrue(rtu.crc_ok(bytes.fromhex("01040000000271cb")))
        self.assertFalse(rtu.crc_ok(bytes.fromhex("01040000000271cc")))
        self.assertFalse(rtu.crc_ok(b"\x01\x04"))

    def test_request_length(self) -> None:
        self.assertIsNone(rtu.request_length(b"\x01"))
        self.assertEqual(rtu.request_length(bytes.fromhex("0104")), 8)
        self.assertEqual(rtu.request_length(bytes.fromhex("0106")), 8)
        self.assertIsNone(rtu.request_length(bytes.fromhex("01100bde0002")))
        self.assertEqual(rtu.request_length(bytes.fromhex("01100bde000204")), 13)
        self.assertEqual(rtu.request_length(bytes.fromhex("012b")), -1)


class FakePort:
    """Stands in for SerialPort: records what the server sends."""
    fd = -1

    def __init__(self) -> None:
        self.sent: list[bytes] = []

    def write(self, data: bytes) -> float:
        self.sent.append(bytes(data))
        return len(data) * 0.001

    def close(self) -> None:
        pass


class ServerLogicTest(unittest.TestCase):
    """The frame parser, driven byte by byte without a port."""

    def setUp(self) -> None:
        self.inv = Inverter(link={"turnaround_ms": 0})
        self.server = rtu.RtuServer(self.inv)
        self.server._port = FakePort()
        self.read_soc = rtu.add_crc(struct.pack(">BBHH", 1, 4, 3171, 1))

    def test_frame_arriving_in_pieces(self) -> None:
        for byte in self.read_soc:
            self.server._feed(bytes([byte]))
        self.assertEqual(self.server._port.sent, [rtu.add_crc(bytes([1, 4, 2, 0, 50]))])

    def test_two_frames_in_one_chunk(self) -> None:
        self.server._feed(self.read_soc + self.read_soc)
        self.assertEqual(len(self.server._port.sent), 2)

    def test_bad_crc_then_recovery_after_silence(self) -> None:
        broken = self.read_soc[:-1] + bytes([self.read_soc[-1] ^ 1])
        self.server._feed(broken)
        self.server._feed(self.read_soc)                         # still inside the same burst: ignored
        self.assertEqual(self.server._port.sent, [])
        self.assertEqual(self.inv.counters["crc_errors"], 1)
        self.server._silence()
        self.server._feed(self.read_soc)
        self.assertEqual(len(self.server._port.sent), 1)

    def test_noise_is_dropped_at_silence(self) -> None:
        self.server._feed(b"\x01\x04\x0c")
        self.server._silence()
        self.assertEqual(self.inv.counters["crc_errors"], 1)
        self.server._feed(self.read_soc)
        self.assertEqual(len(self.server._port.sent), 1)

    def test_unknown_function_code_is_answered_at_silence(self) -> None:
        self.server._feed(rtu.add_crc(bytes([1, 0x2B, 0x0E, 1, 0])))
        self.assertEqual(self.server._port.sent, [])
        self.server._silence()
        self.assertEqual(self.server._port.sent, [rtu.add_crc(bytes([1, 0xAB, 1]))])

    def test_other_slave_address_and_broadcast(self) -> None:
        self.server._feed(rtu.add_crc(struct.pack(">BBHH", 2, 4, 3171, 1)))
        self.assertEqual(self.server._port.sent, [])
        self.assertEqual(self.inv.counters["other_unit"], 1)
        self.inv.update_settings({"write_apply_delay_ms": 0})
        self.server._feed(rtu.add_crc(struct.pack(">BBHH", 0, 6, 3047, 30)))     # broadcast: applied, no reply
        self.assertEqual(self.server._port.sent, [])
        self.assertEqual(self.inv.holding[3047], 30)

    def test_echoing_transceiver(self) -> None:
        write = rtu.add_crc(struct.pack(">BBHH", 1, 6, 3047, 25))    # an FC06 reply equals its request
        self.inv.set_link(echo=True)
        self.server._feed(write)
        reply = self.server._port.sent[0]
        self.assertEqual(reply, write)
        self.server._feed(reply[:3])                                 # the echo comes back in pieces ...
        self.server._feed(reply[3:] + write)                         # ... followed by the same write again
        self.assertEqual(len(self.server._port.sent), 2)
        self.assertEqual(self.inv.stats["h3047"]["w"], 2)

    def test_fault_injection(self) -> None:
        self.inv.update_settings({"fault_mode": "silent"})
        self.server._feed(self.read_soc)
        self.assertEqual(self.server._port.sent, [])
        self.inv.update_settings({"fault_mode": "exception", "fault_exception_code": 6})
        self.server._feed(self.read_soc)
        self.assertEqual(self.server._port.sent[-1], rtu.add_crc(bytes([1, 0x84, 6])))
        self.inv.update_settings({"fault_mode": "bad_crc"})
        self.server._feed(self.read_soc)
        self.assertFalse(rtu.crc_ok(self.server._port.sent[-1]))
        self.inv.update_settings({"fault_mode": "none", "drop_pct": 100})
        before = len(self.server._port.sent)
        self.server._feed(self.read_soc)
        self.assertEqual(len(self.server._port.sent), before)
        self.assertEqual(self.inv.counters["dropped"], 1)


class DirectionTest(unittest.TestCase):
    def test_gpio_request_layout(self) -> None:
        """The ioctl buffers match <linux/gpio.h> (sizes and offsets checked on the Pi)."""
        calls = []

        def fake_ioctl(fd, request, arg):
            calls.append((fd, request, bytes(arg)))
            if request == rtu.GpioDirection._GET_LINE:
                struct.pack_into("<i", arg, 588, 77)                 # the kernel hands back the line fd
            return 0

        with mock.patch.object(rtu.os, "open", return_value=11), mock.patch.object(rtu.os, "close"), \
                mock.patch.object(rtu.fcntl, "ioctl", side_effect=fake_ioctl):
            line = rtu.GpioDirection(18, "/dev/gpiochip0", active_high=True)
            line.send()
            line.receive()
        fd, request, buf = calls[0]
        self.assertEqual((fd, request, len(buf)), (11, 0xC250B407, 592))
        self.assertEqual(struct.unpack_from("<I", buf, 0)[0], 18)                 # offsets[0]
        self.assertEqual(buf[256:267], b"growatt-sim")                            # consumer
        self.assertEqual(struct.unpack_from("<QI", buf, 288), (1 << 3, 1))        # OUTPUT, one attribute
        self.assertEqual(struct.unpack_from("<IIQQ", buf, 320), (2, 0, 0, 1))     # starts low = receive
        self.assertEqual(struct.unpack_from("<I", buf, 560)[0], 1)                # num_lines
        self.assertEqual(calls[1], (77, 0xC010B40F, struct.pack("<QQ", 1, 1)))    # send
        self.assertEqual(calls[2], (77, 0xC010B40F, struct.pack("<QQ", 0, 1)))    # receive

    def test_transceiver_is_held_for_the_length_of_the_frame(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            port = rtu.SerialPort({"port": "pty", "pty_link": os.path.join(tmp, "tty"), "baud": 9600,
                                   "parity": "N", "stopbits": 1})
            self.addCleanup(port.close)
            events = []

            class Recorder:
                active = True

                def send(self):
                    events.append(("send", time.perf_counter()))

                def receive(self):
                    events.append(("receive", time.perf_counter()))

                def close(self):
                    pass

            port.direction = Recorder()
            frame = bytes(20)
            duration = port.write(frame)
            self.assertAlmostEqual(duration, 20 * 10 / 9600, places=6)
            self.assertEqual([name for name, _ in events], ["send", "receive"])
            held = events[1][1] - events[0][1]
            self.assertGreaterEqual(held, duration)
            self.assertLess(held, duration + 0.01)
            self.assertEqual(os.read(port._extra_fd, 64), frame)


# ── Over a virtual serial port, with the bundled RTU master ─────────────────

def wait_until(predicate, timeout: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class RtuOverPtyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "ttyGROWATT")
        self.inv = Inverter(link={"port": "pty", "pty_link": self.path, "turnaround_ms": 2})
        self.server = rtu.RtuServer(self.inv)
        self.server.start()
        self.addCleanup(self.server.stop)
        self.assertTrue(wait_until(lambda: self.inv.link_status["state"] == "open"))
        self.master = RtuMaster(self.path, timeout=0.5)
        self.addCleanup(self.master.close)

    def test_what_the_reader_and_night_charge_do(self) -> None:
        self.inv.set_plant(pv1_w=1700, pv2_w=5600, house_w=600, soc=95)
        self.assertEqual(self.master.read("input", 3005, 2), [0, 17000])
        self.assertEqual(self.master.read("input", 3171, 1), [95])
        self.assertEqual(self.master.read("input", 3180, 2), [1, 1464])          # 6700 W into the battery
        batch = self.master.read("input", 0, 125)
        self.assertEqual((len(batch), batch[0], batch[9], batch[10]), (125, 1, 0, 56000))
        self.assertEqual(self.master.read("holding", 3038, 12),
                         [0x3501, 0x173B, 0x2000, 0x063B, 0x2101, 0x063B, 0x2700, 0x081E, 0, 100, 100, 0])
        slot = [R.encode_tou_start(True, 1, 21, 1), R.encode_tou_end(23, 59)]
        self.master.write_multiple(3038, slot)
        self.assertEqual(self.master.read("holding", 3038, 2), [0x3501, 0x173B])  # not applied yet
        self.assertTrue(wait_until(lambda: self.master.read("holding", 3038, 2) == slot, timeout=2))
        self.master.write_single(3047, 25)
        self.assertTrue(wait_until(lambda: self.master.read("holding", 3047, 1) == [25], timeout=2))
        kinds = [e["kind"] for e in self.inv.log_since(0) if "fc" in e]
        self.assertEqual(kinds.count("write"), 2)
        entry = next(e for e in self.inv.log_since(0) if e["kind"] == "write")
        self.assertEqual(entry["req"], rtu.hexdump(rtu.add_crc(struct.pack(">BBHHBHH", 1, 16, 3038, 2, 4, *slot))))
        self.assertIn("TOU slot 1 start = on, battery first, 21:01", entry["text"])

    def test_exceptions(self) -> None:
        for call, code in ((lambda: self.master.read("input", 0, 126), 3),
                           (lambda: self.master.write_single(3047, 101), 3),
                           (lambda: self.master.request(bytes([5, 0, 1, 0xFF, 0])), 1)):
            with self.assertRaises(RtuException) as ctx:
                call()
            self.assertEqual(ctx.exception.code, code)
        self.inv.update_settings({"strict_addresses": True})
        with self.assertRaises(RtuException) as ctx:
            self.master.read("holding", 10, 1)
        self.assertEqual(ctx.exception.code, 2)
        self.assertEqual(self.master.read("input", 3171, 1), [50])                # the link survives

    def test_wrong_slave_address_times_out(self) -> None:
        other = RtuMaster(self.path, unit=2, timeout=0.3)
        self.addCleanup(other.close)
        with self.assertRaises(RtuError):
            other.read("input", 3171, 1)
        self.assertEqual(self.inv.counters["other_unit"], 1)
        self.inv.set_link(unit=2)
        self.assertTrue(wait_until(lambda: "address 2" in self.inv.link_status["detail"]))
        other2 = RtuMaster(self.path, unit=2, timeout=0.5)       # the virtual port was reopened
        self.addCleanup(other2.close)
        self.assertEqual(other2.read("input", 3171, 1), [50])

    def test_a_corrupted_request_is_not_answered(self) -> None:
        good = rtu.add_crc(struct.pack(">BBHH", 1, 4, 3171, 1))
        os.write(self.master.fd, good[:-1] + bytes([good[-1] ^ 0xFF]))
        self.assertTrue(wait_until(lambda: self.inv.counters["crc_errors"] == 1))
        time.sleep(rtu.SILENCE_S + 0.1)
        self.assertEqual(self.master.read("input", 3171, 1), [50])

    def test_bad_crc_and_no_reply_faults_reach_the_master(self) -> None:
        self.inv.update_settings({"fault_mode": "bad_crc"})
        with self.assertRaises(RtuError) as ctx:
            self.master.read("input", 3171, 1)
        self.assertIn("bad CRC", str(ctx.exception))
        self.inv.update_settings({"fault_mode": "silent"})
        with self.assertRaises(RtuError) as ctx:
            self.master.read("input", 3171, 1)
        self.assertIn("no reply", str(ctx.exception))

    def test_a_missing_port_is_reported_and_retried(self) -> None:
        self.inv.set_link(port=os.path.join(self.tmp.name, "nope"))
        self.assertTrue(wait_until(lambda: self.inv.link_status["state"] == "error"))
        self.assertIn("cannot open", self.inv.link_status["detail"])
        self.inv.set_link(port="pty")
        self.assertTrue(wait_until(lambda: self.inv.link_status["state"] == "open"))


# ── HTTP ────────────────────────────────────────────────────────────────────

class FakeWallbox(BaseHTTPRequestHandler):
    watts = 3680

    def do_GET(self) -> None:
        body = json.dumps({"registers": [{"addr": 0, "value": 4}, {"addr": 12, "value": self.watts}]}).encode()
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args) -> None:
        pass


class HttpTest(unittest.TestCase):
    def setUp(self) -> None:
        self.inv = Inverter(link={"port": "pty"})
        self.ui = start_server("127.0.0.1", 0, self.inv)
        self.url = f"http://127.0.0.1:{self.ui.server_address[1]}"
        self.addCleanup(self.ui.server_close)
        self.addCleanup(self.ui.shutdown)
        self.addCleanup(self.ui.stopping.set)

    def get(self, path: str):
        with urllib.request.urlopen(self.url + path, timeout=3) as r:
            return json.loads(r.read())

    def post(self, path: str, body=None, raw: bytes | None = None):
        data = raw if raw is not None else json.dumps(body or {}).encode()
        with urllib.request.urlopen(urllib.request.Request(self.url + path, data=data, method="POST"), timeout=3) as r:
            return json.loads(r.read())

    def test_page_and_meta(self) -> None:
        with urllib.request.urlopen(self.url + "/", timeout=3) as r:
            self.assertIn(b"Growatt", r.read())
        meta = self.get("/api/meta")
        self.assertEqual(len(meta["registers"]), len(R.REGISTERS))
        self.assertEqual(len(meta["presets"]), 4)
        self.assertTrue(any(s["key"] == "write_apply_delay_ms" for s in meta["settings"]))

    def test_control_api(self) -> None:
        snap = self.post("/api/plant", {"pv1_w": 900, "pv2_w": 3300, "house_w": 500, "soc": 91})
        self.assertEqual((snap["flows"]["pv"], snap["soc"]), (4200, 91))
        self.post("/api/register", {"rid": "i3043", "value": 1234})
        self.assertTrue(next(r for r in self.get("/api/state")["registers"] if r["rid"] == "i3043")["pinned"])
        self.post("/api/release", {"rid": "i3043"})
        snap = self.post("/api/slot", {"n": 1, "enabled": True, "start": "20:00"})
        self.assertEqual((snap["slots"][0]["enabled"], snap["slots"][0]["start"]), (True, "20:00"))
        self.assertEqual(self.post("/api/clock", {"time": "20:30"})["inverter_time"][11:16], "20:30")
        self.assertEqual(self.post("/api/clock", {})["clock_offset_s"], 0)
        self.assertEqual(self.post("/api/settings", {"time_speed": 60})["settings"]["time_speed"], 60)
        self.assertEqual(self.post("/api/link", {"unit": 3})["link"]["unit"], 3)
        self.assertEqual(self.post("/api/reset")["plant"]["pv1_w"], 0)
        self.assertTrue(self.get("/api/log?since=0"))
        self.assertEqual(self.get("/api/history"), [])

    def test_bad_requests_get_400(self) -> None:
        for path, raw in (("/api/plant", b'{"wind_w": 5}'), ("/api/plant", b'{"soc": 140}'),
                          ("/api/register", b'{"rid": "h3047", "value": 500}'), ("/api/register", b"not json"),
                          ("/api/slot", b'{"n": 9}'), ("/api/clock", b'{"time": "noon"}'),
                          ("/api/settings", b'{"time_speed": -1}'), ("/api/link", b'{"baud": 1}')):
            with self.assertRaises(urllib.error.HTTPError) as ctx:
                self.post(path, raw=raw)
            self.assertEqual(ctx.exception.code, 400, path)
            self.assertIn("error", json.loads(ctx.exception.read()))
            ctx.exception.close()
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self.get("/nope")
        self.assertEqual(ctx.exception.code, 404)
        ctx.exception.close()

    def test_event_stream(self) -> None:
        self.inv.set_plant(pv1_w=500)
        self.inv.event("hello")
        with urllib.request.urlopen(self.url + "/api/events", timeout=3) as r:
            self.assertEqual(r.headers["Content-Type"], "text/event-stream")
            lines = [r.readline().decode().strip() for _ in range(7)]
        self.assertTrue(lines[0].startswith("id: "))
        self.assertEqual(lines[1], "event: log")
        self.assertIn("hello", lines[2])
        self.assertEqual(lines[4], "event: state")
        self.assertEqual(json.loads(lines[5][len("data: "):])["plant"]["pv1_w"], 500)

    def test_growatt_modbus_mock_rest_compat(self) -> None:
        status = self.post("/set", {"pv1_w": 5000, "grid_export_w": 4000, "bat_soc": 80})
        self.assertEqual((status["pv1_w"], status["grid_export_w"], status["bat_soc"]), (5000.0, 4000.0, 80))
        sunny = {"u16": {"3171": 85, "3034": 2300},
                 "u32_scaled": {"3005": {"value": 5500.0, "scale": 0.1}, "3009": {"value": 1200.0, "scale": 0.1},
                                "3041": {"value": 0.0, "scale": 0.1}, "3043": {"value": 4800.0, "scale": 0.1},
                                "3178": {"value": 0.0, "scale": 0.1}, "3180": {"value": 800.0, "scale": 0.1}}}
        self.assertEqual(self.post("/scenario", sunny),
                         {"pv1_w": 5500.0, "pv2_w": 1200.0, "grid_import_w": 0.0, "grid_export_w": 4800.0,
                          "grid_v": 230.0, "bat_soc": 85, "bat_discharge_w": 0.0, "bat_charge_w": 800.0})
        self.assertEqual(self.post("/reset")["pv1_w"], 0.0)
        self.assertEqual(self.get("/status")["bat_soc"], 50)

    def test_ev_load_follows_the_wallbox_simulator(self) -> None:
        wallbox = HTTPServer(("127.0.0.1", 0), FakeWallbox)
        threading.Thread(target=wallbox.serve_forever, daemon=True).start()
        self.addCleanup(wallbox.server_close)
        self.addCleanup(wallbox.shutdown)
        follower = EvFollower(self.inv)
        self.inv.set_plant(pv2_w=6000, house_w=500, soc=100)
        self.inv.update_settings({"ev_follow": True, "ev_url": f"http://127.0.0.1:{wallbox.server_address[1]}"})
        follower.poll()
        snap = self.inv.snapshot()
        self.assertEqual((snap["flows"]["ev"], snap["flows"]["export"]), (3680.0, 1820.0))
        self.assertEqual(snap["controller"]["surplus_w"], 5500)      # the controller adds its EV load back
        self.inv.update_settings({"ev_url": "http://127.0.0.1:1"})
        follower.poll()
        self.assertIs(self.inv.ev_follow["ok"], False)
        self.assertEqual(self.inv.snapshot()["flows"]["ev"], 0.0)
        self.inv.update_settings({"ev_url": "file:///etc/hostname"})
        follower.poll()
        self.assertIn("http://", self.inv.ev_follow["error"])


if __name__ == "__main__":
    unittest.main()
