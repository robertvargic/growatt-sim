# Growatt simulator

A stand-in for the Growatt inverter, for testing the solar charger against real
RS-485 hardware or with no hardware at all. It gives you:

- a **Modbus RTU slave** on a serial port, with every register the controller
  reads (`growatt_reader.py`) and writes (`night_charge.py`),
- a small **plant model**: PV, house and EV load, battery and grid stay
  consistent with each other, and the TOU slots and grid charge really act,
- a **web UI** to set PV and load, edit the battery settings, inject faults and
  watch every request of the controller as it happens.

It needs Python 3.10+ and nothing else: no venv, no pip, no pymodbus, no pyserial.
`em2go-sim` next to it does the same for the wallbox.

## Start

```bash
cd ~/growatt-sim
./run.sh --port /dev/serial0     # RS-485 transceiver on the GPIO UART
./run.sh --port /dev/ttyUSB1     # a second USB RS-485 adapter
./run.sh --port pty              # no hardware: virtual port at /tmp/ttyGROWATT
```

Then open **http://\<the Pi\>:5031/**. Port, baud rate, slave address and
direction control can all be changed in the UI (Serial link) while it runs.
Defaults: 9600 8N1, slave address 1, as the real inverter.

## Three ways to connect

| Setup | Simulator port | Controller reads | Tests |
|-------|----------------|------------------|-------|
| Transceiver on the pins, wired to a USB adapter | `/dev/serial0` | `/dev/ttyUSB0` | the real electrical link, both adapters |
| Two USB adapters wired to each other | `/dev/ttyUSB1` | `/dev/ttyUSB0` | the real link; simplest wiring |
| Virtual port | `pty` | `/tmp/ttyGROWATT` | everything but the wire |

The first two can be on one Pi (both adapters plugged into the spare Pi) or on
two machines.

## The bench: transceiver on the pins, USB adapter for the controller

**1. Prepare the Pi.** The UART on GPIO14/15 must be on and not used as a login
console: `sudo raspi-config` → Interface Options → Serial Port → login shell
**No**, hardware **Yes**. On a Pi 3, 4 or Zero W also add `dtoverlay=disable-bt`
to `/boot/firmware/config.txt`, so the pins get the full UART. Your user must be
in the `dialout` group (and `gpio` for direction control). raspberry3 is set up
like this and is a working reference; `/dev/serial0` points at `/dev/ttyAMA0`
there.

**2. Wire the transceiver.**

| Pi header | Transceiver |
|-----------|-------------|
| Pin 8, GPIO14 (TXD) | DI (or TXD) |
| Pin 10, GPIO15 (RXD) | RO (or RXD) |
| Pin 1, 3V3 | VCC |
| Pin 6, GND | GND |
| Pin 12, GPIO18 | DE and /RE tied together, only if the board has these pins |

- A board with only VCC, GND, TXD and RXD switches direction by itself. Leave
  direction control on *Automatic*.
- A MAX485-style board with DE and /RE needs `--de gpio --de-gpio 18` (or the
  same in the UI).
- The Pi's pins are 3.3 V. A MAX485 powered from 5 V drives RO at 5 V, which the
  Pi must not get: use a 3.3 V transceiver (MAX3485, SP3485) or a divider on RO.

**3. Wire the bus.** A to A and B to B on the USB adapter the controller reads.
Some adapters label them the other way round: if nothing comes through, swap
them. On one Pi the two adapters already share ground; between two machines
connect GND as well. Short bench wires need no termination.

**4. Check the link** from the controller's side, before starting the controller:

```bash
python3 -m growatt_sim.client --port /dev/ttyUSB0 test    # 50 reads, reports losses
python3 -m growatt_sim.client --port /dev/ttyUSB0 dump    # every register, decoded
```

| What you see | Likely cause |
|--------------|--------------|
| `test` gets no reply and the UI shows no traffic | A and B swapped, TXD and RXD swapped, wrong port, or the serial console still owns the UART |
| The UI shows the requests, the master gets nothing | The transceiver never switches to send: set direction control, or flip "DE is active low" |
| "Bad frame" errors in the UI | Baud rate or parity differ, or two masters share the line |
| A bad frame after every reply | The transceiver echoes what it sends (/RE tied low): tick the echo box |
| Some requests are lost | Raise "Reply after" to 20–30 ms; with DE on a GPIO see the note under Limits |

**5. Run the controller** against both simulators, on the Pi with the USB adapter:

```bash
cd ~/em2go-sim && ./run.sh &
cd ~/solar_charger
GROWATT_PORT=/dev/ttyUSB0 PC_IP=127.0.0.1 ENV=integration venv/bin/python main.py
```

Inverter, wallbox and Shelly are then all simulated. Nothing real is read or
written, including by the night-charge manager. Tick *Take the EV load from
em2go-sim* in the UI and the loop is closed: when the car charges, grid export
drops, as on the real plant.

To set up the spare Pi, copy the three folders and build the controller's venv
there (it does not copy between machines). Use the same user name: the config
has absolute paths under `/home/robert/solar_charger`. The `*.json` exclude
keeps the BigQuery key and the night-charge log on the live Pi.

```bash
rsync -a ~/growatt-sim ~/em2go-sim spare-pi:
rsync -a --exclude venv --exclude '*.json' ~/solar_charger spare-pi:
ssh spare-pi 'cd solar_charger && python3 -m venv venv && venv/bin/pip install -r requirements.txt'
```

## Without hardware

```bash
./run.sh --port pty
cd ~/solar_charger
GROWATT_PORT=/tmp/ttyGROWATT PC_IP=127.0.0.1 ENV=integration venv/bin/python main.py
```

The controller opens `/tmp/ttyGROWATT` like a serial port. On raspberry3 this
needs the live `solar_charger` service stopped first (it owns port 8099), and
stopping it parks the real wallbox as every stop does.

## The UI

| Panel | What it is for |
|-------|----------------|
| Header | Whether the serial port is open, how many requests came in, bad frames |
| Inverter | PV, load, battery and grid as the master reads them; battery mode; grid charging; the surplus the controller will compute |
| Plant | PV per roof, house load, EV load, SoC and grid voltage, with presets |
| Battery settings | The six TOU slots, charge power, stop SoC and grid charge, editable as in the ShinePhone app; the inverter's clock |
| Faults | Inverter status, fault and warning code; no reply, exception reply, bad CRC, lost requests |
| Serial link | Port, baud rate, parity, slave address, direction control, reply delay |
| Behaviour | Battery size and power, speed of SoC and energy, the inverter's write quirks |
| Last 10 minutes | PV and load; grid and battery |
| Registers | Every modelled register, live. A row flashes blue when the master reads it and orange when it writes it |
| Traffic | Every request and reply, decoded (and in hex with CRC on request), mixed with what the inverter did |

*Surplus for the EV* is computed from the registers exactly as
`power_math.available_for_ev_w()` does with the default settings (3 kW held back
for the battery below 90 % SoC), so you can dial in a test case before the
controller's next tick.

Setting a value in the register table does one of three things: a **simulated**
register (grid, battery, AC output, MPPT voltage and current) is *pinned* until
you release it; an **input** (PV power, SoC, voltages, status, codes, energy)
is set; a **holding** register is written at once.

## Register map

Input registers, FC04. Power is a 32-bit value in two registers, 0.1 W.

| Addr | Register | Unit | Read as |
|------|----------|------|---------|
| 0 | Inverter status | 1 = normal | batch 0–124 |
| 3, 4 | MPPT1 voltage, current | 0.1 V, 0.1 A | batch |
| 5–6 | MPPT1 power | 0.1 W | batch |
| 7, 8 | MPPT2 voltage, current | 0.1 V, 0.1 A | batch |
| 9–10 | MPPT2 power | 0.1 W | batch |
| 35–36 | AC output power | 0.1 W | batch |
| 37 | Grid frequency | 0.01 Hz | batch |
| 38, 42, 46 | AC voltage L1, L2, L3 | 0.1 V | batch |
| 53–54 | Energy today | 0.1 kWh | batch |
| 91–92 | Energy total | 0.1 kWh | batch |
| 93 | Heatsink temperature | 0.1 °C | batch |
| 105, 106 | Fault code, warning code | | batch |
| 3005–3006 | PV1 power | 0.1 W | single |
| 3009–3010 | PV2 power | 0.1 W | single |
| 3034 | Grid voltage (L3) | 0.1 V | single |
| 3041–3042 | Grid import | 0.1 W | single |
| 3043–3044 | Grid export | 0.1 W | single |
| 3171 | Battery SoC | % | single |
| 3178–3179 | Battery discharge | 0.1 W | single |
| 3180–3181 | Battery charge | 0.1 W | single |

Holding registers, FC03 to read, FC06 or FC16 to write.

| Addr | Register | Layout |
|------|----------|--------|
| 3038 / 3039 … 3044 / 3045 | TOU slots 1–4, start / end | start: bit 15 on, bits 14–13 priority (0 load, 1 battery, 2 grid first), hour, minute; end: hour, minute |
| 3050 / 3051, 3052 / 3053 | TOU slots 5–6 | same |
| 3047 | Battery charge power | % |
| 3048 | Charge stop SoC | % |
| 3049 | AC (grid) charge | 0 / 1 |
| 3067 | On-grid discharge stop SoC | % |

The holding registers start with the values of the live inverter at rest (scan
of 2026-11-16): all slots off, 100 %, grid charge off. `--baseline FILE` loads a
whole scan (`~/gw_debug/gw_last.json`, `gw_scan.json`, or a `scan_diff.py`
file): the settings of that scan become the starting state, and addresses the
simulator does not model answer with the scanned values instead of 0.

## What the simulator models

**Taken from the controller's code, which was verified on the live inverter**

- All addresses, scaling and the TOU bit layout above.
- A write is acknowledged at once but applies 0.4 s later: an immediate read
  still shows the old value.
- An FC06 write to a TOU slot start word is acknowledged and ignored. The pair
  written by FC16 works; 3047 and 3049 take FC06.
- Register 3047 scales the charge power: 50 % is 4.8 kW.
- Grid charge switched on inside a battery-first slot charges within the same
  second; outside it only arms the inverter until the slot opens.
- A slot whose end is before its start never runs.
- A battery-first slot without grid charge keeps the battery from feeding the
  house, but the inverter still runs itself off the battery, about 225 W
  (2026-11-17). Set *Inverter self-use* to 225 to see it; it is 0 by default so
  that the balance stays easy to read.
- The on-grid discharge stop SoC (3067): the battery is not discharged at or
  below it. Below it the inverter charges the battery from the grid although
  grid charge is off: 1.17 kW when more than a point is missing, a 35 W trickle
  for the last point. At the limit the battery is at 0 W and the inverter takes
  its own power from the grid (2026-11-17). The controller's night hold raises
  it to the battery's level and puts 10 % back in the morning.

**Assumptions** (not verified on the inverter)

- The energy balance itself: load first means PV feeds the load, then the
  battery, then the grid; a deficit comes from the battery down to 10 % SoC, then
  from the grid. Nothing is clipped at the inverter's rated power.
- Battery first without grid charge charges from PV only. Grid first discharges
  at full power.
- Register 3067 takes FC06 as well as FC16. On the inverter that was not known
  when this was written; the controller tries FC06 first and falls back to FC16.
- Below the on-grid discharge stop SoC the charge rate also respects register
  3047, and it does not happen in a grid-first slot.
- The inverter's own consumption is the same whether the battery or the grid
  supplies it (measured: 225 W from the battery, 120 W from the grid when idle
  at the limit), and it is only counted when the sun gives less than that.
- **Standby is not modelled.** On the real inverter a battery held at its limit
  sends the inverter into standby after about 15 minutes, and it then does not
  wake up when the limit is lowered: only at first light, or for a grid charge
  (2026-11-17). The simulated battery takes the house back the moment the limit
  is lowered.
- The start-word quirk applies to all six slots (it was seen on 3042).
- Values that make no sense are refused with exception 03: more than 100 %, an
  hour above 23, a grid-charge value other than 0 or 1.
- Unmodelled addresses read 0 and accept writes; a read is not limited to one
  block of 125 addresses.
- Inverter status, fault and warning code are just values. A fault does not stop
  the simulated plant.
- MPPT voltages are fixed (240 V and 600 V while producing).

## Recipes for the manual tests

| To test | Do this in the UI |
|---------|-------------------|
| Too little sun to start | *Night* or *Overcast*; the surplus line shows why |
| A one-phase start | Raise PV until *Surplus for the EV* is between 1500 and 4300 W after the reserve |
| A three-phase start | *Full sun* with SoC at 90 % or more |
| The battery reserve | Move SoC across 90 %: the reserve appears and disappears |
| Current follows the sun | Drag a PV slider, or set PV flicker to 10 % |
| Surplus loss, stop and resume | Drag PV down; *Night* stops at once |
| The closed loop | Tick *Take the EV load from em2go-sim* |
| A bus timeout, a flaky bus | Modbus link = *No reply*; Lost requests = 30 % |
| Garbage on the line | Modbus link = *Reply with a bad CRC* |
| Night charge switched on by the controller | Watch 3038–3049 flash, then *Applied* in the traffic 0.4 s later |
| Charging inside the window | Set the inverter clock into a slot; the battery tile shows the charge |
| Night hold (no charge tonight) | After a controller start the slots are on and grid charge is off. Set the inverter clock into a slot: the house runs on the grid; set it to 07:00 and the battery takes over again. The controller follows the Pi's clock: it raises 3067 to the battery's level when its own window opens and puts 10 % back at 07:00 |
| Why the slots alone are not enough | Set *Inverter self-use* to 225 and the inverter clock into a slot: the battery still gives 225 W. Set 3067 to the SoC: it gives nothing |
| A discharge limit above the battery's level | Set 3067 well above the SoC: the battery charges from the grid at 1.17 kW with grid charge off |
| Somebody changed the settings in the app | Edit a slot or the charge power under Battery settings; the guard reports the difference |
| Switched on, but not charging | Grid charge on and slot active, then pin *Battery charge* to 0 |
| A different inverter state at start | `--baseline ~/gw_debug/gw_scan.json` |

The controller decides about the charging window by the Pi's own clock. The
inverter clock in the UI only moves the simulated inverter.

## Control API

All bodies are JSON. Register ids are `i<address>` and `h<address>`.

| Endpoint | Body | Effect |
|----------|------|--------|
| `GET /api/state` | | Full snapshot: flows, slots, registers, link, counters |
| `GET /api/meta` | | Register map, settings, presets |
| `GET /api/log?since=N`, `GET /api/events` | | Traffic and events; the same as a live stream |
| `POST /api/plant` | `{"pv1_w": 1700, "pv2_w": 5600, "house_w": 600, "ev_w": 0, "soc": 95, "grid_v": 230}` | Any subset |
| `POST /api/register` | `{"rid": "i3043", "value": 4000}` or `{"rid": "h3047", "raw": [25]}` | Set, or pin, a register |
| `POST /api/release` | `{"rid": "i3043"}` or `{}` | Release one pin, or all |
| `POST /api/slot` | `{"n": 1, "enabled": true, "priority": 1, "start": "21:01", "end": "23:59"}` | Edit a TOU slot |
| `POST /api/clock` | `{"time": "22:30"}` or `{}` | Set the inverter clock, or follow the system time |
| `POST /api/settings` | `{"time_speed": 60}` | Behaviour switches |
| `POST /api/link` | `{"port": "/dev/ttyUSB1", "baud": 9600, "unit": 1, "de": "gpio", "de_gpio": 18}` | Reopen the serial link |
| `POST /api/reset` | `{}` or `{"settings": true}` | Resting state |

The REST endpoints of the PC-side `growatt-modbus-mock` (`GET /status`,
`POST /set`, `/scenario`, `/reset`) work too, on the same port 5031.

## Tests

```bash
python3 -m unittest discover -s tests
```

## Limits

- **Tested without RS-485 hardware.** The protocol was run over a virtual serial
  port with the controller's own `GrowattReader` and `GrowattBatterySettings`
  (pymodbus 3.12), and the UART on the pins was opened and configured. No frame
  has gone over a real wire yet, and direction control by GPIO or RTS has not
  driven a real transceiver.
- With DE on a GPIO, the line is released from user space about 0.3 ms after the
  last byte. That is enough as long as the master leaves the usual pause between
  frames (3.5 characters, about 4 ms at 9600 baud). An auto-direction board has
  no such timing to get wrong.
- Nothing is saved: a restart begins at the resting state (or the baseline).
