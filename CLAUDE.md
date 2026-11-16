# growatt-sim — Claude context

Simulator of the Growatt inverter for testing `~/solar_charger`: a Modbus RTU
slave on a serial port plus a web UI. The folder is `raspberry3/growatt-sim` on
the SSHFS mount; it is meant to be copied to a spare Pi with RS-485 adapters.
`README.md` is the user-facing reference: wiring, UI, register map, modelled
behaviour, API. `em2go-sim` next to it is the same thing for the wallbox and
shares the UI design.

It is separate from the PC-side `growatt-modbus-mock` (pymodbus + FastAPI).

## Layout

    growatt_sim/registers.py      register map and TOU word encoding — from growatt_reader.py and night_charge.py
    growatt_sim/model.py          Inverter: plant, energy balance, TOU slots, delayed writes, settings, log
    growatt_sim/rtu.py            CRC, frame parser, termios serial port, pty, RS-485 direction control, RTU slave thread
    growatt_sim/web.py            UI + JSON API + SSE stream; EvFollower (reads em2go-sim's power)
    growatt_sim/client.py         RTU master and CLI (test / dump / read / write / write16 / watch)
    growatt_sim/static/index.html the whole UI: one file, no build step, no external resources
    growatt_sim/__main__.py       argument parsing, wiring, tick thread
    tests/test_sim.py             unittest: model with fake clocks, parser byte by byte, RTU over a pty, HTTP

## Rules

- **Standard library only.** Serial access is `termios` + `os.read/write`, GPIO
  is the kernel character device by `ioctl`. No pyserial, no GPIO library.
- **The controller's code is the source for the register map.** When
  `growatt_reader.py` or `night_charge.py` starts using a register, add it to
  `REGISTERS` (the UI and the client pick it up from there) and extend
  `RegisterMapTest`.
- **One lock.** Every public `Inverter` method takes `self.lock`; call `_bump()`
  after a change so the SSE stream pushes it.
- Reads compute the energy balance on the spot (`_flows`), so all registers of
  one moment agree. Only SoC and the energy counters integrate in `tick()`.
- Master writes go through `pending` and apply after `write_apply_delay_ms`;
  UI edits apply at once. Every read path calls `_apply_due()` first.
- Echo from a transceiver cannot be recognised by content (an FC06 reply equals
  its request), which is why it is a link setting and not auto-detected.
- Behaviour that was not verified on the inverter is listed under "Assumptions"
  in the README. Keep that list in step with the code.

## Not yet tested on hardware

Everything was verified over a pty with the controller's `GrowattReader` and
`GrowattBatterySettings` (pymodbus 3.12.1), and `/dev/serial0` was opened and
configured on raspberry3. No frame has crossed a real RS-485 wire, and
`GpioDirection` / `RtsDirection` have never driven a transceiver (the ioctl
struct layout is checked against `<linux/gpio.h>`, sizes confirmed on the Pi).
Expect to tune `turnaround_ms` and the DE release when the bench first runs.

## Commands

```bash
./run.sh --port pty                        # virtual port /tmp/ttyGROWATT, UI :5031
python3 -m unittest discover -s tests      # on the Pi or on the PC through the mount
python3 -m growatt_sim.client --port /tmp/ttyGROWATT test
```

Do not write to `/dev/ttyUSB0` on raspberry3: that is the real inverter, and the
live service owns the line. Do not stop the live `solar_charger` service to try
something out unless the user asks.
