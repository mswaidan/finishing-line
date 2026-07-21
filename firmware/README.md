# ClearCore firmware — finishing line

The C++ twin of the executable spec in
[`src/finishing_line/sim/fake_clearcore.py`](../src/finishing_line/sim/fake_clearcore.py).
Every behaviour here — the echo handshake, the zone move lifecycle with ack
registers, EDGE-triggered sensor stops, shutter feedback, the fail-ON watchdog
that arms on first heartbeat — is pinned down by
[`tests/test_fake_clearcore.py`](../tests/test_fake_clearcore.py). If firmware
and fake disagree, one of them is wrong; fix the pair together.

## Layout

```
finishing_line/
  main.cpp             setup()/loop() entry shim (2 lines); built by PlatformIO
  io_map.h             THE FILE COMMISSIONING EDITS: pins, motors, IP, tuning
  registers.h          Modbus map — mirrors devices/registers.py (source of truth)
  modbus_tcp.h/.cpp    hand-rolled Modbus TCP server (FC 1-6, 16), no 3rd-party lib
  firmware.h/.cpp      the tick: sensors -> echo -> watchdog -> fans -> shutter -> zones
```

## What differs from the legacy modbustest.ino

- **Static IP (192.168.1.19)** — DHCP is gone. The old setup had two clients
  addressing the ClearCore at two different IPs (cell-config discrepancies).
  The rewrite unit uses .19 so it never collides with the legacy production
  unit, which keeps 192.168.1.18 while it runs during the transition.
- **Sensor debounce (10 ms)** — the legacy firmware had none; edges now stop
  belts, so noise immunity is mandatory. Costs ~0.5 mm at 53 mm/s.
- **No third-party Modbus library** — the server is ~200 lines we own.
- **Legacy register vocabulary survives** (100/200 block echo, feed coil 107
  actively drives the IN belt). Rollback still means flashing the OLD
  firmware; this one serves the legacy addresses for observability only.

## Build & flash

PlatformIO. Config is [`platformio.ini`](../platformio.ini) at the repo root;
it points `src_dir` at this folder and pulls the `clearcore` board + Arduino
core from a pinned fork of `platform-atmelsam` (ClearCore has no upstream PIO
board yet — see the ini header). No Boards-Manager step: `pio run` fetches the
pinned platform on first build.

Verified 2026-07-21 (this repo): compiles clean, ~125 KB flash / 47 KB RAM;
flashed and verified over the SAM-BA bootloader. The serial boot banner
(COM port @ 115200) reports the static IP and Ethernet link status — the
authoritative check that the rewrite firmware is the one running.

```
pio run                               # compile
pio run -t upload --upload-port COMx  # flash
pio device monitor                    # serial console @ 115200
```

To flash, put the board in the bootloader: double-tap the ClearCore reset
button — it enumerates as "Teknic ClearCore UF2 Bootloader" and mounts a
CLEAR_BOOT drive. Always pass `--upload-port` explicitly; auto-detect can grab
a stray Bluetooth COM port instead.

Note: `pio` lives inside the VSCode PlatformIO extension's venv
(`~/.platformio/penv/Scripts`), which the extension does not add to PATH — add
it there to run `pio` from any shell.

Note: the ClearCore core is gnu++11/14 — no C++17 (its Common.h `min`/`max`
macros break it), no default member initializers in aggregates, no brace-list
range-for. Keep new code C++11-clean.

## Bench bring-up (before the line)

1. Flash; confirm the serial banner shows the static IP, then ping 192.168.1.19.
2. From the repo: point the driver tests' fixtures at the real device — or
   quicker, run `python -m finishing_line.api --cc 192.168.1.19` and use the
   HMI. No motors or sensors need be wired for the register-level checks.
3. Walk `io_map.h`: confirm every TODO(wiring) assignment against the panel,
   with motors and sensors connected one at a time.
4. Verify the watchdog: stop the orchestrator; fans must force ON within 2 s.
5. Verify a sensor-stop: arm MODE_SENSOR_STOP on a bench zone, trip the target
   sensor by hand, confirm the belt stops and state reads READY.

## Open wiring decisions (all marked TODO(wiring) in io_map.h)

- Which physical motor becomes which zone (M0/M2 assumed for zones, M1 feed).
- Sensor pin assignments (IO0-IO4 assumed for presence + handoffs).
- Shutter: solenoid polarity (energise = open assumed) and end-switch pins.
- Fan relay pins; whether any real airflow feedback sensor gets added.
- INQ_COUNT register currently serves 0 — no queue-count sensor exists; add
  one or drop the register from the HMI display.
