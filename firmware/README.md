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
  finishing_line.ino   Arduino entry shim (2 lines) — the flashable sketch
  io_map.h             THE FILE COMMISSIONING EDITS: pins, motors, IP, tuning
  registers.h          Modbus map — mirrors devices/registers.py (source of truth)
  modbus_tcp.h/.cpp    hand-rolled Modbus TCP server (FC 1-6, 16), no 3rd-party lib
  firmware.h/.cpp      the tick: sensors -> echo -> watchdog -> fans -> shutter -> zones
```

## What differs from the legacy modbustest.ino

- **Static IP (192.168.1.18)** — DHCP is gone. The old setup had two clients
  addressing the ClearCore at two different IPs (cell-config discrepancies).
- **Sensor debounce (10 ms)** — the legacy firmware had none; edges now stop
  belts, so noise immunity is mandatory. Costs ~0.5 mm at 53 mm/s.
- **No third-party Modbus library** — the server is ~200 lines we own.
- **Legacy register vocabulary survives** (100/200 block echo, feed coil 107
  actively drives the INQ belt). Rollback still means flashing the OLD
  firmware; this one serves the legacy addresses for observability only.

## Build & flash

Arduino IDE: install the ClearCore board package (Boards Manager URL
`https://www.teknic.com/files/downloads/package_clearcore_index.json`), open
`finishing_line/finishing_line.ino`, select the ClearCore board, upload over
USB — same flash path as the legacy sketch.

Headless (verified 2026-07-17, arduino-cli 1.5.1, ClearCore:sam 1.7.1 —
compiles clean, 188 KB flash / 49 KB RAM):

```
arduino-cli core install ClearCore:sam \
  --additional-urls https://www.teknic.com/files/downloads/package_clearcore_index.json
arduino-cli compile --fqbn ClearCore:sam:clearcore firmware/finishing_line
arduino-cli upload  --fqbn ClearCore:sam:clearcore -p <COM-port> firmware/finishing_line
```

Note: the ClearCore toolchain is gnu++11 — no default member initializers in
aggregates, no brace-list range-for. Keep new code C++11-clean.

## Bench bring-up (before the line)

1. Flash; confirm ping at 192.168.1.18.
2. From the repo: point the driver tests' fixtures at the real device — or
   quicker, run `python -m finishing_line.api --cc 192.168.1.18` and use the
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
