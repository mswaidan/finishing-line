# ClearCore hardware map

Decided 2026-07-17. This is the wiring contract: `firmware/finishing_line/io_map.h`
implements exactly this table, and commissioning checks each row off physically.

## Motor connectors (step/dir to external open-loop stepper drivers)

| Connector | Drives | Notes |
|---|---|---|
| M0 | Zone 1 belt (INQ↔IF) | |
| M1 | Feed belt (INQ queue) | same role as legacy M1 |
| M2 | Zone 2 belt (S↔FD↔OUT) | |
| M3 | Brush motor | legacy BRUSH_ON coil (108) drives it; legacy tuned values: 10 000 steps/s, accel 100 000 |

## I/O points — all 13 used

Outputs can only live on IO-0…IO-5; DI-6…8 and A-9…12 are input-only.

| Pin | Dir | Function | Register |
|---|---|---|---|
| IO-0 | in | IF presence eye (NPN) | 403 |
| IO-1 | in | S presence eye (NPN) | 404 |
| IO-2 | out | IF fan relay | cmd 301 / feedback 401 |
| IO-3 | out | FD fan relay | cmd 302 / feedback 402 |
| IO-4 | out | Shutter OPEN solenoid | cmd 300 (=1) |
| IO-5 | out | Shutter CLOSE solenoid | cmd 300 (=0) |
| DI-6 | in | FD presence eye (NPN) | 405 |
| DI-7 | in | Shutter open end switch | feedback 400 |
| DI-8 | in | Shutter closed end switch | feedback 400 |
| A-9 | in | Handoff→Z2 eye (part fully on zone 2) | 407 |
| A-10 | in | Handoff→Z1 eye (part fully on zone 1) | 408 |
| A-11 | in | INQ queue-head eye | 409 |
| A-12 | in | OUT occupancy eye | 415 |

Zero spares. Growth path: CCIO-8 expansion boards (8 points each, up to 8
boards) on the serial link — e.g. for a future airflow sensor on the fan
feedback registers, or a safety-circuit-OK input.

## The two new sensors' behaviour (software contract)

- **INQ queue-head eye (409)**: an `INQ→IF` feed move is *blocked* (not
  faulted) while the eye sees nothing — the HMI shows "queue head empty —
  load parts at INQ" and the line proceeds the moment parts appear.
- **OUT occupancy eye (415)**: an outfeed move is *blocked* while a finished
  part sits unremoved at OUT — "outfeed occupied — remove the finished part".
  Prevents pushing a cube into a cube.

## Shutter valve

Double-solenoid 5/2, detented: holds last position de-energised, satisfying
§7's shutter-holds-state rule through any power or control failure. Firmware
energises the commanded side continuously and the opposite side off; position
truth comes from the two end switches (register 400: 0 closed / 1 open /
2 travelling-or-switch-fault).

## Electrical notes

- One 24 VDC supply: ClearCore, sensors, valve solenoids.
- Fans are AC loads: interposing relay or contactor per fan, 24 V coil driven
  from the IO pin. Check the ClearCore datasheet's per-pin sink rating before
  direct-driving anything; when in doubt, relay.
- All presence eyes NPN sinking, matching the legacy sensors' wiring practice.
- Sander and sprayer are NOT ClearCore loads — they remain on the robot
  cabinet's digital outputs (DO3 / DO5), as in the legacy program.
- Safety (e-stops, UR safety config) stays in hardware, outside the ClearCore.

## Shopping list deltas vs today's line

New: 4 photo-eyes (handoff ×2, INQ, OUT), 2 shutter end switches, 5/2
double-solenoid valve, second fan + contactor, 1 stepper driver if zone 1's
belt is new hardware. Reused: 3 presence eyes' wiring practice, feed belt
driver, brush motor + driver, existing fan + contactor.
