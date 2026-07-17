# Finishing line cell controller

Python orchestrator for the Simple Wood Goods automated finishing line.
Process spec: [`docs/finishing-line-state-machine.md`](docs/finishing-line-state-machine.md).
Project context: [`CLAUDE.md`](CLAUDE.md).

## Status

Build order (CLAUDE.md) — steps 1 and 2 are done, 3-6 are stubs.

| Step | | |
|---|---|---|
| 1 | Archaeology | **done** — [`cell-config.yaml`](cell-config.yaml) |
| 2 | State machine + simulator + tests | **done** — 67 tests, no hardware needed |
| 3 | Device drivers | **done vs sims** — ClearCore + UR dashboard; RTDE verified on URSim; real-robot motion awaits hardware |
| 4 | Motion primitives | executor/train/supervisor done; force-mode sanding awaits hardware window |
| 5 | FastAPI / HMI | **done** — `python -m finishing_line.api --sim`, HMI at http://localhost:8000 |
| 6 | Commissioning | not started |

Simulation: fake ClearCore (`sim/fake_clearcore.py`, works today) + URSim via
[`docker-compose.ursim.yml`](docker-compose.ursim.yml). Stage-by-stage guide:
[`docs/simulation.md`](docs/simulation.md).

## Layout

```
src/finishing_line/
  core/        pure state machine — no I/O, no clock, fully simulatable
    model.py       domain types; PartState carries the per-part flash timers
    schedule.py    the period-4 tables (§3), declarative
    timers.py      flash accounting — the fan-on-seconds rule lives here
    guards.py      interlock predicates (§7); each returns WHY it blocked
    machine.py     step(state, inputs) -> (state, intents)
    intents.py     what the core asks the world to do
    pairing.py     lead/trail policy — pluggable, see open items
  config/      typed access to cell-config.yaml + line-config.yaml
  process/     device-SPANNING composite ops (see below)
  devices/     dumb executors: ur_rtde, pymodbus
  sim/         fake line; runs the real machine with no hardware
  api/         FastAPI + websocket; the HMI's only interface
```

### Two config files, and why

- **`cell-config.yaml`** — constants extracted from the old Polyscope program and
  ClearCore firmware. Tuned on the real line, proven by 200 parts/week.
  **Do not re-derive these.**
- **`line-config.yaml`** — new process parameters. Each carries a `provenance`
  key saying whether it is `measured` or `assumed`. Most are assumed.
  `ProcessConfig.unmeasured()` lists what the line is running on faith.

### The `process/` layer

Not in the original architecture, and it has to exist. `CLAUDE.md` describes
`sand_faces` as a URScript primitive, but sanding is a **two-axis raster where
each device owns one axis**: the robot force-holds Z at 6 N and traverses base X
by `height`, while the *conveyor* traverses the belt axis by `width - 12`
(script:2460-2497). Neither device can do it alone, and the PC now owns the
conveyor.

So `process/` sits above the drivers and below the core: the core emits
`SandPart(part_id)` and learns only that it finished. That keeps "no I/O in the
core" intact without pretending the robot is self-sufficient.

## Two decisions that shape everything

**Beat advance is event-driven.** A beat ends when its guards pass, not when a
195 s timer expires (§6: validate per-part timers, never beat counts). Throughput
is therefore an *outcome*, not an input — the controller will not force the
schedule.

**Flash timers bank fan-on seconds only.** Wall-clock at a dead fan is not flash
time. This makes "never under-flash" literally true, and it is why P3 stretches:
the trail's entire flash 1 is that one beat, and the IF fan pauses for the lead's
spray burst. `test_p3_is_measurably_longer_than_the_other_beats` observes it in
the running machine.

## Running

```bash
python -m venv .venv
./.venv/Scripts/python.exe -m pip install -e ".[dev]"
./.venv/Scripts/python.exe -m pytest
```

The core has no hardware dependencies — `ur_rtde` and `pymodbus` are optional
extras (`.[devices]`), so the whole suite runs on any machine.

## Open items

Carried from §8 of the process spec, plus what the build surfaced:

- **Measure `spray_burst_pause_s`.** Currently assumed 30 s. Because flash banks
  fan-on seconds, P3 stretches by exactly this, so the period is `780 + pause`
  for 2 parts. Every second costs 0.5 s/part. **The 6.5 min/part target requires
  this to be zero.**
- **Measure zone transfer time** (assumed 15 s).
- **Confirm the 180 s flash** on water-based lacquer. The old line's tuned dry was
  165 s, but that was one dry cycle after a single coat — different process, not
  evidence either way.
- **Verify all four station pitches are equal** — `INQ→IF`, `IF→S`, `S→FD`,
  `FD→OUT`. Nothing spans IF↔S; a part crosses by handoff with both belts
  running together, and every transition crosses it, so every transition moves
  both belts as one. One belt moves everything on it by one distance, and
  sensor-terminating the run measures that distance rather than decoupling it.
  P2→P3 is the sharpest case (retreat S→IF while bringing FD→S, both parts on
  zone 2). If the gaps differ the fix is physical — no termination rule rescues
  it. See `process/train.py`.
- **Assign the IF↔S handoff sensor(s).** `devices/registers.py` predates knowing
  the crossing was sensor-terminated and has no register for it.
- **Assign the new Modbus registers.** `devices/registers.py` proposes addresses;
  none are confirmed against firmware. Which physical motor becomes which zone is
  also unmapped.
- **Enforce single-master.** Modbus TCP does not arbitrate masters — the old
  Polyscope program and this orchestrator can write the same registers
  concurrently with no complaint. Currently procedural
  (`registers.MASTER_HANDOFF`); should be a firmware lease.
- Confirm the denib pass before coat 2 and its duration.
- Confirm browsers fit the same schedule (`pairing.policy`).
- Add sensor debounce — the firmware has none (`cell-config.yaml`
  discrepancies).
