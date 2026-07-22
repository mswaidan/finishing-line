# Finishing Line

The software that runs Simple Wood Goods' automated finishing line.

## What is this? (start here)

Simple Wood Goods builds wooden storage cubes. Before a cube can be sold, it
needs **finishing**: sanding it smooth, then spraying it with a clear
protective coating called lacquer, then letting that coating dry.

We have a small factory line that does this automatically:

- A **robot arm** holds the sander and the spray gun.
- **Conveyor belts** carry each cube from station to station, like a very
  slow train with one car per stop.
- **Fans** blow air on freshly sprayed cubes to dry them.
- A **movable wall with a window** (we call it the shutter) sits between the
  spraying area and one of the drying areas, so spray mist never lands on a
  cube that is busy drying.

This repository is the "brain" for all of that: one program, running on a
regular PC, that tells the robot, the belts, the fans, and the shutter what
to do and when.

## The journey of one cube

A cube visits stations in a line. Reading left to right:

```
 IN  ->  F1  ->  O  ->  F2  ->  OUT
queue    fan 1   robot   fan 2   done
```

1. **IN** — the waiting line. Cubes sit here until it's their turn.
2. **F1** — a resting spot with a fan overhead (fan #1).
3. **O** — the work station. The robot sands the cube, then sprays coat #1.
4. **F2** — the drying spot (fan #2). The cube sits here while the wet
   lacquer dries. This takes about 3 minutes.
5. Back to **O** — the robot lightly smooths the dried coat and sprays
   coat #2.
6. Back to **F2** — dry again.
7. **OUT** — finished! The cube leaves the line.

Drying is the slow part. The robot needs about a minute per visit, but each
coat needs about **3 minutes** under a fan. If we ran one cube at a time, the
robot would spend most of its day watching paint dry.

## The trick: two cubes leapfrogging

The clever part of this line is that **two cubes share it at once**, taking
turns. While one cube is drying under a fan, the robot works on the other.
They take turns in a repeating 4-step pattern (we call each step a **beat**,
like a drumbeat):

- One cube (the **lead**) always dries at the *downstream* fan (F2).
- Its partner (the **trail**) retreats *backwards* to the upstream fan (F1)
  for its first dry — that's why fan #1 exists.

Because of this leapfrogging, the robot is almost never idle, cubes never
pass each other, and the line finishes a cube roughly every 6.5 minutes
instead of every 10. Over a week, that's the difference between ~200 cubes
and ~370.

The shutter (the wall with the window) opens only while cubes are moving
between stations, and closes whenever the robot sands or sprays — so dust
and spray mist stay in the work area, away from drying cubes.

## The three rules that keep cubes good

Everything in this software follows three simple safety ideas:

1. **Every cube carries its own stopwatch.** The software tracks exactly how
   many seconds *each cube* has spent drying under a *running* fan. A cube
   may not leave a fan early, may not get coat #2 before coat #1 is dry, and
   may not leave the line before coat #2 is dry. No exceptions.
2. **Too much drying is fine; too little never is.** If something goes
   wrong, cubes may sit under fans longer than needed — that's harmless. The
   software is built so that no failure can ever *shorten* a drying time.
3. **When in doubt, stop and ask a human.** If sensors disagree with what
   the software believes (say, a cube isn't where it should be), everything
   stops, the fans keep running, and the screen asks the operator to confirm
   where each cube actually is before continuing.

## Who's in charge of what

- **The PC (this software)** makes every decision: what happens next, which
  belt moves, when the robot works. It shows a control screen (the HMI) in a
  web browser: start/pause buttons, a live picture of the line, and plain
  explanations whenever the line is waiting ("waiting: cube c001 has dried
  142s of 180s").
- **The robot** (a Universal Robots UR5e arm) only knows *how* to sand and
  spray — never *when*. It waits to be told.
- **The ClearCore** (a small industrial controller board) runs the belt
  motors, reads the sensors, and switches the fans and shutter. It also only
  follows orders — with two reflexes of its own: it stops a belt the instant
  a sensor says a cube arrived (so cubes stop in exactly the right spot),
  and if the PC ever goes silent, it stops all belts and forces both fans ON
  so nothing under-dries while nobody is in charge.

Everything above this line is the whole idea. Everything below is detail.

---

## Technical overview

Ground-up rewrite of the finishing line control (see [CLAUDE.md](CLAUDE.md)
for project context, [docs/finishing-line-state-machine.md](docs/finishing-line-state-machine.md)
for the authoritative process spec — the period-4 interleaved schedule,
interlocks, and fault rules the plain-language story above describes).

### Status (build order from CLAUDE.md)

| Step | | |
|---|---|---|
| 1 | Archaeology (legacy constants) | **done** — [`cell-config.yaml`](cell-config.yaml) |
| 2 | State machine + simulator + tests | **done** |
| 3 | Device drivers | **done vs sims** — RTDE verified on URSim; real-robot motion awaits hardware |
| 4 | Motion primitives | executor/train/supervisor done; force-mode sanding awaits hardware window |
| 5 | FastAPI / HMI | **done** — `python -m finishing_line.api --sim` |
| 6 | Commissioning | **in progress** — ordered bring-up plan in [`docs/commissioning.md`](docs/commissioning.md); firmware flashed, sensors + robot comms validated |

108 tests, all runnable with zero hardware, on Windows and Linux.

### Layout

```
src/finishing_line/
  core/        pure state machine — no I/O, no clock, fully simulatable
    model.py       domain types; PartState carries the per-part flash timers
    schedule.py    the period-4 beat tables, declarative
    timers.py      flash accounting — the fan-on-seconds-only rule
    guards.py      interlock predicates; each returns WHY it blocked
    machine.py     step(state, inputs) -> (state, intents); resume() recovery
    pairing.py     lead/trail policy (pluggable — browser question open)
  config/      typed access to cell-config.yaml + line-config.yaml
  process/     orchestration above the drivers
    supervisor.py  the tick: sensors -> step -> intents -> heartbeat
    controller.py  thread-safe facade for the API; pause/halt/batch/ack
    executor.py    ordered intent worker; fault poisons, halt jumps queue
    train.py       the two-belt handoff manoeuvre (sensor-stop targets)
    persistence.py state snapshots; restart restores via the fault flow
  devices/     dumb executors: ClearCore Modbus client, UR dashboard/RTDE
  sim/         fake ClearCore (executable firmware spec), fake robot, physics
  api/         FastAPI + websocket + the HMI page
firmware/      ClearCore C++ — the fake's twin; compiles, ready to flash
tests/         108 tests incl. full-stack integration over real Modbus
```

### Two config files, and why

- **[`cell-config.yaml`](cell-config.yaml)** — archaeology. Constants
  extracted from the old Polyscope program and ClearCore firmware, tuned on
  the real line and proven by 200 parts/week. **Do not re-derive these.**
  Includes a `discrepancies` section for oddities found during extraction.
- **[`line-config.yaml`](line-config.yaml)** — new process parameters. Each
  carries a `provenance` marker (`measured` vs `assumed`); the HMI footer
  lists everything the line currently runs on faith.

### Decisions that shape the code

- **Beat advance is event-driven.** A beat ends when its guards pass, never
  when a timer expires. Throughput is an outcome, not an input.
- **Flash timers bank fan-on seconds only** — measured against fan
  *feedback*, not commands. This makes rule #2 above literally true, and it
  is why the P3 beat stretches by the spray-burst fan pause.
- **Completion means confirmed.** Every device operation reports done only
  when sensors agree (shutter feedback, arrival sensors, move acks) — never
  when a command was merely sent.
- **Belt stops are a firmware reflex.** The ClearCore stops belts on sensor
  *edges* (not levels) in its own 10 ms loop; there are no encoders and no
  closed-loop motors, so the sensor edge is the only positioning truth.
- **Restart = fault.** State persists to disk; a restart with parts on the
  line restores into the same confirm-occupancy-and-resume flow as any
  other fault.

### Running

```bash
python -m venv .venv
./.venv/Scripts/python.exe -m pip install -e ".[dev,api]"
./.venv/Scripts/python.exe -m pytest                     # full suite, no hardware
./.venv/Scripts/python.exe -m finishing_line.api --sim   # HMI at localhost:8000
```

`--sim` runs the entire line in simulation (compressed times): declare a
batch in the HMI, press Run, watch cubes leapfrog. `--cc HOST` targets a
real ClearCore with a fake robot — the conveyor commissioning mode.

Simulation stages, URSim (robot simulator) setup, and the WSL2 environment:
[docs/simulation.md](docs/simulation.md).

### Open items (measure before trusting the throughput math)

- **Spray-burst pause duration** — every second of the P3 fan pause costs
  0.5 s/part; the 6.5 min/part target assumes zero.
- **Zone transfer time** (assumed 15 s) and the **180 s flash** (assumed,
  not measured on this lacquer).
- **All four station pitches must be equal** — the one-advance-per-beat
  schedule depends on it (see `process/train.py`).
- **Wiring assignments** — every `TODO(wiring)` in
  [`firmware/finishing_line/io_map.h`](firmware/finishing_line/io_map.h).
- Browsers on the same schedule vs dedicated blocks (`pairing.policy`).
- Rollback: the legacy Polyscope program + old firmware remain the fallback;
  see `devices/registers.py` MASTER_HANDOFF for the handoff procedure.
