# Finishing Line Rewrite — Project Context

## What this is

Ground-up software rewrite of Simple Wood Goods' automated finishing line (Baltic birch storage cubes and browsers), concurrent with a physical upgrade that adds a second flash fan and a baffle/shutter so parts can interleave. Goal: cut effective cycle from ~10 min/part to ~6.5 min/part.

**Authoritative process spec: `docs/finishing-line-state-machine.md`** — the period-4 interleaved schedule, zone motions, fan/shutter states, interlocks, startup/drain. Read it before touching orchestration logic.

## Throughput targets

- Current: ~200 cubes/week + ~40 browsers/week, line saturated at 8 h/day, 10 min cycle (600 s: sand 60, spray 60, flash 2×180, handling ~120)
- Required: 250 cubes/week within weeks (Q4 inventory build), 300/week next year
- New schedule capacity: ~195 s beats, 2 parts per 4 beats → ~370/week at 8 h/day

## Hardware

- **UR5e** — pneumatic sander + HVLP gun (water-based lacquer, 2 coats, 180 s flash each). Sand to 220; standard: evenly reflective, smooth to touch.
- **Teknic ClearCore** — two reversible conveyor zones (Zone 1: IF↔S, Zone 2: S↔FD↔OUT), position/presence sensors, will also drive both fans + shutter. Firmware source: `modbustest.ino` in THIS repo is authoritative for the rewrite (a copy also exists in a separate personal GitHub repo — treat that one as historical); firmware changes are authored here, in lockstep with `devices/registers.py` and the fake ClearCore.
- **Stations:** INQ (infeed queue, 4 parts) → IF (new upstream fan / staging) → S (sand+spray) → FD (downstream fan) → OUT (offload at fan end).
- **New physical:** upstream fan at IF; rigid baffle panel between IF and S with actuated shutter window (likely pneumatic slide gate); kraft-paper sacrificial facing on spray side.
- Cell PC (currently browser-only) becomes the orchestrator host, **running Linux** (decided 2026-07-17 — `ur-rtde` has no Windows wheel; manylinux wheel deploys the identical stack developed under WSL2). HMI web app (JS) currently hosted on the NAS.

## Architecture (decided)

Python cell controller on the cell PC owns ALL orchestration; devices are dumb executors.

- **Orchestrator:** the state machine as a pure-Python package — beat scheduling, per-part state {coats applied, flash-1 s, flash-2 s}, interlock predicates, fault recovery. No I/O in the core package; fully unit-testable and simulatable.
- **UR5e:** thin library of parameterized URScript motion primitives (`sand_faces`, `spray_pass(coat)`, `safe_pose`, ...) invoked from Python via `ur_rtde`; Dashboard server (port 29999) for load/play/stop and protective-stop recovery. No program flow in URScript.
- **ClearCore:** remains Modbus TCP slave (`pymodbus` from Python; master flips from robot to PC). Existing register vocabulary (states + move commands) survives; add registers for shutter, second fan, new sensors.
- **HMI:** talks only to the orchestrator (FastAPI + websocket). Never directly to robot or ClearCore.
- **Safety stays in hardware** (e-stops, UR safety config). Heartbeats both directions: ClearCore halts zones and UR holds if orchestrator watchdog goes quiet; fans fail ON.

## Build order

1. **Archaeology first:** parse the old Polyscope export (`.urp` = gzipped XML; `.script` = flat generated URScript; `.installation` = TCP/payload/mounting) and ClearCore firmware. Extract every tuned constant — waypoint poses, speeds, accelerations, sander dwell, spray trigger timing/pass overlap, move distances, debounce values — into a structured config file. Do not re-derive tuned values.
2. State machine package + simulator + tests (steady state, startup fill, drain even/odd, every fault case)
3. Device drivers (`ur_rtde`, `pymodbus`) against hardware in maintenance windows
4. Motion primitives consuming the extracted constants
5. FastAPI/websocket layer, HMI hookup
6. Commissioning checklist

## Constraints and conventions

- **The old Polyscope program must remain loadable as rollback.** Line produces 200/week during the transition; any morning must be able to run linear mode.
- Validate per-part timers, never beat counts. Faults may over-flash a part, never under-flash.
- Zone motion only when: robot clear, gun off, shutter open confirmed, destination empty. Spray only when: shutter closed, part located at S, IF fan paused if a wet part sits at IF.
- Timing budget assumes 15 s zone transfers — **measure on the real line** (open item). At 30 s the cycle is ~7 min/part, still above target.
- Two products share the line: cube (rebated front edge) and browser (~40/wk). Verify browsers fit the same schedule or run in dedicated blocks.

## Open items (from the state machine doc)

- Measure actual zone transfer time
- Choose shutter actuator + feedback sensor
- Confirm denib pass before coat 2 and its duration
- Decide whether HMI moves off the NAS onto the orchestrator later
