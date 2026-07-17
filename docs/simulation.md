# Simulation guide

Three stages, each validating a layer the previous one can't reach. The
principle throughout: **the code under test is the code that runs the line** —
only the physics is fake.

| Stage | What's real | What's fake | Validates |
|---|---|---|---|
| A | state machine, guards, timers | everything physical (`FakeLine`) | scheduling, interlocks, fault recovery |
| B | + drivers, process layer, Modbus/RTDE wire traffic | devices (fake ClearCore, URSim) | handshakes, watchdog, intent plumbing |
| C | + FastAPI, HMI | devices | operator flows |

## Stage A — pure Python (no setup)

```
./.venv/Scripts/python.exe -m pytest
```

Covers: steady state, startup fill, drain even/odd/single, protective-stop
recovery, sensor-mismatch resync, the time-compressed 20-part soak, and the
observed P3 stretch. Wall-clock: seconds. Runs anywhere, no hardware, no Docker.

## Stage B — real drivers against simulated devices

### Fake ClearCore (works today)

`src/finishing_line/sim/fake_clearcore.py` is a Modbus TCP server implementing
the register map in `devices/registers.py`: echo handshake, zone-move lifecycle,
shutter actuation delay, fail-ON watchdog (arms on first heartbeat). It is the
executable spec for the real firmware changes.

```python
from finishing_line.sim.fake_clearcore import FakeClearCore
cc = FakeClearCore(port=15020).start()
```

`tests/test_fake_clearcore.py` exercises it through the pymodbus **client** —
the same library `ClearCoreClient` will use. Sensors are inputs the harness
pokes (`cc.set_input(...)`): the fake emulates the controller, the test emulates
the physics. That split is deliberate and mirrors the real line.

### URSim (needs Docker Desktop)

1. Install Docker Desktop (WSL2 backend). Not currently installed on this
   machine.
2. `docker compose -f docker-compose.ursim.yml up -d`
3. Open http://localhost:6080/vnc.html — first boot: confirm the safety
   config, power on, release brakes.
4. `python scripts/ursim_smoke.py` → `SMOKE TEST PASSED`

URSim runs the real Polyscope, so the Dashboard server (29999) and RTDE (30004)
behave like the cabinet. `URClient` develops against it unmodified.

### The ur-rtde Windows problem (verified 2026-07-17)

`pip install ur-rtde` has **no Windows wheel** — it fails on this machine and
would need a Boost/MSVC source build. Options, in order of preference:

1. **Develop the drivers under WSL2.** Docker Desktop already requires WSL2;
   a Linux venv there installs `ur-rtde` from a manylinux wheel and reaches
   both URSim and the fake ClearCore over localhost.
2. Build from source on Windows (Boost + MSVC — hours of yak-shaving, fragile).
3. Fall back to UR's pure-Python RTDE client (no wheel problem, but a different
   API than the decided-on `ur_rtde`).

**Resolved (2026-07-17): the cell PC will run Linux.** So production gets the
manylinux wheel, the `ur_rtde` decision stands, and driver development under
WSL2 uses the identical stack that deploys. The Windows-wheel problem is a
dev-machine footnote, not an architecture constraint.

### WSL2 environment (dev machine)

The distro is `Ubuntu-24.04`, default user `mswaidan`; the venv lives at
`/home/mswaidan/fl-venv` (Python 3.12, ur-rtde, pymodbus, pytest), with the
smoke script staged at `/home/mswaidan/rtde_smoke.py`.

**Verified end to end (2026-07-17):** RTDE receive + control + a moveL
round-trip against URSim, run from PowerShell with:

```powershell
wsl -d Ubuntu-24.04 -- /home/mswaidan/fl-venv/bin/python /home/mswaidan/rtde_smoke.py
```

**The repo is on GitHub** (`mswaidan/finishing-line`, private) and cloned at
`/home/mswaidan/finishing-line` inside WSL, with the project installed editable
into `fl-venv` — the full test suite passes there (verified 2026-07-17). WSL
git authenticates through the Windows credential manager bridge
(`credential.helper` points at `git-credential-manager-core.exe`).

Workflow: the NAS working copy (`L:\...\Code`) and the WSL clone are two
checkouts of the same GitHub remote — push from one, pull in the other.

Two Windows-side quirks, learned the hard way:

- Launch `wsl.exe` from a `C:` working directory; launching from `L:` (a
  network drive WSL can't translate) fails.
- From Git Bash, prefix `wsl.exe` calls with `MSYS_NO_PATHCONV=1` or
  arguments like `/root` get rewritten to Windows paths.

URSim reachability from WSL: `rtde_smoke.py` probes `localhost` first
(mirrored networking) and falls back to the WSL gateway IP (NAT networking),
so it finds the Windows-published container ports either way. Verified: WSL
reaches URSim at plain `localhost`.

### Remote Control mode (one-time URSim step)

e-Series Polyscope **refuses external control scripts in Local mode** — RTDE
*receive* works, but `RTDEControlInterface` fails with a "data
synchronization" timeout. Verified on URSim 5.25: `is in remote control` →
`false` out of the box, and no headless way to flip it (no env var, no launch
flag, no settings file to edit).

One-time fix in the pendant UI (http://localhost:6080/vnc.html): hamburger
menu → **Settings → System → Remote Control → Enable**, then flip the new
Local/Remote toggle (top right) to **Remote**. Survives `docker stop/start`;
must be redone if the container is recreated (`compose down`).

`scripts/rtde_smoke.py` prechecks this via the dashboard and says exactly this
instead of the cryptic timeout.

## Stage C — full stack (after build-order step 5)

Orchestrator + FastAPI + HMI in a browser against the Stage B fakes: batch
declaration, `blocked_by` display, fault acknowledge/resume, the rollback
procedure (`registers.MASTER_HANDOFF`).

## What no simulation validates

Physical truths only a maintenance window can touch:

- Force-mode sanding: URSim has **no physics** — `force_mode`/`tool_contact`
  run but don't behave. The sanding duet's feel is real-line-only.
- Spray/finish quality, actual flash time, the spray-burst pause duration.
- Real transfer times; the four station pitches; handoff sensor placement.
- Sensor noise/debounce behaviour.

The sims exist so that the maintenance-window list is exactly that list and
nothing else.
