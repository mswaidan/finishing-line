# Commissioning plan — finishing line rewrite

Ordered, one-subsystem-at-a-time bring-up of the rewrite on the real line. Each
phase has a **precondition**, a **procedure**, and a **done** bar. Do them in
order — later phases assume the earlier ones passed.

Status as of 2026-07-22: Phase 0 largely done on the bench, Phase 1 bench-validated,
Phase 5 blocked on parts. Everything else is untouched by hardware.

## Standing rules (every phase)

- **E-stop within reach** for any belt or robot motion. First move of any
  actuator is slow and small.
- **Keep the legacy Polyscope program loadable as rollback.** The line makes
  200/week during the transition; any morning must be able to run linear mode
  (`registers.MASTER_HANDOFF`).
- **Watchdog fails ON.** If the orchestrator goes quiet, fans force ON and belts
  halt. Never defeat this — it's what keeps parts drying if the PC dies.
- **One change at a time**, verified, before the next. When you measure a tuned
  value, **write it into config** (see the capture list at the bottom).
- Two Modbus masters must never drive the ClearCore at once (orchestrator vs the
  legacy robot program).

---

## Phase 0 — Power, comms, IO baseline  ✅ (mostly done)

- [x] Rewrite firmware flashed; static IP **192.168.1.19**; serial banner prints IP + link.
- [x] Ping .19; Modbus reads work (`scripts/line_monitor.py`, `scripts/sensor_watch.py`).
- [ ] **Watchdog fail-ON at the line**: stop the orchestrator heartbeat, confirm
  both fans force ON within ~2 s (`watchdog.clearcore_timeout_s`) and zones halt.
  *(Logic verified in tests; re-confirm with real fans in Phase 4.)*

## Phase 1 — Sensors  ✅ bench / ⬜ line placement

Bench-complete: wiring, polarity (`EYES_ACTIVE_LOW`), per-eye reads confirmed, one
dead unit swapped. Remaining work is physical:

- [ ] Mount each eye at its station; trip-check with `sensor_watch` → `present=on`.
- [ ] **Handoff eyes** (Z1/Z2, A10/A9) at the F1↔O crossing — their placement sets
  where the belts stop mid-handoff. Position deliberately.
- [ ] Each presence eye trips reliably at its station with dust/mist margin (the
  F18 can be swapped for the FDM3-0N-1H if a position proves marginal).
- [ ] Confirm every eye's `io_map.h` pin assignment against the panel.

## Phase 2 — Belts / motors  ⬜ (DO FIRST at the line)

Nothing has driven a real belt. Every motor assignment in `io_map.h` is a
`TODO(wiring)` assumption: `Z1_BELT=M0` (IN↔F1), `IN_BELT=M1` (feed), `Z2_BELT=M2`
(O↔F2↔OUT), `O_BRUSH=M3`.

**Precondition:** belts mechanically free, no parts on the line, e-stop ready.

Per motor, **one at a time** (jog via a short `ClearCoreClient.move_zone_mm(zone, mm)`
script, or the feed/brush coils):

- [ ] **Mapping** — confirm which physical belt each connector drives. Fix
  `io_map.h` + reflash if the assumed mapping is wrong.
- [ ] **Direction** — a `+distance` move must go **downstream**. If reversed, flip
  `INVERT_STEP_DIRECTION` and reflash.
- [ ] **Distance calibration** — command a known mm, measure actual travel. Confirm
  the tuned `29.996 steps/mm` (`cell-config` conveyor kinematics; the belt figure,
  *not* the dead `inchesToSteps` 1.466) lands right for this mechanism.
- [ ] Feed belt (M1, coil 107) and brush (M3, coil 108) run on their legacy coils —
  jog those too.

**Done:** every belt moves the correct direction and distance on command.

## Phase 3 — Sensor-stop with a real belt + part  ⬜

- [ ] Arm `MODE_SENSOR_STOP` on a zone (`move_zone_until`), run the belt, feed a
  part, and confirm the **firmware** stops it on the sensor **edge** (not level),
  belt halts, zone reports READY.
- [ ] Measure the stop distance (target ≈0.5 mm at 53 mm/s + 10 ms `DEBOUNCE_MS`).
  Confirm parts seat in position repeatably.

**Done:** parts stop repeatably at each station via the sensor edge, no orchestrator
in the positioning chain.

## Phase 4 — Fans + relays  ⬜

- [ ] Command each fan (F1 = IO2, F2 = IO3) → confirm the interposing relay/contactor
  switches and airflow is present.
- [ ] Fan feedback register mirrors the commanded relay (a real airflow sensor can
  replace it later on the same register).
- [ ] **Fail-ON**: stop the heartbeat → both fans force ON within the watchdog
  timeout. This is the safety backstop — a stalled fan over a wet part is the one
  thing that ruins finish.

**Done:** both fans command on/off and fail ON on a dead heartbeat.

## Phase 5 — Shutter  ⛔ (blocked: end-switches not ordered)

- [ ] Install the baffle panel + actuated shutter + end-switches (DI7 open / DI8 closed).
- [ ] Double-solenoid 5/2 (IO4 open / IO5 close) actuates; holds position de-energised.
- [ ] End-switch feedback → `SH_FB` reads OPEN/CLOSED (not stuck MOVING). **Remove the
  `--bench` shutter stub once real feedback exists.**
- [ ] Interlocks: zone motion gates on shutter OPEN confirmed; spray gates on CLOSED.

**Done:** shutter opens/closes with confirmed feedback and the interlocks hold.

## Phase 6 — Robot process vs real parts  ⬜ (maintenance window)

Comms already proven end-to-end: Dashboard + RTDE receive + control + a jog + a
staged move to `Sand_Base` (2026-07-22). Now the process, on real parts:

- [ ] **Waypoint moves** — `move_to_named` to each process pose (`Sand_Base`,
  `Spray_Base`, `Waypoint_1/2/3`, `Clean_Brush`). Confirm IK + TCP land right, no
  collisions. *(Small moves now — base is in the process joint-range.)*
- [ ] **Sand** on a real part: contact-detect Z, zero FT, force 6 N, one traverse.
  Verify finish (evenly reflective, smooth to 220). Tune force/speed if needed.
- [ ] **Spray**: coat coverage/quality, pass overlap, gun on/off timing, and the
  F1-fan pause during the P3 burst.
- [ ] **Gun-clean**: mount the brush; confirm the HVLP tip cleans at the standoff,
  and confirm/tune the 30 s `brush_duration_s`.

**Done:** sand + spray + gun-clean produce the accepted finish on a real part.

## Phase 7 — Pitch + timing (measure → capture in config)  ⬜

- [ ] **Measure the four station gaps** IN→F1, F1→O, O→F2, F2→OUT. They **must be
  equal** — the one-advance-per-beat schedule depends on it. If not, it's a physical
  fix (or sequenced sub-moves), not a software one. Record into
  `line-config stations.pitch_mm`.
- [ ] Transfer time per zone move (assumed 15 s) → `nominal.transfer_s`.
- [ ] Flash time on the actual water-based lacquer (assumed 180 s) → `flash_seconds`.
- [ ] Spray-burst pause duration (the P3 stretch cost — measure, then minimise).

## Phase 8 — Handoff / train with real parts  ⬜ (needs equal pitch)

- [ ] The two-belt handoff: both belts run together, sensor-terminated. Confirm parts
  cross F1↔O and every zone lands its part correctly (per `process/train.py`).

## Phase 9 — Full interleaved cycle  ⬜

- [ ] `--ur 192.168.1.29 --cc-host 192.168.1.19`: run one lead/trail pair through a
  full period-4. Then a soak of many parts.
- [ ] Confirm effective throughput (target ~6.5 min/part; ~7 at 30 s transfer — both
  above the 250/week requirement).
- [ ] Browser product: same schedule, or dedicated blocks (`pairing.policy`).

## Phase 10 — Faults, safety, rollback  ⬜

- [ ] Watchdog fail-ON (integrated, with everything live).
- [ ] E-stop / protective-stop → recover (`Dashboard.recover_protective_stop`; §7
  posture — zones halt, fans hold, timers keep counting).
- [ ] Sensor-mismatch fault → occupancy scan → confirm-and-resume via the HMI fault
  panel.
- [ ] **Rollback drill**: stop the orchestrator, confirm its Modbus session is closed
  and the heartbeat has stopped, then load + play the legacy `.urp`. Must work any
  morning (`registers.MASTER_HANDOFF`).

---

## Config values to capture (currently assumed / null)

Fill these from the measurements above — nothing should read `assumed`/`null` when
the line goes to production:

- `line-config.yaml`: `stations.pitch_mm` (×4), `nominal.transfer_s`, `flash_seconds`,
  `spray_burst_pause_s`, `clean_gun.duration_s`.
- `io_map.h`: every `TODO(wiring)` — motor↔zone (M0–M3), sensor pins, fan pins,
  shutter solenoid/switch pins.
- Confirm `EYES_ACTIVE_LOW` still matches the eyes as finally installed.

## Tools built for this

- `scripts/sensor_watch.py` — live per-eye state (Phase 1, 3).
- `scripts/line_monitor.py` — read-only view of ClearCore + robot together.
- `python -m finishing_line.api --cc HOST [--bench] [--flash-seconds N]` — orchestrator
  + HMI against the real ClearCore (fake robot); `--bench` stubs the shutter and loosens
  manual-test timing.
- `--ur UR_HOST --cc-host HOST` — full hardware (Phase 9).
- `scripts/rtde_smoke.py` — RTDE receive/control check for the robot.
- Firmware serial banner (115200) — IP + Ethernet link on the ClearCore's USB.
