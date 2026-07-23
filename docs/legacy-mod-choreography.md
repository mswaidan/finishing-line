# Legacy-mod choreography — design notes from the 2026-07-22 night session

The near-term throughput plan: run the interleaved period-4 schedule on the
EXISTING line — unmodified legacy ClearCore firmware (`modbustest.ino`), single
main belt, PC as Modbus master (`devices/legacy_clearcore.py`), robot via RTDE.
No Polyscope programming, no new firmware, rollback = load the untouched `.urp`.

Validated on the production line (after hours, real cubes, feed conveyor live):
entries, retreats, returns, exits — all sensor-referenced, ZERO tuned distances.
Driver: `devices/legacy_clearcore.py`. Harness: `scripts/choreography_poc.py`.

## The direct-entry scheme (final form)

Two sensors run everything: **WORK_AT_ZERO** (WZ, at O) and **ONLOAD** (the
"eye", relocated ~one part-width past the feed/main-belt junction).

```
load    : C1 queue->O   (continuous belt + feed boarding-assist, WZ stop)
P1->P2  : C2 ENTERS queue->O            · C1 carried out to F2
P2->P3  : C1 retreats F2->O (pass + re-approach) · C2 carried to F1
P3->P4  : C2 returns F1->O              · C1 carried to F2
P4->P1' : C3 ENTERS queue->O            · C2 to F2 · C1 to OUT
```

- Every arrival at O is sensor-stopped (continuous mode — the 16-bit DISTANCE
  register caps distance moves at 2184 mm, and the legacy load used continuous
  for exactly this reason).
- Downstream arrivals stop on WZ rising; the F2->O retreat uses the legacy
  return-to-zero (script:3191-3206): pass WZ (HI->LO), then re-approach
  downstream until HI — every part rests at O approached from the same side.
- Departure detection: when O starts occupied, the eye chain is HI->LO first
  (the occupant passing over), then the arrival edge(s).
- F1 and F2 are NOT commanded positions — they are wherever parts rest.
  Fans mount at the observed rest positions. There is no pitch parameter.

## The conservation law (why the geometry is what it is)

On one belt:

1. **Pair spacing = the lead's belt travel between its WZ-arrival and the
   trail's WZ-arrival** — i.e. the trail's journey from wherever it started.
2. **The retreat rigidly rewinds the trail to its pre-entry position.** So the
   trail's F1 rest position IS its staging point, always.
3. Therefore the staged part must sit **fully on the main belt** (else the
   retreat drags it back into the junction — observed stall), and **only
   FEED-ONLY motion advances the follower without inflating the spacing**.
   Any main-belt motion during staging adds 1:1 to spacing and deepens the
   retreat correspondingly. Full-belt staging => spacing ≈ queue->WZ ≈ 1115 mm
   => retreat ~400 mm into the junction. Not fixable by choreography.

Measured geometry (this line): junction->WZ ≈ 1075 mm; eye at ~junction+400;
steady spacing ≈ eye->WZ ≈ 700 mm (max safe ≈ 713 = 1075 − part width 362).

## Staging: just-in-time + sensor-stopped nudge

The last ~centimeters of the feed-only crossing stall intermittently (part's
weight leaves the feed; static main-belt friction). Resolution — in the driver
as `stage_next()`:

- **Phase A, feed-only** (spacing-neutral) toward the eye. Usually completes.
- **Phase B, the NUDGE** (only on stall): main belt + feed together,
  sensor-stopped on ONLOAD HI. The mostly-aboard part grips the moving belt
  (consistent handoff — moving-belt crossings never stalled all night) and
  parks at the eye. Downstream parts slide by the stall shortfall **δ** only.

**Staging runs just-in-time, immediately before each entry (end of beats
P1/P4)** — at that moment the O part's work is done and it departs downstream
on the very next move, and the F2 part is about to exit, so the δ-slide is
harmless. Consequences, all by construction:

- The junction and eye are CLEAR during every retreat (no cross-beat staging).
- The last queued part stages as reliably as the first (nudge needs no queue
  pressure).
- δ self-measures (nudge seconds × belt speed) and is reported every cycle;
  spacing becomes ~713+δ and the retreat lands δ short of the eye.

**Eye placement rule: ≥ one part-width + ~90 mm past the junction** (δ headroom
+ stop latency). If reported nudges ever exceed ~150 mm, look at the junction
mechanically or add eye margin.

(Deferred alternative if nudge grip proves inconsistent: a third eye mid-belt
that triggers feed-start during the entry move, timed so the follower lands at
the eye as the belt stops — sensor-anchored but with open-loop residue. The
nudge is preferred: fully sensor-stopped, zero new parts.)

## Protocol facts (hard-won, don't rediscover)

- Legacy moves fire on REQUEST_ID **change**; seed the counter from echo 206 on
  connect or a second client run repeats the old id and moves nothing.
- 16-bit DISTANCE register: max 2184 mm per distance move; use continuous.
- Velocity/accel default to 0 after ClearCore boot — push params (1600/16000)
  before any motion or nothing moves, silently.
- The feed conveyor is ONE-DIRECTIONAL (coil on/off; firmware hard-codes
  positive velocity). It also shares the VELOCITY register with the main belt.
- Modbus-poll stop latency ≈ 2–5 mm at 53 mm/s — same class as the legacy
  program's own polling. Belt speed = 1600 sps / 29.996 steps-per-mm ≈ 53 mm/s.
- All failure paths idle the belt (continuous mode is unbounded otherwise).

## Open items for the next session

1. Verify nudge grip consistency on the real junction; collect δ stats.
2. Confirm eye margin (move it downstream a touch if δ demands).
3. Chalk the observed F1/F2 rest positions -> F1 fan mounting spec; check
   OUT clearance at the belt end.
4. Fold the robot in: sand/spray/gun-clean at O between transitions (URRobot
   exists; needs a `--legacy` orchestrator mode marrying it to
   LegacyClearCoreClient), F1 fan on a spare robot DO with the P3 burst pause.
5. Flash timers + beat pacing (the POC is keypress-paced; production needs the
   per-part flash discipline from the core state machine).
6. Odd/even drain and browser-product runs.
