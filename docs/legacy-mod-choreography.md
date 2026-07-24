# Legacy-mod choreography — design notes from the 2026-07-22 night session

The near-term throughput plan: run the interleaved period-4 schedule on the
EXISTING line — unmodified legacy ClearCore firmware (`modbustest.ino`), single
main belt, PC as Modbus master (`devices/legacy_clearcore.py`), robot via RTDE.
No Polyscope programming, no new firmware, rollback = load the untouched `.urp`.

Validated on the production line (after hours, real cubes, feed conveyor live):
entries, retreats, returns, exits — all sensor-referenced, ZERO tuned distances.
Driver: `devices/legacy_clearcore.py`. Harness: `scripts/choreography_poc.py`.

## The direct-entry scheme (final form)

Two sensors run everything: **WORK_AT_ZERO** (WZ, at O) and the **STAGING
eye** (discrete 7, ~450 mm past the feed/main-belt junction).

> 2026-07-25 sensor shuffle: during the POC the "eye" was the ONLOAD sensor
> physically relocated downstream. ONLOAD is now back AT the junction (its
> original legacy position — currently unwatched by the choreography), and a
> dedicated STAGING eye (the legacy v1.1 DI-6 sensor, discrete 7) took over
> the downstream park position. Same geometry, different address.

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

Measured geometry (this line): junction->WZ ≈ 1075 mm; staging eye at
junction+450 (2026-07-25 mount — was ~+400 during the POC); steady spacing ≈
eye->WZ ≈ 625 mm (max safe ≈ 713 = 1075 − part width 362).

## Staging: just-in-time + sensor-stopped nudge

The last ~centimeters of the feed-only crossing stall intermittently (part's
weight leaves the feed; static main-belt friction). Resolution — in the driver
as `stage_next()`:

- **Phase A, feed-only** (spacing-neutral) toward the eye. Usually completes.
- **Phase B, the NUDGE** (only on stall): main belt + feed together,
  sensor-stopped on STAGING HI. The mostly-aboard part grips the moving belt
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
+ stop latency). The 2026-07-25 mount at junction+450 gives 88 mm of headroom —
right at the rule; if reported nudges ever exceed ~80 mm, look at the junction
mechanically or add eye margin.

## The junction rule + skip discipline (2026-07-25, validated by handoff_test)

- **Z1 (feed) is governed ONLY by the junction chain on ONLOAD**: HI (nose)
  -> LO (tail clears) -> HI (next follower's nose) => cut, so the follower
  never boards uninvited. If a part rests on the eye at start, the chain
  begins at LO -> HI. **No timeout**: an empty queue keeps Z1 feeding until a
  part shows — this IS the continuous-intake behavior; there is no queue
  sensor and none is needed. The watch persists across driver calls
  (`feed_tick()`, one nonblocking poll per sequencer step).
- **Z2 owns arrivals**: staging parks on STAGING HI (`stop_on_staging`),
  O-arrivals on the WORK_AT_ZERO chains. A live Z1 watch is INHERITED by
  transitions — the feed keeps hunting while Z2 moves and the transition
  loop polls the chain, cutting at the follower's ONLOAD edge. Safe because
  a part parked nose-at-the-eye does not drag under a moving Z2 (phases
  2/5); the disaster mode is only a live coil with a CANCELLED watch.
  Blind shuffles (no chain polling) suspend the feed and resume it after.
- **Entries only from a confirmed staged position.** Not staged at an entry
  beat = SKIP the beat (hole in the pattern), part stays on Z1; next
  beat-end staging retries. The feed-assist boarding fallback is gone — it
  would inflate spacing by ~450 mm and drive the retreat into the junction.
- Empty-line intake = the same primitives: both belts to STAGING, then a
  normal staged entry to O.

(Deferred alternative if nudge grip proves inconsistent: a third eye mid-belt
that triggers feed-start during the entry move, timed so the follower lands at
the eye as the belt stops — sensor-anchored but with open-loop residue. The
nudge is preferred: fully sensor-stopped, zero new parts.)

## Process decisions on this route (operator, 2026-07-26)

These deliberately supersede the rewrite spec's blanket rules for the
legacy-mod route — do not "fix" them back:

- **Flash-2 does not gate flow.** Only flash-1 limits the schedule (the P2
  retreat and P3 return gates). A part exits at the entry beats regardless
  of flash-2 progress and finishes drying on the gravity conveyor past OUT.
  CLAUDE.md's "never under-flash" applies to flash-1 here; flash-2 dries
  off-line by design.
- **Every coat is preceded by sanding.** Coat 2 gets a post-flash-1 scuff
  sand with the same passes as coat 1 (was: coat-2 beats sprayed without
  sanding).
- **The gun-tip brush clean (5 s, line-config override) runs during the
  flash wait**, right after the coat-2 spray + safe pose — off the critical
  path. Pre-spray it measurably lengthened the beat.

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
