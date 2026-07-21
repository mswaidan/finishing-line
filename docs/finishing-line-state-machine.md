# Finishing Line State Machine — Interleaved Two-Fan Schedule

Rev 0 — 2026-07-16. Generalizes the hand-drawn 5-step table into a repeating schedule with explicit zone motions, fan states, shutter states, robot actions, and interlocks.

## 1. Stations and hardware

| ID | Station | Notes |
|----|---------|-------|
| IN | Infeed queue | Existing infeed conveyor, holds up to 4 staged cubes |
| F1 | Infeed flash position | New upstream fan. Doubles as staging slot when fan is off |
| O | Sand / spray station | UR5e work envelope. Pneumatic sander + HVLP gun |
| F2 | Downstream fan position | Existing fan |
| OUT | Outfeed | Offload at the fan end (existing) |

**Zones:** Z1 spans IN ↔ F1; Z2 spans O ↔ F2 ↔ OUT. The F1 ↔ O boundary belongs to neither zone — it is the handoff gap crossed by both belts together (§3). Both reversible via robot command.

**Shutter:** The baffle panel sits in the plane between F1 and O. The window is an actuated shutter (assumed pneumatic slide gate) with states OPEN / CLOSED and position feedback. *If the window is left as a fixed opening, treat the shutter column below as always-OPEN — contamination control then depends entirely on fan ducting and panel geometry.*

## 2. Part roles

Parts run in **pairs**: a **lead (L)** and a **trail (T)**. Their paths differ only in where the first flash happens:

- **Lead:** IN → F1 (stage) → O (coat 1) → F2 (flash 1) → O (coat 2) → F2 (flash 2) → OUT
- **Trail:** IN → F1 (stage) → O (coat 1) → **F1 (flash 1)** → O (coat 2) → F2 (flash 2) → OUT

The trail's retreat to the upstream fan is what keeps the spray station occupied every beat without any part ever passing another. Part order on the conveyor never changes.

## 3. Steady-state schedule (period = 4 beats, completes 2 parts)

Pair *n* = (Lₙ, Tₙ). Previous pair = (Lₙ₋₁, Tₙ₋₁). Next pair = (Lₙ₊₁, Tₙ₊₁).

| Beat | O (robot action) | F1 | F2 | F1 fan | F2 fan | Shutter |
|------|------------------|----|----|--------|--------|---------|
| **P1** | Lₙ — sand + coat 1 | Tₙ staged | Tₙ₋₁ flash 2 | OFF | ON | CLOSED |
| **P2** | Tₙ — sand + coat 1 | empty | Lₙ flash 1 | OFF | ON | CLOSED |
| **P3** | Lₙ — denib + coat 2 | Tₙ flash 1 | empty | ON (pause during spray burst) | OFF | CLOSED |
| **P4** | Tₙ — denib + coat 2 | Lₙ₊₁ staged | Lₙ flash 2 | OFF | ON | CLOSED |

### Zone motions between beats (shutter OPEN for every transition)

| Transition | Direction | Moves |
|-----------|-----------|-------|
| P1 → P2 | ALL DOWNSTREAM | Tₙ₋₁: F2→OUT · Lₙ: O→F2 · Tₙ: F1→O |
| P2 → P3 | ALL UPSTREAM | Lₙ: F2→O · Tₙ: O→F1 |
| P3 → P4 | ALL DOWNSTREAM | Lₙ: O→F2 · Tₙ: F1→O · Lₙ₊₁: IN→F1 |
| P4 → P1' | ALL DOWNSTREAM | Lₙ: F2→OUT · Tₙ: O→F2 · Lₙ₊₁: F1→O · Tₙ₊₁: IN→F1 |

Every transition moves the whole train one station in a single direction — no zone ever runs opposite to its neighbor while parts span the boundary. Outfeed events occur on P1→P2 (trail of previous pair) and P4→P1' (lead of current pair): 2 parts per period.

### Transition choreography (every beat boundary)

1. Robot completes work, gun off, retracts to safe pose → sets ROBOT_CLEAR
2. Verify flash timer satisfied for any part about to leave a fan position (§6)
3. Shutter OPEN (confirm via sensor)
4. Command zone moves per table; confirm part-presence sensors at destinations
5. Shutter CLOSED (confirm)
6. Set fan states for new beat
7. Robot begins work; beat timer starts

## 4. Startup fill

From an empty line, run the steady-state pattern with the "previous pair" slots empty:

| Beat | O | F1 | F2 | Notes |
|------|---|----|----|----|
| Fill 0 | — | L₁ staged | — | L₁ loads IN→F1 |
| Fill 1 (=P1) | L₁ coat 1 | T₁ staged | — | F2 fan OFF (unoccupied) |
| Fill 2 (=P2) | T₁ coat 1 | empty | L₁ flash 1 | Steady state from here |

No special-case logic needed — startup is the steady pattern with unoccupied slots and their fans off.

## 5. End-of-batch drain

**Even part count:** after the final pair's P4, run two more transitions with O idle: P4→P1' (Lₙ out, Tₙ to F2, fan ON 180 s), then Tₙ → OUT. 

**Odd part count (lone lead, no trail):** the lone part runs the lead path with O idle on trail beats: coat 1 → F2 flash 1 → coat 2 → F2 flash 2 → OUT (5 beats). F1 fan never runs.

## 6. Timing and state validation

Beat duration = **flash time (180 s) + transfer (~15 s) ≈ 195 s**. Period ≈ 13 min for 2 parts → **~6.5 min/part effective, ~74/day at 8 h, ~370/week.**

Robot work per beat: coat-1 beats ≈ 90 s (sand + spray), coat-2 beats ≈ 45 s (denib + spray). The robot is idle 55–75% of each beat — **flash time paces the line**, so any future flash reduction (heated air) shortens the beat directly.

**Validate per-part, not per-beat.** The controller tracks each part's state: {coats applied, flash-1 seconds accumulated, flash-2 seconds accumulated}. Guard conditions:
- Part may leave a fan position only if the active flash timer ≥ 180 s
- Part may receive coat 2 only if flash-1 timer complete
- Part may outfeed only if flash-2 timer complete

Beat counting alone will drift from truth on any fault or manual intervention; per-part timers make recovery unambiguous.

## 7. Interlocks and faults

**Zone motion permitted only when:** ROBOT_CLEAR set · gun off · shutter OPEN confirmed · destination slot empty (presence sensors).

**Spray permitted only when:** shutter CLOSED confirmed · part present and located at O · F1 fan paused if a wet part occupies F1.

**Sensor mismatch** (part expected/found disagreement at F1, O, or F2): halt zones, fans remain ON (keeps flashing parts drying), alarm. Recovery = occupancy scan → operator confirms part identities → resume from reconstructed state using per-part timers.

**UR5e protective stop / E-stop:** zones halt immediately; fans hold state; shutter holds state. Flash timers keep counting (drying continues) — parts may over-flash safely, never under-flash.

**Overspray/dust control notes:** F1 fan ducted to blow toward the infeed end, away from the shutter plane. Kraft paper facing on the spray side of the panel, replaced weekly. Shutter closed during all sand and spray operations is the primary barrier; the fan pause during spray bursts is the backstop for the P3 beat, when a wet part sits at F1 during a spray.

## 8. Open items

- Confirm transfer time per zone move (assumed 15 s) — measure with a stopwatch on the current line
- Decide shutter actuator (pneumatic slide gate assumed; shop air already at the station)
- Confirm whether coat 2 gets a denib pass and its duration
- Browser parts: verify they fit the same station geometry and schedule, or run them in dedicated blocks
- Rework loop stays offline at QC (unchanged)
