"""The period-4 interleaved schedule, as declarative tables.

Direct encoding of §3 of docs/finishing-line-state-machine.md. Nothing here
executes; `machine.py` reads these tables.

Two properties make this small:

1.  **Startup needs no special case.** A transition is a list of (from, to)
    station moves. If `from` is empty the move is skipped. Running the steady
    pattern on a half-empty line therefore fills it correctly with no fill
    logic — §4 of the spec.

2.  **Parts are never named in the tables.** The spec describes moves as
    "Lₙ: O→F2", but the part reference is redundant: the occupancy map already
    knows who is at O. Encoding moves as station pairs alone means the tables
    hold for startup, drain, and steady state identically.

THE P3 STRETCH
--------------
P3 is the only beat where a fan pauses: the trail is flashing at F1 while the
lead gets its spray burst at O, and the F1 fan must pause so overspray is not
blown across the shutter plane (§7).

Because flash timers bank fan-on seconds only, the trail's flash 1 cannot
complete inside a nominal 195 s beat — it needs 180 s of *fan-on* time, and the
fan is off for the burst. P3 therefore stretches by the burst duration. This is
intended: the alternative is a part that leaves F1 under-flashed, and §6 makes
"never under-flash" inviolable.

Cost: the period is 780 s + burst, not 780 s. Every second of burst costs
0.5 s/part. `spray_burst_pause_s` is an unmeasured tuned constant — see open
items. The 6.5 min/part target assumes it is zero.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from .model import Direction, FanState, PartRole, ShutterState, Station

Beat = str

BEATS: tuple[Beat, ...] = ("P1", "P2", "P3", "P4")


def next_beat(beat: Beat) -> Beat:
    return BEATS[(BEATS.index(beat) + 1) % len(BEATS)]


@dataclass(frozen=True, slots=True)
class RobotWork:
    """What the robot does at O during a beat.

    `clean_gun` distinguishes the coat-2 beats: instead of sanding, the robot
    cleans the HVLP tip on the brush before spraying coat 2. `nominal_s` is a
    placeholder for throughput math, not a tuned value.
    """

    role: PartRole
    coat: int
    clean_gun: bool
    nominal_s: float


@dataclass(frozen=True, slots=True)
class BeatSpec:
    """One beat of the steady-state schedule (§3 table).

    `f1_fan_pauses_during_spray` is set only on P3, and is the sole reason a
    beat can exceed its nominal duration under normal operation.
    """

    robot: RobotWork
    f1_fan: FanState
    f2_fan: FanState
    shutter: ShutterState
    f1_fan_pauses_during_spray: bool = False


# §3 steady-state schedule. Pair n = (Lₙ, Tₙ).
SCHEDULE: dict[Beat, BeatSpec] = {
    # O: Lₙ sand + coat 1 | F1: Tₙ staged | F2: Tₙ₋₁ flash 2
    "P1": BeatSpec(
        robot=RobotWork(role=PartRole.LEAD, coat=1, clean_gun=False, nominal_s=90.0),
        f1_fan=FanState.OFF,
        f2_fan=FanState.ON,
        shutter=ShutterState.CLOSED,
    ),
    # O: Tₙ sand + coat 1 | F1: empty | F2: Lₙ flash 1
    "P2": BeatSpec(
        robot=RobotWork(role=PartRole.TRAIL, coat=1, clean_gun=False, nominal_s=90.0),
        f1_fan=FanState.OFF,
        f2_fan=FanState.ON,
        shutter=ShutterState.CLOSED,
    ),
    # O: Lₙ gun-clean + coat 2 | F1: Tₙ flash 1 | F2: empty
    # The only beat with a live upstream fan, and so the only one that stretches.
    "P3": BeatSpec(
        robot=RobotWork(role=PartRole.LEAD, coat=2, clean_gun=True, nominal_s=45.0),
        f1_fan=FanState.ON,
        f2_fan=FanState.OFF,
        shutter=ShutterState.CLOSED,
        f1_fan_pauses_during_spray=True,
    ),
    # O: Tₙ gun-clean + coat 2 | F1: Lₙ₊₁ staged | F2: Lₙ flash 2
    "P4": BeatSpec(
        robot=RobotWork(role=PartRole.TRAIL, coat=2, clean_gun=True, nominal_s=45.0),
        f1_fan=FanState.OFF,
        f2_fan=FanState.ON,
        shutter=ShutterState.CLOSED,
    ),
}


@dataclass(frozen=True, slots=True)
class Transition:
    """Zone motions between two beats (§3).

    `moves` are ordered so that a destination is always vacated before it is
    filled. For DOWNSTREAM that means furthest-downstream first; for UPSTREAM,
    furthest-upstream first. The ordering is logical only — the executor groups
    moves by zone and issues one advance per zone, because a single belt
    physically carries every part on it at once.
    """

    direction: Direction
    moves: tuple[tuple[Station, Station], ...]


# §3 "Zone motions between beats". Every transition advances the whole train one
# station in a single direction, so no zone ever opposes its neighbour while a
# part spans the boundary.
TRANSITIONS: dict[Beat, Transition] = {
    # Tₙ₋₁: F2→OUT · Lₙ: O→F2 · Tₙ: F1→O
    "P1": Transition(
        direction=Direction.DOWNSTREAM,
        moves=((Station.F2, Station.OUT), (Station.O, Station.F2), (Station.F1, Station.O)),
    ),
    # Lₙ: F2→O · Tₙ: O→F1   (O vacated before F2 arrives)
    "P2": Transition(
        direction=Direction.UPSTREAM,
        moves=((Station.O, Station.F1), (Station.F2, Station.O)),
    ),
    # Lₙ: O→F2 · Tₙ: F1→O · Lₙ₊₁: IN→F1
    "P3": Transition(
        direction=Direction.DOWNSTREAM,
        moves=((Station.O, Station.F2), (Station.F1, Station.O), (Station.IN, Station.F1)),
    ),
    # Lₙ: F2→OUT · Tₙ: O→F2 · Lₙ₊₁: F1→O · Tₙ₊₁: IN→F1
    "P4": Transition(
        direction=Direction.DOWNSTREAM,
        moves=(
            (Station.F2, Station.OUT),
            (Station.O, Station.F2),
            (Station.F1, Station.O),
            (Station.IN, Station.F1),
        ),
    ),
}

#: Outfeed happens on P1→P2 (previous pair's trail) and P4→P1' (this pair's
#: lead): 2 parts per period, as §3 requires.
OUTFEED_TRANSITIONS: frozenset[Beat] = frozenset({"P1", "P4"})


class Phase(StrEnum):
    """Sub-states within a beat boundary — §3 "Transition choreography".

    The seven numbered steps of the choreography map one-to-one onto these,
    which is why the machine can be a flat dispatch rather than nested logic.
    """

    ROBOT_WORK = "robot_work"        # 1. robot working; ends with ROBOT_CLEAR
    AWAIT_GUARDS = "await_guards"    # 2. flash timers for departing parts (§6)
    SHUTTER_OPEN = "shutter_open"    # 3. open, confirm via sensor
    MOVING = "moving"                # 4. zone moves, confirm presence at destinations
    SHUTTER_CLOSE = "shutter_close"  # 5. close, confirm
    SET_FANS = "set_fans"            # 6. fan states for the new beat
    FAULTED = "faulted"
