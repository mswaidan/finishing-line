"""The period-4 interleaved schedule, as declarative tables.

Direct encoding of §3 of docs/finishing-line-state-machine.md. Nothing here
executes; `machine.py` reads these tables.

Two properties make this small:

1.  **Startup needs no special case.** A transition is a list of (from, to)
    station moves. If `from` is empty the move is skipped. Running the steady
    pattern on a half-empty line therefore fills it correctly with no fill
    logic — §4 of the spec.

2.  **Parts are never named in the tables.** The spec describes moves as
    "Lₙ: S→FD", but the part reference is redundant: the occupancy map already
    knows who is at S. Encoding moves as station pairs alone means the tables
    hold for startup, drain, and steady state identically.

THE P3 STRETCH
--------------
P3 is the only beat where a fan pauses: the trail is flashing at IF while the
lead gets its spray burst at S, and the IF fan must pause so overspray is not
blown across the shutter plane (§7).

Because flash timers bank fan-on seconds only, the trail's flash 1 cannot
complete inside a nominal 195 s beat — it needs 180 s of *fan-on* time, and the
fan is off for the burst. P3 therefore stretches by the burst duration. This is
intended: the alternative is a part that leaves IF under-flashed, and §6 makes
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
    """What the robot does at S during a beat.

    `denib` distinguishes the coat-2 beats. Whether coat 2 actually gets a denib
    pass, and how long it takes, is an OPEN ITEM (§8) — the duration below is a
    placeholder and must not be treated as tuned.
    """

    role: PartRole
    coat: int
    denib: bool
    nominal_s: float


@dataclass(frozen=True, slots=True)
class BeatSpec:
    """One beat of the steady-state schedule (§3 table).

    `if_fan_pauses_during_spray` is set only on P3, and is the sole reason a
    beat can exceed its nominal duration under normal operation.
    """

    robot: RobotWork
    if_fan: FanState
    fd_fan: FanState
    shutter: ShutterState
    if_fan_pauses_during_spray: bool = False


# §3 steady-state schedule. Pair n = (Lₙ, Tₙ).
SCHEDULE: dict[Beat, BeatSpec] = {
    # S: Lₙ sand + coat 1 | IF: Tₙ staged | FD: Tₙ₋₁ flash 2
    "P1": BeatSpec(
        robot=RobotWork(role=PartRole.LEAD, coat=1, denib=False, nominal_s=90.0),
        if_fan=FanState.OFF,
        fd_fan=FanState.ON,
        shutter=ShutterState.CLOSED,
    ),
    # S: Tₙ sand + coat 1 | IF: empty | FD: Lₙ flash 1
    "P2": BeatSpec(
        robot=RobotWork(role=PartRole.TRAIL, coat=1, denib=False, nominal_s=90.0),
        if_fan=FanState.OFF,
        fd_fan=FanState.ON,
        shutter=ShutterState.CLOSED,
    ),
    # S: Lₙ denib + coat 2 | IF: Tₙ flash 1 | FD: empty
    # The only beat with a live upstream fan, and so the only one that stretches.
    "P3": BeatSpec(
        robot=RobotWork(role=PartRole.LEAD, coat=2, denib=True, nominal_s=45.0),
        if_fan=FanState.ON,
        fd_fan=FanState.OFF,
        shutter=ShutterState.CLOSED,
        if_fan_pauses_during_spray=True,
    ),
    # S: Tₙ denib + coat 2 | IF: Lₙ₊₁ staged | FD: Lₙ flash 2
    "P4": BeatSpec(
        robot=RobotWork(role=PartRole.TRAIL, coat=2, denib=True, nominal_s=45.0),
        if_fan=FanState.OFF,
        fd_fan=FanState.ON,
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
    # Tₙ₋₁: FD→OUT · Lₙ: S→FD · Tₙ: IF→S
    "P1": Transition(
        direction=Direction.DOWNSTREAM,
        moves=((Station.FD, Station.OUT), (Station.S, Station.FD), (Station.IF, Station.S)),
    ),
    # Lₙ: FD→S · Tₙ: S→IF   (S vacated before FD arrives)
    "P2": Transition(
        direction=Direction.UPSTREAM,
        moves=((Station.S, Station.IF), (Station.FD, Station.S)),
    ),
    # Lₙ: S→FD · Tₙ: IF→S · Lₙ₊₁: INQ→IF
    "P3": Transition(
        direction=Direction.DOWNSTREAM,
        moves=((Station.S, Station.FD), (Station.IF, Station.S), (Station.INQ, Station.IF)),
    ),
    # Lₙ: FD→OUT · Tₙ: S→FD · Lₙ₊₁: IF→S · Tₙ₊₁: INQ→IF
    "P4": Transition(
        direction=Direction.DOWNSTREAM,
        moves=(
            (Station.FD, Station.OUT),
            (Station.S, Station.FD),
            (Station.IF, Station.S),
            (Station.INQ, Station.IF),
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
