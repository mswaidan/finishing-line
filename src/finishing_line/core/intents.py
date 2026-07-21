"""Intents — what the core asks the outside world to do.

The core package performs no I/O (CLAUDE.md, "Architecture"). Instead `step()`
returns intents; the process layer executes them against real devices and feeds
completions back on the next step.

This is what keeps the state machine simulatable: the simulator satisfies the
same intents with a fake clock and no hardware, so the exact code that runs the
line runs in tests.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import count

from .model import Direction, FanState, ShutterState, Station

_ids = count(1)


def _next_id() -> str:
    return f"i{next(_ids)}"


@dataclass(frozen=True, slots=True)
class Intent:
    """Base. `intent_id` correlates the completion that comes back later."""

    intent_id: str = field(default_factory=_next_id, kw_only=True)


@dataclass(frozen=True, slots=True)
class SandPart(Intent):
    """Sand the face of the part at O.

    Composite: the robot force-holds while the conveyor traverses the part
    beneath the tool, then the robot steps the other axis. Neither device can do
    this alone, which is why it is an intent for the process layer rather than a
    URScript primitive. See process/sander.py.
    """

    part_id: str


@dataclass(frozen=True, slots=True)
class SprayPart(Intent):
    """Apply one coat to the part at O.

    The executor must raise `spray_burst_active` for the duration, so the F1 fan
    interlock (§7) can pause the upstream fan while the gun is live.
    """

    part_id: str
    coat: int


@dataclass(frozen=True, slots=True)
class DenibPart(Intent):
    """Denib pass before coat 2.

    OPEN ITEM (§8): not yet confirmed that this pass exists or how long it takes.
    """

    part_id: str


@dataclass(frozen=True, slots=True)
class MoveToSafePose(Intent):
    """Retract the robot and assert ROBOT_CLEAR — choreography step 1."""


@dataclass(frozen=True, slots=True)
class SetShutter(Intent):
    """Drive the shutter and confirm via its feedback sensor.

    Completion means *confirmed by sensor*, never merely commanded — every zone
    motion is gated on shutter position (§7).
    """

    target: ShutterState


@dataclass(frozen=True, slots=True)
class AdvanceTrain(Intent):
    """Advance the train one station in a single direction.

    `moves` is logical, ordered vacate-before-fill. The executor groups them by
    zone and issues one advance per zone, because one belt carries every part on
    it simultaneously. Moves whose source station is empty are dropped by the
    core before the intent is emitted — that is what makes startup fill work
    with no special-case code (§4).
    """

    direction: Direction
    moves: tuple[tuple[Station, Station], ...]


@dataclass(frozen=True, slots=True)
class SetFan(Intent):
    station: Station
    state: FanState


@dataclass(frozen=True, slots=True)
class HaltZones(Intent):
    """Stop all conveyor motion. Fans keep running — parts mid-flash must keep
    drying through a fault (§7).
    """

    reason: str
