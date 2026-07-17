"""Core domain types.

Pure data. No I/O, no clock, no device knowledge — everything here must be
constructible in a test without touching hardware.

See docs/finishing-line-state-machine.md for the process these types encode.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import StrEnum, auto


class Station(StrEnum):
    """Physical positions a part can occupy.

    Ordered from upstream to downstream — `STATION_ORDER` below depends on it.
    """

    INQ = auto()  # infeed queue, holds up to 4 staged parts
    IF = auto()   # infeed flash position (new upstream fan)
    S = auto()    # sand / spray station (UR5e envelope)
    FD = auto()   # downstream flash position (existing fan)
    OUT = auto()  # offload


STATION_ORDER: tuple[Station, ...] = (Station.INQ, Station.IF, Station.S, Station.FD, Station.OUT)

#: Stations that have a fan. A part only accumulates flash time at one of these.
FAN_STATIONS: frozenset[Station] = frozenset({Station.IF, Station.FD})


class Zone(StrEnum):
    """The two conveyor belts.

        ZONE1  INQ <-> IF
        ZONE2  S <-> FD <-> OUT

    NOTE the gap: nothing spans IF <-> S. A part crosses that boundary by
    HANDOFF — both belts run together until a sensor confirms the part is on the
    receiving belt. §3's "no zone ever runs opposite to its neighbour while parts
    span the boundary" is about exactly this.

    Every transition in the steady schedule crosses IF<->S, because S is
    occupied every beat by design. So the two belts always move together, in the
    same direction: they are one logical train with two motors and a baffle
    between them, not two independently schedulable zones.

    PITCH CONSTRAINT. One belt moves everything on it by one distance, and
    sensor-terminating a run measures that distance rather than decoupling it.
    Since ZONE2 carries parts at both S and FD, and ZONE1 carries both the INQ
    part and the part handing off to S, a single synchronised advance is only
    correct for every part if all four station gaps are equal:

        INQ->IF == IF->S == S->FD == FD->OUT

    P2->P3 is the sharpest case: it retreats S->IF while bringing FD->S, and
    both parts start on ZONE2. Whichever part terminates the run, the other only
    lands correctly if IF->S == S->FD.

    UNVERIFIED — see line-config.yaml stations.pitch_mm. If the gaps differ, the
    schedule tables need sequenced moves rather than one advance per transition.
    """

    ZONE1 = auto()
    ZONE2 = auto()


class Product(StrEnum):
    CUBE = auto()
    BROWSER = auto()


class PartRole(StrEnum):
    """Position within a pair. Determines where flash 1 happens.

    LEAD flashes both coats at FD. TRAIL retreats upstream to IF for flash 1,
    which is what keeps S occupied every beat without any part passing another.
    """

    LEAD = auto()
    TRAIL = auto()


class Direction(StrEnum):
    UPSTREAM = auto()
    DOWNSTREAM = auto()


class FanState(StrEnum):
    OFF = auto()
    ON = auto()


class ShutterState(StrEnum):
    OPEN = auto()
    CLOSED = auto()
    MOVING = auto()
    UNKNOWN = auto()


@dataclass(frozen=True, slots=True)
class PartState:
    """Everything the controller knows about one physical part.

    Per §6 of the process spec: validate per-part, never per-beat. Beat counting
    drifts from truth on any fault or manual intervention; these timers make
    recovery unambiguous because they survive faults, restarts, and operator
    intervention independent of where the schedule thinks it is.

    `flash_1_s` and `flash_2_s` accumulate FAN-ON SECONDS ONLY — wall-clock at a
    fan position does not count if that fan is off. This is a deliberate
    decision (see the P3 note in schedule.py): it makes the "never under-flash"
    rule literally true, at the cost of stretching any beat where a fan pauses.
    """

    part_id: str
    product: Product
    role: PartRole
    pair_index: int

    coats_applied: int = 0
    flash_1_s: float = 0.0
    flash_2_s: float = 0.0

    #: Set when the part is sprayed; cleared when its active flash completes.
    #: Drives the "IF fan paused if a wet part sits at IF" interlock (§7).
    is_wet: bool = False

    def active_flash_seconds(self) -> float:
        """Seconds banked on the flash this part is currently waiting out.

        Which flash is 'active' follows from coat count: after coat 1 the part
        is working on flash 1, after coat 2 on flash 2.
        """
        if self.coats_applied <= 1:
            return self.flash_1_s
        return self.flash_2_s

    def with_flash_advanced(self, dt: float) -> PartState:
        """Bank `dt` seconds against whichever flash is active. Caller must have
        already established that the part is at a fan station with the fan ON.
        """
        if self.coats_applied <= 1:
            return replace(self, flash_1_s=self.flash_1_s + dt)
        return replace(self, flash_2_s=self.flash_2_s + dt)


@dataclass(frozen=True, slots=True)
class SensorSnapshot:
    """Observed reality at one instant, as read from ClearCore.

    Presence sensors report occupancy, never identity — reconciling occupancy
    against the controller's expected part map is what detects a sensor mismatch
    fault (§7).
    """

    occupied: frozenset[Station] = frozenset()
    shutter: ShutterState = ShutterState.UNKNOWN
    if_fan: FanState = FanState.OFF
    fd_fan: FanState = FanState.OFF
    robot_clear: bool = False
    gun_on: bool = False
    inq_count: int = 0

    def fan_state(self, station: Station) -> FanState:
        if station is Station.IF:
            return self.if_fan
        if station is Station.FD:
            return self.fd_fan
        raise ValueError(f"{station} has no fan")


@dataclass(frozen=True, slots=True)
class LineState:
    """Complete controller state. Serializable; survives restart.

    `occupancy` is the controller's belief about which part is where. The
    sensors' `occupied` set is the ground truth for *presence*; disagreement
    between the two is a fault, not something to silently reconcile.
    """

    parts: dict[str, PartState] = field(default_factory=dict)
    occupancy: dict[Station, str] = field(default_factory=dict)

    #: Parts staged at INQ, upstream-most last. Populated when an operator
    #: declares a batch at the HMI — presence sensors report counts, never
    #: identity, so identity has to enter the system by declaration.
    inq_queue: tuple[str, ...] = ()

    #: Intent ids emitted and not yet reported complete.
    pending: tuple[str, ...] = ()

    #: Moves commanded but not yet confirmed at their destinations. Occupancy is
    #: only updated once the train advance completes — the controller must not
    #: believe a part has moved merely because it asked (§3, choreography 4).
    in_flight: tuple[tuple[Station, Station], ...] = ()

    beat: str = "P1"
    pair_index: int = 0
    phase: str = "robot_work"

    if_fan: FanState = FanState.OFF
    fd_fan: FanState = FanState.OFF
    shutter: ShutterState = ShutterState.CLOSED

    #: Set while a spray burst is active, so the IF fan can be held off (§7).
    spray_burst_active: bool = False

    fault: str | None = None
    #: The phase the machine was in when the fault hit. Recovery needs it: a
    #: fault before the beat's train move resumes at the work/guard phases; a
    #: fault after it must NOT replay them (the beat counter only advances at
    #: SET_FANS, so post-move state pairs a stale beat with advanced occupancy).
    fault_phase: str | None = None

    def part_at(self, station: Station) -> PartState | None:
        part_id = self.occupancy.get(station)
        return self.parts.get(part_id) if part_id else None

    def station_of(self, part_id: str) -> Station | None:
        for station, occupant in self.occupancy.items():
            if occupant == part_id:
                return station
        return None

    def fan_state(self, station: Station) -> FanState:
        if station is Station.IF:
            return self.if_fan
        if station is Station.FD:
            return self.fd_fan
        raise ValueError(f"{station} has no fan")
