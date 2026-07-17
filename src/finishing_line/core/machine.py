"""The state machine.

Pure: `step()` takes state plus observations and returns new state plus intents.
No clock, no sockets, no sleeps — time arrives as `dt`. That is what lets the
simulator run a full week of production in milliseconds against the same code
that runs the line.

BEAT ADVANCE IS EVENT-DRIVEN
----------------------------
A beat ends when every guard for its transition passes — not when a 195 s timer
expires. §6 of the spec requires validating per-part timers rather than beat
counts, and a fixed beat clock would contradict that the moment anything ran
long: it would either fault on a schedule that is merely slow, or move a part
that is merely late.

Consequence: throughput is an *outcome*, not an input. The 195 s beat and
6.5 min/part in the spec are predictions to be checked against reality, and the
controller will not force them. Any beat can stretch; P3 routinely does.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from ..config.loader import ProcessConfig
from . import guards, timers
from .intents import (
    AdvanceTrain,
    DenibPart,
    HaltZones,
    Intent,
    MoveToSafePose,
    SandPart,
    SetFan,
    SetShutter,
    SprayPart,
)
from .model import (
    FanState,
    LineState,
    PartRole,
    SensorSnapshot,
    ShutterState,
    Station,
)
from .schedule import SCHEDULE, TRANSITIONS, Phase, next_beat


@dataclass(frozen=True, slots=True)
class Inputs:
    """Everything arriving from outside the core on one step."""

    dt: float
    sensors: SensorSnapshot
    completed: frozenset[str] = frozenset()
    fault: str | None = None


@dataclass(frozen=True, slots=True)
class StepResult:
    state: LineState
    intents: tuple[Intent, ...] = ()
    #: Why the machine is not advancing, if it is not. Surfaced to the HMI —
    #: a line sitting still with no stated reason is how interlocks get bypassed.
    blocked_by: str | None = None


def step(state: LineState, inputs: Inputs, cfg: ProcessConfig) -> StepResult:
    """Advance the machine by `inputs.dt` seconds.

    Flash timers advance on every step regardless of phase — including while
    faulted. Parts keep drying through a protective stop (§7); pausing the timer
    would make a part that flashed for 400 s look under-flashed and force a
    needless rework.
    """
    state = replace(state, parts=timers.advance_flash_timers(state, inputs.dt))
    state = replace(state, pending=tuple(p for p in state.pending if p not in inputs.completed))

    if inputs.fault is not None:
        return _enter_fault(state, inputs.fault)

    if state.phase == Phase.FAULTED:
        return StepResult(state, blocked_by=state.fault)

    # Parts are physically between stations during a train advance, so the
    # sensors legitimately disagree with the controller's map. Checking here
    # would fault on every transfer.
    if state.phase is not Phase.MOVING:
        mismatch = guards.occupancy_mismatch(state, inputs.sensors)
        if mismatch is not None:
            return _enter_fault(state, mismatch)

    if state.pending:
        return StepResult(state, blocked_by=None)

    return _dispatch(state, inputs, cfg)


def resume(
    state: LineState,
    sensors: SensorSnapshot,
    *,
    confirmed_occupancy: dict[Station, str] | None = None,
    beat: str | None = None,
) -> StepResult:
    """Operator-acknowledged recovery from FAULTED — the §7 flow.

    The operator performs an occupancy scan, confirms which part is where, and
    the machine resumes from the per-part timers. This is exactly why §6 keeps
    truth per-part rather than per-beat: the timers survived the fault, so the
    only thing recovery has to re-establish is *where everything is*.

    `confirmed_occupancy` replaces the controller's belief wholesale when given —
    sensors report presence, never identity, so identity can only re-enter by
    operator declaration. The result is still validated against the presence
    sensors; a resume that contradicts them returns to FAULTED immediately
    rather than resuming on a lie.

    `beat` overrides the beat when the operator's reconstruction implies a
    different point in the cycle. If it is wrong, the role-mismatch check in
    robot_work faults again with a message naming the expected role — safe, and
    it tells the operator which way to correct.

    Resumes at ROBOT_WORK. Safe because work emission is idempotent: a part
    already carrying this beat's coat is not re-sprayed (see _robot_work). A
    part that was mid-spray when the fault hit is recorded as coated even though
    the coat may be partial — the controller cannot know how much paint landed.
    Pulling it for QC is the operator's call, per the offline rework loop.
    """
    if state.phase is not Phase.FAULTED:
        return StepResult(state, blocked_by="resume called but machine is not faulted")

    occupancy = dict(confirmed_occupancy) if confirmed_occupancy is not None else state.occupancy
    candidate = replace(
        state,
        occupancy=occupancy,
        beat=beat or state.beat,
        phase=Phase.ROBOT_WORK,
        fault=None,
        pending=(),
        in_flight=(),
    )

    mismatch = guards.occupancy_mismatch(candidate, sensors)
    if mismatch is not None:
        return StepResult(
            replace(state, occupancy=occupancy),
            blocked_by=f"resume rejected: {mismatch}",
        )
    return StepResult(candidate)


def _enter_fault(state: LineState, reason: str) -> StepResult:
    """Halt zones; leave fans running so parts mid-flash keep drying (§7)."""
    if state.phase == Phase.FAULTED:
        return StepResult(state, blocked_by=state.fault)
    state = replace(state, phase=Phase.FAULTED, fault=reason, pending=())
    return StepResult(state, intents=(HaltZones(reason=reason),), blocked_by=reason)


def _dispatch(state: LineState, inputs: Inputs, cfg: ProcessConfig) -> StepResult:
    match state.phase:
        case Phase.ROBOT_WORK:
            return _robot_work(state, inputs, cfg)
        case Phase.AWAIT_GUARDS:
            return _await_guards(state, inputs, cfg)
        case Phase.SHUTTER_OPEN:
            return _emit(state, Phase.MOVING, SetShutter(target=ShutterState.OPEN))
        case Phase.MOVING:
            return _moving(state, inputs, cfg)
        case Phase.SHUTTER_CLOSE:
            return _emit(state, Phase.SET_FANS, SetShutter(target=ShutterState.CLOSED))
        case Phase.SET_FANS:
            return _set_fans(state)
        case _:
            return _enter_fault(state, f"unreachable phase {state.phase}")


def _emit(state: LineState, next_phase: Phase, *intents: Intent) -> StepResult:
    state = replace(
        state,
        phase=next_phase,
        pending=state.pending + tuple(i.intent_id for i in intents),
    )
    return StepResult(state, intents=intents)


def _robot_work(state: LineState, inputs: Inputs, cfg: ProcessConfig) -> StepResult:
    """Choreography step 1: robot works, then retracts and sets ROBOT_CLEAR.

    An idle beat (nothing at S) is normal, not exceptional — it is every startup
    fill beat and every drain beat. Skipping straight to the guards is what lets
    §4 and §5 reuse the steady pattern with no special-case code.
    """
    spec = SCHEDULE[state.beat]
    part = state.part_at(Station.S)

    if part is None:
        return StepResult(replace(state, phase=Phase.AWAIT_GUARDS))

    if part.role is not spec.robot.role:
        return _enter_fault(
            state,
            f"beat {state.beat} expects a {spec.robot.role} at S, found {part.role} "
            f"({part.part_id})",
        )

    # Idempotence: a part already carrying this beat's coat gets no second one.
    # This is what makes resume-at-ROBOT_WORK safe after a fault — the machine
    # re-enters the beat, sees the work is done, and goes straight to the
    # guards instead of double-coating the part.
    if part.coats_applied >= spec.robot.coat:
        return StepResult(replace(state, phase=Phase.AWAIT_GUARDS))

    blocked = guards.spray_station_not_ready(state, inputs.sensors, cfg)
    if blocked is not None:
        return StepResult(state, blocked_by=blocked)

    prep: Intent = (
        DenibPart(part_id=part.part_id) if spec.robot.denib else SandPart(part_id=part.part_id)
    )
    work: tuple[Intent, ...] = (prep,)

    # §7: a wet part at IF may not be blown on while the gun is live. The core
    # plans the pause around the burst rather than around the whole beat, so the
    # trail loses only the spray's worth of fan-on seconds — every second here
    # is a second P3 stretches, and half a second per part.
    #
    # The executor runs a batch in order, which is what makes this correct: the
    # fan is off for exactly the span between these two intents.
    part_at_if = state.part_at(Station.IF)
    pausing = spec.if_fan is FanState.ON and part_at_if is not None and part_at_if.is_wet
    if pausing:
        work += (SetFan(station=Station.IF, state=FanState.OFF),)

    work += (SprayPart(part_id=part.part_id, coat=spec.robot.coat),)

    if pausing:
        work += (SetFan(station=Station.IF, state=FanState.ON),)

    work += (MoveToSafePose(),)

    updated = replace(
        state.parts[part.part_id],
        coats_applied=spec.robot.coat,
        is_wet=True,
    )
    state = replace(state, parts={**state.parts, part.part_id: updated})
    return _emit(state, Phase.AWAIT_GUARDS, *work)


def _await_guards(state: LineState, inputs: Inputs, cfg: ProcessConfig) -> StepResult:
    """Choreography step 2: hold until every departing part has banked its flash.

    This is where the line spends most of a beat, and where P3 stretches past
    nominal while the trail waits out the fan-on seconds it lost to the burst.
    """
    moves = _live_moves(state)
    blocked = guards.departure_blocked(state, inputs.sensors, cfg, moves)
    if blocked is not None:
        return StepResult(state, blocked_by=blocked)
    return StepResult(replace(state, phase=Phase.SHUTTER_OPEN))


def _moving(state: LineState, inputs: Inputs, cfg: ProcessConfig) -> StepResult:
    """Choreography step 4: advance the train, confirm arrival, then believe it.

    Two passes. First: check the guards with the shutter now confirmed open, emit
    the advance, and record the moves as in flight. Second (once the advance
    completes): reconcile against the destination sensors, and only then update
    occupancy.

    Occupancy must not change at command time. A part that was commanded to move
    and did not is exactly the case the sensor-mismatch fault exists to catch,
    and updating the map on command would hide it.
    """
    if state.in_flight:
        arrived = {dst for _src, dst in state.in_flight if dst is not Station.OUT}
        missing = arrived - inputs.sensors.occupied
        if missing:
            return _enter_fault(
                state,
                f"train advance did not confirm at {', '.join(sorted(str(m) for m in missing))}",
            )
        state = _apply_moves(state, state.in_flight)
        return StepResult(replace(state, in_flight=(), phase=Phase.SHUTTER_CLOSE))

    moves = _live_moves(state)
    if not moves:
        return StepResult(replace(state, phase=Phase.SHUTTER_CLOSE))

    blocked = guards.zone_motion_blocked(state, inputs.sensors, cfg, moves)
    if blocked is not None:
        return StepResult(state, blocked_by=blocked)

    transition = TRANSITIONS[state.beat]
    state = replace(state, in_flight=moves)
    return _emit(
        state,
        Phase.MOVING,
        AdvanceTrain(direction=transition.direction, moves=moves),
    )


def _set_fans(state: LineState) -> StepResult:
    """Choreography steps 6-7: fan states for the new beat, then work begins.

    A fan over an empty station stays off — that is the whole of §4's "startup
    needs no special-case logic", falling out of occupancy rather than a rule.
    """
    beat = next_beat(state.beat)
    spec = SCHEDULE[beat]

    if_fan = spec.if_fan if state.occupancy.get(Station.IF) else FanState.OFF
    fd_fan = spec.fd_fan if state.occupancy.get(Station.FD) else FanState.OFF

    pair_index = state.pair_index + (1 if beat == "P1" else 0)
    state = replace(
        state,
        beat=beat,
        pair_index=pair_index,
        phase=Phase.ROBOT_WORK,
        if_fan=if_fan,
        fd_fan=fd_fan,
        shutter=ShutterState.CLOSED,
    )
    return _emit(
        state,
        Phase.ROBOT_WORK,
        SetFan(station=Station.IF, state=if_fan),
        SetFan(station=Station.FD, state=fd_fan),
    )


def _live_moves(state: LineState) -> tuple[tuple[Station, Station], ...]:
    """This beat's transition, minus moves whose source is empty.

    Dropping empty-source moves here — rather than special-casing startup and
    drain — is what makes §4 and §5 free. A half-empty line runs the steady
    pattern and fills correctly.
    """
    transition = TRANSITIONS[state.beat]
    return tuple(
        (src, dst)
        for src, dst in transition.moves
        if (src is Station.INQ and state.inq_queue) or state.occupancy.get(src)
    )


def _apply_moves(state: LineState, moves: tuple[tuple[Station, Station], ...]) -> LineState:
    """Shift the occupancy map. `moves` is ordered vacate-before-fill."""
    occupancy = dict(state.occupancy)
    inq_queue = state.inq_queue
    parts = dict(state.parts)

    for src, dst in moves:
        if src is Station.INQ:
            if not inq_queue:
                continue
            part_id, inq_queue = inq_queue[0], inq_queue[1:]
        else:
            part_id = occupancy.pop(src, None)
            if part_id is None:
                continue

        if dst is Station.OUT:
            parts.pop(part_id, None)
            continue

        occupancy[dst] = part_id

        # A part arriving at a fan is done being wet once it flashes; a part
        # leaving one has, by the guard above, already banked its full flash.
        if src in (Station.IF, Station.FD) and part_id in parts:
            parts[part_id] = replace(parts[part_id], is_wet=False)

    return replace(state, occupancy=occupancy, inq_queue=inq_queue, parts=parts)


__all__ = ["Inputs", "StepResult", "step"]
