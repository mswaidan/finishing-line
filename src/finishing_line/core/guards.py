"""Interlock predicates — §7 of the process spec.

Every guard is a pure function of (state, sensors, config) returning a reason
string when it BLOCKS and None when it passes. Returning the reason rather than
a bool is deliberate: the HMI has to tell an operator *why* the line is sitting
still, and "waiting" with no explanation is how people start bypassing
interlocks.

Guards read sensors, not the controller's own beliefs, wherever the physical
truth is what matters. `state.shutter` is what we commanded; `sensors.shutter`
is where the shutter actually is. Zone motion gates on the latter.
"""

from __future__ import annotations

from collections.abc import Iterable

from ..config.loader import ProcessConfig
from .model import FanState, LineState, SensorSnapshot, ShutterState, Station
from .timers import may_leave_fan

Reason = str | None


def departure_blocked(
    state: LineState,
    sensors: SensorSnapshot,
    cfg: ProcessConfig,
    moves: Iterable[tuple[Station, Station]],
) -> Reason:
    """Choreography steps 1-2: is the train ready to move, shutter aside?

    Deliberately does NOT check the shutter. The choreography opens the shutter
    at step 3 — *after* this check — so requiring it here would deadlock: the
    shutter would never open because the guard demanded it already be open.

    Enforces the §6 departure guard for any part leaving a fan position. That is
    the check that stretches P3 rather than letting an under-flashed part move.
    """
    if not sensors.robot_clear:
        return "robot not clear of the work envelope"
    if sensors.gun_on:
        return "spray gun still live"

    # Physical end-of-line conditions, both BLOCKS rather than faults: the
    # operator fixes them by loading or unloading parts, no recovery needed.
    if (Station.INQ, Station.IF) in moves and not sensors.inq_present:
        return "queue head empty — load parts at INQ"
    if (Station.FD, Station.OUT) in moves and sensors.out_present:
        return "outfeed occupied — remove the finished part at OUT"

    # The train advances as a unit, so a destination counts as free if an
    # earlier move in this transition vacates it. Checking each move against
    # the static sensor snapshot would block IF->S / INQ->IF on the very part
    # that IF->S is carrying out of the way. `moves` is ordered vacate-before-
    # fill precisely so this walk is valid.
    occupied = set(sensors.occupied)
    for source, dest in moves:
        occupied.discard(source)
        if dest is not Station.OUT:
            if dest in occupied:
                return f"destination {dest} is occupied"
            occupied.add(dest)

        part = state.part_at(source)
        if part is None:
            continue
        if source in (Station.IF, Station.FD) and not may_leave_fan(part, cfg):
            banked = part.active_flash_seconds()
            return (
                f"part {part.part_id} has banked {banked:.0f}s of "
                f"{cfg.flash_seconds:.0f}s flash at {source}"
            )
    return None


def zone_motion_blocked(
    state: LineState,
    sensors: SensorSnapshot,
    cfg: ProcessConfig,
    moves: Iterable[tuple[Station, Station]],
) -> Reason:
    """§7 in full: everything `departure_blocked` checks, plus a confirmed-open
    shutter. Checked at choreography step 4, immediately before motion.

    Re-checking the step-2 conditions here is not redundant: the shutter moved in
    between, and the robot may have drifted out of ROBOT_CLEAR while it did.
    """
    if sensors.shutter is not ShutterState.OPEN:
        return f"shutter not confirmed OPEN (sensor reads {sensors.shutter})"
    return departure_blocked(state, sensors, cfg, moves)


def spray_station_not_ready(
    state: LineState, sensors: SensorSnapshot, cfg: ProcessConfig
) -> Reason:
    """Scheduling-time subset of `spray_blocked`: shutter closed, part at S.

    Deliberately omits the IF-fan clause. At scheduling time the fan is still
    legitimately running — the core *plans* the pause into the work batch (see
    machine._robot_work), so demanding it already be paused here would deadlock:
    the fan is only paused as part of the spray this guard would be blocking.

    `spray_blocked` is the real-time interlock, enforced by the executor at the
    instant the gun opens. This one only decides whether the beat can be
    scheduled.
    """
    if sensors.shutter is not ShutterState.CLOSED:
        return f"shutter not confirmed CLOSED (sensor reads {sensors.shutter})"
    if Station.S not in sensors.occupied:
        return "no part present at S"
    if state.part_at(Station.S) is None:
        return "controller has no part recorded at S"
    return None


def spray_blocked(state: LineState, sensors: SensorSnapshot, cfg: ProcessConfig) -> Reason:
    """§7 in full: shutter CLOSED confirmed, part present and located at S, and
    the IF fan paused if a wet part occupies IF.

    The executor must call this immediately before opening the gun. The fan
    clause is the backstop for the P3 beat, not the primary barrier — the closed
    shutter is the primary one.
    """
    blocked = spray_station_not_ready(state, sensors, cfg)
    if blocked is not None:
        return blocked

    part_at_if = state.part_at(Station.IF)
    if part_at_if is not None and part_at_if.is_wet and sensors.if_fan is FanState.ON:
        return f"wet part {part_at_if.part_id} at IF with the IF fan running"
    return None


def occupancy_mismatch(state: LineState, sensors: SensorSnapshot) -> Reason:
    """§7: does the controller's part map disagree with the presence sensors?

    Checked only at the tracked stations — INQ is a queue whose count is
    reported separately, and OUT is downstream of everything we control.

    Disagreement is a fault, never something to reconcile silently. Recovery is
    an occupancy scan plus operator confirmation of identities, because the
    sensors cannot tell us *which* part they can see.
    """
    tracked = (Station.IF, Station.S, Station.FD)
    for station in tracked:
        expected = state.occupancy.get(station) is not None
        observed = station in sensors.occupied
        if expected != observed:
            return (
                f"sensor mismatch at {station}: controller expected "
                f"{'a part' if expected else 'empty'}, sensor reads "
                f"{'a part' if observed else 'empty'}"
            )
    return None
