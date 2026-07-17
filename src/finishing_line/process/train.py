"""Train advance — the two-belt handoff, implemented against ClearCoreClient.

THE MANOEUVRE
-------------
Nothing spans IF <-> S; a part crosses by handoff: both belts run together at
matched speed until a sensor confirms the part is on the receiving belt
(HANDOFF_TO_Z2 downstream, HANDOFF_TO_Z1 for the P2->P3 retreat). Almost every
transition crosses that boundary, so the usual advance is a synchronised
both-belt run; the exceptions are the sparse startup/drain transitions whose
crossing move has an empty source and was dropped by the core.

The run is CONTINUOUS mode, terminated by sensors:

    1. start every involved zone in continuous mode, same direction
    2. if a crossing move is present, wait for the handoff sensor
    3. wait for every destination's presence sensor (absence, for OUT)
    4. idle the zones

Completion means the destination sensors already read occupied — which is what
lets the core's MOVING phase reconcile occupancy against sensors immediately
after the intent completes, without a settling window.

PITCH DEPENDENCE. One belt moves every part on it by one distance; terminating
on sensors measures that distance rather than decoupling the parts sharing a
belt. This implementation is correct only if the four station gaps are equal —
the standing physical assumption (line-config stations.pitch_mm). If
commissioning finds unequal gaps, this module grows sequenced sub-moves and the
schedule's one-advance-per-transition model changes with it.
"""

from __future__ import annotations

import time

from ..core.model import Direction, Station, Zone
from ..devices.clearcore import ClearCoreClient


class TrainError(RuntimeError):
    """A train advance did not confirm — parts are in an unknown position.

    Deliberately fatal: the supervisor faults the machine, and recovery is the
    §7 occupancy-scan flow, not a retry. Re-running belts against unknown part
    positions is how collisions happen.
    """


#: Which belts a single-station move rides on. The IF<->S crossing needs both.
_MOVE_ZONES: dict[tuple[Station, Station], tuple[Zone, ...]] = {
    (Station.INQ, Station.IF): (Zone.ZONE1,),
    (Station.IF, Station.S): (Zone.ZONE1, Zone.ZONE2),
    (Station.S, Station.IF): (Zone.ZONE1, Zone.ZONE2),
    (Station.S, Station.FD): (Zone.ZONE2,),
    (Station.FD, Station.S): (Zone.ZONE2,),
    (Station.FD, Station.OUT): (Zone.ZONE2,),
}


class TrainMover:
    def __init__(self, cc: ClearCoreClient, *, timeout_s: float = 60.0, poll_s: float = 0.02) -> None:
        self._cc = cc
        self._timeout_s = timeout_s
        self._poll_s = poll_s

    def advance(self, direction: Direction, moves: tuple[tuple[Station, Station], ...]) -> None:
        if not moves:
            return
        downstream = direction is Direction.DOWNSTREAM
        zones = {zone for move in moves for zone in _MOVE_ZONES[move]}
        crossing = any(move in (((Station.IF, Station.S)), (Station.S, Station.IF)) for move in moves)

        # The INQ queue rides its own feed conveyor (legacy M1): zone 1 alone
        # never pulls from the queue, so an INQ->IF move needs the feed belt
        # running alongside — and every other transition must leave it OFF, or
        # zone-1 runs would drag unplanned parts onto IF.
        feeding = (Station.INQ, Station.IF) in moves
        if feeding:
            self._cc.set_feed_conveyor(True)
        for zone in zones:
            self._cc.set_zone_continuous(zone, downstream=downstream)
        # Completion = the post-shift occupancy PATTERN, not per-move sensor
        # levels. A level check like "wait until FD present" passes instantly
        # when FD is still occupied by the part that is about to LEAVE (every
        # simultaneous vacate+fill, e.g. FD->OUT with S->FD), stopping the
        # belts before anything moved. The pattern below is provably false
        # before the shift — the train's most-downstream destination is always
        # empty beforehand (guards) — and true only after it:
        #   every destination present  AND  every source-only station empty.
        dests = {dst for _src, dst in moves if dst is not Station.OUT}
        source_only = {
            src for src, _dst in moves if src in (Station.IF, Station.S, Station.FD)
        } - dests

        def shifted() -> bool:
            return all(self._cc.presence(d) for d in dests) and not any(
                self._cc.presence(s) for s in source_only
            )

        try:
            if crossing:
                self._wait(
                    lambda: self._cc.handoff(downstream=downstream),
                    f"handoff sensor ({'to zone 2' if downstream else 'to zone 1'})",
                )
            self._wait(shifted, f"train shift {[f'{s}->{d}' for s, d in moves]}")
        finally:
            # Belts stop no matter what. On the failure path parts are in an
            # unknown position — leaving belts running makes that worse.
            if feeding:
                self._cc.set_feed_conveyor(False)
            for zone in zones:
                self._cc.set_zone_idle(zone)

    def _wait(self, predicate, what: str) -> None:
        deadline = time.monotonic() + self._timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(self._poll_s)
        raise TrainError(f"train advance timed out waiting for {what} ({self._timeout_s:.0f}s)")
