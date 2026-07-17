"""Train advance — the two-belt handoff, on firmware sensor-stops.

THE MANOEUVRE
-------------
Nothing spans IF <-> S; a part crosses by handoff: both belts run together at
matched speed until sensors confirm the transfer. Almost every transition
crosses that boundary, so the usual advance is a synchronised both-belt run.

WHO STOPS THE BELT
------------------
The firmware does. Each involved zone is armed with MODE_SENSOR_STOP and a
per-zone target edge (ZONE*_TARGET); the ClearCore's own loop watches the
sensor and stops as a reflex. The orchestrator's Modbus polling (~20-50 ms of
variance) is out of the positioning chain entirely — which matters on this
line, where there are no encoders and no closed-loop motors: the sensor edge
is the only positioning truth there is, so it gets the tightest stop
available. The orchestrator supervises: it arms the moves, waits for both
zones to report READY, then verifies the post-shift occupancy pattern.

STOP TARGETS ARE EDGES, chosen per transition from the move set. The rule for
each zone is "the LAST sensor event of its part of the shift":

  downstream  zone 1: INQ refill present -> IF rising (departure falls first,
                      refill rises last); crossing only -> HANDOFF_TO_Z2 rising
              zone 2: S->FD present -> FD rising (a departing FD part falls
                      first); FD->OUT alone -> FD falling; crossing only ->
                      S rising
  upstream    zone 1: IF rising (the retreating part seats at IF)
              zone 2: FD->S present -> S rising; retreat alone ->
                      HANDOFF_TO_Z1 rising

Levels would be wrong everywhere a destination starts occupied (every
simultaneous vacate+fill) — the firmware's edge semantics carry the fix that
was previously bolted on here.

PITCH DEPENDENCE unchanged: one belt moves everything on it by one distance,
and stopping on a sensor measures that distance rather than decoupling parts
sharing the belt. Correct only while the four station gaps are equal
(line-config stations.pitch_mm).
"""

from __future__ import annotations

import time

from ..core.model import Direction, Station, Zone
from ..devices.clearcore import ClearCoreClient
from ..devices.registers import SensorTarget


class TrainError(RuntimeError):
    """A train advance did not confirm — parts are in an unknown position.

    Deliberately fatal: the supervisor faults the machine, and recovery is the
    §7 occupancy-scan flow, not a retry. Re-running belts against unknown part
    positions is how collisions happen.
    """


Move = tuple[Station, Station]
_CROSS_DOWN: Move = (Station.IF, Station.S)
_CROSS_UP: Move = (Station.S, Station.IF)


def _zone_targets(
    moves: tuple[Move, ...], downstream: bool
) -> dict[Zone, tuple[SensorTarget, bool]]:
    """Per-zone (sensor, falling) stop edge — the last event of the shift."""
    targets: dict[Zone, tuple[SensorTarget, bool]] = {}
    if downstream:
        if (Station.INQ, Station.IF) in moves:
            targets[Zone.ZONE1] = (SensorTarget.IF_PRESENT, False)
        elif _CROSS_DOWN in moves:
            targets[Zone.ZONE1] = (SensorTarget.HANDOFF_TO_Z2, False)
        if (Station.S, Station.FD) in moves:
            targets[Zone.ZONE2] = (SensorTarget.FD_PRESENT, False)
        elif (Station.FD, Station.OUT) in moves:
            targets[Zone.ZONE2] = (SensorTarget.FD_PRESENT, True)
        elif _CROSS_DOWN in moves:
            targets[Zone.ZONE2] = (SensorTarget.S_PRESENT, False)
    else:
        if _CROSS_UP in moves:
            targets[Zone.ZONE1] = (SensorTarget.IF_PRESENT, False)
        if (Station.FD, Station.S) in moves:
            targets[Zone.ZONE2] = (SensorTarget.S_PRESENT, False)
        elif _CROSS_UP in moves:
            targets[Zone.ZONE2] = (SensorTarget.HANDOFF_TO_Z1, False)
    return targets


class TrainMover:
    def __init__(self, cc: ClearCoreClient, *, timeout_s: float = 60.0, poll_s: float = 0.02) -> None:
        self._cc = cc
        self._timeout_s = timeout_s
        self._poll_s = poll_s

    def advance(self, direction: Direction, moves: tuple[Move, ...]) -> None:
        if not moves:
            return
        downstream = direction is Direction.DOWNSTREAM
        targets = _zone_targets(moves, downstream)
        if not targets:
            raise TrainError(f"no zone stop targets derivable from moves {moves}")

        # The INQ queue rides its own feed conveyor (legacy M1): zone 1 alone
        # never pulls from the queue, so an INQ->IF move needs the feed belt
        # running alongside — and every other transition must leave it OFF, or
        # zone-1 runs would drag unplanned parts onto IF.
        feeding = (Station.INQ, Station.IF) in moves
        if feeding:
            self._cc.set_feed_conveyor(True)
        try:
            for zone, (sensor, falling) in targets.items():
                self._cc.move_zone_until(
                    zone, downstream=downstream, sensor=sensor, falling=falling
                )
            for zone in targets:
                self._cc.wait_zone_ready(zone, timeout_s=self._timeout_s)

            # Firmware stopped every belt on its edge; now verify the world
            # matches the plan. Every destination present, every source-only
            # station empty — false before the shift, true only after it.
            dests = {dst for _src, dst in moves if dst is not Station.OUT}
            source_only = {
                src for src, _dst in moves if src in (Station.IF, Station.S, Station.FD)
            } - dests
            self._wait(
                lambda: all(self._cc.presence(d) for d in dests)
                and not any(self._cc.presence(s) for s in source_only),
                f"post-shift pattern for {[f'{s}->{d}' for s, d in moves]}",
                timeout_s=2.0,
            )
        finally:
            # Belts idle no matter what. On the failure path parts are in an
            # unknown position — leaving armed moves live makes that worse.
            if feeding:
                self._cc.set_feed_conveyor(False)
            for zone in targets:
                self._cc.set_zone_idle(zone)

    def _wait(self, predicate, what: str, timeout_s: float | None = None) -> None:
        deadline = time.monotonic() + (timeout_s or self._timeout_s)
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(self._poll_s)
        raise TrainError(f"train advance timed out waiting for {what}")
