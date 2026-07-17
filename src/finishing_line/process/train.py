"""Train advance — the two-belt handoff.

WHY THIS IS NOT A DRIVER CALL
-----------------------------
`AdvanceTrain` looks like "command two conveyors", but it is a coordinated
manoeuvre across both belts, terminated by a sensor rather than a distance. Same
category as the sanding duet in sander.py: a composite no single device can
carry out, which is why it lives in process/ and not in devices/.

THE MANOEUVRE
-------------
Nothing spans IF <-> S. A part crosses by handoff: both belts run together, at
matched speed, until a sensor confirms the part is on the receiving belt. Belts
must run at the same speed — a part bridging the gap between two belts moving at
different speeds skews or scuffs.

Every transition in the steady schedule crosses IF <-> S (S is occupied every
beat), so every transition is a synchronised both-belt run. There is no
zone-1-only or zone-2-only advance to implement.

    DOWNSTREAM  run both belts forward until the crossing part trips the
                zone-2 handoff sensor, then continue to destination sensors
    UPSTREAM    run both belts reverse until the crossing part trips the
                zone-1 handoff sensor  (P2->P3 only: the trail's retreat)

WHERE THIS GETS DELICATE
------------------------
One belt moves every part on it by one distance. Sensor-terminating the run
measures that distance; it does not decouple the parts sharing the belt.

P2->P3 is the case to get right. Moves are S->IF and FD->S, and BOTH parts start
on zone 2. Zone 2 reverses and both travel together. Terminate on the retreating
part reaching zone 1 and the FD part only lands on S if IF->S == S->FD;
terminate on the FD part reaching S and the retreating part is the one that
misses. There is no termination rule that rescues unequal pitches — the fix is
physical, not logical.

Likewise on P4->P1', zone 1 carries the INQ part while handing another part to
S, so INQ->IF is tied to the same distance.

Hence the design assumption: all four station gaps equal. If commissioning finds
they are not, this module grows sequenced sub-moves and the core's one-advance-
per-transition model in schedule.py has to change with it.

TODO(step 3/4): implement against hardware in a maintenance window.
"""

from __future__ import annotations

from ..core.model import Direction, Station


class TrainMover:
    """Executes `AdvanceTrain` intents across both belts."""

    def advance(self, direction: Direction, moves: tuple[tuple[Station, Station], ...]) -> None:
        """Advance every part one station in `direction`.

        TODO(step 4).

        Sketch:
          1. Both belts to matched velocity in `direction`.
          2. Run until the handoff sensor confirms the crossing part has
             transferred (zone 2's sensor downstream, zone 1's upstream).
          3. Continue to the destination presence sensors; stop both belts.
          4. Confirm every destination in `moves` reads occupied before
             returning — completion means CONFIRMED, not commanded. The core
             faults on an unconfirmed advance rather than believing it.

        A `moves` list whose sources are partly empty is normal, not an error:
        that is startup fill and drain (§4, §5) running the steady pattern with
        unoccupied slots. Terminate on whichever parts are actually present.
        """
        raise NotImplementedError

    def _handoff_sensor(self, direction: Direction) -> str:
        """Which sensor confirms the IF<->S crossing, per direction.

        TODO: assign the register. devices/registers.py has no handoff sensor
        yet — the proposed map predates knowing the crossing was sensor-
        terminated, and it needs one sensor per direction (or one that reports
        which belt the part is on).
        """
        raise NotImplementedError
