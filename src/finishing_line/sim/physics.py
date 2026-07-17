"""PhysicsSim — parts on belts, for the Stage B harness.

Plays the role reality plays: watches what the fake ClearCore's registers say
the motors are doing, moves imaginary parts accordingly, and pokes the
presence/handoff sensors the orchestrator then reads back over Modbus. It
never looks at the orchestrator's state — only at motor registers — so the
control stack is exercised exactly as it will be against the real line:
command out over Modbus, truth back through sensors.

Movement model: when the zones a part's next hop rides on all run in the same
direction for `transfer_s` of accumulated belt-on time, the whole train shifts
one station. The IF<->S crossing needs BOTH belts (the handoff) and raises the
corresponding handoff sensor; handoff sensors clear when the belts stop.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from ..core.model import Station
from ..devices.registers import Command, New
from .fake_clearcore import MODE_CONTINUOUS, FakeClearCore

_DOWNSTREAM_HOPS: tuple[tuple[Station, Station, tuple[int, ...]], ...] = (
    # ordered downstream-first so one pass vacates before it fills
    (Station.FD, Station.OUT, (2,)),
    (Station.S, Station.FD, (2,)),
    (Station.IF, Station.S, (1, 2)),   # handoff: both belts
    (Station.INQ, Station.IF, (1,)),
)
_UPSTREAM_HOPS: tuple[tuple[Station, Station, tuple[int, ...]], ...] = (
    # upstream-first for the same reason
    (Station.S, Station.IF, (1, 2)),   # handoff: both belts
    (Station.FD, Station.S, (2,)),
)


@dataclass
class PhysicsSim:
    fake: FakeClearCore
    inq_count: int = 0
    transfer_s: float = 0.3
    tick_s: float = 0.02

    #: Which stations physically hold a part. INQ is a count, not a slot.
    occupied: set[Station] = field(default_factory=set)
    outfed: int = 0

    _accum: float = 0.0
    _stop: threading.Event = field(default_factory=threading.Event)
    _thread: threading.Thread | None = None

    def start(self) -> "PhysicsSim":
        self._publish()
        self._thread = threading.Thread(target=self._run, daemon=True, name="physics-sim")
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    # ------------------------------------------------------------------ loop

    def _run(self) -> None:
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            self._tick(now - last)
            last = now
            time.sleep(self.tick_s)

    def _zone_running(self, zone_index: int) -> tuple[bool, bool]:
        """(running, downstream) from the fake's registers — command truth."""
        mode = self.fake.holding[
            New.ZONE1_MOTION_MODE if zone_index == 1 else New.ZONE2_MOTION_MODE
        ]
        direction = self.fake.coils[
            New.ZONE1_DIRECTION if zone_index == 1 else New.ZONE2_DIRECTION
        ]
        return mode == MODE_CONTINUOUS, bool(direction)

    def _tick(self, dt: float) -> None:
        z1_run, z1_down = self._zone_running(1)
        z2_run, z2_down = self._zone_running(2)

        if not (z1_run or z2_run):
            # Belts stopped: reset the travel accumulator and drop handoffs.
            self._accum = 0.0
            self.fake.set_input(New.HANDOFF_TO_Z1, 0)
            self.fake.set_input(New.HANDOFF_TO_Z2, 0)
            return

        self._accum += dt
        if self._accum < self.transfer_s:
            return
        self._accum = 0.0

        # A hop happens when every belt it rides on is running, all in the
        # same direction. Mixed directions move nothing — matching the spec's
        # rule that no zone opposes its neighbour while parts span it.
        running = {1: (z1_run, z1_down), 2: (z2_run, z2_down)}

        def hop_enabled(zones: tuple[int, ...], want_down: bool) -> bool:
            return all(running[z][0] and running[z][1] == want_down for z in zones)

        downstream = (z1_down if z1_run else z2_down)
        hops = _DOWNSTREAM_HOPS if downstream else _UPSTREAM_HOPS
        for src, dest, zones in hops:
            if not hop_enabled(zones, downstream):
                continue
            if src is Station.INQ:
                # The queue rides the feed conveyor (legacy M1): parts leave
                # INQ only while it runs — zone 1 alone never pulls from it.
                feed_on = bool(self.fake.coils[Command.FEED_CONVEYOR])
                if feed_on and self.inq_count > 0 and Station.IF not in self.occupied:
                    self.inq_count -= 1
                    self.occupied.add(Station.IF)
                continue
            if src not in self.occupied:
                continue
            if dest is Station.OUT:
                self.occupied.discard(src)
                self.outfed += 1
                continue
            if dest in self.occupied:
                continue  # blocked; reality doesn't overlap parts
            self.occupied.discard(src)
            self.occupied.add(dest)
            if zones == (1, 2):  # the crossing completed: raise the handoff
                self.fake.set_input(
                    New.HANDOFF_TO_Z2 if downstream else New.HANDOFF_TO_Z1, 1
                )
        self._publish()

    def _publish(self) -> None:
        self.fake.set_input(New.IF_PRESENT, int(Station.IF in self.occupied))
        self.fake.set_input(New.S_PRESENT, int(Station.S in self.occupied))
        self.fake.set_input(New.FD_PRESENT, int(Station.FD in self.occupied))
        self.fake.set_input(New.INQ_COUNT, self.inq_count, table="input")
