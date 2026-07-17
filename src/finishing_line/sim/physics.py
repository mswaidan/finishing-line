"""PhysicsSim — parts on belts, for the Stage B harness.

Plays the role reality plays: watches what the fake ClearCore's registers say
the motors are doing, moves imaginary parts accordingly, and pokes the
presence/handoff sensors the orchestrator then reads back over Modbus. It
never looks at the orchestrator's state — only at motor registers — so the
control stack is exercised exactly as it will be against the real line:
command out over Modbus, truth back through sensors.

Movement model: when the zones a part's next hop rides on all run in the same
direction for `transfer_s` of accumulated belt-on time, the whole train shifts
one station — in TWO PHASES. Sources vacate first, then after `transit_s` the
destinations fill. The gap is not decoration: parts are shorter than the
station pitch, so a real presence sensor at a vacated-and-refilled station
sees fall-then-rise as the inter-part gap passes — and the firmware's
sensor-stop mode triggers on exactly those edges. A teleporting shift would
hold such a sensor high through the whole move and the armed edge would never
fire.

The IF<->S crossing needs BOTH belts (the handoff) and raises the
corresponding handoff sensor on arrival; handoff sensors clear when the belts
stop.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from ..core.model import Station
from ..devices.registers import MODE_CONTINUOUS, MODE_SENSOR_STOP, Command, New
from .fake_clearcore import STATE_MOVING, FakeClearCore

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
    #: Inter-part sensor gap duration (phase A -> phase B). Defaults to a
    #: fraction of transfer_s; must stay comfortably above the fake's tick so
    #: the firmware observes both edges.
    transit_s: float | None = None

    #: Which stations physically hold a part. INQ is a count, not a slot.
    occupied: set[Station] = field(default_factory=set)
    outfed: int = 0
    #: How long a finished part sits at OUT before the imaginary operator
    #: removes it (drives the OUT_PRESENT eye and its outfeed block).
    out_dwell_s: float = 0.5
    _out_clear_at: float | None = None

    _accum: float = 0.0
    #: (deadline, arrivals) during the inter-part gap; arrivals are
    #: (dest, zones) pairs, dest OUT meaning "left the line".
    _transit: tuple[float, list[tuple[Station, tuple[int, ...]]]] | None = None
    _stop: threading.Event = field(default_factory=threading.Event)
    _thread: threading.Thread | None = None

    def __post_init__(self) -> None:
        if self.transit_s is None:
            self.transit_s = max(0.05, self.transfer_s * 0.4)

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
        """(running, downstream) from the fake's registers — command truth.

        A sensor-stop zone runs until the FIRMWARE stops it (state leaves
        MOVING), so mode alone is not enough: mode stays SENSOR_STOP after the
        stop, exactly like a real drive that halted on its own reflex.
        """
        mode = self.fake.holding[
            New.ZONE1_MOTION_MODE if zone_index == 1 else New.ZONE2_MOTION_MODE
        ]
        state = self.fake.input_regs[
            New.ZONE1_STATE if zone_index == 1 else New.ZONE2_STATE
        ]
        direction = self.fake.coils[
            New.ZONE1_DIRECTION if zone_index == 1 else New.ZONE2_DIRECTION
        ]
        running = mode == MODE_CONTINUOUS or (mode == MODE_SENSOR_STOP and state == STATE_MOVING)
        return running, bool(direction)

    def _tick(self, dt: float) -> None:
        # Publish unconditionally: inq_count changes from OUTSIDE (batch
        # declarations) while belts are stopped, and an event-only publish
        # would leave the INQ eye stale — blocking the very feed move that
        # would create the next event.
        if self._out_clear_at is not None and time.monotonic() >= self._out_clear_at:
            self._out_clear_at = None
        self._publish()

        # Phase B first: complete an in-flight shift when the gap has passed.
        # Arrivals land even if a belt already stopped (a part that crossed
        # its stop sensor coasts the last distance) — matching a real stop.
        if self._transit is not None:
            deadline, arrivals = self._transit
            if time.monotonic() < deadline:
                return
            for dest, zones in arrivals:
                if dest is Station.OUT:
                    self.outfed += 1
                    # The part occupies the outfeed until "removed" (dwell).
                    self._out_clear_at = time.monotonic() + self.out_dwell_s
                    continue
                self.occupied.add(dest)
                if zones == (1, 2):  # crossing completed: raise the handoff
                    down = bool(self.fake.coils[New.ZONE1_DIRECTION])
                    self.fake.set_input(
                        New.HANDOFF_TO_Z2 if down else New.HANDOFF_TO_Z1, 1
                    )
            self._transit = None
            self._publish()
            return

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

        # Plan the shift against a working copy (vacate-before-fill ordering),
        # then execute it in two phases: sources clear NOW, destinations fill
        # after transit_s — the inter-part gap a real sensor sees.
        running = {1: (z1_run, z1_down), 2: (z2_run, z2_down)}

        def hop_enabled(zones: tuple[int, ...], want_down: bool) -> bool:
            return all(running[z][0] and running[z][1] == want_down for z in zones)

        downstream = (z1_down if z1_run else z2_down)
        hops = _DOWNSTREAM_HOPS if downstream else _UPSTREAM_HOPS
        work = set(self.occupied)
        arrivals: list[tuple[Station, tuple[int, ...]]] = []
        for src, dest, zones in hops:
            if not hop_enabled(zones, downstream):
                continue
            if src is Station.INQ:
                # The queue rides the feed conveyor (legacy M1): parts leave
                # INQ only while it runs — zone 1 alone never pulls from it.
                feed_on = bool(self.fake.coils[Command.FEED_CONVEYOR])
                if feed_on and self.inq_count > 0 and Station.IF not in work:
                    self.inq_count -= 1
                    arrivals.append((Station.IF, zones))
                    work.add(Station.IF)
                continue
            if src not in work:
                continue
            if dest is not Station.OUT and dest in work:
                continue  # blocked; reality doesn't overlap parts
            work.discard(src)
            self.occupied.discard(src)  # phase A: the source vacates now
            arrivals.append((dest, zones))
            if dest is not Station.OUT:
                work.add(dest)

        if arrivals:
            self._transit = (time.monotonic() + (self.transit_s or 0.05), arrivals)
        self._publish()

    def _publish(self) -> None:
        self.fake.set_input(New.IF_PRESENT, int(Station.IF in self.occupied))
        self.fake.set_input(New.S_PRESENT, int(Station.S in self.occupied))
        self.fake.set_input(New.FD_PRESENT, int(Station.FD in self.occupied))
        self.fake.set_input(New.INQ_PRESENT, int(self.inq_count > 0))
        self.fake.set_input(New.OUT_PRESENT, int(self._out_clear_at is not None))
        self.fake.set_input(New.INQ_COUNT, self.inq_count, table="input")
