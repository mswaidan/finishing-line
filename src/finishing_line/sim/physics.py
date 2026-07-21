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

The F1<->O crossing needs BOTH belts (the handoff) and raises the
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
    (Station.F2, Station.OUT, (2,)),
    (Station.O, Station.F2, (2,)),
    (Station.F1, Station.O, (1, 2)),   # handoff: both belts
    (Station.IN, Station.F1, (1,)),
)
_UPSTREAM_HOPS: tuple[tuple[Station, Station, tuple[int, ...]], ...] = (
    # upstream-first for the same reason
    (Station.O, Station.F1, (1, 2)),   # handoff: both belts
    (Station.F2, Station.O, (2,)),
)


@dataclass
class PhysicsSim:
    fake: FakeClearCore
    in_count: int = 0
    transfer_s: float = 0.3
    tick_s: float = 0.02
    #: Inter-part sensor gap duration (phase A -> phase B). Defaults to a
    #: fraction of transfer_s; must stay comfortably above the fake's tick so
    #: the firmware observes both edges.
    transit_s: float | None = None

    #: Which stations physically hold a part. IN is a count, not a slot.
    occupied: set[Station] = field(default_factory=set)
    outfed: int = 0
    #: How long a finished part sits at OUT before the imaginary operator
    #: removes it (drives the OUT_EYE eye and its outfeed block).
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
            New.Z1_MODE if zone_index == 1 else New.Z2_MODE
        ]
        state = self.fake.input_regs[
            New.Z1_STATE if zone_index == 1 else New.Z2_STATE
        ]
        direction = self.fake.coils[
            New.Z1_DIR if zone_index == 1 else New.Z2_DIR
        ]
        running = mode == MODE_CONTINUOUS or (mode == MODE_SENSOR_STOP and state == STATE_MOVING)
        return running, bool(direction)

    def _tick(self, dt: float) -> None:
        # Publish unconditionally: in_count changes from OUTSIDE (batch
        # declarations) while belts are stopped, and an event-only publish
        # would leave the IN eye stale — blocking the very feed move that
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
                    down = bool(self.fake.coils[New.Z1_DIR])
                    self.fake.set_input(
                        New.Z2_EYE if down else New.Z1_EYE, 1
                    )
            self._transit = None
            self._publish()
            return

        z1_run, z1_down = self._zone_running(1)
        z2_run, z2_down = self._zone_running(2)

        if not (z1_run or z2_run):
            # Belts stopped: reset the travel accumulator and drop handoffs.
            self._accum = 0.0
            self.fake.set_input(New.Z1_EYE, 0)
            self.fake.set_input(New.Z2_EYE, 0)
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
            if src is Station.IN:
                # The queue rides the feed conveyor (legacy M1): parts leave
                # IN only while it runs — Z1 alone never pulls from it.
                feed_on = bool(self.fake.coils[Command.FEED_CONVEYOR])
                if feed_on and self.in_count > 0 and Station.F1 not in work:
                    self.in_count -= 1
                    arrivals.append((Station.F1, zones))
                    work.add(Station.F1)
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
        self.fake.set_input(New.F1_EYE, int(Station.F1 in self.occupied))
        self.fake.set_input(New.O_EYE, int(Station.O in self.occupied))
        self.fake.set_input(New.F2_EYE, int(Station.F2 in self.occupied))
        self.fake.set_input(New.IN_EYE, int(self.in_count > 0))
        self.fake.set_input(New.OUT_EYE, int(self._out_clear_at is not None))
        self.fake.set_input(New.IN_COUNT, self.in_count, table="input")
