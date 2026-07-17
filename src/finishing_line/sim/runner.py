"""Simulator — runs the real state machine against a fake line.

The core is pure, so the simulator needs no hardware and no wall-clock: it
satisfies intents with nominal durations and steps time forward in fixed
increments. The code under test is the code that runs the line.

This is what makes "every fault case" (CLAUDE.md build order step 2) a tractable
test suite rather than a maintenance-window gamble.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

from ..config.loader import ProcessConfig
from ..core.intents import (
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
from ..core.machine import Inputs, StepResult, step
from ..core.model import (
    FanState,
    LineState,
    SensorSnapshot,
    ShutterState,
    Station,
)


@dataclass
class FakeLine:
    """A fake physical line: satisfies intents after nominal durations.

    Tracks its own occupancy independently of the controller's belief, so tests
    can desynchronise the two on purpose and check that the mismatch faults.
    """

    cfg: ProcessConfig
    occupied: set[Station] = field(default_factory=set)
    shutter: ShutterState = ShutterState.CLOSED
    if_fan: FanState = FanState.OFF
    fd_fan: FanState = FanState.OFF
    robot_clear: bool = True
    gun_on: bool = False

    #: FIFO. Intents run ONE AT A TIME, in submission order — the executor
    #: contract. Running a batch concurrently would let the IF fan pause and
    #: resume in the same instant as the spray it is meant to bracket, hiding
    #: the P3 stretch entirely.
    queue: list[Intent] = field(default_factory=list)
    current: Intent | None = None
    remaining: float = 0.0

    def submit(self, intents: tuple[Intent, ...]) -> None:
        self.queue.extend(intents)

    def _duration(self, intent: Intent) -> float:
        match intent:
            case SandPart():
                return self.cfg.robot_coat1_s - 30.0
            case DenibPart():
                return self.cfg.denib_duration_s
            case SprayPart():
                return 30.0
            case AdvanceTrain():
                return self.cfg.transfer_s
            case SetShutter():
                return 1.0
            case MoveToSafePose():
                return 3.0
            case _:
                return 0.0

    def _begin(self, intent: Intent) -> None:
        match intent:
            case SprayPart():
                self.gun_on = True
                self.robot_clear = False
            case SandPart() | DenibPart():
                self.robot_clear = False
            case SetShutter():
                self.shutter = ShutterState.MOVING
            case HaltZones():
                pass

    def advance(self, dt: float) -> frozenset[str]:
        """Step the fake line by `dt`; return ids that completed.

        Drains the queue sequentially, so several zero-duration intents can
        finish inside one tick while a long one spans many.
        """
        done: set[str] = set()
        budget = dt

        while True:
            if self.current is None:
                if not self.queue:
                    break
                self.current = self.queue.pop(0)
                self.remaining = self._duration(self.current)
                self._begin(self.current)

            if self.remaining > budget:
                self.remaining -= budget
                break

            budget -= self.remaining
            self._finish(self.current)
            done.add(self.current.intent_id)
            self.current = None
            self.remaining = 0.0

        return frozenset(done)

    def _finish(self, intent: Intent) -> None:
        match intent:
            case SprayPart():
                self.gun_on = False
            case MoveToSafePose():
                self.robot_clear = True
            case SetShutter(target=target):
                self.shutter = target
            case SetFan(station=station, state=state):
                if station is Station.IF:
                    self.if_fan = state
                else:
                    self.fd_fan = state
            case AdvanceTrain(moves=moves):
                for src, dst in moves:
                    self.occupied.discard(src)
                    if dst is not Station.OUT:
                        self.occupied.add(dst)

    def sensors(self, inq_count: int) -> SensorSnapshot:
        return SensorSnapshot(
            occupied=frozenset(self.occupied),
            shutter=self.shutter,
            if_fan=self.if_fan,
            fd_fan=self.fd_fan,
            robot_clear=self.robot_clear,
            gun_on=self.gun_on,
            inq_count=inq_count,
        )


@dataclass
class SimResult:
    state: LineState
    elapsed_s: float
    outfeed_count: int
    steps: int
    last_blocked_by: str | None


def run(
    state: LineState,
    line: FakeLine,
    cfg: ProcessConfig,
    *,
    until_outfeed: int = 2,
    dt: float = 1.0,
    max_seconds: float = 20_000.0,
) -> SimResult:
    """Run until `until_outfeed` parts have left, or `max_seconds` elapses.

    The IF-fan pause is modelled here rather than in the core: while the gun is
    live, the fan is forced off, so a part at IF banks nothing. That is exactly
    the P3 stretch, and the simulator is where its throughput cost becomes
    visible.
    """
    elapsed = 0.0
    steps = 0
    seen = set(state.parts)
    outfeed = 0
    result: StepResult | None = None

    while elapsed < max_seconds and outfeed < until_outfeed:
        completed = line.advance(dt)

        # §7 backstop: pause the upstream fan while the gun is live.
        effective_if_fan = FanState.OFF if line.gun_on else line.if_fan
        sim_state = replace(state, if_fan=effective_if_fan, fd_fan=line.fd_fan)

        result = step(sim_state, Inputs(dt=dt, sensors=line.sensors(len(state.inq_queue)),
                                        completed=completed), cfg)
        state = result.state
        line.submit(result.intents)

        gone = seen - set(state.parts)
        outfeed += len(gone)
        seen -= gone

        elapsed += dt
        steps += 1

    return SimResult(
        state=state,
        elapsed_s=elapsed,
        outfeed_count=outfeed,
        steps=steps,
        last_blocked_by=result.blocked_by if result else None,
    )
