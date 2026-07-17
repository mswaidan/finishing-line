"""Supervisor — the orchestrator's main loop.

Each tick: read both devices, merge into a SensorSnapshot, step the pure
machine, hand its intents to the executor, heartbeat the watchdog. The
machine stays clockless and I/O-free; this is the only place wall-time and
devices meet.

TWO TRUTH RULES, both load-bearing:

1. **Flash timers run on fan FEEDBACK, not fan belief.** Before stepping, the
   state's fan fields are overwritten from the ClearCore's sensed feedback —
   so when the executor pauses the IF fan for a spray burst, the trail at IF
   banks nothing for exactly the seconds the fan was actually off. This is
   the fan-on-seconds-only rule made physical; trusting the commanded state
   would quietly under-flash the P3 trail every period.

2. **Faults come from the executor and the watchdog, not from guesswork.** A
   failed device call latches the executor; a tripped ClearCore watchdog
   means this loop itself stalled long enough that the firmware took over
   (zones halted, fans forced on). Both surface as machine faults; recovery
   is machine.resume() + executor.reset(), the §7 operator flow.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, replace

from ..config.loader import ProcessConfig
from ..core.machine import Inputs, StepResult, step
from ..core.model import FanState, LineState, SensorSnapshot, Station
from ..devices.clearcore import ClearCoreClient, ClearCoreInputs
from .executor import Executor
from .robot import RobotDevice


def build_sensors(cc: ClearCoreInputs, robot_clear: bool, gun_on: bool) -> SensorSnapshot:
    """Merge the conveyor side (ClearCore) and robot side into the core's view."""
    occupied = set()
    if cc.if_present:
        occupied.add(Station.IF)
    if cc.s_present:
        occupied.add(Station.S)
    if cc.fd_present:
        occupied.add(Station.FD)
    return SensorSnapshot(
        occupied=frozenset(occupied),
        shutter=cc.shutter,
        if_fan=FanState.ON if cc.if_fan_on else FanState.OFF,
        fd_fan=FanState.ON if cc.fd_fan_on else FanState.OFF,
        robot_clear=robot_clear,
        gun_on=gun_on,
        inq_count=cc.inq_count,
    )


@dataclass
class Supervisor:
    cc: ClearCoreClient
    robot: RobotDevice
    executor: Executor
    cfg: ProcessConfig
    state: LineState

    tick_hz: float = 20.0

    def tick(self, dt: float) -> StepResult:
        """One supervision cycle. Separated from run() so tests can drive it
        with a controlled clock.
        """
        cc_inputs = self.cc.read_inputs()
        sensors = build_sensors(cc_inputs, self.robot.is_clear(), self.robot.gun_on())

        fault = self.executor.fault_reason()
        if fault is None and cc_inputs.watchdog_tripped:
            fault = (
                "ClearCore watchdog tripped: this loop stalled past the firmware "
                "timeout; zones halted, fans forced ON"
            )

        # Rule 1: timers advance on sensed fan state, not commanded state.
        state = replace(
            self.state,
            if_fan=sensors.if_fan,
            fd_fan=sensors.fd_fan,
        )

        was_faulted = self.state.fault is not None
        result = step(state, Inputs(dt=dt, sensors=sensors,
                                    completed=self.executor.completed(),
                                    fault=fault), self.cfg)
        self.state = result.state
        self.executor.submit(result.intents)

        # §7 on fault entry: flashing parts keep drying. A fault landing inside
        # the P3 spray bracket leaves the IF fan legitimately OFF over a wet
        # part (the batch died between fan-off and fan-on) — so force fans ON
        # over every occupied fan station. Over-flash is safe by design; a
        # stalled fan over wet finish is not. Direct device call, bypassing the
        # (poisoned) executor.
        if result.state.fault is not None and not was_faulted:
            for station in (Station.IF, Station.FD):
                if station in sensors.occupied:
                    self.cc.set_fan(station, True)

        self.cc.heartbeat()
        return result

    def idle_tick(self, dt: float) -> None:
        """A paused tick: timers and watchdog only, no scheduling.

        Operator pause must not stop parts drying (flash timers keep banking,
        against fan FEEDBACK as always) and must not go silent on the watchdog
        (a pause is not a dead orchestrator — tripping it would force fans on
        and halt zones out from under a deliberate operator action). What it
        does NOT do is step the machine: no new intents, so the line holds at
        the next phase boundary while in-flight executor work finishes.
        """
        from ..core.timers import advance_flash_timers

        cc_inputs = self.cc.read_inputs()
        state = replace(
            self.state,
            if_fan=FanState.ON if cc_inputs.if_fan_on else FanState.OFF,
            fd_fan=FanState.ON if cc_inputs.fd_fan_on else FanState.OFF,
        )
        self.state = replace(state, parts=advance_flash_timers(state, dt))
        self.cc.heartbeat()

    def run(self, *, until, timeout_s: float) -> StepResult:
        """Tick at tick_hz until `until(state)` is true or the deadline passes.

        Returns the last StepResult; the caller inspects state/fault. Wall-time
        dt per tick — the machine sees real elapsed seconds, so process-config
        durations are wall-clock here (the harness compresses them via config,
        never via a fake clock).
        """
        period = 1.0 / self.tick_hz
        deadline = time.monotonic() + timeout_s
        last = time.monotonic()
        result = StepResult(self.state)
        while time.monotonic() < deadline:
            now = time.monotonic()
            result = self.tick(now - last)
            last = now
            if until(self.state):
                return result
            time.sleep(period)
        return result
