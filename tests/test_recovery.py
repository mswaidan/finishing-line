"""Fault recovery — the §7 flow, end to end.

Every test here drives the real machine through a fault and out the other side.
The claim under test is the one §6 stakes the design on: per-part timers make
recovery unambiguous.
"""

from __future__ import annotations

from dataclasses import replace

from finishing_line.core.intents import HaltZones
from finishing_line.core.machine import Inputs, resume, step
from finishing_line.core.model import (
    FanState,
    SensorSnapshot,
    ShutterState,
    Station,
)
from finishing_line.core.schedule import Phase
from finishing_line.sim.runner import FakeLine, run

from .conftest import staged


def _drive(state, line, cfg, seconds, dt=1.0, inject_fault_at=None):
    """Run the loop manually so tests can inject faults mid-flight."""
    elapsed = 0.0
    result = None
    while elapsed < seconds:
        completed = line.advance(dt)
        effective_if = FanState.OFF if line.gun_on else line.f1_fan
        sim_state = replace(state, f1_fan=effective_if, f2_fan=line.f2_fan)
        fault = None
        if inject_fault_at is not None and elapsed >= inject_fault_at:
            fault = "protective stop"
            inject_fault_at = None
        result = step(
            sim_state,
            Inputs(dt=dt, sensors=line.sensors(len(state.in_queue)),
                   completed=completed, fault=fault),
            cfg,
        )
        state = result.state
        line.submit(result.intents)
        elapsed += dt
        if state.phase is Phase.FAULTED:
            break
    return state, line, result, elapsed


def test_protective_stop_halts_zones_but_parts_keep_drying(cfg):
    """§7: zones halt immediately; fans hold state; flash timers keep counting."""
    state = staged("L1", "T1", "L2", "T2")
    line = FakeLine(cfg=cfg)

    # Run well into the schedule, then hit the e-stop mid-flight.
    state, line, result, _ = _drive(state, line, cfg, 600.0, inject_fault_at=250.0)

    assert state.phase is Phase.FAULTED
    assert state.fault == "protective stop"
    assert any(isinstance(i, HaltZones) for i in result.intents)

    # A part is flashing somewhere; freeze the world and let time pass.
    flashing = [p for p in state.parts.values() if p.coats_applied >= 1]
    assert flashing, "fault should have landed after at least one coat"
    before = {p.part_id: p.active_flash_seconds() for p in flashing}

    sensors = line.sensors(len(state.in_queue))
    result = step(state, Inputs(dt=120.0, sensors=sensors), cfg)
    state = result.state

    still_at_fans = [
        p for p in state.parts.values()
        if p.part_id in before and state.station_of(p.part_id) in (Station.F1, Station.F2)
    ]
    for part in still_at_fans:
        station = state.station_of(part.part_id)
        if state.fan_state(station) is FanState.ON:
            assert part.active_flash_seconds() == before[part.part_id] + 120.0, (
                f"{part.part_id} stopped drying during the fault"
            )
    assert state.phase is Phase.FAULTED, "time passing must not clear a fault"


def test_resume_after_protective_stop_completes_the_batch(cfg):
    """The full §7 arc: fault, operator ack, resume, and every part still
    outfeeds fully flashed. The per-part timers carry the truth across the gap.
    """
    state = staged("L1", "T1")
    line = FakeLine(cfg=cfg)

    state, line, _, elapsed = _drive(state, line, cfg, 900.0, inject_fault_at=300.0)
    assert state.phase is Phase.FAULTED

    # Operator scan agrees with the controller here (nothing physically moved),
    # so resume with no corrections.
    result = resume(state, line.sensors(len(state.in_queue)))
    assert result.state.phase is not Phase.FAULTED
    assert result.state.fault is None
    assert result.intents, "resume must emit the re-home (MoveToSafePose)"

    # The interrupted work batch died with the fault; the fake line must not
    # keep acting on stale intents the machine no longer tracks. The re-home
    # intent resume emitted is the one thing that DOES run.
    line.queue.clear()
    line.current = None
    line.gun_on = False
    line.submit(result.intents)

    sim = run(result.state, line, cfg, until_outfeed=2, max_seconds=10_000.0)
    assert sim.outfeed_count == 2, f"batch did not finish; blocked by {sim.last_blocked_by}"
    assert sim.state.fault is None


def test_resume_does_not_double_coat(cfg):
    """A part sprayed just before the fault must not be sprayed again after.

    The machine records the coat at emission, so on resume the robot_work
    idempotence check sees it and skips to the guards.
    """
    state = staged("L1", "T1")
    line = FakeLine(cfg=cfg)

    # Fault right after the first work batch is emitted (t=54 area is P1 work
    # on L1 in the fill; 60s is safely mid-batch).
    state, line, _, _ = _drive(state, line, cfg, 300.0, inject_fault_at=60.0)
    assert state.phase is Phase.FAULTED
    coated = [p for p in state.parts.values() if p.coats_applied == 1]
    assert coated, "expected the fault to land after coat 1 was recorded"
    coats_at_fault = {p.part_id: p.coats_applied for p in state.parts.values()}

    result = resume(state, line.sensors(len(state.in_queue)))
    line.queue.clear()
    line.current = None
    line.robot_clear = True
    line.gun_on = False

    # Complete the re-home, then step through the resumed beat's ROBOT_WORK.
    # No part may gain a coat.
    rehome_ids = frozenset(i.intent_id for i in result.intents)
    nxt = step(result.state, Inputs(dt=1.0, sensors=line.sensors(0), completed=rehome_ids), cfg)
    for pid, coats in coats_at_fault.items():
        if pid in nxt.state.parts:
            assert nxt.state.parts[pid].coats_applied == coats, f"{pid} was re-coated on resume"


def test_resume_with_corrected_occupancy_after_sensor_mismatch(cfg):
    """Mismatch faults; the operator's scan finds the part one station along;
    resume with the corrected map is accepted and validated against sensors.
    """
    from .conftest import make_part
    from finishing_line.core.model import LineState, PartRole

    part = make_part("p1", PartRole.LEAD, coats_applied=1, flash_1_s=50.0)
    # Controller believes O; the part is actually at F2.
    state = LineState(parts={"p1": part}, occupancy={Station.O: "p1"})
    truth = SensorSnapshot(
        occupied=frozenset({Station.F2}), shutter=ShutterState.CLOSED, robot_clear=True
    )

    faulted = step(state, Inputs(dt=1.0, sensors=truth), cfg)
    assert faulted.state.phase is Phase.FAULTED

    # Wrong correction (agrees with neither belief nor sensors) is rejected.
    rejected = resume(
        faulted.state, truth, confirmed_occupancy={Station.F1: "p1"}
    )
    assert rejected.state.phase is Phase.FAULTED
    assert "resume rejected" in rejected.blocked_by

    # Correct reconstruction resumes, and the part kept its banked flash time.
    resumed = resume(
        faulted.state, truth, confirmed_occupancy={Station.F2: "p1"}, beat="P2"
    )
    assert resumed.state.phase is Phase.ROBOT_WORK
    assert resumed.state.parts["p1"].flash_1_s == 50.0
    assert resumed.state.occupancy == {Station.F2: "p1"}


def test_resume_is_refused_when_not_faulted(cfg):
    state = staged("L1", "T1")
    sensors = SensorSnapshot(shutter=ShutterState.CLOSED, robot_clear=True)
    result = resume(state, sensors)
    assert result.state.phase is not Phase.FAULTED
    assert "not faulted" in result.blocked_by
