"""End-to-end schedule behaviour, driven through the simulator.

These exercise the real state machine — the only fake is the line itself.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from finishing_line.core.model import FanState, PartRole, Station
from finishing_line.core.schedule import Phase
from finishing_line.sim.runner import FakeLine, run

from .conftest import staged


def test_startup_fills_from_empty_with_no_special_case_logic(cfg):
    """§4's central claim: startup is the steady pattern with empty slots.

    Nothing in the machine knows what 'startup' is. If this passes, the claim
    holds and §4 costs zero code.
    """
    state = staged("L1", "T1", "L2", "T2")
    line = FakeLine(cfg=cfg)

    result = run(state, line, cfg, until_outfeed=1, max_seconds=6000.0)

    assert result.outfeed_count == 1, f"nothing outfed; blocked by {result.last_blocked_by}"
    assert result.state.phase is not Phase.FAULTED


def test_a_full_period_outfeeds_exactly_two_parts(cfg):
    """§3: 2 parts per 4-beat period."""
    state = staged("L1", "T1", "L2", "T2", "L3", "T3")
    line = FakeLine(cfg=cfg)

    result = run(state, line, cfg, until_outfeed=2, max_seconds=8000.0)

    assert result.outfeed_count == 2
    assert result.state.fault is None


def test_no_part_ever_outfeeds_under_flashed(cfg):
    """The invariant the whole design exists to protect."""
    state = staged("L1", "T1", "L2", "T2")
    line = FakeLine(cfg=cfg)

    outfed: list = []
    seen = dict(state.parts)

    # Re-run manually so we can inspect parts at the moment they leave.
    from finishing_line.core.machine import Inputs, step

    elapsed, dt = 0.0, 1.0
    while elapsed < 8000.0 and len(outfed) < 2:
        completed = line.advance(dt)
        effective_if = FanState.OFF if line.gun_on else line.if_fan
        sim_state = replace(state, if_fan=effective_if, fd_fan=line.fd_fan)
        result = step(
            sim_state,
            Inputs(dt=dt, sensors=line.sensors(len(state.inq_queue)), completed=completed),
            cfg,
        )
        for part_id in set(sim_state.parts) - set(result.state.parts):
            outfed.append(seen[part_id])
        seen.update(result.state.parts)
        state = result.state
        line.submit(result.intents)
        elapsed += dt

    assert outfed, "no parts completed"
    for part in outfed:
        assert part.coats_applied == cfg.coats, f"{part.part_id} left with {part.coats_applied} coats"
        assert part.flash_1_s >= cfg.flash_seconds, f"{part.part_id} under-flashed on coat 1"
        assert part.flash_2_s >= cfg.flash_seconds, f"{part.part_id} under-flashed on coat 2"


def _beat_durations(cfg, seconds: float = 3000.0) -> dict[str, list[float]]:
    """Wall-clock spent in each beat across a sim run."""
    from finishing_line.core.machine import Inputs, step

    state = staged("L1", "T1", "L2", "T2", "L3", "T3")
    line = FakeLine(cfg=cfg)
    durations: dict[str, list[float]] = {}
    current, started, elapsed, dt = state.beat, 0.0, 0.0, 1.0

    while elapsed < seconds:
        completed = line.advance(dt)
        effective_if = FanState.OFF if line.gun_on else line.if_fan
        sim_state = replace(state, if_fan=effective_if, fd_fan=line.fd_fan)
        result = step(
            sim_state,
            Inputs(dt=dt, sensors=line.sensors(len(state.inq_queue)), completed=completed),
            cfg,
        )
        state = result.state
        line.submit(result.intents)
        if state.beat != current:
            durations.setdefault(current, []).append(elapsed - started)
            current, started = state.beat, elapsed
        elapsed += dt

    return durations


def test_p3_is_measurably_longer_than_the_other_beats(cfg):
    """The P3 stretch, observed in the running machine rather than asserted.

    P3 is the only beat where the trail must bank a full flash while its fan is
    paused for the lead's spray burst. If this ever stops being true, either the
    fan pause has quietly vanished or timers stopped honouring fan state — both
    of which ship soft parts.
    """
    durations = _beat_durations(cfg)
    # Startup and drain beats run in seconds because nothing is flashing; only
    # the loaded beats are paced by a flash timer, so take the longest instance
    # of each beat as its steady-state duration.
    steady = {beat: max(times) for beat, times in durations.items() if times}
    assert steady.get("P3", 0) > 100, "sim did not reach steady state"

    p3 = steady["P3"]
    others = [t for beat, t in steady.items() if beat != "P3" and t > 100]
    assert others, "no loaded comparison beats"

    assert p3 > max(others), f"P3 ({p3:.0f}s) should exceed every other beat ({max(others):.0f}s)"
    # The excess is the burst pause the trail spent banking nothing.
    assert p3 - max(others) == pytest.approx(cfg.spray_burst_pause_s, abs=3.0)


def test_p3_stretches_by_the_burst_pause(cfg):
    """The consequence of banking fan-on seconds only.

    The trail's entire flash 1 is the P3 beat, and the IF fan is off while the
    lead is sprayed. So P3 must run longer than a nominal beat — this asserts
    the cost is real and shows up in wall-clock, not just in theory.
    """
    no_pause = replace(cfg, spray_burst_pause_s=0.0)
    assert cfg.nominal_period_s() > no_pause.nominal_period_s()
    assert cfg.nominal_period_s() == pytest.approx(780.0 + 30.0)
    assert cfg.nominal_seconds_per_part() == pytest.approx(405.0)

    # Every second of burst costs half a second per part.
    slower = replace(cfg, spray_burst_pause_s=60.0)
    delta = slower.nominal_seconds_per_part() - cfg.nominal_seconds_per_part()
    assert delta == pytest.approx(15.0)


def test_target_cycle_requires_a_zero_pause(cfg):
    """6.5 min/part is only reachable if the IF fan never pauses."""
    assert cfg.nominal_seconds_per_part() > 390.0
    no_pause = replace(cfg, spray_burst_pause_s=0.0)
    assert no_pause.nominal_seconds_per_part() == pytest.approx(390.0)


def test_sensor_mismatch_faults_and_halts_zones(cfg):
    """§7: halt zones, keep fans running, alarm."""
    from finishing_line.core.intents import HaltZones
    from finishing_line.core.machine import Inputs, step
    from finishing_line.core.model import SensorSnapshot

    state = staged("L1", "T1")
    phantom = SensorSnapshot(occupied=frozenset({Station.S}), robot_clear=True)
    result = step(state, Inputs(dt=1.0, sensors=phantom), cfg)

    assert result.state.phase is Phase.FAULTED
    assert result.blocked_by and "mismatch" in result.blocked_by
    assert any(isinstance(i, HaltZones) for i in result.intents)


def test_flash_timers_keep_running_while_faulted(cfg):
    """§7: parts keep drying through a protective stop. Over-flash is safe;
    freezing the timer would make a dried part look under-flashed on recovery.
    """
    from finishing_line.core.machine import Inputs, step
    from finishing_line.core.model import LineState, SensorSnapshot

    from .conftest import make_part

    part = make_part("p1", PartRole.LEAD, coats_applied=1)
    state = LineState(
        parts={"p1": part},
        occupancy={Station.FD: "p1"},
        fd_fan=FanState.ON,
        phase=Phase.FAULTED,
        fault="protective stop",
    )
    sensors = SensorSnapshot(occupied=frozenset({Station.FD}))
    result = step(state, Inputs(dt=60.0, sensors=sensors), cfg)

    assert result.state.parts["p1"].flash_1_s == 60.0
    assert result.state.phase is Phase.FAULTED
