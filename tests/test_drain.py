"""End-of-batch drain (§5) and the long soak.

§5's claim mirrors §4's: drain is the steady pattern with slots emptying out,
no special-case logic. The machine has no drain code — these tests prove none
is needed.
"""

from __future__ import annotations

from dataclasses import replace

from finishing_line.core.schedule import Phase
from finishing_line.sim.runner import FakeLine, run

from .conftest import staged


def test_even_batch_drains_completely(cfg):
    """4 parts in, 4 out, line empty, no fault — and the machine keeps idling
    on the steady pattern afterwards rather than needing a stop state.
    """
    state = staged("L1", "T1", "L2", "T2")
    line = FakeLine(cfg=cfg)

    result = run(state, line, cfg, until_outfeed=4, max_seconds=15_000.0)

    assert result.outfeed_count == 4, f"drained {result.outfeed_count}/4; blocked by {result.last_blocked_by}"
    assert not result.state.parts, "all parts should be gone from tracking"
    assert not result.state.occupancy, "line should be physically empty"
    assert result.state.fault is None


def test_odd_batch_lone_lead_drains(cfg):
    """§5 odd count: the lone lead runs the lead path with S idle on trail
    beats — coat 1, F2 flash 1, coat 2, F2 flash 2, OUT. The F1 fan never runs
    for it, and the role-mismatch check never fires because the lone part is
    never at O on a trail beat.
    """
    state = staged("L1", "T1", "L2")  # L2 is the lone lead
    line = FakeLine(cfg=cfg)

    result = run(state, line, cfg, until_outfeed=3, max_seconds=20_000.0)

    assert result.outfeed_count == 3, f"drained {result.outfeed_count}/3; blocked by {result.last_blocked_by}"
    assert result.state.fault is None
    assert not result.state.parts


def test_single_part_batch(cfg):
    """Degenerate drain: one lone lead, nothing else, straight through."""
    state = staged("L1")
    line = FakeLine(cfg=cfg)

    result = run(state, line, cfg, until_outfeed=1, max_seconds=15_000.0)

    assert result.outfeed_count == 1
    assert result.state.fault is None


def test_empty_line_holds_instead_of_cycling_beats(cfg):
    """After drain (or before the first batch) the machine must HOLD, not spin
    P1..P4 forever — every empty beat would cycle the physical shutter over
    nothing. The hold reason surfaces so the HMI can say what to do next.
    """
    from finishing_line.core.machine import Inputs, step
    from finishing_line.core.model import LineState, SensorSnapshot, ShutterState

    state = LineState()
    sensors = SensorSnapshot(shutter=ShutterState.CLOSED, robot_clear=True)

    results = [step(state, Inputs(dt=1.0, sensors=sensors), cfg)]
    for _ in range(10):
        results.append(step(results[-1].state, Inputs(dt=1.0, sensors=sensors), cfg))

    final = results[-1]
    assert final.state.beat == "P1", "beat advanced on an empty line"
    assert final.state.phase == Phase.ROBOT_WORK
    assert not any(r.intents for r in results), "an empty line must not actuate anything"
    assert "declare a batch" in final.blocked_by


def test_soak_a_production_run(cfg):
    """Time-compressed soak: 20 parts straight through on a shortened flash.

    This is the test that catches rare ordering bugs the scenario tests miss —
    a deadlock or a mis-sequenced beat that only appears at a pair boundary
    several periods in. Flash and robot times are config, so compressing them
    exercises identical logic at a fraction of the wall-clock.
    """
    fast = replace(
        cfg,
        flash_seconds=6.0,
        transfer_s=2.0,
        robot_coat1_s=32.0,  # FakeLine sand = coat1 - 30
        robot_coat2_s=3.0,
        clean_gun_duration_s=1.0,
        spray_burst_pause_s=2.0,
    )
    ids = [f"{'L' if i % 2 == 0 else 'T'}{i // 2 + 1}" for i in range(20)]
    state = staged(*ids)
    line = FakeLine(cfg=fast)

    result = run(state, line, fast, until_outfeed=20, dt=0.5, max_seconds=19_000.0)

    assert result.outfeed_count == 20, (
        f"soak stalled at {result.outfeed_count}/20; blocked by {result.last_blocked_by}"
    )
    assert result.state.fault is None
    assert result.state.phase is not Phase.FAULTED
    assert not result.state.parts, "nothing left behind"
