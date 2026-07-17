"""The schedule tables — structural properties of §3.

These are cheap invariants that would have caught the P3 contradiction at
review time, which is why they exist.
"""

from __future__ import annotations

from finishing_line.core.schedule import (
    BEATS,
    OUTFEED_TRANSITIONS,
    SCHEDULE,
    TRANSITIONS,
    next_beat,
)
from finishing_line.core.model import (
    STATION_ORDER,
    Direction,
    FanState,
    PartRole,
    ShutterState,
)


def test_period_is_four_beats_and_cycles():
    assert len(BEATS) == 4
    assert [next_beat(b) for b in BEATS] == ["P2", "P3", "P4", "P1"]


def test_every_beat_has_a_transition():
    assert set(SCHEDULE) == set(TRANSITIONS) == set(BEATS)


def test_shutter_is_closed_for_all_robot_work():
    """§3: shutter CLOSED on every beat. It is the primary contamination barrier."""
    assert all(spec.shutter is ShutterState.CLOSED for spec in SCHEDULE.values())


def test_each_role_works_twice_per_period():
    """Two parts complete per period, each getting two coats."""
    roles = [spec.robot.role for spec in SCHEDULE.values()]
    assert roles.count(PartRole.LEAD) == 2
    assert roles.count(PartRole.TRAIL) == 2
    coats = sorted(spec.robot.coat for spec in SCHEDULE.values())
    assert coats == [1, 1, 2, 2]


def test_exactly_one_beat_pauses_the_if_fan():
    """P3 alone — and it is the only beat that can stretch past nominal."""
    pausing = [b for b, spec in SCHEDULE.items() if spec.if_fan_pauses_during_spray]
    assert pausing == ["P3"]


def test_a_paused_fan_is_never_also_the_only_fan_off():
    """P3 runs IF and rests FD, so a fan pause never leaves both fans dead."""
    spec = SCHEDULE["P3"]
    assert spec.if_fan is FanState.ON
    assert spec.fd_fan is FanState.OFF


def test_two_parts_outfeed_per_period():
    """§3: outfeed on P1->P2 and P4->P1'."""
    assert OUTFEED_TRANSITIONS == {"P1", "P4"}
    outfeeds = [b for b, t in TRANSITIONS.items() if any(dst.name == "OUT" for _, dst in t.moves)]
    assert sorted(outfeeds) == ["P1", "P4"]


def test_every_move_is_exactly_one_station():
    """§3: every transition advances the train one station, never two."""
    for beat, transition in TRANSITIONS.items():
        for src, dst in transition.moves:
            gap = STATION_ORDER.index(dst) - STATION_ORDER.index(src)
            assert abs(gap) == 1, f"{beat}: {src}->{dst} spans {abs(gap)} stations"


def test_move_direction_matches_the_declared_direction():
    """No zone ever opposes its neighbour while parts span the boundary."""
    for beat, transition in TRANSITIONS.items():
        want = 1 if transition.direction is Direction.DOWNSTREAM else -1
        for src, dst in transition.moves:
            gap = STATION_ORDER.index(dst) - STATION_ORDER.index(src)
            assert gap == want, f"{beat}: {src}->{dst} opposes {transition.direction}"


def test_moves_vacate_before_they_fill():
    """Ordering invariant: no move fills a station that a later move vacates."""
    for beat, transition in TRANSITIONS.items():
        filled: set = set()
        for src, _dst in transition.moves:
            assert src not in filled, f"{beat}: {src} is filled before it is vacated"
            filled.update({d for _s, d in transition.moves[: transition.moves.index((src, _dst))]})


def test_no_two_moves_share_a_destination():
    for beat, transition in TRANSITIONS.items():
        dests = [dst for _src, dst in transition.moves]
        assert len(dests) == len(set(dests)), f"{beat}: two parts sent to one station"
