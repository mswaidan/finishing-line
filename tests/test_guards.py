"""Interlock predicates — §7."""

from __future__ import annotations

from dataclasses import replace

from finishing_line.core.guards import occupancy_mismatch, spray_blocked, zone_motion_blocked
from finishing_line.core.model import (
    FanState,
    LineState,
    PartRole,
    SensorSnapshot,
    ShutterState,
    Station,
)

from .conftest import make_part

MOVE_IF_TO_S = ((Station.IF, Station.S),)


def _sensors(**kw) -> SensorSnapshot:
    base = dict(
        occupied=frozenset({Station.IF}),
        shutter=ShutterState.OPEN,
        robot_clear=True,
        gun_on=False,
    )
    return SensorSnapshot(**(base | kw))


def _state_with_part_at_if(**part_kw) -> LineState:
    part = make_part("p1", PartRole.TRAIL, **part_kw)
    return LineState(parts={"p1": part}, occupancy={Station.IF: "p1"})


def test_zone_motion_needs_robot_clear(cfg):
    state = _state_with_part_at_if(coats_applied=1, flash_1_s=180.0)
    blocked = zone_motion_blocked(state, _sensors(robot_clear=False), cfg, MOVE_IF_TO_S)
    assert blocked and "robot not clear" in blocked


def test_zone_motion_needs_gun_off(cfg):
    state = _state_with_part_at_if(coats_applied=1, flash_1_s=180.0)
    blocked = zone_motion_blocked(state, _sensors(gun_on=True), cfg, MOVE_IF_TO_S)
    assert blocked and "gun" in blocked


def test_zone_motion_needs_shutter_confirmed_open(cfg):
    """Commanded-open is not open. The guard reads the sensor."""
    state = _state_with_part_at_if(coats_applied=1, flash_1_s=180.0)
    for reading in (ShutterState.CLOSED, ShutterState.MOVING, ShutterState.UNKNOWN):
        blocked = zone_motion_blocked(state, _sensors(shutter=reading), cfg, MOVE_IF_TO_S)
        assert blocked and "shutter" in blocked


def test_zone_motion_needs_empty_destination(cfg):
    state = _state_with_part_at_if(coats_applied=1, flash_1_s=180.0)
    sensors = _sensors(occupied=frozenset({Station.IF, Station.S}))
    blocked = zone_motion_blocked(state, sensors, cfg, MOVE_IF_TO_S)
    assert blocked and "occupied" in blocked


def test_zone_motion_blocks_an_under_flashed_part_leaving_a_fan(cfg):
    """The guard that stretches P3 rather than moving a soft part."""
    state = _state_with_part_at_if(coats_applied=1, flash_1_s=150.0)
    blocked = zone_motion_blocked(state, _sensors(), cfg, MOVE_IF_TO_S)
    assert blocked and "150s of 180s" in blocked

    ready = _state_with_part_at_if(coats_applied=1, flash_1_s=180.0)
    assert zone_motion_blocked(ready, _sensors(), cfg, MOVE_IF_TO_S) is None


def test_feed_blocked_when_queue_head_eye_is_empty(cfg):
    """Declared parts but nothing physically staged: block with a reason the
    operator can act on, don't run belts into a timeout fault.
    """
    state = LineState(inq_queue=("p9",))
    sensors = _sensors(occupied=frozenset(), inq_present=False)
    blocked = zone_motion_blocked(state, sensors, cfg, ((Station.INQ, Station.IF),))
    assert blocked and "load parts" in blocked

    staged_sensors = _sensors(occupied=frozenset(), inq_present=True)
    assert zone_motion_blocked(state, staged_sensors, cfg, ((Station.INQ, Station.IF),)) is None


def test_outfeed_blocked_while_finished_part_awaits_removal(cfg):
    """Never push a cube into a cube: the OUT eye holds the outfeed move."""
    part = make_part("p1", PartRole.LEAD, coats_applied=2, flash_1_s=180.0, flash_2_s=180.0)
    state = LineState(parts={"p1": part}, occupancy={Station.FD: "p1"})
    sensors = _sensors(occupied=frozenset({Station.FD}), out_present=True)
    blocked = zone_motion_blocked(state, sensors, cfg, ((Station.FD, Station.OUT),))
    assert blocked and "remove the finished part" in blocked

    cleared = _sensors(occupied=frozenset({Station.FD}), out_present=False)
    assert zone_motion_blocked(state, cleared, cfg, ((Station.FD, Station.OUT),)) is None


def test_outfeed_destination_is_never_considered_occupied(cfg):
    """OUT is offload — it always accepts a part."""
    part = make_part("p1", PartRole.LEAD, coats_applied=2, flash_2_s=180.0)
    state = LineState(parts={"p1": part}, occupancy={Station.FD: "p1"})
    sensors = _sensors(occupied=frozenset({Station.FD, Station.OUT}))
    assert zone_motion_blocked(state, sensors, cfg, ((Station.FD, Station.OUT),)) is None


def test_spray_needs_shutter_confirmed_closed(cfg):
    part = make_part("p1", PartRole.LEAD)
    state = LineState(parts={"p1": part}, occupancy={Station.S: "p1"})
    sensors = SensorSnapshot(occupied=frozenset({Station.S}), shutter=ShutterState.OPEN)
    blocked = spray_blocked(state, sensors, cfg)
    assert blocked and "CLOSED" in blocked


def test_spray_blocked_by_wet_part_at_if_with_fan_running(cfg):
    """§7 backstop for the P3 beat."""
    lead = make_part("lead", PartRole.LEAD)
    trail = make_part("trail", PartRole.TRAIL, coats_applied=1, is_wet=True)
    state = LineState(
        parts={"lead": lead, "trail": trail},
        occupancy={Station.S: "lead", Station.IF: "trail"},
    )
    sensors = SensorSnapshot(
        occupied=frozenset({Station.S, Station.IF}),
        shutter=ShutterState.CLOSED,
        if_fan=FanState.ON,
    )
    blocked = spray_blocked(state, sensors, cfg)
    assert blocked and "wet part" in blocked

    paused = replace(sensors, if_fan=FanState.OFF)
    assert spray_blocked(state, paused, cfg) is None


def test_a_dry_part_at_if_does_not_block_spray(cfg):
    lead = make_part("lead", PartRole.LEAD)
    staged_trail = make_part("trail", PartRole.TRAIL, is_wet=False)
    state = LineState(
        parts={"lead": lead, "trail": staged_trail},
        occupancy={Station.S: "lead", Station.IF: "trail"},
    )
    sensors = SensorSnapshot(
        occupied=frozenset({Station.S, Station.IF}),
        shutter=ShutterState.CLOSED,
        if_fan=FanState.ON,
    )
    assert spray_blocked(state, sensors, cfg) is None


def test_occupancy_mismatch_detects_a_phantom_part():
    state = LineState()
    sensors = SensorSnapshot(occupied=frozenset({Station.S}))
    mismatch = occupancy_mismatch(state, sensors)
    assert mismatch and "sensor mismatch at s" in mismatch.lower()


def test_occupancy_mismatch_detects_a_missing_part():
    part = make_part("p1", PartRole.LEAD)
    state = LineState(parts={"p1": part}, occupancy={Station.FD: "p1"})
    mismatch = occupancy_mismatch(state, SensorSnapshot(occupied=frozenset()))
    assert mismatch and "fd" in mismatch.lower()


def test_agreeing_occupancy_is_not_a_fault():
    part = make_part("p1", PartRole.LEAD)
    state = LineState(parts={"p1": part}, occupancy={Station.S: "p1"})
    sensors = SensorSnapshot(occupied=frozenset({Station.S}))
    assert occupancy_mismatch(state, sensors) is None
