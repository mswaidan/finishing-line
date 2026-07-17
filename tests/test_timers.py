"""Flash timer semantics — the rule the whole schedule rests on."""

from __future__ import annotations

from dataclasses import replace

from finishing_line.core.model import FanState, LineState, PartRole, Station
from finishing_line.core.timers import (
    advance_flash_timers,
    may_outfeed,
    may_receive_coat_2,
)

from .conftest import make_part


def test_flash_accumulates_only_while_the_fan_is_on(cfg):
    part = make_part("p1", PartRole.TRAIL, coats_applied=1)
    state = LineState(parts={"p1": part}, occupancy={Station.IF: "p1"}, if_fan=FanState.ON)

    parts = advance_flash_timers(state, dt=10.0, cfg=cfg)
    assert parts["p1"].flash_1_s == 10.0

    # Fan pauses for a spray burst: the part banks nothing. This is the P3 case,
    # and it is the entire reason P3 stretches past its nominal 195 s.
    paused = replace(state, parts=parts, if_fan=FanState.OFF)
    parts = advance_flash_timers(paused, dt=10.0, cfg=cfg)
    assert parts["p1"].flash_1_s == 10.0, "wall-clock at a dead fan is not flash time"


def test_part_away_from_a_fan_banks_nothing(cfg):
    part = make_part("p1", PartRole.LEAD, coats_applied=1)
    state = LineState(
        parts={"p1": part}, occupancy={Station.S: "p1"}, if_fan=FanState.ON, fd_fan=FanState.ON
    )
    parts = advance_flash_timers(state, dt=60.0, cfg=cfg)
    assert parts["p1"].flash_1_s == 0.0


def test_timer_targets_the_active_coat(cfg):
    """After coat 1 the part banks flash 1; after coat 2, flash 2."""
    part = make_part("p1", PartRole.LEAD, coats_applied=1)
    state = LineState(parts={"p1": part}, occupancy={Station.FD: "p1"}, fd_fan=FanState.ON)
    parts = advance_flash_timers(state, dt=180.0, cfg=cfg)
    assert parts["p1"].flash_1_s == 180.0
    assert parts["p1"].flash_2_s == 0.0

    after_coat_2 = replace(state, parts={"p1": replace(parts["p1"], coats_applied=2)})
    parts = advance_flash_timers(after_coat_2, dt=180.0, cfg=cfg)
    assert parts["p1"].flash_1_s == 180.0
    assert parts["p1"].flash_2_s == 180.0


def test_coat_2_guard_requires_complete_flash_1(cfg):
    part = make_part("p1", PartRole.TRAIL, coats_applied=1, flash_1_s=179.9)
    assert not may_receive_coat_2(part, cfg)
    assert may_receive_coat_2(replace(part, flash_1_s=180.0), cfg)


def test_outfeed_guard_requires_both_coats_and_flash_2(cfg):
    part = make_part("p1", PartRole.LEAD, coats_applied=2, flash_1_s=180.0, flash_2_s=179.9)
    assert not may_outfeed(part, cfg)
    assert may_outfeed(replace(part, flash_2_s=180.0), cfg)

    under_coated = make_part("p2", PartRole.LEAD, coats_applied=1, flash_2_s=999.0)
    assert not may_outfeed(under_coated, cfg)


def test_wet_clears_the_moment_the_flash_completes(cfg):
    """Wet means "coated and flash incomplete" — it must clear when the flash
    finishes, not when the part later moves. A fully-flashed part tagged wet
    misleads the HMI and keeps the §7 IF-fan pause armed for a dry part.
    """
    part = make_part("p1", PartRole.TRAIL, coats_applied=1, flash_1_s=170.0, is_wet=True)
    state = LineState(parts={"p1": part}, occupancy={Station.IF: "p1"}, if_fan=FanState.ON)

    parts = advance_flash_timers(state, dt=5.0, cfg=cfg)
    assert parts["p1"].is_wet, "still 5s short of the flash target"

    state = replace(state, parts=parts)
    parts = advance_flash_timers(state, dt=6.0, cfg=cfg)
    assert parts["p1"].flash_1_s >= cfg.flash_seconds
    assert not parts["p1"].is_wet, "flash complete — the part is dry"


def test_wet_persists_while_flash_is_interrupted(cfg):
    """A part pulled off a fan mid-flash stays wet — no time, no drying."""
    part = make_part("p1", PartRole.LEAD, coats_applied=1, flash_1_s=100.0, is_wet=True)
    state = LineState(parts={"p1": part}, occupancy={Station.S: "p1"}, fd_fan=FanState.ON)
    parts = advance_flash_timers(state, dt=500.0, cfg=cfg)
    assert parts["p1"].is_wet, "no fan time banked, so no drying happened"


def test_over_flashing_is_always_allowed(cfg):
    """§7: a fault may over-flash a part, never under-flash it."""
    part = make_part("p1", PartRole.LEAD, coats_applied=2, flash_1_s=180.0, flash_2_s=4000.0)
    assert may_outfeed(part, cfg)
