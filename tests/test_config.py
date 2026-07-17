"""Config loading against the real YAML files.

These read cell-config.yaml and line-config.yaml from the repo, so they fail if
a key is renamed or a tuned constant is lost — which is the point. The whole
value of the archaeology pass is that these numbers stop being re-derived.
"""

from __future__ import annotations

import pytest

from finishing_line.config.loader import (
    load_process_config,
    load_products,
    load_sand_config,
)


def test_process_config_loads():
    cfg = load_process_config()
    assert cfg.flash_seconds == 180.0
    assert cfg.coats == 2


def test_sand_constants_match_the_old_program():
    """Tuned on the real line; proven by 200 parts/week. If these drift from
    cell-config.yaml, the rewrite has quietly re-derived a finish quality that
    took years to settle.
    """
    sand = load_sand_config()
    assert sand.z_force_n == 6.0, "main sand force (script:2463)"
    assert sand.width_inset_mm == 12, "pass inset that keeps the tool on the part"
    assert sand.movel_v == 0.05, "50 mm/s process feed"
    assert sand.movel_a == 0.5
    assert sand.contact_search_distance_m == 1000.0, "unbounded; contact stops the probe"
    assert sand.ft_wait_steady_ms == 2000


def test_products_carry_the_legacy_job_dimensions():
    """Cube and browser dimensions must match read_job() (script:2120)."""
    products = load_products()
    assert products["cube"].legacy_job_id == 1
    assert (products["cube"].width_mm, products["cube"].height_mm) == (362, 355)
    assert products["browser"].legacy_job_id == 2
    assert (products["browser"].width_mm, products["browser"].height_mm) == (362, 349)


def test_nominal_period_includes_the_burst_stretch():
    cfg = load_process_config()
    beat = cfg.flash_seconds + cfg.transfer_s
    assert cfg.nominal_period_s() == pytest.approx(beat * 4 + cfg.spray_burst_pause_s)


def test_unmeasured_parameters_are_declared():
    """Everything the line currently runs on faith. This list should shrink as
    the open items in §8 get answered; it must never silently be empty.
    """
    cfg = load_process_config()
    unmeasured = cfg.unmeasured()
    assert "flash_seconds" in unmeasured, "180 s flash is assumed, not measured"
    assert "spray_burst_pause_s" in unmeasured, "the beat-stretching pause is unmeasured"
    assert "transfer_s" in unmeasured, "15 s transfer is assumed (§8)"
