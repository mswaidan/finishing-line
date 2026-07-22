"""Sander choreography + URRobot state — verified without hardware.

Force-mode *feel* is a maintenance-window item, but the CALL ORDER (the
belt<->robot handshake) and the URRobot state transitions are pure control flow,
locked here against recording fakes. Mirrors the legacy sequence
script:2460-2497.
"""

from __future__ import annotations

import pytest

from finishing_line.config.loader import BrushConfig, ProductSpec, SandConfig, SprayConfig
from finishing_line.core.model import Zone
from finishing_line.process.gun_clean import GunClean
from finishing_line.process.robot_ur import URRobot
from finishing_line.process.sander import Sander
from finishing_line.process.sprayer import Sprayer

CUBE = ProductSpec(name="cube", legacy_job_id=1, width_mm=362, height_mm=355, depth_mm=349)

SAND_CFG = SandConfig(
    z_force_n=6.0, width_inset_mm=12, movel_a=0.5, movel_v=0.05,
    contact_search_distance_m=1000.0, stopl_on_contact=3.0,
    stopl_on_force_end=5.0, ft_wait_steady_ms=2000,
)
SPRAY_CFG = SprayConfig(
    width_inset_mm=12, approach_z_m=0.1, approach_a=1.2, approach_v=0.25,
    height_a=0.5, height_v=0.05,
)
BRUSH_CFG = BrushConfig(
    contact_v=0.05, retract_off_mm=3.0, retract_a=0.5, retract_v=0.1,
    settle_before_on_s=0.0, duration_s=0.0, settle_after_off_s=0.0,
)


class FakeUR:
    """Records every call into a shared, order-preserving log."""

    def __init__(self, log: list) -> None:
        self.log = log

    def move_to_named(self, wp): self.log.append(("ur.move_to_named", wp))
    def contact_detect_z(self, speed_ms): self.log.append(("ur.contact_detect_z", speed_ms))
    def zero_ft(self, settle_s=0.1, pre_wait_s=0.0): self.log.append(("ur.zero_ft", pre_wait_s))
    def set_tool(self, on): self.log.append(("ur.set_tool", on))
    def begin_force_z(self, newtons): self.log.append(("ur.begin_force_z", newtons))
    def end_force_mode(self, decel=None): self.log.append(("ur.end_force_mode", decel))
    def move_base_x_mm(self, distance_mm, a, v): self.log.append(("ur.move_base_x_mm", distance_mm))


class FakeCC:
    def __init__(self, log: list) -> None:
        self.log = log

    def move_zone_mm(self, zone, distance_mm, **kw):
        self.log.append(("cc.move_zone_mm", zone, distance_mm))
        return 1

    def wait_zone_ready(self, zone, **kw):
        self.log.append(("cc.wait_zone_ready", zone))


def test_sand_face_call_order():
    """The exact legacy order: approach, probe, zero, tool on, force, the duet,
    force off, tool off — reversing tool/force drags a dead sander (script:2497).
    """
    log: list = []
    Sander(FakeUR(log), FakeCC(log), SAND_CFG).sand_face(CUBE)
    assert [c[0] for c in log] == [
        "ur.move_to_named",
        "ur.contact_detect_z",
        "ur.zero_ft",
        "ur.set_tool",       # on
        "ur.begin_force_z",
        "cc.move_zone_mm",   # belt +pass
        "cc.wait_zone_ready",
        "ur.move_base_x_mm",  # robot -step
        "cc.move_zone_mm",   # belt -pass
        "cc.wait_zone_ready",
        "ur.move_base_x_mm",  # robot +step
        "ur.end_force_mode",
        "ur.set_tool",       # off
    ]


def test_sand_face_uses_tuned_values_and_zone2():
    log: list = []
    Sander(FakeUR(log), FakeCC(log), SAND_CFG).sand_face(CUBE)
    assert ("ur.move_to_named", "Sand_Base") in log
    assert ("ur.contact_detect_z", 0.05) in log
    assert ("ur.zero_ft", 2.0) in log            # 2000 ms -> 2.0 s pre-wait
    assert ("ur.begin_force_z", 6.0) in log
    assert ("ur.end_force_mode", 5.0) in log     # stopl_on_force_end
    assert log.count(("ur.set_tool", True)) == 1
    assert log.count(("ur.set_tool", False)) == 1
    # Belt runs on Z2, +pass then -pass; pass = width - inset = 362 - 12 = 350.
    assert [c for c in log if c[0] == "cc.move_zone_mm"] == [
        ("cc.move_zone_mm", Zone.Z2, 350.0),
        ("cc.move_zone_mm", Zone.Z2, -350.0),
    ]
    # Robot height step = height = 355, -step then +step (returns to origin).
    assert [c[1] for c in log if c[0] == "ur.move_base_x_mm"] == [-355.0, 355.0]


def test_traverse_fault_still_lifts_force_and_kills_tool():
    """A belt failure mid-traverse must still end force mode and stop the tool —
    the `finally` in sand_face. Otherwise the arm holds 6 N on a dead tool.
    """
    log: list = []

    class Boom(FakeCC):
        def wait_zone_ready(self, zone, **kw):
            self.log.append(("cc.wait_zone_ready", zone))
            raise RuntimeError("belt stuck")

    with pytest.raises(RuntimeError):
        Sander(FakeUR(log), Boom(log), SAND_CFG).sand_face(CUBE)
    assert [c[0] for c in log][-2:] == ["ur.end_force_mode", "ur.set_tool"]
    assert log[-1] == ("ur.set_tool", False)


# ------------------------------------------------------------------ URRobot


def _make_robot(log: list) -> URRobot:
    ur = FakeUR(log)  # one arm: shared by safe_pose + all composites (as in prod)
    cc = FakeCC(log)
    sander = Sander(ur, cc, SAND_CFG)
    sprayer = Sprayer(ur, cc, SPRAY_CFG)  # constructed, not exercised here
    gun_clean = GunClean(ur, cc, BRUSH_CFG)
    return URRobot(ur, sander, sprayer, gun_clean, lambda pid: CUBE)


def test_urrobot_sand_drops_clear_and_runs_the_sander():
    log: list = []
    robot = _make_robot(log)
    assert robot.is_clear() is True
    robot.sand("p1")
    assert robot.is_clear() is False           # not clear until safe_pose
    assert ("ur.move_to_named", "Sand_Base") in log  # sander executed


def test_urrobot_safe_pose_parks_and_restores_clear():
    log: list = []
    robot = _make_robot(log)
    robot.sand("p1")
    robot.safe_pose()
    assert robot.is_clear() is True
    assert log[-1] == ("ur.move_to_named", "Sand_Base")  # parked at clear waypoint


def test_urrobot_gun_off():
    robot = _make_robot([])
    assert robot.gun_on() is False  # sand/spray/clean_gun covered in their own tests
