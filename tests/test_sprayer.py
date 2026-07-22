"""Sprayer choreography — verified without hardware.

Spray *quality* is a maintenance-window item; the motion CALL ORDER, the gun-on
windows, the Z2 belt distances, and the spray<->default TCP wrapping are pure
control flow, locked here against recording fakes. cube = script:3060-3111,
browser = script:2856-2941.
"""

from __future__ import annotations

import pytest

from finishing_line.config.loader import BrushConfig, ProductSpec, SprayConfig
from finishing_line.core.model import Zone
from finishing_line.process.gun_clean import GunClean
from finishing_line.process.robot_ur import URRobot
from finishing_line.process.sander import Sander
from finishing_line.process.sprayer import Sprayer

CUBE = ProductSpec(name="cube", legacy_job_id=1, width_mm=362, height_mm=355, depth_mm=349)
BROWSER = ProductSpec(name="browser", legacy_job_id=2, width_mm=362, height_mm=349, depth_mm=235)
STEREOCAB = ProductSpec(name="45", legacy_job_id=3, width_mm=724, height_mm=235, depth_mm=349)

CFG = SprayConfig(
    width_inset_mm=12, approach_z_m=0.1, approach_a=1.2, approach_v=0.25,
    height_a=0.5, height_v=0.05,
)
BRUSH_CFG = BrushConfig(
    contact_v=0.05, retract_off_mm=3.0, retract_a=0.5, retract_v=0.1,
    settle_before_on_s=0.0, duration_s=0.0, settle_after_off_s=0.0,
)


class FakeUR:
    def __init__(self, log: list) -> None:
        self.log = log

    def use_spray_tcp(self): self.log.append(("ur.use_spray_tcp",))
    def use_default_tcp(self): self.log.append(("ur.use_default_tcp",))
    def move_to_named(self, wp): self.log.append(("ur.move_to_named", wp))
    def set_sprayer(self, on): self.log.append(("ur.set_sprayer", on))
    def move_base_x_mm(self, d, a, v): self.log.append(("ur.move_base_x_mm", d))
    def move_base_z_mm(self, d, a, v): self.log.append(("ur.move_base_z_mm", d))


class FakeCC:
    def __init__(self, log: list) -> None:
        self.log = log

    def move_zone_mm(self, zone, d, **kw):
        self.log.append(("cc.move_zone_mm", zone, d))
        return 1

    def wait_zone_ready(self, zone, **kw):
        self.log.append(("cc.wait_zone_ready", zone))


def _spray(product: ProductSpec) -> list:
    log: list = []
    Sprayer(FakeUR(log), FakeCC(log), CFG).spray(product, coat=1)
    return log


def test_cube_call_order_and_gun_windows():
    """Horizontal: gun toggled per stroke over three waypoints; belt −pass
    overlaps the Waypoint_3 movej. Wrapped in spray/default TCP.
    """
    assert _spray(CUBE) == [
        ("ur.use_spray_tcp",),
        ("ur.move_to_named", "Waypoint_1"),
        ("ur.set_sprayer", True),
        ("cc.move_zone_mm", Zone.Z2, 350.0),
        ("cc.wait_zone_ready", Zone.Z2),
        ("ur.set_sprayer", False),
        ("ur.move_to_named", "Waypoint_2"),
        ("ur.set_sprayer", True),
        ("ur.move_base_x_mm", -355.0),
        ("ur.set_sprayer", False),
        ("cc.move_zone_mm", Zone.Z2, -350.0),
        ("ur.move_to_named", "Waypoint_3"),   # overlaps the belt -pass
        ("cc.wait_zone_ready", Zone.Z2),
        ("ur.set_sprayer", True),
        ("ur.move_base_x_mm", 355.0),
        ("ur.set_sprayer", False),
        ("ur.set_sprayer", False),            # finally (idempotent)
        ("ur.use_default_tcp",),
    ]


def test_browser_call_order_gun_continuous():
    """Vertical: 0.1 m standoff, gun ON for one continuous raster, back to base."""
    assert _spray(BROWSER) == [
        ("ur.use_spray_tcp",),
        ("ur.move_to_named", "Spray_Base"),
        ("ur.move_base_z_mm", 100.0),         # 0.1 m -> 100 mm base-Z standoff
        ("ur.set_sprayer", True),
        ("cc.move_zone_mm", Zone.Z2, 350.0),
        ("cc.wait_zone_ready", Zone.Z2),
        ("ur.move_base_x_mm", -349.0),
        ("cc.move_zone_mm", Zone.Z2, -350.0),
        ("cc.wait_zone_ready", Zone.Z2),
        ("ur.move_base_x_mm", 349.0),
        ("ur.set_sprayer", False),
        ("ur.move_to_named", "Spray_Base"),
        ("ur.set_sprayer", False),            # finally
        ("ur.use_default_tcp",),
    ]


def test_cube_gun_toggled_thrice_browser_once():
    assert [c for c in _spray(CUBE) if c == ("ur.set_sprayer", True)] == [("ur.set_sprayer", True)] * 3
    assert [c for c in _spray(BROWSER) if c == ("ur.set_sprayer", True)] == [("ur.set_sprayer", True)]


def test_fault_mid_spray_drops_gun_and_restores_tcp():
    log: list = []

    class Boom(FakeCC):
        def wait_zone_ready(self, zone, **kw):
            self.log.append(("cc.wait_zone_ready", zone))
            raise RuntimeError("belt stuck")

    with pytest.raises(RuntimeError):
        Sprayer(FakeUR(log), Boom(log), CFG).spray(CUBE, coat=1)
    assert log[-2:] == [("ur.set_sprayer", False), ("ur.use_default_tcp",)]


def test_unsupported_product_raises_but_restores_tcp():
    log: list = []
    with pytest.raises(NotImplementedError):
        Sprayer(FakeUR(log), FakeCC(log), CFG).spray(STEREOCAB, coat=1)
    # No routine ran, but the spray TCP was set and must still be restored.
    assert log == [("ur.use_spray_tcp",), ("ur.set_sprayer", False), ("ur.use_default_tcp",)]


def test_urrobot_spray_dispatches_and_clears_gun():
    log: list = []
    ur, cc = FakeUR(log), FakeCC(log)
    robot = URRobot(
        ur, Sander(ur, cc, None), Sprayer(ur, cc, CFG),
        GunClean(ur, cc, BRUSH_CFG), lambda pid: BROWSER,
    )
    robot.spray("p1", 1)
    assert robot.gun_on() is False        # window closed on return
    assert robot.is_clear() is False      # not clear until safe_pose
    assert ("ur.move_to_named", "Spray_Base") in log  # browser routine executed
