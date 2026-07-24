"""Gun-clean (brush) choreography — verified without hardware.

Whether the tip actually cleans is a maintenance-window item; the call order,
the default-TCP use, the standoff back-off, and the brush-off-on-any-exit
guarantee are pure control flow, locked here. Legacy: script:3122-3164.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from finishing_line.config.loader import BrushConfig
from finishing_line.process.gun_clean import GunClean

CFG = BrushConfig(
    contact_v=0.05, retract_off_mm=3.0, retract_a=0.5, retract_v=0.1,
    settle_before_on_s=0.0, duration_s=0.0, settle_after_off_s=0.0,
)


class FakeUR:
    def __init__(self, log: list) -> None:
        self.log = log

    def use_default_tcp(self): self.log.append(("ur.use_default_tcp",))
    def move_to_named(self, wp): self.log.append(("ur.move_to_named", wp))
    def contact_detect_z(self, speed_ms): self.log.append(("ur.contact_detect_z", speed_ms))
    def move_base_z_mm(self, d, a, v): self.log.append(("ur.move_base_z_mm", d))


class FakeCC:
    def __init__(self, log: list) -> None:
        self.log = log

    def set_brush(self, on): self.log.append(("cc.set_brush", on))


def test_clean_call_order():
    """Default (sand) TCP, goto brush, contact up, brush on/off — NO back-off
    after contact (removed 2026-07-26: the tip stays on the brush; bristle
    compliance does the work)."""
    log: list = []
    GunClean(FakeUR(log), FakeCC(log), CFG).clean()
    assert log == [
        ("ur.use_default_tcp",),
        ("ur.move_to_named", "Clean_Brush"),
        ("ur.contact_detect_z", 0.05),
        ("cc.set_brush", True),
        ("cc.set_brush", False),
    ]


def test_brush_off_even_if_the_hold_is_interrupted(monkeypatch):
    """If anything throws during the brush hold, the brush must still be shut off
    — never leave a spinning brush running against a dead program.
    """
    log: list = []
    cfg = replace(CFG, duration_s=99.0)  # distinctive so we fail only in the hold

    def boom(seconds):
        if seconds == 99.0:
            raise RuntimeError("hold interrupted")

    monkeypatch.setattr("time.sleep", boom)
    with pytest.raises(RuntimeError):
        GunClean(FakeUR(log), FakeCC(log), cfg).clean()
    assert ("cc.set_brush", True) in log
    assert log[-1] == ("cc.set_brush", False)  # finally turned it off
