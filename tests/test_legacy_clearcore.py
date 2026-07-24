"""LegacyClearCoreClient + belt adapter against the fake legacy firmware.

Pins the protocol behaviors that cost real debugging on the line (2026-07-22):
request-id dedup + cross-client seeding, the 16-bit distance cap, auto-pushed
params, the WORK_AT_ZERO edge chains (arrival / pass+re-approach), boarding
feed-cut on first STAGING rising, and two-phase staging with the belt nudge.
The fake emulates the controller; these tests poke the sensors as physics.
"""

from __future__ import annotations

import threading
import time

import pytest

pytest.importorskip("pymodbus.client")

from finishing_line.config.loader import ProductSpec, SandConfig
from finishing_line.core.model import Zone
from finishing_line.devices.legacy_belt import LegacyBeltAdapter
from finishing_line.devices.legacy_clearcore import (
    LegacyClearCoreClient,
    LegacyClearCoreError,
)
from finishing_line.process.sander import Sander
from finishing_line.sim.fake_legacy_clearcore import FakeLegacyClearCore

PORT = 15041
V = 6000  # test velocity: fast moves, still wide enough windows to poke sensors


@pytest.fixture()
def rig():
    fake = FakeLegacyClearCore(port=PORT).start()
    cc = LegacyClearCoreClient("127.0.0.1", port=PORT, poll_s=0.005,
                               invert_sensors={}).connect()
    cc.set_params(V, 60000)
    timers: list[threading.Timer] = []

    def poke(delay_s: float, sensor: str, value: bool) -> None:
        t = threading.Timer(delay_s, fake.set_sensor, args=(sensor, value))
        t.start()
        timers.append(t)

    yield fake, cc, poke
    for t in timers:
        t.cancel()
    cc.close()
    fake.stop()


def test_sensor_polarity_inversion_normalizes_mixed_fleets(rig):
    """An F18 replacement eye reads inverted (empty = HI); the driver
    normalizes per line-config so True always means part present — including
    inside the edge chains, which read through the same helper."""
    fake, _cc, _poke = rig
    cc2 = LegacyClearCoreClient("127.0.0.1", port=PORT, poll_s=0.005,
                                invert_sensors={"offload": True}).connect()
    try:
        # Raw LOW (F18 idle state on an inverted eye) => normalized "present"?
        # No: F18 empty = HI = raw True => normalized False. Part = LO => True.
        fake.set_sensor("offload", True)   # raw HI = F18 sees NOTHING
        assert cc2.read_inputs().offload is False
        fake.set_sensor("offload", False)  # raw LO = part present
        assert cc2.read_inputs().offload is True
        # Unlisted sensors stay active-high.
        fake.set_sensor("onload", True)
        assert cc2.read_inputs().onload is True
    finally:
        cc2.close()
        fake.set_sensor("offload", False)
        fake.set_sensor("onload", False)


def test_staging_eye_read_with_f18_polarity(rig):
    """Legacy v1.1's staging eye (discrete 7), normalized like every sensor:
    F18 polarity means raw HI = empty, raw LO = part parked at staging."""
    fake, _cc, _poke = rig
    cc2 = LegacyClearCoreClient("127.0.0.1", port=PORT, poll_s=0.005,
                                invert_sensors={"staging": True}).connect()
    try:
        fake.set_sensor("staging", True)    # raw HI = F18 sees nothing
        assert cc2.staging_present() is False
        fake.set_sensor("staging", False)   # raw LO = part present
        assert cc2.staging_present() is True
    finally:
        cc2.close()
        fake.set_sensor("staging", False)


def test_params_are_pushed_and_echoed(rig):
    fake, cc, _ = rig
    assert fake.holding[102] == V and fake.holding[103] == 60000
    # A fresh client auto-pushes tuned defaults before its first move — the
    # boot-velocity-0 trap (VelMax(0) moves nothing) is mitigated by design.
    cc2 = LegacyClearCoreClient("127.0.0.1", port=PORT, poll_s=0.005).connect()
    cc2.move_mm(10.0)
    assert fake.holding[102] > 0
    cc2.close()


def test_move_fires_only_on_request_id_change(rig):
    fake, cc, _ = rig
    cc.move_mm(20.0)
    assert fake.moves_started == 1
    # Same id re-written: the firmware must NOT move (its dedup) — and a fresh
    # client seeds its counter from the echo, so its move DOES fire.
    cc._write_reg_echoed(106, fake.holding[106])
    assert fake.moves_started == 1
    cc2 = LegacyClearCoreClient("127.0.0.1", port=PORT, poll_s=0.005).connect()
    cc2.move_mm(20.0)
    assert fake.moves_started == 2
    cc2.close()


def test_distance_overflow_guard(rig):
    _fake, cc, _ = rig
    with pytest.raises(LegacyClearCoreError, match="16-bit"):
        cc.move_mm(2500.0)


def test_move_blocks_until_complete(rig):
    fake, cc, _ = rig
    t0 = time.monotonic()
    cc.move_mm(200.0)  # 5999 steps / 6000 sps ~ 1.0 s
    elapsed = time.monotonic() - t0
    assert 0.8 < elapsed < 3.0
    assert not fake.moving()


def test_downstream_arrival_chain(rig):
    fake, cc, poke = rig
    fake.set_sensor("work_at_zero", True)   # O occupied at move start
    poke(0.3, "work_at_zero", False)        # departing part clears the eye
    poke(0.7, "work_at_zero", True)         # arriving part trips it
    res = cc.transition_move(50.0, stop_on_work_zero=True, o_occupied=True)
    assert res["arrived"] is True
    assert fake.holding[100] == 3, "belt must be idled by the arrival stop"
    assert res["seconds"] < 1.5, "stopped on the edge, not the distance cap"


def test_pass_through_retreat_with_reapproach(rig):
    fake, cc, poke = rig
    fake.set_sensor("work_at_zero", True)    # occupant on the eye
    poke(0.3, "work_at_zero", False)         # occupant departs upstream
    poke(0.6, "work_at_zero", True)          # arriver reaches the eye
    poke(0.9, "work_at_zero", False)         # arriver passes over (HI->LO)
    poke(1.4, "work_at_zero", True)          # re-approach lands back on it
    res = cc.transition_move(-50.0, stop_on_work_zero=True, o_occupied=True,
                             pass_through=True, reapproach_cap_mm=400.0)
    assert res["arrived"] is True
    assert fake.moves_started == 2, "main retreat + the re-approach nudge"
    assert fake.holding[100] == 3


def test_cap_reached_without_arrival_is_reported_not_raised(rig):
    _fake, cc, _ = rig
    res = cc.transition_move(50.0, stop_on_work_zero=True)  # never poke WZ
    assert res["arrived"] is False


def test_boarding_feed_cuts_on_first_rising_edge(rig):
    fake, cc, poke = rig
    poke(0.3, "staging", True)  # enterer's nose reaches the eye mid-move
    res = cc.transition_move(300.0, feed=True)
    assert res["entered"] is True
    assert fake.coils[107] == 0, "feed must be cut at the FIRST rising edge"


def test_stage_feed_only_never_moves_the_belt(rig):
    fake, cc, poke = rig
    poke(0.2, "staging", True)
    st = cc.stage_next(feed_timeout_s=2.0)
    assert st == {"staged": True, "nudged": False, "nudge_s": 0.0}
    assert fake.moves_started == 0, "phase A is feed-only: belt untouched"
    assert fake.coils[107] == 0


def test_stage_nudge_finishes_a_stalled_crossing(rig):
    fake, cc, poke = rig
    poke(0.6, "staging", True)  # fires only after phase A gives up at 0.3 s
    st = cc.stage_next(feed_timeout_s=0.3, nudge_timeout_s=5.0)
    assert st["staged"] is True and st["nudged"] is True
    assert st["nudge_s"] > 0
    assert fake.moves_started == 1, "the nudge is a capped belt move"
    assert fake.holding[100] == 3 and fake.coils[107] == 0, "belt idled, feed off"


def test_belt_adapter_runs_the_sander_composite(rig):
    fake, cc, _ = rig

    class RecUR:  # minimal UR recorder, same shape as tests/test_sander.py
        def __init__(self): self.log = []
        def move_to_named(self, wp): self.log.append(("move_to_named", wp))
        def contact_detect_z(self, speed_ms): self.log.append(("contact", speed_ms))
        def zero_ft(self, settle_s=0.1, pre_wait_s=0.0): self.log.append(("zero_ft",))
        def set_tool(self, on): self.log.append(("tool", on))
        def begin_force_z(self, newtons): self.log.append(("force", newtons))
        def end_force_mode(self, decel=None): self.log.append(("force_end",))
        def move_base_x_mm(self, d, a, v): self.log.append(("base_x", d))

    cfg = SandConfig(z_force_n=6.0, width_inset_mm=12, movel_a=0.5, movel_v=0.05,
                     contact_search_distance_m=1000.0, stopl_on_contact=3.0,
                     stopl_on_force_end=5.0, ft_wait_steady_ms=0)
    cube = ProductSpec(name="cube", legacy_job_id=1, width_mm=100, height_mm=90, depth_mm=90)
    ur = RecUR()
    Sander(ur, LegacyBeltAdapter(cc), cfg).sand_face(cube)
    # The composite's two belt passes became two real legacy distance moves.
    assert fake.moves_started == 2
    assert ("base_x", -90.0) in ur.log and ("base_x", 90.0) in ur.log
    assert not fake.moving()
