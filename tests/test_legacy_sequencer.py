"""LegacySequencer/-Controller orchestration — deterministic, no hardware.

The belt layer is proven separately (test_legacy_clearcore.py + the real line,
twice); these tests inject a FakeTrain and verify the ORCHESTRATION: beat
order, direct-entry occupancy flow, per-part flash gating (never under-flash),
the OFFLOAD gate, fan modes incl. the P3 spray pause, staging failure, halt,
restart confirm-or-clear, and the lone-part drain.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from finishing_line.config.loader import FanMode, ProcessConfig
from finishing_line.core.model import Station
from finishing_line.process.legacy_controller import LegacyController
from finishing_line.process.legacy_sequencer import (
    PHASE_FAULTED,
    PHASE_IDLE,
    LegacySequencer,
)
from finishing_line.sim.fake_robot import FakeRobot

CFG = ProcessConfig(
    flash_seconds=0.15, coats=2, spray_burst_pause_s=0.0, transfer_s=0.0,
    robot_coat1_s=0.1, robot_coat2_s=0.1, clean_gun_enabled=True,
    clean_gun_duration_s=0.1, provenance={},
)
ALWAYS_ON = {"f1": FanMode("always_on"), "f2": FanMode("always_on")}


class FakeTrain:
    def __init__(self) -> None:
        self.log: list[str] = []
        self.offload = False
        self.arrive = True
        self.available = 0  # physical parts at the infeed (the sensors' truth)
        self.stage_result = {"staged": True, "nudged": False, "nudge_s": 0.0}
        self.last_spacing_mm = 700.0

    def _res(self):
        return {"arrived": self.arrive, "entered": None, "seconds": 0.01}

    def load(self):
        self.log.append("load")
        return self._res()

    def stage(self, **_kw):
        self.log.append("stage")
        res = dict(self.stage_result)
        res["staged"] = res["staged"] and self.available > 0
        if res["staged"]:
            self.available -= 1
        return res

    def entry(self, *, o_occupied, feed_assist=False):
        self.log.append(f"entry(assist={feed_assist})")
        return self._res()

    def retreat(self, *, o_occupied):
        self.log.append("retreat")
        return self._res()

    def return_to_o(self, *, o_occupied):
        self.log.append("return")
        return self._res()

    def blind(self):
        self.log.append("blind")

    def idle(self):
        self.log.append("idle")

    def sensors(self):
        return SimpleNamespace(server_state=1, work_at_zero=True,
                               onload=False, offload=self.offload)


def make_seq(fans=ALWAYS_ON, fan_do=None, robot=None, train=None,
             continuous=True, parts_available=99):
    train = train or FakeTrain()
    train.available = parts_available
    robot = robot or FakeRobot(work_s=0.005, spray_s=0.005, retract_s=0.0)
    seq = LegacySequencer(train, robot, CFG, fans, fan_do=fan_do, tick_s=0.005,
                          continuous_intake=continuous)
    return seq, train, robot


def run_until(seq, predicate, timeout_s=15.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        blocked = seq.step()
        if predicate(seq, blocked):
            return blocked
    raise AssertionError(f"condition never reached; phase={seq.phase} "
                         f"fault={seq.fault} occ={seq.occ}")


def test_four_part_soak_completes_in_order_never_underflashed():
    seq, train, robot = make_seq(parts_available=4)
    seq.declare_batch("cube", ["L1", "T1", "L2", "T2"])
    run_until(seq, lambda s, b: len(s.completed) == 4 and s.phase == PHASE_IDLE,
              timeout_s=30.0)
    assert seq.completed == ["L1", "T1", "L2", "T2"]
    for pid in ("L1", "T1", "L2", "T2"):
        p = seq.parts[pid]
        assert p.coats_applied == 2
        assert p.flash_1_s >= CFG.flash_seconds, f"{pid} under-flashed coat 1"
        assert p.flash_2_s >= CFG.flash_seconds, f"{pid} under-flashed coat 2"
    coats = [e for e in robot.log if e[0].startswith("spray")]
    assert coats == [
        ("spray1", "L1"), ("spray1", "T1"), ("spray2", "L1"), ("spray2", "T1"),
        ("spray1", "L2"), ("spray1", "T2"), ("spray2", "L2"), ("spray2", "T2"),
    ]
    # Direct entry everywhere: idle intake + 3 in-pattern entries, no load.
    assert train.log.count("load") == 0
    assert sum(1 for e in train.log if e.startswith("entry")) == 4
    assert train.log.count("retreat") == 2 and train.log.count("return") == 2


def test_flash_gate_blocks_the_retreat_until_banked():
    seq, train, _robot = make_seq(parts_available=2)
    seq.declare_batch("cube", ["L1", "T1"])
    blocked = run_until(seq, lambda s, b: b is not None and "flash" in b)
    assert "L1" in blocked and "F2" in blocked
    assert "retreat" not in train.log, "retreat must wait for flash-1"
    run_until(seq, lambda s, b: "retreat" in train.log)


def test_offload_gate_blocks_the_exit():
    seq, train, _robot = make_seq()
    train.offload = True  # a finished part sits unremoved at OUT
    seq.declare_batch("cube", ["L1", "T1"])
    blocked = run_until(seq, lambda s, b: b is not None and "remove" in b)
    assert "OUT" in blocked
    before = len(seq.completed)
    train.offload = False  # operator takes the part
    run_until(seq, lambda s, b: len(s.completed) > before)


def test_p3_spray_pause_with_controllable_f1_fan():
    fan_log: list[tuple[str, int, bool]] = []
    robot = FakeRobot(work_s=0.005, spray_s=0.005, retract_s=0.0)

    def fan_do(do: int, on: bool) -> None:
        fan_log.append(("fan", do, on))
        robot.log.append((f"fan_{'on' if on else 'off'}", str(do)))

    fans = {"f1": FanMode("robot_do", do=2), "f2": FanMode("always_on")}
    seq, train, robot = make_seq(fans=fans, fan_do=fan_do, robot=robot, parts_available=2)
    seq.declare_batch("cube", ["L1", "T1"])
    run_until(seq, lambda s, b: len(s.completed) == 2 and s.phase == PHASE_IDLE,
              timeout_s=30.0)
    log = robot.log
    spray2_l1 = log.index(("spray2", "L1"))  # the P3 beat: wet T1 at F1
    assert ("fan_off", "2") in log[:spray2_l1], "F1 fan must pause before the burst"
    assert ("fan_on", "2") in log[spray2_l1:], "and resume after it"
    assert all(entry[1] == 2 for entry in fan_log)


def test_always_on_fans_never_toggle():
    fan_log: list = []
    seq, _train, _robot = make_seq(fan_do=lambda do, on: fan_log.append((do, on)), parts_available=2)
    seq.declare_batch("cube", ["L1", "T1"])
    run_until(seq, lambda s, b: len(s.completed) == 2, timeout_s=30.0)
    assert fan_log == []


def test_staging_failure_faults_and_idles_the_belt():
    # Batch mode: declared parts that never appear ARE a fault. (Continuous
    # mode skips instead — covered by the starved-line test.)
    seq, train, _robot = make_seq(continuous=False, parts_available=2)
    train.stage_result = {"staged": False, "nudged": True, "nudge_s": 1.0}
    seq.declare_batch("cube", ["L1", "T1"])
    run_until(seq, lambda s, b: s.phase == PHASE_FAULTED)
    assert "not found at the infeed" in seq.fault or "staging failed" in seq.fault
    assert train.log[-1] == "idle"


def test_lone_part_drains_with_both_coats():
    seq, train, _robot = make_seq(parts_available=1)
    seq.declare_batch("cube", ["L1"])
    run_until(seq, lambda s, b: s.completed == ["L1"] and s.phase == PHASE_IDLE,
              timeout_s=30.0)
    p = seq.parts["L1"]
    assert p.coats_applied == 2
    assert p.flash_1_s >= CFG.flash_seconds and p.flash_2_s >= CFG.flash_seconds
    assert "retreat" in train.log, "lone part still retreats for coat 2"


def test_continuous_intake_mints_beat_derived_identities():
    """No batching: parts appear because the sensors deliver them. Identities
    are minted, roles come from the beat (idle/P4 entry = LEAD, P1 = TRAIL)."""
    seq, _train, _robot = make_seq(parts_available=2)
    run_until(seq, lambda s, b: len(s.completed) == 2 and s.phase == PHASE_IDLE,
              timeout_s=30.0)
    assert seq.completed == ["c0001", "c0002"]
    assert str(seq.parts["c0001"].role) == "lead"
    assert str(seq.parts["c0002"].role) == "trail"
    assert all(str(p.product) == "cube" for p in seq.parts.values())
    assert seq.fault is None


def test_starved_line_skips_drains_and_resumes():
    """Skip-and-drain: an empty infeed at an entry beat skips the entry (no
    fault), in-flight parts finish, and the line resumes when parts appear."""
    seq, train, _robot = make_seq(parts_available=1)
    run_until(seq, lambda s, b: s.completed == ["c0001"] and s.phase == PHASE_IDLE,
              timeout_s=30.0)
    assert seq.fault is None, "starvation must never fault"
    p = seq.parts["c0001"]
    assert p.coats_applied == 2
    assert p.flash_1_s >= CFG.flash_seconds and p.flash_2_s >= CFG.flash_seconds
    # Parts arrive later in the day: the idle intake picks them up.
    train.available = 2
    run_until(seq, lambda s, b: len(s.completed) == 3 and s.phase == PHASE_IDLE,
              timeout_s=30.0)
    assert seq.completed == ["c0001", "c0002", "c0003"]
    assert seq.fault is None


def test_intake_product_switch_changes_minted_parts():
    seq, train, _robot = make_seq(parts_available=1)
    run_until(seq, lambda s, b: len(s.completed) == 1, timeout_s=30.0)
    seq.set_intake_product("browser")
    train.available = 1
    run_until(seq, lambda s, b: len(s.completed) == 2 and s.phase == PHASE_IDLE,
              timeout_s=30.0)
    assert seq.completed[1] == "b0002"
    assert str(seq.parts["b0002"].product) == "browser"


def test_controller_halt_at_boundary_and_snapshot_shape(tmp_path):
    seq, train, _robot = make_seq(parts_available=2)
    ctl = LegacyController(seq, CFG, state_file=str(tmp_path / "s.json")).start()
    try:
        ctl.declare_batch("cube", ["L1", "T1"])
        ctl.set_running(True)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not seq.occ:
            time.sleep(0.01)
        assert seq.occ, "line never started"
        ctl.halt("test halt")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and seq.phase != PHASE_FAULTED:
            time.sleep(0.01)
        assert seq.phase == PHASE_FAULTED and "test halt" in seq.fault
        snap = ctl.snapshot()
        assert snap["mode"] == "legacy"
        assert snap["clearcore"]["shutter"] == "NONE"
        assert set(snap["clearcore"]["sensors"]) == {"F1", "O", "F2", "IN", "OUT", "in_count"}
        assert snap["config"]["flash_seconds"] == CFG.flash_seconds
        assert snap["fault"] == snap["blocked_by"]
    finally:
        ctl.close()


def test_restart_with_parts_faults_then_confirm_or_clear(tmp_path):
    state = str(tmp_path / "s.json")
    seq, _train, _robot = make_seq()
    ctl = LegacyController(seq, CFG, state_file=state)
    ctl.declare_batch("cube", ["L1", "T1"])
    seq.occ[Station.O] = seq.queue.pop(0)  # simulate mid-run
    seq.beat = "P2"
    ctl._save()

    # Restart #1: confirm occupancy -> resumes at the persisted beat.
    seq2, _t2, _r2 = make_seq()
    ctl2 = LegacyController(seq2, CFG, state_file=state)
    assert seq2.phase == PHASE_FAULTED and "restarted" in seq2.fault
    ok, reason = ctl2.ack_fault({"o": "L1"}, beat="P2")
    assert ok, reason
    assert seq2.occ == {Station.O: "L1"} and seq2.beat == "P2"
    assert seq2.phase not in (PHASE_FAULTED, PHASE_IDLE)

    # Restart #2: operator cleared the belt -> empty ack clears the line.
    seq3, _t3, _r3 = make_seq()
    ctl3 = LegacyController(seq3, CFG, state_file=state)
    assert seq3.phase == PHASE_FAULTED
    ok, _ = ctl3.ack_fault(None)
    assert ok
    assert seq3.phase == PHASE_IDLE and not seq3.occ
