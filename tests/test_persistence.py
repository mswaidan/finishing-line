"""State persistence: round-trip fidelity, restore-as-fault, crash survival.

The finale is the test that matters: kill the whole orchestrator stack with a
part mid-flash, build a fresh one from the snapshot, and finish the batch
through the standard operator confirm-and-resume flow.
"""

from __future__ import annotations

import json
import time

import pytest

from finishing_line.core.model import FanState, LineState, PartRole, Station
from finishing_line.core.schedule import Phase
from finishing_line.process.persistence import (
    StateStore,
    as_restored,
    deserialize_state,
    serialize_state,
)

from .conftest import make_part


def _sample_state() -> LineState:
    lead = make_part("L1", PartRole.LEAD, coats_applied=1, flash_1_s=42.5, is_wet=True)
    trail = make_part("T1", PartRole.TRAIL)
    return LineState(
        parts={"L1": lead, "T1": trail},
        occupancy={Station.F2: "L1"},
        in_queue=("T1",),
        beat="P2",
        phase=str(Phase.AWAIT_GUARDS),
        pair_index=3,
        f2_fan=FanState.ON,
    )


def test_round_trip_preserves_everything_that_matters():
    state = _sample_state()
    restored, declared = deserialize_state(
        json.loads(json.dumps(serialize_state(state, declared=7)))
    )
    assert declared == 7
    assert restored.beat == "P2"
    assert restored.pair_index == 3
    assert restored.occupancy == {Station.F2: "L1"}
    assert restored.in_queue == ("T1",)
    part = restored.parts["L1"]
    assert part.flash_1_s == 42.5
    assert part.coats_applied == 1 and part.is_wet
    assert restored.parts["T1"].role is PartRole.TRAIL


def test_restore_with_parts_is_a_fault_with_correct_reentry():
    """The saved phase becomes fault_phase, so machine.resume() re-enters at
    the right point relative to the beat's train move — same §7 machinery as
    any other fault.
    """
    restored = as_restored(_sample_state())
    assert restored.phase == str(Phase.FAULTED)
    assert "restarted" in restored.fault
    assert restored.fault_phase == str(Phase.AWAIT_GUARDS)
    assert restored.pending == () and restored.in_flight == ()


def test_restore_of_faulted_snapshot_keeps_its_own_fault():
    state = _sample_state()
    faulted = LineState(
        parts=state.parts, occupancy=state.occupancy, in_queue=state.in_queue,
        beat="P2", phase=str(Phase.FAULTED), fault="gun clogged",
        fault_phase=str(Phase.ROBOT_WORK),
    )
    restored = as_restored(faulted)
    assert restored.fault == "gun clogged"
    assert restored.fault_phase == str(Phase.ROBOT_WORK)


def test_restore_of_empty_line_needs_no_ceremony():
    restored = as_restored(LineState(beat="P3", phase=str(Phase.SET_FANS)))
    assert restored.fault is None
    assert restored.phase == str(Phase.ROBOT_WORK)


def test_store_save_load_and_corrupt_handling(tmp_path):
    store = StateStore(tmp_path / "state.json")
    assert store.load() is None

    store.save(_sample_state(), declared=4)
    loaded = store.load()
    assert loaded is not None and loaded[1] == 4

    (tmp_path / "state.json").write_text("{ not json", encoding="utf-8")
    assert store.load() is None, "corrupt snapshot must mean fresh start, not crash"
    assert (tmp_path / "state.corrupt").exists(), "the evidence is preserved"


def test_throttle_saves_structural_changes_immediately(tmp_path):
    store = StateStore(tmp_path / "state.json", min_interval_s=60.0)
    state = _sample_state()
    store.save(state, 0)

    # Timer-only churn inside the interval: no save.
    from dataclasses import replace

    ticked = replace(
        state, parts={**state.parts, "L1": replace(state.parts["L1"], flash_1_s=43.0)}
    )
    assert store.maybe_save(ticked, 0) is False

    # A beat change is structural: saves despite the interval.
    assert store.maybe_save(replace(ticked, beat="P3"), 0) is True


@pytest.mark.usefixtures()
def test_restart_mid_flash_and_finish_the_batch(tmp_path):
    """The crash the feature exists for: orchestrator dies with a part
    mid-flash; a NEW stack restores from disk, the operator confirms
    occupancy through the normal flow, and the batch completes — with the
    restored part's banked flash time carried across the gap.
    """
    pytest.importorskip("pymodbus.client")
    from finishing_line.config.loader import ProcessConfig
    from finishing_line.devices.clearcore import ClearCoreClient
    from finishing_line.process.controller import LineController
    from finishing_line.process.executor import Executor
    from finishing_line.process.supervisor import Supervisor
    from finishing_line.process.train import TrainMover
    from finishing_line.sim.fake_clearcore import FakeClearCore
    from finishing_line.sim.fake_robot import FakeRobot
    from finishing_line.sim.physics import PhysicsSim

    cfg = ProcessConfig(
        flash_seconds=1.5, coats=2, spray_burst_pause_s=0.25, transfer_s=0.25,
        robot_coat1_s=0.5, robot_coat2_s=0.5, denib_enabled=True,
        denib_duration_s=0.2, provenance={},
    )
    store_path = tmp_path / "state.json"
    fake = FakeClearCore(port=15027, watchdog_timeout_s=5.0, shutter_actuation_s=0.05).start()
    physics = PhysicsSim(fake, transfer_s=0.25, tick_s=0.02).start()

    def build_stack():
        cc = ClearCoreClient("127.0.0.1", port=15027, poll_s=0.01).connect()
        robot = FakeRobot(work_s=0.2, spray_s=0.25, retract_s=0.05)
        executor = Executor(cc, robot, TrainMover(cc, timeout_s=10.0))
        sup = Supervisor(cc=cc, robot=robot, executor=executor, cfg=cfg, state=LineState())
        ctl = LineController(sup, executor, tick_hz=25.0,
                             store=StateStore(store_path, min_interval_s=0.2))
        return cc, executor, sup, ctl

    # --- first life: declare, run, wait until a part is flashing, then die.
    cc1, ex1, sup1, ctl1 = build_stack()
    ctl1.declare_batch("cube", ["L1", "T1"])
    physics.in_count = 2
    ctl1.start()
    ctl1.set_running(True)

    deadline = time.monotonic() + 60.0
    banked = 0.0
    while time.monotonic() < deadline:
        parts = sup1.state.parts
        flashing = [p for p in parts.values() if p.flash_1_s > 0.3]
        if flashing:
            banked = flashing[0].flash_1_s
            break
        time.sleep(0.05)
    assert banked > 0.3, "no part ever started flashing"

    # Hard stop: no graceful close-save; whatever hit disk is what survives.
    ctl1._stop.set()
    ctl1._thread.join(timeout=5.0)
    ex1.close()
    cc1.close()

    # --- second life: fresh stack restores from the snapshot.
    cc2, ex2, sup2, ctl2 = build_stack()
    try:
        assert sup2.state.phase == str(Phase.FAULTED), "restore with parts must fault"
        assert "restarted" in sup2.state.fault
        restored_part = next(p for p in sup2.state.parts.values() if p.flash_1_s > 0)
        assert restored_part.flash_1_s <= banked + 0.5, "timers must not inflate"
        assert restored_part.flash_1_s > 0, "banked flash must survive the crash"

        ctl2.start()
        ctl2.set_running(True)
        # Operator flow: confirm the controller's own occupancy (the physics
        # world never stopped, so belief and sensors still agree).
        resumed, reason = ctl2.ack_fault(
            {st.name: pid for st, pid in sup2.state.occupancy.items()}
        )
        assert resumed, f"resume rejected: {reason}"

        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline and sup2.state.parts:
            time.sleep(0.1)
        assert not sup2.state.parts, (
            f"batch did not finish after restart: {ctl2.snapshot()['blocked_by']}"
        )
        assert sup2.state.fault is None
    finally:
        ctl2.close()
        ex2.close()
        cc2.close()
        physics.stop()
        fake.stop()
