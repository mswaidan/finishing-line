"""Stage B integration: the full orchestrator stack over real Modbus.

Real state machine -> real Executor -> real ClearCoreClient -> real Modbus TCP
-> fake ClearCore firmware, with PhysicsSim playing reality (parts move only
because motor registers say belts run; sensors are the only way truth returns).
The robot is a FakeRobot; everything else is the production code path.

This is the test that catches what the pure-Python sim cannot: handshake
races, sensor-timing assumptions, executor threading, and the supervisor's
fan-feedback truth rule — all through the actual wire protocol.
"""

from __future__ import annotations

import time

import pytest

pytest.importorskip("pymodbus.client")

from finishing_line.config.loader import ProcessConfig
from finishing_line.core.model import LineState, PartRole, PartState, Product, Station
from finishing_line.core.schedule import Phase
from finishing_line.devices.clearcore import ClearCoreClient
from finishing_line.process.executor import Executor
from finishing_line.process.supervisor import Supervisor
from finishing_line.process.train import TrainMover
from finishing_line.sim.fake_clearcore import FakeClearCore
from finishing_line.sim.fake_robot import FakeRobot
from finishing_line.sim.physics import PhysicsSim

PORT = 15024


def _staged(*part_ids: str) -> LineState:
    parts = {
        pid: PartState(
            part_id=pid,
            product=Product.CUBE,
            role=PartRole.LEAD if i % 2 == 0 else PartRole.TRAIL,
            pair_index=i // 2,
        )
        for i, pid in enumerate(part_ids)
    }
    return LineState(parts=parts, inq_queue=tuple(part_ids))


@pytest.fixture()
def stack():
    """The full Stage B rig, torn down in reverse order."""
    fake = FakeClearCore(port=PORT, watchdog_timeout_s=3.0, shutter_actuation_s=0.05).start()
    cc = ClearCoreClient("127.0.0.1", port=PORT, poll_s=0.01).connect()
    robot = FakeRobot(work_s=0.2, spray_s=0.25, retract_s=0.05)
    executor = Executor(cc, robot, TrainMover(cc, timeout_s=10.0))
    physics = PhysicsSim(fake, transfer_s=0.25, tick_s=0.02)

    yield fake, cc, robot, executor, physics

    physics.stop()
    executor.close()
    cc.close()
    fake.stop()


#: Compressed process config: identical logic, wall-clock shrunk ~100x.
FAST = ProcessConfig(
    flash_seconds=1.2,
    coats=2,
    spray_burst_pause_s=0.25,
    transfer_s=0.25,
    robot_coat1_s=0.5,
    robot_coat2_s=0.5,
    denib_enabled=True,
    denib_duration_s=0.2,
    provenance={},
)


def test_two_parts_full_cycle_over_modbus(stack):
    """A lead/trail pair goes from INQ to OUT through the real stack.

    Every belt move here happened because the executor commanded continuous
    mode over Modbus, physics moved the part, sensors confirmed, and the
    machine reconciled — the complete production causal chain.
    """
    fake, cc, robot, executor, physics = stack
    state = _staged("L1", "T1")
    physics.inq_count = 2
    physics.start()

    sup = Supervisor(cc=cc, robot=robot, executor=executor, cfg=FAST, state=state, tick_hz=25.0)
    sup.run(until=lambda s: not s.parts and not s.occupancy, timeout_s=90.0)

    assert sup.state.fault is None, f"faulted: {sup.state.fault}"
    assert not sup.state.parts, "controller should have outfed both parts"
    # The final part coasts through the physics transit gap after its stop
    # edge — give phase B a moment to land it at OUT.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and physics.outfed < 2:
        time.sleep(0.05)
    assert physics.outfed == 2, f"physics saw {physics.outfed} parts leave, expected 2"
    assert physics.occupied == set(), "line should be physically empty"

    # The robot did both coats on both parts, in schedule order for the pair.
    coats = [entry for entry in robot.log if entry[0].startswith("spray")]
    assert coats == [
        ("spray1", "L1"), ("spray1", "T1"), ("spray2", "L1"), ("spray2", "T1"),
    ]


def test_flash_time_banks_only_while_fan_feedback_is_on(stack):
    """Supervisor truth rule: a part at a fan with the fan OFF banks nothing.

    Run the pair partway, then compare each part's banked flash against the
    guard threshold at the moment it left a fan — the machine's own guards
    enforce this, so reaching OUT without fault is itself the proof; here we
    additionally check no part ever outfed under-flashed.
    """
    fake, cc, robot, executor, physics = stack
    state = _staged("L1", "T1")
    physics.inq_count = 2
    physics.start()

    seen: dict[str, object] = dict(state.parts)
    outfed = []

    sup = Supervisor(cc=cc, robot=robot, executor=executor, cfg=FAST, state=state, tick_hz=25.0)

    def until(s):
        for pid in set(seen) - set(s.parts):
            outfed.append(seen[pid])
        seen.update(s.parts)
        return len(outfed) == 2

    sup.run(until=until, timeout_s=90.0)

    assert len(outfed) == 2, f"blocked: fault={sup.state.fault}"
    for part in outfed:
        assert part.flash_1_s >= FAST.flash_seconds, f"{part.part_id} under-flashed coat 1"
        assert part.flash_2_s >= FAST.flash_seconds, f"{part.part_id} under-flashed coat 2"


def test_robot_failure_faults_the_line_but_fans_survive(stack):
    """A mid-run device failure lands in the §7 posture: machine FAULTED,
    zones idle, fans still running for whatever is mid-flash.
    """
    fake, cc, robot, executor, physics = stack
    state = _staged("L1", "T1")
    physics.inq_count = 2
    physics.start()

    real_spray = robot.spray

    def failing_spray(part_id, coat):
        if coat == 2:
            raise RuntimeError("gun clogged")
        real_spray(part_id, coat)

    robot.spray = failing_spray

    sup = Supervisor(cc=cc, robot=robot, executor=executor, cfg=FAST, state=state, tick_hz=25.0)
    sup.run(until=lambda s: s.phase is Phase.FAULTED, timeout_s=90.0)

    assert sup.state.phase is Phase.FAULTED
    assert "gun clogged" in sup.state.fault
    # §7: a part was flashing when the gun died; its fan must still be running.
    from finishing_line.devices.registers import New

    assert fake.holding[New.ZONE1_MOTION_MODE] == 3, "zones must be idle"
    assert fake.holding[New.ZONE2_MOTION_MODE] == 3
    flashing_stations = [s for s in (Station.IF, Station.FD) if s in physics.occupied]
    if flashing_stations:
        fan_cmds = {Station.IF: New.IF_FAN_CMD, Station.FD: New.FD_FAN_CMD}
        assert any(
            fake.holding[fan_cmds[s]] == 1 for s in flashing_stations
        ), "no fan running over a mid-flash part after fault"
