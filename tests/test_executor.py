"""Executor semantics: FIFO order, fault poisoning, out-of-band halt.

Uses the fake ClearCore over real Modbus plus a FakeRobot, because the
executor's job is precisely the seam between intents and devices.
"""

from __future__ import annotations

import time

import pytest

pytest.importorskip("pymodbus.client")

from finishing_line.core.intents import (
    HaltZones,
    MoveToSafePose,
    SandPart,
    SetFan,
    SprayPart,
)
from finishing_line.core.model import FanState, Station
from finishing_line.devices.clearcore import ClearCoreClient
from finishing_line.devices.registers import New
from finishing_line.process.executor import Executor
from finishing_line.process.train import TrainMover
from finishing_line.sim.fake_clearcore import MODE_IDLE, FakeClearCore
from finishing_line.sim.fake_robot import FakeRobot

PORT = 15023


@pytest.fixture(scope="module")
def fake():
    server = FakeClearCore(port=PORT, shutter_actuation_s=0.05).start()
    yield server
    server.stop()


@pytest.fixture()
def cc(fake):
    client = ClearCoreClient("127.0.0.1", port=PORT, poll_s=0.01).connect()
    yield client
    client.close()


@pytest.fixture()
def rig(cc):
    robot = FakeRobot(work_s=0.05, spray_s=0.05, retract_s=0.01)
    executor = Executor(cc, robot, TrainMover(cc, timeout_s=2.0))
    yield executor, robot
    executor.close()


def _drain(executor, want: int, timeout=5.0) -> set[str]:
    done: set[str] = set()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and len(done) < want:
        done |= executor.completed()
        time.sleep(0.01)
    return done


def test_batch_executes_in_order(fake, rig):
    """The P3 bracket: fan OFF, spray, fan ON — order is the guarantee."""
    executor, robot = rig
    batch = (
        SetFan(station=Station.IF, state=FanState.OFF),
        SprayPart(part_id="p1", coat=2),
        SetFan(station=Station.IF, state=FanState.ON),
        MoveToSafePose(),
    )
    executor.submit(batch)
    done = _drain(executor, want=4)

    assert {i.intent_id for i in batch} == done
    assert robot.log == [("spray2", "p1"), ("safe_pose", "")]
    # After the ordered batch, the fan command must have landed back ON.
    assert fake.holding[New.IF_FAN_CMD] == 1


def test_device_failure_poisons_and_halts(fake, cc, rig):
    executor, robot = rig

    def explode(part_id):
        raise RuntimeError("spindle jam")

    robot.sand = explode
    executor.submit((SandPart(part_id="p1"), MoveToSafePose()))

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and executor.fault_reason() is None:
        time.sleep(0.01)

    assert "spindle jam" in executor.fault_reason()
    # Nothing after the failure ran, and nothing completed.
    time.sleep(0.2)
    assert executor.completed() == frozenset()
    assert robot.log == [], "work after the failure must not run"
    # Zones were idled by the halt.
    assert fake.holding[New.ZONE1_MOTION_MODE] == MODE_IDLE
    assert fake.holding[New.ZONE2_MOTION_MODE] == MODE_IDLE

    executor.reset()
    assert executor.fault_reason() is None


def test_halt_zones_jumps_the_queue(fake, rig):
    """A halt must not wait behind a long job: it acts on the calling thread."""
    executor, robot = rig
    robot.work_s = 1.0  # long sand job occupies the worker
    executor.submit((SandPart(part_id="p1"),))
    time.sleep(0.1)  # worker is now inside the sand call

    t0 = time.monotonic()
    executor.submit((HaltZones(reason="e-stop"),))
    halt_latency = time.monotonic() - t0

    assert halt_latency < 0.5, f"halt took {halt_latency:.2f}s — it queued behind the job"
    assert executor.fault_reason() == "e-stop"
    assert fake.holding[New.ZONE1_MOTION_MODE] == MODE_IDLE
    # Fans were NOT touched by the halt (§7: parts keep drying).


def test_fans_untouched_by_halt(fake, cc, rig):
    executor, _robot = rig
    cc.set_fan(Station.FD, True)
    time.sleep(0.05)
    executor.halt("test halt")
    time.sleep(0.05)
    assert fake.holding[New.FD_FAN_CMD] == 1, "halt must not stop a running fan"
    executor.reset()
