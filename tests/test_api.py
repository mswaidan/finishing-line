"""API layer over the full sim stack — effectively the Stage C smoke test.

The TestClient drives the same FastAPI app operators will use; behind it sit
the real controller, supervisor thread, executor, ClearCoreClient, Modbus,
fake firmware, and physics. Batch declaration, run, and fault recovery all
travel the full HTTP -> device round trip.
"""

from __future__ import annotations

import time

import pytest

pytest.importorskip("fastapi")
pytest.importorskip("pymodbus.client")

from fastapi.testclient import TestClient

from finishing_line.api.app import create_app
from finishing_line.config.loader import ProcessConfig
from finishing_line.core.model import LineState
from finishing_line.devices.clearcore import ClearCoreClient
from finishing_line.process.controller import LineController
from finishing_line.process.executor import Executor
from finishing_line.process.supervisor import Supervisor
from finishing_line.process.train import TrainMover
from finishing_line.sim.fake_clearcore import FakeClearCore
from finishing_line.sim.fake_robot import FakeRobot
from finishing_line.sim.physics import PhysicsSim

PORT = 15025

FAST = ProcessConfig(
    flash_seconds=1.2, coats=2, spray_burst_pause_s=0.25, transfer_s=0.25,
    robot_coat1_s=0.5, robot_coat2_s=0.5, denib_enabled=True,
    denib_duration_s=0.2, provenance={"flash_seconds": "assumed"},
)


@pytest.fixture()
def rig():
    fake = FakeClearCore(port=PORT, watchdog_timeout_s=3.0, shutter_actuation_s=0.05).start()
    cc = ClearCoreClient("127.0.0.1", port=PORT, poll_s=0.01).connect()
    robot = FakeRobot(work_s=0.2, spray_s=0.25, retract_s=0.05)
    executor = Executor(cc, robot, TrainMover(cc, timeout_s=10.0))
    physics = PhysicsSim(fake, transfer_s=0.25, tick_s=0.02).start()
    supervisor = Supervisor(cc=cc, robot=robot, executor=executor, cfg=FAST, state=LineState())
    controller = LineController(supervisor, executor, tick_hz=25.0).start()

    # Sim-mode rule: declared parts appear on the fake infeed too.
    orig = controller.declare_batch

    def declare_and_feed(product, part_ids):
        staged = orig(product, part_ids)
        physics.inq_count += len(staged)
        return staged

    controller.declare_batch = declare_and_feed
    client = TestClient(create_app(controller))

    yield client, controller, physics, robot

    controller.close()
    physics.stop()
    executor.close()
    cc.close()
    fake.stop()


def _await(predicate, timeout=60.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def test_hmi_and_state_served(rig):
    client, *_ = rig
    page = client.get("/")
    assert page.status_code == 200 and "FINISHING LINE" in page.text

    state = client.get("/state").json()
    assert state["enabled"] is False
    assert state["beat"] == "P1"
    assert state["config"]["unmeasured"] == ["flash_seconds"]


def test_batch_validation(rig):
    client, *_ = rig
    assert client.post("/batch", json={"product": "chair", "count": 2}).status_code == 422
    assert client.post("/batch", json={"product": "cube"}).status_code == 422

    ok = client.post("/batch", json={"product": "cube", "part_ids": ["a", "b"]})
    assert ok.status_code == 200 and ok.json()["staged"] == ["a", "b"]

    dup = client.post("/batch", json={"product": "cube", "part_ids": ["a"]})
    assert dup.status_code == 409


def test_full_cycle_driven_over_http(rig):
    """Operator flow end to end: declare a pair, press Run, watch it finish."""
    client, controller, physics, _robot = rig

    staged = client.post("/batch", json={"product": "cube", "count": 2}).json()["staged"]
    assert len(staged) == 2
    state = client.get("/state").json()
    assert state["parts"][staged[0]]["role"] == "lead"
    assert state["parts"][staged[1]]["role"] == "trail"

    assert client.post("/run", json={"enabled": True}).status_code == 200

    def finished():
        s = client.get("/state").json()
        return not s["parts"] and s["fault"] is None

    assert _await(finished, timeout=90.0), (
        f"line did not finish: {client.get('/state').json()['blocked_by']}"
    )
    assert physics.outfed == 2


def test_pause_holds_schedule_but_timers_run(rig):
    client, controller, physics, _robot = rig
    client.post("/batch", json={"product": "cube", "count": 2})
    client.post("/run", json={"enabled": True})

    # Wait until a part is flashing somewhere, then pause.
    def flashing():
        s = client.get("/state").json()
        return any(p["coats"] >= 1 and p["station"] in ("IF", "FD")
                   for p in s["parts"].values())
    assert _await(flashing, timeout=60.0), "no part reached a fan"

    client.post("/run", json={"enabled": False})
    # Pause takes effect at the beat boundary: the current transition finishes
    # (so fans are set for every occupied station), then the machine holds.
    assert _await(
        lambda: client.get("/state").json()["phase"] == "robot_work", timeout=30.0
    ), "machine never reached the pause hold point"
    time.sleep(0.2)
    before = client.get("/state").json()
    time.sleep(0.6)
    after = client.get("/state").json()

    assert after["beat"] == before["beat"], "schedule advanced while held"
    assert after["phase"] == "robot_work", "machine left the hold point while paused"
    flashed = [
        (after["parts"][pid]["flash_1_s"] + after["parts"][pid]["flash_2_s"])
        - (p["flash_1_s"] + p["flash_2_s"])
        for pid, p in before["parts"].items()
        if pid in after["parts"] and after["parts"][pid]["station"] in ("IF", "FD")
    ]
    assert flashed, "expected a part at a fan station at the hold point"
    assert any(d > 0.3 for d in flashed), "flash timers must keep banking while paused"


def test_halt_then_ack_resumes(rig):
    client, controller, physics, _robot = rig
    client.post("/batch", json={"product": "cube", "count": 2})
    client.post("/run", json={"enabled": True})

    def working():
        return client.get("/state").json()["parts"] and any(
            p["station"] in ("S", "IF", "FD")
            for p in client.get("/state").json()["parts"].values()
        )
    assert _await(working, timeout=60.0)

    assert client.post("/halt", json={"reason": "test stop"}).status_code == 200
    assert _await(lambda: client.get("/state").json()["fault"] is not None, timeout=10.0)
    assert "test stop" in client.get("/state").json()["fault"]

    # Ack with the controller's own occupancy (nothing physically moved).
    occupancy = client.get("/state").json()["occupancy"]
    ack = client.post("/fault/ack", json={"occupancy": occupancy})
    assert ack.status_code == 200, ack.text

    def finished():
        s = client.get("/state").json()
        return not s["parts"] and s["fault"] is None
    assert _await(finished, timeout=90.0), (
        f"did not finish after resume: {client.get('/state').json()['blocked_by']}"
    )


def test_ack_when_not_faulted_is_rejected(rig):
    client, *_ = rig
    assert client.post("/fault/ack", json={}).status_code == 409


def test_websocket_streams_state(rig):
    client, *_ = rig
    with client.websocket_connect("/events") as ws:
        first = ws.receive_json()
        assert first["beat"] == "P1"
        second = ws.receive_json()
        assert "parts" in second
