"""The fake ClearCore, exercised through the pymodbus CLIENT.

Deliberate pairing: the real ClearCoreClient driver will use this exact client
library, so every test here also verifies that our hand-rolled server framing
and pymodbus agree on the wire format. When the driver is written (Stage B), it
develops against this server before touching hardware — and the behaviour these
tests pin down is the spec for the real firmware changes.
"""

from __future__ import annotations

import time

import pytest

pymodbus_client = pytest.importorskip("pymodbus.client")

from finishing_line.devices.registers import Command, Echo, New, Status
from finishing_line.sim.fake_clearcore import (
    MODE_DISTANCE,
    MODE_IDLE,
    STATE_MOVING,
    STATE_READY,
    FakeClearCore,
)

PORT = 15020


@pytest.fixture(scope="module")
def fake():
    server = FakeClearCore(
        port=PORT,
        watchdog_timeout_s=0.4,
        shutter_actuation_s=0.1,
    ).start()
    yield server
    server.stop()


@pytest.fixture()
def client(fake):
    c = pymodbus_client.ModbusTcpClient("127.0.0.1", port=PORT, timeout=2.0)
    assert c.connect()
    yield c
    c.close()


def _await(predicate, timeout=2.0, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def test_echo_handshake(client):
    """The 200-block echo is the handshake the old program relied on and the
    new driver keeps: write a command, poll until the echo agrees.
    """
    client.write_register(Command.VELOCITY, 1600)
    assert _await(
        lambda: client.read_input_registers(Echo.VELOCITY, count=1).registers[0] == 1600
    ), "echo never confirmed the velocity write"

    client.write_coil(Command.DIRECTION, True)
    assert _await(lambda: client.read_discrete_inputs(Echo.DIRECTION, count=1).bits[0] is True)


def test_zone_distance_move_lifecycle(client):
    """A zone move is recognised only on a REQUEST_ID change, runs for
    distance/velocity seconds, and lands back at READY — the legacy lifecycle,
    per zone.
    """
    client.write_register(Command.VELOCITY, 10_000)
    client.write_register(New.ZONE1_DISTANCE, 2_000)          # 0.2 s at 10k steps/s
    client.write_register(New.ZONE1_MOTION_MODE, MODE_DISTANCE)

    # Writing mode + distance alone must NOT start a move.
    time.sleep(0.1)
    assert client.read_input_registers(New.ZONE1_STATE, count=1).registers[0] == STATE_READY

    client.write_register(New.ZONE1_REQUEST_ID, 41)
    assert _await(
        lambda: client.read_input_registers(New.ZONE1_STATE, count=1).registers[0] == STATE_MOVING
    ), "move never started after REQUEST_ID changed"
    assert _await(
        lambda: client.read_input_registers(New.ZONE1_STATE, count=1).registers[0] == STATE_READY
    ), "move never completed"

    # Same id again: no new move.
    client.write_register(New.ZONE1_REQUEST_ID, 41)
    time.sleep(0.1)
    assert client.read_input_registers(New.ZONE1_STATE, count=1).registers[0] == STATE_READY

    client.write_register(New.ZONE1_MOTION_MODE, MODE_IDLE)


def test_zones_are_independent(client):
    """Zone 2 idles while zone 1 moves — two motors, two blocks."""
    client.write_register(Command.VELOCITY, 5_000)
    client.write_register(New.ZONE1_DISTANCE, 1_500)
    client.write_register(New.ZONE1_MOTION_MODE, MODE_DISTANCE)
    client.write_register(New.ZONE1_REQUEST_ID, 7)

    assert _await(
        lambda: client.read_input_registers(New.ZONE1_STATE, count=1).registers[0] == STATE_MOVING
    )
    assert client.read_input_registers(New.ZONE2_STATE, count=1).registers[0] == STATE_READY
    assert _await(
        lambda: client.read_input_registers(New.ZONE1_STATE, count=1).registers[0] == STATE_READY
    )
    client.write_register(New.ZONE1_MOTION_MODE, MODE_IDLE)


def test_shutter_feedback_lags_command_through_moving(client):
    """Feedback is sensed position, not an echo: it passes through MOVING (2)
    during actuation. Zone motion gates on feedback, so the lag is load-bearing.
    """
    client.write_register(New.SHUTTER_CMD, 1)
    assert _await(
        lambda: client.read_input_registers(New.SHUTTER_FEEDBACK, count=1).registers[0] == 2
    ), "shutter never reported MOVING"
    assert _await(
        lambda: client.read_input_registers(New.SHUTTER_FEEDBACK, count=1).registers[0] == 1
    ), "shutter never confirmed OPEN"

    client.write_register(New.SHUTTER_CMD, 0)
    assert _await(
        lambda: client.read_input_registers(New.SHUTTER_FEEDBACK, count=1).registers[0] == 0
    )


def test_watchdog_forces_fans_on_and_recovers(client):
    """The §7 fail-ON contract: a silent orchestrator halts zones and forces
    both fans ON, so parts mid-flash keep drying. Heartbeat resuming clears it.
    """
    client.write_register(New.IF_FAN_CMD, 0)
    client.write_register(New.FD_FAN_CMD, 0)
    client.write_register(New.HEARTBEAT, 1)
    assert _await(
        lambda: client.read_input_registers(New.WATCHDOG_TRIPPED, count=1).registers[0] == 0
    )

    # Go silent past the timeout.
    assert _await(
        lambda: client.read_input_registers(New.WATCHDOG_TRIPPED, count=1).registers[0] == 1,
        timeout=2.0,
    ), "watchdog never tripped"
    fans = [
        client.read_input_registers(New.IF_FAN_FEEDBACK, count=1).registers[0],
        client.read_input_registers(New.FD_FAN_FEEDBACK, count=1).registers[0],
    ]
    assert fans == [1, 1], "fans must fail ON while tripped"

    # Heartbeat resumes -> trip clears, fans return to commanded (off).
    client.write_register(New.HEARTBEAT, 2)
    assert _await(
        lambda: client.read_input_registers(New.WATCHDOG_TRIPPED, count=1).registers[0] == 0
    ), "watchdog never recovered"
    assert _await(
        lambda: client.read_input_registers(New.IF_FAN_FEEDBACK, count=1).registers[0] == 0
    )


def test_presence_and_handoff_sensors_are_inputs(fake, client):
    """Sensors are physics, not controller state: the fake only reports what
    the harness pokes. This is the seam the Stage B harness drives.
    """
    fake.set_input(New.S_PRESENT, 1)
    fake.set_input(New.HANDOFF_TO_Z2, 1)
    assert _await(lambda: client.read_discrete_inputs(New.S_PRESENT, count=1).bits[0] is True)
    assert _await(lambda: client.read_discrete_inputs(New.HANDOFF_TO_Z2, count=1).bits[0] is True)
    fake.set_input(New.S_PRESENT, 0)
    fake.set_input(New.HANDOFF_TO_Z2, 0)
    assert _await(lambda: client.read_discrete_inputs(New.S_PRESENT, count=1).bits[0] is False)


def test_legacy_status_block_is_readable(client):
    """Rollback observability: the legacy addresses still answer."""
    assert client.read_input_registers(Status.SERVER_STATE, count=1).registers[0] == STATE_READY


def test_port_collision_fails_loudly(fake):
    """A second server on a held port must raise, not half-start. A silent
    bind failure leaves clients talking to whatever stale process owns the
    port — the worst possible failure mode to debug.
    """
    with pytest.raises(RuntimeError, match="cannot bind"):
        FakeClearCore(port=PORT).start()
