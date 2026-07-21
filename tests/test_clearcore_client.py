"""ClearCoreClient driver against the fake ClearCore — the Stage B pairing.

This is real driver code speaking real Modbus TCP to the executable register-map
spec. When the firmware is written, these tests describe exactly what the driver
expects of it.
"""

from __future__ import annotations

import time

import pytest

pytest.importorskip("pymodbus.client")

from finishing_line.config.loader import load_conveyor_kinematics
from finishing_line.core.model import ShutterState, Station, Zone
from finishing_line.devices.clearcore import ClearCoreClient, ClearCoreError
from finishing_line.devices.registers import Command, New
from finishing_line.sim.fake_clearcore import MODE_IDLE, STATE_READY, FakeClearCore

PORT = 15021


@pytest.fixture(scope="module")
def fake():
    server = FakeClearCore(port=PORT, watchdog_timeout_s=0.4, shutter_actuation_s=0.1).start()
    yield server
    server.stop()


@pytest.fixture()
def cc(fake):
    client = ClearCoreClient("127.0.0.1", port=PORT, poll_s=0.01).connect()
    # Fast fake moves: high velocity so distance moves finish in ~tens of ms.
    client._write_register(Command.VELOCITY, 50_000)
    yield client
    client.close()


def test_connect_refused_is_loud():
    with pytest.raises(ClearCoreError, match="cannot reach"):
        ClearCoreClient("127.0.0.1", port=1, timeout_s=0.3).connect()


def test_move_uses_the_tuned_mm_to_steps_conversion(fake, cc):
    """362mm (cube width) must command floor(362 * 29.9962) = 10858 steps —
    the legacy conversion, floor and all. If this drifts, parts land in the
    wrong place while every unit test stays green.
    """
    steps = cc.move_zone_mm(Zone.Z1, 362.0)
    kin = load_conveyor_kinematics()
    assert steps == kin.mm_to_steps(362.0) == 10858
    assert fake.holding[New.Z1_DIST] == 10858
    cc.wait_zone_ready(Zone.Z1)


def test_negative_distance_travels_on_the_direction_coil(fake, cc):
    """Registers are unsigned; sign is the coil. -100mm = coil False + abs steps."""
    steps = cc.move_zone_mm(Zone.Z2, -100.0)
    assert steps > 0
    assert fake.coils[New.Z2_DIR] == 0
    cc.wait_zone_ready(Zone.Z2)

    cc.move_zone_mm(Zone.Z2, 100.0)
    assert fake.coils[New.Z2_DIR] == 1
    cc.wait_zone_ready(Zone.Z2)


def test_move_lifecycle_ack_then_ready(fake, cc):
    """The race the ack registers exist to close: move_zone_mm returns only
    after the firmware acknowledged THIS move, so a following wait_zone_ready
    can never mistake not-yet-started for done.
    """
    for _ in range(5):  # repeat: the race is timing-dependent by nature
        cc.move_zone_mm(Zone.Z1, 50.0)
        cc.wait_zone_ready(Zone.Z1, timeout_s=5.0)
    assert fake.input_regs[New.Z1_STATE] == STATE_READY


def test_continuous_and_idle_modes(fake, cc):
    """The handoff manoeuvre's modes: run until told to stop."""
    cc.set_zone_continuous(Zone.Z1, downstream=True)
    time.sleep(0.05)
    assert fake.holding[New.Z1_MODE] == 2
    assert fake.coils[New.Z1_DIR] == 1

    cc.set_zone_idle(Zone.Z1)
    time.sleep(0.05)
    assert fake.holding[New.Z1_MODE] == MODE_IDLE


def test_shutter_command_confirm_split(cc):
    """set_shutter commands; wait_shutter confirms via the SENSED position,
    passing through MOVING on the way. Zone motion gates on the confirmation.
    """
    cc.set_shutter(ShutterState.OPEN)
    cc.wait_shutter(ShutterState.OPEN, timeout_s=2.0)
    assert cc.shutter_state() is ShutterState.OPEN

    cc.set_shutter(ShutterState.CLOSED)
    cc.wait_shutter(ShutterState.CLOSED, timeout_s=2.0)

    with pytest.raises(ValueError):
        cc.set_shutter(ShutterState.MOVING)


def test_fan_command_and_feedback(cc):
    cc.set_fan(Station.F1, True)
    time.sleep(0.05)
    assert cc.fan_on(Station.F1) is True
    cc.set_fan(Station.F1, False)
    time.sleep(0.05)
    assert cc.fan_on(Station.F1) is False


def test_read_inputs_reflects_poked_sensors(fake, cc):
    fake.set_input(New.O_EYE, 1)
    fake.set_input(New.Z2_EYE, 1)
    fake.set_input(New.IN_COUNT, 3, table="input")
    time.sleep(0.05)

    inputs = cc.read_inputs()
    assert inputs.o_eye is True
    assert inputs.z2_eye is True
    assert inputs.in_count == 3
    assert inputs.f1_eye is False

    fake.set_input(New.O_EYE, 0)
    fake.set_input(New.Z2_EYE, 0)
    fake.set_input(New.IN_COUNT, 0, table="input")


def test_heartbeat_keeps_watchdog_happy_and_silence_trips_it(cc):
    """The driver's heartbeat holds the watchdog off; stopping it trips the
    §7 fail-ON contract, and resuming recovers.
    """
    for _ in range(5):
        cc.heartbeat()
        time.sleep(0.1)
    assert cc.watchdog_tripped() is False

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and not cc.watchdog_tripped():
        time.sleep(0.05)
    assert cc.watchdog_tripped() is True, "silence must trip the watchdog"
    assert cc.fan_on(Station.F1) and cc.fan_on(Station.F2), "fans must fail ON"

    cc.heartbeat()
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and cc.watchdog_tripped():
        time.sleep(0.05)
    assert cc.watchdog_tripped() is False
