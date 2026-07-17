"""ClearCore driver — Modbus TCP master via pymodbus.

Dumb-executor contract: no process logic, no retries that mask faults. Every
public method either verifiably did the thing or raises ClearCoreError; the
supervisor decides what a failure means.

HANDSHAKE MODEL — different from the legacy program, deliberately.

The old Polyscope program needed the 200-block echo for *every* parameter write
because its Modbus client abstraction gave no write confirmation. pymodbus TCP
writes are request/response — a returned write IS confirmation the register
changed. So plain parameter writes need no echo polling.

What protocol confirmation does NOT give is firmware-loop acknowledgement: the
register can hold the new value before the firmware has acted on it. That
matters exactly once — move acceptance — and is what ZONE*_REQID_ACK exists
for. The move sequence is therefore:

    write direction, mode, distance      (protocol-confirmed, no polling)
    write a fresh REQUEST_ID             (firmware acts only on id CHANGE)
    poll ack == id                       (move accepted and running)
    ...
    poll state == READY                  (move complete)

Distances are converted mm -> steps with the tuned legacy conversion
(ConveyorKinematics, floor and all), so commanded distances land where the old
line put them.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from ..config.loader import ConveyorKinematics, load_conveyor_kinematics
from ..core.model import ShutterState, Station, Zone
from .registers import New

try:
    from pymodbus.client import ModbusTcpClient
except ImportError:  # pragma: no cover - exercised only where pymodbus is absent
    ModbusTcpClient = None


class ClearCoreError(RuntimeError):
    """A command failed or a confirmation never arrived."""


@dataclass(frozen=True, slots=True)
class _ZoneRegs:
    mode: int
    distance: int
    reqid: int
    direction: int
    state: int
    ack: int


_ZONES: dict[Zone, _ZoneRegs] = {
    Zone.ZONE1: _ZoneRegs(
        New.ZONE1_MOTION_MODE, New.ZONE1_DISTANCE, New.ZONE1_REQUEST_ID,
        New.ZONE1_DIRECTION, New.ZONE1_STATE, New.ZONE1_REQID_ACK,
    ),
    Zone.ZONE2: _ZoneRegs(
        New.ZONE2_MOTION_MODE, New.ZONE2_DISTANCE, New.ZONE2_REQUEST_ID,
        New.ZONE2_DIRECTION, New.ZONE2_STATE, New.ZONE2_REQID_ACK,
    ),
}

_FAN_CMD = {Station.IF: New.IF_FAN_CMD, Station.FD: New.FD_FAN_CMD}
_FAN_FEEDBACK = {Station.IF: New.IF_FAN_FEEDBACK, Station.FD: New.FD_FAN_FEEDBACK}

_MODE_DISTANCE = 0
_MODE_CONTINUOUS = 2
_MODE_IDLE = 3
_STATE_READY = 1

_SHUTTER_FROM_FEEDBACK = {0: ShutterState.CLOSED, 1: ShutterState.OPEN, 2: ShutterState.MOVING}


@dataclass(frozen=True, slots=True)
class ClearCoreInputs:
    """One coherent read of everything the ClearCore reports.

    Only the conveyor side of the world — robot_clear / gun_on come from the UR
    driver, and the supervisor merges both into the core's SensorSnapshot.
    """

    if_present: bool
    s_present: bool
    fd_present: bool
    handoff_to_z1: bool
    handoff_to_z2: bool
    inq_count: int
    shutter: ShutterState
    if_fan_on: bool
    fd_fan_on: bool
    watchdog_tripped: bool


class ClearCoreClient:
    def __init__(
        self,
        host: str,
        port: int = 502,
        *,
        timeout_s: float = 2.0,
        poll_s: float = 0.02,
        kinematics: ConveyorKinematics | None = None,
    ) -> None:
        if ModbusTcpClient is None:
            raise ClearCoreError("pymodbus is not installed (pip install .[devices])")
        self._client = ModbusTcpClient(host, port=port, timeout=timeout_s)
        self._poll_s = poll_s
        self._kinematics = kinematics or load_conveyor_kinematics()
        self._request_id = 0
        self._heartbeat = 0

    # ------------------------------------------------------------- lifecycle

    def connect(self) -> "ClearCoreClient":
        if not self._client.connect():
            raise ClearCoreError(f"cannot reach ClearCore at {self._client.comm_params.host}")
        return self

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "ClearCoreClient":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------- low level

    def _write_register(self, address: int, value: int) -> None:
        if self._client.write_register(address, value).isError():
            raise ClearCoreError(f"write_register({address}, {value}) failed")

    def _write_coil(self, address: int, value: bool) -> None:
        if self._client.write_coil(address, value).isError():
            raise ClearCoreError(f"write_coil({address}, {value}) failed")

    def _read_register(self, address: int) -> int:
        r = self._client.read_input_registers(address, count=1)
        if r.isError():
            raise ClearCoreError(f"read_input_registers({address}) failed")
        return r.registers[0]

    def _read_discrete(self, address: int) -> bool:
        r = self._client.read_discrete_inputs(address, count=1)
        if r.isError():
            raise ClearCoreError(f"read_discrete_inputs({address}) failed")
        return bool(r.bits[0])

    def _poll_until(self, predicate, timeout_s: float, what: str) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(self._poll_s)
        raise ClearCoreError(f"timeout waiting for {what} ({timeout_s:.1f}s)")

    # ------------------------------------------------------------ zone moves

    def move_zone_mm(self, zone: Zone, distance_mm: float, *, accept_timeout_s: float = 2.0) -> int:
        """Start a distance move; sign of `distance_mm` selects direction.

        Returns once the firmware ACKS the move (running), not when it
        completes — pair with wait_zone_ready(). Returns the commanded steps.
        """
        regs = _ZONES[zone]
        steps = self._kinematics.mm_to_steps(distance_mm)
        self._write_coil(regs.direction, distance_mm >= 0)
        self._write_register(regs.mode, _MODE_DISTANCE)
        self._write_register(regs.distance, steps)

        self._request_id = self._request_id % 65535 + 1  # 1..65535, never repeats 0
        request_id = self._request_id
        self._write_register(regs.reqid, request_id)
        self._poll_until(
            lambda: self._read_register(regs.ack) == request_id,
            accept_timeout_s,
            f"{zone} to ack move {request_id}",
        )
        return steps

    def wait_zone_ready(self, zone: Zone, timeout_s: float = 60.0) -> None:
        regs = _ZONES[zone]
        self._poll_until(
            lambda: self._read_register(regs.state) == _STATE_READY,
            timeout_s,
            f"{zone} to finish its move",
        )

    def set_zone_continuous(self, zone: Zone, *, downstream: bool) -> None:
        """Run a zone until told otherwise — the handoff manoeuvre's mode."""
        regs = _ZONES[zone]
        self._write_coil(regs.direction, downstream)
        self._write_register(regs.mode, _MODE_CONTINUOUS)

    def set_zone_idle(self, zone: Zone) -> None:
        self._write_register(_ZONES[zone].mode, _MODE_IDLE)

    # -------------------------------------------------------- fans / shutter

    def set_fan(self, station: Station, on: bool) -> None:
        self._write_register(_FAN_CMD[station], 1 if on else 0)

    def fan_on(self, station: Station) -> bool:
        """Feedback, not command — what the fan is actually doing."""
        return bool(self._read_register(_FAN_FEEDBACK[station]))

    def set_shutter(self, target: ShutterState) -> None:
        """Command only. Confirmation comes from shutter_state()/wait_shutter —
        the split matters because zone motion gates on the SENSED position.
        """
        if target not in (ShutterState.OPEN, ShutterState.CLOSED):
            raise ValueError(f"cannot command shutter to {target}")
        self._write_register(New.SHUTTER_CMD, 1 if target is ShutterState.OPEN else 0)

    def shutter_state(self) -> ShutterState:
        raw = self._read_register(New.SHUTTER_FEEDBACK)
        return _SHUTTER_FROM_FEEDBACK.get(raw, ShutterState.UNKNOWN)

    def wait_shutter(self, target: ShutterState, timeout_s: float = 5.0) -> None:
        self._poll_until(
            lambda: self.shutter_state() is target,
            timeout_s,
            f"shutter to confirm {target}",
        )

    # ------------------------------------------------------ sensors / health

    def read_inputs(self) -> ClearCoreInputs:
        return ClearCoreInputs(
            if_present=self._read_discrete(New.IF_PRESENT),
            s_present=self._read_discrete(New.S_PRESENT),
            fd_present=self._read_discrete(New.FD_PRESENT),
            handoff_to_z1=self._read_discrete(New.HANDOFF_TO_Z1),
            handoff_to_z2=self._read_discrete(New.HANDOFF_TO_Z2),
            inq_count=self._read_register(New.INQ_COUNT),
            shutter=self.shutter_state(),
            if_fan_on=self.fan_on(Station.IF),
            fd_fan_on=self.fan_on(Station.FD),
            watchdog_tripped=bool(self._read_register(New.WATCHDOG_TRIPPED)),
        )

    def heartbeat(self) -> None:
        """Advance the watchdog counter. Call at watchdog.orchestrator_heartbeat_hz;
        going quiet for clearcore_timeout_s halts zones and forces fans ON.
        """
        self._heartbeat = self._heartbeat % 65535 + 1
        self._write_register(New.HEARTBEAT, self._heartbeat)

    def watchdog_tripped(self) -> bool:
        return bool(self._read_register(New.WATCHDOG_TRIPPED))
