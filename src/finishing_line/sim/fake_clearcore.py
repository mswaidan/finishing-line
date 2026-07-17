"""Fake ClearCore — a Modbus TCP server that behaves like the firmware will.

This is the executable reference for the register map in devices/registers.py:
the echo handshake, the zone-move lifecycle, shutter actuation with feedback
delay, and the fail-ON watchdog. The real firmware changes get written against
the behaviour encoded here, and the real ClearCoreClient driver is developed
against this server before it ever touches hardware (Stage B).

SCOPE — controller, not physics. The fake emulates what the *ClearCore* does:
registers, timing, state transitions. It does not know where parts are; presence
and handoff sensors are plain inputs that a test (or the Stage B harness) pokes
via set_input(). The physics lives in the caller, exactly as it does on the
real line.

WHY HAND-ROLLED. pymodbus 3.14 deprecated its entire server datastore mid-3.x
(setValues is already gone), so building the reference fake on it means chasing
a moving API. Modbus TCP framing is a few dozen lines; the value of this module
is the firmware behaviour, which is ours either way. Tests talk to it through
the pymodbus *client* — the same library the real driver uses, so every test
doubles as a check that the client and our framing agree.

Legacy registers (1-6, 100-108, 200-208) are carried and echoed so the old
vocabulary stays observable, but legacy M0 motion is NOT emulated — the
orchestrator drives the zone blocks (310/320), and rollback runs against the
real firmware, not this fake.
"""

from __future__ import annotations

import asyncio
import struct
import threading
import time
from dataclasses import dataclass, field

from ..devices.registers import Command, Echo, New, Status

TABLE_SIZE = 1024

# Modbus function codes
_READ_COILS = 1
_READ_DISCRETE = 2
_READ_HOLDING = 3
_READ_INPUT = 4
_WRITE_COIL = 5
_WRITE_REGISTER = 6
_WRITE_REGISTERS = 16

#: Zone motion modes, matching the legacy vocabulary (cell-config enums).
MODE_DISTANCE = 0
MODE_CONTINUOUS = 2
MODE_IDLE = 3

#: Zone states (cell-config server_state enum).
STATE_NOT_READY = 0
STATE_READY = 1
STATE_MOVING = 2


@dataclass
class _ZoneBlock:
    mode_reg: int
    dist_reg: int
    reqid_reg: int
    state_reg: int
    ack_reg: int
    last_reqid: int = 0
    move_done_at: float | None = None


@dataclass
class FakeClearCore:
    """Runs a Modbus TCP server plus a firmware-behaviour tick loop.

    Timing parameters are shrunk for tests; the defaults are NOT process values.
    """

    port: int = 15020
    tick_s: float = 0.01
    watchdog_timeout_s: float = 2.0
    shutter_actuation_s: float = 0.25
    #: steps/sec used to derive zone move durations from DISTANCE / VELOCITY.
    default_velocity: int = 1600

    coils: list[int] = field(default_factory=lambda: [0] * TABLE_SIZE)
    discrete: list[int] = field(default_factory=lambda: [0] * TABLE_SIZE)
    holding: list[int] = field(default_factory=lambda: [0] * TABLE_SIZE)
    input_regs: list[int] = field(default_factory=lambda: [0] * TABLE_SIZE)

    def __post_init__(self) -> None:
        self._zones = (
            _ZoneBlock(New.ZONE1_MOTION_MODE, New.ZONE1_DISTANCE,
                       New.ZONE1_REQUEST_ID, New.ZONE1_STATE, New.ZONE1_REQID_ACK),
            _ZoneBlock(New.ZONE2_MOTION_MODE, New.ZONE2_DISTANCE,
                       New.ZONE2_REQUEST_ID, New.ZONE2_STATE, New.ZONE2_REQID_ACK),
        )
        self.holding[New.ZONE1_MOTION_MODE] = MODE_IDLE
        self.holding[New.ZONE2_MOTION_MODE] = MODE_IDLE
        self.input_regs[New.ZONE1_STATE] = STATE_READY
        self.input_regs[New.ZONE2_STATE] = STATE_READY
        self.input_regs[Status.SERVER_STATE] = STATE_READY  # legacy, static
        self._last_heartbeat = 0
        self._heartbeat_seen_at = time.monotonic()
        #: The watchdog ARMS ON THE FIRST HEARTBEAT, not at power-on. Before an
        #: orchestrator has ever announced itself there is nothing to supervise,
        #: and tripping at power-on would force the fans on every time the line
        #: sits powered but idle. Once armed, it never disarms.
        self._watchdog_armed = False
        self._shutter_deadline: float | None = None
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop = threading.Event()

    # ------------------------------------------------------------------ hooks

    def set_input(self, register: int, value: int, *, table: str = "discrete") -> None:
        """Test/physics hook: poke a sensor the firmware merely reports.

        Single-slot list writes are atomic under the GIL, so this is safe to
        call from the test thread while the server runs.
        """
        getattr(self, {"discrete": "discrete", "input": "input_regs"}[table])[register] = value

    @property
    def watchdog_tripped(self) -> bool:
        return bool(self.input_regs[New.WATCHDOG_TRIPPED])

    # ------------------------------------------------------------- firmware

    def _tick(self, now: float) -> None:
        """One firmware loop: the behaviour the real firmware must reproduce."""
        # Echo handshake — the fake's updateLocals(). Commands land in the
        # 100-block; the 200-block echo is how the master knows they landed.
        for cmd, echo in zip(Command, Echo):
            if cmd in (Command.DIRECTION, Command.FEED_CONVEYOR, Command.BRUSH_ON):
                self.discrete[echo] = self.coils[cmd]
            else:
                self.input_regs[echo] = self.holding[cmd]

        # Watchdog: heartbeat must keep advancing. Stale -> zones halt, fans
        # forced ON (§7: fans fail ON — a dead orchestrator must not stop parts
        # drying). Recovers when the heartbeat advances again.
        hb = self.holding[New.HEARTBEAT]
        if hb != self._last_heartbeat:
            self._watchdog_armed = True
            self._last_heartbeat = hb
            self._heartbeat_seen_at = now
            self.input_regs[New.WATCHDOG_TRIPPED] = 0
        elif self._watchdog_armed and now - self._heartbeat_seen_at > self.watchdog_timeout_s:
            self.input_regs[New.WATCHDOG_TRIPPED] = 1

        tripped = bool(self.input_regs[New.WATCHDOG_TRIPPED])

        # Fans: contactor-fast, command mirrors to feedback — unless tripped,
        # in which case both are forced ON regardless of command.
        self.input_regs[New.IF_FAN_FEEDBACK] = 1 if tripped else self.holding[New.IF_FAN_CMD]
        self.input_regs[New.FD_FAN_FEEDBACK] = 1 if tripped else self.holding[New.FD_FAN_CMD]

        # Shutter: feedback lags command by the actuation time, reading MOVING
        # (2) in between. Watchdog does NOT move the shutter — it holds state
        # (§7: shutter holds on fault).
        cmd = self.holding[New.SHUTTER_CMD]
        fb = self.input_regs[New.SHUTTER_FEEDBACK]
        if cmd != fb and fb != 2:
            self.input_regs[New.SHUTTER_FEEDBACK] = 2
            self._shutter_deadline = now + self.shutter_actuation_s
        elif fb == 2 and self._shutter_deadline is not None and now >= self._shutter_deadline:
            self.input_regs[New.SHUTTER_FEEDBACK] = cmd
            self._shutter_deadline = None

        # Zones: same lifecycle as the legacy conveyor. A distance move is
        # recognised only when REQUEST_ID changes; duration = distance/velocity.
        velocity = self.holding[Command.VELOCITY] or self.default_velocity
        for zone in self._zones:
            if tripped:
                zone.move_done_at = None
                self.input_regs[zone.state_reg] = STATE_READY
                continue
            mode = self.holding[zone.mode_reg]
            if mode == MODE_CONTINUOUS:
                zone.move_done_at = None
                self.input_regs[zone.state_reg] = STATE_MOVING
            elif mode == MODE_IDLE:
                zone.move_done_at = None
                self.input_regs[zone.state_reg] = STATE_READY
            elif mode == MODE_DISTANCE:
                reqid = self.holding[zone.reqid_reg]
                if reqid != zone.last_reqid:
                    zone.last_reqid = reqid
                    distance = self.holding[zone.dist_reg]
                    zone.move_done_at = now + distance / velocity
                    self.input_regs[zone.state_reg] = STATE_MOVING
                    # Ack AFTER the move is set up: the ack promises "this id's
                    # move is running", never merely "I saw the number".
                    self.input_regs[zone.ack_reg] = reqid
                elif zone.move_done_at is not None and now >= zone.move_done_at:
                    zone.move_done_at = None
                    self.input_regs[zone.state_reg] = STATE_READY

    # ------------------------------------------------------------- protocol

    def _handle_pdu(self, pdu: bytes) -> bytes:
        fc = pdu[0]
        if fc in (_READ_COILS, _READ_DISCRETE):
            addr, count = struct.unpack(">HH", pdu[1:5])
            table = self.coils if fc == _READ_COILS else self.discrete
            bits = table[addr : addr + count]
            nbytes = (count + 7) // 8
            packed = bytearray(nbytes)
            for i, bit in enumerate(bits):
                if bit:
                    packed[i // 8] |= 1 << (i % 8)
            return bytes([fc, nbytes]) + bytes(packed)
        if fc in (_READ_HOLDING, _READ_INPUT):
            addr, count = struct.unpack(">HH", pdu[1:5])
            table = self.holding if fc == _READ_HOLDING else self.input_regs
            values = table[addr : addr + count]
            return bytes([fc, count * 2]) + b"".join(struct.pack(">H", v) for v in values)
        if fc == _WRITE_COIL:
            addr, value = struct.unpack(">HH", pdu[1:5])
            self.coils[addr] = 1 if value == 0xFF00 else 0
            return pdu[:5]
        if fc == _WRITE_REGISTER:
            addr, value = struct.unpack(">HH", pdu[1:5])
            self.holding[addr] = value
            return pdu[:5]
        if fc == _WRITE_REGISTERS:
            addr, count = struct.unpack(">HH", pdu[1:5])
            for i in range(count):
                (self.holding[addr + i],) = struct.unpack(">H", pdu[6 + 2 * i : 8 + 2 * i])
            return pdu[:5]
        return bytes([fc | 0x80, 0x01])  # illegal function

    async def _serve_client(self, reader: asyncio.StreamReader,
                            writer: asyncio.StreamWriter) -> None:
        try:
            while True:
                header = await reader.readexactly(7)
                tid, pid, length, unit = struct.unpack(">HHHB", header)
                pdu = await reader.readexactly(length - 1)
                response = self._handle_pdu(pdu)
                writer.write(struct.pack(">HHHB", tid, pid, len(response) + 1, unit) + response)
                await writer.drain()
        except (asyncio.IncompleteReadError, ConnectionResetError):
            pass
        finally:
            writer.close()

    async def _run(self, bound: threading.Event) -> None:
        try:
            server = await asyncio.start_server(self._serve_client, "127.0.0.1", self.port)
        except OSError as exc:
            self._start_error = exc
            bound.set()
            return
        bound.set()
        try:
            while not self._stop.is_set():
                self._tick(time.monotonic())
                await asyncio.sleep(self.tick_s)
        finally:
            server.close()
            await server.wait_closed()

    # ------------------------------------------------------------ lifecycle

    def start(self) -> "FakeClearCore":
        """Start the server; RAISES if the port cannot be bound.

        Failing fast matters more than it looks: a silent bind failure leaves
        a client happily connecting to whatever stale process still holds the
        port — a half-alive system that mostly works is far worse to debug
        than one that refuses to start.
        """
        self._stop.clear()
        self._start_error: OSError | None = None
        bound = threading.Event()

        def runner() -> None:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(self._run(bound))
            self._loop.close()

        self._thread = threading.Thread(target=runner, daemon=True, name="fake-clearcore")
        self._thread.start()
        if not bound.wait(timeout=5.0):
            raise RuntimeError("fake ClearCore never reported bind status")
        if self._start_error is not None:
            self._thread.join(timeout=1.0)
            raise RuntimeError(
                f"fake ClearCore cannot bind 127.0.0.1:{self.port} — another process "
                f"holds the port (a stale simulator?). Kill it or pick another port. "
                f"[{self._start_error}]"
            ) from self._start_error
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
