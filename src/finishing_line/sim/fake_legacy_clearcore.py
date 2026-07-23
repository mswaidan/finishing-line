"""Fake LEGACY ClearCore — executable twin of modbustest.ino for offline tests.

Mirrors the legacy firmware's observable behavior, pinned by
tests/test_legacy_clearcore.py:

- 100-block commands, 200-block echo (input regs / discretes).
- SERVER_STATE (input reg 1): 1 ready, 2 moving.
- Distance moves fire ONLY on a REQUEST_ID change while MOTION_MODE==0
  (ino:305); duration = DISTANCE steps / VELOCITY sps — so a never-pushed
  velocity (boot default 0) makes moves never complete, exactly like
  VelMax(0) on the real firmware.
- MOTION_MODE 3 (idle) cancels motion; 2 (continuous) runs until told.
- Feed (coil 107) / brush (coil 108) are level-driven state, no physics.
- The three sensors (WORK_AT_ZERO=4, OFFLOAD=5, ONLOAD=6) are plain inputs
  the test harness pokes via set_sensor() — the fake emulates the controller,
  the test emulates the physics (same split as fake_clearcore).

Motion is evaluated lazily from the wall clock — no tick thread.
"""

from __future__ import annotations

import asyncio
import struct
import threading
import time

_READ_COILS, _READ_DISCRETE, _READ_HOLDING, _READ_INPUT = 1, 2, 3, 4
_WRITE_COIL, _WRITE_REGISTER, _WRITE_REGISTERS = 5, 6, 16

_MODE, _DIRECTION, _VELOCITY, _ACCEL, _DISTANCE, _POSITION, _REQID = range(100, 107)
_FEED, _BRUSH = 107, 108
_STATE = 1
_SENSORS = {"run": 3, "work_at_zero": 4, "offload": 5, "onload": 6}


class FakeLegacyClearCore:
    def __init__(self, port: int = 15040) -> None:
        self.port = port
        self.holding = [0] * 512
        self.input_regs = [0] * 512
        self.coils = [0] * 512
        self.discrete = [0] * 512
        self._old_request_id = 0
        self._move_end: float | None = None
        self._continuous = False
        self.moves_started = 0  # test observability
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------- harness

    def set_sensor(self, name: str, value: bool) -> None:
        self.discrete[_SENSORS[name]] = 1 if value else 0

    def moving(self) -> bool:
        return self._state() == 2

    # ------------------------------------------------------------ behavior

    def _state(self) -> int:
        mode = self.holding[_MODE]
        if mode == 2:
            return 2
        if mode == 3:
            return 1
        if self._move_end is not None:
            if self._move_end == float("inf") or time.monotonic() < self._move_end:
                return 2
            self._move_end = None
        return 1

    def _on_write(self, addr: int, value: int) -> None:
        if addr == _REQID and self.holding[_MODE] == 0 and value != self._old_request_id:
            self._old_request_id = value
            velocity = self.holding[_VELOCITY]
            steps = self.holding[_DISTANCE]
            self._move_end = (
                float("inf") if velocity == 0 else time.monotonic() + steps / velocity
            )
            self.moves_started += 1
        elif addr == _MODE and value == 3:
            self._move_end = None  # MoveStopDecel

    def _refresh(self) -> None:
        """updateLocals(): copy the command block into the echo block."""
        for i in range(_MODE, _REQID + 1):
            self.input_regs[i + 100] = self.holding[i]
        self.discrete[_DIRECTION + 100] = self.coils[_DIRECTION]
        self.discrete[_FEED + 100] = self.coils[_FEED]
        self.discrete[_BRUSH + 100] = self.coils[_BRUSH]
        self.input_regs[_STATE] = self._state()

    # ------------------------------------------------------------- protocol

    def _handle_pdu(self, pdu: bytes) -> bytes:
        fc = pdu[0]
        self._refresh()
        if fc in (_READ_COILS, _READ_DISCRETE):
            addr, count = struct.unpack(">HH", pdu[1:5])
            table = self.coils if fc == _READ_COILS else self.discrete
            packed = bytearray((count + 7) // 8)
            for i, bit in enumerate(table[addr:addr + count]):
                if bit:
                    packed[i // 8] |= 1 << (i % 8)
            return bytes([fc, len(packed)]) + bytes(packed)
        if fc in (_READ_HOLDING, _READ_INPUT):
            addr, count = struct.unpack(">HH", pdu[1:5])
            table = self.holding if fc == _READ_HOLDING else self.input_regs
            values = table[addr:addr + count]
            return bytes([fc, count * 2]) + b"".join(struct.pack(">H", v) for v in values)
        if fc == _WRITE_COIL:
            addr, value = struct.unpack(">HH", pdu[1:5])
            self.coils[addr] = 1 if value == 0xFF00 else 0
            return pdu[:5]
        if fc == _WRITE_REGISTER:
            addr, value = struct.unpack(">HH", pdu[1:5])
            self.holding[addr] = value
            self._on_write(addr, value)
            return pdu[:5]
        return bytes([fc | 0x80, 0x01])

    async def _serve(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
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

    # ------------------------------------------------------------ lifecycle

    def start(self) -> "FakeLegacyClearCore":
        bound = threading.Event()

        async def main() -> None:
            server = await asyncio.start_server(self._serve, "127.0.0.1", self.port)
            bound.set()
            async with server:
                await server.serve_forever()

        def runner() -> None:
            self._loop = asyncio.new_event_loop()
            try:
                self._loop.run_until_complete(main())
            except asyncio.CancelledError:
                pass

        self._thread = threading.Thread(target=runner, daemon=True, name="fake-legacy-cc")
        self._thread.start()
        if not bound.wait(timeout=5.0):
            raise RuntimeError("fake legacy ClearCore never bound")
        return self

    def stop(self) -> None:
        if self._loop is not None:
            for task in asyncio.all_tasks(self._loop):
                self._loop.call_soon_threadsafe(task.cancel)
        if self._thread is not None:
            self._thread.join(timeout=5.0)
