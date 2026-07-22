"""Intent executor — turns core intents into device calls.

The core emits intents and never blocks; this runs them on a single worker
thread, strictly FIFO. The ordering is not an implementation detail: the core
plans the P3 fan pause as [fan OFF, spray, fan ON] within one batch, so
executing out of order (or concurrently) would blow overspray across a wet
part. One worker, one queue, no reordering — the guarantee the core's
batch construction relies on.

FAILURE MODEL. An intent that raises poisons the executor: the fault reason is
latched, the queue is drained, and nothing further executes until `reset()`.
No retries — a failed device call means the physical world no longer matches
the controller's belief, and §7's answer to that is fault + operator recovery,
not optimism.

`HaltZones` never queues. A fault-halt arriving behind a 90-second sand job
would be a halt that doesn't halt — it executes immediately on the calling
thread (zone idles are fast Modbus writes; fans deliberately untouched, §7).

Completion means the device call RETURNED, and every device method blocks
until physically confirmed (shutter feedback, train arrival sensors, robot
motion done) — so a completed id reaching the core is a statement about the
world, not about a command buffer.
"""

from __future__ import annotations

import queue
import threading

from ..core.intents import (
    AdvanceTrain,
    CleanGun,
    HaltZones,
    Intent,
    MoveToSafePose,
    SandPart,
    SetFan,
    SetShutter,
    SprayPart,
)
from ..core.model import FanState, Zone
from ..devices.clearcore import ClearCoreClient
from .robot import RobotDevice
from .train import TrainMover


class Executor:
    def __init__(self, cc: ClearCoreClient, robot: RobotDevice, train: TrainMover) -> None:
        self._cc = cc
        self._robot = robot
        self._train = train
        self._queue: queue.Queue[Intent] = queue.Queue()
        self._lock = threading.Lock()
        self._done: set[str] = set()
        self._fault: str | None = None
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, daemon=True, name="intent-executor")
        self._worker.start()

    # -------------------------------------------------------------- interface

    def submit(self, intents: tuple[Intent, ...]) -> None:
        for intent in intents:
            if isinstance(intent, HaltZones):
                self.halt(intent.reason)
                with self._lock:
                    self._done.add(intent.intent_id)
                continue
            self._queue.put(intent)

    def completed(self) -> frozenset[str]:
        """Ids finished since the last call. Drains."""
        with self._lock:
            done, self._done = frozenset(self._done), set()
            return done

    def fault_reason(self) -> str | None:
        with self._lock:
            return self._fault

    def halt(self, reason: str) -> None:
        """Stop all zone motion now, ahead of anything queued. Fans keep
        running — parts mid-flash must keep drying through a fault (§7).
        """
        with self._lock:
            if self._fault is None:
                self._fault = reason
        self._drain()
        self._cc.set_zone_idle(Zone.Z1)
        self._cc.set_zone_idle(Zone.Z2)

    def reset(self) -> None:
        """Clear the fault latch after operator recovery (machine.resume)."""
        self._drain()
        with self._lock:
            self._fault = None
            self._done.clear()

    def close(self) -> None:
        self._stop.set()
        self._queue.put(None)  # type: ignore[arg-type]  # wake the worker
        self._worker.join(timeout=5.0)

    # ----------------------------------------------------------------- worker

    def _drain(self) -> None:
        try:
            while True:
                self._queue.get_nowait()
        except queue.Empty:
            pass

    def _run(self) -> None:
        while not self._stop.is_set():
            intent = self._queue.get()
            if intent is None or self._stop.is_set():
                return
            with self._lock:
                poisoned = self._fault is not None
            if poisoned:
                continue  # a faulted executor executes nothing further
            try:
                self._execute(intent)
            except Exception as exc:  # noqa: BLE001 - any device failure faults the line
                self.halt(f"{type(intent).__name__} failed: {exc}")
                continue
            with self._lock:
                self._done.add(intent.intent_id)

    def _execute(self, intent: Intent) -> None:
        match intent:
            case SetFan(station=station, state=state):
                self._cc.set_fan(station, state is FanState.ON)
            case SetShutter(target=target):
                self._cc.set_shutter(target)
                self._cc.wait_shutter(target)
            case AdvanceTrain(direction=direction, moves=moves):
                self._train.advance(direction, moves)
            case SandPart(part_id=part_id):
                self._robot.sand(part_id)
            case CleanGun(part_id=part_id):
                self._robot.clean_gun(part_id)
            case SprayPart(part_id=part_id, coat=coat):
                self._robot.spray(part_id, coat)
            case MoveToSafePose():
                self._robot.safe_pose()
            case _:
                raise ValueError(f"unknown intent {type(intent).__name__}")
