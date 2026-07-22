"""LineController — the bridge between HTTP-world and the supervisor thread.

The supervisor loop runs in a background thread; the API serves requests from
FastAPI's event loop. Every mutation crosses that boundary here, under one
lock, applied between ticks — the supervisor itself stays single-threaded and
never learns the API exists.

Command semantics (the operator's contract):

- **run/pause** — pause holds the schedule at the next phase boundary; work
  already handed to the executor finishes, parts keep drying, the watchdog
  keeps getting fed. Resume picks up exactly where the machine held.
- **halt** — immediate: executor poisoned, zones idled now, machine faults on
  the next tick. The §7 posture, operator-initiated.
- **ack_fault** — the §7 recovery flow: operator confirms which part is where,
  machine.resume() validates it against live sensors and either resumes or
  stays faulted with the reason. The executor's fault latch clears only on a
  successful resume.
- **declare_batch** — identity enters the system here (sensors count parts,
  never name them). Lead/trail roles are assigned by global arrival order so
  pairing survives multiple small batches.
"""

from __future__ import annotations

import threading
import time
from dataclasses import replace

from ..config.loader import ProcessConfig
from ..core import machine
from ..core.model import LineState, PartRole, PartState, Product, Station
from ..core.schedule import Phase
from .executor import Executor
from .persistence import StateStore, as_restored
from .supervisor import Supervisor, build_sensors


class LineController:
    def __init__(
        self,
        supervisor: Supervisor,
        executor: Executor,
        *,
        tick_hz: float = 20.0,
        store: StateStore | None = None,
    ) -> None:
        self._sup = supervisor
        self._executor = executor
        self._tick_s = 1.0 / tick_hz
        self._lock = threading.Lock()
        self._enabled = False
        self._blocked_by: str | None = None
        self._declared = 0  # global arrival counter: drives lead/trail parity
        self._store = store
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        # Restore before the first tick. A snapshot with parts on the line
        # comes back FAULTED (see persistence.as_restored): a restarted
        # controller's belief is exactly as trustworthy as a faulted one's,
        # so it goes through the same operator confirm-and-resume flow.
        if store is not None:
            loaded = store.load()
            if loaded is not None:
                state, declared = loaded
                self._sup.state = as_restored(state)
                self._declared = declared
                self._blocked_by = self._sup.state.fault

    # ------------------------------------------------------------------ loop

    def start(self) -> "LineController":
        self._thread = threading.Thread(target=self._loop, daemon=True, name="line-controller")
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        if self._store is not None:
            with self._lock:
                self._store.save(self._sup.state, self._declared)

    def _loop(self) -> None:
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            dt, last = now - last, now
            with self._lock:
                # Pause takes effect at the BEAT BOUNDARY: keep ticking until
                # the machine reaches ROBOT_WORK (or faults). Freezing
                # mid-transition can strand a just-arrived part at a fan
                # station whose fan was never switched on — the transition must
                # finish so _set_fans covers every occupied station, then hold.
                at_boundary = self._sup.state.phase in (Phase.ROBOT_WORK, Phase.FAULTED)
                if self._enabled or not at_boundary:
                    result = self._sup.tick(dt)
                    self._blocked_by = result.blocked_by
                else:
                    self._sup.idle_tick(dt)
                state, declared = self._sup.state, self._declared
            # Persist OUTSIDE the lock: a slow disk (this repo lives on a NAS)
            # must not stall the tick. Structural changes save immediately;
            # timer-only churn is throttled — its loss on crash errs safe.
            if self._store is not None:
                self._store.maybe_save(state, declared)
            time.sleep(self._tick_s)

    # -------------------------------------------------------------- commands

    def set_running(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = enabled

    def halt(self, reason: str = "operator halt") -> None:
        # Executor halt is thread-safe and immediate; the machine picks the
        # fault up on its next tick via executor.fault_reason().
        self._executor.halt(reason)

    def ack_fault(
        self,
        confirmed_occupancy: dict[str, str] | None = None,
        beat: str | None = None,
    ) -> tuple[bool, str | None]:
        """§7 recovery. Returns (resumed, reason-if-not)."""
        with self._lock:
            if self._sup.state.phase != Phase.FAULTED:
                return False, "machine is not faulted"
            occupancy = (
                {Station[k.upper()]: v for k, v in confirmed_occupancy.items()}
                if confirmed_occupancy is not None
                else None
            )
            cc_inputs = self._sup.cc.read_inputs()
            sensors = build_sensors(
                cc_inputs, self._sup.robot.is_clear(), self._sup.robot.gun_on()
            )
            result = machine.resume(
                self._sup.state, sensors, confirmed_occupancy=occupancy, beat=beat
            )
            self._sup.state = result.state
            if result.state.phase == Phase.FAULTED:
                return False, result.blocked_by
            # Order matters: reset clears the executor's fault latch, THEN the
            # re-home intent resume emitted (MoveToSafePose) gets queued.
            self._executor.reset()
            self._executor.submit(result.intents)
            self._blocked_by = None
            return True, None

    def declare_batch(self, product: str, part_ids: list[str]) -> list[str]:
        """Append operator-declared parts to the logical IN queue."""
        prod = Product(product)
        with self._lock:
            state = self._sup.state
            new_parts = {}
            for pid in part_ids:
                if pid in state.parts or pid in state.in_queue:
                    raise ValueError(f"part id {pid!r} already exists")
                role = PartRole.LEAD if self._declared % 2 == 0 else PartRole.TRAIL
                new_parts[pid] = PartState(
                    part_id=pid, product=prod, role=role, pair_index=self._declared // 2
                )
                self._declared += 1
            self._sup.state = replace(
                state,
                parts={**state.parts, **new_parts},
                in_queue=state.in_queue + tuple(part_ids),
            )
        return part_ids

    # -------------------------------------------------------------- snapshot

    def snapshot(self) -> dict:
        """One JSON-ready view of everything the HMI shows. Read-locked so it
        never interleaves with a tick.
        """
        with self._lock:
            state: LineState = self._sup.state
            cfg: ProcessConfig = self._sup.cfg
            enabled = self._enabled
            blocked = self._blocked_by
            try:
                cc = self._sup.cc.read_inputs()
                cc_view = {
                    "shutter": cc.shutter.name,
                    "fans": {"F1": cc.f1_fan_on, "F2": cc.f2_fan_on},
                    "sensors": {
                        "F1": cc.f1_eye, "O": cc.o_eye, "F2": cc.f2_eye,
                        "IN": cc.in_eye, "OUT": cc.out_eye,
                        "in_count": cc.in_count,
                    },
                    "watchdog_tripped": cc.watchdog_tripped,
                }
            except Exception as exc:  # noqa: BLE001 - snapshot must not take the API down
                cc_view = {"error": f"clearcore unreachable: {exc}"}

        station_of = {pid: st.name for st, pid in state.occupancy.items()}
        return {
            "enabled": enabled,
            "beat": state.beat,
            "phase": str(state.phase),
            "spraying": state.spray_burst_active,
            "fault": state.fault,
            "blocked_by": state.fault or blocked,
            "occupancy": {st.name: pid for st, pid in state.occupancy.items()},
            "in_queue": list(state.in_queue),
            "parts": {
                pid: {
                    "station": station_of.get(pid, "IN" if pid in state.in_queue else None),
                    "product": str(p.product),
                    "role": str(p.role),
                    "pair": p.pair_index,
                    "coats": p.coats_applied,
                    "flash_1_s": round(p.flash_1_s, 1),
                    "flash_2_s": round(p.flash_2_s, 1),
                    "flash_target_s": cfg.flash_seconds,
                    "wet": p.is_wet,
                }
                for pid, p in state.parts.items()
            },
            "clearcore": cc_view,
            "config": {
                "flash_seconds": cfg.flash_seconds,
                "nominal_s_per_part": round(cfg.nominal_seconds_per_part(), 1),
                "unmeasured": list(cfg.unmeasured()),
            },
        }
