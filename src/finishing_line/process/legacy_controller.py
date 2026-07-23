"""LegacyController — the API/HMI facade for the legacy-mod sequencer.

Presents the same five-method surface the FastAPI layer consumes (snapshot,
set_running, halt, declare_batch, ack_fault), so create_app() and the HMI page
work unchanged in legacy mode.

Threading model: this thread is the ONLY executor of sequencer steps, and the
sequencer thread is the only owner of the Modbus client (pymodbus is not
thread-safe, and the composites drive the belt mid-work). API-thread commands
mutate only GIL-atomic structures (list append, dict assignment); snapshot()
builds a fresh dict from plain reads. No lock is held across blocking steps —
that is what keeps the HMI live during a 90 s sand.

Halt is the agreed BOUNDARY halt: the flag is honored between steps (~instant
during flash waits, up to one composite/move otherwise). The physical e-stop
is the emergency stop.

Recovery is minimal by decision: state persists continuously; a restart with
parts on the belt comes up FAULTED showing exactly what was there, and the
operator either confirms the occupancy (resume) or clears the line (<= 3
parts by hand) via the HMI ack panel.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

from ..config.loader import ProcessConfig
from ..core.model import PartRole, PartState, Product, Station
from .legacy_sequencer import PHASE_FAULTED, LegacySequencer


class LegacyController:
    def __init__(
        self,
        sequencer: LegacySequencer,
        cfg: ProcessConfig,
        *,
        state_file: str | None = None,
        save_every_s: float = 5.0,
    ) -> None:
        self._seq = sequencer
        self._cfg = cfg
        self._path = Path(state_file) if state_file else None
        self._save_every_s = save_every_s
        self._enabled = False
        self._halt_reason: str | None = None
        self._blocked: str | None = None
        self._dirty = False
        self._last_save = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        sequencer._on_change = self._mark_dirty
        self._restore()

    # ------------------------------------------------------------ lifecycle

    def start(self) -> "LegacyController":
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="legacy-controller")
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=10.0)
        self._save()

    def _loop(self) -> None:
        seq = self._seq
        while not self._stop.is_set():
            if self._halt_reason is not None:
                reason, self._halt_reason = self._halt_reason, None
                self._enabled = False
                seq.halt_now(reason)
            if self._enabled and seq.phase != PHASE_FAULTED:
                self._blocked = seq.step()
            else:
                seq.bank()  # paused/faulted parts keep drying (§7 heritage)
                time.sleep(0.1)
            now = time.monotonic()
            if self._dirty or now - self._last_save > self._save_every_s:
                self._save()
                self._dirty = False
                self._last_save = now

    def _mark_dirty(self) -> None:
        self._dirty = True

    # ---------------------------------------------------------- persistence

    def _save(self) -> None:
        if self._path is None:
            return
        seq = self._seq
        data = {
            "declared": seq.declared,
            "beat": seq.beat,
            "queue": list(seq.queue),
            "completed": list(seq.completed),
            "occupancy": {st.name: pid for st, pid in seq.occ.items()},
            "parts": {
                pid: {
                    "product": str(p.product), "role": str(p.role),
                    "pair_index": p.pair_index, "coats": p.coats_applied,
                    "flash_1_s": p.flash_1_s, "flash_2_s": p.flash_2_s,
                    "wet": p.is_wet,
                } for pid, p in seq.parts.items()
            },
        }
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._path.write_text(json.dumps(data, indent=1), encoding="utf-8")

    def _restore(self) -> None:
        if self._path is None or not self._path.exists():
            return
        data = json.loads(self._path.read_text(encoding="utf-8"))
        seq = self._seq
        seq.declared = data["declared"]
        seq.beat = data["beat"]
        seq.queue = list(data["queue"])
        seq.completed = list(data["completed"])
        seq.occ = {Station[name]: pid for name, pid in data["occupancy"].items()}
        seq.parts = {
            pid: PartState(
                part_id=pid, product=Product(p["product"]), role=PartRole(p["role"]),
                pair_index=p["pair_index"], coats_applied=p["coats"],
                flash_1_s=p["flash_1_s"], flash_2_s=p["flash_2_s"], is_wet=p["wet"],
            ) for pid, p in data["parts"].items()
        }
        if seq.occ:
            seq.fault = ("restarted with parts on the line — confirm occupancy "
                         "in the fault panel, or clear the belt and acknowledge")
            seq.phase = PHASE_FAULTED

    # -------------------------------------------------------------- surface

    def set_running(self, enabled: bool) -> None:
        self._enabled = enabled

    def halt(self, reason: str = "operator halt") -> None:
        self._halt_reason = reason  # honored at the next step boundary

    def declare_batch(self, product: str, part_ids: list[str]) -> list[str]:
        return self._seq.declare_batch(product, part_ids)

    def ack_fault(
        self,
        confirmed_occupancy: dict[str, str] | None = None,
        beat: str | None = None,
    ) -> tuple[bool, str | None]:
        seq = self._seq
        if seq.phase != PHASE_FAULTED:
            return False, "machine is not faulted"
        occupancy = {
            Station[k.upper()]: v
            for k, v in (confirmed_occupancy or {}).items() if v
        }
        if not occupancy:
            seq.clear_line()
            return True, None
        reason = seq.confirm_occupancy(occupancy, beat)
        if reason is not None:
            return False, reason
        return True, None

    def snapshot(self) -> dict:
        seq, cfg = self._seq, self._cfg
        station_of = {pid: st.name for st, pid in seq.occ.items()}
        sensors = seq.sensors
        return {
            "mode": "legacy",
            "enabled": self._enabled,
            "beat": seq.beat,
            "phase": seq.phase,
            "spraying": seq.spraying,
            "fault": seq.fault,
            "blocked_by": seq.fault or self._blocked,
            "stage_note": seq.stage_note,
            "occupancy": {st.name: pid for st, pid in seq.occ.items()},
            "in_queue": list(seq.queue),
            "completed": list(seq.completed),
            "parts": {
                pid: {
                    "station": station_of.get(pid, "IN" if pid in seq.queue else None),
                    "product": str(p.product), "role": str(p.role),
                    "coats": p.coats_applied,
                    "flash_1_s": round(p.flash_1_s, 1),
                    "flash_2_s": round(p.flash_2_s, 1),
                    "wet": p.is_wet,
                } for pid, p in seq.parts.items()
            },
            "clearcore": {
                "shutter": "NONE",  # no shutter on the legacy-mod route
                "fans": {"F1": seq._fan_on(Station.F1), "F2": seq._fan_on(Station.F2)},
                "sensors": {
                    "F1": bool(sensors.onload) if sensors else None,
                    "O": bool(sensors.work_at_zero) if sensors else None,
                    "F2": None,  # no eye at F2 on this route
                    "IN": bool(seq.queue),
                    "OUT": bool(sensors.offload) if sensors else None,
                    "in_count": len(seq.queue),
                },
                "watchdog_tripped": False,
            },
            "config": {
                "flash_seconds": cfg.flash_seconds,
                "nominal_s_per_part": cfg.nominal_period_s() / 2,
                "unmeasured": sorted(
                    k for k, v in cfg.provenance.items() if v != "measured"),
            },
        }
