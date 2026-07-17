"""State persistence — LineState survives an orchestrator restart.

The per-part flash timers are the truth the whole design protects (§6);
losing them to a crash would force choosing between re-flashing everything
(wasted hours) or guessing (shipping soft parts). So the controller snapshots
state to disk as it runs, and on startup a snapshot restores.

RESTORE IS A FAULT, deliberately. While the orchestrator was down, the world
kept moving: parts kept drying, an operator may have pulled one, belts may
have been jogged. A restored controller's belief is exactly as trustworthy as
a faulted one's — so a snapshot with parts on the line comes back in FAULTED
with "confirm occupancy and resume", reusing the §7 operator flow
(machine.resume) end to end: occupancy confirmation, sensor validation,
robot re-home, correct re-entry phase via fault_phase. An empty-line snapshot
restores directly; there is nothing at risk.

TIMER SEMANTICS ACROSS THE GAP: the snapshot under-counts — up to
save-interval seconds lost to the crash, plus any drying that happened while
down. Under-counting errs toward MORE flashing, which §7 declares safe
(over-flash allowed, under-flash never). This is why a ~1 s save throttle is
acceptable: the error direction is the safe one.

What is NOT persisted: pending intent ids and in-flight moves — they name
executor work that died with the process; resume re-establishes what matters
(the robot re-home) itself.
"""

from __future__ import annotations

import json
import os
from dataclasses import replace
from pathlib import Path
from time import monotonic

from ..core.model import (
    FanState,
    LineState,
    PartRole,
    PartState,
    Product,
    ShutterState,
    Station,
)
from ..core.schedule import Phase

SCHEMA_VERSION = 1


def serialize_state(state: LineState, declared: int) -> dict:
    return {
        "v": SCHEMA_VERSION,
        "declared": declared,
        "beat": state.beat,
        "phase": str(state.phase),
        "pair_index": state.pair_index,
        "fault": state.fault,
        "fault_phase": state.fault_phase,
        "if_fan": str(state.if_fan),
        "fd_fan": str(state.fd_fan),
        "shutter": str(state.shutter),
        "occupancy": {st.name: pid for st, pid in state.occupancy.items()},
        "inq_queue": list(state.inq_queue),
        "parts": {
            pid: {
                "product": str(p.product),
                "role": str(p.role),
                "pair_index": p.pair_index,
                "coats_applied": p.coats_applied,
                "flash_1_s": p.flash_1_s,
                "flash_2_s": p.flash_2_s,
                "is_wet": p.is_wet,
            }
            for pid, p in state.parts.items()
        },
    }


def deserialize_state(data: dict) -> tuple[LineState, int]:
    if data.get("v") != SCHEMA_VERSION:
        raise ValueError(f"unsupported snapshot schema {data.get('v')!r}")
    parts = {
        pid: PartState(
            part_id=pid,
            product=Product(p["product"]),
            role=PartRole(p["role"]),
            pair_index=int(p["pair_index"]),
            coats_applied=int(p["coats_applied"]),
            flash_1_s=float(p["flash_1_s"]),
            flash_2_s=float(p["flash_2_s"]),
            is_wet=bool(p["is_wet"]),
        )
        for pid, p in data["parts"].items()
    }
    state = LineState(
        parts=parts,
        occupancy={Station[k]: v for k, v in data["occupancy"].items()},
        inq_queue=tuple(data["inq_queue"]),
        beat=data["beat"],
        pair_index=int(data["pair_index"]),
        # Normalise to the enum at the boundary: plain strings pass equality
        # checks but silently fail any `is` comparison against Phase members.
        phase=Phase(data["phase"]),
        if_fan=FanState(data["if_fan"]),
        fd_fan=FanState(data["fd_fan"]),
        shutter=ShutterState(data["shutter"]),
        fault=data["fault"],
        fault_phase=data["fault_phase"],
    )
    return state, int(data["declared"])


def as_restored(state: LineState) -> LineState:
    """Convert a loaded snapshot into the state the controller starts with.

    Parts on the line -> FAULTED, routed through the §7 recovery flow. The
    saved phase becomes fault_phase (unless the snapshot was already faulted,
    which keeps its own), so resume re-enters at the right point relative to
    the beat's train move. Pending/in-flight are gone with the old process.
    """
    state = replace(state, pending=(), in_flight=())
    if not state.parts and not state.occupancy and not state.inq_queue:
        return replace(state, phase=str(Phase.ROBOT_WORK), fault=None, fault_phase=None)
    if state.fault is not None:
        return replace(state, phase=str(Phase.FAULTED))
    return replace(
        state,
        fault_phase=str(state.phase),
        phase=str(Phase.FAULTED),
        fault=(
            "orchestrator restarted with parts on the line — confirm occupancy "
            "and resume (flash timers were preserved and err toward extra drying)"
        ),
    )


class StateStore:
    """Atomic JSON snapshots with a save throttle.

    Structural changes (anything except the ever-ticking flash timers) save
    immediately; timer-only changes respect `min_interval_s` — their loss on
    crash errs safe. Writes are tmp + os.replace, atomic on one filesystem.
    """

    def __init__(self, path: str | os.PathLike, *, min_interval_s: float = 1.0) -> None:
        self._path = Path(path)
        self._min_interval_s = min_interval_s
        self._last_save = 0.0
        self._last_structural: object = None

    # ------------------------------------------------------------------ save

    def save(self, state: LineState, declared: int) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._path.with_suffix(".tmp")
        tmp.write_text(json.dumps(serialize_state(state, declared)), encoding="utf-8")
        os.replace(tmp, self._path)
        self._last_save = monotonic()
        self._last_structural = self._structural_key(state, declared)

    def maybe_save(self, state: LineState, declared: int) -> bool:
        key = self._structural_key(state, declared)
        if key != self._last_structural:
            self.save(state, declared)
            return True
        if monotonic() - self._last_save >= self._min_interval_s:
            self.save(state, declared)
            return True
        return False

    @staticmethod
    def _structural_key(state: LineState, declared: int) -> tuple:
        return (
            declared,
            state.beat,
            str(state.phase),
            state.fault,
            tuple(sorted((st.name, pid) for st, pid in state.occupancy.items())),
            state.inq_queue,
            tuple(sorted((pid, p.coats_applied, p.is_wet) for pid, p in state.parts.items())),
        )

    # ------------------------------------------------------------------ load

    def load(self) -> tuple[LineState, int] | None:
        """Restored (state, declared), or None for a fresh start.

        A corrupt snapshot must not take the orchestrator down: it is set
        aside as .corrupt (for the postmortem) and the line starts fresh —
        which, with parts physically present, the occupancy-mismatch guard
        will immediately surface to the operator anyway.
        """
        if not self._path.exists():
            return None
        try:
            data = json.loads(self._path.read_text(encoding="utf-8"))
            return deserialize_state(data)
        except (ValueError, KeyError, TypeError) as exc:
            corrupt = self._path.with_suffix(".corrupt")
            try:
                os.replace(self._path, corrupt)
            except OSError:
                pass
            print(f"WARNING: unreadable state snapshot set aside at {corrupt}: {exc}")
            return None
