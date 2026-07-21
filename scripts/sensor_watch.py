"""Live ClearCore sensor change-watcher — read-only bench bring-up aid.

    <venv>/python scripts/sensor_watch.py [--cc HOST] [--hz N]

Polls the ClearCore's input snapshot and prints a timestamped line the instant
ANY signal changes — nothing else. Purpose: wire one sensor harness at a time,
trip each eye by hand, and confirm from the printed register NAME that the
physical sensor lands on the io_map.h assignment you expect (bench step 3).

Read-only: it never writes a register, sends a heartbeat, or commands motion.
Reconnects on its own if the board drops (power-cycle, cable pull) so you can
leave it running through the whole wiring session. Default host is the rewrite
unit, 192.168.1.19.
"""

from __future__ import annotations

import argparse
import time

from finishing_line.devices.clearcore import ClearCoreClient, ClearCoreError

# Snapshot field -> friendly label. Order = display order.
FIELDS = [
    ("in_eye", "IN_eye"),
    ("f1_eye", "F1_eye"),
    ("o_eye", "O_eye"),
    ("f2_eye", "F2_eye"),
    ("out_eye", "OUT_eye"),
    ("z1_eye", "Z1_eye (->Z1)"),
    ("z2_eye", "Z2_eye (->Z2)"),
    ("in_count", "IN_count"),
    ("shutter", "shutter"),
    ("f1_fan_on", "F1_fan"),
    ("f2_fan_on", "F2_fan"),
    ("watchdog_tripped", "watchdog_tripped"),
]


def _val(snapshot, field):
    v = getattr(snapshot, field)
    return v.name if hasattr(v, "name") else v  # ShutterState -> its name


def stamp() -> str:
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int((t % 1) * 1000):03d}"


def emit(msg: str) -> None:
    print(f"[{stamp()}] {msg}", flush=True)


def snapshot_dict(cc) -> dict:
    d = cc.read_inputs()
    return {field: _val(d, field) for field, _ in FIELDS}


def main() -> int:
    ap = argparse.ArgumentParser(prog="sensor_watch", description=__doc__)
    ap.add_argument("--cc", default="192.168.1.19", metavar="HOST")
    ap.add_argument("--hz", type=float, default=10.0, help="poll rate (default 10)")
    args = ap.parse_args()
    period = 1.0 / max(args.hz, 1.0)
    labels = dict(FIELDS)

    emit(f"connecting to ClearCore at {args.cc} ...")
    cc = None
    prev: dict | None = None
    while True:
        try:
            if cc is None:
                cc = ClearCoreClient(args.cc, timeout_s=1.0).connect()
                prev = snapshot_dict(cc)
                emit("CONNECTED — baseline:")
                for field, label in FIELDS:
                    print(f"           {label:16} = {prev[field]}", flush=True)
                emit("watching — trip a sensor; changes print below")
            cur = snapshot_dict(cc)
            for field, label in FIELDS:
                if cur[field] != prev[field]:
                    emit(f"{label:16} {prev[field]}  ->  {cur[field]}")
            prev = cur
            time.sleep(period)
        except ClearCoreError as exc:
            emit(f"LOST connection ({exc}); retrying...")
            if cc is not None:
                cc.close()
            cc = None
            prev = None
            time.sleep(1.0)
        except KeyboardInterrupt:
            if cc is not None:
                cc.close()
            emit("stopped")
            return 0


if __name__ == "__main__":
    raise SystemExit(main())
