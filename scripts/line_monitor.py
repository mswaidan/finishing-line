"""Read-only live monitor for the finishing line's three systems.

    <venv>/python scripts/line_monitor.py [--cc HOST] [--ur HOST] [--hz N]

Purely OBSERVATIONAL. It connects to the ClearCore (Modbus), the UR5e Dashboard
(mode/safety), and the UR5e RTDE receive stream (pose/joints/IO), and redraws a
single live screen a few times a second. It never commands anything — no zone
moves, no fan/shutter writes, no watchdog heartbeat, and it opens RTDE *receive*
only, never the control interface. Safe to leave running while you wire up
sensors and bring the cell online.

The three panels are independent: each connects, polls, and reconnects on its
own, so one device being offline (or not wired yet) never blanks the others.

RTDE needs ur_rtde (Linux cell PC / WSL2); on Windows that panel shows
UNAVAILABLE while the ClearCore and Dashboard panels keep working. Defaults:
ClearCore 192.168.1.19 (the rewrite unit; legacy production unit is .18),
UR5e 192.168.1.32.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

# The package is installed editable (pyproject), so this import works from any
# cwd. Reuse the real drivers so the monitor sees exactly what the orchestrator
# would — no second, drifting definition of "what the ClearCore reports".
from finishing_line.devices.clearcore import ClearCoreClient, ClearCoreError
from finishing_line.devices.ur import Dashboard, URError

RETRY_EVERY_S = 3.0  # throttle reconnect attempts so a down device can't stall the frame

# ------------------------------------------------------------------ ANSI paint

_NO_COLOR = bool(os.environ.get("NO_COLOR"))


def _c(text: str, code: str) -> str:
    return text if _NO_COLOR else f"\x1b[{code}m{text}\x1b[0m"


def green(t: str) -> str:  return _c(t, "32")
def red(t: str) -> str:    return _c(t, "31")
def yellow(t: str) -> str: return _c(t, "33")
def dim(t: str) -> str:    return _c(t, "2")
def bold(t: str) -> str:   return _c(t, "1")


def dot(active: bool) -> str:
    """Filled bright dot for an asserted signal, dim dot for a quiet one."""
    return green("●") if active else dim("·")


def enable_vt_on_windows() -> None:
    """Turn on ANSI escape processing for legacy Windows consoles. Modern
    Windows Terminal / VS Code terminals already do this; the call is harmless
    there and on non-Windows.
    """
    if os.name != "nt":
        return
    try:
        import ctypes

        k = ctypes.windll.kernel32
        k.SetConsoleMode(k.GetStdHandle(-11), 7)  # ENABLE_VIRTUAL_TERMINAL_PROCESSING | ...
    except Exception:
        pass  # worst case: raw escape codes; the data is still there


# --------------------------------------------------------------- pollers

class _Panel:
    """Common reconnect throttle. Subclasses implement _connect()/_read()."""

    def __init__(self, name: str) -> None:
        self.name = name
        self._conn = None
        self._next_retry = 0.0
        self.error: str | None = None
        self.data = None

    def _connect(self):  # -> connection object; raises on failure
        raise NotImplementedError

    def _read(self, conn):  # -> data; raises on failure
        raise NotImplementedError

    def poll(self) -> None:
        now = time.monotonic()
        if self._conn is None:
            if now < self._next_retry:
                return
            try:
                self._conn = self._connect()
                self.error = None
            except Exception as exc:  # connection failures are expected during bring-up
                self._conn = None
                self.error = str(exc)
                self._next_retry = now + RETRY_EVERY_S
                return
        try:
            self.data = self._read(self._conn)
            self.error = None
        except Exception as exc:
            self._drop(f"{exc}")

    def _drop(self, why: str) -> None:
        try:
            self.close()
        finally:
            self._conn = None
            self.data = None
            self.error = why
            self._next_retry = time.monotonic() + RETRY_EVERY_S

    def close(self) -> None:
        pass

    @property
    def online(self) -> bool:
        return self._conn is not None and self.data is not None


class ClearCorePanel(_Panel):
    def __init__(self, host: str) -> None:
        super().__init__("CLEARCORE")
        self.host = host

    def _connect(self):
        # Short timeout: a down device must not block the frame for seconds.
        return ClearCoreClient(self.host, timeout_s=1.0).connect()

    def _read(self, conn):
        return conn.read_inputs()

    def close(self) -> None:
        if self._conn is not None:
            self._conn.close()


class DashboardPanel(_Panel):
    def __init__(self, host: str) -> None:
        super().__init__("UR-DASH")
        self.host = host

    def _connect(self):
        return Dashboard(self.host, timeout_s=2.0).connect()

    def _read(self, conn):
        return {"mode": conn.robot_mode(), "safety": conn.safety_status()}

    def close(self) -> None:
        if self._conn is not None:
            self._conn.close()


class RtdePanel(_Panel):
    """RTDE *receive* only — passive telemetry, works in Local control mode and
    never takes control of the arm. Falls back gracefully when ur_rtde is absent
    (Windows) or a getter is missing on this ur_rtde version.
    """

    def __init__(self, host: str) -> None:
        super().__init__("UR-RTDE")
        self.host = host

    def _connect(self):
        try:
            from rtde_receive import RTDEReceiveInterface
        except ImportError as exc:
            raise URError("ur_rtde not installed (Linux/WSL2 only)") from exc
        return RTDEReceiveInterface(self.host)

    def _read(self, conn):
        def safe(fn, default=None):
            try:
                return fn()
            except Exception:
                return default

        # Digital-output readback method name varies across ur_rtde versions;
        # try the common ones and degrade to None rather than crash the panel.
        def out_bits():
            for attr in ("getActualDigitalOutputBits", "getDigitalOutState"):
                fn = getattr(conn, attr, None)
                if fn is None:
                    continue
                if attr == "getDigitalOutState":
                    return [bool(fn(i)) for i in range(8)]
                bits = fn()
                return [bool(bits & (1 << i)) for i in range(8)]
            return None

        return {
            "pose": safe(conn.getActualTCPPose),
            "q": safe(conn.getActualQ),
            "mode": safe(conn.getRobotMode),
            "safety": safe(conn.getSafetyMode),
            "outs": out_bits(),
        }

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.disconnect()
            except Exception:
                pass


# --------------------------------------------------------------- rendering

_ROBOT_MODE = {  # ur_rtde getRobotMode() ints
    -1: "NO_CONTROLLER", 0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING",
    3: "POWER_OFF", 4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE", 7: "RUNNING",
}
_SAFETY_MODE = {
    1: "NORMAL", 2: "REDUCED", 3: "PROT_STOP", 4: "RECOVERY", 5: "SAFEGUARD_STOP",
    6: "SYS_EMG_STOP", 7: "ROBOT_EMG_STOP", 8: "VIOLATION", 9: "FAULT",
}


def _status_tag(panel: _Panel) -> str:
    if panel.online:
        return green("[ONLINE]")
    if panel.error and "not installed" in panel.error:
        return yellow("[UNAVAILABLE]")
    return red("[OFFLINE]")


def render_clearcore(p: ClearCorePanel) -> list[str]:
    head = f"{bold('CLEARCORE')}  {p.host}:502   {_status_tag(p)}"
    if not p.online:
        return [head, "  " + dim(p.error or "connecting...")]
    d = p.data
    wd = red("TRIPPED") if d.watchdog_tripped else green("ok")
    sh = {"OPEN": green, "CLOSED": dim, "MOVING": yellow}.get(d.shutter.name, str)
    return [
        head,
        f"  eyes      IN {dot(d.in_eye)}   F1 {dot(d.f1_eye)}   "
        f"O {dot(d.o_eye)}   F2 {dot(d.f2_eye)}   OUT {dot(d.out_eye)}",
        f"  handoff   →Z1 {dot(d.z1_eye)}   →Z2 {dot(d.z2_eye)}"
        f"       IN count {d.in_count}",
        f"  shutter   {sh(d.shutter.name)}        watchdog {wd}",
        f"  fans      F1 {(green('ON') if d.f1_fan_on else dim('off'))}   "
        f"F2 {(green('ON') if d.f2_fan_on else dim('off'))}",
    ]


def render_dashboard(p: DashboardPanel) -> list[str]:
    head = f"{bold('UR5e / DASHBOARD')}  {p.host}:29999   {_status_tag(p)}"
    if not p.online:
        return [head, "  " + dim(p.error or "connecting...")]
    mode, safety = p.data["mode"], p.data["safety"]
    mc = green if mode == "RUNNING" else yellow
    sc = green if safety == "NORMAL" else red
    return [head, f"  robot mode {mc(mode)}     safety {sc(safety)}"]


def render_rtde(p: RtdePanel) -> list[str]:
    head = f"{bold('UR5e / RTDE')}  {p.host}:30004   {_status_tag(p)}   {dim('receive only')}"
    if not p.online:
        return [head, "  " + dim(p.error or "connecting...")]
    d = p.data
    lines = [head]
    if d["pose"]:
        x, y, z, rx, ry, rz = d["pose"]
        lines.append(f"  TCP   x{x:+.3f} y{y:+.3f} z{z:+.3f}  rx{rx:+.3f} ry{ry:+.3f} rz{rz:+.3f}  (m/rad)")
    if d["q"]:
        deg = "  ".join(f"{q * 57.2958:+6.1f}" for q in d["q"])
        lines.append(f"  q     {deg}  (deg)")
    mode = _ROBOT_MODE.get(d["mode"], str(d["mode"]))
    safety = _SAFETY_MODE.get(d["safety"], str(d["safety"]))
    lines.append(f"  mode  {mode}     safety {safety}")
    if d["outs"] is not None:
        marks = "  ".join(
            f"{i}:{dot(d['outs'][i])}" for i in range(min(6, len(d["outs"])))
        )
        lines.append(f"  DO    {marks}   {dim('(3=sander  5=sprayer/gun)')}")
    else:
        lines.append(f"  DO    {dim('unavailable on this ur_rtde version')}")
    return lines


def frame(panels, hz: float) -> str:
    clock = time.strftime("%H:%M:%S")
    out = [
        f"{bold('finishing-line monitor')}  {dim('read-only')}  {clock}   "
        f"{dim(f'{hz:g} Hz · Ctrl-C to quit')}",
        "",
    ]
    cc, dash, rtde = panels
    out += render_clearcore(cc) + [""]
    out += render_dashboard(dash) + [""]
    out += render_rtde(rtde)
    # Home cursor, clear each line as we overwrite, then clear anything below.
    body = "\x1b[H" + "\n".join(line + "\x1b[K" for line in out) + "\x1b[J"
    return body


def main() -> int:
    ap = argparse.ArgumentParser(prog="line_monitor", description=__doc__)
    ap.add_argument("--cc", default="192.168.1.19", metavar="HOST", help="ClearCore Modbus host")
    ap.add_argument("--ur", default="192.168.1.32", metavar="HOST", help="UR5e host")
    ap.add_argument("--hz", type=float, default=4.0, help="screen refresh rate")
    args = ap.parse_args()

    enable_vt_on_windows()
    panels = (ClearCorePanel(args.cc), DashboardPanel(args.ur), RtdePanel(args.ur))
    period = 1.0 / max(args.hz, 0.5)

    sys.stdout.write("\x1b[2J\x1b[?25l")  # clear screen, hide cursor
    try:
        while True:
            for p in panels:
                p.poll()
            sys.stdout.write(frame(panels, args.hz))
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        return 0
    finally:
        for p in panels:
            p.close()
        sys.stdout.write("\x1b[?25h\n")  # show cursor
        sys.stdout.flush()


if __name__ == "__main__":
    raise SystemExit(main())
