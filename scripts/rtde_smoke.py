"""RTDE smoke test — ur_rtde against URSim, run from WSL2 or the Linux cell PC.

    ~/fl-venv/bin/python scripts/rtde_smoke.py [host]

Without a host argument it probes candidates in order: localhost first (works
in WSL2 mirrored-networking mode), then the WSL gateway (NAT mode, where
Windows-published container ports live on the host side of the NAT).

Success = connect RTDE receive + control, read the TCP pose, round-trip a
zero-length moveL. That exercises the exact interfaces URClient uses.
"""

from __future__ import annotations

import socket
import sys

RTDE_PORT = 30004


def candidate_hosts() -> list[str]:
    if len(sys.argv) > 1:
        return [sys.argv[1]]
    hosts = ["127.0.0.1"]
    try:
        with open("/etc/resolv.conf", encoding="utf-8") as fh:
            for line in fh:
                if line.startswith("nameserver"):
                    hosts.append(line.split()[1])
    except OSError:
        pass
    return hosts


def reachable(host: str) -> bool:
    try:
        socket.create_connection((host, RTDE_PORT), timeout=2.0).close()
        return True
    except OSError:
        return False


def main() -> int:
    host = next((h for h in candidate_hosts() if reachable(h)), None)
    if host is None:
        print(f"FAILED: no candidate host answers on RTDE port {RTDE_PORT}")
        print("Is URSim up on the Windows side?  docker compose -f docker-compose.ursim.yml up -d")
        return 1
    print(f"URSim reachable at {host}:{RTDE_PORT}")

    from rtde_receive import RTDEReceiveInterface

    rr = RTDEReceiveInterface(host)
    pose = rr.getActualTCPPose()
    print(f"  receive:  connected, TCP pose {[round(v, 4) for v in pose]}")
    mode = rr.getRobotMode()
    print(f"  robot mode: {mode} (7 = RUNNING)")

    if mode != 7:
        print("FAILED: robot not RUNNING — power on via scripts/ursim_smoke.py "
              "guidance or the pendant UI, then rerun")
        rr.disconnect()
        return 1

    # e-Series refuses external control scripts in Local mode; RTDEControl then
    # fails with an unhelpful "data synchronization" timeout. Diagnose it here.
    try:
        dash = socket.create_connection((host, 29999), timeout=5.0)
        dash.recv(4096)
        dash.sendall(b"is in remote control\n")
        in_remote = dash.recv(4096).decode().strip().lower() == "true"
        dash.close()
        if not in_remote:
            print("FAILED: Polyscope is in LOCAL control mode — RTDE control will be refused.")
            print("One-time fix in the pendant UI (http://localhost:6080/vnc.html):")
            print("  hamburger menu > Settings > System > Remote Control > Enable,")
            print("  then flip the new Local/Remote toggle (top right) to Remote.")
            print("Survives docker stop/start; must be redone after compose down/recreate.")
            rr.disconnect()
            return 1
    except OSError:
        print("  (dashboard not reachable from here; skipping remote-control precheck)")

    from rtde_control import RTDEControlInterface

    rc = RTDEControlInterface(host)
    print("  control:  connected")
    ok = rc.moveL(pose, 0.05, 0.5)  # zero-length move: full command round-trip
    print(f"  moveL round-trip: {'ok' if ok else 'FAILED'}")
    rc.disconnect()
    rr.disconnect()

    if not ok:
        return 1
    print("RTDE SMOKE TEST PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
