"""URSim smoke test — proves the simulator is up and reachable.

Deliberately stdlib-only: the Dashboard server (port 29999) is a plain
line-oriented TCP protocol, so this needs no ur_rtde — which matters, because
ur-rtde ships no Windows wheel and this must run before that problem is solved.

    python scripts/ursim_smoke.py [host]

Expected output ends with "SMOKE TEST PASSED". If the robot is still booting
(first run needs the safety confirmation in the pendant UI at
http://localhost:6080/vnc.html), robotmode reports BOOTING or POWER_OFF —
the script says so and still counts the connection itself as success.
"""

from __future__ import annotations

import socket
import sys

HOST = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
DASHBOARD_PORT = 29999
RTDE_PORT = 30004


def dashboard(sock: socket.socket, command: str) -> str:
    sock.sendall(command.encode() + b"\n")
    return sock.recv(4096).decode().strip()


def main() -> int:
    print(f"connecting to dashboard at {HOST}:{DASHBOARD_PORT} ...")
    try:
        sock = socket.create_connection((HOST, DASHBOARD_PORT), timeout=5.0)
    except OSError as exc:
        print(f"FAILED: cannot connect: {exc}")
        print("Is the container up?  docker compose -f docker-compose.ursim.yml up -d")
        return 1

    with sock:
        banner = sock.recv(4096).decode().strip()
        print(f"  banner:     {banner!r}")
        print(f"  robotmode:  {dashboard(sock, 'robotmode')!r}")
        print(f"  program:    {dashboard(sock, 'get loaded program')!r}")
        print(f"  safety:     {dashboard(sock, 'safetystatus')!r}")

    print(f"checking RTDE port {RTDE_PORT} accepts connections ...")
    try:
        socket.create_connection((HOST, RTDE_PORT), timeout=5.0).close()
        print("  RTDE port open")
    except OSError as exc:
        print(f"FAILED: RTDE port refused: {exc}")
        return 1

    print("SMOKE TEST PASSED")
    print("(If robotmode is not RUNNING: open http://localhost:6080/vnc.html,")
    print(" confirm the safety config, then power on + release brakes.)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
