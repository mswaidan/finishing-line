"""Dashboard layer against a live URSim container.

Skips cleanly when URSim is down (`docker compose -f docker-compose.ursim.yml
up -d` to enable), so the default suite stays hardware-free. Everything here is
non-destructive: state queries plus power-on, which is idempotent and the state
the sim should be in anyway.
"""

from __future__ import annotations

import socket

import pytest

from finishing_line.devices.ur import Dashboard, URError

HOST = "127.0.0.1"


def _ursim_up() -> bool:
    try:
        socket.create_connection((HOST, 29999), timeout=1.0).close()
        return True
    except OSError:
        return False


pytestmark = pytest.mark.skipif(not _ursim_up(), reason="URSim not running on :29999")


@pytest.fixture()
def dash():
    with Dashboard(HOST) as d:
        yield d


def test_connect_reads_banner_and_talks(dash):
    mode = dash.robot_mode()
    assert mode, "robotmode returned an empty reply"
    assert " " not in mode, f"prefix not stripped: {mode!r}"


def test_safety_status_reports(dash):
    assert dash.safety_status() in {
        "NORMAL", "REDUCED", "PROTECTIVE_STOP", "RECOVERY",
        "SAFEGUARD_STOP", "SYSTEM_EMERGENCY_STOP", "ROBOT_EMERGENCY_STOP",
        "VIOLATION", "FAULT", "AUTOMATIC_MODE_SAFEGUARD_STOP",
        "SYSTEM_THREE_POSITION_ENABLING_STOP",
    }


def test_power_on_and_release_reaches_running(dash):
    """Idempotent on an already-running robot; from POWER_OFF this is the real
    bring-up URClient's supervisor will rely on after a container restart.
    """
    dash.power_on_and_release(timeout_s=90.0)
    assert dash.robot_mode() == "RUNNING"


def test_unknown_command_does_not_hang(dash):
    """The protocol answers one line per command even for garbage — the driver
    must never deadlock waiting for a second line that isn't coming.
    """
    reply = dash.command("definitely-not-a-command")
    assert reply  # any single-line answer is fine; hanging is the failure


def test_connect_refused_is_loud():
    with pytest.raises(URError, match="cannot reach"):
        Dashboard(HOST, port=1, timeout_s=0.3).connect()
