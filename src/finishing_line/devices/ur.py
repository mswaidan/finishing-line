"""UR5e driver — Dashboard (lifecycle) + RTDE via ur_rtde (motion).

Two layers because they have different jobs and different dependencies:

- **Dashboard** (port 29999): load/play/stop, power, brake release,
  protective-stop recovery. Plain line-oriented TCP — stdlib only, runs
  anywhere, including this Windows dev machine against URSim.
- **URClient** (RTDE, port 30004): motion and I/O via `ur_rtde`. Linux-wheel
  only, so it runs under WSL2 in dev and on the Linux cell PC in production.
  Imported lazily: constructing a URClient without ur_rtde installed raises,
  importing this module does not.

All tuned constants come from cell-config.yaml (RobotSetup / SandConfig).
No program flow lives here — primitives only, sequenced by process/.
"""

from __future__ import annotations

import socket
import time

from ..config.loader import RobotSetup, load_robot_setup


class URError(RuntimeError):
    pass


class Dashboard:
    """UR Dashboard server client. Line-oriented TCP, one reply per command.

    The banner arrives on connect; every command returns exactly one line.
    Replies are English sentences, not codes — match loosely and surface the
    raw text in errors so the operator sees what the robot actually said.
    """

    def __init__(self, host: str, port: int = 29999, timeout_s: float = 5.0) -> None:
        self._host = host
        self._port = port
        self._timeout_s = timeout_s
        self._sock: socket.socket | None = None

    def connect(self) -> "Dashboard":
        try:
            self._sock = socket.create_connection((self._host, self._port), self._timeout_s)
        except OSError as exc:
            raise URError(f"cannot reach dashboard at {self._host}:{self._port}: {exc}") from exc
        banner = self._readline()
        if "Dashboard Server" not in banner:
            raise URError(f"unexpected dashboard banner: {banner!r}")
        return self

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def __enter__(self) -> "Dashboard":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    def _readline(self) -> str:
        assert self._sock is not None
        chunks = b""
        while not chunks.endswith(b"\n"):
            data = self._sock.recv(4096)
            if not data:
                raise URError("dashboard connection closed")
            chunks += data
        return chunks.decode().strip()

    def command(self, cmd: str) -> str:
        if self._sock is None:
            raise URError("dashboard not connected")
        self._sock.sendall(cmd.encode() + b"\n")
        return self._readline()

    # ------------------------------------------------------------ lifecycle

    def robot_mode(self) -> str:
        """e.g. 'RUNNING', 'POWER_OFF', 'IDLE', 'BOOTING'."""
        return self.command("robotmode").removeprefix("Robotmode: ")

    def safety_status(self) -> str:
        """e.g. 'NORMAL', 'PROTECTIVE_STOP', 'ROBOT_EMERGENCY_STOP'."""
        return self.command("safetystatus").removeprefix("Safetystatus: ")

    def power_on_and_release(self, timeout_s: float = 60.0) -> None:
        """Bring the arm to RUNNING from any powered-off state."""
        self.command("power on")
        self.command("brake release")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.robot_mode() == "RUNNING":
                return
            time.sleep(1.0)
        raise URError(f"robot never reached RUNNING (mode: {self.robot_mode()})")

    def recover_protective_stop(self, timeout_s: float = 30.0) -> None:
        """§7 recovery path. The 5-second rule is the robot's, not ours: UR
        refuses the unlock until 5s after the stop, replying 'Cannot unlock
        protective stop until 5s after occurrence' — so retry, don't fail fast.
        """
        self.command("close safety popup")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            reply = self.command("unlock protective stop")
            if "Protective stop releasing" in reply:
                return
            if self.safety_status() == "NORMAL":
                return
            time.sleep(1.0)
        raise URError(f"protective stop never released (status: {self.safety_status()})")

    def load_program(self, name: str) -> None:
        """Rollback path — loads a .urp by pendant-visible path. The
        orchestrator must have released its Modbus session first
        (registers.MASTER_HANDOFF).
        """
        reply = self.command(f"load {name}")
        if not reply.startswith("Loading program"):
            raise URError(f"load failed: {reply!r}")

    def play(self) -> None:
        reply = self.command("play")
        if reply != "Starting program":
            raise URError(f"play failed: {reply!r}")

    def stop(self) -> None:
        reply = self.command("stop")
        if reply != "Stopped":
            raise URError(f"stop failed: {reply!r}")


class URClient:
    """RTDE motion + I/O. Requires ur_rtde (Linux/WSL2).

    Constants are injected from cell-config via RobotSetup/SandConfig — this
    class knows HOW to move, never WHERE or WHY.
    """

    SANDER_OUTPUT = 3   # standard digital out — legacy "Start tool"
    SPRAYER_OUTPUT = 5  # standard digital out — legacy "SprayerOn"

    def __init__(self, host: str, setup: RobotSetup | None = None) -> None:
        try:
            from rtde_control import RTDEControlInterface
            from rtde_io import RTDEIOInterface
            from rtde_receive import RTDEReceiveInterface
        except ImportError as exc:  # pragma: no cover
            raise URError(
                "ur_rtde is not installed. No Windows wheel exists — use the "
                "WSL2 environment (docs/simulation.md) or the Linux cell PC."
            ) from exc

        self._setup = setup or load_robot_setup()
        self._control = RTDEControlInterface(host)
        self._receive = RTDEReceiveInterface(host)
        self._io = RTDEIOInterface(host)
        # TCP and payload are safety-relevant: force readings and singularity
        # limits are computed against them. Set before any motion.
        self._control.setTcp(list(self._setup.tcp))
        self._control.setPayload(self._setup.payload_mass_kg, list(self._setup.payload_cog_m))

    def close(self) -> None:
        self._control.disconnect()
        self._receive.disconnect()
        self._io.disconnect()

    # --------------------------------------------------------------- motion

    def move_to_named(self, waypoint: str) -> None:
        """movej to a cell-config waypoint, preserving the legacy solve:
        inverse kin on the pose, seeded by the recorded joints so the arm
        stays out of alternate elbow solutions.
        """
        wp = self._setup.waypoints[waypoint]
        q = self._control.getInverseKinematics(list(wp.p), qnear=list(wp.q))
        if not self._control.moveJ(q, self._setup.movej_v, self._setup.movej_a):
            raise URError(f"moveJ to {waypoint} failed")

    def move_base_x_mm(self, distance_mm: float, a: float, v: float) -> None:
        """The robot's axis of the sanding raster: base-frame X, tool attitude
        held. pose_add semantics, exactly like the legacy moveHeight.
        """
        pose = self._receive.getActualTCPPose()
        pose[0] += distance_mm / 1000.0
        if not self._control.moveL(pose, v, a):
            raise URError("moveL (base X) failed")

    def move_base_z_mm(self, distance_mm: float, a: float, v: float) -> None:
        """Base-frame Z move — the vertical spray's standoff approach
        (calculate_point_to_move_towards base Z+, script:2869).
        """
        pose = self._receive.getActualTCPPose()
        pose[2] += distance_mm / 1000.0
        if not self._control.moveL(pose, v, a):
            raise URError("moveL (base Z) failed")

    def contact_detect_z(self, *, speed_ms: float = 0.05) -> None:
        """Probe base Z+ until tool contact; ur_rtde retracts to the contact
        point itself — the legacy program's backtrack, in one call.
        """
        if not self._control.moveUntilContact([0, 0, speed_ms, 0, 0, 0]):
            raise URError("moveUntilContact failed")

    # ---------------------------------------------------------------- force

    def zero_ft(self, settle_s: float = 0.1, pre_wait_s: float = 0.0) -> None:
        """Zero the force/torque sensor. `pre_wait_s` is a time-based stand-in
        for the legacy Robotiq rq_wait_ft_sensor_steady (force.ft_sensor
        .wait_steady_ms): let the reading settle after the contact stop before
        zeroing. ur_rtde exposes no FT-steady predicate, so the tuned wait is
        applied as a fixed delay.
        """
        if pre_wait_s > 0.0:
            time.sleep(pre_wait_s)
        self._control.zeroFtSensor()
        time.sleep(settle_s)

    def begin_force_z(self, newtons: float) -> None:
        """force_mode with the tuned frame/selection/limits; only the Z
        magnitude varies by operation (6/7/10 N in cell-config).
        """
        self._control.forceMode(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],          # task frame
            [0, 0, 1, 0, 0, 0],                       # Z compliant only
            [0.0, 0.0, newtons, 0.0, 0.0, 0.0],
            2,                                        # frame fixed to base
            [0.1, 0.1, 0.15, 0.3490658503988659, 0.3490658503988659, 0.3490658503988659],
        )

    def end_force_mode(self, decel: float | None = None) -> None:
        """Leave force mode, then optionally stopL at `decel` (m/s^2) to halt any
        residual motion — the legacy stopl(motion.stopl.on_force_end),
        script:2497.
        """
        self._control.forceModeStop()
        if decel is not None:
            self._control.stopL(decel)

    # ------------------------------------------------------------------- io

    def set_tool(self, on: bool) -> None:
        self._io.setStandardDigitalOut(self.SANDER_OUTPUT, on)

    def set_sprayer(self, on: bool) -> None:
        self._io.setStandardDigitalOut(self.SPRAYER_OUTPUT, on)

    def set_digital_out(self, output: int, on: bool) -> None:
        """Generic standard DO — e.g. the legacy-mod fan relays
        (line-config legacy_mode.fans robot_do numbers)."""
        self._io.setStandardDigitalOut(output, on)

    # ------------------------------------------------------------------- tcp

    def use_spray_tcp(self) -> None:
        """Switch to the spray tool frame (RobotSetup.spray_tcp). The spray
        waypoints' inverse-kin only solves correctly under it (script:2859).
        """
        self._control.setTcp(list(self._setup.spray_tcp))

    def use_default_tcp(self) -> None:
        """Restore the sanding/default TCP. The sand-frame waypoints (Sand_Base,
        ...) were recorded under it, so any post-spray movej must use it.
        """
        self._control.setTcp(list(self._setup.tcp))

    # ---------------------------------------------------------------- state

    def tcp_pose(self) -> list[float]:
        return self._receive.getActualTCPPose()

    def is_steady(self) -> bool:
        return self._control.isSteady()
