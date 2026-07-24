"""Legacy ClearCore driver — the 100/200-block protocol, PC as master.

Speaks to the UNMODIFIED legacy firmware (modbustest.ino) exactly the way the
old Polyscope program does (programs/modbustest.script, Conveyor_Move_mm at
script:2026): write a command into the 100-block, poll the 200-block echo until
it matches, and trigger distance moves by CHANGING REQUEST_ID. Completion is
SERVER_STATE returning to 1 (the firmware sets 2 on move start and 1 on
StepsComplete — ino:305/337).

This is the "legacy-mod" route's device layer: the interleaved choreography
driven from Python against the production line's existing firmware, no
Polyscope programming, no firmware changes, no new sensors.

TWO FIRMWARE FACTS THAT BITE
----------------------------
- Velocity/accel limits read as 0 after a ClearCore boot; VelMax(0) moves
  nothing. set_params() (the legacy Set_Conveyor_Params) MUST run before the
  first move — move_mm() enforces this.
- The direction coil is not echo-waited by the legacy program either: it is
  written before the distance register, whose echo-wait serializes both (the
  firmware copies the whole block per loop pass).

Register semantics (robot-side names, cell-config):
  holding 100 MOTION_MODE (0=distance, 1=position, 2=continuous, 3=idle)
  coil    101 DIRECTION   (True = positive mm = downstream)
  holding 102 VELOCITY / 103 ACCELERATION (steps/s, steps/s^2)
  holding 104 DISTANCE    (unsigned steps; sign travels on the coil)
  holding 106 REQUEST_ID  (move fires on CHANGE while mode==0)
  coils   107 FEED_CONVEYOR / 108 BRUSH_ON (level-driven)
  input   1   SERVER_STATE (0 not ready, 1 ready, 2 moving)
  discrete 4 WORK_AT_ZERO / 5 OFFLOAD / 6 ONLOAD (the three original sensors)
  discrete 7 STAGING (legacy v1.1 only): the choreography's park eye, ~450 mm
             past the feed junction. Since 2026-07-25 ONLOAD sits back AT the
             junction and the choreography watches STAGING instead.
  echo: input regs / discretes at command address + 100
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from ..config.loader import (
    ConveyorKinematics,
    load_conveyor_kinematics,
    load_legacy_sensor_inversion,
)
from .registers import Command, Echo, Status

_SENSOR_ADDR = {
    "work_at_zero": Status.WORK_AT_ZERO,
    "offload": Status.OFFLOAD,
    "onload": Status.ONLOAD,
    "staging": Status.STAGING,  # legacy v1.1 firmware only
}

try:
    from pymodbus.client import ModbusTcpClient
except ImportError:  # pragma: no cover
    ModbusTcpClient = None

_ECHO_OFFSET = 100  # command addr + 100 = its echo (100->200 block)

MODE_DISTANCE = 0
MODE_CONTINUOUS = 2
MODE_IDLE = 3

STATE_READY = 1
STATE_MOVING = 2


class LegacyClearCoreError(RuntimeError):
    """A command failed, an echo never confirmed, or a move never finished."""


@dataclass(frozen=True, slots=True)
class LegacyInputs:
    """One read of everything the legacy firmware reports.

    `staging` needs legacy v1.1 firmware (discrete 7) — the choreography
    requires it anyway, so read_inputs() reads it unconditionally.
    """

    server_state: int
    work_at_zero: bool
    offload: bool
    onload: bool
    staging: bool


class LegacyClearCoreClient:
    def __init__(
        self,
        host: str,
        port: int = 502,
        *,
        timeout_s: float = 2.0,
        poll_s: float = 0.02,
        echo_timeout_s: float = 3.0,
        kinematics: ConveyorKinematics | None = None,
        invert_sensors: dict[str, bool] | None = None,
    ) -> None:
        if ModbusTcpClient is None:
            raise LegacyClearCoreError("pymodbus is not installed")
        self._client = ModbusTcpClient(host, port=port, timeout=timeout_s)
        self._host = host
        self._poll_s = poll_s
        self._echo_timeout_s = echo_timeout_s
        self.kinematics = kinematics or load_conveyor_kinematics()
        # Per-sensor polarity (line-config legacy_mode.sensor_polarity): the
        # legacy firmware serves raw reads, so mixed sensor fleets (original
        # active-high eyes + F18 replacements, active-low by default) get
        # normalized HERE — True always means part present, everywhere above.
        if invert_sensors is None:
            invert_sensors = load_legacy_sensor_inversion()
        self._invert = {
            _SENSOR_ADDR[name]: flag for name, flag in invert_sensors.items()
        }
        self._request_id = 0
        self._params_pushed = False
        #: Remaining ONLOAD edges of a feed watch handed over by
        #: transition_move(stop_on_staging) — non-empty means Z1 is running
        #: and feed_tick() owns the cut.
        self._feed_phases: list[str] = []

    def _sensor(self, addr: int) -> bool:
        """A sensor read normalized to True = part present."""
        raw = self._read_discrete(addr)
        return (not raw) if self._invert.get(addr, False) else raw

    # ------------------------------------------------------------- lifecycle

    def connect(self) -> "LegacyClearCoreClient":
        if not self._client.connect():
            raise LegacyClearCoreError(f"cannot reach legacy ClearCore at {self._host}")
        # Seed the request-id from the firmware's echo of the LAST id it saw.
        # Moves fire on id CHANGE only; a fresh process starting from 0 would
        # re-write the same id as the previous run and the firmware would
        # (correctly) ignore the move. The legacy program solved this with
        # random ids (script:2097); seeding + increment is the deterministic
        # equivalent.
        self._request_id = self._read_input_reg(Echo.REQUEST_ID)
        return self

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "LegacyClearCoreClient":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------- low level

    def _read_input_reg(self, addr: int) -> int:
        r = self._client.read_input_registers(addr, count=1)
        if r.isError():
            raise LegacyClearCoreError(f"read_input_registers({addr}) failed: {r}")
        return r.registers[0]

    def _read_discrete(self, addr: int) -> bool:
        r = self._client.read_discrete_inputs(addr, count=1)
        if r.isError():
            raise LegacyClearCoreError(f"read_discrete_inputs({addr}) failed: {r}")
        return bool(r.bits[0])

    def _poll_until(self, predicate, timeout_s: float, what: str) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(self._poll_s)
        raise LegacyClearCoreError(f"timeout waiting for {what} ({timeout_s:.1f}s)")

    def _write_reg_echoed(self, addr: int, value: int) -> None:
        """Write a 100-block holding register and wait for its 200-block echo —
        the legacy handshake (their Modbus client gave no write confirmation;
        the echo doubles as firmware-loop acknowledgement, which we still want).
        """
        if self._client.write_register(addr, value).isError():
            raise LegacyClearCoreError(f"write_register({addr}, {value}) failed")
        self._poll_until(
            lambda: self._read_input_reg(addr + _ECHO_OFFSET) == value,
            self._echo_timeout_s,
            f"echo of reg {addr} == {value}",
        )

    def _write_coil(self, addr: int, value: bool) -> None:
        if self._client.write_coil(addr, value).isError():
            raise LegacyClearCoreError(f"write_coil({addr}, {value}) failed")

    # ------------------------------------------------------------- commands

    def set_params(self, velocity_sps: int | None = None, accel_sps2: int | None = None) -> None:
        """Set_Conveyor_Params (script:1984). MANDATORY after ClearCore boot —
        limits default to 0 and VelMax(0) silently moves nothing. Defaults are
        the tuned production values (1600 / 16000).
        """
        v = velocity_sps if velocity_sps is not None else self.kinematics.velocity_steps_per_sec
        a = accel_sps2 if accel_sps2 is not None else self.kinematics.acceleration_steps_per_sec2
        self._write_reg_echoed(Command.VELOCITY, v)
        self._write_reg_echoed(Command.ACCELERATION, a)
        self._params_pushed = True

    def _start_move(self, mm: float) -> tuple[int, float]:
        """Arm and fire a distance move; returns (steps, suggested timeout)."""
        if not self._params_pushed:
            self.set_params()

        self._write_reg_echoed(Command.MOTION_MODE, MODE_DISTANCE)
        steps = self.kinematics.mm_to_steps(mm)
        if steps > 65535:
            raise LegacyClearCoreError(
                f"{abs(mm):.0f} mm = {steps} steps overflows the 16-bit DISTANCE "
                "register (max 2184 mm per distance move) — use continuous mode "
                "or split the move"
            )
        # Direction coil, no echo wait — matching the legacy program; the
        # distance echo below serializes it (same firmware copy loop).
        self._write_coil(Command.DIRECTION, mm >= 0)
        self._write_reg_echoed(Command.DISTANCE, steps)

        # Move fires on REQUEST_ID *change* (ino:305). Legacy used random
        # 0..65000; seeded increment (connect()) is the deterministic version.
        self._request_id = self._request_id % 65000 + 1
        self._write_reg_echoed(Command.REQUEST_ID, self._request_id)

        v = max(self.kinematics.velocity_steps_per_sec, 1)
        return steps, steps / v + 10.0

    def start_move_mm(self, mm: float) -> int:
        """Arm and fire a distance move; returns immediately after the
        request-id echo (move accepted and running). Pair with wait_ready().
        Public so composites can overlap belt and robot motion, exactly like
        the legacy program (e.g. the spray belt-return during a movej).
        """
        steps, _ = self._start_move(mm)
        return steps

    def wait_ready(self, timeout_s: float = 60.0) -> None:
        """Block until SERVER_STATE returns to READY (move complete)."""
        self._poll_until(
            lambda: self._read_input_reg(Status.SERVER_STATE) == STATE_READY,
            timeout_s,
            "SERVER_STATE READY",
        )

    def move_mm(self, mm: float, *, timeout_s: float | None = None) -> int:
        """Conveyor_Move_mm (script:2026): distance move, sign = direction
        (positive = downstream). Blocks until SERVER_STATE returns to READY.
        Returns commanded steps.

        Echo confirmed => the firmware pass that copied the id also started the
        move (STATE=2 in the same loop iteration), so STATE==1 from here means
        COMPLETE, never not-started-yet.
        """
        steps, suggested = self._start_move(mm)
        self.wait_ready(timeout_s if timeout_s is not None else suggested)
        return steps

    def transition_move(
        self,
        nominal_mm: float,
        *,
        stop_on_work_zero: bool = False,
        stop_on_staging: bool = False,
        o_occupied: bool = False,
        pass_through: bool = False,
        feed: bool = False,
        continuous: bool = False,
        overshoot_mm: float = 400.0,
        reapproach_cap_mm: float = 60.0,
        timeout_s: float | None = None,
    ) -> dict:
        """One interleave transition on the single belt.

        Commands a distance move of |nominal| + overshoot in nominal's
        direction — the overshoot is a runaway CAP, not a target — and watches
        sensors while the belt runs:

        - stop_on_work_zero: position truth at O comes from the WORK_AT_ZERO
          eye, exactly like the legacy program. Phase chain: if `o_occupied`,
          first the departing part passes over the eye (HI then LO — robust to
          where exactly it rested); then the arriving part trips it (HI). A
          downstream arrival stops there (legacy load, script:2390). An
          upstream arrival (`pass_through`, the F2->O retreat) instead runs
          HI -> LO and then RE-APPROACHES downstream until HI — the legacy
          return-to-zero (script:3191-3206) — so every part rests at O having
          approached from the same side, direction-independent.
        - feed: the queue (Z1) belt runs for the WHOLE maneuver by default;
          the ONLY early cut is the junction chain on ONLOAD (2026-07-25
          rule): HI (part 1's nose) -> LO (part 1's tail clears) -> HI
          (part 2's nose arrives) -> feed CUT, so the follower does not board
          Z2. If ONLOAD is already HI at start, part 1 is the one on the eye
          and the chain starts mid-way (LO -> HI remaining). 'entered' True =
          chain completed (follower parked at the junction); False = no
          follower appeared before the move ended (nothing waiting behind —
          informational, not a fault).
        - stop_on_staging: Z2 stops on the STAGING eye's rising edge (the
          intake park). If the feed chain is still incomplete at that point,
          Z1 is deliberately LEFT RUNNING with NO timeout — the queue belt
          feeds continuously until a follower trips ONLOAD. The caller owns
          the rest of the watch via feed_tick() (nonblocking, one poll per
          call); 'feed_running' True in the result marks this handoff.
          Exceptions still cut the feed.

        - continuous: run in Move_Continuous instead of a capped distance move —
          for sensor-terminated maneuvers longer than the 16-bit DISTANCE
          register allows (2184 mm), i.e. the legacy-style direct load. Requires
          stop_on_work_zero (the sensor IS the stop); the deadline is the only
          other bound, and the belt is idled on any failure path.

        Returns {'arrived', 'entered', 'feed_running', 'seconds'} (None = not
        requested; False = requested but not achieved — investigate).
        """
        if continuous and not stop_on_work_zero:
            raise ValueError("continuous transition requires stop_on_work_zero")
        if stop_on_work_zero and stop_on_staging:
            raise ValueError("pick ONE stop sensor: work_zero or staging")
        self._feed_phases = []  # a new maneuver supersedes any old feed watch
        if feed:
            self.set_feed(True)
        arrived: bool | None = None
        entered: bool | None = None
        t0 = time.monotonic()
        try:
            if continuous:
                self.move_continuous(downstream=nominal_mm >= 0)
                deadline = time.monotonic() + (timeout_s if timeout_s is not None else 120.0)
            else:
                cap = abs(nominal_mm) + (overshoot_mm if stop_on_work_zero else 0.0)
                _steps, suggested = self._start_move(cap if nominal_mm >= 0 else -cap)
                deadline = time.monotonic() + (timeout_s if timeout_s is not None else suggested)

            wz_phases: list[str] = []
            if stop_on_work_zero:
                arrived = False
                if o_occupied:
                    wz_phases += ["depart_hi", "depart_lo"]
                wz_phases += ["arrive_hi"] + (["arrive_lo"] if pass_through else [])
            if stop_on_staging:
                arrived = False
            feed_phases: list[str] = []
            if feed:
                entered = False
                feed_phases = (["lo", "hi"] if self._sensor(Status.ONLOAD)
                               else ["hi", "lo", "hi"])

            while True:
                if time.monotonic() >= deadline:
                    raise LegacyClearCoreError(
                        f"timeout in transition move ({nominal_mm:+.0f} mm nominal)"
                    )
                if wz_phases:
                    wz = self._sensor(Status.WORK_AT_ZERO)
                    ph = wz_phases[0]
                    if (ph.endswith("hi") and wz) or (ph.endswith("lo") and not wz):
                        wz_phases.pop(0)
                        if not wz_phases:  # final edge of the chain: stop here
                            arrived = True
                            self._write_reg_echoed(Command.MOTION_MODE, MODE_IDLE)
                if stop_on_staging and arrived is False:
                    if self._sensor(Status.STAGING):  # intake park: stop Z2
                        arrived = True
                        self._write_reg_echoed(Command.MOTION_MODE, MODE_IDLE)
                if feed_phases:
                    on = self._sensor(Status.ONLOAD)
                    ph = feed_phases[0]
                    if (ph == "hi" and on) or (ph == "lo" and not on):
                        feed_phases.pop(0)
                        if not feed_phases:  # follower at the junction: cut Z1
                            entered = True
                            self.set_feed(False)
                if self._read_input_reg(Status.SERVER_STATE) == STATE_READY:
                    break
                time.sleep(self._poll_s)

            # Legacy return-to-zero tail: after an upstream pass, nudge back
            # DOWNSTREAM until the eye reads HI (script:3201-3206).
            if pass_through and arrived:
                _s, _t = self._start_move(reapproach_cap_mm)
                arrived = False
                while time.monotonic() < deadline + 15.0:
                    if self._sensor(Status.WORK_AT_ZERO):
                        arrived = True
                        self._write_reg_echoed(Command.MOTION_MODE, MODE_IDLE)
                    if self._read_input_reg(Status.SERVER_STATE) == STATE_READY:
                        break
                    time.sleep(self._poll_s)

            # Intake handoff: Z2 parked at staging but no follower yet — hand
            # the ONLOAD watch to the caller (feed_tick) and leave Z1 running.
            # NO timeout by design: the queue belt feeds until a part shows.
            if feed and not entered and stop_on_staging:
                self._feed_phases = feed_phases
            return {"arrived": arrived, "entered": entered,
                    "feed_running": bool(self._feed_phases),
                    "seconds": time.monotonic() - t0}
        except BaseException:
            # Deadline, Modbus failure, Ctrl-C — never leave the belt running
            # (vital in continuous mode, harmless in distance mode).
            try:
                self.move_idle()
            except Exception:
                pass
            raise
        finally:
            if feed and not self._feed_phases:
                self.set_feed(False)  # never leave the queue belt running

    def feed_tick(self) -> bool | None:
        """One nonblocking poll of a feed watch left running by
        transition_move(stop_on_staging): advances the ONLOAD chain and cuts
        Z1 the moment the follower's nose trips the eye. Returns True on the
        cut (watch complete), False while still waiting, None if no watch is
        active. NO internal timeout — an empty queue just keeps the feed
        running, by design; call every tick until True."""
        if not self._feed_phases:
            return None
        on = self._sensor(Status.ONLOAD)
        ph = self._feed_phases[0]
        if (ph == "hi" and on) or (ph == "lo" and not on):
            self._feed_phases.pop(0)
            if not self._feed_phases:  # follower at the junction: cut Z1
                self.set_feed(False)
                return True
        return False

    def move_idle(self) -> None:
        """Move_Idle (script:2083): decel-stop the main conveyor."""
        self._write_reg_echoed(Command.MOTION_MODE, MODE_IDLE)

    def move_continuous(self, *, downstream: bool) -> None:
        """Move_Continuous (script:2065): run the belt until told otherwise.
        UNBOUNDED — callers own the stop condition. Used by sensor-terminated
        maneuvers too long for the 16-bit distance register (the legacy load).
        """
        self._write_coil(Command.DIRECTION, downstream)
        self._write_reg_echoed(Command.MOTION_MODE, MODE_CONTINUOUS)

    def set_feed(self, on: bool) -> None:
        """Feed conveyor (coil 107): level-driven, runs while set."""
        self._write_coil(Command.FEED_CONVEYOR, on)

    def set_brush(self, on: bool) -> None:
        self._write_coil(Command.BRUSH_ON, on)

    def staging_present(self) -> bool:
        """The staging eye — legacy v1.1 firmware only (discrete 7, polarity
        normalized like every other sensor). True = a part is parked at the
        staging point ~450 mm past the feed junction; NOT an infeed-queue
        signal — nothing watches the queue upstream of the junction. Against
        the original v1.0 firmware this raises LegacyClearCoreError."""
        return self._sensor(Status.STAGING)

    def stage_next(self, *, feed_timeout_s: float = 10.0, nudge_timeout_s: float = 15.0,
                   nudge: bool = True) -> dict:
        """Two-phase staging: park the queue head at the STAGING eye.

        Phase A — FEED ONLY (spacing-neutral: the main belt does not move, so
        the pair spacing and retreat depth are unaffected). The crossing onto
        the static main belt can stall near the end (belt friction once the
        part's weight leaves the feed) — physics, not a fault.

        Phase B — the NUDGE (only if A stalls): Z2 alone, capped, stops on
        STAGING HI. The mostly-aboard part grips the moving belt (consistent
        handoff) and parks at the eye. Downstream parts slide by the stall
        shortfall δ — run staging only at BEAT-END (work done, O part
        departing next move) where that slide is harmless. δ inflates spacing
        and shifts the retreat landing by −δ. The 2026-07-25 mount puts the
        eye at junction+450 ≈ part width (362) + 88 mm of δ-margin — right at
        the placement rule; watch the reported nudge distance.

        Z1 CUT RULE (2026-07-25): the feed is governed ONLY by the junction
        chain on ONLOAD — never by STAGING, never by a timeout. The staged
        part starts ON the eye (chain LO -> HI: it leaves, the next follower
        arrives) or short of it (HI -> LO -> HI). If the chain is incomplete
        when this returns, Z1 is LEFT RUNNING and feed_tick() owns the cut —
        an empty queue keeps feeding until a part shows, by design.
        Exceptions still cut the feed.

        Returns {'staged', 'nudged', 'nudge_s', 'feed_running'}.
        """
        self._feed_phases = []
        self.set_feed(True)
        phases = (["lo", "hi"] if self._sensor(Status.ONLOAD)
                  else ["hi", "lo", "hi"])
        entered = False

        def chain_tick() -> None:
            nonlocal entered
            if entered:
                return
            on = self._sensor(Status.ONLOAD)
            ph = phases[0]
            if (ph == "hi" and on) or (ph == "lo" and not on):
                phases.pop(0)
                if not phases:  # next follower at the junction: cut Z1
                    entered = True
                    self.set_feed(False)

        def result(staged: bool, nudged: bool, nudge_s: float) -> dict:
            if not entered:
                self._feed_phases = phases  # hand the watch to feed_tick()
            return {"staged": staged, "nudged": nudged, "nudge_s": nudge_s,
                    "feed_running": not entered}

        try:
            deadline = time.monotonic() + feed_timeout_s
            while time.monotonic() < deadline:
                chain_tick()
                if self._sensor(Status.STAGING):
                    return result(True, False, 0.0)
                time.sleep(self._poll_s)
            if not nudge:
                return result(False, False, 0.0)
            # Capped DISTANCE move, not continuous: if the PC dies mid-nudge
            # the firmware finishes at most nudge_cap_mm and stops on its own
            # (the legacy firmware has no watchdog — caps are the mitigation).
            # Z2 only — if the chain already cut Z1, re-running the feed here
            # would push the follower past the junction.
            t0 = time.monotonic()
            self._start_move(400.0)  # nudge cap: well past any stall shortfall
            try:
                deadline = time.monotonic() + nudge_timeout_s
                while time.monotonic() < deadline:
                    chain_tick()
                    if self._sensor(Status.STAGING):
                        return result(True, True, time.monotonic() - t0)
                    if self._read_input_reg(Status.SERVER_STATE) == STATE_READY:
                        break  # cap reached without the eye firing
                    time.sleep(self._poll_s)
                return result(False, True, time.monotonic() - t0)
            finally:
                self.move_idle()  # nudge always ends with the belt stopped
        except BaseException:
            try:
                self.set_feed(False)
                self.move_idle()
            except Exception:
                pass
            raise

    # -------------------------------------------------------------- sensing

    def read_inputs(self) -> LegacyInputs:
        return LegacyInputs(
            server_state=self._read_input_reg(Status.SERVER_STATE),
            work_at_zero=self._sensor(Status.WORK_AT_ZERO),
            offload=self._sensor(Status.OFFLOAD),
            onload=self._sensor(Status.ONLOAD),
            staging=self._sensor(Status.STAGING),
        )
