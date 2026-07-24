"""LegacySequencer — the interleaved schedule on the legacy line, automatic.

The direct-entry choreography validated on the real line (2026-07-22,
docs/legacy-mod-choreography.md) with the robot folded in:

    beat P1  robot: LEAD sand+coat1   -> P1->P2 : trail ENTERS queue->O
    beat P2  robot: TRAIL sand+coat1  -> P2->P3 : lead retreats F2->O
    beat P3  robot: LEAD clean+coat2  -> P3->P4 : trail returns F1->O
    beat P4  robot: TRAIL clean+coat2 -> P4->P1': next lead ENTERS

Reused from the core untouched: PartState + the flash discipline (bank fan-on
seconds, never under-flash), ProcessConfig, SCHEDULE's beat vocabulary, the
robot composites. Interlocks are STRUCTURAL: one thread owns belt and robot,
so nothing moves concurrently by construction — the honesty of this route.

Fan truth (line-config legacy_mode.fans): 'always_on' banks wall-clock
(parity with today's line), 'robot_do' follows the commanded DO incl. the P3
spray-burst pause, 'none' banks wall-clock with a loud warning. Command is
truth — there is no feedback wire on this route.

Phases are sized for the agreed boundary-halt: robot work and belt moves
block (composites are the ~90 s worst case); flash waits tick at tick_s so
pause/halt respond immediately where the line spends most of its time.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import replace

from ..config.loader import FanMode, ProcessConfig
from ..core.model import PartRole, PartState, Product, Station
from ..core.schedule import BEATS, SCHEDULE, next_beat
from .legacy_train import LegacyTrain
from .robot import RobotDevice

PHASE_IDLE = "idle"
PHASE_ROBOT = "robot_work"
PHASE_FLASH = "flash_wait"
PHASE_STAGE = "stage"
PHASE_TRANSITION = "transition"
PHASE_FAULTED = "faulted"

#: Beats whose outgoing transition is an ENTRY (and, at steady state, an exit).
_ENTRY_BEATS = ("P1", "P4")


class LegacySequencer:
    def __init__(
        self,
        train: LegacyTrain,
        robot: RobotDevice,
        cfg: ProcessConfig,
        fans: dict[str, FanMode],
        *,
        fan_do: Callable[[int, bool], None] | None = None,
        continuous_intake: bool = True,
        intake_product: str = "cube",
        tick_s: float = 0.2,
        on_change: Callable[[], None] | None = None,
    ) -> None:
        self._train = train
        self._robot = robot
        self._cfg = cfg
        self._fans = fans
        self._fan_do = fan_do
        self._tick_s = tick_s
        self._on_change = on_change or (lambda: None)

        self.parts: dict[str, PartState] = {}
        self.occ: dict[Station, str] = {}
        self.queue: list[str] = []
        self.completed: list[str] = []
        self.declared = 0
        # Continuous intake: the line runs all day; parts are anonymous until
        # the infeed sensors deliver one, then an identity is MINTED. Declared
        # ids (the batch panel, still useful for testing) are consumed first.
        self._continuous = continuous_intake
        self.intake_product = intake_product
        self.minted = 0
        self._pair = -1
        self._pending_enterer: str | None = None
        #: Belt travel a starved stage-probe's nudge already caused this beat;
        #: the next blind shuffle subtracts it so stations stay true.
        self._probe_slide_mm = 0.0
        self.beat: str = BEATS[0]
        self.phase: str = PHASE_IDLE
        self.fault: str | None = None
        self.spraying = False
        self.stage_note: str = ""
        self._f1_commanded = fans["f1"].kind == "always_on"
        self._last_bank = time.monotonic()
        self.sensors = None  # last LegacyInputs, published to the HMI

        if fans["f1"].kind == "none":
            print("WARNING: no F1 fan configured — trail flash-1 banks wall-clock "
                  "with NO airflow. Do not trust flash validation in this state.")

    # -------------------------------------------------------------- helpers

    def _fan_on(self, station: Station) -> bool:
        mode = self._fans["f1" if station is Station.F1 else "f2"]
        if mode.kind == "robot_do":
            return self._f1_commanded if station is Station.F1 else True
        return True  # always_on and none both pass wall-clock (none warns once)

    def _set_f1(self, on: bool) -> None:
        mode = self._fans["f1"]
        if mode.controllable and self._f1_commanded != on:
            self._f1_commanded = on
            if self._fan_do is not None and mode.do is not None:
                self._fan_do(mode.do, on)

    def bank(self) -> None:
        """Credit flash time — call often; wall-clock between calls is banked
        for every coated part sitting at a fan station whose fan is on."""
        now = time.monotonic()
        dt, self._last_bank = now - self._last_bank, now
        if dt <= 0:
            return
        for station in (Station.F1, Station.F2):
            pid = self.occ.get(station)
            if pid is None:
                continue
            part = self.parts[pid]
            if part.coats_applied < 1 or not self._fan_on(station):
                continue
            part = part.with_flash_advanced(dt)
            if part.is_wet and part.active_flash_seconds() >= self._cfg.flash_seconds:
                part = replace(part, is_wet=False)
            self.parts[pid] = part

    def _flash_block(self, pid: str, station: str) -> str | None:
        part = self.parts[pid]
        if part.coats_applied < 1:
            return None
        need = self._cfg.flash_seconds - part.active_flash_seconds()
        if need > 0:
            return f"flash: {pid} needs {need:.0f}s more at {station}"
        return None

    def _fault(self, reason: str) -> str:
        self.fault = reason
        self.phase = PHASE_FAULTED
        try:
            self._train.idle()
        except Exception:
            pass
        self._on_change()
        return reason

    # ------------------------------------------------------------ the loop

    def step(self) -> str | None:
        """One bounded action. Returns a blocked/waiting reason or None.
        Blocking spans: robot work (longest), belt moves; flash waits return
        after one tick so the controller stays responsive."""
        self.bank()
        if self.phase == PHASE_FAULTED:
            return self.fault
        # The persistent Z1 watch (no timeout, by design): whenever a stage
        # or intake left the feed running, one nonblocking poll per step cuts
        # it the moment a follower trips ONLOAD — including while the robot
        # works or a flash wait ticks.
        try:
            self._train.feed_tick()
        except Exception as exc:
            return self._fault(f"feed watch failed: {exc}")
        if self.phase == PHASE_IDLE:
            return self._step_idle()
        if self.phase == PHASE_ROBOT:
            return self._step_robot()
        if self.phase == PHASE_FLASH:
            return self._step_flash()
        if self.phase == PHASE_STAGE:
            return self._step_stage()
        if self.phase == PHASE_TRANSITION:
            return self._step_transition()
        raise AssertionError(self.phase)

    def _mint(self, role: PartRole) -> str:
        """Continuous intake: identity is created when the sensors deliver a
        physical part. Roles derive from the BEAT (P4/idle entry = LEAD,
        P1 entry = TRAIL), so holes in the pattern can never break pairing."""
        prod = Product(self.intake_product)
        self.minted += 1
        pid = f"{self.intake_product[0]}{self.minted:04d}"
        if role is PartRole.LEAD:
            self._pair += 1
        self.parts[pid] = PartState(
            part_id=pid, product=prod, role=role, pair_index=max(self._pair, 0))
        self._on_change()
        return pid

    def set_intake_product(self, product: str) -> None:
        Product(product)  # validate
        self.intake_product = product
        self._on_change()

    def _step_idle(self) -> str | None:
        if self.occ:
            # Restored/odd state: parts on the belt while idle is a controller
            # decision (confirm-or-clear), never something to run through.
            time.sleep(self._tick_s)
            return "parts on the line — confirm occupancy or clear before running"
        if not self.queue and not self._continuous:
            time.sleep(self._tick_s)
            return "line empty — declare a batch to begin"
        # Intake (validated 2026-07-25, scripts/handoff_test.py): BOTH belts
        # run and the first part parks at STAGING; Z1 obeys the junction rule
        # and stays running between attempts (an empty queue just keeps
        # feeding). Z2's capped move re-fires each idle pass until a part
        # arrives. Once staged, the normal sensor-stopped entry takes it to O
        # — the same staged-entry path as every later part.
        try:
            self.sensors = sens = self._train.sensors()  # keep the HMI live
            if not sens.staging:
                res = self._train.intake()
                if not res.get("arrived"):
                    if not self._continuous:
                        return self._fault("declared parts not found at the infeed")
                    return "waiting for parts — intake running (feed stays on)"
            res = self._train.entry(o_occupied=False)
        except Exception as exc:
            return self._fault(f"belt failed during intake: {exc}")
        if not res.get("arrived"):
            return self._fault("first entry never reached work-zero")
        enterer = self.queue.pop(0) if self.queue else self._mint(PartRole.LEAD)
        self.occ[Station.O] = enterer
        self.beat = BEATS[0]
        self.phase = PHASE_ROBOT
        self.sensors = self._train.sensors()
        self._on_change()
        return None

    def _step_robot(self) -> str | None:
        spec = SCHEDULE[self.beat]
        pid = self.occ.get(Station.O)
        if pid is not None:
            part = self.parts[pid]
            # No role-mismatch fault: with skip-and-drain holes, roles are
            # descriptive. The real guard is coat idempotence — a part only
            # receives the beat's coat if it NEEDS it; a mismatched part just
            # waits a beat (over-flash is safe by design, §6).
            if part.coats_applied < spec.robot.coat:
                # The robot composite is the one long span nothing polls: a
                # follower arriving at ONLOAD mid-sand would be pushed across
                # the junction onto a sweeping Z2. Pause the hunt; the flash
                # wait right after polls every tick.
                resume_feed = self._train.feed_suspend()
                try:
                    if spec.robot.clean_gun:
                        self._robot.clean_gun(pid)
                    else:
                        self._robot.sand(pid)
                    # §7 heritage: never blow on a wet F1 part with the gun
                    # live. Only possible with a controllable F1 fan.
                    f1_pid = self.occ.get(Station.F1)
                    pausing = (f1_pid is not None and self.parts[f1_pid].is_wet
                               and self._fans["f1"].controllable)
                    if pausing:
                        self._set_f1(False)
                    self.bank()  # close the banking span before the pause
                    self.spraying = True
                    try:
                        self._robot.spray(pid, spec.robot.coat)
                    finally:
                        self.spraying = False
                    self.bank()  # spray span banks (or not) under pause state
                    if pausing:
                        self._set_f1(True)
                    self._robot.safe_pose()
                except Exception as exc:  # device failure = fault, belt idled
                    # No feed_resume here: _fault idles everything, and a live
                    # coil with a cancelled watch is the known disaster combo.
                    return self._fault(f"robot failed during {self.beat}: {exc}")
                if resume_feed:
                    self._train.feed_resume()
                self.parts[pid] = replace(
                    part, coats_applied=spec.robot.coat, is_wet=True)
                self._on_change()
        self.phase = PHASE_FLASH
        return None

    def _step_flash(self) -> str | None:
        self.sensors = self._train.sensors()
        blocked = self._departure_block()
        if blocked:
            time.sleep(self._tick_s)
            return blocked
        entry = self.beat in _ENTRY_BEATS and (bool(self.queue) or self._continuous)
        self.phase = PHASE_STAGE if entry else PHASE_TRANSITION
        return None

    def _departure_block(self) -> str | None:
        if self.beat in _ENTRY_BEATS:
            pid = self.occ.get(Station.F2)
            if pid:
                block = self._flash_block(pid, "F2")
                if block:
                    return block
                if self.sensors is not None and self.sensors.offload:
                    return "remove the finished part at OUT"
        elif self.beat == "P2":
            pid = self.occ.get(Station.F2)
            if pid:
                block = self._flash_block(pid, "F2")
                if block:
                    return block
        else:  # P3
            pid = self.occ.get(Station.F1)
            if pid:
                block = self._flash_block(pid, "F1")
                if block:
                    return block
        return None

    def _step_stage(self) -> str | None:
        try:
            self.sensors = sens = self._train.sensors()
        except Exception as exc:
            return self._fault(f"sensor read failed before staging: {exc}")
        if not sens.onload:
            # Junction empty: the Z1 watch parks every ready follower ON the
            # ONLOAD eye, so a dark eye means there is nothing to stage —
            # skip without the feed window or the max-nudge belt slide
            # (observed waste, 2026-07-25). A rare mid-junction straggler
            # (previous stage failed outright) stays put for the operator.
            if not self._continuous:
                return self._fault("declared parts not found at the infeed")
            self.stage_note = "junction empty — entry skipped"
            self._pending_enterer = None
            self.phase = PHASE_TRANSITION
            return None
        try:
            res = self._train.stage()
        except Exception as exc:
            return self._fault(f"belt failed during staging: {exc}")
        if not res["staged"]:
            if not self._continuous:
                return self._fault("staging failed (feed + nudge) — check the junction")
            # SKIP the beat (decision 2026-07-25): entries only ever run from
            # a confirmed staged position — an unstaged part stays on Z1
            # where a stopped belt can't disturb it, and the next beat-end
            # stage gets another try. The nudge is still the jam
            # discriminator (anything mid-junction surfaces within its cap),
            # and its belt travel already slid every part by its measured
            # distance — record it so the following shuffle compensates.
            if res.get("nudged"):
                self._probe_slide_mm = res["nudge_s"] * self._train.mm_per_s
                self.stage_note = (f"nothing staged — entry skipped "
                                   f"(nudge slid ~{self._probe_slide_mm:.0f} mm; "
                                   f"shuffle compensates)")
            else:
                self.stage_note = "nothing staged — entry skipped"
            self._pending_enterer = None
            self.phase = PHASE_TRANSITION
            return None
        if res["nudged"]:
            self.stage_note = (f"staged via nudge "
                               f"~{res['nudge_s'] * self._train.mm_per_s:.0f} mm")
        else:
            self.stage_note = "staged (feed only)"
        role = PartRole.TRAIL if self.beat == "P1" else PartRole.LEAD
        self._pending_enterer = (
            self.queue.pop(0) if self.queue else self._mint(role))
        self.phase = PHASE_TRANSITION
        return None

    def _step_transition(self) -> str | None:
        beat = self.beat
        try:
            if beat in _ENTRY_BEATS:
                if self._pending_enterer is not None:
                    if self.occ.get(Station.F1):
                        return self._fault(
                            "F1 occupied at an entry beat — impossible occupancy")
                    res = self._train.entry(
                        o_occupied=self.occ.get(Station.O) is not None)
                    if not res.get("arrived"):
                        return self._fault("entry never reached work-zero")
                    enterer, self._pending_enterer = self._pending_enterer, None
                    self._shift_downstream(enterer=enterer)
                elif self.occ:
                    self._train.blind(reduce_mm=self._probe_slide_mm)
                    self._shift_downstream(enterer=None)
            elif beat == "P2":
                if self.occ.get(Station.F2):
                    res = self._train.retreat(
                        o_occupied=self.occ.get(Station.O) is not None)
                    if not res.get("arrived"):
                        return self._fault("retreat never confirmed at work-zero")
                    if self.occ.get(Station.O):
                        self.occ[Station.F1] = self.occ.pop(Station.O)
                    self.occ[Station.O] = self.occ.pop(Station.F2)
                # else: lone part keeps O for coat 2 — no move
            else:  # P3
                if self.occ.get(Station.F1):
                    if self.occ.get(Station.F2):
                        return self._fault("F2 occupied during the P3 return — impossible occupancy")
                    res = self._train.return_to_o(
                        o_occupied=self.occ.get(Station.O) is not None)
                    if not res.get("arrived"):
                        return self._fault("return never confirmed at work-zero")
                    if self.occ.get(Station.O):
                        self.occ[Station.F2] = self.occ.pop(Station.O)
                    self.occ[Station.O] = self.occ.pop(Station.F1)
                elif self.occ:
                    self._train.blind()
                    self._shift_downstream(enterer=None)
        except Exception as exc:
            return self._fault(f"belt failed during {beat} transition: {exc}")

        self._probe_slide_mm = 0.0  # consumed (or made moot) by this transition
        self.sensors = self._train.sensors()
        self.beat = next_beat(beat)
        # F1 fan for the new beat (robot_do mode): on iff a wet part rests there.
        f1_pid = self.occ.get(Station.F1)
        self._set_f1(f1_pid is not None and self.parts[f1_pid].is_wet)
        if not self.occ and not self.queue:
            self.phase = PHASE_IDLE
        else:
            self.phase = PHASE_ROBOT
        self._on_change()
        return None

    def _shift_downstream(self, *, enterer: str | None) -> None:
        pid = self.occ.pop(Station.F2, None)
        if pid:
            self.completed.append(pid)
        if self.occ.get(Station.O):
            self.occ[Station.F2] = self.occ.pop(Station.O)
        if self.occ.get(Station.F1):
            self.occ[Station.O] = self.occ.pop(Station.F1)
        if enterer is not None:
            self.occ[Station.O] = enterer

    # ------------------------------------------------------------ commands

    def declare_batch(self, product: str, part_ids: list[str]) -> list[str]:
        prod = Product(product)
        for pid in part_ids:
            if pid in self.parts or pid in self.queue:
                raise ValueError(f"part id {pid!r} already exists")
            role = PartRole.LEAD if self.declared % 2 == 0 else PartRole.TRAIL
            self.parts[pid] = PartState(
                part_id=pid, product=prod, role=role, pair_index=self.declared // 2)
            self.declared += 1
            self.queue.append(pid)
        self._on_change()
        return part_ids

    def halt_now(self, reason: str) -> None:
        """Boundary halt: called between steps by the controller thread."""
        self._fault(reason)

    def clear_line(self) -> None:
        """Operator confirmed the belt is physically empty: drop everything."""
        for pid in list(self.occ.values()):
            self.parts.pop(pid, None)
        self.occ.clear()
        self.fault = None
        self.phase = PHASE_IDLE
        self._on_change()

    def confirm_occupancy(self, occupancy: dict[Station, str], beat: str | None) -> str | None:
        """Operator confirmed which part is where after a restart. Resumes at
        ROBOT_WORK of the given (or persisted) beat — idempotence in
        _step_robot makes re-entering a completed beat safe."""
        unknown = [pid for pid in occupancy.values() if pid not in self.parts]
        if unknown:
            return f"unknown part ids: {unknown}"
        self.occ = dict(occupancy)
        if beat:
            self.beat = beat
        self.fault = None
        self.phase = PHASE_ROBOT if self.occ else PHASE_IDLE
        self._on_change()
        return None
