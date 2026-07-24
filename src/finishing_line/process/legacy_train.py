"""LegacyTrain — the validated choreography verbs over the legacy driver.

Thin by design: every method is one of the maneuvers proven on the real line
(2026-07-22, docs/legacy-mod-choreography.md), expressed once so the sequencer
reads like the choreography it runs. All belt motion is CAPPED distance moves
(crash-safe: a dead PC leaves at most the cap, then the firmware stops).

Nominal distances are runaway caps, not targets — every arrival is
sensor-stopped. Defaults reflect the measured geometry (staging eye->WZ
~625 mm since the 2026-07-25 eye remount at junction+450, queue->WZ
~1115 mm); they only matter as caps and for the blind fill/drain shuffles.
"""

from __future__ import annotations

from ..devices.legacy_clearcore import LegacyClearCoreClient, LegacyInputs


class LegacyTrain:
    def __init__(
        self,
        cc: LegacyClearCoreClient,
        *,
        load_nominal_mm: float = 1200.0,
        pitch_nominal_mm: float = 750.0,
    ) -> None:
        self._cc = cc
        self._load_mm = load_nominal_mm
        self._pitch_mm = pitch_nominal_mm
        #: Last measured sensor-stopped travel — the live spacing estimate,
        #: used for blind fill/drain shuffles once one exists.
        self.last_spacing_mm: float | None = None

    @property
    def mm_per_s(self) -> float:
        k = self._cc.kinematics
        return k.velocity_steps_per_sec / k.microsteps_per_mm

    def _record(self, res: dict) -> dict:
        if res.get("arrived"):
            mmps = self._cc.kinematics.velocity_steps_per_sec / self._cc.kinematics.microsteps_per_mm
            self.last_spacing_mm = res["seconds"] * mmps
        return res

    # ------------------------------------------------------------ maneuvers

    def intake(self) -> dict:
        """Empty-line intake: BOTH belts run; the first part rides queue ->
        STAGING and Z2 parks there (validated 2026-07-25, handoff_test phase
        1). Z1 obeys the junction rule throughout and may be LEFT RUNNING —
        feed_tick() owns the cut once a follower shows."""
        return self._cc.transition_move(
            self._load_mm, stop_on_staging=True, feed=True)

    def stage(self, **kwargs) -> dict:
        """JIT staging: feed-only bulk, sensor-stopped Z2 nudge on stall.
        Z1 is governed only by the junction chain (may be left running)."""
        return self._cc.stage_next(**kwargs)

    def feed_tick(self) -> bool | None:
        """Advance the persistent Z1 watch (one nonblocking poll)."""
        return self._cc.feed_tick()

    def entry(self, *, o_occupied: bool) -> dict:
        """Staged enterer rides STAGING -> O, sensor-stopped. Entries ONLY
        run from a confirmed staged position — an unstaged part skips the
        beat and stays on Z1 (decision 2026-07-25). A live Z1 watch is
        inherited: the feed keeps hunting during the move and the driver
        cuts it at the follower's ONLOAD edge."""
        return self._record(self._cc.transition_move(
            self._pitch_mm, stop_on_work_zero=True, o_occupied=o_occupied))

    def retreat(self, *, o_occupied: bool) -> dict:
        """F2 part returns upstream to O — legacy return-to-zero (pass the eye
        HI->LO, then re-approach downstream until HI)."""
        return self._record(self._cc.transition_move(
            -self._pitch_mm, stop_on_work_zero=True,
            o_occupied=o_occupied, pass_through=True))

    def return_to_o(self, *, o_occupied: bool) -> dict:
        """F1 part (retreated trail) rides back down to O."""
        return self._record(self._cc.transition_move(
            self._pitch_mm, stop_on_work_zero=True, o_occupied=o_occupied))

    def blind(self, *, reduce_mm: float = 0.0) -> None:
        """Fill/drain shuffle with no O-arrival: one spacing, open loop.

        reduce_mm compensates belt motion that already happened this beat —
        a starved stage-probe's nudge slides every part by its measured
        travel, so the shuffle moves that much less and parts land back on
        their stations (model and belt agree again).

        If the persistent Z1 watch is live (a failed stage left the feed
        hunting), the feed is SUSPENDED for the shuffle — Z2 is about to move
        and a part crossing the junction would ride away — and resumed after;
        the watch itself survives untouched."""
        distance = (self.last_spacing_mm or self._pitch_mm) - reduce_mm
        if distance > 1.0:
            resume = self._cc.feed_watch_active
            if resume:
                self._cc.set_feed(False)
            self._cc.move_mm(distance)
            if resume:
                self._cc.set_feed(True)

    def idle(self) -> None:
        """Stop everything (fault/halt path): Z2 idled, Z1 cut, watch cancelled."""
        self._cc.feed_stop()
        self._cc.move_idle()

    def sensors(self) -> LegacyInputs:
        return self._cc.read_inputs()
