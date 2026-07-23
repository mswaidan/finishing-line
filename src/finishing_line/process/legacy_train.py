"""LegacyTrain — the validated choreography verbs over the legacy driver.

Thin by design: every method is one of the maneuvers proven on the real line
(2026-07-22, docs/legacy-mod-choreography.md), expressed once so the sequencer
reads like the choreography it runs. All belt motion is CAPPED distance moves
(crash-safe: a dead PC leaves at most the cap, then the firmware stops).

Nominal distances are runaway caps, not targets — every arrival is
sensor-stopped. Defaults reflect the measured geometry (eye->WZ ~700 mm,
queue->WZ ~1115 mm); they only matter as caps and for the blind fill/drain
shuffles.
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

    def _record(self, res: dict) -> dict:
        if res.get("arrived"):
            mmps = self._cc.kinematics.velocity_steps_per_sec / self._cc.kinematics.microsteps_per_mm
            self.last_spacing_mm = res["seconds"] * mmps
        return res

    # ------------------------------------------------------------ maneuvers

    def load(self) -> dict:
        """First part: queue -> O in one run. Feed boards it (cut at first
        ONLOAD rising), belt stops on the WORK_AT_ZERO arrival."""
        return self._record(self._cc.transition_move(
            self._load_mm, stop_on_work_zero=True, feed=True))

    def stage(self) -> dict:
        """JIT staging: feed-only bulk, sensor-stopped belt nudge on stall."""
        return self._cc.stage_next()

    def entry(self, *, o_occupied: bool, feed_assist: bool = False) -> dict:
        """Staged enterer rides eye -> O. feed_assist only if staging failed
        (boarding cut at first ONLOAD rising)."""
        return self._record(self._cc.transition_move(
            self._pitch_mm, stop_on_work_zero=True,
            o_occupied=o_occupied, feed=feed_assist))

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

    def blind(self) -> None:
        """Fill/drain shuffle with no O-arrival: one spacing, open loop."""
        self._cc.move_mm(self.last_spacing_mm or self._pitch_mm)

    def idle(self) -> None:
        self._cc.move_idle()

    def sensors(self) -> LegacyInputs:
        return self._cc.read_inputs()
