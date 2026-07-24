"""Gun cleaning — the HVLP tip on the rotating brush.

Despite the schedule historically calling this "denib", it is NOT a product
operation: on coat-2 beats the gun tip is cleaned against the spray-cleanoff
brush (legacy coil 108, O_BRUSH) so the second coat sprays clean. Translated
from the legacy brush routine (script:3122-3164):

  goto Clean_Brush -> contact-detect up into the brush -> settle -> BRUSH_ON
  -> hold duration_s -> BRUSH_OFF -> settle.

No force mode: the tip stays AT the contact point while the brush spins —
the legacy back-off standoff was removed 2026-07-26 (operator: tip must
touch the brush; the bristles supply the compliance). Uses the default (sanding) TCP, matching the legacy set_tcp
before Clean_Brush. Leaves the arm at the brush; safe_pose retracts it.

The brush hold blocks the executor for ~duration_s (per the RobotDevice
"every method blocks to completion" contract); the controller loop keeps
ticking and feeding the watchdog on its own thread. An operator halt lands
after the hold completes.

NEEDS-VALIDATION (real line): the standoff distance, the brush duration, and
that the tip actually cleans. The call order is locked in tests/test_gun_clean.py.
"""

from __future__ import annotations

import time

from ..config.loader import BrushConfig
from ..devices.clearcore import ClearCoreClient
from ..devices.ur import URClient


class GunClean:
    def __init__(self, ur: URClient, cc: ClearCoreClient, cfg: BrushConfig) -> None:
        self._ur = ur
        self._cc = cc
        self._cfg = cfg

    def clean(self) -> None:
        """Clean the HVLP tip on the brush. Blocks ~duration_s; brush off by
        return whatever happens.
        """
        cfg, ur, cc = self._cfg, self._ur, self._cc
        ur.use_default_tcp()  # brush uses the sand/default frame (script:3123)
        ur.move_to_named("Clean_Brush")
        ur.contact_detect_z(speed_ms=cfg.contact_v)  # up into the brush
        # No back-off (2026-07-26): the tip stays on the contact point;
        # bristle compliance does the work. cfg.retract_off_mm is unused.
        time.sleep(cfg.settle_before_on_s)
        cc.set_brush(True)
        try:
            time.sleep(cfg.duration_s)  # tip held against the spinning brush
        finally:
            cc.set_brush(False)  # never leave the brush running, even on fault
        time.sleep(cfg.settle_after_off_s)
