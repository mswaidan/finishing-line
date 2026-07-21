"""Spraying — the robot/conveyor duet for lacquer coats.

Like the Sander this interleaves UR motion with Z2 belt moves (the O station is
on Z2), but it is NON-CONTACT: no force mode, the gun (DO5) toggles instead. Two
product routines, translated verbatim from the legacy program:

  cube (JOB 1, script:3060-3111)    — horizontal. Gun toggled per stroke across
      three waypoints: a belt sweep at Waypoint_1, then two height sweeps at
      Waypoint_2 / Waypoint_3.
  browser (JOB 2, script:2856-2941) — vertical. A base-Z standoff, then the gun
      held ON through one continuous belt/height raster back to Spray_Base.

The other JOBs (45, sc3/sc4, job7) are legacy stereocab products outside this
line's cube+browser scope (CLAUDE.md) and raise.

Both use the spray TCP (RobotSetup.spray_tcp), restored to the sanding TCP on the
way out so the next movej to a sand-frame waypoint (e.g. safe_pose -> Sand_Base)
solves correctly.

`coat` is not a motion parameter: the legacy runs the identical spray routine for
coat 1 and coat 2 (the `dried2` state gated only load/unload, not the path).

NEEDS-VALIDATION (real line): spray quality, gun-on timing, and the standoff only
prove out against a physical part — URSim has no physics. The call/gun order is
locked in tests/test_sprayer.py.
"""

from __future__ import annotations

from ..config.loader import ProductSpec, SprayConfig
from ..core.model import Zone
from ..devices.clearcore import ClearCoreClient
from ..devices.ur import URClient

_CUBE_JOB = 1
_BROWSER_JOB = 2


class Sprayer:
    def __init__(self, ur: URClient, cc: ClearCoreClient, cfg: SprayConfig) -> None:
        self._ur = ur
        self._cc = cc
        self._cfg = cfg

    def spray(self, product: ProductSpec, coat: int) -> None:
        """Apply one coat to the part at O. Blocks until done, gun off, TCP
        restored. `coat` does not vary the motion (see module docstring).
        """
        self._ur.use_spray_tcp()
        try:
            if product.legacy_job_id == _CUBE_JOB:
                self._spray_cube(product)
            elif product.legacy_job_id == _BROWSER_JOB:
                self._spray_vertical(product)
            else:
                raise NotImplementedError(
                    f"no spray routine for {product.name} (JOB "
                    f"{product.legacy_job_id}); this line runs cube + browser"
                )
        finally:
            # Gun off + sand-frame TCP restored on ANY exit — a fault mid-spray
            # must never leave the gun live or the next movej solving under the
            # spray frame.
            self._ur.set_sprayer(False)
            self._ur.use_default_tcp()

    def _pass_mm(self, product: ProductSpec) -> float:
        return float(product.width_mm - self._cfg.width_inset_mm)

    def _spray_cube(self, product: ProductSpec) -> None:
        """JOB 1 horizontal — gun toggled per stroke (script:3060-3111)."""
        cfg, ur, cc = self._cfg, self._ur, self._cc
        pass_mm, step_mm = self._pass_mm(product), float(product.height_mm)

        # Stroke 1 — Waypoint_1, belt sweep +pass.
        ur.move_to_named("Waypoint_1")
        ur.set_sprayer(True)
        cc.move_zone_mm(Zone.Z2, pass_mm)
        cc.wait_zone_ready(Zone.Z2)
        ur.set_sprayer(False)
        # Stroke 2 — Waypoint_2, height sweep -step.
        ur.move_to_named("Waypoint_2")
        ur.set_sprayer(True)
        ur.move_base_x_mm(-step_mm, a=cfg.height_a, v=cfg.height_v)
        ur.set_sprayer(False)
        # Belt returns -pass, overlapping the movej to Waypoint_3 (script:2924):
        # command the belt (returns on ack), move the arm, then join the belt.
        cc.move_zone_mm(Zone.Z2, -pass_mm)
        ur.move_to_named("Waypoint_3")
        cc.wait_zone_ready(Zone.Z2)
        # Stroke 3 — height sweep +step.
        ur.set_sprayer(True)
        ur.move_base_x_mm(step_mm, a=cfg.height_a, v=cfg.height_v)
        ur.set_sprayer(False)

    def _spray_vertical(self, product: ProductSpec) -> None:
        """JOB 2 vertical — standoff, then gun ON through one raster (script:2856-2941)."""
        cfg, ur, cc = self._cfg, self._ur, self._cc
        pass_mm, step_mm = self._pass_mm(product), float(product.height_mm)

        ur.move_to_named("Spray_Base")
        ur.move_base_z_mm(cfg.approach_z_m * 1000.0, a=cfg.approach_a, v=cfg.approach_v)
        ur.set_sprayer(True)  # gun ON for the whole raster
        cc.move_zone_mm(Zone.Z2, pass_mm)
        cc.wait_zone_ready(Zone.Z2)
        ur.move_base_x_mm(-step_mm, a=cfg.height_a, v=cfg.height_v)
        cc.move_zone_mm(Zone.Z2, -pass_mm)
        cc.wait_zone_ready(Zone.Z2)
        ur.move_base_x_mm(step_mm, a=cfg.height_a, v=cfg.height_v)
        ur.set_sprayer(False)
        ur.move_to_named("Spray_Base")
