"""Sanding — the robot/conveyor duet.

WHY THIS LAYER EXISTS
---------------------
CLAUDE.md says the UR gets "a thin library of parameterized URScript motion
primitives (sand_faces, ...)" and that devices are dumb executors. `sand_faces`
cannot honour that, because sanding is not a robot operation. It is a two-axis
raster in which each device owns one axis:

    robot   force-holds Z at 6 N (tuned) and traverses base X by `height`
    conveyor traverses the belt axis by `width - 12`

From the old program (script:2460-2497), one face is:

    contact-detect Z+ -> zero FT -> force_mode(Z, 6 N)
      conveyor +350 mm   (width - inset)   \\
      wait SERVER_STATE == 1                 | belt axis
      robot   -355 mm in base X            /  robot axis
      conveyor -350 mm
      wait SERVER_STATE == 1
      robot   +355 mm in base X
    end_force_mode -> stopl(5.0)

Neither device can do that alone, and the PC now owns the conveyor. So the
composite lives here: above the drivers, below the core. The core emits
`SandPart(part_id)` and learns only that it finished.

This keeps "no I/O in the core" intact without pretending the robot is
self-sufficient. It is the one place in the system where two protocols are
interleaved inside a single force-controlled operation, and the latency budget
is the old ClearCore handshake's: SERVER_STATE polled at 50 Hz.
"""

from __future__ import annotations

from ..config.loader import ProductSpec, SandConfig
from ..core.model import Zone
from ..devices.clearcore import ClearCoreClient
from ..devices.ur import URClient


class Sander:
    """Executes `SandPart` intents by interleaving UR and ClearCore calls.

    All motion constants come from cell-config.yaml via `SandConfig` — they are
    tuned on the real line and must not be re-derived here.
    """

    def __init__(self, ur: URClient, cc: ClearCoreClient, cfg: SandConfig) -> None:
        self._ur = ur
        self._cc = cc
        self._cfg = cfg

    def sand_face(self, product: ProductSpec) -> None:
        """Raster one face of the part currently at O.

        Preserves the old program's order exactly (script:2460-2497): approach,
        contact-detect Z, zero the FT sensor, tool on, force-hold Z, run the
        duet, then end force mode and stop the tool.

        NEEDS-VALIDATION (real line): force-mode feel and the FT zero settle
        only behave against a physical part — URSim has no physics. This method
        is structurally verified by tests/test_sander.py (call order) but its
        tuning is a maintenance-window item.
        """
        cfg = self._cfg
        self._ur.move_to_named("Sand_Base")
        self._ur.contact_detect_z(speed_ms=cfg.movel_v)
        self._ur.zero_ft(pre_wait_s=cfg.ft_wait_steady_ms / 1000.0)
        self._ur.set_tool(True)  # DO3 — sander on
        self._ur.begin_force_z(newtons=cfg.z_force_n)
        try:
            self._traverse(product)
        finally:
            # Tool off AFTER force mode ends (script:2493-2497): reversing the
            # order drags a dead sander across the finished face. In a finally
            # so a mid-traverse fault still lifts the force and kills the tool.
            self._ur.end_force_mode(decel=cfg.stopl_on_force_end)
            self._ur.set_tool(False)

    def _traverse(self, product: ProductSpec) -> None:
        """The duet itself: belt axis, robot axis, belt axis, robot axis.

        `pass_mm` is `width - inset`, the tuned pass length that keeps the tool
        from running off the end of the part. `step_mm` is the full part height;
        the robot covers it in one base-X move at movel_v (~7 s at 50 mm/s). Both
        the belt (+pass then -pass) and the robot (-step then +step) return to
        their start, so a part riding along on Z2 (the flash part at F2) ends
        where it began.

        The belt runs on ZONE 2 — the O station sits on Z2 (O<->F2<->OUT). The
        wait_zone_ready between each belt move and the following robot move is
        the SERVER_STATE handshake; without it the robot steps the height axis
        while the part is still travelling.
        """
        cfg = self._cfg
        pass_mm = float(product.width_mm - cfg.width_inset_mm)
        step_mm = float(product.height_mm)

        self._cc.move_zone_mm(Zone.Z2, pass_mm)
        self._cc.wait_zone_ready(Zone.Z2)  # SERVER_STATE == 1
        self._ur.move_base_x_mm(-step_mm, a=cfg.movel_a, v=cfg.movel_v)
        self._cc.move_zone_mm(Zone.Z2, -pass_mm)
        self._cc.wait_zone_ready(Zone.Z2)
        self._ur.move_base_x_mm(step_mm, a=cfg.movel_a, v=cfg.movel_v)
