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

        TODO(step 4): implement against hardware in a maintenance window.

        Sequence, preserving the old program's order exactly:
          1. self._ur.move_to_named("Sand_Base")
          2. self._ur.contact_detect_z(distance=cfg.contact_search_distance_m)
          3. self._ur.zero_ft(wait_steady_ms=cfg.ft_wait_steady_ms)
          4. self._ur.set_tool(True)                     # DO3
          5. self._ur.begin_force_z(newtons=cfg.z_force_n)
          6. self._traverse(product)
          7. self._ur.end_force_mode(stopl=cfg.stopl_on_force_end)
          8. self._ur.set_tool(False)

        The tool is stopped AFTER force mode ends, matching script:2493-2497 —
        reversing that order drags a dead sander across a finished face.
        """
        raise NotImplementedError

    def _traverse(self, product: ProductSpec) -> None:
        """The duet itself: belt axis, robot axis, belt axis, robot axis.

        TODO(step 4).

        `pass_mm` is `width - inset`, the tuned pass length that keeps the tool
        from running off the end of the part. `step_mm` is the full part height;
        the robot covers it in one 355 mm base-X move at 50 mm/s (~7 s).

            pass_mm = product.width_mm - self._cfg.width_inset_mm
            step_mm = product.height_mm

            self._cc.move_distance_mm(pass_mm)
            self._cc.wait_ready()                       # SERVER_STATE == 1
            self._ur.move_base_x_mm(-step_mm, a=..., v=...)
            self._cc.move_distance_mm(-pass_mm)
            self._cc.wait_ready()
            self._ur.move_base_x_mm(+step_mm, a=..., v=...)

        `wait_ready()` between the belt move and the robot move is not optional:
        it is the SERVER_STATE handshake, and without it the robot steps the
        height axis while the part is still travelling.
        """
        raise NotImplementedError
