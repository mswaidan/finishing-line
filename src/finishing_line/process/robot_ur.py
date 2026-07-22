"""URRobot — the real RobotDevice: URClient motion + the Sander composite.

Implements the process/robot.py Protocol against a live UR5e (via ur_rtde), so
the Executor runs the exact same intents it runs against FakeRobot. Every method
BLOCKS until the operation is physically complete — the Executor owns threading.

`is_clear` / `gun_on` are operation-boundary truth. Because each method blocks to
completion, a flag flipped at the boundaries is accurate exactly when the §7
interlocks read it: zone motion is only commanded after `safe_pose` has returned,
and spray permission is only checked while the gun window is open. (Simple bool
reads/writes are atomic under the GIL; the Executor writes them, the supervisor
tick reads them — same as FakeRobot, no lock needed.)

`sand`, `spray`, and `clean_gun` (the gun-tip brush clean) are all implemented —
URRobot fully realizes RobotDevice, so `--ur` can run the whole interleaved
schedule against real hardware. Force-mode and spray *feel* is still maintenance-
window territory (URSim has no physics); the call-order contracts are locked in
tests/test_sander.py, test_sprayer.py, and test_gun_clean.py.
"""

from __future__ import annotations

from collections.abc import Callable

from ..config.loader import ProductSpec
from ..devices.ur import URClient
from .gun_clean import GunClean
from .sander import Sander
from .sprayer import Sprayer


class URRobot:
    #: Waypoint the arm parks at to assert ROBOT_CLEAR (clear of the transfer
    #: envelope so belts can move parts beneath it). Sand_Base is the highest
    #: defined pose and the sander's own staging pose; a dedicated clear pose is
    #: an open item (cell-config defines no home/safe waypoint).
    CLEAR_WAYPOINT = "Sand_Base"

    def __init__(
        self,
        ur: URClient,
        sander: Sander,
        sprayer: Sprayer,
        gun_clean: GunClean,
        resolve_product: Callable[[str], ProductSpec],
    ) -> None:
        self._ur = ur
        self._sander = sander
        self._sprayer = sprayer
        self._gun_clean = gun_clean
        self._resolve_product = resolve_product
        self._clear = True
        self._gun = False

    # ------------------------------------------------------------ operations

    def sand(self, part_id: str) -> None:
        """Sand the face of the part at O. Blocks until done, tool stopped."""
        self._clear = False
        self._sander.sand_face(self._resolve_product(part_id))

    def clean_gun(self, part_id: str) -> None:
        """Clean the HVLP tip on the brush before a coat (see GunClean). Blocks
        until done. `part_id` is unused — the clean is per-beat, not per-part.
        """
        self._clear = False
        self._gun_clean.clean()

    def spray(self, part_id: str, coat: int) -> None:
        """Apply one coat to the part at O. Blocks until done, gun off by return.

        `_gun` is held True across the whole window so `gun_on()` gates zone
        motion honestly (§7), even though the Sprayer toggles DO5 per stroke.
        """
        self._clear = False
        self._gun = True
        try:
            self._sprayer.spray(self._resolve_product(part_id), coat)
        finally:
            self._gun = False

    def safe_pose(self) -> None:
        """Retract to the clear waypoint; ROBOT_CLEAR truth follows."""
        self._ur.move_to_named(self.CLEAR_WAYPOINT)
        self._clear = True

    # ----------------------------------------------------------------- facts

    def is_clear(self) -> bool:
        return self._clear

    def gun_on(self) -> bool:
        return self._gun
