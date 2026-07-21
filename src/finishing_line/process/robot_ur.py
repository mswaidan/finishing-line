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

`sand` is implemented; `denib` and `spray` are the next composites (the Sprayer
choreography and the denib open item, §8) and raise until then. A full schedule
run under `--ur` needs them — but the conveyor+sand plumbing is exercisable now.
"""

from __future__ import annotations

from collections.abc import Callable

from ..config.loader import ProductSpec
from ..devices.ur import URClient
from .sander import Sander


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
        resolve_product: Callable[[str], ProductSpec],
    ) -> None:
        self._ur = ur
        self._sander = sander
        self._resolve_product = resolve_product
        self._clear = True
        self._gun = False

    # ------------------------------------------------------------ operations

    def sand(self, part_id: str) -> None:
        """Sand the face of the part at O. Blocks until done, tool stopped."""
        self._clear = False
        self._sander.sand_face(self._resolve_product(part_id))

    def denib(self, part_id: str) -> None:
        """OPEN ITEM (§8): the denib pass is unconfirmed (existence + duration).
        Implement alongside the coat-2 choreography once decided.
        """
        raise NotImplementedError("denib pass not yet defined (§8 open item)")

    def spray(self, part_id: str, coat: int) -> None:
        """Next composite — process/sprayer.py (the 4-JOB spray choreography).
        When it lands it must hold `self._gun` True across the live-gun window so
        `gun_on()` gates zone motion honestly (§7).
        """
        raise NotImplementedError("spray composite not yet written")

    def safe_pose(self) -> None:
        """Retract to the clear waypoint; ROBOT_CLEAR truth follows."""
        self._ur.move_to_named(self.CLEAR_WAYPOINT)
        self._clear = True

    # ----------------------------------------------------------------- facts

    def is_clear(self) -> bool:
        return self._clear

    def gun_on(self) -> bool:
        return self._gun
