"""RobotDevice — the executor's view of the robot.

A protocol rather than a class so the same Executor runs against three
implementations without knowing which it has:

- `sim.fake_robot.FakeRobot` — Stage A/B harness; sleeps instead of moving.
- `process.robot_ur.URRobot` — the real implementation: URClient motion plus
  the Sander composite. `sand`/`safe_pose`/`is_clear`/`gun_on` are done;
  `spray`/`denib` are the next composites (Sprayer + the §8 denib item) and
  raise until written. Force-mode feel is validated only against a physical
  part (URSim has no physics); the call-order contract is locked in
  tests/test_sander.py.

Every method BLOCKS until the operation is physically complete — the Executor
provides the threading, devices provide the truth. `is_clear`/`gun_on` are the
two robot facts the interlocks consume (§7); they must reflect reality, not
intent, because zone motion and spray permission gate on them.
"""

from __future__ import annotations

from typing import Protocol


class RobotDevice(Protocol):
    def sand(self, part_id: str) -> None:
        """Sand the face of the part at O. Blocks until done, tool stopped."""
        ...

    def denib(self, part_id: str) -> None:
        """Denib pass before coat 2. Blocks until done."""
        ...

    def spray(self, part_id: str, coat: int) -> None:
        """Apply one coat. Gun must be off again by return."""
        ...

    def safe_pose(self) -> None:
        """Retract clear of the transfer envelope. ROBOT_CLEAR truth follows."""
        ...

    def is_clear(self) -> bool:
        """Robot is out of the transfer envelope — gates zone motion (§7)."""
        ...

    def gun_on(self) -> bool:
        """Gun is live right now — gates zone motion (§7)."""
        ...
