"""FakeRobot — RobotDevice that sleeps instead of moving.

Used by the Stage B harness so the full orchestrator stack (machine, executor,
ClearCore driver, real Modbus) runs without a robot. Durations are compressed;
the *state truth* is exact: `is_clear` goes false the moment work starts,
`gun_on` is true precisely for the spray window — because those two facts feed
the §7 interlocks and the harness must exercise them honestly.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class FakeRobot:
    work_s: float = 0.3
    spray_s: float = 0.3
    retract_s: float = 0.05

    #: Log of (operation, part_id) in execution order, for test assertions.
    log: list[tuple[str, str]] = field(default_factory=list)

    _clear: bool = True
    _gun: bool = False

    def sand(self, part_id: str) -> None:
        self._clear = False
        self.log.append(("sand", part_id))
        time.sleep(self.work_s)

    def clean_gun(self, part_id: str) -> None:
        self._clear = False
        self.log.append(("clean_gun", part_id))
        time.sleep(self.work_s)

    def spray(self, part_id: str, coat: int) -> None:
        self._clear = False
        self.log.append((f"spray{coat}", part_id))
        self._gun = True
        try:
            time.sleep(self.spray_s)
        finally:
            self._gun = False

    def safe_pose(self) -> None:
        self.log.append(("safe_pose", ""))
        time.sleep(self.retract_s)
        self._clear = True

    def is_clear(self) -> bool:
        return self._clear

    def gun_on(self) -> bool:
        return self._gun
