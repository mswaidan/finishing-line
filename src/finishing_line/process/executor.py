"""Intent executor — turns core intents into device calls.

The core emits intents and never blocks. The executor runs them, and reports
completion by intent id on a later step. Nothing here decides *what* to do; it
only carries out what the core already decided.

TODO(step 3/4): implement against hardware in maintenance windows.
"""

from __future__ import annotations

from collections.abc import Iterable

from ..core.intents import Intent


class Executor:
    """Runs intents against real devices.

    Contract with the core:
      - `submit()` accepts intents and returns immediately.
      - `completed()` reports ids of intents that have finished since last call.
      - An intent that fails raises to the supervisor, which faults the machine
        rather than retrying — a failed spray or a failed zone move both mean
        the line's physical state no longer matches the controller's belief.

    Completion means *confirmed*, never *commanded*. `SetShutter` completes when
    the feedback sensor agrees, because every zone motion is gated on shutter
    position (§7) and a commanded-but-stuck shutter would open that gate on a
    lie.
    """

    def submit(self, intents: Iterable[Intent]) -> None:
        raise NotImplementedError

    def completed(self) -> frozenset[str]:
        raise NotImplementedError
