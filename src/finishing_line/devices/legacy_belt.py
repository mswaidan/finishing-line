"""LegacyBeltAdapter — lets the robot composites drive the legacy single belt.

The Sander/Sprayer/GunClean composites were written against the rewrite's
ClearCoreClient zone API (`move_zone_mm(Zone.Z2, ...)` / `wait_zone_ready` /
`set_brush`). On the legacy-mod route there is one belt and it IS the belt the
composites' raster moves were extracted from (`Conveyor_Move_mm`, script), so
the adapter is a name-for-name shim: every zone argument maps to THE conveyor.

Non-blocking start + separate wait is preserved deliberately — the cube spray
overlaps the belt return with a movej (script:3092), and serializing it would
add seconds per coat.
"""

from __future__ import annotations

from ..core.model import Zone
from .legacy_clearcore import LegacyClearCoreClient


class LegacyBeltAdapter:
    def __init__(self, cc: LegacyClearCoreClient) -> None:
        self._cc = cc

    def move_zone_mm(self, zone: Zone, distance_mm: float, **_kw) -> int:
        """Start a belt move (returns once accepted, like the zone API)."""
        return self._cc.start_move_mm(distance_mm)

    def wait_zone_ready(self, zone: Zone, timeout_s: float = 60.0) -> None:
        self._cc.wait_ready(timeout_s=timeout_s)

    def set_brush(self, on: bool) -> None:
        self._cc.set_brush(on)
