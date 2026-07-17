from __future__ import annotations

import pytest

from finishing_line.config.loader import ProcessConfig
from finishing_line.core.model import (
    LineState,
    PartRole,
    PartState,
    Product,
    Station,
)


@pytest.fixture
def cfg() -> ProcessConfig:
    """Nominal config with the assumed values from line-config.yaml."""
    return ProcessConfig(
        flash_seconds=180.0,
        coats=2,
        spray_burst_pause_s=30.0,
        transfer_s=15.0,
        robot_coat1_s=90.0,
        robot_coat2_s=45.0,
        denib_enabled=True,
        denib_duration_s=20.0,
        provenance={"flash_seconds": "assumed", "spray_burst_pause_s": "assumed"},
    )


def make_part(part_id: str, role: PartRole, **kw) -> PartState:
    return PartState(
        part_id=part_id,
        product=Product.CUBE,
        role=role,
        pair_index=kw.pop("pair_index", 0),
        **kw,
    )


@pytest.fixture
def empty_line() -> LineState:
    return LineState()


def staged(*part_ids: str) -> LineState:
    """A line with parts queued at INQ and nothing on the belt — the §4 startup
    condition.
    """
    parts = {}
    for i, pid in enumerate(part_ids):
        role = PartRole.LEAD if i % 2 == 0 else PartRole.TRAIL
        parts[pid] = make_part(pid, role, pair_index=i // 2)
    return LineState(parts=parts, inq_queue=tuple(part_ids), occupancy={})


def place(state: LineState, station: Station, part_id: str) -> LineState:
    from dataclasses import replace

    return replace(state, occupancy={**state.occupancy, station: part_id})
