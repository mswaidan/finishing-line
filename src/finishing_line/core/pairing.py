"""Pairing policy — how parts from the infeed queue become (lead, trail) pairs.

Pluggable because whether browsers fit the same schedule is an OPEN ITEM (§8).
Isolating the decision here means answering it later changes one function, not
the scheduler.

The schedule itself is product-agnostic: it moves a train of parts and never
reads a product. Only pairing cares.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

from .model import PartRole, PartState, Product


class PairingPolicy(Protocol):
    def pair(self, queue: Sequence[PartState]) -> list[tuple[PartState, PartState | None]]:
        """Group a queue into ordered pairs. A trailing `None` means a lone lead,
        which runs the odd-count drain path of §5.
        """
        ...


def assign_roles(queue: Sequence[PartState]) -> list[PartState]:
    """Stamp alternating LEAD/TRAIL roles over a queue, in order.

    Role is positional, not a property of the part: the first of a pair leads and
    flashes both coats at FD, the second trails and retreats to IF for flash 1.
    """
    from dataclasses import replace

    return [
        replace(part, role=PartRole.LEAD if i % 2 == 0 else PartRole.TRAIL, pair_index=i // 2)
        for i, part in enumerate(queue)
    ]


class SameProduct:
    """Default: a pair must be two of the same product.

    Conservative — it never assumes a cube and a browser share a beat budget,
    which is exactly the unanswered question. A product change simply starts a
    new pair; the odd part out runs the §5 lone-lead path.
    """

    def pair(self, queue: Sequence[PartState]) -> list[tuple[PartState, PartState | None]]:
        pairs: list[tuple[PartState, PartState | None]] = []
        i = 0
        while i < len(queue):
            lead = queue[i]
            trail = queue[i + 1] if i + 1 < len(queue) else None
            if trail is not None and trail.product is not lead.product:
                trail = None
            pairs.append((lead, trail))
            i += 1 if trail is None else 2
        return pairs


class AnyProduct:
    """Pair whatever is next, regardless of product.

    Only correct once browsers are confirmed to fit the same beat. Until then it
    will happily schedule a mixed pair that may not.
    """

    def pair(self, queue: Sequence[PartState]) -> list[tuple[PartState, PartState | None]]:
        return [
            (queue[i], queue[i + 1] if i + 1 < len(queue) else None)
            for i in range(0, len(queue), 2)
        ]


class BrowsersInDedicatedBlocks:
    """Never mix; additionally refuse to interleave browser and cube pairs.

    The fallback if browsers turn out to need their own geometry or timing —
    run them as a block, at the cost of a changeover.
    """

    def pair(self, queue: Sequence[PartState]) -> list[tuple[PartState, PartState | None]]:
        cubes = [p for p in queue if p.product is Product.CUBE]
        browsers = [p for p in queue if p.product is Product.BROWSER]
        return SameProduct().pair(cubes) + SameProduct().pair(browsers)


POLICIES: dict[str, type] = {
    "same_product": SameProduct,
    "any": AnyProduct,
    "browsers_in_dedicated_blocks": BrowsersInDedicatedBlocks,
}


def from_name(name: str) -> PairingPolicy:
    """Resolve `pairing.policy` from line-config.yaml."""
    try:
        return POLICIES[name]()
    except KeyError:
        raise ValueError(f"unknown pairing policy {name!r}; expected one of {sorted(POLICIES)}")
