"""Pairing policies. The cube/browser question is open (§8) — these lock in the
behaviour of each answer so switching is a config change, not a rewrite.
"""

from __future__ import annotations

from dataclasses import replace

from finishing_line.core.model import PartRole, Product
from finishing_line.core.pairing import (
    AnyProduct,
    BrowsersInDedicatedBlocks,
    SameProduct,
    assign_roles,
    from_name,
)

from .conftest import make_part


def _queue(*products: Product):
    return [
        replace(make_part(f"p{i}", PartRole.LEAD), product=p) for i, p in enumerate(products)
    ]


def test_roles_alternate_lead_then_trail():
    roles = [p.role for p in assign_roles(_queue(*[Product.CUBE] * 4))]
    assert roles == [PartRole.LEAD, PartRole.TRAIL, PartRole.LEAD, PartRole.TRAIL]


def test_roles_carry_pair_index():
    indices = [p.pair_index for p in assign_roles(_queue(*[Product.CUBE] * 4))]
    assert indices == [0, 0, 1, 1]


def test_same_product_pairs_matching_parts():
    pairs = SameProduct().pair(_queue(Product.CUBE, Product.CUBE))
    assert len(pairs) == 1
    lead, trail = pairs[0]
    assert trail is not None and lead.product is trail.product


def test_same_product_refuses_to_mix():
    """A product change starts a new pair; the cube runs as a lone lead."""
    pairs = SameProduct().pair(_queue(Product.CUBE, Product.BROWSER, Product.BROWSER))
    assert pairs[0][1] is None, "cube should not be paired with a browser"
    assert pairs[1][1] is not None, "the two browsers should pair"


def test_odd_count_leaves_a_lone_lead():
    """§5: a lone lead runs the lead path with S idle on trail beats."""
    pairs = SameProduct().pair(_queue(*[Product.CUBE] * 3))
    assert pairs[-1][1] is None


def test_any_product_will_mix():
    pairs = AnyProduct().pair(_queue(Product.CUBE, Product.BROWSER))
    assert pairs[0][1] is not None


def test_dedicated_blocks_groups_by_product():
    queue = _queue(Product.CUBE, Product.BROWSER, Product.CUBE, Product.BROWSER)
    pairs = BrowsersInDedicatedBlocks().pair(queue)
    assert all(
        trail is None or lead.product is trail.product for lead, trail in pairs
    ), "blocks must never mix"
    assert len(pairs) == 2


def test_policy_resolves_by_config_name():
    assert isinstance(from_name("same_product"), SameProduct)
    assert isinstance(from_name("any"), AnyProduct)


def test_unknown_policy_is_rejected_loudly():
    import pytest

    with pytest.raises(ValueError, match="unknown pairing policy"):
        from_name("whatever")
