import numpy as np
import pytest
from _armkit.model import Joint, KinematicModel


def rev(name, parent, child, lower=-1.0, upper=1.0):
    return Joint(name, "revolute", parent, child, np.eye(4), np.array([0, 0, 1.0]), lower, upper)


def fixed(name, parent, child):
    return Joint(name, "fixed", parent, child, np.eye(4), None, None, None)


def test_actuated_joints_excludes_fixed():
    m = KinematicModel(
        name="t",
        joints=[
            rev("j1", "base", "l1"),
            fixed("f1", "l1", "l2"),
            rev("j2", "l2", "tip"),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    assert [j.name for j in m.actuated_joints] == ["j1", "j2"]
    assert m.dof == 2


def test_chain_orders_base_to_tip():
    m = KinematicModel(
        name="t",
        joints=[
            rev("j2", "l1", "tip"),
            rev("j1", "base", "l1"),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    assert [j.name for j in m.chain()] == ["j1", "j2"]
    assert m.base_link == "base"
    assert m.tip_link == "tip"


def test_chain_raises_on_branching():
    # Covers: `if len(branches) > 1` -> "branching at link ...".
    # Two joints share the same parent ("base"), so the walk from the
    # root finds two candidate next-steps at once.
    m = KinematicModel(
        name="t",
        joints=[
            rev("j1", "base", "l1"),
            rev("j2", "base", "l2"),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    with pytest.raises(ValueError, match="branching at link"):
        m.chain()


def test_chain_raises_on_multiple_roots():
    # Covers: `if len(set(roots)) > 1` -> "multiple root links: ...".
    # Two joints forming two separate chains: base->l1 and x->y. Neither
    # "base" nor "x" appears as any joint's child, so both register as
    # roots and this check fires before the walk ever starts. (This is
    # also the scenario a reader might expect to produce a distinct
    # "disconnected" message -- it doesn't; the multiple-roots check
    # fires first because both dangling parents are picked up as roots.)
    m = KinematicModel(
        name="t",
        joints=[
            rev("j1", "base", "l1"),
            rev("j2", "x", "y"),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    with pytest.raises(ValueError, match="multiple root links"):
        m.chain()


def test_chain_raises_on_disconnected_joints():
    # Covers: `if len(ordered) != len(self.joints)` -> "disconnected
    # joints in kinematic model". A root chain (base->l1) plus an
    # unrelated 2-cycle (x->y, y->x). The cycle's nodes are each some
    # other joint's child within the cycle itself, so neither "x" nor
    # "y" is ever picked up as a root; only "base" is. The walk from
    # "base" reaches "l1" and stops (nothing has parent "l1"), leaving
    # the two cycle joints unvisited, so the final count check fires.
    # This cycle is unreachable from the root, so it does NOT trip the
    # cycle guard in the walk -- that guard only fires for cycles the
    # walk actually steps into (see test_chain_raises_on_reachable_cycle).
    m = KinematicModel(
        name="t",
        joints=[
            rev("j1", "base", "l1"),
            rev("j2", "x", "y"),
            rev("j3", "y", "x"),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    with pytest.raises(ValueError, match="disconnected joints"):
        m.chain()


def test_chain_raises_on_no_root():
    # Covers: `if not roots` -> "kinematic model has no root link
    # (cycle?)". A pure 2-cycle (x->y, y->x) and nothing else: every
    # joint's parent is also some joint's child, so no link ever
    # qualifies as a root.
    m = KinematicModel(
        name="t",
        joints=[
            rev("j1", "x", "y"),
            rev("j2", "y", "x"),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    with pytest.raises(ValueError, match="no root link"):
        m.chain()


@pytest.mark.timeout(5)
def test_chain_raises_on_reachable_cycle():
    # Covers: `if current in seen` -> "cycle in kinematic model at link
    # ...". A root chain (base->l1) leading directly into a cycle
    # (l1->l2, l2->l1). Unlike test_chain_raises_on_disconnected_joints,
    # this cycle IS reachable from the root, so the walk steps into it
    # and would loop forever without the seen-set guard. Time-bounded
    # deliberately: a regression of the guard would hang rather than
    # fail, and this is the one test where that's actually possible.
    m = KinematicModel(
        name="t",
        joints=[
            rev("j1", "base", "l1"),
            rev("j2", "l1", "l2"),
            rev("j3", "l2", "l1"),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    with pytest.raises(ValueError, match="cycle in kinematic model"):
        m.chain()
