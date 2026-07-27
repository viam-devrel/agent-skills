import numpy as np
import pytest
from _armkit.model import Joint, KinematicModel


def test_actuated_joints_excludes_fixed():
    m = KinematicModel(
        name="t",
        joints=[
            Joint("j1", "revolute", "base", "l1", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
            Joint("f1", "fixed", "l1", "l2", np.eye(4), None, None, None),
            Joint("j2", "revolute", "l2", "tip", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    assert [j.name for j in m.actuated_joints] == ["j1", "j2"]
    assert m.dof == 2


def test_chain_orders_base_to_tip():
    m = KinematicModel(
        name="t",
        joints=[
            Joint("j2", "revolute", "l1", "tip", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
            Joint("j1", "revolute", "base", "l1", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
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
            Joint("j1", "revolute", "base", "l1", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
            Joint("j2", "revolute", "base", "l2", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
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
            Joint("j1", "revolute", "base", "l1", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
            Joint("j2", "revolute", "x", "y", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
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
    m = KinematicModel(
        name="t",
        joints=[
            Joint("j1", "revolute", "base", "l1", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
            Joint("j2", "revolute", "x", "y", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
            Joint("j3", "revolute", "y", "x", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
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
            Joint("j1", "revolute", "x", "y", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
            Joint("j2", "revolute", "y", "x", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    with pytest.raises(ValueError, match="no root link"):
        m.chain()
