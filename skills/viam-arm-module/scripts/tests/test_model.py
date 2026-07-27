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


def test_chain_raises_on_multiple_leaves_without_declared_tip():
    # CHANGED (Task A3b, Part 2): this scenario used to raise "branching
    # at link 'base': [...]" because chain() diagnosed branching at the
    # first fork it walked into. RDK instead reasons about leaves --
    # verified against RDK v1.0.0 via the probe on a synthetic two-leaf
    # URDF: "need exactly one end effector, have [finger_r finger_l]".
    # Branching itself is legal now (see test_chain_allows_branching_
    # with_declared_tip); it is only an error when there's no declared
    # tip to disambiguate which leaf is the end effector.
    m = KinematicModel(
        name="t",
        joints=[
            rev("j1", "base", "l1"),
            rev("j2", "base", "l2"),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    with pytest.raises(ValueError, match=r"need exactly one end effector, have \['l1', 'l2'\]"):
        m.chain()


def test_chain_allows_branching_with_declared_tip():
    # Part 2/3: once a tip is declared, branching elsewhere in the tree
    # is legal -- chain() just walks root->tip and ignores the sibling
    # branch. dof still counts the sibling branch's joints (Part 4,
    # tested in test_dof_counts_branch_joints_not_on_chain).
    m = KinematicModel(
        name="t",
        joints=[
            rev("j1", "base", "l1"),
            rev("j2", "base", "l2"),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
        primary_output_frame="l1",
    )
    assert [j.name for j in m.chain()] == ["j1"]
    assert m.tip_link == "l1"


def test_dof_counts_branch_joints_not_on_chain():
    # Part 4: RDK builds its flat input vector via BFS over the WHOLE
    # frame system, not along the tip path -- sibling-branch joints
    # still occupy input slots. Verified against RDK v1.0.0 via the
    # probe on an equivalent branching SVA fixture with output_frames:
    # DoF=3 for a trunk joint plus one joint on each of two branches,
    # even though only 2 of those 3 joints are on the declared tip's
    # chain.
    m = KinematicModel(
        name="t",
        joints=[
            rev("trunk", "root", "base"),
            rev("j1", "base", "l1"),
            rev("j2", "base", "l2"),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
        primary_output_frame="l1",
    )
    assert [j.name for j in m.chain()] == ["trunk", "j1"]
    assert m.dof == 3
    assert {j.name for j in m.actuated_joints} == {"trunk", "j1", "j2"}


def test_declared_tip_not_reachable_raises():
    m = KinematicModel(
        name="t",
        joints=[rev("j1", "base", "l1")],
        links={}, source_format="urdf", source_path="t.urdf",
        primary_output_frame="ghost",
    )
    with pytest.raises(ValueError, match="declared tip 'ghost' is not reachable"):
        m.chain()


def test_bfs_order_matches_chain_order_for_serial_models():
    # Part 4's no-op guarantee: for any serial (non-branching) model,
    # BFS-over-the-whole-tree order must equal the root->tip chain
    # order, since there's only ever one joint to step to at each link.
    m = KinematicModel(
        name="t",
        joints=[
            rev("j2", "l1", "tip"),
            rev("j1", "base", "l1"),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    assert [j.name for j in m.actuated_joints] == [j.name for j in m.chain()]


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


def test_chain_raises_on_no_joints():
    # Covers: `if not self.joints` -> "kinematic model has no joints".
    # A model with zero joints (e.g. a single-link URDF like RDK's
    # capsule.urdf) previously fell through to `roots = []`, which
    # tripped the "no root link (cycle?)" check and misdiagnosed a
    # jointless model as a cyclic one. This must be caught first, with
    # a message describing the actual condition.
    m = KinematicModel(
        name="t", joints=[], links={}, source_format="urdf", source_path="t.urdf",
    )
    with pytest.raises(ValueError, match="no joints"):
        m.chain()
