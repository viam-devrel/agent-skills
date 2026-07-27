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


def test_chain_raises_on_disconnected_chains():
    # Two joints forming two separate chains: base->l1 and x->y.
    # Note: this does NOT reach the "disconnected joints" message. Both
    # "base" and "x" fail to appear as any joint's child, so both are
    # picked up as distinct roots, and the implementation's multiple-root
    # check fires first. Asserting on the message actually observed,
    # not the one a reader might assume from the "disconnection" framing.
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
