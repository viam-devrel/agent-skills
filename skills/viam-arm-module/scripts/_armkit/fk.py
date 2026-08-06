"""Forward kinematics for a KinematicModel.

Why this exists: forward kinematics lives only in Go, inside RDK's
referenceframe package (Model.Transform). Authors of Python and C++
Viam arm modules have no way to check their own kinematics model
without standing up a full robot -- they cannot ask "what pose does my
URDF produce at this joint configuration?" in their own language. This
module answers that question directly from a parsed KinematicModel, so
`armkit` can validate poses the same way it validates topology.

Like urdf.py, `armkit` is a validator: everything this module raises to
a caller is a ValueError, with context, matching the house exception
contract. joint_values() already raises ValueError on a DoF mismatch;
nothing here is allowed to let that surface as an IndexError or
KeyError instead.
"""
from __future__ import annotations

import numpy as np

from .model import Joint, KinematicModel
from .transforms import axis_angle_to_matrix

ROTARY_TYPES = {"revolute", "continuous"}


def joint_transform(joint: Joint, value: float) -> np.ndarray:
    """The 4x4 transform a single joint contributes at `value`.

    `joint.origin @ motion`: origin is the fixed parent-to-joint-frame
    transform from the URDF <origin>; motion is the joint's own
    contribution at this value -- a rotation about `joint.axis` for
    revolute/continuous, a translation along `joint.axis * value` for
    prismatic, and identity for fixed.
    """
    if joint.type in ROTARY_TYPES:
        motion = np.eye(4)
        motion[:3, :3] = axis_angle_to_matrix(joint.axis, value)
    elif joint.type == "prismatic":
        motion = np.eye(4)
        motion[:3, 3] = joint.axis * value
    elif joint.type == "fixed":
        motion = np.eye(4)
    else:
        raise ValueError(
            f"joint {joint.name!r} has unsupported type {joint.type!r} for forward kinematics"
        )
    return joint.origin @ motion


def link_poses(model: KinematicModel, inputs) -> dict[str, np.ndarray]:
    """Every link's 4x4 pose relative to the model's base link.

    Walks model.bfs_joints() -- every joint in the tree, not just the
    ones on chain()'s root-to-tip path -- so a branch link off the tip
    path (e.g. a second gripper finger) still gets a pose. BFS order
    guarantees each joint's parent link is already placed before that
    joint is processed, since the parent was either the root (seeded
    below) or was itself a `child` earlier in the same walk. A link
    that no joint ever references (fully disconnected from the tree --
    e.g. an unused "world" link some URDFs declare) has no pose here,
    correctly: there is nothing to compute it from.

    `inputs` is the flat BFS-ordered input vector (see
    KinematicModel.joint_values); a DoF mismatch is raised there and
    propagates unchanged, since a ValueError from a mismatched input
    count is already exactly the contract this module needs.
    """
    vals = model.joint_values(inputs)
    poses: dict[str, np.ndarray] = {}
    for j in model.bfs_joints():
        parent = poses.setdefault(j.parent, np.eye(4))
        poses[j.child] = parent @ joint_transform(j, vals[j.name])
    return poses


def forward_kinematics(model: KinematicModel, inputs) -> np.ndarray:
    """The 4x4 pose of `model.tip_link` relative to `model.base_link`."""
    return link_poses(model, inputs)[model.tip_link]
