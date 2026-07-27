"""Forward-kinematics tests.

Expected poses below are literal values captured by running
tools/rdkprobe (RDK v1.0.0) by hand against these exact fixtures on
2026-07-27 -- not hand-derived, per this project's history of getting
hand-computed FK wrong twice. Each probe invocation is recorded next to
its assertion so a future re-run can reproduce it.

Point comparisons are exact-ish (RDK prints 9 decimals of millimeters).
Orientation comparisons go through a quaternion built from our rotation
matrix, since q and -q represent the same rotation -- see
`_quaternion_from_matrix` / `assert_pose_matches` below.
"""
from __future__ import annotations

import numpy as np
import pytest

from _armkit.fk import forward_kinematics, joint_transform, link_poses
from _armkit.transforms import axis_angle_to_matrix
from _armkit.urdf import parse_urdf

POINT_TOL_MM = 1e-6


def _quaternion_from_matrix(r: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> unit quaternion (w, x, y, z).

    Shepperd's method: pick the numerically stable branch based on the
    trace and the largest diagonal entry, avoiding the sqrt-of-a-small-
    or-negative-number issue a single naive formula runs into near
    180-degree rotations.
    """
    m = r
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])


def assert_pose_matches(pose: np.ndarray, expected_point_mm, expected_quat, point_tol=POINT_TOL_MM, quat_tol=1e-6):
    point = pose[:3, 3]
    assert np.allclose(point, expected_point_mm, atol=point_tol), (point, expected_point_mm)

    q = _quaternion_from_matrix(pose[:3, :3])
    expected_q = np.array(expected_quat, dtype=float)
    # q and -q represent the identical rotation -- compare the absolute
    # dot product against 1.0 rather than components directly.
    dot = float(np.dot(q, expected_q))
    assert abs(abs(dot) - 1.0) < quat_tol, (q, expected_q, dot)


def test_mimic_serial_matches_rdk(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27:
    #   go run . test_mimic_serial.urdf --at 0.1,-0.4
    #   POSE  point_mm=[-19.568679001 0.000000000 295.034065440]
    #         quat=[0.980066578 0.000000000 -0.198669331 0.000000000]
    m = parse_urdf(fixtures / "test_mimic_serial.urdf")
    pose = forward_kinematics(m, [0.1, -0.4])
    assert_pose_matches(
        pose,
        [-19.568679001, 0.0, 295.034065440],
        [0.98006658, 0, -0.19866933, 0],
    )


def test_ur20_matches_rdk(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27:
    #   go run . ur20.urdf --at 0.1,-0.4,0.7,0.2,-0.3,0.5
    #   POSE  point_mm=[-1332.073428033 -483.810901083 238.695348151]
    #         quat=[0.590775551 0.624281425 -0.198576774 0.470982181]
    m = parse_urdf(fixtures / "ur20.urdf")
    pose = forward_kinematics(m, [0.1, -0.4, 0.7, 0.2, -0.3, 0.5])
    assert_pose_matches(
        pose,
        [-1332.073428033, -483.810901083, 238.695348151],
        [0.590775551, 0.624281425, -0.198576774, 0.470982181],
    )


def test_two_link_zero_position(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27:
    #   go run . two_link.urdf --at 0,0
    #   POSE  point_mm=[2000.000000000 0.000000000 0.000000000]
    #         quat=[1.000000000 0.000000000 0.000000000 0.000000000]
    m = parse_urdf(fixtures / "two_link.urdf")
    pose = forward_kinematics(m, [0.0, 0.0])
    assert_pose_matches(pose, [2000.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])


def test_two_link_quarter_turn_j1(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27:
    #   go run . two_link.urdf --at 1.5707963267948966,0
    #   POSE  point_mm=[1000.000000000 1000.000000000 0.000000000]
    #         quat=[0.707106781 0.000000000 0.000000000 0.707106781]
    # Tip is 1000 + 1000*cos(q1), 1000*sin(q1), 0 -- rotating j1 (base
    # joint) both revolves the tip about the origin AND reorients it.
    m = parse_urdf(fixtures / "two_link.urdf")
    pose = forward_kinematics(m, [np.pi / 2, 0.0])
    assert_pose_matches(pose, [1000.0, 1000.0, 0.0], [0.707106781, 0.0, 0.0, 0.707106781])


def test_two_link_j2_rotates_without_moving_tip(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27. Note: RDK rejects an *exact* pi
    # input here -- j2's <limit> is "-3.14159"/"3.14159" (5 decimal
    # places), and math.pi (3.14159265358979) sits just outside that
    # bound ("input out of bounds"). 3.14159 (the limit's own value) is
    # used instead -- this is a case where the probe contradicted the
    # prompt's literal "0, pi" and was trusted over it.
    #   go run . two_link.urdf --at 0,3.14159
    #   POSE  point_mm=[2000.000000000 0.000000000 0.000000000]
    #         quat=[0.000001327 0.000000000 0.000000000 1.000000000]
    # The tip link sits at the j2 frame: rotating j2 changes orientation
    # (a ~180-degree turn about Z, i.e. diag(-1, -1, 1)) but the tip
    # position is unchanged, since j2's own translation doesn't move
    # under its own rotation.
    m = parse_urdf(fixtures / "two_link.urdf")
    pose = forward_kinematics(m, [0.0, 3.14159])
    assert np.allclose(pose[:3, 3], [2000.0, 0.0, 0.0], atol=POINT_TOL_MM)
    assert np.allclose(pose[:3, :3], np.diag([-1.0, -1.0, 1.0]), atol=1e-5)


def test_revolute_prismatic_matches_rdk(fixtures):
    # Prismatic is the last untested branch in joint_transform: FK mixes
    # millimeters (translation) and radians (rotation) in one flat
    # `vals` dict, and a units slip there ("did prismatic get treated
    # like a radian, or a revolute's axis*value like meters?") would
    # regress silently without a dedicated test.
    #
    # Probe (RDK v1.0.0), 2026-07-27:
    #   go run . revolute_prismatic.urdf --at 0.5,300
    #   POSE  point_mm=[-143.827661581 263.274768567 500.000000000]
    #         quat=[0.968912422 0.000000000 0.000000000 0.247403959]
    m = parse_urdf(fixtures / "revolute_prismatic.urdf")
    pose = forward_kinematics(m, [0.5, 300.0])
    assert_pose_matches(
        pose,
        [-143.827661581, 263.274768567, 500.000000000],
        [0.968912422, 0.0, 0.0, 0.247403959],
    )

    # The pose only pins the translation axis*value math; it says
    # nothing about whether the joint's OWN limits got mm-scaled at
    # parse time (urdf.py multiplies prismatic lower/upper by M_TO_MM).
    # That's the other half of the units contract, asserted directly.
    j2 = next(j for j in m.chain() if j.name == "j2")
    assert j2.lower == -500.0
    assert j2.upper == 500.0


def test_link_poses_covers_branch_links_not_just_chain(fixtures):
    # two_leaf.urdf has 4 links (base, palm, finger_l, finger_r) but a
    # single tip path (base -> palm -> finger_l) visits only 3 of them.
    # link_poses's own docstring promises "every link's pose", so a
    # branch link RDK would compute a perfectly well-defined pose for
    # (finger_r) must not be silently dropped -- A6's inspect_meshes
    # iterates model.links and would KeyError on whatever chain() skips.
    m = parse_urdf(fixtures / "two_leaf.urdf", tip="finger_l")
    assert m.dof == 3  # j0, jl, jr -- branch joints consume input slots too
    poses = link_poses(m, [0.0, 0.0, 0.0])
    assert set(poses.keys()) == {"base", "palm", "finger_l", "finger_r"}


def test_link_poses_omits_orphan_link_with_no_joint(fixtures):
    # ur20.urdf declares a "world" link that no joint ever references as
    # parent or child -- it is disconnected from the tree entirely, not
    # merely off the tip path. It has no pose (there is nothing to
    # compute it from), and link_poses must not invent one.
    m = parse_urdf(fixtures / "ur20.urdf")
    poses = link_poses(m, [0.1, -0.4, 0.7, 0.2, -0.3, 0.5])
    assert "world" not in poses


def test_two_link_link_poses(fixtures):
    # link1's pose is exactly j1's origin translation (1000mm along x)
    # at the zero configuration -- hand-computable, and cross-checked:
    # the tip pose in this same table matches the probe result above
    # for test_two_link_zero_position.
    m = parse_urdf(fixtures / "two_link.urdf")
    poses = link_poses(m, [0.0, 0.0])
    assert set(poses.keys()) == {"base", "link1", "tip"}
    assert np.allclose(poses["base"], np.eye(4))
    assert np.allclose(poses["link1"][:3, 3], [1000.0, 0.0, 0.0])
    assert np.allclose(poses["tip"][:3, 3], [2000.0, 0.0, 0.0])


def test_forward_kinematics_wrong_input_count_raises(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    with pytest.raises(ValueError, match="expected 2 input value"):
        forward_kinematics(m, [0.0])


def test_joint_transform_fixed_is_identity_times_origin(fixtures):
    m = parse_urdf(fixtures / "ur20.urdf")
    fixed = next(j for j in m.joints if j.type == "fixed")
    t = joint_transform(fixed, 0.0)
    assert np.allclose(t, fixed.origin)


@pytest.mark.parametrize("axis,expected_quat,label", [
    (np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0, 0.0], "x"),
    (np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0, 0.0], "y"),
    (np.array([0.0, 0.0, 1.0]), [0.0, 0.0, 0.0, 1.0], "z"),
])
def test_quaternion_from_matrix_handles_180_degree_rotations(axis, expected_quat, label):
    # _quaternion_from_matrix uses Shepperd's method, which branches on
    # the trace and (once the trace isn't positive) on which diagonal
    # entry dominates. Every pose test above takes the trace>0 branch
    # (branch 1) -- none of them get anywhere near 180 degrees. This
    # exercises the other three branches directly: a 180-degree turn
    # about X makes m[0,0] the dominant diagonal entry (branch 2), about
    # Y makes m[1,1] dominant (branch 3), and about Z falls through to
    # the else (branch 4). Confirmed by instrumenting
    # _quaternion_from_matrix's branch logic during development: X, Y, Z
    # took branches 2, 3, 4 respectively, each producing (up to float
    # noise ~6e-17 on w) exactly the expected quaternion below -- so this
    # is not a test that happens to pass via branch 1.
    r = axis_angle_to_matrix(axis, np.pi)
    q = _quaternion_from_matrix(r)
    dot = float(np.dot(q, np.array(expected_quat)))
    assert abs(abs(dot) - 1.0) < 1e-6, (label, q, expected_quat, dot)


def test_joint_transform_unsupported_type_raises():
    from _armkit.model import Joint

    j = Joint(
        name="weird", type="floating", parent="a", child="b",
        origin=np.eye(4), axis=None, lower=None, upper=None,
    )
    with pytest.raises(ValueError, match="floating"):
        joint_transform(j, 0.0)
