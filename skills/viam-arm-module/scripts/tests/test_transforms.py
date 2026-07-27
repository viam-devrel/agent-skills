import numpy as np
import pytest

from _armkit.transforms import axis_angle_to_matrix, rpy_to_matrix


def test_rpy_identity_is_identity():
    # Every rpy in the fixtures is "0 0 0", so without this test the
    # function could return the identity unconditionally and the whole
    # suite would still pass.
    assert np.allclose(rpy_to_matrix(0, 0, 0), np.eye(3))


def test_rpy_yaw_quarter_turn():
    # A 90-degree yaw about Z should rotate +X onto +Y.
    expected = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    assert np.allclose(rpy_to_matrix(0, 0, np.pi / 2), expected, atol=1e-9)


@pytest.mark.parametrize("roll,pitch,yaw", [
    (0.3, -0.7, 1.1),
    (np.pi / 4, np.pi / 6, -np.pi / 3),
    (-2.5, 0.9, 3.0),
    (1.5707963267948966, 0.0, 0.0),
])
def test_rpy_is_orthonormal_rotation(roll, pitch, yaw):
    # Covers non-trivial angle combinations: the matrix must be a
    # genuine rotation (orthonormal columns, determinant 1), not just
    # "some matrix" -- a wrong-axis or mis-scaled implementation would
    # fail this even if it happened to pass the identity/quarter-turn
    # special cases above.
    r = rpy_to_matrix(roll, pitch, yaw)
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(r), 1.0, atol=1e-9)


def test_axis_angle_zero_angle_is_identity():
    # Without this test, a broken formula could still coincidentally
    # produce the identity for a couple of well-chosen (axis, angle)
    # pairs; angle=0 must always fall through to identity regardless
    # of axis.
    axis = np.array([0.0, 0.0, 1.0])
    assert np.allclose(axis_angle_to_matrix(axis, 0.0), np.eye(3), atol=1e-9)


def test_axis_angle_quarter_turn_about_z():
    # A 90-degree rotation about +Z should match the same known matrix
    # rpy_to_matrix produces for a pure yaw quarter-turn -- both rotation
    # constructions must agree, since FK relies on that.
    expected = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    axis = np.array([0.0, 0.0, 1.0])
    assert np.allclose(axis_angle_to_matrix(axis, np.pi / 2), expected, atol=1e-9)


@pytest.mark.parametrize("axis,angle", [
    (np.array([1.0, 0.0, 0.0]), 0.4),
    (np.array([0.0, 1.0, 0.0]), -1.2),
    (np.array([0.0, 0.0, 1.0]), 2.7),
    (np.array([1.0, 1.0, 1.0]) / np.sqrt(3), 0.9),
    (np.array([1.0, -2.0, 2.0]) / 3.0, -2.4),
])
def test_axis_angle_is_orthonormal_rotation(axis, angle):
    # Covers non-trivial, non-axis-aligned axes: the result must be a
    # genuine rotation (orthonormal columns, determinant 1), not just
    # "some matrix built from sin/cos" -- a sign error in Rodrigues'
    # formula would fail this even where it happened to pass the
    # special cases above.
    r = axis_angle_to_matrix(axis, angle)
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(r), 1.0, atol=1e-9)
