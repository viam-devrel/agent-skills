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


def test_viam_spatialmath_rotation_matrix_elements_are_column_major():
    """Pins the actual (undocumented) layout of viam.spatialmath's
    RotationMatrix.elements, independent of anything in transforms.py.

    RotationMatrix's own class docstring claims `elements[3*row + col]`
    (row-major) -- that is WRONG. The buffer underneath is nalgebra's,
    which is column-major. rpy_to_matrix/axis_angle_to_matrix both
    reshape the raw `elements` list with order="F" to compensate.

    If a future SDK release ever actually fixes this (making `elements`
    genuinely row-major, matching its own docstring), reshaping with
    order="F" would silently start producing the TRANSPOSE of the
    intended rotation -- for an orthogonal matrix, its own inverse --
    with no exception raised anywhere in the call chain. A quaternion
    comparison cannot catch this (quaternions carry no buffer layout),
    which is exactly why this test exists: it pins the raw layout
    directly against a hand-known rotation matrix, bypassing
    transforms.py's own reshape entirely, so a layout change shows up
    here first, as a loud failure, instead of downstream as a silently
    inverted pose.
    """
    import viam.spatialmath as sm

    # 90-degree yaw about Z -- same known matrix as
    # test_rpy_yaw_quarter_turn/test_axis_angle_quarter_turn_about_z.
    expected = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    elements = sm.EulerAngles(0.0, 0.0, np.pi / 2).to_quaternion().to_rotation_matrix().elements
    assert len(elements) == 9

    col_major = np.array(elements).reshape(3, 3, order="F")
    row_major = np.array(elements).reshape(3, 3)

    assert np.allclose(col_major, expected, atol=1e-9)
    # The trap, pinned explicitly: reading the buffer as the SDK's own
    # docstring claims (row-major) gives the TRANSPOSE, not the matrix.
    # If this assertion ever fails, the SDK's layout changed and
    # rpy_to_matrix/axis_angle_to_matrix's order="F" reshape must be
    # revisited before anything else in this file is trusted.
    assert np.allclose(row_major, expected.T, atol=1e-9)
    assert not np.allclose(row_major, expected, atol=1e-3)
