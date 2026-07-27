import numpy as np
import pytest

from _armkit.transforms import rpy_to_matrix


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
