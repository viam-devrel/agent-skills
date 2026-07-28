import numpy as np
import pytest

from _armkit.transforms import axis_angle_to_matrix, rpy_to_matrix

# --- Test-only reference oracles -------------------------------------------
#
# These are the pure-numpy rpy_to_matrix/axis_angle_to_matrix implementations
# transforms.py used *before* commit ec95f51 switched to delegating to
# viam.spatialmath's FFI (recoverable verbatim via
# `git show ec95f51^:skills/viam-arm-module/scripts/_armkit/transforms.py`).
# They are kept here, deliberately, as executable documentation: both were
# verified against RDK v1.0.0 across the corpus scan (162 files, 0 verdict
# changes, 0 DoF changes, max FK delta 0.000e+00) before the FFI swap, so
# they are a second, independently-implemented, RDK-proven source of truth
# for the rotation math -- not a placeholder or a guess.
#
# Do NOT import these into _armkit/transforms.py or any production module.
# Their only job is to be compared against the real (FFI-backed)
# implementations below, in test_ffi_matches_reference_oracle_*.


def _reference_rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Pre-FFI rpy_to_matrix: URDF fixed-axis roll-pitch-yaw -> 3x3 rotation (Rz @ Ry @ Rx)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _reference_axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Pre-FFI axis_angle_to_matrix: Rodrigues' rotation formula. `axis` must be unit length."""
    k = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


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


@pytest.mark.parametrize("roll,pitch,yaw", [
    (0.0, 0.0, 0.0),                       # zero
    (0.5, 0.0, 0.0),                       # single-axis: roll only
    (0.0, 0.5, 0.0),                       # single-axis: pitch only
    (0.0, 0.0, 0.5),                       # single-axis: yaw only
    (0.3, -0.7, 1.1),                      # all three nonzero
    (np.pi / 4, np.pi / 6, -np.pi / 3),    # all three nonzero, mixed sign
    (-2.5, 0.9, 3.0),                      # negatives, one angle near +pi
    (-0.1, -0.2, -0.3),                    # all negative
    (np.pi, 0.0, 0.0),                     # exactly +pi, single axis
    (0.0, -np.pi, 0.0),                    # exactly -pi, single axis
    (3.5, -4.0, 5.5),                      # all three past +-pi in magnitude
])
def test_ffi_rpy_matches_reference_oracle(roll, pitch, yaw):
    # The FFI-backed rpy_to_matrix must agree with the pre-FFI, RDK-proven
    # pure-numpy implementation on the actual ROTATION -- this is a
    # version-independent property (it makes no assumption about how the
    # SDK represents a rotation internally), unlike a test pinning
    # RotationMatrix.elements' buffer layout, which is exactly what broke
    # between viam-sdk 0.79.2 and 0.80.0 (see transforms.py's module
    # docstring) and is why transforms.py no longer reads that buffer at
    # all. This oracle comparison is what actually caught that break: it
    # fails on both versions if rpy_to_matrix disagrees with the reference,
    # regardless of which buffer convention is currently in fashion.
    # 1e-12 because the measured max delta across the full RDK-verified
    # corpus was exactly 0.0 -- anything looser here would be hiding
    # something, not tolerating noise.
    actual = rpy_to_matrix(roll, pitch, yaw)
    expected = _reference_rpy_to_matrix(roll, pitch, yaw)
    assert np.allclose(actual, expected, atol=1e-12), (actual, expected)


@pytest.mark.parametrize("axis,angle", [
    (np.array([1.0, 0.0, 0.0]), 0.0),                          # zero angle
    (np.array([1.0, 0.0, 0.0]), 0.4),                          # axis-aligned, small angle
    (np.array([0.0, 1.0, 0.0]), -1.2),                         # axis-aligned, negative
    (np.array([0.0, 0.0, 1.0]), 2.7),                          # axis-aligned, near-pi-ish
    (np.array([1.0, 0.0, 0.0]), np.pi),                        # axis-aligned, exactly +pi
    (np.array([0.0, 1.0, 0.0]), -np.pi),                       # axis-aligned, exactly -pi
    (np.array([0.0, 0.0, 1.0]), np.pi - 1e-6),                 # axis-aligned, just under +pi
    (np.array([0.0, 0.0, 1.0]), -np.pi + 1e-6),                # axis-aligned, just above -pi
    (np.array([1.0, 1.0, 1.0]) / np.sqrt(3), 0.9),             # skewed axis, small angle
    (np.array([1.0, -2.0, 2.0]) / 3.0, -2.4),                  # skewed axis, negative
    (np.array([1.0, 1.0, 1.0]) / np.sqrt(3), np.pi),           # skewed axis, exactly pi
    (np.array([1.0, -2.0, 2.0]) / 3.0, np.pi - 1e-3),          # skewed axis, near pi
])
def test_ffi_axis_angle_matches_reference_oracle(axis, angle):
    # Same purpose as test_ffi_rpy_matches_reference_oracle above, for
    # axis_angle_to_matrix. Angles at and near +-pi are where Rodrigues'
    # formula and a quaternion-mediated conversion are most likely to
    # diverge if either is subtly wrong (sin(angle) passing through zero,
    # sign conventions on the "long way around" rotation), so they're
    # deliberately over-represented here rather than left to the
    # orthonormality sweep's more arbitrary angle choices.
    actual = axis_angle_to_matrix(axis, angle)
    expected = _reference_axis_angle_to_matrix(axis, angle)
    assert np.allclose(actual, expected, atol=1e-12), (actual, expected)


# REMOVED: test_viam_spatialmath_rotation_matrix_elements_are_column_major
# used to live here, pinning RotationMatrix.elements as column-major on
# viam-sdk 0.79.2. viam-sdk 0.80.0 flipped that buffer to row-major --
# silently, no deprecation, no version note -- which made that test both
# WRONG (asserting something now false) and beside the point: it pinned an
# FFI implementation detail transforms.py no longer depends on at all.
# rpy_to_matrix/axis_angle_to_matrix now go through Quaternion's w/i/j/k
# scalar accessors instead of RotationMatrix.elements (see transforms.py's
# module docstring for the full incident writeup), so there is no buffer
# layout left to pin here. test_ffi_rpy_matches_reference_oracle and
# test_ffi_axis_angle_matches_reference_oracle above are the replacement:
# they assert the property that actually matters -- the FFI's rotation
# agrees with the RDK-proven numpy reference -- which holds regardless of
# how any given SDK version represents a rotation internally, and it is
# exactly what caught this incident (23 failures under 0.80.0, including
# these two tests).


# ---------------------------------------------------------------------------
# matrix_to_wxyz_quaternion (Fix 4, A7 review): armkit.py's --at pose output
# and tests/test_fk.py's assert_pose_matches used to each carry an
# independently-written copy of this Shepperd's-method conversion. They
# agreed (verified over 4000 random rotations, worst 1-|dot| = 4.44e-16),
# but a regression in the CLI's copy had nothing to catch it -- the test
# helper structurally could not, since it was a DIFFERENT implementation.
# Unified here, in the one module whose stated purpose is keeping rotation
# constructions in one place; both callers now import this.
# ---------------------------------------------------------------------------

from _armkit.transforms import matrix_to_wxyz_quaternion


def test_matrix_to_wxyz_quaternion_identity():
    q = matrix_to_wxyz_quaternion(np.eye(3))
    assert np.allclose(q, [1.0, 0.0, 0.0, 0.0], atol=1e-9)


@pytest.mark.parametrize("axis,expected_quat,label", [
    (np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0, 0.0], "x"),
    (np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0, 0.0], "y"),
    (np.array([0.0, 0.0, 1.0]), [0.0, 0.0, 0.0, 1.0], "z"),
])
def test_matrix_to_wxyz_quaternion_180_degree_rotations(axis, expected_quat, label):
    # Exercises all four branches of Shepperd's method: a 180-degree turn
    # about X/Y/Z makes m[0,0]/m[1,1]/neither diagonal entry dominate,
    # taking the trace<=0 branches a near-identity rotation never reaches.
    r = axis_angle_to_matrix(axis, np.pi)
    q = matrix_to_wxyz_quaternion(r)
    dot = float(np.dot(q, np.array(expected_quat)))
    assert abs(abs(dot) - 1.0) < 1e-6, (label, q, expected_quat, dot)


@pytest.mark.parametrize("seed", range(50))
def test_matrix_to_wxyz_quaternion_round_trips_random_rotations(seed):
    # Broad regression net: for an arbitrary rotation, converting to a
    # quaternion and back to a matrix (via axis_angle_to_matrix on the
    # quaternion's own axis-angle form) must reproduce the original matrix.
    rng = np.random.default_rng(seed)
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = rng.uniform(-np.pi, np.pi)
    r = axis_angle_to_matrix(axis, angle)
    q = matrix_to_wxyz_quaternion(r)
    assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-9)
    w, x, y, z = q
    reconstructed = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    assert np.allclose(reconstructed, r, atol=1e-9), (r, reconstructed)
