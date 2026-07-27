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
    # pure-numpy implementation on the actual ROTATION, not on any buffer
    # layout -- this is the mathematical counterpart to
    # test_viam_spatialmath_rotation_matrix_elements_are_column_major below,
    # which instead pins the SDK's raw layout. Together: the layout test
    # catches an SDK-side buffer change at its source; this one catches any
    # mathematical divergence regardless of cause (a layout change the
    # layout test somehow missed, a different SDK bug entirely, etc.).
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

    See test_ffi_rpy_matches_reference_oracle /
    test_ffi_axis_angle_matches_reference_oracle above for the
    complementary check: this test catches an SDK-side buffer change at
    its source, those catch any mathematical divergence regardless of
    cause. Neither subsumes the other -- keep both.
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
