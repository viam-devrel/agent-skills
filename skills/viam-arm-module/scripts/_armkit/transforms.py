"""Rotation and unit-conversion helpers shared by the URDF and SVA
parsers, and by forward kinematics.

Keeping this math in one place is what lets both parsers agree on the
same rotation construction and unit conventions (KinematicModel is
always millimeters and radians internally) without either parser
importing from the other.

Both rotation constructions below delegate to `viam.spatialmath`
rather than reimplementing the math in numpy. That module is a ctypes
binding to `libviam_rust_utils` -- the same native library behind RDK
-- so its conversions are canonical, not a second, independently-wrong
Python reimplementation of what RDK itself does. This matters most for
a future orientation-vector task, whose five orientation types include
Viam's own orientation-vector format, which only this library speaks
natively.

**Layout trap:** `viam.spatialmath.RotationMatrix.elements` is
COLUMN-major, despite that class's own docstring claiming
`elements[3*row + col]` (row-major) -- the buffer is nalgebra's, not
whatever the docstring assumes. Reshaping it as documented (row-major)
silently yields the TRANSPOSE of the intended rotation -- for a
rotation matrix, its own inverse -- with no exception raised. Both
functions below reshape with `order="F"` to compensate; see
`tests/test_transforms.py::test_viam_spatialmath_rotation_matrix_elements_are_column_major`
for the dedicated test pinning this, and what breaks if the buffer
order ever changes.
"""
from __future__ import annotations

import numpy as np
from viam.spatialmath import AxisAngle, EulerAngles

M_TO_MM = 1000.0


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw -> 3x3 rotation.

    Delegates to viam.spatialmath's EulerAngles -> Quaternion ->
    RotationMatrix conversion (the same conversion RDK itself uses),
    rather than hand-building Rz @ Ry @ Rx in numpy.
    """
    elements = EulerAngles(roll, pitch, yaw).to_quaternion().to_rotation_matrix().elements
    return np.array(elements).reshape(3, 3, order="F")


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' rotation formula, via viam.spatialmath. `axis` must be unit length.

    This is the revolute/continuous-joint counterpart to rpy_to_matrix's
    fixed-orientation rotation: a URDF <axis> plus a joint value, turned
    into the same kind of 3x3 rotation. Living beside rpy_to_matrix (not
    in fk.py) keeps both rotation constructions in one module, since FK
    composes them and they must agree on convention -- and now both
    delegate to the same canonical backend.
    """
    elements = AxisAngle(axis[0], axis[1], axis[2], angle).to_quaternion().to_rotation_matrix().elements
    return np.array(elements).reshape(3, 3, order="F")
