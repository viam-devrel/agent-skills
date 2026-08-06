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

**Why this reads `Quaternion.w/i/j/k`, never `RotationMatrix.elements`:**
`elements` is a flat 9-value buffer whose row-vs-column-major layout is
an FFI implementation detail, not part of any documented contract --
and it is NOT stable. viam-sdk 0.79.2 handed back a column-major
buffer (despite the class's own docstring claiming row-major);
viam-sdk 0.80.0 flipped it to genuinely row-major, silently, with no
deprecation or version note. A `reshape(..., order="F")` written to
compensate for 0.79.2's layout became WRONG the moment 0.80.0 shipped
-- every rotation silently became its own transpose (for an orthogonal
matrix, its inverse), with no exception anywhere, on a `viam-sdk>=0.79`
floor that resolves 0.80.0 on any cold install today. `Quaternion`'s
`w`/`i`/`j`/`k` are individually-named scalar accessors, not a buffer a
caller has to guess the layout of -- reconstructing the 3x3 from those
four numbers (see `_matrix_from_quaternion` below) is layout-agnostic
by construction and measured identical (max diff 1.1e-16) on both
0.79.2 and 0.80.0. A future reader who sees an unused `RotationMatrix`
import and is tempted to "simplify" back to `.elements`: don't --
that's exactly the footgun this function exists to avoid.
"""
from __future__ import annotations

import numpy as np
from viam.spatialmath import AxisAngle, EulerAngles

M_TO_MM = 1000.0


def _matrix_from_quaternion(q) -> np.ndarray:
    """viam.spatialmath Quaternion -> 3x3 rotation, via its scalar w/i/j/k
    accessors -- never via RotationMatrix.elements (see module docstring).
    """
    w, x, y, z = q.w, q.i, q.j, q.k
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw -> 3x3 rotation.

    Delegates to viam.spatialmath's EulerAngles -> Quaternion conversion
    (the same conversion RDK itself uses) rather than hand-building
    Rz @ Ry @ Rx in numpy.
    """
    return _matrix_from_quaternion(EulerAngles(roll, pitch, yaw).to_quaternion())


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' rotation formula, via viam.spatialmath. `axis` must be unit length.

    This is the revolute/continuous-joint counterpart to rpy_to_matrix's
    fixed-orientation rotation: a URDF <axis> plus a joint value, turned
    into the same kind of 3x3 rotation. Living beside rpy_to_matrix (not
    in fk.py) keeps both rotation constructions in one module, since FK
    composes them and they must agree on convention -- and now both
    delegate to the same canonical backend.
    """
    return _matrix_from_quaternion(AxisAngle(axis[0], axis[1], axis[2], angle).to_quaternion())


def matrix_to_wxyz_quaternion(r: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> unit quaternion (w, x, y, z), via Shepperd's
    method: pick the numerically stable branch based on the trace and the
    largest diagonal entry, avoiding the sqrt-of-a-small-or-negative-number
    issue a single naive formula runs into near 180-degree rotations.

    Plain numpy, not a round-trip through viam.spatialmath's
    RotationMatrix. This module no longer reads RotationMatrix.elements
    at all (see the module docstring: its buffer layout flipped between
    viam-sdk 0.79.2 and 0.80.0 with no warning), so introducing it here
    -- in the reverse, CONSTRUCTOR direction, which was never verified
    in the first place -- would reintroduce exactly the class of risk
    the rest of this module was rewritten to avoid, for a caller (the
    CLI's --at pose output) that has no need of the native library at
    all: this is pure quaternion algebra, not an FFI call.

    Previously implemented twice, independently, by armkit.py's --at pose
    output and tests/test_fk.py's assert_pose_matches -- they agreed
    (verified over 4000 random rotations, worst 1-|dot| = 4.44e-16), but
    that agreement was not being tested: a regression in one copy had
    nothing to catch it. Unified here, in the one module whose stated
    purpose is keeping rotation constructions in one place.
    """
    trace = np.trace(r)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])
