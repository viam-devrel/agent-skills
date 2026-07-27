"""Rotation and unit-conversion helpers shared by the URDF and SVA
parsers, and by forward kinematics.

Keeping this math in one place is what lets both parsers agree on the
same rotation construction and unit conventions (KinematicModel is
always millimeters and radians internally) without either parser
importing from the other.
"""
from __future__ import annotations

import numpy as np

M_TO_MM = 1000.0


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw -> 3x3 rotation (Rz @ Ry @ Rx)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' rotation formula. `axis` must be unit length.

    This is the revolute/continuous-joint counterpart to rpy_to_matrix's
    fixed-orientation rotation: a URDF <axis> plus a joint value, turned
    into the same kind of 3x3 rotation. Living beside rpy_to_matrix (not
    in fk.py) keeps both rotation constructions in one module, since FK
    composes them and they must agree on convention.
    """
    k = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)
