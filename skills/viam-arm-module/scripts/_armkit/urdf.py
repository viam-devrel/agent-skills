"""URDF parsing into the common KinematicModel.

Why this exists: nothing in the Viam ecosystem validates a kinematics
file. URDF is the format Viam recommends reusing when a manufacturer
ships one, and it is the only practical way to carry mesh collision
geometry -- so it is the format this toolkit reads first.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .model import Joint, KinematicModel, Link

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


def _origin(elem: ET.Element | None) -> np.ndarray:
    t = np.eye(4)
    if elem is None:
        return t
    xyz = [float(v) for v in elem.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in elem.get("rpy", "0 0 0").split()]
    t[:3, :3] = rpy_to_matrix(*rpy)
    t[:3, 3] = np.array(xyz) * M_TO_MM
    return t


def parse_urdf(path: str | Path) -> KinematicModel:
    path = Path(path)
    root = ET.parse(path).getroot()

    links: dict[str, Link] = {}
    for le in root.findall("link"):
        links[le.get("name")] = Link(name=le.get("name"))

    joints: list[Joint] = []
    for je in root.findall("joint"):
        jtype = je.get("type")
        axis_elem = je.find("axis")
        axis = None
        if jtype != "fixed":
            raw = [float(v) for v in (axis_elem.get("xyz") if axis_elem is not None else "1 0 0").split()]
            vec = np.array(raw, dtype=float)
            norm = np.linalg.norm(vec)
            if norm == 0:
                raise ValueError(f"joint {je.get('name')!r} has a zero-length axis")
            axis = vec / norm

        limit = je.find("limit")
        lower = upper = None
        if jtype == "continuous":
            lower, upper = -np.inf, np.inf
        elif limit is not None:
            lower = float(limit.get("lower", 0.0))
            upper = float(limit.get("upper", 0.0))
            if jtype == "prismatic":
                lower, upper = lower * M_TO_MM, upper * M_TO_MM

        joints.append(Joint(
            name=je.get("name"), type=jtype,
            parent=je.find("parent").get("link"),
            child=je.find("child").get("link"),
            origin=_origin(je.find("origin")),
            axis=axis, lower=lower, upper=upper,
        ))

    return KinematicModel(
        name=root.get("name", path.stem), joints=joints, links=links,
        source_format="urdf", source_path=str(path),
    )
