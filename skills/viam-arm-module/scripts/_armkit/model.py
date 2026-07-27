"""Common parsed representation for kinematic models.

Both the URDF and SVA parsers produce a KinematicModel; every consumer
(FK, mesh inspection, every CLI subcommand) reads only this. Keeping one
representation is what stops URDF parsing from being reimplemented per
subcommand.

Units: translations are ALWAYS millimeters, angles ALWAYS radians.
Conversion from URDF (meters) or SVA (degrees) happens at parse time.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

ACTUATED_TYPES = {"revolute", "continuous", "prismatic"}


@dataclass(eq=False)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray          # 4x4 homogeneous, translation in mm
    axis: np.ndarray | None     # unit 3-vector, None for fixed
    lower: float | None         # radians (or mm for prismatic)
    upper: float | None

    @property
    def actuated(self) -> bool:
        return self.type in ACTUATED_TYPES


@dataclass
class Link:
    name: str
    collision_meshes: list[str] = field(default_factory=list)
    visual_meshes: list[str] = field(default_factory=list)
    collision_primitives: list[dict] = field(default_factory=list)


@dataclass(eq=False)
class KinematicModel:
    name: str
    joints: list[Joint]
    links: dict[str, Link]
    source_format: str          # "urdf" | "sva" | "dh"
    source_path: str

    @property
    def actuated_joints(self) -> list[Joint]:
        return [j for j in self.chain() if j.actuated]

    @property
    def dof(self) -> int:
        return len(self.actuated_joints)

    def chain(self) -> list[Joint]:
        """Joints ordered base to tip. Raises on missing/multiple roots, branching, cycles, or disconnection."""
        if not self.joints:
            raise ValueError("kinematic model has no joints")
        by_parent = {}
        for j in self.joints:
            by_parent.setdefault(j.parent, []).append(j)
        children = {j.child for j in self.joints}
        roots = [j.parent for j in self.joints if j.parent not in children]
        if not roots:
            raise ValueError("kinematic model has no root link (cycle?)")
        if len(set(roots)) > 1:
            raise ValueError(f"multiple root links: {sorted(set(roots))}")

        ordered, current, seen = [], roots[0], set()
        while current in by_parent:
            if current in seen:
                raise ValueError(f"cycle in kinematic model at link {current!r}")
            seen.add(current)
            branches = by_parent[current]
            if len(branches) > 1:
                names = sorted(b.name for b in branches)
                raise ValueError(f"branching at link {current!r}: {names}")
            ordered.append(branches[0])
            current = branches[0].child
        if len(ordered) != len(self.joints):
            raise ValueError("disconnected joints in kinematic model")
        return ordered

    @property
    def base_link(self) -> str:
        return self.chain()[0].parent

    @property
    def tip_link(self) -> str:
        return self.chain()[-1].child
