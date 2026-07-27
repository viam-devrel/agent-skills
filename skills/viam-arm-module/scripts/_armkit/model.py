"""Common parsed representation for kinematic models.

Both the URDF and SVA parsers produce a KinematicModel; every consumer
(FK, mesh inspection, every CLI subcommand) reads only this. Keeping one
representation is what stops URDF parsing from being reimplemented per
subcommand.

Units: translations are ALWAYS millimeters, angles ALWAYS radians.
Conversion from URDF (meters) or SVA (degrees) happens at parse time.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

ACTUATED_TYPES = {"revolute", "continuous", "prismatic"}


@dataclass(eq=False)
class Mimic:
    """A mimic relationship: this joint's value = multiplier * source's + offset.

    RDK derives a mimic joint's position at runtime rather than giving it
    its own input slot, so a joint carrying this is excluded from
    actuated_joints/dof even though it is a real joint in the tree and
    still contributes a transform (see KinematicModel._bfs_all_joints).
    """
    source: str                 # name of the joint this one mimics
    multiplier: float = 1.0
    offset: float = 0.0


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
    mimic: Mimic | None = None

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
    # Declared end-effector link name (URDF `tip=` argument to parse_urdf,
    # or SVA `output_frames`). None means "auto-detect the single leaf".
    # Stored here -- rather than threaded through as a parse-time-only
    # argument -- so FK and later tasks (CLI --tip, SVA output_frames)
    # can read what the model's output frame is.
    primary_output_frame: str | None = None

    def _validate_roots(self) -> tuple[dict[str, list[Joint]], str]:
        """Shared precondition checks for both chain() and actuated_joints/dof.

        Returns (by_parent, root_link). Raises on an empty model or on
        zero/multiple root links -- checks that must fire regardless of
        whether a tip is declared or how many leaves the tree has.
        """
        if not self.joints:
            raise ValueError("kinematic model has no joints")
        by_parent: dict[str, list[Joint]] = {}
        for j in self.joints:
            by_parent.setdefault(j.parent, []).append(j)
        children = {j.child for j in self.joints}
        roots = [j.parent for j in self.joints if j.parent not in children]
        if not roots:
            raise ValueError("kinematic model has no root link (cycle?)")
        if len(set(roots)) > 1:
            raise ValueError(f"multiple root links: {sorted(set(roots))}")
        return by_parent, roots[0]

    def _bfs_all_joints(self) -> list[Joint]:
        """Every joint in the tree, BFS-ordered from the root.

        This is RDK's model.go:150 input-ordering strategy: the flat
        input vector is built by BFS over the WHOLE frame system, not
        along the tip path, so sibling-branch joints occupy slots
        between chain-path joints. This walk is independent of any
        declared tip. It also doubles as the disconnection/cycle check
        (Part 2): it must reach every joint in the model exactly once,
        regardless of which link ends up as the chosen tip -- unlike the
        old check (`len(ordered) != len(self.joints)` after walking
        root->tip), which would misfire once branching-with-a-declared-
        tip made "on the tip path" a legitimately incomplete subset of
        "in the model".
        """
        by_parent, root = self._validate_roots()
        order: list[Joint] = []
        seen_links = {root}
        queue = deque([root])
        while queue:
            link = queue.popleft()
            # Sorted for determinism -- mirrors RDK's bfsFrameNames,
            # which sorts each parent's children before enqueueing them.
            for j in sorted(by_parent.get(link, []), key=lambda j: j.child):
                if j.child in seen_links:
                    raise ValueError(f"cycle in kinematic model at link {j.child!r}")
                seen_links.add(j.child)
                order.append(j)
                queue.append(j.child)
        if len(order) != len(self.joints):
            raise ValueError("disconnected joints in kinematic model")
        return order

    def _resolve_tip(self) -> str:
        """Pick (or validate) the model's end-effector link.

        - a declared tip (primary_output_frame) is used as-is; branching
          elsewhere in the tree is legal in this case
        - no declared tip, exactly one leaf: that leaf is the tip
        - no declared tip, multiple leaves: ValueError listing them
          (sorted), matching RDK's "need exactly one end effector, have
          [...]" diagnosis

        This is a precondition for the model being queryable at ALL, not
        just for chain(): RDK never produces a Model (so never has a DoF
        to report) for a tree with an unresolved tip -- an ambiguous
        branching model with no declared output frame fails at load
        time. actuated_joints/dof must raise here too, or a caller could
        get a DoF for a model RDK would never have accepted.
        """
        children_links = {j.child for j in self.joints}
        parent_links = {j.parent for j in self.joints}
        leaves = children_links - parent_links

        if self.primary_output_frame is not None:
            tip = self.primary_output_frame
            if tip not in children_links:
                raise ValueError(
                    f"declared tip {tip!r} is not reachable from the root "
                    f"(available leaves: {sorted(leaves)})"
                )
            return tip
        if len(leaves) == 1:
            return next(iter(leaves))
        if len(leaves) > 1:
            raise ValueError(f"need exactly one end effector, have {sorted(leaves)}")
        raise ValueError("kinematic model has no end effector (leaf) link")

    @property
    def actuated_joints(self) -> list[Joint]:
        # BFS over the ENTIRE tree (Part 4), not just chain()'s root->tip
        # path -- a branching model's dof counts branch joints too.
        # Mimic joints are excluded (Part 1): they're real joints in the
        # tree and still contribute a transform, they just don't consume
        # an input slot -- RDK derives their value at runtime instead.
        order = self._bfs_all_joints()
        self._resolve_tip()  # raise if the tip is ambiguous -- see _resolve_tip
        return [j for j in order if j.actuated and j.mimic is None]

    @property
    def dof(self) -> int:
        return len(self.actuated_joints)

    def chain(self) -> list[Joint]:
        """Joints ordered root to tip.

        Raises on missing/multiple roots, cycles, or disconnection
        (via _bfs_all_joints -- see there for why those checks moved),
        or on an ambiguous/unreachable tip (via _resolve_tip).
        """
        self._bfs_all_joints()  # validate roots/cycles/disconnection first
        tip = self._resolve_tip()

        child_to_joint = {j.child: j for j in self.joints}
        ordered_rev: list[Joint] = []
        current, seen = tip, set()
        while current in child_to_joint:
            if current in seen:
                raise ValueError(f"cycle in kinematic model at link {current!r}")
            seen.add(current)
            j = child_to_joint[current]
            ordered_rev.append(j)
            current = j.parent
        return list(reversed(ordered_rev))

    @property
    def base_link(self) -> str:
        return self.chain()[0].parent

    @property
    def tip_link(self) -> str:
        return self.chain()[-1].child
