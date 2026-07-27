"""URDF parsing into the common KinematicModel.

Why this exists: nothing in the Viam ecosystem validates a kinematics
file. URDF is the format Viam recommends reusing when a manufacturer
ships one, and it is the only practical way to carry mesh collision
geometry -- so it is the format this toolkit reads first.

`armkit` is itself a validator, so malformed input is its expected
diet, not an exceptional case: parse_urdf raises ValueError -- and
only ValueError -- for every failure mode, with the source path and
offending joint/element named in the message, so a CLI wrapper can
rely on a single exception type for its exit-code contract.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .model import Joint, KinematicModel, Link, Mimic
from .transforms import M_TO_MM, rpy_to_matrix

# Joint types this toolkit knows how to interpret. URDF also defines
# "floating" and "planar"; both are rejected rather than silently
# treated as dof-0 fixed joints.
SUPPORTED_JOINT_TYPES = {"revolute", "continuous", "prismatic", "fixed"}


def _origin(elem: ET.Element | None, context: str) -> np.ndarray:
    t = np.eye(4)
    if elem is None:
        return t
    try:
        xyz = [float(v) for v in elem.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in elem.get("rpy", "0 0 0").split()]
    except ValueError as e:
        raise ValueError(f"{context}: origin has a non-numeric component ({e})") from e
    if len(xyz) != 3:
        raise ValueError(f"{context}: origin xyz has {len(xyz)} components; expected 3")
    if len(rpy) != 3:
        raise ValueError(f"{context}: origin rpy has {len(rpy)} components; expected 3")
    t[:3, :3] = rpy_to_matrix(*rpy)
    t[:3, 3] = np.array(xyz) * M_TO_MM
    return t


def _parse(path: Path) -> KinematicModel:
    if not path.exists():
        raise ValueError(f"{path}: file not found")
    if path.is_dir():
        raise ValueError(f"{path}: expected a URDF file, got a directory")
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as e:
        raise ValueError(f"{path}: malformed XML ({e})") from e

    # A default xmlns on <robot> (e.g. xmlns="http://www.ros.org/urdf")
    # makes ElementTree prefix EVERY tag in the tree as "{uri}tag", not
    # just the root -- so stripping only the root tag would fix the
    # misleading "not a URDF file" message here while silently breaking
    # every findall("link")/findall("joint") below, replacing it with an
    # even more confusing "no joints" report. Strip uniformly instead.
    for elem in root.iter():
        if elem.tag.startswith("{"):
            elem.tag = elem.tag.split("}", 1)[1]

    if root.tag != "robot":
        raise ValueError(f"{path}: root element is <{root.tag}>, not <robot> -- not a URDF file")

    links: dict[str, Link] = {}
    for le in root.findall("link"):
        name = le.get("name")
        if not name:
            raise ValueError(f"{path}: <link> element missing a 'name' attribute")
        links[name] = Link(name=name)

    joints: list[Joint] = []
    for je in root.findall("joint"):
        jname = je.get("name")
        if not jname:
            raise ValueError(f"{path}: <joint> element missing a 'name' attribute")
        context = f"{path}: joint {jname!r}"

        jtype = je.get("type")
        if jtype not in SUPPORTED_JOINT_TYPES:
            raise ValueError(
                f"{context} has unsupported type {jtype!r}; expected one of "
                f"{sorted(SUPPORTED_JOINT_TYPES)}"
            )

        parent_elem = je.find("parent")
        child_elem = je.find("child")
        if parent_elem is None or not parent_elem.get("link"):
            raise ValueError(f"{context} is missing a <parent link=\"...\"/>")
        if child_elem is None or not child_elem.get("link"):
            raise ValueError(f"{context} is missing a <child link=\"...\"/>")

        axis_elem = je.find("axis")
        axis = None
        if jtype != "fixed":
            try:
                raw = [float(v) for v in (axis_elem.get("xyz") if axis_elem is not None else "1 0 0").split()]
            except ValueError as e:
                raise ValueError(f"{context} has a non-numeric axis component ({e})") from e
            if len(raw) != 3:
                raise ValueError(f"{context} has a {len(raw)}-component axis; expected 3")
            vec = np.array(raw, dtype=float)
            norm = np.linalg.norm(vec)
            if norm == 0:
                raise ValueError(f"{context} has a zero-length axis")
            axis = vec / norm

        limit = je.find("limit")
        lower = upper = None
        if jtype == "continuous":
            lower, upper = -np.inf, np.inf
        elif limit is not None:
            try:
                lower = float(limit.get("lower", 0.0))
            except ValueError as e:
                raise ValueError(f"{context} has a non-numeric limit 'lower' ({e})") from e
            try:
                upper = float(limit.get("upper", 0.0))
            except ValueError as e:
                raise ValueError(f"{context} has a non-numeric limit 'upper' ({e})") from e
            if jtype == "prismatic":
                lower, upper = lower * M_TO_MM, upper * M_TO_MM

        mimic_elem = je.find("mimic")
        mimic = None
        if mimic_elem is not None:
            source = mimic_elem.get("joint")
            if not source:
                raise ValueError(f"{context} has a <mimic> element missing a 'joint' attribute")
            try:
                multiplier = float(mimic_elem.get("multiplier", 1.0))
            except ValueError as e:
                raise ValueError(f"{context} has a non-numeric mimic 'multiplier' ({e})") from e
            try:
                offset = float(mimic_elem.get("offset", 0.0))
            except ValueError as e:
                raise ValueError(f"{context} has a non-numeric mimic 'offset' ({e})") from e
            mimic = Mimic(source=source, multiplier=multiplier, offset=offset)
            # RDK zeroes a mimic joint's own limits (model_urdf.go:191):
            # they're meaningless once the source joint's limits govern.
            lower, upper = 0.0, 0.0

        joints.append(Joint(
            name=jname, type=jtype,
            parent=parent_elem.get("link"),
            child=child_elem.get("link"),
            origin=_origin(je.find("origin"), context),
            axis=axis, lower=lower, upper=upper,
            mimic=mimic,
        ))

    for j in joints:
        for role, ln in (("parent", j.parent), ("child", j.child)):
            if ln not in links:
                raise ValueError(f"{path}: joint {j.name!r} names {role} link {ln!r}, which is not declared")

    joint_names = {j.name for j in joints}
    for j in joints:
        if j.mimic is not None and j.mimic.source not in joint_names:
            raise ValueError(
                f"{path}: joint {j.name!r} mimics {j.mimic.source!r}, which is not declared"
            )

    return KinematicModel(
        name=root.get("name", path.stem), joints=joints, links=links,
        source_format="urdf", source_path=str(path),
    )


def parse_urdf(path: str | Path, tip: str | None = None) -> KinematicModel:
    path = Path(path)
    try:
        model = _parse(path)
        if tip is not None:
            if tip not in model.links:
                leaves = sorted(
                    {j.child for j in model.joints} - {j.parent for j in model.joints}
                )
                raise ValueError(
                    f"{path}: declared tip {tip!r} is not a link in this model "
                    f"(available leaves: {leaves})"
                )
            model.primary_output_frame = tip
        return model
    except ValueError:
        raise
    except Exception as e:
        # Safety net: guarantees the "one exception type" contract even
        # for failure modes this parser doesn't anticipate by name. The
        # wording is deliberate: this branch means armkit itself broke,
        # not that the caller's file is bad -- don't blame the input.
        raise ValueError(
            f"internal error while parsing {path} ({type(e).__name__}: {e}) "
            "-- this is an armkit bug, not a problem with your file"
        ) from e
