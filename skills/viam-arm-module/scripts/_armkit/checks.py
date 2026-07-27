"""Validation findings for an already-parsed, already-structurally-valid
KinematicModel.

Why this exists: `armkit validate` reports several independent classes of
problem a kinematics file can have -- missing/inverted joint limits, a likely
meters/millimeters mixup, `continuous` joints RDK treats as unlimited, an
`--at` configuration outside declared limits -- and this module is the one
place that knows all of them, so armkit.py itself stays a thin CLI wrapper
around parsing, these checks, and forward kinematics.

Every function here takes an already-successfully-parsed KinematicModel and
its already-computed chain()/actuated_joints -- callers own computing those
exactly once (see the ordering-trap note in armkit.py) and pass the results
in, rather than each check re-deriving them and re-raising on a broken model.
Nothing in this module raises: a structurally broken model is armkit.py's
job to catch as a `structure` finding before any of these run at all.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .model import Joint, KinematicModel

# Heuristic thresholds for the unit-scale warning: a single joint
# translating more than this is more plausibly an un-converted meters value
# than a real millimeter offset; a whole base-to-tip chain shorter than this
# is more plausibly a mis-scaled (meters-as-mm) tiny arm than a real one.
UNIT_SCALE_MAX_JOINT_MM = 10_000.0
UNIT_SCALE_MIN_CHAIN_MM = 10.0


@dataclass
class Finding:
    level: str   # "error" | "warn"
    code: str
    message: str
    remedy: str | None = None   # armkit-authored fix suggestion, never RDK's own wording

    @property
    def is_error(self) -> bool:
        return self.level == "error"


def check_joint_limits(actuated: list[Joint]) -> list[Finding]:
    """missing-limits / zero-limits / inverted-limits / continuous-joint.

    Order matters, per an ordering trap found by earlier review: an actuated
    joint with no <limit> element at all gets lower = upper = None (urdf.py).
    `None == None` is True, so checking zero-limits first misdiagnoses a
    missing limit as a zero-width one; `None > None` raises TypeError,
    crashing the CLI. missing-limits must be checked first, with `is None`.

    `continuous` joints are exempt from all three: urdf.py always gives them
    lower=upper=-/+inf regardless of whether a <limit> element is present, so
    neither comparison would ever fire for them anyway -- but they still get
    their own warning, since RDK treats them as having no limit at all.
    """
    findings: list[Finding] = []
    for j in actuated:
        if j.type == "continuous":
            findings.append(Finding(
                "warn", "continuous-joint",
                f"joint {j.name!r} is continuous; RDK gives it infinite limits",
            ))
            continue
        if j.lower is None or j.upper is None:
            findings.append(Finding(
                "error", "missing-limits",
                f"joint {j.name!r} is actuated but declares no <limit>",
            ))
        elif j.lower == j.upper:
            findings.append(Finding(
                "error", "zero-limits",
                f"joint {j.name!r} has zero-width limits (lower == upper == {j.lower})",
            ))
        elif j.lower > j.upper:
            findings.append(Finding(
                "error", "inverted-limits",
                f"joint {j.name!r} has inverted limits (lower={j.lower} > upper={j.upper})",
            ))
    return findings


def check_unit_scale(model: KinematicModel, chain: list[Joint]) -> list[Finding]:
    """unit-scale warn: a lone huge joint translation, or a tiny whole chain.

    Both are heuristics for a meters/millimeters authoring mistake, not
    something armkit can know for certain -- hence warn, not error. The
    per-joint check walks the WHOLE tree (model.bfs_joints()), not just
    `chain` (chain()'s root-to-tip path), so an oversized joint on a branch
    that isn't on the tip path is still caught; the "whole chain" reach
    check is specifically about `chain`, the declared/resolved tip path.
    """
    findings: list[Finding] = []

    all_joints = model.bfs_joints()
    if all_joints:
        biggest = max(all_joints, key=lambda j: np.linalg.norm(j.origin[:3, 3]))
        d = float(np.linalg.norm(biggest.origin[:3, 3]))
        if d > UNIT_SCALE_MAX_JOINT_MM:
            findings.append(Finding(
                "warn", "unit-scale",
                f"joint {biggest.name!r} translates {d:.1f} mm "
                f"(> {UNIT_SCALE_MAX_JOINT_MM:.0f} mm) -- check for a meters/millimeters mixup",
            ))

    reach = sum(float(np.linalg.norm(j.origin[:3, 3])) for j in chain)
    if reach < UNIT_SCALE_MIN_CHAIN_MM:
        findings.append(Finding(
            "warn", "unit-scale",
            f"base-to-tip chain reach is only {reach:.3f} mm "
            f"(< {UNIT_SCALE_MIN_CHAIN_MM:.0f} mm) -- check for a meters/millimeters mixup",
        ))
    return findings


def check_dof(actuated: list[Joint], expect_dof: int | None) -> list[Finding]:
    """dof-mismatch: --expect-dof given and the model's DOF differs."""
    if expect_dof is None:
        return []
    dof = len(actuated)
    if dof != expect_dof:
        return [Finding("error", "dof-mismatch", f"expected {expect_dof} DOF, model has {dof}")]
    return []


def check_at_bounds(actuated: list[Joint], values: list[float]) -> list[Finding]:
    """input-out-of-bounds: an --at value outside its joint's declared limits.

    Closes a deliberate divergence recorded in test_parity.py
    (test_fk_does_not_enforce_joint_limits_yet): fk.py computes a pose for
    any input regardless of limits, matching RDK's Transform(), which
    rejects an out-of-range input outright. This is where armkit closes that
    gap -- as a finding rather than refusing to compute, since even an
    out-of-bounds pose is worth showing the user.

    A joint with missing/zero/inverted limits (already reported separately
    by check_joint_limits) is skipped here rather than raising or
    misreporting -- there is no sound "in bounds" question to ask about it.
    continuous joints carry +-inf limits and always pass.
    """
    findings: list[Finding] = []
    for j, v in zip(actuated, values):
        if j.type == "continuous" or j.lower is None or j.upper is None:
            continue
        if v < j.lower or v > j.upper:
            findings.append(Finding(
                "error", "input-out-of-bounds",
                f"--at value {v} for joint {j.name!r} is outside its limits [{j.lower}, {j.upper}]",
            ))
    return findings
