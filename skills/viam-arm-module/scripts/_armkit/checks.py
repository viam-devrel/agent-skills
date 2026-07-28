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
    joint: str | None = None          # the single joint this finding is about, if any
    remedy: str | None = None         # armkit-authored fix suggestion, never RDK's own wording
    joints: list[str] | None = None   # MULTIPLE joints, for a finding about more than one at once

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
                joint=j.name,
            ))
            continue
        if j.lower is None or j.upper is None:
            findings.append(Finding(
                "error", "missing-limits",
                f"joint {j.name!r} is actuated but declares no <limit>",
                joint=j.name,
            ))
        elif j.lower == j.upper:
            findings.append(Finding(
                "error", "zero-limits",
                f"joint {j.name!r} has zero-width limits (lower == upper == {j.lower})",
                joint=j.name,
            ))
        elif j.lower > j.upper:
            findings.append(Finding(
                "error", "inverted-limits",
                f"joint {j.name!r} has inverted limits (lower={j.lower} > upper={j.upper})",
                joint=j.name,
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
                joint=biggest.name,
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


def check_joints_off_chain(
    model: KinematicModel, chain: list[Joint], actuated: list[Joint], tip: str,
) -> list[Finding]:
    """joints-off-chain: an actuated joint that is not on the resolved tip's
    root-to-tip path.

    This is a SCOPE check, not an RDK-parity one -- see the "armkit is
    right" divergence recorded for it in tests/test_parity.py, alongside
    the missing-mesh and missing-<origin> divergences. armkit validates a
    VIAM ARM MODULE's kinematics: one serial chain, optionally with a tool
    attached -- not a general URDF/whole-robot validator. `actuated` (BFS
    over the WHOLE frame system, per A3b) deliberately matches RDK's own
    DoF, because RDK models whole robots where a branch's joints
    legitimately consume input slots. A Viam arm module is not a whole
    robot: an actuated joint off the chain means JointPositions will
    return fewer values than the module's own declared DoF -- an arity
    mismatch in GetKinematics and in motion planning, regardless of what
    that off-chain joint actually IS.

    Error level, not warn -- the user's own call: the most common real
    input is a vendor URDF with the gripper still attached, and passing
    that silently is exactly the failure this toolkit exists to prevent.
    No override flag exists (an --allow-off-chain-joints option was
    considered and deliberately not added).

    Runs identically whether the tip was auto-selected or given via --tip
    -- the scope violation depends only on chain membership, not on how
    the tip was resolved. (In practice this can only ever fire when --tip
    was given: URDF is a tree, so a model with a genuinely unique leaf --
    the only way auto-selection succeeds at all -- has no branching
    anywhere, meaning every actuated joint is necessarily on that one
    chain. Still computed the same way in both cases rather than special-
    cased, since the invariant is what keeps this correct, not an
    assumption about how it's reached.)

    A single Finding carries ALL offending joint names under `joints`
    (plural) rather than `joint` (singular, used elsewhere) or one Finding
    per joint: the natural report here is "N actuated joints are not on
    the chain", one aggregated fact, and forcing that into N separate
    findings (or into `joint` by picking one arbitrarily) would fight the
    single-message shape without adding information a consumer couldn't
    already get from `joints`. `joint` stays None on this finding.

    The remedy does NOT assert why the model branches (a gripper, a
    second arm, a camera or tool mount all produce this exact shape --
    armkit cannot tell which, the same class of overclaiming the --tip
    remedy used to make about which leaf is the "real" end effector; see
    _structure_finding). It DOES name the concrete consequence (arity
    mismatch) so the stakes are clear without a guess.

    When off-chain joints OUTNUMBER on-chain ones, the declared tip is far
    more likely wrong than the file is out of scope (a dual-arm robot
    whose fork point was suggested as --tip, per _first_fork_link, is the
    motivating case: 12 of 13 actuated joints end up off-chain). The
    remedy adds a conditional line for exactly this shape, suggesting the
    model's actual leaves as alternative --tip candidates -- reusing the
    same leaf computation _require_resolvable_tip uses for the multi-leaf
    diagnosis, so the suggestion always names real, valid --tip targets.
    """
    chain_names = {j.name for j in chain}
    off_chain = [j.name for j in actuated if j.name not in chain_names]
    if not off_chain:
        return []
    count = len(off_chain)
    on_chain_count = len(actuated) - count
    message = (
        f"{count} actuated joint{'s' if count != 1 else ''} "
        f"{'is' if count == 1 else 'are'} not on the arm's chain to {tip!r}: {off_chain!r}."
    )
    remedy_lines = [
        "-> A Viam arm module's kinematics must describe one serial arm: every actuated",
        "   joint must lie on the chain to the tip, or JointPositions will return fewer",
        "   values than the declared DoF. Off-chain joints usually belong to a separate",
        "   component (a gripper, a second arm, a camera or tool mount). Remove them from",
        "   the file, or set --tip further out if they are genuinely part of this arm.",
    ]
    if count > on_chain_count:
        parent_links = {j.parent for j in model.joints}
        child_links = {j.child for j in model.joints}
        leaves = sorted(child_links - parent_links)
        remedy_lines += [
            f"   more joints are off the chain than on it -- '--tip {tip}' is probably not your",
            f"   arm's output frame; try one of: {', '.join(leaves)}",
        ]
    remedy = "\n".join(remedy_lines)
    return [Finding("error", "joints-off-chain", message, joints=off_chain, remedy=remedy)]


def check_rdk_parity_risks(model: KinematicModel) -> list[Finding]:
    """Findings that make the RDK-parity divergences (see tests/test_parity.py
    and tools/rdkprobe/README.md) concrete for THIS file, in place of a fixed
    disclaimer printed on every invocation regardless of relevance:

    - `mesh-references`: RDK loads every referenced mesh file at parse time
      and hard-fails if one can't be found on disk; armkit does not resolve
      mesh paths at all (that's A6's job -- `unresolved-mesh`/`heavy-mesh`
      are a later, per-file finer-grained check). A file that references
      any mesh carries a real risk of an RDK-only rejection armkit cannot
      see, and this is the single most common reason RDK rejects a real
      vendor file -- flagging just the COUNT is cheap and honest without
      pulling A6's resolution/inspection logic forward.
    - `missing-origin`: RDK v1.0.0 panics (SIGSEGV, model_urdf.go:196) on a
      joint with no <origin> element; armkit follows the URDF spec and
      defaults it to identity, which is correct but means armkit will
      happily pass a file that crashes RDK. Named per joint -- that's
      actionable, since the user can add an explicit
      <origin xyz="0 0 0" rpy="0 0 0"/> and move on.

    Both are warn-level: neither is an armkit-detectable defect in the
    file, they are places armkit's own (correct) permissiveness could be
    hiding an RDK failure. Callers should only add these when the model has
    no OTHER error findings -- RDK parity is moot for a file that didn't
    even pass armkit's own checks.
    """
    findings: list[Finding] = []

    mesh_count = sum(len(link.visual_meshes) + len(link.collision_meshes) for link in model.links.values())
    if mesh_count:
        findings.append(Finding(
            "warn", "mesh-references",
            f"file references {mesh_count} mesh file{'s' if mesh_count != 1 else ''} that armkit "
            "does not resolve; RDK rejects a file whose meshes are missing",
        ))

    for j in model.joints:
        if not j.has_declared_origin:
            findings.append(Finding(
                "warn", "missing-origin",
                f"joint {j.name!r} has no <origin>; RDK v1.0.0 panics on this "
                "(armkit follows the URDF spec and defaults to identity)",
                joint=j.name,
            ))

    return findings


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
                joint=j.name,
            ))
    return findings
