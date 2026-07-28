#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=1.26", "trimesh>=4.0", "pycollada>=0.8", "viam-sdk>=0.79"]
# ///
"""armkit -- validate and inspect Viam arm kinematics files.

Why this exists
---------------
Nothing in the Viam ecosystem validates a kinematics file before it reaches
RDK. A URDF whose joints are effectively in the wrong order, whose
translations were authored in meters where Viam expects millimeters, or
whose `continuous` joint silently carries infinite limits, all parse
cleanly on their own and only misbehave once a real arm is planning motion
against them. Forward kinematics itself lives only in Go, inside RDK's
referenceframe package -- authors of Python and C++ Viam arm modules have
no way to ask "what pose does my model produce at this joint
configuration?" without standing up a full robot. armkit answers both
questions offline, directly from the kinematics file, in whatever language
its author is comfortable running a small script in.

Only `validate` is implemented today. `meshes`, `simplify`, and `convert`
are later work -- this script does not stub them.

RDK-parity note (read before trusting a PASS): armkit deliberately diverges
from RDK in two recorded places (see tests/test_parity.py) -- RDK hard-fails
when a mesh file it references cannot be found on disk, while armkit does
not resolve mesh paths at all yet (mesh support is later work); and RDK
*panics* on a joint with no <origin> element, while armkit follows the URDF
spec and defaults it to identity, which is correct but means armkit will
happily validate a file RDK would crash on. **An armkit PASS does not
guarantee RDK will load the file.**
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from _armkit.checks import (
    Finding,
    check_at_bounds,
    check_dof,
    check_joint_limits,
    check_joints_off_chain,
    check_rdk_parity_risks,
    check_unit_scale,
)
from _armkit.fk import forward_kinematics
from _armkit.model import KinematicModel
from _armkit.transforms import matrix_to_wxyz_quaternion
from _armkit.urdf import parse_urdf

# RDK's own wording for the multi-leaf case (referenceframe's
# model_json.go, matched verbatim in _armkit/model.py's
# _require_resolvable_tip -- see test_parity.py). Matched here only to
# decide whether to attach a --tip remedy; the message text itself is never
# altered, so parity with RDK's diagnosis stays intact.
_MULTI_LEAF_MARKER = "need exactly one end effector, have "

RDK_DIVERGENCE_NOTE = (
    "An armkit PASS does not guarantee RDK will load this file: RDK hard-fails on an "
    "unresolved mesh file (armkit does not check mesh paths yet) and panics on a joint "
    "missing <origin> (armkit correctly defaults it to identity per the URDF spec)."
)

SVA_NOT_IMPLEMENTED = (
    "SVA/DH JSON kinematics files are not yet supported by armkit (planned for a later "
    "task; see _armkit/sva.py). Only .urdf files are implemented today."
)

# What makes --json safe to extend: new finding `code`s (A6 will add
# unresolved-mesh/heavy-mesh; A8/A9 will add more) are additive and do not
# require a consumer update, AS LONG AS consumers switch on `level`
# (error/warn) for anything they don't recognize, rather than hard-failing
# on an unknown code. State this explicitly rather than leaving it
# implicit -- an implicit contract is not one a consumer can rely on.
_JSON_CONTRACT = "unknown finding `code`s should be handled by `level` (error/warn), not by code"

# For a tool distributed by copy-paste inside a skill (no package manager
# entry, no `pip show`), this string is what makes a bug report actionable
# -- "which armkit" has no other answer. Bump manually; there is no
# packaging metadata to derive it from since armkit.py ships as a single
# PEP 723 file, not an installed distribution.
ARMKIT_VERSION = "0.1.0"


def _usage_error(message: str) -> None:
    """Print `message` and exit 2 -- a usage/environment problem, not a finding
    about the user's file. Plain `sys.exit(message)` would NOT do this: Python
    exits with status 1 for a string argument, only status 2 for an int, so
    every exit-2 path below exits with an explicit `sys.exit(2)` after
    printing, not `sys.exit(message)`.
    """
    print(message, file=sys.stderr)
    sys.exit(2)


def _first_fork_link(model: KinematicModel) -> str | None:
    """The first link, walking from the root, with more than one child
    joint -- or None if the tree never forks (shouldn't happen when this is
    only called for the multi-leaf case) or the fork is the root itself
    (which cannot be used as --tip; see below).

    This is what `_structure_finding` suggests instead of a leaf: armkit
    cannot know which of several leaves is an arm's intended end effector
    (a gripper finger, one arm of a dual-arm robot, a camera flange, a pump
    -- the real corpus has all four), but it CAN compute where the chain
    stops being unambiguous, which is useful regardless of which leaf turns
    out to be the right one -- for a gripper, that's the tool flange itself.

    Uses model.bfs_joints() (whole-tree order), not chain() -- chain()
    requires an already-resolved tip, which is exactly what doesn't exist
    yet here. bfs_joints() only validates roots/cycles/disconnection, which
    already succeeded by the time this is called (chain() checks those
    first, via the same method, before ever reaching the multi-leaf error).

    Skips a fork at the ROOT link deliberately: `--tip <root>` would itself
    raise ("declared tip ... is not reachable from the root") since a root
    is by definition never any joint's child -- suggesting it would hand
    back a remedy that fails when followed. Measured across the fixtures
    this fix was built against: none forked at the root (every one has a
    single trunk joint before any branching), but the guard costs nothing
    and a suggestion that doesn't work is worse than none.
    """
    try:
        order = model.bfs_joints()
    except ValueError:
        return None
    children_links = {j.child for j in order}
    counts: dict[str, int] = {}
    for j in order:
        counts[j.parent] = counts.get(j.parent, 0) + 1
    seen: set[str] = set()
    for j in order:
        if j.parent in seen:
            continue
        seen.add(j.parent)
        if counts[j.parent] > 1 and j.parent in children_links:
            return j.parent
    return None


def _structure_finding(model: KinematicModel, message: str) -> Finding:
    """Build the `structure` finding for a chain()/dof failure.

    RDK's own wording is kept byte-for-byte in `message` -- test_parity.py
    pins it for parity, and armkit being MORE useful than RDK is the point
    of armkit existing, not a reason to paraphrase RDK's diagnosis. A remedy
    is attached as a SEPARATE field, only for the multi-leaf case -- a
    cycle, disconnection, or multiple-roots failure gets no remedy here,
    since declaring a tip does not give the model a coherent tree to walk.

    The remedy names the FORK POINT (_first_fork_link), not a leaf.
    Measured during review: suggesting the alphabetically-first leaf
    (an earlier version of this fix) named a gripper finger on a real
    mycobot file and one arm of a dual-arm robot on another -- a developer
    who copies that suggestion gets forward kinematics to the wrong frame,
    with no error, which is worse than the dead end this fix started from.
    armkit genuinely does not know why a model branches or which leaf is
    the intended end effector -- the real corpus has grippers, dual-arm
    robots, camera flanges, and pumps all branching this way -- so the
    wording below makes no claim about which, or why.
    """
    if _MULTI_LEAF_MARKER not in message:
        return Finding("error", "structure", message)

    fork = _first_fork_link(model)
    if fork is not None:
        remedy = (
            "-> this model has more than one end-effector candidate, so armkit cannot\n"
            "   tell which is your arm's output frame. The chain first branches at\n"
            f"   {fork!r} -- if that is your arm's tool flange, use --tip {fork};\n"
            "   otherwise pick one of the links listed above.\n"
            "   Re-run with --tip <link>."
        )
    else:
        remedy = (
            "-> this model has more than one end-effector candidate, so armkit cannot\n"
            "   tell which is your arm's output frame.\n"
            "   Re-run with --tip <link>, picking one of the links listed above."
        )
    return Finding("error", "structure", message, remedy=remedy)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="armkit",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--version", action="version", version=f"armkit {ARMKIT_VERSION}",
    )
    # NOT required: a bare `armkit.py` (no subcommand at all) should print
    # help and exit 0, not argparse's usage error -- see main()'s
    # `if args.command is None` branch. A recognized subcommand missing its
    # OWN required arguments (e.g. `validate` with no file) still exits 2;
    # that is argparse's ordinary positional-argument handling and is
    # unaffected by this.
    subparsers = parser.add_subparsers(dest="command")

    validate_parser = subparsers.add_parser(
        "validate",
        help="validate a kinematics file and report structural/limit findings",
        description=(
            "Parse a kinematics file, check its topology and joint limits, "
            "flag likely unit-scale mistakes, and optionally compute a "
            "forward-kinematics pose at a given joint configuration.\n\n"
            + RDK_DIVERGENCE_NOTE
            + "\n\n--json contract: " + _JSON_CONTRACT + "."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    validate_parser.add_argument("file", help="path to a kinematics file (.urdf today)")
    validate_parser.add_argument(
        "--at", metavar="Q1,Q2,...",
        help="joint values (radians; millimeters for prismatic joints), comma-separated, "
             "one per degree of freedom -- prints the resulting tip pose",
    )
    validate_parser.add_argument(
        "--expect-dof", type=int, metavar="N",
        help="fail if the model's degrees of freedom is not exactly N",
    )
    validate_parser.add_argument(
        "--tip", metavar="LINK",
        help="declare the end-effector link (needed for a model with more than one leaf)",
    )
    validate_parser.add_argument(
        "--json", action="store_true",
        help="emit machine-readable JSON instead of text",
    )
    validate_parser.set_defaults(func=cmd_validate)

    return parser


def cmd_validate(args: argparse.Namespace) -> int:
    path = Path(args.file)

    if path.suffix == ".json":
        _usage_error(SVA_NOT_IMPLEMENTED)
    if path.suffix != ".urdf":
        _usage_error(
            f"{path}: only files with a .urdf extension are supported today "
            "(.json/SVA support is planned; see _armkit/sva.py)"
        )

    try:
        model = parse_urdf(path, tip=args.tip)
    except ValueError as e:
        if "libviam_rust_utils" in str(e):
            _usage_error(
                "armkit could not load viam-sdk's native library on this platform.\n"
                "Supported: Linux (glibc/musl) x86_64/aarch64/armv7, macOS x86_64/arm64, Windows x64."
            )
        return _report(path, None, [Finding("error", "parse", str(e))], args)

    # Bind chain() to a local ONCE and reuse it. dof/base_link/tip_link are
    # properties that each independently call chain(), which raises on a
    # branching/cyclic/multi-rooted/disconnected model -- calling any of
    # them here (or worse, all three, as a naive summary line would) turns
    # one bad model into a traceback or four identical errors instead of one
    # exit-1 finding. Everything below derives from this single `chain`.
    try:
        chain = model.chain()
    except ValueError as e:
        return _report(path, None, [_structure_finding(model, str(e))], args)

    base = chain[0].parent
    tip = chain[-1].child
    actuated = model.actuated_joints
    dof = len(actuated)
    # model.primary_output_frame is None exactly when --tip was not given:
    # parse_urdf only sets it from an explicit `tip=` argument. A model can
    # have exactly one leaf and still not be the flange a user expects
    # (e.g. a sensor frame ahead of the "real" tool frame) -- without this,
    # nothing distinguishes "you told armkit the tip" from "armkit picked
    # the only leaf it found".
    tip_auto_selected = model.primary_output_frame is None
    summary = {
        "name": model.name, "dof": dof, "base": base, "tip": tip,
        "tip_auto_selected": tip_auto_selected,
    }

    findings: list[Finding] = []
    findings += check_dof(actuated, args.expect_dof)
    findings += check_joints_off_chain(model, chain, actuated, tip)
    findings += check_joint_limits(actuated)
    findings += check_unit_scale(model, chain)

    pose_report = None
    if args.at is not None:
        try:
            at_values = [float(v) for v in args.at.split(",")]
        except ValueError:
            _usage_error(f"--at: could not parse {args.at!r} as comma-separated numbers")
        if len(at_values) != dof:
            _usage_error(f"--at expects {dof} value(s) (dof={dof}), got {len(at_values)}")

        findings += check_at_bounds(actuated, at_values)

        pose = forward_kinematics(model, at_values)
        point_mm = [float(x) for x in pose[:3, 3]]
        quat_wxyz = [float(q) for q in matrix_to_wxyz_quaternion(pose[:3, :3])]
        pose_report = {"point_mm": point_mm, "quat_wxyz": quat_wxyz}

    # RDK-parity risk findings (Fix 2) are added only once nothing else has
    # already failed -- RDK parity is moot for a file that didn't even pass
    # armkit's own checks, so a FAIL should not also be cluttered with them.
    if not any(f.is_error for f in findings):
        findings += check_rdk_parity_risks(model)

    return _report(path, summary, findings, args, pose_report)


# Codes emitted by check_rdk_parity_risks -- used only to summarize them
# into the "rdk_parity" field below; the findings themselves are still
# printed/JSON'd like any other finding.
_RDK_PARITY_CODES = {"mesh-references", "missing-origin"}


def _report(
    path: Path,
    summary: dict | None,
    findings: list[Finding],
    args: argparse.Namespace,
    pose_report: dict | None = None,
) -> int:
    ok = not any(f.is_error for f in findings)
    verdict = "PASS" if ok else "FAIL"

    # A specific, per-file answer to "does armkit PASS mean RDK will load
    # this?" in place of a fixed disclaimer printed every run regardless of
    # relevance (Fix 2). Moot on a FAIL for a different reason -- None
    # there, not an empty/misleading "guaranteed" claim.
    if ok:
        parity_findings = [f for f in findings if f.code in _RDK_PARITY_CODES]
        rdk_parity = {"guaranteed": not parity_findings, "reasons": [f.message for f in parity_findings]}
    else:
        rdk_parity = None

    if args.json:
        payload = {
            # Bump this on any BREAKING change to this shape (a renamed or
            # removed key, a changed type). Adding a finding `code` is NOT
            # breaking -- see "contract" below -- and does not need a bump.
            "schema_version": 1,
            "file": str(path),
            "summary": summary,
            "findings": [
                {
                    "level": f.level, "code": f.code, "joint": f.joint, "joints": f.joints,
                    "message": f.message, "remedy": f.remedy,
                }
                for f in findings
            ],
            "pose": pose_report,
            "verdict": verdict,
            "rdk_parity": rdk_parity,
            "contract": _JSON_CONTRACT,
        }
        print(json.dumps(payload, indent=2))
    else:
        if summary is not None:
            tip_note = (
                " (auto-selected: only leaf; override with --tip)"
                if summary["tip_auto_selected"] else ""
            )
            print(f"{summary['name']}: {summary['dof']} actuated joints, "
                  f"base {summary['base']} -> tip {summary['tip']}{tip_note}")
        else:
            print(f"{path}: could not determine model structure")
        for f in findings:
            tag = "ERROR" if f.is_error else "WARN"
            print(f"  [{tag}] {f.code}: {f.message}")
            if f.remedy:
                for line in f.remedy.splitlines():
                    print(f"          {line}")
        if pose_report is not None:
            p, q = pose_report["point_mm"], pose_report["quat_wxyz"]
            print(f"pose at --at: point_mm=[{p[0]:.9f} {p[1]:.9f} {p[2]:.9f}] "
                  f"quat_wxyz=[{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}]")
        print(verdict)

    return 0 if ok else 1


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.command is None:
        parser.print_help()
        return 0
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
