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
import ast
import json
import re
import sys
from pathlib import Path

import numpy as np

from _armkit.checks import Finding, check_at_bounds, check_dof, check_joint_limits, check_unit_scale
from _armkit.fk import forward_kinematics
from _armkit.urdf import parse_urdf

# RDK's own wording for the multi-leaf case (referenceframe's
# model_json.go, matched verbatim in _armkit/model.py's
# _require_resolvable_tip -- see test_parity.py). Matched here only to
# decide whether to attach a --tip remedy; the message text itself is never
# altered, so parity with RDK's diagnosis stays intact.
_MULTI_LEAF_RE = re.compile(r"need exactly one end effector, have (\[.*\])$")

RDK_DIVERGENCE_NOTE = (
    "An armkit PASS does not guarantee RDK will load this file: RDK hard-fails on an "
    "unresolved mesh file (armkit does not check mesh paths yet) and panics on a joint "
    "missing <origin> (armkit correctly defaults it to identity per the URDF spec)."
)

SVA_NOT_IMPLEMENTED = (
    "SVA/DH JSON kinematics files are not yet supported by armkit (planned for a later "
    "task; see _armkit/sva.py). Only .urdf files are implemented today."
)


def _usage_error(message: str) -> None:
    """Print `message` and exit 2 -- a usage/environment problem, not a finding
    about the user's file. Plain `sys.exit(message)` would NOT do this: Python
    exits with status 1 for a string argument, only status 2 for an int, so
    every exit-2 path below exits with an explicit `sys.exit(2)` after
    printing, not `sys.exit(message)`.
    """
    print(message, file=sys.stderr)
    sys.exit(2)


def _matrix_to_wxyz_quaternion(r: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation matrix -> unit quaternion (w, x, y, z).

    Plain numpy (Shepperd's method), not a round-trip through
    viam.spatialmath's RotationMatrix -- that class's constructor direction
    (Python floats -> native buffer) is unverified here, and _armkit/
    transforms.py already found the READ direction of that same FFI class
    silently transposed unless reshaped `order="F"` (see its "Layout trap"
    docstring). This formula has no native-buffer-layout ambiguity to get
    wrong: it operates only on the already-verified numpy rotation matrix
    fk.py produces.
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
    return float(w), float(x), float(y), float(z)


def _structure_finding(message: str) -> Finding:
    """Build the `structure` finding for a chain()/dof failure.

    RDK's own wording is kept byte-for-byte in `message` -- test_parity.py
    pins it for parity, and armkit being MORE useful than RDK is the point
    of armkit existing, not a reason to paraphrase RDK's diagnosis. A remedy
    is attached as a SEPARATE field, only for the multi-leaf case: 30 of 84
    real vendor URDFs surveyed branch (a gripper shipping attached to the
    arm is the common cause), and that failure is fixable with --tip, using
    a leaf armkit already has in hand from the error message itself. A
    cycle, disconnection, or multiple-roots failure gets no remedy here --
    declaring a tip does not give the model a coherent tree to walk, so
    suggesting --tip for those would send a user down a dead end of its own.
    """
    match = _MULTI_LEAF_RE.search(message)
    if match is None:
        return Finding("error", "structure", message)
    try:
        leaves = ast.literal_eval(match.group(1))
    except (ValueError, SyntaxError):
        leaves = []
    suggestion = leaves[0] if leaves else "<link>"
    remedy = (
        "-> this model branches (common when a gripper ships attached to the arm).\n"
        "   Re-run with --tip <link> to declare the end effector, e.g.\n"
        f"   --tip {suggestion}"
    )
    return Finding("error", "structure", message, remedy=remedy)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="armkit",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate_parser = subparsers.add_parser(
        "validate",
        help="validate a kinematics file and report structural/limit findings",
        description=(
            "Parse a kinematics file, check its topology and joint limits, "
            "flag likely unit-scale mistakes, and optionally compute a "
            "forward-kinematics pose at a given joint configuration.\n\n"
            + RDK_DIVERGENCE_NOTE
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
        return _report(path, None, [_structure_finding(str(e))], args)

    base = chain[0].parent
    tip = chain[-1].child
    actuated = model.actuated_joints
    dof = len(actuated)
    summary = {"name": model.name, "dof": dof, "base": base, "tip": tip}

    findings: list[Finding] = []
    findings += check_dof(actuated, args.expect_dof)
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
        quat_wxyz = list(_matrix_to_wxyz_quaternion(pose[:3, :3]))
        pose_report = {"point_mm": point_mm, "quat_wxyz": quat_wxyz}

    return _report(path, summary, findings, args, pose_report)


def _report(
    path: Path,
    summary: dict | None,
    findings: list[Finding],
    args: argparse.Namespace,
    pose_report: dict | None = None,
) -> int:
    ok = not any(f.is_error for f in findings)
    verdict = "PASS" if ok else "FAIL"

    if args.json:
        payload = {
            "file": str(path),
            "summary": summary,
            "findings": [
                {"level": f.level, "code": f.code, "message": f.message, "remedy": f.remedy}
                for f in findings
            ],
            "pose": pose_report,
            "verdict": verdict,
            "note": RDK_DIVERGENCE_NOTE,
        }
        print(json.dumps(payload, indent=2))
    else:
        if summary is not None:
            print(f"{summary['name']}: {summary['dof']} actuated joints, "
                  f"base {summary['base']} -> tip {summary['tip']}")
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
        print(f"note: {RDK_DIVERGENCE_NOTE}")

    return 0 if ok else 1


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
