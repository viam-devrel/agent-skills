"""Parity suite: armkit's accept/reject verdict and DoF must match RDK's.

The oracle is a small Go program (referenceframe.KinematicModelFromFile)
that is NOT run from this suite -- that would make tests depend on a Go
toolchain and a scratchpad path that won't exist in CI. Instead, each
expected verdict below is a literal value captured by running that probe
once, by hand, against RDK v1.0.0 (per the pinned `go.viam.com/rdk v1.0.0`
in the probe's go.mod) on 2026-07-27. If armkit's behavior legitimately
needs to change in the future, re-run the probe and update the literal
here deliberately -- don't just make the assertion pass.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from _armkit.fk import forward_kinematics
from _armkit.urdf import parse_urdf


def _verdict(fixtures, name, tip=None):
    try:
        m = parse_urdf(fixtures / name, tip=tip)
        return ("ACCEPT", m.dof)
    except ValueError as e:
        return ("REJECT", str(e))


def test_mimic_serial_matches_rdk(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27:
    #   ACCEPT  test_mimic_serial.urdf  DoF=2
    # This is the Part 1 bug fixture: joint3 mimics joint1, so RDK (and
    # now armkit) excludes it from the input schema.
    verdict, dof = _verdict(fixtures, "test_mimic_serial.urdf")
    assert verdict == "ACCEPT"
    assert dof == 2


def test_ur20_matches_rdk(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27:
    #   ACCEPT  ur20.urdf  DoF=6
    verdict, dof = _verdict(fixtures, "ur20.urdf")
    assert verdict == "ACCEPT"
    assert dof == 6


def test_two_link_matches_rdk(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27:
    #   ACCEPT  two_link.urdf  DoF=2
    verdict, dof = _verdict(fixtures, "two_link.urdf")
    assert verdict == "ACCEPT"
    assert dof == 2


def test_two_leaf_matches_rdk(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27, on this exact fixture:
    #   REJECT  two_leaf.urdf  need exactly one end effector, have [finger_r finger_l]
    # (Go map iteration order is randomized, so RDK's own listing order
    # isn't stable across runs -- only the message shape and leaf SET are
    # a parity claim. armkit sorts its own listing deterministically,
    # which is a strict improvement, not a divergence: RDK doesn't
    # promise an order, so any order is compatible with its contract.)
    verdict, msg = _verdict(fixtures, "two_leaf.urdf")
    assert verdict == "REJECT"
    assert "need exactly one end effector" in msg
    assert "finger_l" in msg and "finger_r" in msg


def test_declared_tip_case(fixtures):
    # A declared tip is an armkit/RDK-SVA capability with no URDF-side
    # equivalent in RDK (KinematicModelFromFile takes no tip argument for
    # .urdf files -- there is nothing to probe here for URDF specifically).
    # What IS probed is the underlying claim this capability relies on:
    # dof must count joints on branches the tip doesn't visit. Verified
    # via the probe against an equivalent synthetic SVA fixture with
    # output_frames (RDK v1.0.0, 2026-07-27):
    #   ACCEPT  branch_test.json (trunk + 2 branch joints, output_frames=[branch_a])  DoF=3
    # two_leaf.urdf has the same shape (1 trunk joint, 2 branch joints),
    # so declaring tip="finger_l" here must also accept with dof=3.
    m = parse_urdf(fixtures / "two_leaf.urdf", tip="finger_l")
    assert m.dof == 3
    assert [j.name for j in m.chain()] == ["j0", "jl"]


def test_mimic_of_mimic_matches_rdk(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27:
    #   ACCEPT  mimic_of_mimic.urdf  DoF=1
    # RDK ACCEPTS a mimic-of-a-mimic and composes the multiplier/offset
    # transitively rather than rejecting the chain. j2 mimics q1
    # (multiplier=2.0, offset=0.1); j3 mimics j2 (multiplier=3.0,
    # offset=0.2) -- composed, j3 = 6.0*q1 + 0.5.
    #
    # This composed value was cross-checked against an actual RDK pose,
    # not just arithmetic: feeding q1=0.3 through this file and feeding
    # [0.3, 0.7, 2.3] (the hand-expanded non-mimic equivalent: q1=0.3,
    # j2=2*0.3+0.1=0.7, j3=6*0.3+0.5=2.3) through an otherwise-identical
    # 3-independent-joint URDF produced IDENTICAL poses on RDK v1.0.0 via
    # the probe's `--at` pose mode (point_mm=[0 0 300],
    # quat=[-0.079120889 0 0 0.996865028] for both). Composing with the
    # multiplier updated before the offset (the wrong order) instead
    # gives j3=2.6 and a visibly different pose on the same probe run --
    # confirming RDK really does update offset using the OLD multiplier
    # (model_json.go:194-208), not a coincidence of the arithmetic.
    verdict, dof = _verdict(fixtures, "mimic_of_mimic.urdf")
    assert verdict == "ACCEPT"
    assert dof == 1
    m = parse_urdf(fixtures / "mimic_of_mimic.urdf")
    j3 = next(j for j in m.chain() if j.name == "j3")
    assert (j3.mimic.source, j3.mimic.multiplier, j3.mimic.offset) == ("q1", 6.0, 0.5)


def test_mimic_cycle_matches_rdk(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27:
    #   REJECT  mimic_cycle.urdf  circular mimic joint reference detected: joint "j2"
    # j2 mimics j3, j3 mimics j2 -- value derivation would never
    # terminate. Fixed alongside the leaf-based topology work: a mimic
    # cycle is the same failure mode as chain()'s cycle guard, one layer
    # up, and previously slipped through unchecked (armkit used to
    # ACCEPT this with dof=1).
    verdict, msg = _verdict(fixtures, "mimic_cycle.urdf")
    assert verdict == "REJECT"
    assert 'circular mimic joint reference detected: joint "j2"' in msg


def test_mimic_of_fixed_matches_rdk(fixtures):
    # Probe (RDK v1.0.0), 2026-07-27:
    #   REJECT  mimic_of_fixed.urdf  mimic joint references non-existent
    #           source joint: joint "j3" references source "j2" which
    #           has no DoF
    # j3 mimics j2, a fixed joint -- there's no position to derive a
    # value from. Previously slipped through unchecked (armkit used to
    # ACCEPT this with dof=1).
    verdict, msg = _verdict(fixtures, "mimic_of_fixed.urdf")
    assert verdict == "REJECT"
    assert (
        'mimic joint references non-existent source joint: '
        'joint "j3" references source "j2" which has no DoF'
    ) in msg


def test_mesh_and_origin_divergences_are_deliberate(fixtures):
    # TWO deliberate, RECORDED divergences from RDK, not drift -- both
    # share the same consequence, stated explicitly: an armkit PASS does
    # NOT imply RDK will load this file.
    #
    # 1. RDK loads mesh files during parse and hard-fails when they're
    #    missing. Probe (RDK v1.0.0), 2026-07-27:
    #      REJECT  meshed.urdf  failed to build mesh map: failed to load
    #              mesh file .../fixtures/meshes/link1.stl (referenced as
    #              meshes/link1.stl): open .../meshes/link1.stl: no such
    #              file or directory
    #    (Both mycobot gripper URDFs in the wider corpus reject the same
    #    way, for the same reason.) armkit does NOT resolve/require mesh
    #    files at parse time -- unresolved meshes are meant to surface as
    #    a validator finding (a later task), not an ACCEPT/REJECT parse
    #    failure, because a validator's job is to report what's wrong
    #    with a file, not refuse to look at it.
    #
    # 2. A joint with no <origin> at all makes RDK PANIC (SIGSEGV, nil
    #    pointer dereference indexing an empty childRPY slice) at
    #    model_urdf.go:196, rather than return an error -- verified via
    #    the probe (RDK v1.0.0, 2026-07-27; see no_origin.urdf below).
    #    armkit accepts this: the URDF spec defaults <origin> to
    #    identity, and armkit implements that default (_origin() in
    #    urdf.py). 14 of the 159 corpus files (the Dobot CR/Nova family's
    #    "dummy_joint", plus 2 mycobot files) have a joint missing
    #    <origin> -- but every one of them ALSO references a missing
    #    mesh, so RDK's mesh check (divergence #1, which runs first)
    #    fails before the panic path is ever reached; the panic stays
    #    latent in the corpus today, not exercised, but it is real and it
    #    belongs on the record.
    m = parse_urdf(fixtures / "meshed.urdf")
    assert m.dof == 1

    m = parse_urdf(fixtures / "no_origin.urdf")
    assert m.dof == 1


def test_joints_off_chain_is_a_deliberate_scope_divergence(fixtures):
    # A THIRD deliberate, RECORDED divergence -- but of a different
    # character than the mesh/origin pair above. Those two are about
    # KINEMATIC FIDELITY (does armkit compute/accept what RDK computes/
    # accepts), and armkit is more permissive there because it is *right*
    # and RDK is not (or hasn't run yet). This one is about SCOPE, not
    # fidelity: armkit's dof/chain() MATH still matches RDK exactly here --
    # see test_declared_tip_case above, which verifies m.dof == 3 for this
    # SAME fixture shape (trunk + 2 branch joints) against an equivalent
    # SVA file with output_frames via the probe (RDK DoF=3). RDK counts
    # every actuated frame in the whole tree because it models WHOLE
    # ROBOTS; that counting is correct and unchanged here -- confirmed
    # again below.
    #
    # What diverges is what `armkit validate` (the CLI layer --
    # _armkit/checks.py's check_joints_off_chain, not this parsing/model
    # layer) DOES with that number. armkit validates a VIAM ARM MODULE's
    # kinematics: one serial chain, optionally with a tool attached -- not
    # a general URDF/whole-robot validator. An actuated joint reachable
    # from the root but NOT on the path to the declared tip (a gripper's
    # fingers, shipped in the same URDF as the arm; a second arm on a
    # dual-arm torso; a camera flange; a pump) is a real scope violation
    # for that narrower target: a module built from this file would
    # declare a DoF (via BFS, matching RDK) that its own
    # GetKinematics/motion planning cannot actually drive past the chain
    # path. `check_joints_off_chain` rejects it with an error --
    # something RDK has no concept of and would never flag, since RDK is
    # not scoped to "one arm" at all.
    #
    # armkit is right here in the same sense as the mesh/origin pair: not
    # because its KINEMATICS diverge from RDK's (they don't -- m.dof below
    # matches the probe-verified figure exactly), but because the CLI
    # layer built on top of that kinematics enforces a scope RDK was never
    # asked to enforce. See _armkit/checks.py:check_joints_off_chain for
    # the full reasoning and tests/test_cli.py for CLI-level coverage
    # (including the interaction with the multi-leaf --tip remedy: on a
    # gripper-bearing file, following the fork-point suggestion resolves
    # the STRUCTURE error and then surfaces THIS one -- the intended
    # sequence, not a bug).
    m = parse_urdf(fixtures / "two_leaf.urdf", tip="palm")
    assert m.dof == 3  # matches RDK's own count -- kinematic parity intact
    on_chain = {j.name for j in m.chain()}
    assert on_chain == {"j0"}
    off_chain = sorted(j.name for j in m.actuated_joints if j.name not in on_chain)
    assert off_chain == ["jl", "jr"]  # what check_joints_off_chain would report


def test_fk_does_not_enforce_joint_limits_yet(fixtures):
    # A THIRD divergence, recorded like the two above but of a different
    # character: RDK's Transform() validates each input against the
    # joint's <limit> before computing a pose; _armkit/fk.py does not --
    # forward_kinematics() will happily compute a pose for an
    # out-of-range input. Unlike the mesh/no-origin divergences (armkit
    # is deliberately more permissive there because it is *right* and
    # RDK is not), this one armkit intends to CLOSE: it's an omission,
    # not a design choice, and belongs with the other validation
    # findings in A7 (see the "input-out-of-bounds" row added to A7's
    # findings table in the plan) -- not implemented here, since A4's
    # scope is computing poses, not validating inputs.
    #
    # Probe (RDK v1.0.0), 2026-07-27, against two_link.urdf (limits
    # -3.14159/3.14159 on both joints):
    #   go run . two_link.urdf --at 0,3.141592653589793   (math.pi, 2.65e-6 over)
    #     REJECT  Transform error: Frame: probe.j2 (joint 1): input out
    #             of bounds, input 3.14159 needs to be within range
    #             [3.14159 -3.14159]
    #   go run . two_link.urdf --at 0,100.0                (~16 revolutions)
    #     REJECT  Transform error: Frame: probe.j2 (joint 1): input out
    #             of bounds, input 100.00000 needs to be within range
    #             [3.14159 -3.14159]
    #   go run . two_link.urdf --at 1000000,0
    #     REJECT  Transform error: Frame: probe.j1 (joint 0): input out
    #             of bounds, input 1000000.00000 needs to be within
    #             range [3.14159 -3.14159]
    # armkit computes a pose for all three inputs above instead of
    # rejecting.
    #
    # This is exactly why test_fk.py's
    # test_two_link_j2_rotates_without_moving_tip uses the literal
    # 3.14159 rather than math.pi: RDK rejects math.pi as out-of-bounds
    # (first probe run above), so a test wanting the ~180-degree pose has
    # to stay just inside the limit armkit doesn't yet check.
    m = parse_urdf(fixtures / "two_link.urdf")

    pose = forward_kinematics(m, [0.0, math.pi])
    assert np.allclose(pose[:3, 3], [2000.0, 0.0, 0.0], atol=1e-6)

    pose = forward_kinematics(m, [0.0, 100.0])
    assert np.allclose(pose[:3, 3], [2000.0, 0.0, 0.0], atol=1e-6)

    pose = forward_kinematics(m, [1_000_000.0, 0.0])
    assert not np.allclose(pose[:3, 3], [2000.0, 0.0, 0.0], atol=1e-3)
