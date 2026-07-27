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

import pytest

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


def test_mesh_divergence_is_deliberate(fixtures):
    # DELIBERATE, RECORDED DIVERGENCE from RDK, not drift:
    # RDK loads mesh files during parse and hard-fails when they're
    # missing. Probe (RDK v1.0.0), 2026-07-27:
    #   REJECT  meshed.urdf  failed to build mesh map: failed to load
    #           mesh file .../fixtures/meshes/link1.stl (referenced as
    #           meshes/link1.stl): open .../meshes/link1.stl: no such
    #           file or directory
    # (Both mycobot gripper URDFs in the wider corpus reject the same
    # way, for the same reason.) armkit does NOT resolve/require mesh
    # files at parse time -- unresolved meshes are meant to surface as a
    # validator finding (a later task), not an ACCEPT/REJECT parse
    # failure, because a validator's job is to report what's wrong with
    # a file, not refuse to look at it. So armkit accepts this file today.
    m = parse_urdf(fixtures / "meshed.urdf")
    assert m.dof == 1
