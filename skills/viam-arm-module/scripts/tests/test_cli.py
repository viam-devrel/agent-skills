"""CLI-level tests for armkit.py, invoked as a subprocess (sys.executable),
never imported -- this is what a real user runs, PEP 723 header and all.

One test (test_isolated_uv_run_resolves_pep723_dependencies) additionally
shells out via `uv run --isolated`, proving the PEP 723 dependency block
itself is correct: every other test here runs the script under the dev
venv's interpreter, which would happily resolve `_armkit`'s imports even if
armkit.py's own `dependencies = [...]` list were wrong or incomplete. Only
`--isolated` builds a throwaway environment strictly from that header.
"""
from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1]
ARMKIT = SCRIPTS / "armkit.py"


def run(*args):
    return subprocess.run(
        [sys.executable, str(ARMKIT), *args],
        capture_output=True, text=True,
    )


def write_urdf(tmp_path, xml, name="test.urdf"):
    p = tmp_path / name
    p.write_text(xml)
    return p


# ---------------------------------------------------------------------------
# Passing model
# ---------------------------------------------------------------------------

def test_validate_passes_on_good_model(fixtures):
    r = run("validate", str(fixtures / "two_link.urdf"))
    assert r.returncode == 0, r.stderr
    assert "2 actuated joints" in r.stdout
    assert "base base -> tip tip" in r.stdout
    assert "PASS" in r.stdout


# ---------------------------------------------------------------------------
# parse error -> exit 1
# ---------------------------------------------------------------------------

def test_validate_fails_on_missing_file():
    r = run("validate", "/nonexistent.urdf")
    assert r.returncode == 1
    assert "parse" in r.stdout
    assert "FAIL" in r.stdout


# ---------------------------------------------------------------------------
# --expect-dof mismatch -> exit 1
# ---------------------------------------------------------------------------

def test_validate_reports_dof_mismatch(fixtures):
    r = run("validate", str(fixtures / "two_link.urdf"), "--expect-dof", "6")
    assert r.returncode == 1
    assert "dof-mismatch" in r.stdout
    assert "expected 6" in r.stdout


def test_validate_dof_match_does_not_report_mismatch(fixtures):
    r = run("validate", str(fixtures / "two_link.urdf"), "--expect-dof", "2")
    assert r.returncode == 0
    assert "dof-mismatch" not in r.stdout


# ---------------------------------------------------------------------------
# --at prints a pose, RDK-verified value
# ---------------------------------------------------------------------------

def test_validate_at_config_prints_pose(fixtures):
    r = run("validate", str(fixtures / "two_link.urdf"), "--at", "0,0")
    assert r.returncode == 0, r.stderr
    assert "2000" in r.stdout


def test_validate_at_matches_rdk_probe_on_ur20(fixtures):
    # RDK v1.0.0 via tools/rdkprobe, 2026-07-27:
    #   POSE ur20.urdf --at 0.1,-0.4,0.7,0.2,-0.3,0.5
    #   point_mm=[-1332.073428033 -483.810901083 238.695348151]
    #   quat=[0.590775551 0.624281425 -0.198576774 0.470982181]  (w x y z)
    r = run("validate", str(fixtures / "ur20.urdf"), "--at", "0.1,-0.4,0.7,0.2,-0.3,0.5", "--json")
    assert r.returncode == 0, r.stderr
    payload = json.loads(r.stdout)
    point = payload["pose"]["point_mm"]
    assert np.allclose(point, [-1332.073428033, -483.810901083, 238.695348151], atol=1e-6)
    # Quaternion carries a global-sign ambiguity (q and -q are the same
    # rotation); accept either sign against the probe's literal.
    quat = np.array(payload["pose"]["quat_wxyz"])
    target = np.array([0.590775551, 0.624281425, -0.198576774, 0.470982181])
    assert np.allclose(quat, target, atol=1e-6) or np.allclose(quat, -target, atol=1e-6)


# ---------------------------------------------------------------------------
# malformed model -> exit 1 with "structure", not a traceback
# ---------------------------------------------------------------------------

def test_validate_reports_structure_finding_not_traceback(fixtures):
    # two_leaf.urdf has two leaves and no declared --tip: chain()/dof raise
    # "need exactly one end effector". This must NOT surface as a Python
    # traceback (the ordering trap this task exists to catch).
    r = run("validate", str(fixtures / "two_leaf.urdf"))
    assert r.returncode == 1
    assert "structure" in r.stdout
    assert "Traceback" not in r.stdout
    assert "Traceback" not in r.stderr


def test_validate_structure_finding_is_reported_once(fixtures):
    # Binding chain() to a local and reusing it means a broken model
    # produces exactly one structure finding, not four (dof, base_link,
    # tip_link, and the summary line each independently raising).
    r = run("validate", str(fixtures / "two_leaf.urdf"))
    assert r.stdout.count("] structure:") == 1


# ---------------------------------------------------------------------------
# --at outside limits -> exit 1 with input-out-of-bounds
# ---------------------------------------------------------------------------

def test_validate_at_outside_limits_reports_finding(fixtures):
    # two_link.urdf limits are +-3.14159 on both joints; 100.0 is ~16
    # revolutions past the limit.
    r = run("validate", str(fixtures / "two_link.urdf"), "--at", "0,100.0")
    assert r.returncode == 1
    assert "input-out-of-bounds" in r.stdout


def test_validate_at_within_limits_does_not_report_finding(fixtures):
    r = run("validate", str(fixtures / "two_link.urdf"), "--at", "0,0")
    assert r.returncode == 0
    assert "input-out-of-bounds" not in r.stdout


# ---------------------------------------------------------------------------
# continuous joint -> warn, still exits 0
# ---------------------------------------------------------------------------

def test_validate_continuous_joint_warns_but_passes(tmp_path):
    path = write_urdf(tmp_path, """
    <robot name="cont">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="continuous">
        <parent link="base"/><child link="tip"/>
        <origin xyz="1 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
    </robot>
    """)
    r = run("validate", str(path))
    assert r.returncode == 0, r.stderr
    assert "continuous-joint" in r.stdout
    assert "PASS" in r.stdout


# ---------------------------------------------------------------------------
# missing-limits must be diagnosed correctly, not misreported/crash
# ---------------------------------------------------------------------------

def test_validate_missing_limits_reports_correct_code_not_zero_limits(tmp_path):
    # A revolute joint with no <limit> element at all gets lower=upper=None
    # in the parser. missing-limits must fire, not zero-limits (None ==
    # None is True) and not a TypeError crash from None > None.
    path = write_urdf(tmp_path, """
    <robot name="nolimit">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <origin xyz="1 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
    </robot>
    """)
    r = run("validate", str(path))
    assert r.returncode == 1
    assert "missing-limits" in r.stdout
    assert "zero-limits" not in r.stdout
    assert "Traceback" not in r.stdout


def test_validate_zero_limits_reported(tmp_path):
    path = write_urdf(tmp_path, """
    <robot name="zerolimit">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <origin xyz="1 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="0.5" upper="0.5" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    r = run("validate", str(path))
    assert r.returncode == 1
    assert "zero-limits" in r.stdout


def test_validate_inverted_limits_reported(tmp_path):
    path = write_urdf(tmp_path, """
    <robot name="invertedlimit">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <origin xyz="1 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="1.0" upper="-1.0" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    r = run("validate", str(path))
    assert r.returncode == 1
    assert "inverted-limits" in r.stdout


# ---------------------------------------------------------------------------
# --json
# ---------------------------------------------------------------------------

def test_validate_json_is_valid_json_with_matching_verdict(fixtures):
    r = run("validate", str(fixtures / "two_link.urdf"), "--json")
    assert r.returncode == 0, r.stderr
    payload = json.loads(r.stdout)
    assert payload["verdict"] == "PASS"
    assert payload["summary"]["dof"] == 2


def test_validate_json_fail_verdict_matches_exit_code(fixtures):
    r = run("validate", str(fixtures / "two_link.urdf"), "--expect-dof", "6", "--json")
    assert r.returncode == 1
    payload = json.loads(r.stdout)
    assert payload["verdict"] == "FAIL"
    assert any(f["code"] == "dof-mismatch" for f in payload["findings"])


def test_validate_json_includes_rdk_divergence_note(fixtures):
    r = run("validate", str(fixtures / "two_link.urdf"), "--json")
    payload = json.loads(r.stdout)
    assert "RDK" in payload["note"]


# ---------------------------------------------------------------------------
# .json input -> not-yet-implemented message
# ---------------------------------------------------------------------------

def test_validate_json_extension_not_yet_implemented(fixtures):
    r = run("validate", str(fixtures / "ur20.json"))
    assert r.returncode != 0
    combined = r.stdout + r.stderr
    assert "not" in combined.lower() and "implement" in combined.lower()


# ---------------------------------------------------------------------------
# unknown extension -> exit 2
# ---------------------------------------------------------------------------

def test_validate_unknown_extension_exits_2(tmp_path):
    path = tmp_path / "arm.xml"
    path.write_text("<robot/>")
    r = run("validate", str(path))
    assert r.returncode == 2


def test_validate_missing_arguments_exits_2():
    r = run()
    assert r.returncode == 2


# ---------------------------------------------------------------------------
# --help mentions the RDK-parity divergence
# ---------------------------------------------------------------------------

def test_help_mentions_rdk_divergence():
    r = run("validate", "--help")
    assert r.returncode == 0
    assert "RDK" in r.stdout


# ---------------------------------------------------------------------------
# The real user path: uv run --isolated, proving the PEP 723 header itself
# is correct (not just the dev venv this test suite normally runs under).
# ---------------------------------------------------------------------------

@pytest.mark.timeout(300)
@pytest.mark.skipif(shutil.which("uv") is None, reason="uv not installed")
def test_isolated_uv_run_resolves_pep723_dependencies(fixtures):
    r = subprocess.run(
        ["uv", "run", "--isolated", str(ARMKIT), "validate", str(fixtures / "ur20.urdf")],
        capture_output=True, text=True, timeout=290,
    )
    assert r.returncode == 0, f"stdout={r.stdout!r} stderr={r.stderr!r}"
    assert "6 actuated joints" in r.stdout
