"""CLI-level tests for armkit.py, invoked as a subprocess (sys.executable),
never imported -- this is what a real user runs, PEP 723 header and all.

One test (test_uv_run_resolves_pep723_dependencies_from_a_cold_cache)
additionally shells out via `uv run`, proving the PEP 723 dependency block
itself is correct: every other test here runs the script under the dev
venv's interpreter, which would happily resolve `_armkit`'s imports even if
armkit.py's own `dependencies = [...]` list were wrong or incomplete. That
test copies the script (and `_armkit/`) to a fresh path per run so uv keys
a cold resolution rather than reusing a previously-resolved environment --
see its docstring for why `--isolated` alone does not achieve this.
"""
from __future__ import annotations

import json
import os
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
# Fix 1: multi-leaf structure failures get a --tip remedy; other structure
# failures (cycle, disconnection, multiple roots) do not.
# ---------------------------------------------------------------------------

def test_validate_multi_leaf_structure_finding_suggests_tip(fixtures):
    # This is the single most common real-world failure: 30/84 real vendor
    # URDFs branch because a gripper ships attached to the arm. RDK's own
    # wording ("need exactly one end effector, have [...]") is kept
    # verbatim for parity, but nothing in it mentions the fix -- a user's
    # only realistic next move without a remedy is to read armkit's source.
    r = run("validate", str(fixtures / "two_leaf.urdf"))
    assert r.returncode == 1
    assert "need exactly one end effector, have ['finger_l', 'finger_r']" in r.stdout
    assert "--tip" in r.stdout
    # armkit has the leaf list in hand and should suggest a concrete one,
    # not just name the flag.
    assert "--tip finger_l" in r.stdout


def test_validate_cycle_structure_finding_has_no_tip_remedy(tmp_path):
    # A cycle isn't fixed by declaring an end effector -- there's no
    # coherent tree to walk regardless of which link is named tip. The
    # remedy must be attached ONLY to the multi-leaf diagnosis.
    path = write_urdf(tmp_path, """
    <robot name="cyclic">
      <link name="a"/><link name="b"/>
      <joint name="j1" type="revolute">
        <parent link="a"/><child link="b"/>
        <origin xyz="1 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="j2" type="revolute">
        <parent link="b"/><child link="a"/>
        <origin xyz="1 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    r = run("validate", str(path))
    assert r.returncode == 1
    assert "structure" in r.stdout
    assert "--tip" not in r.stdout


def test_validate_multi_leaf_remedy_resolves_the_model(fixtures):
    # The remedy this fix prints must actually work -- re-running with the
    # suggested --tip must pass.
    r = run("validate", str(fixtures / "two_leaf.urdf"), "--tip", "finger_l")
    assert r.returncode == 0, r.stdout
    assert "PASS" in r.stdout


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
# Fix 2: the RDK-parity note becomes specific (mesh references, a joint
# missing <origin>) or is absent entirely, rather than a fixed disclaimer
# printed on every invocation regardless of relevance.
# ---------------------------------------------------------------------------

def test_validate_warns_about_mesh_references_on_pass(fixtures):
    # meshed.urdf has one visual mesh (base.dae) and one collision mesh
    # (link1.stl) and is otherwise a valid, passing 1-DOF model.
    r = run("validate", str(fixtures / "meshed.urdf"))
    assert r.returncode == 0, r.stdout
    assert "mesh-references" in r.stdout
    assert "2 mesh" in r.stdout
    assert "PASS" in r.stdout


def test_validate_warns_about_missing_origin_on_pass(fixtures):
    # no_origin.urdf's sole joint has no <origin> element -- armkit
    # defaults it to identity (correct), but RDK v1.0.0 panics on this.
    r = run("validate", str(fixtures / "no_origin.urdf"))
    assert r.returncode == 0, r.stdout
    assert "missing-origin" in r.stdout
    assert "'j1'" in r.stdout
    assert "PASS" in r.stdout


def test_validate_prints_nothing_about_rdk_parity_when_neither_applies(fixtures):
    # two_link.urdf has no meshes and every joint declares <origin>.
    r = run("validate", str(fixtures / "two_link.urdf"))
    assert r.returncode == 0
    assert "mesh-references" not in r.stdout
    assert "missing-origin" not in r.stdout


def test_validate_drops_rdk_parity_findings_on_fail(fixtures):
    # meshed.urdf has mesh references, but forcing a DOF mismatch makes it
    # FAIL for an unrelated reason -- RDK parity is moot for a file that
    # didn't even pass armkit's own checks, so the mesh warning must not
    # appear alongside the real failure.
    r = run("validate", str(fixtures / "meshed.urdf"), "--expect-dof", "5")
    assert r.returncode == 1
    assert "dof-mismatch" in r.stdout
    assert "mesh-references" not in r.stdout


def test_validate_json_rdk_parity_present_on_pass_null_on_fail(fixtures):
    r_pass = run("validate", str(fixtures / "two_link.urdf"), "--json")
    payload_pass = json.loads(r_pass.stdout)
    assert payload_pass["rdk_parity"] == {"guaranteed": True, "reasons": []}

    r_mesh = run("validate", str(fixtures / "meshed.urdf"), "--json")
    payload_mesh = json.loads(r_mesh.stdout)
    assert payload_mesh["rdk_parity"]["guaranteed"] is False
    assert len(payload_mesh["rdk_parity"]["reasons"]) == 1

    r_fail = run("validate", str(fixtures / "meshed.urdf"), "--expect-dof", "5", "--json")
    payload_fail = json.loads(r_fail.stdout)
    assert payload_fail["rdk_parity"] is None


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


def test_validate_json_has_no_generic_note_field(fixtures):
    # Replaced by the per-file "rdk_parity" field (Fix 2) -- a fixed prose
    # blob is not something a consumer can act on; see
    # test_validate_json_rdk_parity_present_on_pass_null_on_fail.
    r = run("validate", str(fixtures / "two_link.urdf"), "--json")
    payload = json.loads(r.stdout)
    assert "note" not in payload


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
# The real user path: `uv run` resolving armkit.py's OWN PEP 723 header from
# a cold cache key, proving the dependency list itself is correct -- not
# just the dev venv this test suite normally runs under.
# ---------------------------------------------------------------------------

@pytest.mark.timeout(300)
@pytest.mark.skipif(shutil.which("uv") is None, reason="uv not installed")
def test_uv_run_resolves_pep723_dependencies_from_a_cold_cache(fixtures, tmp_path):
    # `--isolated` does NOT force a cold resolution: uv prints "--isolated is
    # a no-op for Python scripts with inline metadata, which always run in
    # isolation" -- it changes nothing. Worse, on a WARM cache uv keys its
    # resolved environment by script path, so re-running against the SAME
    # path after a dependency is deliberately removed from the header still
    # passes, reusing the stale environment (measured by hand: deleting
    # "viam-sdk>=0.79" from armkit.py's dependencies and re-running `uv run
    # --isolated armkit.py validate ...` against the original path exits 0
    # anyway). Copying the script to a fresh path each run gives uv a cache
    # key it has never resolved before, forcing it to actually read the
    # dependency block -- while still reusing uv's downloaded package cache
    # (no dependency version pins changed), so a warm run stays fast.
    #
    # `_armkit/` must come along too: armkit.py imports it via sys.path[0]
    # (the directory containing the running script), so copying armkit.py
    # alone breaks the import with an unrelated ModuleNotFoundError that has
    # nothing to do with the PEP 723 header this test exists to check.
    shutil.copytree(SCRIPTS / "_armkit", tmp_path / "_armkit")
    copy = tmp_path / "armkit.py"
    copy.write_text(ARMKIT.read_text())
    copy.chmod(0o755)
    r = subprocess.run(
        ["uv", "run", str(copy), "validate", str(fixtures / "ur20.urdf")],
        capture_output=True, text=True, timeout=290,
    )
    assert r.returncode == 0, f"stdout={r.stdout!r} stderr={r.stderr!r}"
    assert "6 actuated joints" in r.stdout


# ---------------------------------------------------------------------------
# Native library load failure -> exit 2 with a platform message, not exit 1
# with "this is an armkit bug"
# ---------------------------------------------------------------------------

def test_native_library_load_failure_exits_2_with_platform_message(fixtures, tmp_path):
    """Simulates viam-sdk's native library failing to load (e.g. an old
    glibc under a manylinux wheel), WITHOUT touching the real, working
    libviam_rust_utils on this machine: a sitecustomize.py shim placed on
    PYTHONPATH monkeypatches `viam._native.load_native_lib` to raise OSError
    before armkit ever imports the real thing.

    This exit-2 path is fragile and worth pinning explicitly, not just
    trusting the code once written -- three facts have to hold together for
    it to work at all, each discovered by tracing the actual call chain
    rather than assumed:

    1. `viam._native.load_native_lib` raises OSError (its own docstring says
       so), not ValueError.
    2. The native library load is LAZY: `viam.spatialmath._ffi.lib()` is
       called per rotation conversion (from EulerAngles/AxisAngle
       constructors), not at import time. So the OSError does not surface
       when armkit.py imports `_armkit.urdf` at startup -- it surfaces
       later, from INSIDE `_armkit.urdf.parse_urdf()`, the first time a
       joint with an <origin> element is parsed and rpy_to_matrix() runs.
    3. `parse_urdf`'s own broad `except Exception as e: raise ValueError(...)`
       catch-all (a DELIBERATE safety net for failure modes it doesn't
       anticipate by name -- see urdf.py) is what turns that OSError into a
       ValueError carrying the original text. armkit.py's
       `if "libviam_rust_utils" in str(e)` check then matches against that
       wrapped text.

    If a future change narrows urdf.py's catch-all to `except ValueError`
    only (removing the safety net), the OSError from step 2 would propagate
    OUT of parse_urdf uncaught -- armkit.py's `except ValueError` wouldn't
    catch it either, and the CLI would crash with a raw traceback instead of
    exiting 2 with a helpful message. This test is what would catch that
    regression; it is not just documentation of how the path works today.
    """
    shim_dir = tmp_path / "shim"
    shim_dir.mkdir()
    (shim_dir / "sitecustomize.py").write_text(
        "import viam._native\n"
        "\n"
        "def _boom(*a, **kw):\n"
        "    raise OSError('simulated: cannot dlopen libviam_rust_utils (test shim, not real)')\n"
        "\n"
        "viam._native.load_native_lib = _boom\n"
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = str(shim_dir) + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")

    r = subprocess.run(
        [sys.executable, str(ARMKIT), "validate", str(fixtures / "two_link.urdf")],
        capture_output=True, text=True, env=env,
    )
    assert r.returncode == 2, f"stdout={r.stdout!r} stderr={r.stderr!r}"
    assert "Linux" in r.stderr and "macOS" in r.stderr and "Windows" in r.stderr
