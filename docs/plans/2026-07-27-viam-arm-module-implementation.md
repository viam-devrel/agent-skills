# viam-arm-module Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the `viam-arm-module` skill — a phase-gated agentic workflow for authoring Viam arm modules in Go, C++, and Python, backed by an executable kinematics toolkit.

**Architecture:** Three workstreams. (A) `armkit`, a `uv`-run Python toolkit that validates kinematics files, inspects and simplifies meshes, and runs live-machine checks — built TDD. (B) `references/`, source-verified knowledge extracted from RDK, the three SDKs, and published arm modules. (C) `SKILL.md`, the phase machine that drives an agent through the workflow, plus marketplace registration.

**Tech Stack:** Python 3.11+ run via `uv` with PEP 723 inline metadata; `numpy` (FK), `trimesh`+`pycollada` (mesh I/O incl. COLLADA), `viam-sdk` (live checks); `pytest` for development.

**Spec:** `docs/plans/2026-07-27-viam-arm-module-design.md`

---

## Scope note

Workstreams A and B are independent and can proceed in parallel. C depends on both. A is
real software with tests; B is documentation verified against source; C is assembly.

## Execution order — revised after A3b

The original order (all of A, then B, then C) was written before we had velocity data.
Four tasks consumed ~28 commits and a dozen review rounds; 21 tasks remain. The review
depth is earning its cost — fourteen findings, all real, several that would have shipped
silently wrong FK — but the sizing assumed otherwise.

**Revised to ship a thin vertical slice first**, so there is a usable skill with one real
gate before the long tail:

| # | Tasks | Outcome |
|---|---|---|
| 1 | A0 (probe), **A4** (FK), **A7** (validate CLI) | Phase 1's gate is real and runnable |
| 2 | **C1** (SKILL.md), **C2** (registration) | A working, installable skill |
| 3 | A6, A8, A9 | Mesh inspection, `meshes`/`simplify`/`convert` |
| 4 | **B4 `driver-reference.md`** (pulled forward) | How a validated file becomes a module's `GetKinematics` |
| 5 | A5a/A5b/A5c (SVA, split) | Second parser |
| 6 | A10, A11, A12 | Cross-check and live-machine checks |
| 7 | B1–B3, B5–B8 | Remaining references |

**Reordered after the slice-acceptance review, on evidence.** All 84 real vendor URDFs on
this machine were run through the developer flow: **52 (61.9%) PASS as-is**, 29 (34.5%)
need the documented URDF edit, 2 have no joints, 1 is parse-rejected.

- **Meshes stay first.** Mesh resolution is the largest remaining correctness gap between
  an armkit PASS and RDK actually loading the file — confirmed on `cr5_robot.urdf` and
  both mycobot gripper URDFs. It is also the only specced-but-unimplemented findings-table
  entry, and `urdf.py` already collects the reference lists A6 needs, so A6 consumes
  existing data rather than building a subsystem.
- **`driver-reference.md` moves ahead of SVA.** A developer finishes Phase 1 with a
  validated URDF and then falls off a cliff: nothing says how that file becomes the
  module's `GetKinematics` response. It is the only place a reader needs something no
  skill covers, and Phases 2–3 — where the actual work of building an arm module happens —
  have neither tooling nor reference material.
- **SVA drops back.** The enforced opinion is already "URDF as-is when the vendor ships
  one," and 62% of vendor arms ship URDF that passes. SVA serves on-ramp 3 (vendor
  protocol only, hand-authored kinematics), the rarest of the three.

Sequenced by *where a developer gets stuck*, which is what the acceptance data measures.

**Why B moves last:** the references describe behavior. Written now they would describe
*intended* behavior; written after the toolkit they describe *verified* behavior.

**Known risk, unmitigated so far:** Workstream B is nine documents written from reading
SDK source. That is precisely the method that produced this session's worst defects — the
522 mm orientation error, the unpassable mesh assertions, the self-loop folding rule. The
Go probe grounds RDK claims but does nothing for Python or C++ SDK claims, which B is full
of. Before starting B, decide how its claims get verified; "every claim cites file:line"
is the current rule and is not sufficient alone, since a citation can be real and the
conclusion drawn from it still wrong.

**Scope adjustments within the slice:** A7's findings table includes `unresolved-mesh` and
`heavy-mesh`, which depend on A6. Ship A7 without them and add them when A6 lands, rather
than pulling A6 forward. C1's `SKILL.md` describes all seven workflow phases — that is the
knowledge deliverable and is correct — but must state plainly which gates have tooling
today and which are manual.

### Task A0: Promote the Go probe into the repo

**Files:** create `tools/rdkprobe/{main.go,go.mod,go.sum,README.md}` — at the **repo root**,
deliberately *not* under `skills/`. Anything under `skills/viam-arm-module/` ships to every
user who installs the skill via the plugin symlink, and this is a 91 KB Go module no end
user runs.

The probe is now load-bearing: it is the parity oracle *and* A4's FK oracle, and it
currently lives in a session-scoped scratchpad. If it vanishes, the next person re-derives
parity by reading RDK — the method that produced most of this session's defects.

- Move `main.go` verbatim from the scratchpad (supports `--at q1,q2,...`, printing RDK's
  `Transform()` point and quaternion at 9 decimals).
- `README.md` states what it is for, how to run it, that it pins RDK v1.0.0, and that
  `tests/test_parity.py`'s literals came from it.
- **Not part of the skill's published surface** — it is a development tool needing a Go
  toolchain, not something end users run. Say so in the README.

### Parity drift

`tests/test_parity.py` pins RDK v1.0.0 with date-stamped literals. When RDK bumps: re-run
the probe over the parity fixtures, diff against the recorded literals, and re-run the
corpus scan expecting 102 accepted / 30 end-effector / 22 jointless / 1 undeclared-link
across the 155 real files. A changed verdict is a **finding, not a test to update** — it
means RDK's behavior moved and armkit must follow deliberately.

**The same rule applies to `viam-sdk` upgrades.** `transforms.py`'s `order="F"` reshape
depends on an *undocumented* buffer layout in a vendored native library whose own docstring
claims the opposite. An SDK upgrade that "fixes" the buffer order is the same class of
event as an RDK verdict change, and must be treated as a finding rather than a test to
update. `test_transforms.py` pins the layout; if it fails after an upgrade, investigate
before touching it.

**Corpus-scan trap:** the topology checks are lazy properties. A scan that only calls
`parse_urdf` without evaluating `m.dof` never runs them, and 52 rejects silently
reclassify as accepts.

> **Second corpus-scan trap: a broken command produces a plausible zero.** During A6
> review, a reviewer reported "no mesh assets exist anywhere under `~/src`" — a false
> negative that, if believed, would have discredited A6's measured triangle distribution
> and restored an invented threshold. Root cause: `timeout` is **not installed on macOS**
> (it ships as `gtimeout` via coreutils), so the command never launched; `2>/dev/null`
> swallowed the `command not found`; and `wc -l` counted zero lines from a process that
> never started.
>
> Nothing in that output distinguishes *"searched and found nothing"* from *"never
> searched."* This matters for every counting operation in this plan — corpus buckets,
> mesh statistics, drift checks — because they are all measurements where **absence is the
> evidence**, and absence is exactly what a broken command also produces.
>
> Three rules, adopted:
> 1. **Positive-control any null result.** Run the same search against something known to
>    exist. A `find ~/src -name "*.urdf" | wc -l` alongside would have exposed the zero
>    instantly.
> 2. **Do not `2>/dev/null` a command you have not already seen succeed.** Suppressing
>    stderr is noise reduction for a known-good command, not a first run.
> 3. **Report absence with the command that produced it**, and hedge it. "This `find`
>    returned nothing, not positively controlled" invites the check that "there are no
>    meshes" forecloses.
>
> A measurement that contradicts established context should make you suspect the
> measurement first.

> **The corpus is not currently reproducible — fix this.** The 102/30/22/1 figures come
> from ad-hoc scans over "every `.urdf` under `~/src` and `~/go/pkg/mod/go.viam.com`" on
> one machine. No manifest or script was preserved, so the numbers above cannot be
> re-derived by anyone else, which makes the drift procedure unrunnable as written.
> Discovered during A4b when a re-scan could only find 75 of the files.
>
> **Add `tools/corpus_scan.py`** (alongside `rdkprobe`, not shipped with the skill): walk
> a configurable list of roots, parse each `.urdf`, force `m.dof`, and print per-bucket
> counts plus the file list per bucket. Commit a `corpus-manifest.txt` recording the exact
> files and their verdicts at the time of capture, so a future scan diffs against a real
> baseline rather than a remembered number. Machine-specific paths are fine as long as the
> manifest says which machine and when.

## File structure

```
skills/viam-arm-module/
  SKILL.md                              # C: the phase machine
  references/
    kinematics-reference.md             # B
    meshes-3d-reference.md              # B
    driver-reference.md                 # B
    operations-reference.md             # B
    motion-delegation-reference.md      # B
    simulated-arm-reference.md          # B
    packaging-reference.md              # B
    cheatsheet.md                       # B
    sdk-gaps.md                         # B
  scripts/
    armkit.py                           # A: offline entry (PEP 723)
    armkit_live.py                      # A: live entry (PEP 723)
    pyproject.toml                      # A: dev/test deps only
    _armkit/
      __init__.py
      urdf.py                           # A: URDF parsing
      sva.py                            # A: SVA/DH model JSON parsing
      model.py                          # A: common parsed representation
      fk.py                             # A: forward kinematics
      meshes.py                         # A: mesh inspection/simplification
      live.py                           # A: viam-sdk helpers (lazy import)
    tests/
      conftest.py
      fixtures/
        two_link.urdf                   # synthetic, hand-computable
        ur20.urdf                       # real, from urdf-to-sva-converter
        ur20.json                       # real, SVA
        ur5e.json                       # real, from RDK
      test_urdf.py  test_sva.py  test_fk.py  test_meshes.py  test_cli.py
plugins/viam-arm-module/
  .claude-plugin/plugin.json            # C
  skills/viam-arm-module -> symlink     # C
```

**Key boundary:** `_armkit/model.py` defines one parsed representation (`KinematicModel`,
`Joint`, `Link`). `urdf.py` and `sva.py` both produce it; `fk.py`, `meshes.py`, and every
CLI subcommand consume only it. Neither parser is imported by anything except its own
tests and the CLI dispatcher. This is what stops the three-copies-of-URDF-parsing problem
the spec calls out.

**Shared math lives in `_armkit/transforms.py`** — `rpy_to_matrix`, `axis_angle_to_matrix`,
and `M_TO_MM`. Both parsers and `fk.py` import from it. This exists because `sva.py` needs
`rpy_to_matrix` for `euler_angles`, and importing it from `urdf.py` would violate the
boundary above.

**`transforms.py` is backed by `viam.spatialmath`** (task A4b). That module is a ctypes
binding to `libviam_rust_utils` — the same native library behind RDK — so its conversions
are canonical rather than a Python reimplementation. This matters most for A5, whose five
orientation types include Viam's own orientation-vector format.

> **Layout trap — `RotationMatrix.elements` is COLUMN-major.** The class docstring says
> `elements[3*row + col]` (row-major), and that is wrong: the buffer is nalgebra's, which
> is column-major. Measured against the verified `rpy_to_matrix(0.1, 0.2, 0.3)`:
>
> | interpretation | max abs diff |
> |---|---|
> | `np.array(elements).reshape(3, 3)` | **0.565** |
> | `np.array(elements).reshape(3, 3, order="F")` | **1.1e-16** |
>
> Read as documented you get the transpose — for a rotation matrix, its inverse — which
> silently inverts every rotation with no exception raised. Quaternion comparisons do not
> expose this, because quaternions carry no layout. **Pin the layout with a dedicated
> test** so a future SDK change that "fixes" the buffer order fails loudly instead of
> silently inverting poses.
>
> **Testing lesson, proven by mutation during A4b — carry this into A5.** A 180° rotation
> matrix is symmetric (`R = 2·axis·axisᵀ − I`), so reshaping its flat buffer row-major or
> column-major produces the *same* matrix. Under a deliberately wrong `order="C"`, **every
> exact-`±π` case still passed** while every non-exact case failed. A rotation test suite
> that checks only at `±π` is blind to a transpose. Always include angles *near* but not
> at `±π` (e.g. `π − 1e-6`), and prefer asymmetric rotations when pinning any layout.
> Orthonormality and `det == 1` are equally blind — a transposed rotation is still a
> rotation.
>
> **Root cause, confirmed in source (not just measured):** `rust-utils`'
> `viam_rotation_matrix_from_quaternion` returns `to_raw_pointer(&rot)` where
> `rot: Rotation3<f64>` — nalgebra, which is column-major. Meanwhile RDK's Go convention
> really is row-major: `rotationMatrix.go:88` defines `At(row, col) = mat[3*row+col]`, and
> `Mul` computes `X = mat[0]*v.X + mat[1]*v.Y + mat[2]*v.Z`. So the SDK docstring's claim
> that the raw buffer coincides with RDK's row-major convention is **wrong**; its golden
> parity tests likely pass because they compare via quaternion, which carries no layout.
> Verified against released `viam-sdk 0.79.2` (FFI-backed) — not a stale-checkout artifact.
> **This belongs in `sdk-gaps.md` (task B1) as a genuine, filable SDK bug.**

**Dependency cost of `viam-sdk`, measured during A4b review and accepted deliberately:**
13 → 25 packages, venv 71 MB (the `viam` package alone is 20.1 MB), pulling the full gRPC
stack plus `pymongo` and `dnspython` onto the offline `validate` path. First-run PEP 723
download goes from ~20 MB to ~90 MB, cached thereafter. Recorded so a future reader knows
this was weighed rather than drifted into.

**Platform support** (resolved via `uv pip compile --python-platform`): Linux glibc and
musl on x86_64/aarch64/armv7 (Raspberry Pi, Alpine), macOS x86_64/arm64, Windows x64.
**Windows ARM64 fails** — no matching wheel and no sdist, so it fails at *dependency
resolution before Python starts*, which armkit cannot catch or explain. There is no
degrade path by design: `transforms.py` imports `viam.spatialmath` at module scope and
`urdf.py`'s `_origin()` calls `rpy_to_matrix` per joint, so no viam-sdk means no parsing
at all, not even topology checks.

**Do NOT use the `referenceframe` FFI in `rust-utils`.** The local branch
`design/referenceframe-fk-parsing` implements `rf_model_from_bytes`, `rf_model_dof`,
`rf_model_transform`, `rf_model_geometries_at`, and `rf_model_limits` with URDF parsing and
RDK golden-fixture parity tests — substantially what armkit reimplements in Python. It is
an **experimental workstream, unreleased and not expected to merge upstream** (confirmed by
the author). The shipped `libviam_rust_utils.dylib` exports zero `rf_model` symbols.
armkit keeps its own Python parser and FK. Recorded so this overlap is not rediscovered
and re-litigated. Note the distinction from `viam.spatialmath`, which *is* merged,
released, and correct to depend on.

## Error-handling contract

**This applies to every parser and every CLI subcommand. It was missing from the first
draft of this plan and was added after Task A3 review surfaced five escaping exception
types.**

`armkit` is a validator. **Malformed input is the expected input, not the exception.** A
traceback is never an acceptable response to a bad kinematics file — it defeats the CLI's
exit-code contract, which is the only thing an agent can depend on.

Rules:

1. **`parse_urdf` and `parse_sva` raise `ValueError` and nothing else.** Wrap the parse
   body so `AttributeError`, `TypeError`, `IndexError`, `ET.ParseError`, and `OSError` all
   leave as `ValueError`. Measured during A3 review: a missing `<parent>` gives
   `AttributeError`, malformed XML gives `ET.ParseError` (whose MRO is `SyntaxError`, *not*
   `ValueError`), and a nonexistent path gives `FileNotFoundError` — none catchable by a
   single `except ValueError`.
2. **Every parse error names the file and the offending element.** Bare messages like
   `could not convert string to float: 'a'` are useless to someone staring at a 400-line
   URDF. The parser has the path and the joint name in hand; include them.
3. **Reject structurally unusable input at parse time rather than passing it through.**
   Unknown or missing joint `type`, an axis without exactly three components, a link or
   joint without a name. A model that parses "successfully" into 0 actuated joints and
   reports PASS is worse than one that fails loudly.
4. **CLI subcommands catch `ValueError` only**, and convert it to an error finding with
   exit 1. If a subcommand needs a broader catch, the parser is not holding up its end.
5. **Do not narrow `parse_urdf`'s `except Exception` safety net.** A non-obvious contract
   depends on it: `viam-sdk`'s `load_native_lib` raises `OSError`, and the load is *lazy*
   (`_ffi.lib()` runs per-conversion, not at import), so a native-library failure surfaces
   inside `parse_urdf`. The catch-all converts it to a `ValueError` carrying the original
   text, which is what `armkit.py` matches on to emit its platform message and exit 2.
   Narrow the catch and that path silently regresses to a traceback. Verified during A7
   review with a `sitecustomize` shim making `load_native_lib` raise.
6. **Every `pytest.raises(ValueError)` MUST supply a `match=` argument.** This convention is
   load-bearing, not style. Because the parsers convert internal bugs into the same
   exception type as user-input errors, a bare `pytest.raises(ValueError)` will pass when
   the parser crashes on its own bug — a green test proving nothing. Verified during A3
   review: an injected internal `AttributeError` surfaces as
   `ValueError: <path>: failed to parse URDF (AttributeError: ...)`, indistinguishable
   from a real finding without a `match=`. The catch-all is otherwise well-sized — it
   preserves `__cause__` via `from e`, names the original type in the message, and leaves
   `KeyboardInterrupt`/`SystemExit` alone, since those derive from `BaseException`.

## Conventions

- **Running:** `uv run skills/viam-arm-module/scripts/armkit.py <subcommand>`. PEP 723
  headers make this work with no install step. This is a new convention for the repo,
  which currently uses stdlib-only scripts with lazy imports — the design doc explains why.
- **Testing:** `cd skills/viam-arm-module/scripts && uv run pytest`. The `pyproject.toml`
  exists for development only; end users never need it. Run from *inside* `scripts/` —
  the `uv run --project <dir> pytest` form invoked from the repo root does **not** load
  `[tool.pytest.ini_options]` (pytest reports no configfile), so `testpaths` is ignored,
  collection falls back to scanning the cwd, **and `pythonpath` is ignored too — `_armkit`
  will not import**, giving `ModuleNotFoundError`. That failure is loud and points at the
  cause, which is the intended behavior. Verified during Task A1 review.
- **`uv.lock` is committed** alongside `pyproject.toml`, pinning the dev/test environment.
  It has no effect on end users, who run the scripts via `uv run` + PEP 723. For a toolkit
  whose correctness rests on numpy numerics, a reproducible dev environment is worth the
  file. Ratified during Task A1 review.
- **Every script** carries a "Why this exists" docstring naming the gap it fills, matching
  `skills/local-viam-server/machine_up.py`.
- **Units:** URDF is meters/radians. The parsed `KinematicModel` is **always millimeters
  and radians internally**; conversion happens at parse and at display. Every test asserts
  units explicitly.
- **SVA units are not uniform — this is a trap.** Translations are millimeters and joint
  `min`/`max` are degrees, unconditionally. But **orientation angular units depend on
  `orientation.type`** (`spatialmath/orientation_json.go` defines five):

  | `type` | angular unit |
  |---|---|
  | `ov_degrees` | `th` in degrees |
  | `ov_radians` | radians |
  | `euler_angles` | **radians**, RPY in the same `Rz·Ry·Rx` order as URDF |
  | `axis_angles` | radians |
  | `quaternion` | unitless |

  Do not assume "SVA means degrees." `ur5e.json` uses `ov_degrees` throughout; `ur20.json`
  uses `euler_angles` throughout. Measured during Task A1 review: reading `ur20.json`'s
  `euler_angles` as degrees puts the tip **522 mm** away from the same arm's URDF; reading
  them as radians agrees to **0.000 mm**.

---

# Workstream A — armkit

### Task A1: Scaffold and fixtures

**Files:**
- Create: `skills/viam-arm-module/scripts/pyproject.toml`
- Create: `skills/viam-arm-module/scripts/uv.lock` (generated by Step 5)
- Create: `skills/viam-arm-module/scripts/_armkit/__init__.py`
- Create: `skills/viam-arm-module/scripts/tests/conftest.py`
- Create: `skills/viam-arm-module/scripts/tests/fixtures/two_link.urdf`
- Create: `skills/viam-arm-module/scripts/tests/fixtures/meshed.urdf`
- Modify: `.gitignore` (add `.venv/` and `.pytest_cache/`)

**Fixture coverage note.** Across the real fixtures every joint is `revolute` or `fixed`.
No fixture exercises `continuous` (A7's `continuous-joint` warning) or `prismatic`
(`fk.py`'s prismatic branch and `urdf.py`'s millimeter limit conversion). A4 and A7 must
build those cases inline with `tmp_path` or those paths ship untested.

- [ ] **Step 1: Create the dev project file**

```toml
# skills/viam-arm-module/scripts/pyproject.toml
# Development/test only. End users run the scripts via `uv run` and PEP 723 headers.
[project]
name = "armkit-dev"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = ["numpy>=1.26", "trimesh>=4.0", "pycollada>=0.8"]

[dependency-groups]
dev = ["pytest>=8.0"]

[tool.pytest.ini_options]
testpaths = ["tests"]
```

- [ ] **Step 2: Create the synthetic fixture**

Two revolute joints, 1 m apart on X, rotating about Z. Chosen so FK is hand-computable:
at `[0, 0]` the tip is at (2000, 0, 0) mm; at `[pi/2, 0]` it is at (0, 2000, 0) mm.

```xml
<!-- skills/viam-arm-module/scripts/tests/fixtures/two_link.urdf -->
<robot name="two_link">
  <link name="base"/>
  <link name="link1"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="tip"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1"/>
  </joint>
</robot>
```

- [ ] **Step 3: Copy real fixtures**

```bash
cd /Users/nick.hehr/src/agent-skills/skills/viam-arm-module/scripts/tests/fixtures
cp ~/src/urdf-to-sva-converter/ur20.urdf .
cp ~/src/urdf-to-sva-converter/ur20.json .
cp ~/go/pkg/mod/go.viam.com/rdk@v1.0.0/components/arm/sim/kinematics/ur5e.json .
chmod u+w ur5e.json   # Go module cache files are read-only
```

- [ ] **Step 4: Create conftest with a fixtures path helper**

```python
# skills/viam-arm-module/scripts/tests/conftest.py
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

@pytest.fixture
def fixtures() -> Path:
    return Path(__file__).parent / "fixtures"
```

- [ ] **Step 5: Verify the toolchain runs**

Run: `cd skills/viam-arm-module/scripts && uv run pytest -q`
Expected: `no tests ran` (exit 5). This confirms `uv` resolves deps and finds the project.

- [ ] **Step 6: Commit**

```bash
git add skills/viam-arm-module/scripts
git commit -m "feat(armkit): scaffold project, fixtures, and test harness"
```

---

### Task A2: Common model representation

**Files:**
- Create: `skills/viam-arm-module/scripts/_armkit/model.py`
- Test: `skills/viam-arm-module/scripts/tests/test_model.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/test_model.py
import numpy as np
from _armkit.model import Joint, KinematicModel

def test_actuated_joints_excludes_fixed():
    m = KinematicModel(
        name="t",
        joints=[
            Joint("j1", "revolute", "base", "l1", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
            Joint("f1", "fixed", "l1", "l2", np.eye(4), None, None, None),
            Joint("j2", "revolute", "l2", "tip", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    assert [j.name for j in m.actuated_joints] == ["j1", "j2"]
    assert m.dof == 2

def test_chain_orders_base_to_tip():
    m = KinematicModel(
        name="t",
        joints=[
            Joint("j2", "revolute", "l1", "tip", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
            Joint("j1", "revolute", "base", "l1", np.eye(4), np.array([0, 0, 1.0]), -1.0, 1.0),
        ],
        links={}, source_format="urdf", source_path="t.urdf",
    )
    assert [j.name for j in m.chain()] == ["j1", "j2"]
    assert m.base_link == "base"
    assert m.tip_link == "tip"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd skills/viam-arm-module/scripts && uv run pytest tests/test_model.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named '_armkit.model'`

- [ ] **Step 3: Implement**

```python
# _armkit/model.py
"""Common parsed representation for kinematic models.

Both the URDF and SVA parsers produce a KinematicModel; every consumer
(FK, mesh inspection, every CLI subcommand) reads only this. Keeping one
representation is what stops URDF parsing from being reimplemented per
subcommand.

Units: translations are ALWAYS millimeters, angles ALWAYS radians.
Conversion from URDF (meters) or SVA (degrees) happens at parse time.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

ACTUATED_TYPES = {"revolute", "continuous", "prismatic"}


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray          # 4x4 homogeneous, translation in mm
    axis: np.ndarray | None     # unit 3-vector, None for fixed
    lower: float | None         # radians (or mm for prismatic)
    upper: float | None

    @property
    def actuated(self) -> bool:
        return self.type in ACTUATED_TYPES


@dataclass
class Link:
    name: str
    collision_meshes: list[str] = field(default_factory=list)
    visual_meshes: list[str] = field(default_factory=list)
    collision_primitives: list[dict] = field(default_factory=list)


@dataclass
class KinematicModel:
    name: str
    joints: list[Joint]
    links: dict[str, Link]
    source_format: str          # "urdf" | "sva" | "dh"
    source_path: str

    @property
    def actuated_joints(self) -> list[Joint]:
        return [j for j in self.chain() if j.actuated]

    @property
    def dof(self) -> int:
        return len(self.actuated_joints)

    def chain(self) -> list[Joint]:
        """Joints ordered base to tip. Raises on branching or disconnection."""
        by_parent = {}
        for j in self.joints:
            by_parent.setdefault(j.parent, []).append(j)
        children = {j.child for j in self.joints}
        roots = [j.parent for j in self.joints if j.parent not in children]
        if not roots:
            raise ValueError("kinematic model has no root link (cycle?)")
        if len(set(roots)) > 1:
            raise ValueError(f"multiple root links: {sorted(set(roots))}")

        ordered, current = [], roots[0]
        while current in by_parent:
            branches = by_parent[current]
            if len(branches) > 1:
                names = sorted(b.name for b in branches)
                raise ValueError(f"branching at link {current!r}: {names}")
            ordered.append(branches[0])
            current = branches[0].child
        if len(ordered) != len(self.joints):
            raise ValueError("disconnected joints in kinematic model")
        return ordered

    @property
    def base_link(self) -> str:
        return self.chain()[0].parent

    @property
    def tip_link(self) -> str:
        return self.chain()[-1].child
```

- [ ] **Step 4: Run tests**

Run: `cd skills/viam-arm-module/scripts && uv run pytest tests/test_model.py -q`
Expected: PASS (2 passed)

- [ ] **Step 5: Commit**

```bash
git add skills/viam-arm-module/scripts/_armkit/model.py skills/viam-arm-module/scripts/tests/test_model.py
git commit -m "feat(armkit): common kinematic model representation"
```

---

### Task A3: URDF parser — joints and links

**Files:**
- Create: `skills/viam-arm-module/scripts/_armkit/urdf.py`
- Test: `skills/viam-arm-module/scripts/tests/test_urdf.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/test_urdf.py
import numpy as np
from _armkit.urdf import parse_urdf

def test_parses_two_link(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    assert m.name == "two_link"
    assert m.dof == 2
    assert [j.name for j in m.chain()] == ["j1", "j2"]
    assert m.base_link == "base"
    assert m.tip_link == "tip"

def test_converts_meters_to_millimeters(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    # URDF says xyz="1 0 0"; internal representation is mm.
    assert np.isclose(m.chain()[0].origin[0, 3], 1000.0)

def test_limits_stay_in_radians(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    assert np.isclose(m.chain()[0].lower, -3.14159)

def test_parses_real_ur20(fixtures):
    m = parse_urdf(fixtures / "ur20.urdf")
    assert m.dof == 6
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd skills/viam-arm-module/scripts && uv run pytest tests/test_urdf.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named '_armkit.urdf'`

- [ ] **Step 3: Implement**

```python
# _armkit/urdf.py
"""URDF parsing into the common KinematicModel.

Why this exists: nothing in the Viam ecosystem validates a kinematics
file. URDF is the format Viam recommends reusing when a manufacturer
ships one, and it is the only practical way to carry mesh collision
geometry -- so it is the format this toolkit reads first.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .model import Joint, KinematicModel, Link

M_TO_MM = 1000.0


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw -> 3x3 rotation (Rz @ Ry @ Rx)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _origin(elem: ET.Element | None) -> np.ndarray:
    t = np.eye(4)
    if elem is None:
        return t
    xyz = [float(v) for v in elem.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in elem.get("rpy", "0 0 0").split()]
    t[:3, :3] = rpy_to_matrix(*rpy)
    t[:3, 3] = np.array(xyz) * M_TO_MM
    return t


def parse_urdf(path: str | Path) -> KinematicModel:
    path = Path(path)
    root = ET.parse(path).getroot()

    links: dict[str, Link] = {}
    for le in root.findall("link"):
        links[le.get("name")] = Link(name=le.get("name"))

    joints: list[Joint] = []
    for je in root.findall("joint"):
        jtype = je.get("type")
        axis_elem = je.find("axis")
        axis = None
        if jtype != "fixed":
            raw = [float(v) for v in (axis_elem.get("xyz") if axis_elem is not None else "1 0 0").split()]
            vec = np.array(raw, dtype=float)
            norm = np.linalg.norm(vec)
            if norm == 0:
                raise ValueError(f"joint {je.get('name')!r} has a zero-length axis")
            axis = vec / norm

        limit = je.find("limit")
        lower = upper = None
        if jtype == "continuous":
            lower, upper = -np.inf, np.inf
        elif limit is not None:
            lower = float(limit.get("lower", 0.0))
            upper = float(limit.get("upper", 0.0))
            if jtype == "prismatic":
                lower, upper = lower * M_TO_MM, upper * M_TO_MM

        joints.append(Joint(
            name=je.get("name"), type=jtype,
            parent=je.find("parent").get("link"),
            child=je.find("child").get("link"),
            origin=_origin(je.find("origin")),
            axis=axis, lower=lower, upper=upper,
        ))

    return KinematicModel(
        name=root.get("name", path.stem), joints=joints, links=links,
        source_format="urdf", source_path=str(path),
    )
```

- [ ] **Step 4: Run tests**

Run: `cd skills/viam-arm-module/scripts && uv run pytest tests/test_urdf.py -q`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add skills/viam-arm-module/scripts/_armkit/urdf.py skills/viam-arm-module/scripts/tests/test_urdf.py
git commit -m "feat(armkit): URDF joint and link parsing"
```

---

### Task A4: Forward kinematics

**Files:**
- Create: `skills/viam-arm-module/scripts/_armkit/fk.py`
- Modify: `skills/viam-arm-module/scripts/_armkit/transforms.py` (add `axis_angle_to_matrix`)
- Test: `skills/viam-arm-module/scripts/tests/test_fk.py`

This is the task that closes the gap the spec names: FK exists only in Go's
`referenceframe`, so Python and C++ module authors have no way to check their own model.

**`axis_angle_to_matrix` goes in `transforms.py`, not `fk.py`** — it belongs beside
`rpy_to_matrix` rather than splitting the rotation helpers across two modules. `fk.py`
imports it. Give it its own unit test (orthonormality and `det == 1` over a spread of
axes and angles); a rotation helper that only gets exercised through FK is one whose
failures surface as confusing pose errors rather than as a failing unit test.

**A3b is complete, and it did most of A4's design work.** `KinematicModel.joint_values(inputs)`
takes the flat BFS-ordered input vector and returns a value per joint name with mimics
already derived, so `link_poses` needs no ordering knowledge, no name→index map, and no
mimic handling. The reviewer wrote and verified this implementation during A3b review:

```python
def link_poses(model, inputs):
    vals = model.joint_values(inputs)
    chain = model.chain()
    poses = {chain[0].parent: np.eye(4)}
    current = np.eye(4)
    for j in chain:
        current = current @ joint_transform(j, vals[j.name])
        poses[j.child] = current.copy()
    return poses
```

**Use the pose oracle, not hand-computed values.** The Go probe supports
`go run . <file> --at q1,q2,...`, printing RDK's `Transform()` point and quaternion at 9
decimals. This plan's hand-computed FK assertions were wrong twice before review caught
them; the oracle removes that whole failure class.

Verified against RDK v1.0.0 during A3b review — use these directly as test assertions:

| model | inputs | RDK point (mm) | RDK quaternion |
|---|---|---|---|
| `test_mimic_serial.urdf` | `0.1, -0.4` | `[-19.568679001, 0.0, 295.034065440]` | `[0.98006658, 0, -0.19866933, 0]` |
| `ur20.urdf` | `0.1,-0.4,0.7,0.2,-0.3,0.5` | `[-1332.073428033, -483.810901083, 238.695348151]` | `[0.59077555, 0.62428142, -0.19857677, 0.47098218]` |

The `ur20` point has now been triangulated three ways: the A1 fixture review computed it
independently, A3b's `link_poses` draft reproduced it, and the probe reports it from RDK.

**Confirm the quaternion component order from the probe's own source before asserting on
it** — RDK's `quat.Number` has `Real, Imag, Jmag, Kmag`, so the printed order is almost
certainly `(w, x, y, z)`, but check rather than assume. Comparing points alone is a valid
first step if orientation conventions prove fiddly; comparing both is the goal.

- [ ] **Step 1: Write the failing test**

The synthetic fixture was designed to make these hand-computable.

```python
# tests/test_fk.py
import numpy as np
import pytest
from _armkit.fk import forward_kinematics, link_poses
from _armkit.urdf import parse_urdf

def test_zero_config_is_fully_extended(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    pose = forward_kinematics(m, [0.0, 0.0])
    assert np.allclose(pose[:3, 3], [2000.0, 0.0, 0.0])

def test_first_joint_swings_tip(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    # tip = (1000 + 1000*cos q1, 1000*sin q1, 0); at q1 = pi/2 that is (1000, 1000, 0).
    pose = forward_kinematics(m, [np.pi / 2, 0.0])
    assert np.allclose(pose[:3, 3], [1000.0, 1000.0, 0.0], atol=1e-9)

def test_last_joint_does_not_move_the_tip(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    # The tip link sits AT the j2 frame, so rotating j2 changes the tip's
    # orientation but not its position. This is a URDF convention worth
    # asserting -- getting it backwards is a common source of bad models.
    pose = forward_kinematics(m, [0.0, np.pi])
    assert np.allclose(pose[:3, 3], [2000.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(pose[:3, :3], np.diag([-1.0, -1.0, 1.0]), atol=1e-9)

def test_wrong_input_count_raises(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    with pytest.raises(ValueError, match="expects 2 joint values, got 3"):
        forward_kinematics(m, [0.0, 0.0, 0.0])

def test_link_poses_covers_every_link(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    poses = link_poses(m, [0.0, 0.0])
    assert set(poses) == {"base", "link1", "tip"}
    assert np.allclose(poses["link1"][:3, 3], [1000.0, 0.0, 0.0])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd skills/viam-arm-module/scripts && uv run pytest tests/test_fk.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named '_armkit.fk'`

- [ ] **Step 3: Implement**

```python
# _armkit/fk.py
"""Forward kinematics over a parsed KinematicModel.

Why this exists: FK lives only in Go's referenceframe package. Python and
C++ arm modules must implement get_end_position() with no SDK support, and
nothing anywhere lets an author check a kinematics file before shipping it.

Translations are millimeters, joint values radians (millimeters for
prismatic joints), matching the KinematicModel contract.
"""
from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from .model import KinematicModel


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' rotation formula. `axis` must be unit length."""
    k = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


def joint_transform(joint, value: float) -> np.ndarray:
    """Origin offset followed by the joint's own motion."""
    motion = np.eye(4)
    if joint.type in ("revolute", "continuous"):
        motion[:3, :3] = axis_angle_to_matrix(joint.axis, value)
    elif joint.type == "prismatic":
        motion[:3, 3] = joint.axis * value
    elif joint.type != "fixed":
        raise ValueError(f"unsupported joint type {joint.type!r} on joint {joint.name!r}")
    return joint.origin @ motion


def link_poses(model: KinematicModel, values: Sequence[float]) -> dict[str, np.ndarray]:
    """Pose of every link relative to the base link."""
    chain = model.chain()
    actuated = [j for j in chain if j.actuated]
    if len(values) != len(actuated):
        raise ValueError(
            f"model {model.name!r} expects {len(actuated)} joint values, got {len(values)}"
        )

    it = iter(values)
    poses = {chain[0].parent: np.eye(4)}
    current = np.eye(4)
    for joint in chain:
        value = next(it) if joint.actuated else 0.0
        current = current @ joint_transform(joint, value)
        poses[joint.child] = current.copy()
    return poses


def forward_kinematics(model: KinematicModel, values: Sequence[float]) -> np.ndarray:
    """Pose of the tip link relative to the base link, as a 4x4 matrix."""
    return link_poses(model, values)[model.tip_link]
```

- [ ] **Step 4: Run tests**

Run: `cd skills/viam-arm-module/scripts && uv run pytest tests/test_fk.py -q`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
git add skills/viam-arm-module/scripts/_armkit/fk.py skills/viam-arm-module/scripts/tests/test_fk.py
git commit -m "feat(armkit): forward kinematics"
```

---

### Task A5: SVA and DH model JSON parser

> **Split into three tasks after A3b.** As written below this is one task, but it needs:
> the uniform link/joint mapping, five orientation types spanning three angular
> conventions, `output_frames`, DH support, mimic-in-SVA, and geometry. A3b demonstrated
> that a task this wide generates several review rounds and hidden depth.
>
> - **A5a — SVA core.** Uniform mapping, links/joints, limits (degrees→radians), the
>   trailing-link fixed joint. Ships with `ov_degrees` only; other orientation types raise
>   a clear "unsupported orientation type" error.
> - **A5b — Orientations.** `ov_radians`, `euler_angles` (reusing `rpy_to_matrix`),
>   `axis_angles`, `quaternion`. Each with its own unit test against a known rotation.
>   The `ur20.urdf` ↔ `ur20.json` 0.000 mm agreement is the integration gate.
> - **A5c — `output_frames`, DH, mimic-in-SVA.** Honor `output_frames` (more than one is
>   an error, matching RDK); DH as a `kinematic_param_type`; SVA mimic configs.
>   **This is the trigger for extracting `model.py`'s topology trio** (`_validate_roots`,
>   `_bfs_all_joints`, `_require_resolvable_tip`) into their own module — deferred twice
>   on the reviewer's advice, and this is the task that was named as the trigger.

**Files:**
- Create: `skills/viam-arm-module/scripts/_armkit/sva.py`
- Test: `skills/viam-arm-module/scripts/tests/test_sva.py`

Read `~/go/pkg/mod/go.viam.com/rdk@v1.0.0/referenceframe/model_json.go` first — it is the
authority on the schema. `kinematic_param_type` accepts `"SVA"` (or empty, the default)
and `"DH"`. SVA translations are millimeters and orientations use Viam's orientation
vector; joint limits are **degrees** and must be converted to radians on parse.

**SVA is structured differently from URDF, and this is the whole difficulty of this task.**
URDF gives every joint a `parent`, a `child`, and an `origin`. SVA instead describes one
alternating chain in which links *and* joints are both nodes, each naming only its
`parent` — and the **offsets live on the links, not the joints**. From `ur5e.json`:

```
base_link      parent: world                translation z=162.5   (+ geometry)
shoulder_pan_joint   parent: base_link      axis, min/max degrees (no translation)
shoulder_link  parent: shoulder_pan_joint   translation 0         (+ geometry)
shoulder_lift_joint  parent: shoulder_link  axis, min/max
upper_arm_link parent: shoulder_lift_joint  translation x=-425    (+ geometry)
...
ee_link        parent: wrist_3_joint        translation           <- trailing tip offset
```

**Folding rule — use the uniform mapping.** Every SVA node becomes a `Joint`:

- SVA joint `J` → `Joint(name=J.id, type=J.type, parent=<previous frame>, child=J.id,
  origin=T(J's parent link), axis=J.axis, limits=deg2rad(J.min/max))`
- SVA link `L` → `Joint(name=f"{L.id}_offset", type="fixed", parent=<preceding joint id>,
  child=L.id, origin=T(L))`

SVA joints have no `child` field; this mapping synthesizes one. Verified against
`ur5e.json` during Task A2 review — it produces `dof == 6`, `chain()[-1].type == "fixed"`,
`base_link == "base_link"`, `tip_link == "ee_link"`, and an `actuated_joints` order of
shoulder_pan → shoulder_lift → elbow → wrist_1 → wrist_2 → wrist_3, all against an
unmodified `_armkit/model.py`.

**Do not instead fold each link into the following joint and append one trailing `fixed`
joint.** That was this plan's original wording and it is a trap: the last real joint's
`child` is already `ee_link`, so the trailing joint either reuses `ee_link` as its own
child — a self-loop, which `chain()` now rejects but which silently hung before the Task A2
fix — or invents a name like `ee_link_tip`, which breaks `tip_link == "ee_link"`.

**Consequence to expect:** `link_poses` will contain both `<joint_id>` and `<link_id>`
frames for SVA models. That is fine and arguably desirable (Viam's frame system names joint
frames too), and A6 is unaffected because it keys off `model.links`.

**Orientations are polymorphic — read the Conventions section's unit table before writing
any code here.** `orientation.type` selects the representation and its angular unit; the
authority is `spatialmath/orientation_json.go` plus `orientation_vector.go` and
`eulerangles.go`. The two fixtures deliberately disagree: `ur5e.json` uses `ov_degrees`,
`ur20.json` uses `euler_angles` (radians). Implement at minimum `ov_degrees`,
`ov_radians`, and `euler_angles`; raise a clear error on the rest rather than guessing.

`euler_angles` is radian RPY in the same `Rz·Ry·Rx` order as URDF, so `rpy_to_matrix` from
`_armkit/urdf.py` can be reused directly — do not write a second implementation.

Cover each orientation type with its own unit test against a known rotation before wiring
it into parsing.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_sva.py
import numpy as np
import pytest
from _armkit.sva import parse_sva

def test_parses_rdk_ur5e(fixtures):
    m = parse_sva(fixtures / "ur5e.json")
    assert m.dof == 6
    assert m.source_format == "sva"

def test_limits_converted_to_radians(fixtures):
    m = parse_sva(fixtures / "ur5e.json")
    # Verified: ur5e.json declares shoulder_pan_joint min/max as -360/+360 degrees.
    assert np.isclose(m.actuated_joints[0].lower, -2 * np.pi, atol=1e-6)
    assert np.isclose(m.actuated_joints[0].upper, 2 * np.pi, atol=1e-6)

def test_trailing_link_becomes_fixed_tip_transform(fixtures):
    m = parse_sva(fixtures / "ur5e.json")
    # ee_link has no joint after it; it must survive as a fixed transform,
    # otherwise the tip frame lands on wrist_3 and every pose is wrong.
    assert m.chain()[-1].type == "fixed"
    assert m.tip_link == "ee_link"

def test_dh_param_type_is_flagged(fixtures, tmp_path):
    p = tmp_path / "dh.json"
    p.write_text('{"name":"d","kinematic_param_type":"DH","links":[],"joints":[]}')
    m = parse_sva(p)
    assert m.source_format == "dh"

def test_unknown_param_type_raises(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text('{"name":"b","kinematic_param_type":"XYZ","links":[],"joints":[]}')
    with pytest.raises(ValueError, match="supported params are SVA and DH"):
        parse_sva(p)
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd skills/viam-arm-module/scripts && uv run pytest tests/test_sva.py -q`
Expected: FAIL — no module `_armkit.sva`

- [ ] **Step 3: Implement**

Mirror RDK's `model_json.go` structure: `links[]` carry `id`, `parent`, `translation`,
`orientation`, and optional `geometry`; `joints[]` carry `id`, `type`, `parent`, `axis`,
`max`, `min`. Reuse `_armkit.model` types. Convert degree limits to radians. Build the
orientation matrix from Viam's orientation vector (`OrientationConfig`), and raise on
unknown `kinematic_param_type` with a message matching RDK's own wording.

Error text must match the test: `supported params are SVA and DH`.

- [ ] **Step 4: Run tests**

Run: `cd skills/viam-arm-module/scripts && uv run pytest tests/test_sva.py -q`
Expected: PASS (4 passed)

- [ ] **Step 5: Cross-check FK against a real arm**

Add to `tests/test_fk.py` — the same arm from two formats must agree:

```python
def test_ur20_urdf_and_sva_agree(fixtures):
    from _armkit.sva import parse_sva
    a = parse_urdf(fixtures / "ur20.urdf")
    b = parse_sva(fixtures / "ur20.json")
    q = [0.1, -0.4, 0.7, 0.2, -0.3, 0.5]
    assert np.allclose(forward_kinematics(a, q)[:3, 3],
                       forward_kinematics(b, q)[:3, 3], atol=1.0)  # 1 mm
```

If this fails, the bug is real and is exactly what the toolkit exists to catch —
investigate before proceeding. Do not loosen the tolerance to make it pass.

- [ ] **Step 6: Commit**

```bash
git add skills/viam-arm-module/scripts/_armkit/sva.py skills/viam-arm-module/scripts/tests/
git commit -m "feat(armkit): SVA and DH model JSON parsing"
```

---

### Task A6: Mesh inspection

**Files:**
- Create: `skills/viam-arm-module/scripts/_armkit/meshes.py`
- Modify: `skills/viam-arm-module/scripts/_armkit/urdf.py` (populate mesh references)
- Test: `skills/viam-arm-module/scripts/tests/test_meshes.py`

`trimesh` handles STL/OBJ/PLY natively and COLLADA via `pycollada`. COLLADA matters: ROS
URDFs overwhelmingly reference `.dae`.

**What `meshed.urdf` does not yet cover** (extend it in place if you want these): no
`<origin>` inside the `<visual>`/`<collision>` elements, so `MeshReport.origin_offset_mm`
has no fixture; and no single link carries both a visual and a collision mesh, so a
pairing or dedup bug would go uncaught. Flagged during Task A1 review.

- [ ] **Step 1: Extend the URDF parser to record mesh references**

Parse `<collision>` and `<visual>` children of each `<link>`, recording `<geometry><mesh
filename="..."/>` paths and primitive geometry. Resolve `package://pkg/rest` relative to
the URDF's directory, walking up for a directory named `pkg`. Record unresolved paths
rather than raising — reporting missing meshes is a feature.

- [ ] **Step 2: Write the failing test**

```python
# tests/test_meshes.py
from _armkit.meshes import inspect_meshes
from _armkit.urdf import parse_urdf

def test_reports_missing_meshes(fixtures):
    # meshed.urdf references two meshes that do not exist on disk: one via
    # package:// and one relative. Neither resolves; both must be reported.
    m = parse_urdf(fixtures / "meshed.urdf")
    report = inspect_meshes(m)
    assert len(report) == 2
    assert all(r.path for r in report)
    assert all(not r.resolved for r in report)

def test_splits_collision_from_visual(fixtures):
    m = parse_urdf(fixtures / "meshed.urdf")
    kinds = {r.kind for r in inspect_meshes(m)}
    assert kinds == {"collision", "visual"}

def test_no_meshes_is_empty_not_an_error(fixtures):
    # ur20.urdf carries only <box> collision primitives and no <visual> at all.
    m = parse_urdf(fixtures / "ur20.urdf")
    assert inspect_meshes(m) == []
```

- [ ] **Step 3: Run to verify it fails**

Run: `cd skills/viam-arm-module/scripts && uv run pytest tests/test_meshes.py -q`
Expected: FAIL — no module `_armkit.meshes`

- [ ] **Step 4: Implement**

`inspect_meshes(model) -> list[MeshReport]` where `MeshReport` carries `link`, `path`,
`kind` (`collision`/`visual`), `resolved`, and — when resolved — `triangles`,
`bbox_mm`, `origin_offset_mm`, `bytes`. Import `trimesh` lazily inside the function so
parsing-only callers pay nothing.

- [ ] **Step 5: Run tests**

Run: `cd skills/viam-arm-module/scripts && uv run pytest -q`
Expected: PASS (all)

- [ ] **Step 6: Commit**

```bash
git add skills/viam-arm-module/scripts
git commit -m "feat(armkit): mesh reference resolution and inspection"
```

---

### Task A7: `armkit.py validate`

**Files:**
- Create: `skills/viam-arm-module/scripts/armkit.py`
- Create: `skills/viam-arm-module/scripts/_armkit/checks.py`
- Test: `skills/viam-arm-module/scripts/tests/test_cli.py`

This is the Phase 1 gate. **Exit code is the contract:** 0 = pass, 1 = failure, 2 = usage
error. An agent depends on this.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_cli.py
import subprocess, sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1]

def run(*args):
    return subprocess.run(
        [sys.executable, str(SCRIPTS / "armkit.py"), *args],
        capture_output=True, text=True,
    )

def test_validate_passes_on_good_model(fixtures):
    r = run("validate", str(fixtures / "two_link.urdf"))
    assert r.returncode == 0, r.stderr
    assert "2 actuated joints" in r.stdout

def test_validate_fails_on_missing_file():
    r = run("validate", "/nonexistent.urdf")
    assert r.returncode == 1

def test_validate_reports_joint_count_mismatch(fixtures):
    r = run("validate", str(fixtures / "two_link.urdf"), "--expect-dof", "6")
    assert r.returncode == 1
    assert "expected 6" in r.stdout

def test_validate_at_config_prints_pose(fixtures):
    r = run("validate", str(fixtures / "two_link.urdf"), "--at", "0,0")
    assert r.returncode == 0
    assert "2000" in r.stdout
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd skills/viam-arm-module/scripts && uv run pytest tests/test_cli.py -q`
Expected: FAIL — armkit.py does not exist

- [ ] **Step 3: Implement the entry script**

```python
#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=1.26", "trimesh>=4.0", "pycollada>=0.8"]
# ///
"""armkit — validate and inspect Viam arm kinematics files.

Why this exists
---------------
Nothing in the Viam ecosystem validates a kinematics file. RDK will load a
model whose joints are in the wrong order, whose URDF was authored in meters
where Viam expects millimeters, or whose `continuous` joints silently became
`revolute` with infinite limits. All of these parse cleanly and plan wrongly.

Forward kinematics exists only in Go's referenceframe package, so authors of
Python and C++ arm modules have no way to check their own model at all.

Subcommands: validate, meshes, simplify, convert
"""
```

Dispatch with `argparse` sub-parsers. `validate` accepts a path, `--at "q1,q2,..."`
(radians), `--expect-dof N`, `--expect FILE`, and `--cross-check` (Task A10). Route by
extension: `.urdf` to `parse_urdf`, `.json` to `parse_sva`, anything else exits 2 with the
same message RDK gives — `only files with .json and .urdf file extensions are supported`.

- [ ] **Step 4: Implement the trap checks**

`_armkit/checks.py` returns a list of `Finding(level, code, message)` where level is
`error` or `warn`. Errors set exit 1; warnings do not. From the spec:

| code | level | condition |
|---|---|---|
| `unit-scale` | warn | any link translation > 10 000 mm or whole chain < 10 mm — likely a meters/mm error |
| `continuous-joint` | warn | joint type `continuous`; RDK gives it infinite limits |
| `dof-mismatch` | error | `--expect-dof` given and DOF differs |
| `zero-limits` | error | actuated joint with `lower == upper` |
| `inverted-limits` | error | `lower > upper` |
| `unresolved-mesh` | warn | mesh reference that does not resolve on disk |
| `heavy-mesh` | warn | resolved collision mesh over a threshold — **see the measured distribution below; 5 000 was a bad guess** |

> **The original 5 000-triangle threshold was invented, and A6 measured why it is wrong.**
> Across 913 successfully-loaded meshes from 56 real vendor URDFs:
>
> | min | p10 | p25 | median | p75 | p90 | p95 | max |
> |---|---|---|---|---|---|---|---|
> | 2 636 | 3 542 | 7 916 | **17 601** | 33 292 | 58 122 | 127 177 | 448 644 |
>
> 5 000 sits below the 10th percentile — nearly every real vendor mesh would trip it, so
> the warning would fire on almost every file and mean nothing.
>
> **Decided during A6 review: threshold 50 000, collision meshes only.** Just under p90,
> so it flags roughly the worst decile and stays quiet on a typical arm.
>
> **Collision-only is not a refinement, it is the whole signal.** Measured across four
> vendor arms, the collision and visual triangle distributions are *identical* (n=15 each,
> median 20 830, p90 38 110, max 233 248) because **vendors ship the same mesh file for
> both**. A 233k-triangle visual mesh is fine — it renders once. The same file as
> collision geometry is checked every planning step. Scoping to `kind == "collision"` also
> halves finding volume by not double-reporting one file.
>
> Say what good looks like, because the corpus median is itself bad practice — collision
> geometry wants hundreds of triangles, not tens of thousands:
>
> ```
> [WARN] heavy-mesh: collision mesh 'meshes/link1.dae' on link 'link1' has 233,248
>        triangles (typical vendor collision meshes are ~18,000; motion planning wants
>        far fewer). Consider a convex hull or a primitive -- see `armkit simplify`.
> ```

> **`validate` must use the cheap path. Measured during A6 review:**
>
> | operation | reviewer | implementer | notes |
> |---|---|---|---|
> | `parse_urdf` | 2.3 ms | ~0.3 ms | |
> | resolution only (14 refs) | 0.25 ms | ~0.34 ms | agree |
> | full load (14 refs) | 1505.7 ms | ~108 ms | **disk-cache dependent** |
>
> **The two measurements disagree by 14×, and the implementer diagnosed why rather than
> restating either number:** its own earlier corpus scan had already read every mesh under
> `~/src/mycobot_ros2`, warming the OS page cache, so its "cold process" run was not
> cold-disk. Both figures are real; they measure different cache states. The honest range
> is **~300× warm to ~6 000× cold**, and a first run on a user's machine is the cold case.
>
> Caching's own contribution was isolated separately from that confound: 183 ms without
> dedup vs 108 ms with, over 7 distinct files behind 14 references.
>
> The conclusion is unchanged and does not depend on which figure is right. `unresolved-mesh` needs **only** resolution, and it is the finding that
> closes the largest PASS-vs-RDK-loads gap. `inspect_meshes` therefore takes
> `load: bool = True`; `validate` calls it with `load=False` so every unresolved mesh is
> caught for free, and `armkit meshes` / `heavy-mesh` opt into loading. Without this, A8
> must choose between a 600× slower hot path and leaving the most common RDK-rejection
> reason unchecked.
>
> Other A6 corpus data worth having in A8: of 1 024 mesh references, **89% resolved and
> loaded, 3% resolved but failed to load, 8% did not resolve**. All 31 load failures trace
> to 8 distinct broken vendor COLLADA files (`DaeBrokenRefError: Material not found`) —
> defects in the vendor's own files, not in armkit. The unresolved cases are real
> `package://` path mismatches, including 100% of UR mesh references in this checkout.
| `dh-format` | warn | `kinematic_param_type: "DH"` — supported but not recommended |
| `missing-limits` | error | actuated, non-`continuous` joint whose `lower`/`upper` are `None` |
| `structure` | error | `chain()` raised — branching, cycle, multiple roots, or disconnection |
| `parse` | error | the parser raised `ValueError` — surface its message verbatim |
| `input-out-of-bounds` | error | a `--at` value falls outside its joint's declared limits |
| `joints-off-chain` | error | actuated joints exist that are not on the path to the tip |

> **Reported joints ≠ joints to edit. This bit us in C1 and will bite A9.**
> `joints-off-chain` lists only *actuated* off-chain joints, because mimic joints are
> excluded from `actuated_joints` by design (A3b, matching RDK). A gripper's remaining
> joints are usually `fixed` or `mimic`, so they are **not listed and still must go**.
>
> Measured on a real mycobot gripper URDF: armkit names one joint
> (`gripper_controller`); deleting only that joint turns a `joints-off-chain` error into a
> **`parse` error**, because five other joints `<mimic>` it. The actual edit is 6 joints
> and 6 links — the whole gripper subtree.
>
> Any guidance or tooling that removes off-chain joints must work **from the tip
> downward** (delete every joint whose parent is the tip or downstream of it), not from
> the list armkit prints. A9's `simplify` should implement it that way, and any reference
> doc describing the fix must say so.

**`joints-off-chain` enforces the scope: one arm, not a robot.** RDK's BFS counts every
actuated joint in the tree toward DoF — correct for RDK, wrong for a Viam arm module.
Measured on a mycobot 320 with its gripper attached and `--tip gripper_base`:

```
dof (all actuated, BFS):   7
actuated ON the chain:     6   joint2_to_joint1 ... joint6output_to_joint6
actuated OFF the chain:    1   gripper_controller
```

The arm is 6-DoF. Shipping that file yields a module declaring a 7-DoF arm whose
`JointPositions` returns 6 values — an arity mismatch in `GetKinematics` and in motion
planning. The gripper is a separate Viam component.

This is the one place armkit deliberately diverges from RDK on *scope* rather than on
kinematic fidelity. Record it in `test_parity.py` alongside the other divergences, in the
"armkit is right" group.

**`input-out-of-bounds` closes a divergence found during A4 review.** RDK's `Transform()`
validates inputs against joint limits and errors; armkit's `fk.py` does no limit checking
at all. Measured on `two_link.urdf` (limits ±3.14159):

| input | RDK | armkit |
|---|---|---|
| `[0, math.pi]` — 2.65e-6 over | REJECT `input out of bounds` | `[2000, 0, 0]` |
| `[0, 100.0]` — ~16 revolutions | REJECT | `[2000, 0, 0]` |
| `[1e6, 0]` | REJECT | `[1936.75, -350.00, 0]` |

Every model with finite limits is affected. Without this check, `armkit validate --at`
reports success on configurations RDK refuses to evaluate. `continuous` joints carry
`±inf` limits and must pass any value. Recorded in `test_parity.py` during A4; enforcement
belongs here rather than in `fk.py`, whose scope is computing poses.

**`nonunit-axis` was removed.** It was defined as "axis that is not unit length after
normalization attempt," which can never fire — `parse_urdf` always normalizes, so the
post-parse norm is exactly 1.0 for every 3-vector (measured during A3 review). Axis arity
is now rejected at parse time per the error-handling contract and surfaces as `parse`.

Do not check `root.tag == "robot"` in the CLI — that belongs in the parser, per contract
rule 3. A non-URDF root currently parses and then produces the misleading diagnosis
`kinematic model has no root link (cycle?)`.

Print a summary line (`<name>: <n> actuated joints, base <base> -> tip <tip>`), then each
finding, then `PASS` or `FAIL`.

**Two ordering traps, both found during Task A2 review:**

**`dof`, `base_link`, and `tip_link` are properties that raise.** They each call `chain()`,
which throws on a branched, cyclic, multi-rooted, or disconnected model. The summary line
above reads all three *before* any check runs, so on a malformed model it produces an
uncaught traceback instead of exit 1 with a finding — defeating the whole exit-code
contract on exactly the inputs armkit exists to catch. Wrap the summary in
`try/except ValueError` and convert the failure into a `structure` error finding. Also
compute `chain()` once into a local and reuse it, or a broken model raises four identical
errors.

**`missing-limits` must be checked before `zero-limits` and `inverted-limits`.** A3 leaves
`lower = upper = None` for an actuated joint with no `<limit>` element. Measured: `lower ==
upper` is `True` for `None == None`, so `zero-limits` fires with a misleading diagnosis;
and `lower > upper` raises `TypeError: '>' not supported between instances of 'NoneType'
and 'NoneType'`, crashing the CLI. Guard on `is None` first and emit `missing-limits`
instead. (`continuous` joints are exempt — A3 gives them `±inf`, and both comparisons are
correctly `False` there.)

- [ ] **Step 5: Run tests**

Run: `cd skills/viam-arm-module/scripts && uv run pytest -q`
Expected: PASS (all)

- [ ] **Step 6: Verify it runs standalone via uv**

Run: `uv run --isolated skills/viam-arm-module/scripts/armkit.py validate skills/viam-arm-module/scripts/tests/fixtures/ur20.urdf`
Expected: exit 0, summary reporting 6 actuated joints. (`ur20.urdf` uses `<box>` collision
primitives and references no meshes, so expect no `unresolved-mesh` findings here — use
`meshed.urdf` to exercise that path.)

`--isolated` is essential: without it the subprocess resolves imports from the dev venv,
so a missing or wrong entry in `armkit.py`'s PEP 723 `dependencies` block would pass every
test and still break the real `uv run armkit.py` user path. Add a test that shells out
with `--isolated` so this is enforced rather than remembered.

**Diagnose a native-library failure as an environment problem, not an armkit bug.**
Measured during A4b review: when the `viam-sdk` wheel installs but `libviam_rust_utils`
fails to `dlopen` (old glibc under a manylinux wheel, broken install), the error-handling
contract's catch-all reports `internal error while parsing <file> (OSError: ...) -- this is
an armkit bug`, telling the user to file a bug about their own environment. Special-case it
in `armkit.py`'s top-level handler and **exit 2** (usage/environment), not 1 — it is not a
finding about the user's file:

```python
except ValueError as e:
    if "libviam_rust_utils" in str(e):
        sys.exit("armkit could not load viam-sdk's native library on this platform.\n"
                 "Supported: Linux (glibc/musl) x86_64/aarch64/armv7, macOS x86_64/arm64, Windows x64.")
```

**Do not add a blanket `timeout = N` under `[tool.pytest.ini_options]`.** On a cold uv
cache the `--isolated` test downloads and resolves numpy, trimesh, and pycollada, which
can exceed any sane global bound and will flake. Keep `@pytest.mark.timeout(...)` targeted
at the individual tests where a hang is actually possible — currently only
`test_chain_raises_on_reachable_cycle`. Flagged during Task A2 review.

This is the real user path — PEP 723 with no install step. It must work before committing.

- [ ] **Step 7: Commit**

```bash
git add skills/viam-arm-module/scripts
git commit -m "feat(armkit): validate subcommand with kinematics trap checks"
```

---

### Task A8: `armkit.py meshes`

**First, extract `_armkit/report.py`.** A7 review found the seam is now due: `armkit.py`
grew 260 → 342 lines, and `_report` alone is 64 lines rendering both text and JSON while
knowing about `rdk_parity` summarization, remedy indentation, pose formatting, and the
`schema_version`/`contract` fields. A8 (`meshes`) and A9 (`simplify`/`convert`) each need
the same two-format rendering with different payloads; duplicating it three ways is how
the JSON schema drifts between subcommands — which matters now that `schema_version: 1`
is a published promise. Move `_report`, `Finding` rendering, and `_RDK_PARITY_CODES` there
before adding anything. Drops `armkit.py` to roughly 270 and gives the contract one owner.

**Mesh reference lists already exist.** A7 populates `Link.visual_meshes` and
`Link.collision_meshes` from the XML (12 lines, no disk access, no trimesh). A6/A8 consume
those lists rather than re-parsing, and own the actual resolution, `unresolved-mesh`,
`heavy-mesh`, and trimesh loading.

**Files:**
- Modify: `skills/viam-arm-module/scripts/armkit.py`
- Test: `skills/viam-arm-module/scripts/tests/test_cli.py`

- [ ] **Step 1: Write the failing test** — `meshes` on `ur20.urdf` exits 0 and prints one row per mesh reference with a `MISSING` marker for unresolved ones.
- [ ] **Step 2: Run it and watch it fail.**
- [ ] **Step 3: Implement** — table over `inspect_meshes`: link, kind, triangles, bbox, size, status. Add `--json` for machine-readable output.
- [ ] **Step 4: Run tests.** Expected: PASS.
- [ ] **Step 5: Commit** — `feat(armkit): meshes subcommand`

---

### Task A9: `armkit.py simplify` and `convert`

**Files:**
- Create: `skills/viam-arm-module/scripts/_armkit/simplify.py`
- Create: `skills/viam-arm-module/scripts/_armkit/convert.py`
- Modify: `skills/viam-arm-module/scripts/armkit.py`
- Test: `skills/viam-arm-module/scripts/tests/test_simplify.py`

Ports of two existing tools. Read them first: `~/src/urdf-simplifier/main.go` (Go, replaces
mesh collision geometry with bounding prisms) and `~/src/urdf-to-sva-converter/src/urdf_to_sva/`
(Python — `parser.py`, `transform.py`, `converter.py`, `models.py`).

Vendor rather than depend, per the spec. `convert` is an escape hatch, not the default
path: only when no URDF exists or SVA is specifically required.

**Heads-up on `ur20.urdf`:** it declares an orphan `world` link that no joint attaches to.
This is deliberate fixture fidelity — real vendor URDFs carry exactly this kind of wart.
If `convert` iterates `model.links` and looks up `link_poses[...]`, it will `KeyError` on a
link with no pose. Handle it gracefully; do not "fix" the fixture. Flagged during Task A1
review.

- [ ] **Step 1: Write the failing test** — `simplify` on a URDF with a resolvable mesh emits a URDF whose collision geometry is a `<box>`, preserving the visual mesh. Requires a small fixture mesh; generate a cube STL with `trimesh` in `conftest.py`.
- [ ] **Step 2: Run it and watch it fail.**
- [ ] **Step 3: Port `urdf-simplifier`** into `_armkit/simplify.py` using `trimesh` bounding boxes.
- [ ] **Step 4: Run tests.** Expected: PASS.
- [ ] **Step 5: Commit** — `feat(armkit): simplify subcommand, port of urdf-simplifier`
- [ ] **Step 6: Port the converter** into `_armkit/convert.py`, carrying over its existing tests where they apply.
- [ ] **Step 7: Run tests and commit** — `feat(armkit): convert subcommand, vendored urdf-to-sva-converter`

---

### Task A10: `--cross-check` against RDK

**Files:**
- Modify: `skills/viam-arm-module/scripts/armkit.py`
- Create: `skills/viam-arm-module/scripts/_armkit/crosscheck.py`

The spec's option (c): offline FK is the fast gate; this validates the toolkit's FK against
RDK's own implementation, catching drift. This is also the job the built-in `sim` arm keeps
after being cut as a workflow milestone.

- [ ] **Step 1: Write the integration test**, marked `@pytest.mark.integration` and skipped when `viam-server` is not on PATH — it must not break the default test run.
- [ ] **Step 2: Implement** — write a temp machine config with RDK's built-in `sim` arm (`{"model": "sim", "attributes": {"model-path": "<abs path>"}}`), start `viam-server`, connect with the Viam SDK, command N joint configs, compare `get_end_position()` against `forward_kinematics`. Reuse `skills/local-viam-server/machine_up.py` rather than reimplementing server startup.
- [ ] **Step 3: Run it against `ur5e.json`.** Expected: agreement within 1 mm and 0.1 degrees.
- [ ] **Step 4: Document the result** in `references/kinematics-reference.md` — a measured statement that the toolkit's FK matches RDK's is worth more than an assurance.
- [ ] **Step 5: Commit** — `feat(armkit): cross-check FK against RDK's sim arm`

---

### Task A11: `armkit_live.py fk-diff`

**Files:**
- Create: `skills/viam-arm-module/scripts/armkit_live.py`
- Create: `skills/viam-arm-module/scripts/_armkit/live.py`

The Phase 5 gate. Separate entry point so the offline loop never pulls `viam-sdk`.

- [ ] **Step 1: Write the entry script** with its own PEP 723 header:

```python
#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk>=0.60", "numpy>=1.26"]
# ///
```

- [ ] **Step 2: Implement `fk-diff`** — connect to a machine (reuse the auth flow in `skills/local-viam-server/machine_up.py`; do not invent a new one), sample N configs within joint limits, `move_to_joint_positions`, read `get_end_position()`, diff against `forward_kinematics`. Report per-config error plus max and mean. Exit 1 if max exceeds `--tolerance-mm` (default 5).
- [ ] **Step 3: Guard it** — refuse to run without `--i-have-cleared-the-workspace`. This moves a real arm; an agent must not trigger motion incidentally.
- [ ] **Step 4: Test** what is testable offline — argument parsing, config sampling within limits, error math — with the SDK mocked. Live behavior is verified in Task C4.
- [ ] **Step 5: Commit** — `feat(armkit): fk-diff live hardware validation`

---

### Task A12: `armkit_live.py ops-test`

**Files:**
- Modify: `skills/viam-arm-module/scripts/armkit_live.py`

**Advisory gate — module development completes without this passing.** Say so in `--help`.

- [ ] **Step 1: Implement the interrupt test** — start a long `move_to_joint_positions`, issue a second command mid-flight, assert the first returns rather than blocking to completion.
- [ ] **Step 2: Implement the session-drop test** — start a long move, drop the client connection, reconnect, assert the arm stopped.
- [ ] **Step 3: Report per-language expectations** — Go with `SingleOperationManager` should pass both; Python can pass the drop test but needs hand-rolled single-flight for the interrupt test; C++ needs both hand-rolled. Print which behavior was expected for the module's language so a failure is interpretable.
- [ ] **Step 4: Same workspace guard as `fk-diff`.**
- [ ] **Step 5: Commit** — `feat(armkit): ops-test cancellation checks (advisory)`

---

### Task A3b: RDK parity

**Files:**
- Modify: `_armkit/model.py`, `_armkit/urdf.py`, `_armkit/sva.py` (when A5 lands)
- Test: `tests/test_model.py`, `tests/test_urdf.py`, `tests/test_parity.py` (new)

**Goal: armkit accepts exactly what RDK accepts — no more, no less.** Four gaps, all
measured against RDK v1.0.0 by executing it, not by reading it.

**A reusable Go probe exists** at `<scratchpad>/rdkprobe/` — a ~25-line program calling
`referenceframe.KinematicModelFromFile` and printing ACCEPT/DoF or REJECT/error for each
path given. Use it as the oracle for every claim in this task. If it has been cleaned up,
recreate it; the cost is minutes and it is the only way to check parity honestly.

**Correction on the record:** an earlier draft of this plan asserted RDK accepts branching
models and armkit was wrongly rejecting ~30 of 84 corpus URDFs. That was wrong.
`SimpleModel` supports branching *structurally*, but `model_json.go:129` requires exactly
one leaf when no `output_frames` is declared, and URDF cannot declare one. Measured on a
synthetic two-leaf URDF: `REJECT — need exactly one end effector, have [finger_l finger_r]`.

#### 1. Mimic joints must not consume input slots

RDK excludes mimic frames from the input schema and derives their value at runtime as
`multiplier * inputs[source] + offset` (`model.go`, `mimicMappings`). Measured on
`referenceframe/testfiles/test_mimic_serial.urdf`: **RDK DoF=2, armkit DoF=3.** A live
correctness bug, independent of branching.

- Parse `<mimic joint="..." multiplier="..." offset="..."/>` in `urdf.py`. `multiplier`
  defaults to 1, `offset` to 0.
- Add a `mimic` field to `Joint`. A mimic joint is still a joint in the tree — it just
  takes no input slot.
- `actuated_joints` and `dof` must exclude mimic joints.
- FK derives the mimic value from its source joint's value.
- RDK zeroes a mimic joint's own `Min`/`Max` (`model_urdf.go:191`) because the source
  joint's limits govern. Match that.

#### 2. Multi-leaf models: match RDK's diagnosis

Replace the `branching at link ...` error with RDK's framing. A **leaf** is a link that is
never any joint's parent. Rules:

- exactly one leaf, no declared tip → that leaf is the tip
- multiple leaves, no declared tip → `need exactly one end effector, have [...]`, listing
  the leaves sorted
- a declared tip → use it; branching is legal

This makes `chain()` leaf-based rather than branch-based. Roughly 29 corpus files hit this
path; they should keep failing, but with RDK's message.

#### 3. Declared tip support

- SVA JSON: honor `output_frames`. More than one is an error in RDK
  (`multiple output frames are not yet supported`) — match that.
- URDF has no equivalent, so add a `--tip LINK` CLI flag. This is the escape hatch that
  lets a user analyze a gripper-bearing vendor URDF by naming the arm's flange.
- A declared tip that does not exist in the model is an error naming the available leaves.

#### 4. BFS input ordering

Once a declared tip permits branching, input order stops being obvious. RDK orders the
flat input vector by **BFS over the frame system** and maps chain frames to offsets,
because "sibling-branch frames with nonzero DoF occupy slots between chain frames"
(`model.go:150`). Serial models are unaffected — BFS order equals chain order. Implement
BFS ordering in `actuated_joints` and verify a serial model's order is unchanged.

**Gate:** a new `tests/test_parity.py` asserting armkit's accept/reject verdict and DoF
match the Go probe's, across a fixture list including `test_mimic_serial.urdf`, a
synthetic two-leaf file, a declared-tip case, and the existing fixtures. Where armkit
deliberately differs — it reports unresolved meshes as a finding where RDK hard-fails
during parse — assert the difference explicitly so it is a decision on the record rather
than drift.

---

# Workstream B — references

Independent of A. Each task extracts from source and cites file and line. **Rule: every
claim carries a source reference.** This repo's skills are source-verified, and the design
doc's gap table is only credible if the references are too.

Primary sources: `~/go/pkg/mod/go.viam.com/rdk@v1.0.0`, `~/src/viam-cpp-sdk`,
`~/src/viam-python-sdk`, `~/src/viam-api`, and the modules `~/src/universal-robots`,
`~/src/viam-ufactory-xarm`, `~/src/viam-dobot`, `~/src/viam-mycobot`,
`~/src/viam-waveshare-roarm`.

### Task B1: `sdk-gaps.md`

Write this **first** — it is the highest-value standalone artifact and several other
references point at it.

- [ ] Transcribe the design doc's gap table with full evidence: exact file paths and line numbers for each claim.
- [ ] Include the language capability matrix (FK in-process / `Get3DModels` / self-cancellation / single-flight).
- [ ] For gap 1, sketch the Python SDK change concretely enough to become a PR: `get_3d_models` on the `Arm` ABC, client, and service, mirroring `get_kinematics`.
- [ ] Re-verify every claim against source before committing. Do not copy assertions forward from the design doc unchecked.
- [ ] Commit — `docs(arm-module): SDK gap analysis`

### Task B2: `kinematics-reference.md`

- [ ] SVA vs URDF vs inline DH — the two API-level formats (`KINEMATICS_FILE_FORMAT_SVA`/`_URDF`) versus `kinematic_param_type` (`"SVA"`/`"DH"`). These are different levels; say so explicitly.
- [ ] SVA model JSON schema from `referenceframe/model_json.go`.
- [ ] Geometry primitives from `spatialmath/geometry.go`; why mesh collision geometry effectively requires URDF (`MeshData` needs inline bytes; `MeshFilePath` is URDF round-trip only).
- [ ] The six traps, each with a symptom and a detection method.
- [ ] Worked example: reading `ur5e.json`.
- [ ] Record the Task A10 cross-check result.
- [ ] Commit.

### Task B3: `meshes-3d-reference.md`

- [ ] Collision vs visual meshes — different purposes, different weight budgets.
- [ ] `Get3DModels` per language, with the Python hole stated plainly and its Phase 2 consequence.
- [ ] `GetKinematicsResponse.meshes_by_urdf_filepath` and how RDK populates it.
- [ ] Formats, units, origin conventions; packaging meshes with the module binary.
- [ ] Commit.

### Task B4: `driver-reference.md`

- [ ] Required-methods table per language, sourced: Go `Arm` interface, C++ `arm.hpp` pure virtuals, Python abstract methods.
- [ ] Config and `Validate` patterns per language.
- [ ] Connect/reconnect, and `DoCommand` as an escape hatch.
- [ ] Worked comparison of `EndPosition` across `viam-ufactory-xarm` (Go, `model.Transform`) and `universal-robots` (C++, controller TCP pose).
- [ ] Commit.

### Task B5: `operations-reference.md`

- [ ] The two levels: self-cancellation vs single-flight. Conflating them is the mistake to prevent.
- [ ] Go: `operation.SingleOperationManager`, `CancelOtherWithLabel`.
- [ ] Python: `run_with_operation`, `opid` metadata, `await op.is_cancelled()` — and what it does *not* give you.
- [ ] C++: nothing module-side; `RobotClient`'s operation methods are the client API and are not a substitute.
- [ ] Hand-rolled single-flight patterns for Python and C++.
- [ ] The blocking contract the Go `Arm` docs state, and which languages can honor it.
- [ ] Commit.

### Task B6: `motion-delegation-reference.md`

- [ ] The three properties, each with its source citation: weak dependencies (no cycle), `GoToInputs` (no IK recursion), `CurrentInputs` (no FK recursion).
- [ ] Worked delegation code for Python and C++.
- [ ] The frame-correctness trap — transform into the arm's base frame, not `world`. Show the wrong version and the right one; this looks correct on a bench robot and breaks in a work cell.
- [ ] Latency budget and caching; the runtime-coupling caveat for the module README.
- [ ] Commit.

### Task B7: `simulated-arm-reference.md`

- [ ] The pattern from `viam-dobot/arm/simulated.go`, annotated.
- [ ] `-simulated` naming and `meta.json` with both models.
- [ ] Per-language translation: async task or thread instead of a goroutine; motion delegation instead of in-process `armplanning`.
- [ ] **The Python Phase 2 limitation** and its two workarounds, per the design doc.
- [ ] Commit.

### Task B8: `packaging-reference.md` and `cheatsheet.md`

- [ ] Packaging: embedding kinematics (`go:embed` and equivalents), meshes alongside the binary, cross-compile, `viam module upload`, clean-machine reload.
- [ ] Cheatsheet: the repo convention — dense tables, no prose. Cover the required-methods matrix, `armkit` subcommands with exit codes, the six traps, and the FK ladder.
- [ ] Commit.

---

# Workstream C — assembly

Depends on A and B.

### Task C1: `SKILL.md`

**Files:**
- Create: `skills/viam-arm-module/SKILL.md`

**State the scope plainly.** armkit supports building a Viam arm module — **one serial arm
chain, optionally with a tool attached**. It is not a general URDF validator. Multi-arm
robots, mobile bases, and humanoids are out of scope and should not be accommodated. A
dual-arm file like `mybuddy.urdf` is simply not a supported input; do not document how to
coax it through.

**State the platform requirements**, including that Windows ARM64 is unsupported and fails
during dependency resolution *before Python runs*, so armkit cannot catch or explain it.
Supported: Linux glibc/musl on x86_64/aarch64/armv7, macOS x86_64/arm64, Windows x64.

- [ ] **Step 1: Write frontmatter.** `name: viam-arm-module`. The `description` must trigger on arm-module authoring without stealing from `viam-go-motion-vision` (motion planning, frame system) or `viam-cpp`. Trigger on: building/porting an arm driver, kinematics files, URDF/SVA, `Get3DModels`, arm module scaffolding. Study the sibling skills' descriptions first; keep within Claude Desktop's length limit — commit `0c6254d` shortened descriptions for exactly this reason.
- [ ] **Step 2: Write the phase machine** — seven phases, each with artifact, gate, and exact gate command. Gates are skippable with evidence, not by assertion.
- [ ] **Step 3: Write the Phase 0 triage decision tree** — three on-ramps, and the FK ladder with its four rungs.
- [ ] **Step 4: Cross-reference, do not duplicate** — point at `viam-go-motion-vision`, `viam-cpp`, `viam-python`, `viam-modules-fleet`, and `local-viam-server` for machine bring-up.
- [ ] **Step 5: State the enforced opinions explicitly**, including that `ops-test` is advisory.
- [ ] **Step 6: Commit.**

### Task C2: Marketplace registration

**Files:**
- Create: `plugins/viam-arm-module/.claude-plugin/plugin.json`
- Create symlink: `plugins/viam-arm-module/skills/viam-arm-module -> ../../../skills/viam-arm-module`
- Modify: `.claude-plugin/marketplace.json`
- Modify: `README.md`

- [ ] **Step 1: Create the plugin manifest**, matching `plugins/viam-cpp/.claude-plugin/plugin.json`.
- [ ] **Step 2: Create the symlink** exactly as the siblings do (`ls -la plugins/viam-cpp/skills/` to confirm the form).
- [ ] **Step 3: Add the entry to `marketplace.json`** and bump `metadata.version`.
- [ ] **Step 4: Update `README.md`** — add to both skill tables, update the total line count, and extend the "How Skills Work Together" diagram.
- [ ] **Step 5: Verify** the plugin loads and the skill is listed.
- [ ] **Step 6: Commit.**

### Task C3: End-to-end dry run — simulated only

- [ ] **Step 1: Pick an arm not yet covered** — a Feetech or Waveshare arm from `~/src` with a URDF is ideal.
- [ ] **Step 2: Run Phases 0-2 following only `SKILL.md`.** Take notes wherever the skill is ambiguous or wrong.
- [ ] **Step 3: Confirm the Phase 2 gate genuinely passes** — the arm renders in the web app's 3D scene with its own meshes and a plan executes.
- [ ] **Step 4: Fix what the dry run exposed.** A workflow that has never been walked is a hypothesis.
- [ ] **Step 5: Commit fixes.**

### Task C4: End-to-end with hardware

- [ ] **Step 1: Run Phases 3-6 against a real arm.**
- [ ] **Step 2: Verify `fk-diff` passes** on hardware — this is the first real exercise of Task A11.
- [ ] **Step 3: Run `ops-test`** and record the actual result per language. If it fails in Python, that is evidence for gap 2, not a bug in the tool.
- [ ] **Step 4: Fix what hardware exposed** and commit.

---

## Follow-on work (not in this plan)

- File gaps 1-3 as SDK issues, `sdk-gaps.md` as the evidence base. Gap 1 is small enough to submit as a PR.
- Propose `armkit validate` for the `viam` CLI.
- Upstream `simplify` and `convert` fixes to `urdf-simplifier` and `urdf-to-sva-converter`.

## Open questions from the spec

- Whether to consolidate `motion-delegation-reference.md` into `driver-reference.md`. Decide after B4 and B6 are drafted and their real sizes are known.
- Whether the vendored tools should eventually be replaced by upstream-published ones.
