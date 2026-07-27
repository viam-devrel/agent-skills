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

## Conventions

- **Running:** `uv run skills/viam-arm-module/scripts/armkit.py <subcommand>`. PEP 723
  headers make this work with no install step. This is a new convention for the repo,
  which currently uses stdlib-only scripts with lazy imports — the design doc explains why.
- **Testing:** `uv run --project skills/viam-arm-module/scripts pytest`. The `pyproject.toml`
  exists for development only; end users never need it.
- **Every script** carries a "Why this exists" docstring naming the gap it fills, matching
  `skills/local-viam-server/machine_up.py`.
- **Units:** URDF is meters/radians. Viam is millimeters/degrees. The parsed `KinematicModel`
  is **always millimeters and radians internally**; conversion happens at parse and at
  display. Every test asserts units explicitly.

---

# Workstream A — armkit

### Task A1: Scaffold and fixtures

**Files:**
- Create: `skills/viam-arm-module/scripts/pyproject.toml`
- Create: `skills/viam-arm-module/scripts/_armkit/__init__.py`
- Create: `skills/viam-arm-module/scripts/tests/conftest.py`
- Create: `skills/viam-arm-module/scripts/tests/fixtures/two_link.urdf`

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

Run: `uv run --project skills/viam-arm-module/scripts pytest -q`
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

Run: `uv run --project skills/viam-arm-module/scripts pytest tests/test_model.py -q`
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

Run: `uv run --project skills/viam-arm-module/scripts pytest tests/test_model.py -q`
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

Run: `uv run --project skills/viam-arm-module/scripts pytest tests/test_urdf.py -q`
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

Run: `uv run --project skills/viam-arm-module/scripts pytest tests/test_urdf.py -q`
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
- Test: `skills/viam-arm-module/scripts/tests/test_fk.py`

This is the task that closes the gap the spec names: FK exists only in Go's
`referenceframe`, so Python and C++ module authors have no way to check their own model.

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

Run: `uv run --project skills/viam-arm-module/scripts pytest tests/test_fk.py -q`
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

Run: `uv run --project skills/viam-arm-module/scripts pytest tests/test_fk.py -q`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
git add skills/viam-arm-module/scripts/_armkit/fk.py skills/viam-arm-module/scripts/tests/test_fk.py
git commit -m "feat(armkit): forward kinematics"
```

---

### Task A5: SVA and DH model JSON parser

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

**Folding rule:** for a serial chain, joint `J`'s origin is the transform of the link that
is `J`'s parent. So build the chain by following `parent` links across both node types,
then emit one `Joint` per SVA joint whose `origin` comes from its parent link, plus a
trailing `fixed` joint carrying the final link's transform (`ee_link` above) so the tip
frame is right. SVA joints have no `child` field — derive it from the link that names the
joint as its parent.

Orientations use Viam's orientation vector (`{"type": "ov_degrees", "value": {x, y, z,
th}}`), not RPY. Implement OV-to-matrix conversion in `_armkit/sva.py`; the authority is
`spatialmath/orientation_vector.go`. Cover it with its own unit test against a known
rotation before wiring it into parsing.

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

Run: `uv run --project skills/viam-arm-module/scripts pytest tests/test_sva.py -q`
Expected: FAIL — no module `_armkit.sva`

- [ ] **Step 3: Implement**

Mirror RDK's `model_json.go` structure: `links[]` carry `id`, `parent`, `translation`,
`orientation`, and optional `geometry`; `joints[]` carry `id`, `type`, `parent`, `axis`,
`max`, `min`. Reuse `_armkit.model` types. Convert degree limits to radians. Build the
orientation matrix from Viam's orientation vector (`OrientationConfig`), and raise on
unknown `kinematic_param_type` with a message matching RDK's own wording.

Error text must match the test: `supported params are SVA and DH`.

- [ ] **Step 4: Run tests**

Run: `uv run --project skills/viam-arm-module/scripts pytest tests/test_sva.py -q`
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
    m = parse_urdf(fixtures / "ur20.urdf")
    report = inspect_meshes(m)
    assert all(r.path for r in report)
    # ur20.urdf ships without mesh files in this repo, so all are unresolved.
    assert any(not r.resolved for r in report)

def test_no_meshes_is_empty_not_an_error(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    assert inspect_meshes(m) == []
```

- [ ] **Step 3: Run to verify it fails**

Run: `uv run --project skills/viam-arm-module/scripts pytest tests/test_meshes.py -q`
Expected: FAIL — no module `_armkit.meshes`

- [ ] **Step 4: Implement**

`inspect_meshes(model) -> list[MeshReport]` where `MeshReport` carries `link`, `path`,
`kind` (`collision`/`visual`), `resolved`, and — when resolved — `triangles`,
`bbox_mm`, `origin_offset_mm`, `bytes`. Import `trimesh` lazily inside the function so
parsing-only callers pay nothing.

- [ ] **Step 5: Run tests**

Run: `uv run --project skills/viam-arm-module/scripts pytest -q`
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

Run: `uv run --project skills/viam-arm-module/scripts pytest tests/test_cli.py -q`
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
| `heavy-mesh` | warn | resolved collision mesh over 5 000 triangles |
| `nonunit-axis` | error | axis that is not unit length after normalization attempt |
| `dh-format` | warn | `kinematic_param_type: "DH"` — supported but not recommended |

Print a summary line (`<name>: <n> actuated joints, base <base> -> tip <tip>`), then each
finding, then `PASS` or `FAIL`.

- [ ] **Step 5: Run tests**

Run: `uv run --project skills/viam-arm-module/scripts pytest -q`
Expected: PASS (all)

- [ ] **Step 6: Verify it runs standalone via uv**

Run: `uv run skills/viam-arm-module/scripts/armkit.py validate skills/viam-arm-module/scripts/tests/fixtures/ur20.urdf`
Expected: exit 0, summary reporting 6 actuated joints and `unresolved-mesh` warnings.

This is the real user path — PEP 723 with no install step. It must work before committing.

- [ ] **Step 7: Commit**

```bash
git add skills/viam-arm-module/scripts
git commit -m "feat(armkit): validate subcommand with kinematics trap checks"
```

---

### Task A8: `armkit.py meshes`

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
