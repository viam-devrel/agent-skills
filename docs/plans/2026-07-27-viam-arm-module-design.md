# viam-arm-module Skill Design

An opinionated agentic workflow for building Viam arm modules in Go, C++, and Python.
One cross-language skill that drives an agent from "I have an arm" to "a published module
with correct kinematics, working 3D visualization, and sane cancellation semantics."

## Problem

Building a Viam arm module today requires assembling knowledge that exists nowhere in one
place: which kinematics file format to use and why, how to get mesh collision geometry to
work, what `Get3DModels` is for, how cancellation is supposed to behave, and which of
these the SDK gives you for free in your chosen language (the answer differs sharply by
language). The failure mode is not a compile error — it is a module that loads, plans, and
moves the arm to the wrong place.

Most of these failures are silent. A kinematics file with joints in the wrong order parses
fine. A URDF interpreted in meters instead of millimeters is off by 1000x and still loads.
A `continuous` joint becomes a `revolute` joint with infinite limits without complaint.

## Goals

- Make adding a new arm substantially easier than it is today, in all three languages.
- Encode the correct opinion at each decision point, not a menu of options.
- Produce a checkable artifact at every phase, so an agent can detect its own failures.
- Document what the SDKs are missing, with source-verified evidence, as a byproduct.

## Non-goals

- Deriving kinematics from CAD/STEP with no kinematic description. Out of scope.
- A module code generator. Templates rot across three SDKs and hide understanding the
  developer needs when FK is subtly wrong.
- Fixing the SDK gaps. This spec documents them and prototypes fixes; landing them
  upstream is follow-on work.

## Structure

One cross-language skill, `skills/viam-arm-module/`:

```
SKILL.md                 # the phase machine
references/              # knowledge, split by concern
scripts/                 # armkit toolkit
```

`SKILL.md` is procedural. `references/` is the knowledge it reaches into. SDK mechanics
that already exist elsewhere in this repo (`viam-go-motion-vision`, `viam-cpp`,
`viam-python`) are cross-referenced, not duplicated.

## The workflow

Seven phases. Each ends in a concrete artifact and a verification gate. Gates are
skippable **with evidence, not by assertion** — the "port an existing driver" on-ramp
often arrives with several phases already satisfied, and the skill verifies that rather
than blindly re-running them.

### Phase 0 · Triage → `docs/arm-brief.md`

Classify the on-ramp:

1. **Vendor URDF/xacro** — common for ROS-supported arms (UR, xArm, Kinova)
2. **Existing non-Viam driver** — ROS2 driver, LeRobot config, MuJoCo MJCF to port
3. **Vendor SDK/protocol only** — serial/TCP protocol plus a datasheet

Record DOF, joint order, units, limits, and tool frame. Two decisions get locked here:

**FK strategy**, as a ranked ladder:

1. Vendor controller reports TCP pose — cheapest, no dependency (UR, xArm)
2. Go: `model.Transform(joints)` — in-process, no network
3. Any language: delegate to the motion service — works everywhere, costs a gRPC hop
4. Hand-rolled FK — last resort

**Language.** Rungs 1 and 3 are available in every language, so the FK gap informs the
choice but does not force it. Go remains the path of least resistance when the controller
reports nothing, because rung 2 is free there.

**Gate:** brief written, both decisions recorded with justification.

### Phase 1 · Kinematics → `kinematics/<arm>.{urdf,json}`

**If the manufacturer ships a URDF, use the URDF as it is.** Do not convert. URDF is a
first-class format at the API level (`KINEMATICS_FILE_FORMAT_URDF`) and is the only
practical way to carry mesh collision geometry.

Author SVA JSON only when no URDF exists. Use inline DH parameters only when transcribing
a textbook DH table, and treat the result as a stepping stone, never a deliverable.

Simplify collision meshes so planning stays fast.

**Gate:** `uv run scripts/armkit.py validate` passes — parses, joint count matches the
driver, axes and limits sane, meshes resolve, FK at known configs matches expected poses.

### Phase 2 · Simulated model → `<ns>:<family>:<arm>-simulated`

Codifies the pattern already proven in `viam-dobot/arm/simulated.go` and the SO-101
module: a background goroutine interpolates joints against a realtime clock,
`MoveToJointPositions` blocks until done/stopped/cancelled, `MoveToPosition` delegates to
planning, `Get3DModels` serves the link meshes, `Kinematics` returns the shared model.

The reference implementations are Go, and the pattern needs translation per language. In
Python and C++ the interpolation loop becomes an async task or thread, and
`MoveToPosition` delegates to the motion service rather than calling `armplanning` in
process.

**Gap 1 blocks this phase's gate in Python.** Because the Python SDK has no
`get_3d_models`, a Python simulated model cannot serve visualization meshes, so "renders
with its own meshes" is unreachable there today. Until that gap is closed, the Python path
either degrades the gate to kinematics-and-geometry only (still visible in the 3D scene,
without custom visual meshes) or authors the simulated model in Go alongside a Python real
driver. The skill must state this explicitly rather than let an agent grind against an
impossible gate.

`meta.json` is authored here, declaring **both** models — it is required to exercise the
module against `viam-server` and the Viam web app at all.

Naming convention, from `viam-dobot`: `viam:dobot:cr10a` and `viam:dobot:cr10a-simulated`.

**Gate:** the arm renders in the web app's 3D scene with its own meshes, joints slew, and
a motion plan executes. No hardware involved.

### Phase 3 · Real driver → `<ns>:<family>:<arm>`

Config and `Validate`, connect/reconnect, joint read/write, `MoveToJointPositions`,
`MoveThroughJointPositions`, `Stop`, `IsMoving`, `Geometries`. `EndPosition` and
`MoveToPosition` follow the Phase 0 strategy.

**Gate:** module reloads on a real machine, joints read back from hardware, one commanded
move completes.

### Phase 4 · Operations and lifecycle

Cancellation semantics, blocking contracts, session-drop halt, reconnect, `Close`.
Per-language content, because the SDKs differ sharply here (see gap 2).

**Gate (advisory, non-blocking):** `armkit_live.py ops-test` — a second command cancels the
first; dropping the client session halts motion. Module development can complete without
this passing.

### Phase 5 · Hardware validation

**Gate:** `uv run scripts/armkit_live.py fk-diff` — sample N joint configs, command the
arm, diff reported `EndPosition` against computed FK. Pose error within tolerance across
the sample set. Verify joint limits are enforced and unit conventions hold.

### Phase 6 · Package and publish

Kinematics and meshes packaged with the binary, cross-compile, `viam module upload`.

**Gate:** clean-machine reload succeeds.

## Enforced opinions

- Kinematics before code
- Simulated model before hardware
- Every arm module ships both a real and a `-simulated` model
- URDF as-is when the vendor ships one; convert only under duress
- Cancellation is not optional (even though its test is)

## The toolkit

Scripts run via `uv` with PEP 723 inline dependency metadata — a new convention for this
repo, which currently uses stdlib-only scripts with lazy SDK imports. The dependency lift
is load-bearing: ROS URDFs overwhelmingly reference `.dae` (COLLADA) meshes, which the
stdlib cannot parse, so a stdlib-only toolkit would fail at the primary on-ramp.

```
scripts/
  armkit.py        # PEP 723: numpy, trimesh, pycollada
                   #   validate · meshes · simplify · convert
  armkit_live.py   # PEP 723: viam-sdk, numpy
                   #   fk-diff · ops-test
  _armkit/         # shared: urdf.py sva.py fk.py meshes.py
```

Two entry points so the offline loop stays fast and does not pull the SDK. `_armkit`
resolves via `sys.path[0]` without installation. `validate`, `simplify`, and `fk-diff` all
need URDF parsing and FK, so the shared package prevents three copies drifting apart.

| Subcommand | Gate | Purpose |
|---|---|---|
| `validate <file>` | Phase 1 (blocking) | Parse `.urdf`/`.json`; joint inventory, unit sanity, mesh resolution, unsupported joint types; FK at a config; `--expect` to diff known poses |
| `meshes <file>` | Phase 1–2 | Per-mesh triangle count, bounding box, origin offset, size, missing-file report |
| `simplify <urdf>` | Phase 1 | Port of `urdf-simplifier`: mesh collision geometry to bounding prisms |
| `convert <urdf>` | escape hatch | Vendored `urdf-to-sva-converter`; only when SVA is specifically required |
| `fk-diff <machine>` | Phase 5 (blocking) | Live: reported `EndPosition` vs computed FK across N configs |
| `ops-test <machine>` | Phase 4 (advisory) | Live: interrupt test and session-drop test |

Both `urdf-to-sva-converter` and `urdf-simplifier` are vendored (ported to Python for the
latter) rather than invoked as external tools, so the skill is self-contained. Fixes
should be upstreamed to their source repos.

### FK computation

`numpy` FK offline is the fast gate. `--cross-check` stands up RDK's built-in `sim` arm
against the same file and diffs the toolkit's FK against RDK's own, catching
implementation drift. This gives the built-in `sim` arm a role as a verification backend
rather than a workflow milestone.

### Traps `validate` exists to catch

Each of these produces a file that parses cleanly and plans wrongly:

- **Units.** URDF is meters/radians; Viam SVA is millimeters/degrees.
- **`continuous` joints.** RDK converts them to `revolute` with infinite limits.
- **Joint order.** Must match the driver's array order. Nothing checks this today.
- **Fixed-joint folding.** RDK folds `fixed` joints into static offsets; a tool frame
  behind one is silently not where you think.
- **Mesh weight.** Every collision triangle is checked during planning.
- **Frame identity.** Which link is the end/tool frame that `EndPosition` is relative to.

## References layout

| File | Contents |
|---|---|
| `kinematics-reference.md` | SVA vs URDF vs inline DH, model JSON schema, joint types, geometry primitives, units, the six traps, worked examples |
| `meshes-3d-reference.md` | Collision vs visual meshes, `Get3DModels` per language, formats, packaging |
| `driver-reference.md` | Per-language `Arm` surface, required-methods table, config/`Validate`, connect/reconnect |
| `operations-reference.md` | Cancellation per language, `SingleOperationManager`, `run_with_operation`, C++ hand-roll, blocking contracts |
| `motion-delegation-reference.md` | FK/IK via the motion service, frame-correctness trap, weak-dependency proof, latency budget |
| `simulated-arm-reference.md` | The dobot/SO-101 pattern codified, `-simulated` naming, `meta.json` with both models |
| `packaging-reference.md` | Embedding kinematics, cross-compile, upload, clean-machine reload |
| `cheatsheet.md` | Repo convention |
| `sdk-gaps.md` | The appendix below |

## Motion service delegation

The motion service closes both the FK and IK gaps for Python and C++. Three properties had
to hold, and all three are verified in RDK v1.0.0:

**No dependency cycle.** The motion builtin declares components as *weak* dependencies
(`services/motion/builtin/builtin.go:46`); its only hard `Validate` dependency is the
internal framesystem service. An arm module can hard-depend on motion without forming a
graph edge back.

**No recursion on IK.** Motion executes plans through `framesystem.InputEnabled` →
`GoToInputs`, never `MoveToPosition`. So `arm.MoveToPosition → motion.Move(arm) → plan →
arm.GoToInputs` terminates.

**No recursion on FK.** `framesystem.GetPose(component, dst)` is
`TransformPose(zero-pose-in-component-frame → dst)`, resolved from `CurrentInputs` — joint
positions. It never calls `EndPosition`.

Three caveats the skill must encode:

- **Frame correctness.** `EndPosition`'s contract is the pose of the end effector relative
  to the arm's *own base*. `GetPose(arm, "world")` is world-relative — identical only when
  the arm sits at the world origin. Getting this wrong looks correct on a bench and breaks
  in a real work cell. Transform into the arm's base frame explicitly.
- **Latency.** `get_end_position` may be polled at data-capture frequency; delegation makes
  every call a network hop. Cache against joint state or accept the cost knowingly.
- **Runtime coupling.** The arm degrades if the motion service is not configured, and must
  be present in the machine's frame system. State this in the module README.

## SDK gaps

Source-verified against `viam-api` protos, RDK v1.0.0, `viam-cpp-sdk`, and
`viam-python-sdk`. Ranked by fixability.

| # | Gap | Evidence | Severity | Upstream ask |
|---|---|---|---|---|
| 1 | `Get3DModels` missing from Python SDK entirely | In proto; in Go `Arm` interface; `= 0` in C++ `arm.hpp:133`; absent from `viam-python-sdk` | High | Purely additive. File first. Python arm modules cannot serve visualization meshes at all — this blocks the Phase 2 gate for Python. |
| 2 | No single-flight operation manager outside Go | Go: `operation.SingleOperationManager`. Python: `viam/operations.py` provides `Operation`/`run_with_operation` — self-cancellation only, no cancel-other. C++: nothing module-side; `RobotClient` operation methods are the client API | High (C++), Medium (Python) | The Go `Arm` docs promise "blocks until done or a new operation cancels this one" — a contract only Go can honor out of the box |
| 3 | No FK/IK outside Go | `get_end_position`/`move_to_position` mandatory in both Python and C++; neither ships FK | Medium — mitigated, not closed, by motion delegation | Expose FK from a parsed model in Python/C++ |
| 4 | No kinematics validation anywhere; no published SVA schema | Nothing in any SDK or the CLI | Medium | `viam kinematics validate` CLI subcommand; `armkit` is the prototype |
| 5 | Built-in `sim` arm's 3D models are a compiled-in table | `models3d.ArmTo3DModelParts` covers 7 arms; unknown model returns an empty mesh map | Medium | Let `sim` load meshes from a path |
| 6 | SVA cannot practically reference mesh files | `GeometryConfig.MeshData` requires inline bytes; `MeshFilePath` is documented as URDF round-trip only | Low | Documents why URDF-first is correct |
| 7 | No arm conformance suite in any SDK | — | Low | `ops-test`/`fk-diff` are the prototype |

### Language capability matrix

| | FK in-process | `Get3DModels` | Self-cancellation | Single-flight cancel |
|---|---|---|---|---|
| Go | yes (`model.Transform`) | required interface method | yes | yes (`SingleOperationManager`) |
| C++ | no | required (`= 0`) | no | no |
| Python | no | **absent from SDK** | yes (`run_with_operation`) | no |

## Ecosystem integration

Three routes, cheapest first:

1. **Ship here.** The skill is consumed via this marketplace repo. No coordination needed.
   *In scope for this spec.*
2. **File gaps 1–3** as SDK issues, with `sdk-gaps.md` as the evidence base. Gap 1 is small
   enough to submit as a PR. *Follow-on.*
3. **Graduate `armkit validate`** into the `viam` CLI, and the phase structure into an
   official arm-module template, if the workflow proves out. *Follow-on.*

## Ground truth

Patterns are extracted from source, per this repo's convention:

- RDK built-in arms (`components/arm/*`), `referenceframe`, `services/motion/builtin`,
  `robot/framesystem`
- Published arm modules: `universal-robots` (C++), `viam-ufactory-xarm` (Go),
  `viam-dobot` (Go, including its simulated model), `viam-mycobot`,
  `viam-waveshare-roarm`
- Arm surfaces across all three SDKs, which doubles as the gap analysis

## Open questions

- Whether to consolidate `motion-delegation-reference.md` into `driver-reference.md`. Nine
  reference files is more than sibling skills carry (2–3 each), though this skill spans
  three languages and seven phases.
- Whether `armkit`'s vendored converter and simplifier should eventually be replaced by
  upstream-published tools (gap 4, route 3).
