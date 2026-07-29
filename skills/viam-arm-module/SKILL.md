---
name: viam-arm-module
description: >
  Guides building a Viam arm module -- one serial arm chain, optionally with a
  tool -- from kinematics through a simulated model, real driver, and
  packaging, in Go, Python, or C++. Use whenever a developer is building or
  porting an arm driver, authoring or troubleshooting arm kinematics files
  (URDF, SVA, DH), running `armkit validate`, implementing
  `GetKinematics`/`Get3DModels` for an arm, or scaffolding a new arm module.
  Not for motion planning or frame-system work on an existing arm
  (viam-go-motion-vision owns that), general C++ SDK questions (viam-cpp), or
  CLI/registry/fleet operations (viam-modules-fleet).
---

# Viam Arm Module Skill

You drive an agent through building a Viam arm module: a phase machine, backed
by `armkit`, an offline kinematics validator.

## Scope

**One serial arm chain, optionally with a tool or gripper attached.** Not a
general URDF validator. Multi-arm robots, mobile bases, and humanoids are out
of scope -- a dual-arm robot such as the Elephant Robotics myBuddy is not a
supported input, and nothing here should be stretched to accommodate it. If
the file describes more than one arm, stop and say so rather than trying to
make it fit.

RDK parity governs kinematic *fidelity* (poses, mimic composition, orientation
conventions) -- `armkit` matches RDK there. RDK parity does **not** govern
*scope*: RDK models whole robots, an arm module describes one arm. Where the
two disagree, the arm module wins. This is why `armkit` rejects a gripper's
joints riding along in the arm's kinematics (`joints-off-chain`, below) even
though RDK itself would happily count them.

## The seven phases

Each phase produces one artifact and has one gate. **Only Phase 1's gate is
executable today** (`armkit validate`). Every other gate is a documented
manual check -- there is no `armkit meshes`, `simplify`, `convert`, `fk-diff`,
or `ops-test` yet. Do not tell a user or act as if those subcommands exist;
run `uv run skills/viam-arm-module/scripts/armkit.py --help` if you need to
confirm what's actually implemented.

| # | Phase | Artifact | Gate | Tooling |
|---|---|---|---|---|
| 0 | Triage | `docs/arm-brief.md` | On-ramp classified, DOF/joint order/units/limits/tool frame recorded, FK strategy, language, and kinematics-file format all decided with justification | None -- manual |
| 1 | Kinematics | `kinematics/<arm>.urdf` (or `.json` once SVA lands) | `armkit validate` passes | **`armkit validate` -- real, run it** |
| 2 | Simulated model | `<ns>:<family>:<arm>-simulated` | Renders in the web app's 3D scene with its own meshes, joints slew, a motion plan executes -- no hardware. **Python: collision meshes yes, visual `.glb` meshes no, see note below** | None -- manual verification in the web app |
| 3 | Real driver | `<ns>:<family>:<arm>` | Module reloads on real hardware, joints read back, one commanded move completes | None -- manual |
| 4 | Operations & lifecycle | Cancellation, blocking, session-drop halt, reconnect, `Close` | **Advisory, non-blocking.** Cancel-other is Go-only -- see note below | Planned (`armkit_live.py ops-test`) -- not built. |
| 5 | Hardware validation | -- | Pose error within tolerance across N sampled joint configs; limits enforced; units hold | Planned (`armkit_live.py fk-diff`) -- not built. Do this by hand: command known configs, compare `EndPosition` against expected poses. |
| 6 | Package & publish | Uploaded module | Clean-machine reload succeeds | None -- use `viam module upload` (see `viam-modules-fleet`) |

Gates are skippable **with evidence, not by assertion** -- "porting an existing
driver" often arrives with several phases already satisfied (a working URDF,
a driver that already reads joints). Check that the evidence actually
satisfies the gate; don't re-run a phase that's already proven, and don't
skip one that isn't.

### Two gates are not a uniform bar across languages

**Phase 2's mesh gate is two channels in Python -- one works, one doesn't.**
`GetKinematics`'s `meshes_by_urdf_filepath` **is** supported by the Python
SDK, and RDK's `UnmarshalModelXML` consumes it for motion planning's
collision geometry -- a Python simulated model can ship real `.stl`/`.dae`
meshes today. `Get3DModels` is the gap: absent from the Python SDK's `Arm`
service dispatch entirely, so Python can't ship the `.glb` render assets the
web app's 3D scene shows. Don't grind against `Get3DModels`; degrade the
gate to kinematics-and-collision-geometry only (correct collision volumes in
the 3D scene, no visual mesh), or author the simulated model in Go alongside
a Python real driver. `references/driver-reference.md` §4-§5 has the
mesh-map key/content-type convention, a verified working pattern, and the
trap in the SDK's own example code that looks like it implements
`Get3DModels` but is never called.

**Phase 4's cancel-other behavior is native only in Go**
(`operation.SingleOperationManager`). Python's `run_with_operation` gives
self-cancellation only, with no way to cancel a different in-flight
operation. C++ has no operation manager module-side at all -- expect to
hand-roll single-flight cancellation there. Since this gate is already
advisory, a Python or C++ module can ship without it passing, but don't
imply cancel-other is a small lift in either language.

## Phase 0: three decisions

Classify the on-ramp, then lock three decisions before writing `docs/arm-brief.md`:

**On-ramp** (one of):
1. **Vendor URDF/xacro** -- common for ROS-supported arms (UR, xArm, Kinova)
2. **Existing non-Viam driver** -- ROS2 driver, LeRobot config, MuJoCo MJCF to port
3. **Vendor SDK/protocol only** -- serial/TCP protocol plus a datasheet
4. **Existing Viam module, different language** -- e.g. porting a Go arm
   module to Python. Uniquely gives you a working reference for behavior,
   kinematics files already in a Viam-supported format, and Phases 0-1
   largely pre-answered -- but not the driver itself, since the target SDK
   differs from the one the reference module was built on.

**FK strategy**, ranked cheapest first:
1. Vendor controller reports TCP pose directly -- no dependency (UR, xArm)
2. Go: `model.Transform(joints)` -- in-process, no network hop
3. Any language: delegate to the motion service -- works everywhere, costs a gRPC hop per call
4. Hand-rolled FK -- last resort

These rungs describe what the vendor controller and language runtime can
do, not what an existing module actually uses. The xArm controller reports
TCP pose directly (rung 1), yet the Go xArm module computes `EndPosition`
via `x.model.Transform(joints)` (rung 2) anyway, because rung 2 costs
nothing in Go. Don't read a reference module's implementation as evidence
the ladder is wrong -- read it as a language-specific choice within it.

**The ladder informs language choice; it doesn't force it.** Rung 2
(`model.Transform`) is Go-only; rungs 1 and 3 work in any language. So when
the controller reports pose directly or a gRPC hop to the motion service is
acceptable, language is a preference, not a constraint. Go remains the path
of least resistance only when the controller reports nothing at all --
rung 2 is free there, while Python and C++ have no in-process FK today.

**Kinematics-file format**: URDF when the vendor ships one, or whenever
there's mesh collision geometry to carry -- this is the default and needs
no justification (see Enforced opinions, below). SVA only when forced: no
URDF exists and the source doesn't map to one. Record which and why.

Record DOF, joint order, units, limits, and tool frame in the brief. Gate:
brief written, on-ramp classified, and all three decisions recorded with
justification.

### From a validated file to a module's kinematics

Phase 1 produces a file; Phase 2's driver code has to serve it.
`GetKinematics` returns the same file `armkit validate` just checked --
embedded in (or shipped alongside) the module binary, read once at startup.
`references/driver-reference.md` §4 has the verified Python
`get_kinematics()`/mesh-map convention; per-language packaging mechanics
beyond that remain a Phase 6 concern -- until then, read the file straight
from disk relative to the module's working directory.

**Units change at each of these boundaries, and converting between them is
the module author's job -- nothing in the SDK does it for you:**

| boundary | units |
|---|---|
| URDF source | metres, radians |
| vendor SDK (e.g. xArm) | millimetres, degrees |
| Viam arm API (`JointPositions`, `Pose`) | millimetres, degrees |

Go's `referenceframe.Input` is radians internally, but a Python or C++
driver talks to the Viam API directly, in millimetres and degrees -- get a
conversion wrong here and the arm moves to a wildly wrong position with no
error raised.

**Converting orientation is a representation change, not a units problem.**
Vendor SDKs typically report roll/pitch/yaw; Viam's `Pose` uses an
orientation vector -- `viam.spatialmath` (ships with the Python SDK,
FFI-backed) converts between them correctly via `EulerAngles`/`Quaternion`.
Use `Quaternion`'s `w`/`i`/`j`/`k` accessors, never `RotationMatrix.elements`:
its buffer layout changed between viam-sdk 0.79.2 and 0.80.0 and will
silently transpose the rotation on whichever version you didn't test against
(`references/driver-reference.md` §7).

## Using armkit

```
uv run skills/viam-arm-module/scripts/armkit.py validate <file> [--at Q1,Q2,...] [--expect-dof N] [--tip LINK] [--json]
```

Only `.urdf` is implemented today -- `.json`/SVA exits 2 with a "not yet
supported" message (`_armkit/sva.py` is planned, unwritten). `--tip` declares
the end-effector link when the model has more than one leaf. `--at` prints
the tip pose (mm + wxyz quaternion) at a joint configuration; its value count
must match DOF. `--expect-dof` fails the run if DOF doesn't match.

**Exit-code contract, depend on it:**
- `0` -- PASS, no errors (warnings may still be present)
- `1` -- FAIL, findings reported (parse errors, structural errors, limit/unit problems)
- `2` -- usage or environment problem (bad CLI args, unsupported file extension, native-library load failure) -- not a statement about the file's kinematics

`--json` emits a machine-readable report instead of text. **Handle unknown
finding `code`s by `level`** (`error`/`warn`), not by switching on `code` --
new codes are added over time and are additive, not breaking.

### Two things you will hit first

**A vendor URDF with a gripper attached branches.** `validate` reports a
`structure` error ("need exactly one end effector, have [...]") and suggests
`--tip <link>` at the first fork point. Re-running with that `--tip` then
typically surfaces `joints-off-chain`: a Viam arm module's kinematics must
describe only the arm, so any actuated joint not on the path to the declared
tip is an error.

**Fixing it means removing the gripper subtree, not just the joints armkit
names.** armkit lists only *actuated* off-chain joints; a gripper's other
joints are usually `fixed` or `mimic`, so they aren't listed but still must
go. Deleting only the named joint will break the file -- on a real mycobot
gripper URDF, five other joints `<mimic>` the one armkit names, and removing
it alone turns a `joints-off-chain` error into a `parse` error.

Work from the tip instead of from the list:

1. Copy the vendor file to `kinematics/<arm>.urdf` -- edit the copy, never
   the vendor original.
2. Delete every `<joint>` whose `<parent link>` is the declared tip or any
   link downstream of it, plus the `<link>` elements they introduce.
3. Re-run `validate` with no `--tip`. The trimmed arm should now have a
   single leaf and auto-resolve.

On that mycobot file the edit is 6 joints and 6 links, and the result passes
with `6 actuated joints, base base -> tip gripper_base`.

**An armkit PASS does not guarantee RDK will load the file.** Two known,
deliberate divergences: RDK hard-fails when a mesh file it references can't
be resolved on disk, while `armkit` counts mesh references but does not yet
resolve paths (mesh support is unbuilt). And RDK *panics* on a joint with no
`<origin>` element, while `armkit` follows the URDF spec and correctly
defaults it to identity -- so `armkit` will happily PASS a file that crashes
RDK. When `armkit` PASSes cleanly (no `mesh-references`/`missing-origin`
findings), its JSON output's `rdk_parity.guaranteed` field reflects this
directly; otherwise treat a PASS as necessary, not sufficient.

## Phase 3: behaviour over structure

**When porting from a reference implementation in another language, take its
behaviour, not its structure**: controller semantics, error/fault codes,
what `Stop` must do to the hardware, and speed/acceleration bounds carry
across languages; registration mechanics, the concurrency model, resource
lifecycle, and how FK gets computed do not.
`references/driver-reference.md` §1 has the worked case -- porting the Go
xArm module's structure into Python would have added an unneeded
dependency, an unexecutable FK strategy, and reinvented trajectory
smoothing.

**The arm method set is not uniform across languages, either.** Python's
`Arm` ABC lacks `move_through_joint_positions` entirely -- no ABC method, no
RPC dispatch -- though it's present on both Go's and C++'s `Arm`; combined
with `get_3d_models`'s absence (Phase 2, above), don't assume method parity
when porting. `references/driver-reference.md` §2 has the full per-language
method table.

## Requirements

Python 3.11+, `uv`. `armkit.py` is a single PEP 723 file -- `uv run` fetches
its dependencies (`numpy`, `trimesh`, `pycollada`, `viam-sdk`) on first run,
no install step.

Supported platforms: Linux glibc/musl on x86_64/aarch64/armv7, macOS
x86_64/arm64, Windows x64. **Windows ARM64 is unsupported** -- there's no
matching wheel and no sdist, so resolution fails inside `uv` before Python
even starts. `armkit` cannot catch or explain this: if a user hits a raw `uv`
resolver error on Windows ARM64, that's the reason, not a bug in the script.

## Enforced opinions

- **Kinematics before code.** Phase 1 blocks Phase 2 for a reason -- don't start
  driver code against an unvalidated model.
- **Simulated model before hardware.** Phase 2 blocks Phase 3.
- **Every arm module ships both a real and a `-simulated` model** (naming:
  `viam:dobot:cr10a` / `viam:dobot:cr10a-simulated`). `meta.json` can't
  declare both before both exist: it gains the simulated model at Phase 2
  and the real model at Phase 3, so "declared together" is the state Phase 6
  ships, not a Phase 2 gate. (The reference Go module this skill points at,
  `viam-ufactory-xarm`, declares no simulated model at all -- that's the gap
  this opinion exists to close, not evidence the opinion is optional.)
- **URDF as-is when the vendor ships one.** Do not convert to SVA unless
  forced -- URDF is the only practical way to carry mesh collision geometry.
- **Cancellation is not optional**, even though its test (`ops-test`, Phase 4)
  is advisory. A module can ship without `ops-test` passing; it should not
  ship with cancellation unimplemented.

## Cross-references

Point at these rather than duplicating their content:

- **`references/driver-reference.md`** -- this skill's own depth: structure
  vs. behaviour when porting, the per-language arm method-set table, FK/IK
  rung selection in practice, the two mesh channels, blocking-vendor-SDK
  cancellation, and `viam.spatialmath`, all written from a real Python xArm
  port
- **`viam-go-motion-vision`** -- motion planning, frame systems, `PlanRequest`,
  motion-service delegation for FK/IK once the arm exists
- **`viam-cpp`** -- C++ SDK mechanics generally (module registration, CMake,
  threading) beyond arm-specific patterns
- **`viam-python`** -- Python SDK mechanics generally (async client patterns,
  `EasyResource`, module registration)
- **`viam-modules-fleet`** -- `viam module build`/`upload`, `meta.json`
  authoring mechanics, registry operations, fleet deployment (Phase 6)
- **`local-viam-server`** -- bringing up a machine and local `viam-server` to
  test against (needed before Phase 2's web-app gate is checkable)
