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
of scope -- a dual-arm URDF (e.g. `mybuddy.urdf`) is not a supported input, and
nothing here should be stretched to accommodate it. If the file describes
more than one arm, stop and say so rather than trying to make it fit.

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
| 0 | Triage | `docs/arm-brief.md` | On-ramp classified, DOF/joint order/units/limits/tool frame recorded, FK strategy and language both decided with justification | None -- manual |
| 1 | Kinematics | `kinematics/<arm>.urdf` (or `.json` once SVA lands) | `armkit validate` passes | **`armkit validate` -- real, run it** |
| 2 | Simulated model | `<ns>:<family>:<arm>-simulated` | Renders in the web app's 3D scene with its own meshes, joints slew, a motion plan executes -- no hardware | None -- manual verification in the web app |
| 3 | Real driver | `<ns>:<family>:<arm>` | Module reloads on real hardware, joints read back, one commanded move completes | None -- manual |
| 4 | Operations & lifecycle | Cancellation, blocking, session-drop halt, reconnect, `Close` | **Advisory, non-blocking:** a second command cancels the first; dropping the client session halts motion | Planned (`armkit_live.py ops-test`) -- not built. Module development can complete without this gate passing. |
| 5 | Hardware validation | -- | Pose error within tolerance across N sampled joint configs; limits enforced; units hold | Planned (`armkit_live.py fk-diff`) -- not built. Do this by hand: command known configs, compare `EndPosition` against expected poses. |
| 6 | Package & publish | Uploaded module | Clean-machine reload succeeds | None -- use `viam module upload` (see `viam-modules-fleet`) |

Gates are skippable **with evidence, not by assertion** -- "porting an existing
driver" often arrives with several phases already satisfied (a working URDF,
a driver that already reads joints). Check that the evidence actually
satisfies the gate; don't re-run a phase that's already proven, and don't
skip one that isn't.

## Phase 0: two decisions

Classify the on-ramp, then lock two decisions before writing `docs/arm-brief.md`:

**On-ramp** (one of):
1. **Vendor URDF/xacro** -- common for ROS-supported arms (UR, xArm, Kinova)
2. **Existing non-Viam driver** -- ROS2 driver, LeRobot config, MuJoCo MJCF to port
3. **Vendor SDK/protocol only** -- serial/TCP protocol plus a datasheet

**FK strategy**, ranked cheapest first:
1. Vendor controller reports TCP pose directly -- no dependency (UR, xArm)
2. Go: `model.Transform(joints)` -- in-process, no network hop
3. Any language: delegate to the motion service -- works everywhere, costs a gRPC hop per call
4. Hand-rolled FK -- last resort

**Language** follows from the FK ladder but isn't forced by it: rungs 1 and 3
work in any language, so the FK gap is a preference signal, not a hard
constraint. Go remains the path of least resistance when the controller
reports nothing, because rung 2 is free there and Python/C++ have no
in-process FK today.

Record DOF, joint order, units, limits, and tool frame in the brief. Gate:
brief written, both decisions recorded with justification.

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
describe only the arm, so any actuated joint that isn't on the path to the
declared tip is an error, not a warning -- it usually belongs to the gripper
or a second tool, not the arm. In most vendor URDFs the off-chain joints are
a single `<joint>` element -- the gripper's actuator -- so the fix is usually
one small edit rather than restructuring the file.

**An armkit PASS does not guarantee RDK will load the file.** Two known,
deliberate divergences: RDK hard-fails when a mesh file it references can't
be resolved on disk, while `armkit` counts mesh references but does not yet
resolve paths (mesh support is unbuilt). And RDK *panics* on a joint with no
`<origin>` element, while `armkit` follows the URDF spec and correctly
defaults it to identity -- so `armkit` will happily PASS a file that crashes
RDK. When `armkit` PASSes cleanly (no `mesh-references`/`missing-origin`
findings), its JSON output's `rdk_parity.guaranteed` field reflects this
directly; otherwise treat a PASS as necessary, not sufficient.

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
  `viam:dobot:cr10a` / `viam:dobot:cr10a-simulated`), declared together in
  `meta.json`.
- **URDF as-is when the vendor ships one.** Do not convert to SVA unless
  forced -- URDF is the only practical way to carry mesh collision geometry.
- **Cancellation is not optional**, even though its test (`ops-test`, Phase 4)
  is advisory. A module can ship without `ops-test` passing; it should not
  ship with cancellation unimplemented.

## Cross-references

Point at these rather than duplicating their content:

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
