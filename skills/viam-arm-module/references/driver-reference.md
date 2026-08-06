# Arm Driver Reference

> **Provenance convention.** Every substantive claim below is tagged.
> `[verified]` — run or measured, in this session or a session producing
> the evidence this reference draws on (a real Python port of the
> UFactory xArm 6, built end-to-end against this skill). `[source]` — read
> directly from SDK/RDK/module source, with a `file:line` citation, but
> not executed. Nothing here is "should work" or "probably" — if a claim
> can't be pinned to one of the two tags, it isn't in this document.
> Versions checked: `viam-sdk` 0.79.2 and 0.80.0 (PyPI), plus a
> `viam-python-sdk` git checkout ~381 commits past `v0.34.0`;
> `xarm-python-sdk` 1.18.4; `go.viam.com/rdk` `v1.0.0`; `viam-cpp-sdk`
> `releases/v0.35.0-34-gd37e2eee`; `universal-robots` (C++ module)
> `b746241a09f6`.
>
> This document exists because reading an SDK and writing down what it
> looks like it does are not the same activity, and the gap between them
> is where driver bugs live. Every table row and code fragment here was
> either run against the thing it describes, or is explicitly marked as
> read-only and unexecuted. Where C++ coverage is thin, it's thin on
> purpose — no C++ port was built to verify against, and a confident guess
> would be worse than an honest gap.

---

## 1. Structure vs. behavior — read this before porting anything

The single most consequential mistake available when porting an arm driver
from one language to another is copying the reference implementation's
**structure** instead of its **behavior**. They look similar enough from
the outside — both are "an Arm driver for this vendor" — that it's easy to
not notice you're doing it.

**Take from a reference implementation**: which controller registers or
SDK calls actually matter, error/fault-code semantics, what `Stop` must
concretely do to the hardware (not just "stop" — does it also need to
re-arm the arm so the next command works?), speed/acceleration bounds and
defaults, the connect/reconnect state machine, how a fault state gets
detected and cleared before the next move. This is vendor and hardware
domain knowledge. It is expensive to rediscover and it does not change
with implementation language.

**Do not take**: module registration mechanics, the concurrency model,
resource lifecycle, how `EndPosition`/`GetEndPosition` gets computed, how
`MoveToPosition` is satisfied, whether a motion-service dependency is
needed. These are answers to *language-specific* questions, and a
different language can have a strictly better answer available — not just
a different one.

`[verified]` Concretely, porting the Go `viam-ufactory-xarm` module's
*structure* into Python would have produced a worse driver in three
specific ways:

1. **An unneeded dependency.** The Go module hard-requires a `motion`
   service dependency to implement `MoveToPosition`
   (`"xarm cannot do MoveToPosition without specifying a motion
   service"` — the Go module's own error string) because Go's arm has no
   native Cartesian-move primitive over its hand-rolled Modbus protocol.
   `xarm-python-sdk`'s `set_position()` *is* a native Cartesian move — a
   direct wire command, same tier as its `get_position()`. A Python
   port that copied Go's structure would carry a dependency it doesn't
   need.
2. **An FK strategy Python cannot execute.** The Go module's `EndPosition`
   computes pose via `x.model.Transform(joints)` — in-process forward
   kinematics against its own embedded kinematics model. There is no
   equivalent in Python or C++ today (§6 has the full FK/IK story). A
   line-by-line port would either not compile (no `Transform` to call) or
   get quietly reimplemented as a from-scratch FK solver — hand-rolled FK,
   the ladder's last resort, for a problem the vendor SDK already solves
   for free via a direct controller query.
3. **Reinvented trajectory smoothing.** The Go module hand-rolls
   trapezoidal velocity-profile interpolation between waypoints
   (`comm.go`'s `createRawJointSteps`) because it has to — it's streaming
   raw joint targets over Modbus at a fixed rate and the controller does
   no smoothing of its own. `xarm-python-sdk` exposes speed/acceleration
   parameters directly on `set_servo_angle`/`set_position`, so the
   controller does this itself. Porting Go's interpolation code into
   Python would be strictly redundant, several hundred lines of surface
   area for a problem the vendor SDK doesn't have.

The rule of thumb: if the reference implementation's code exists *because*
of something about its language or its wire protocol, it's structure —
leave it. If it exists because of something about the physical arm, it's
behavior — port it.

---

## 2. Required-method set, per language

`[verified]` (Python) / `[source]` (Go, C++) — see citations per cell.

| Capability | Go (`Arm` interface) | Python (`Arm` ABC) | C++ (`Arm` class) |
|---|---|---|---|
| End-effector pose | `EndPosition` | `get_end_position` (abstract) | `get_end_position` (pure virtual) |
| Move to pose | `MoveToPosition` | `move_to_position` (abstract) | `move_to_position` (pure virtual) |
| Get joint positions | `JointPositions` | `get_joint_positions` (abstract) | `get_joint_positions` (pure virtual) |
| Move to joint positions | `MoveToJointPositions` | `move_to_joint_positions` (abstract) | `move_to_joint_positions` (pure virtual) |
| Move through waypoints | `MoveThroughJointPositions` | **absent — no ABC method, no RPC dispatch** | `move_through_joint_positions` (pure virtual) |
| Stop | `Stop` (via `resource.Actuator`) | `stop` (abstract) | `stop` (via `Stoppable`) |
| Is moving | `IsMoving` (via `resource.Actuator`) | `is_moving` (abstract) | `is_moving` (pure virtual) |
| Kinematics | `Kinematics` (via `framesystem.InputEnabled`) | `get_kinematics` (abstract) | `get_kinematics` (pure virtual) |
| Collision geometry | `Geometries` (via `resource.Shaped`) | `get_geometries` — **optional**, default raises `MethodNotImplementedError` | `get_geometries` (pure virtual — **mandatory**) |
| 3D visual models | `Get3DModels` | **absent — no ABC method, no RPC dispatch** (proto stub exists, unreachable) | `get_3d_models` (pure virtual — **mandatory**) |
| Status | `Status` (via `resource.Resource`) | `get_status` — optional, default raises `NotImplementedError` | `get_status` (pure virtual — **mandatory**) |
| Arbitrary commands | `DoCommand` (via `resource.Resource`) | `do_command` — optional, default raises `NotImplementedError` | `do_command` (pure virtual — **mandatory**) |
| Close/lifecycle | `Close` (via `resource.Resource`) | not abstract; optional `close()` hook | inherited from `Component`/`Resource` |
| Frame-system inputs | `CurrentInputs`/`GoToInputs` (via `framesystem.InputEnabled`) | not present as separate methods (covered by `get_joint_positions`/`move_to_joint_positions`) | not present as separate methods |

Citations:
- Go: `type Arm interface` — `rdk@v1.0.0/components/arm/arm.go:127-153`
  (`EndPosition:134`, `MoveToPosition:138`, `MoveToJointPositions:142`,
  `MoveThroughJointPositions:146`, `JointPositions:149`,
  `Get3DModels:152`); `resource.Actuator` —
  `rdk@v1.0.0/resource/resource.go:217-223` (`IsMoving:219`, `Stop:222`);
  `resource.Shaped` — `resource/resource.go:239-243` (`Geometries:242`);
  `resource.Resource` — `resource/resource.go:70-84`
  (`DoCommand:75`, `Status:78`, `Close:83`); `framesystem.InputEnabled` —
  `rdk@v1.0.0/robot/framesystem/framesystem.go:39-43`.
- Python: `viam.components.arm.arm.Arm` — installed `viam-sdk` 0.80.0,
  `viam/components/arm/arm.py`. Seven `@abc.abstractmethod`-decorated
  methods, confirmed by grepping the file directly:
  `get_end_position:32`, `move_to_position:60`,
  `move_to_joint_positions:92`, `get_joint_positions:125`, `stop:152`,
  `is_moving:174`, `get_kinematics:196`. `get_geometries` default —
  `viam/components/component_base.py:61-75` (raises
  `MethodNotImplementedError`). `get_status`/`do_command` defaults —
  `viam/components/component_base.py:46,49` (raise `NotImplementedError`).
  `move_through_joint_positions`/`get_3d_models`: zero matches for either
  name anywhere in `arm.py`, `service.py`, or `client.py` — confirmed by
  direct grep, not inference from absence in one file.
- C++: `viam::sdk::Arm` — `viam-cpp-sdk/src/viam/sdk/components/arm.hpp`.
  `get_end_position:60`, `move_to_position:70`, `get_joint_positions:79`,
  `move_to_joint_positions:88`, `move_through_joint_positions:103`,
  `is_moving:108`, `get_status:112`, `do_command:117`,
  `get_kinematics:122`, `get_3d_models:133`, `get_geometries:148` — all
  pure virtual (`= 0`). `stop` — `Stoppable::stop`,
  `viam-cpp-sdk/src/viam/sdk/resource/stoppable.hpp:13`.

### What this table means in practice

- **Python has no multi-waypoint move.** `MoveThroughJointPositions` is a
  first-class method in both Go and C++ (a single call that moves through
  an ordered list of joint-position waypoints); Python's `Arm` has neither
  the ABC method nor RPC dispatch for it — the generated protobuf stub
  exists (`MoveThroughJointPositions`/`MoveThroughJointPositionsStreamed`
  appear in the compiled `arm_pb2.py` service descriptor) but nothing
  reaches it. A Python driver that needs multi-waypoint motion has to
  either call `move_to_joint_positions` repeatedly (loses whatever
  server-side path-blending a single multi-waypoint call would give) or
  accept the gap.
- **C++ has no escape hatch from `get_3d_models`/`get_status`/`do_command`
  /`get_geometries`.** All four are pure virtual — a C++ arm module must
  implement all of them or it won't compile. Python treats three of the
  four (`get_status`, `do_command`, `get_geometries`) as optional
  overrides with a default that raises at call time, not at class
  definition time. This means a C++ arm module is *forced* to decide what
  its 3D visual models are on day one; a Python one can defer indefinitely
  (§4 covers the consequence of deferring it forever).
- **Go bundles more into embedded interfaces.** `Stop`/`IsMoving` (from
  `resource.Actuator`) and `Geometries` (from `resource.Shaped`) aren't on
  Go's `Arm` interface directly — they're inherited from generic
  resource-level interfaces every actuator-like component gets. This is a
  Go idiom (interface embedding), not a different feature set — the
  methods exist, they're just declared once for every actuator instead of
  once per component type.

---

## 3. FK/IK strategy in practice

The ranked ladder lives in `SKILL.md`; this is what implementing rungs 1
and 3 actually looks like, plus the concrete case where Python beats Go.

### Rung 1 — controller reports pose directly

`[verified]` `xarm-python-sdk`'s `get_position()` calls
`self.arm_cmd.get_tcp_pose()` — a real command sent to the controller, not
a local computation (confirmed by reading `xarm/x3/base.py` and by using
it end-to-end in a built driver: `get_position(is_radian=False)` returns
`(code, [x, y, z, roll, pitch, yaw])`, `code == 0` on success). This is the
cheapest rung whenever the vendor protocol exposes it, and it costs
nothing beyond the connection you already hold for everything else.

**Don't assume a reference implementation exercises the best rung its own
vendor supports.** The Go `viam-ufactory-xarm` module has rung 1 available
on this exact hardware (`xarm-python-sdk`'s `get_position()` proves the
controller supports it) and uses rung 2 instead
(`x.model.Transform(joints)`, in-process FK against its own embedded
kinematics model) — not because rung 1 doesn't exist, but because
`model.Transform` is free to reach for in Go. Checking a reference
implementation's *behavior* to infer a vendor's *capability* is a real
trap: the two can and do diverge, in either direction.

### Rung 3 — delegate to the motion service

Available in any language via `Motion.get_pose(component_name, destination_frame)`,
which resolves the named component's pose through the frame system
server-side (built from that component's own `GetKinematics` +
`GetJointPositions`) — a real gRPC round trip, but zero new dependencies
beyond the motion service every machine already runs. This is the rung to
reach for when there's no controller to ask (a simulated model with no
hardware behind it) and no in-process FK (Python, C++ today).

### Where Python beats Go: native Cartesian moves

`[verified]` `xarm-python-sdk`'s `set_position(x, y, z, roll, pitch, yaw,
speed=, mvacc=, wait=False)` is a direct, native Cartesian move command —
same tier as `get_position()`, no IK solver required client-side. The Go
module cannot do this: its hand-rolled Modbus protocol has no equivalent
command, so `MoveToPosition` is implemented by delegating to a `motion`
service dependency it hard-requires. A Python driver built against
`xarm-python-sdk` has **zero required or optional dependencies** for
`move_to_position` — verified directly:
`validate_config` on a built driver returns `([], [])`.

### Practical rung selection for a two-model module

Every arm module ships a real driver and a simulated model (`SKILL.md`,
Enforced Opinions). They don't get the same rung, and Phase 0's ladder
doesn't say so:

- **Real driver**: rung 1, if the vendor SDK has it (check — don't
  assume, see above).
- **Simulated model**: rung 1 doesn't exist (no controller). Rung 2
  doesn't exist in Python/C++ today. Rung 3 (motion-service delegation)
  is what's left for anything beyond joint-position bookkeeping.

---

## 4. The two mesh channels

Both feed off the same `commonpb.Mesh` proto message (`content_type` +
raw bytes) but are otherwise unrelated, feed different consumers, and only
one of them is reachable from Python today.

| | `GetKinematics` mesh channel | `Get3DModels` |
|---|---|---|
| Field | `GetKinematicsResponse.meshes_by_urdf_filepath` | `Get3DModelsResponse.models` |
| Payload | `.stl`/`.dae`, keyed by normalized URDF path | `.glb` (`model/gltf-binary`), keyed by part name |
| Consumed by | `UnmarshalModelXML` → collision/kinematic geometry, used for planning | Web app's 3D scene, visual rendering |
| Python support | **Yes** — return a 3-tuple from `get_kinematics()` | **No** — absent from `Arm` ABC and `ArmRPCService` |
| C++ support | Yes — `KinematicsDataURDF.meshes_by_urdf_filepath` | Yes — pure virtual, mandatory |
| Go support | Yes — same field, populated by `KinematicModelToProtobuf`/`extractMeshMapFromModelConfig` | Yes |

`[source]` `KinematicsDataURDF` struct with `meshes_by_urdf_filepath` —
`viam-cpp-sdk/src/viam/sdk/common/kinematics.hpp:52-55`. Go's
`KinematicModelFromProtobuf`/`KinematicModelToProtobuf` —
`rdk@v1.0.0/referenceframe/model.go:27-56,60-81`. Go's URDF mesh-path
normalization and disk-loading — `rdk@v1.0.0/referenceframe/model_urdf.go`
(`buildMeshMapFromURDF`) and `xml_conversions.go`
(`normalizeURDFMeshPath:27-37`).

`[verified]` **Key/content-type convention** (documented nowhere
Python-facing; reverse-engineered from the Go RDK source above, then used
in a real Python driver's `get_kinematics()` and confirmed to produce the
expected map):

- Key: take the URDF's `<mesh filename="...">` value, strip the
  `package://` scheme if present, then drop the first remaining path
  segment (the ROS package name).
  `"package://description/meshes/xarm6meshes/link_base.stl"` becomes
  `"meshes/xarm6meshes/link_base.stl"`.
- `Mesh.content_type`: the bare lowercase file extension — `"stl"` or
  `"ply"` — **not** a MIME type. (Contrast `Get3DModels`, whose payload
  convention, per the Go module's own usage, is `"model/gltf-binary"`, a
  real MIME type. The two channels don't even agree on what goes in the
  same-named field.)
- Get this wrong and there is no error anywhere in the chain:
  `GetKinematics` still returns successfully, the mesh map is silently the
  wrong shape, and RDK's lookup just misses.

```python
# Verified pattern — see kinematics_io.py in the built Python xArm module.
import re
from pathlib import Path
from viam.proto.common import Mesh

_MESH_FILENAME_RE = re.compile(r'<mesh\s+filename="([^"]+)"')

def normalize_urdf_mesh_path(raw: str) -> str:
    if raw.startswith("package://"):
        raw = raw[len("package://"):]
        if "/" in raw:
            raw = raw.split("/", 1)[1]
    return raw

def load_mesh_map(urdf_path: Path, kinematics_dir: Path) -> dict[str, Mesh]:
    text = urdf_path.read_text()
    mesh_map = {}
    for raw_path in _MESH_FILENAME_RE.findall(text):
        key = normalize_urdf_mesh_path(raw_path)
        mesh_file = kinematics_dir / key
        if mesh_file.is_file():
            ext = mesh_file.suffix.lstrip(".").lower()
            mesh_map[key] = Mesh(content_type=ext, mesh=mesh_file.read_bytes())
    return mesh_map

# get_kinematics() then returns:
#   (KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF, urdf_bytes, mesh_map)
```

**What this does and doesn't get you in Python**: a Python arm's kinematics
file, with real STL/DAE mesh geometry, is available to RDK's frame system
for collision/planning purposes — full parity with what a Go module gets
by parsing a URDF locally with `useURDFs: true`. It does **not** give you
the web app's polished visual 3D scene — that's `Get3DModels`'s `.glb`
channel specifically, and it is genuinely absent from Python (§5). Don't
conflate the two: shipping the first is real, useful progress, and is not
a substitute for the second.

---

## 5. `Get3DModels` in Python: absent, and a trap in the SDK's own example

`[verified]` `Get3DModels` has real generated gRPC stubs
(`viam/gen/component/arm/v1/arm_grpc.py`, `arm_pb2.py` — both
`ArmServiceBase.Get3DModels` and `ArmServiceStub.Get3DModels` exist,
confirmed in both the PyPI 0.80.0 wheel and a `viam-python-sdk` git
checkout ~381 commits past `v0.34.0`). It is not on the `Arm` ABC (no
`get_3d_models` abstract method), and `ArmRPCService` — the server-side
handler class in `viam/components/arm/service.py` — has no `Get3DModels`
handler. The proto method exists; nothing in the Python SDK reaches it in
either direction.

**The SDK's own reference example implements it anyway, and it does
nothing.** `examples/complex_module/src/arm/my_arm.py:121-123` (git
checkout above):

```python
async def get_3d_models(self, extra: Optional[Dict[str, Any]] = None, **kwargs) -> Mapping[str, Mesh]:
    # This arm has no meshes to report.
    return {}
```

This method is never called by anything: it isn't declared abstract on
`Arm`, and `ArmRPCService` has no handler that would invoke it. A
developer treating `my_arm.py` as a structural reference — which is
otherwise a reasonable thing to do, see §1 — can copy this pattern
directly into their own arm and believe they've implemented
`Get3DModels` support. They haven't; nothing calls it, ever, in this SDK
version. If you want a Python arm module to serve visual 3D models today,
there is no path — degrade to the `GetKinematics` mesh channel (§4) for
collision geometry, or build the simulated/visual half of the module in
Go alongside a Python real driver.

---

## 6. Blocking vendor SDKs and cancellation

Many hardware vendor SDKs — `xarm-python-sdk` among them — are
synchronous and blocking: no `asyncio`, no callbacks, just function calls
that don't return until the underlying socket operation completes.

`[verified]` `xarm-python-sdk`'s socket layer has a real, bounded connect
timeout (`xarm/core/comm/socket_port.py:66`, `sock.settimeout(3)`) —
confirmed by connecting to a non-routable address
(`XArmAPI('10.255.255.1', is_radian=False)`) and observing a clean
`Exception: connect socket failed` after ~1 second, not a hang.

### The obvious approach, and why it's wrong

In an `asyncio`-based Python module, the obvious move is
`await asyncio.to_thread(blocking_call, ...)` so a slow vendor call
doesn't stall the whole event loop. That part is correct and necessary.
The trap is what happens next: many vendor SDKs offer a convenience
"block until this move finishes" parameter (`xarm-python-sdk`'s
`set_servo_angle(..., wait=True)` / `set_position(..., wait=True)`). It
looks like the natural pairing with `to_thread` — dispatch once, await
completion. It is not usably cancellable.

`[verified]` Once a call is dispatched to a thread via
`asyncio.to_thread`, `asyncio`'s cancellation machinery can cancel the
*awaiting coroutine* — the caller gets a `CancelledError` — but it cannot
stop the OS thread that's still blocked inside the vendor SDK's call.
"Cancelling" a `wait=True` move this way abandons the coroutine while the
real move keeps executing underneath, unobserved.

### The pattern that actually works

Send non-blocking (`wait=False`), then poll for completion from the
*awaiting* coroutine, checking cancellation between polls:

```python
# Verified pattern, from a real driver's move_to_joint_positions.
from viam.operations import run_with_operation

@run_with_operation
async def move_to_joint_positions(self, positions, extra=None, **kwargs):
    operation = self.get_operation(kwargs)
    code = await asyncio.to_thread(
        self._arm.set_servo_angle, angle=list(positions.values),
        speed=self._speed, mvacc=self._accel, is_radian=False, wait=False,
    )
    _check(code, "set_servo_angle")
    await self._wait_until_joints_reach(operation, list(positions.values))

async def _wait_until_joints_reach(self, operation, target_deg, timeout=30.0):
    elapsed = 0.0
    while elapsed < timeout:
        if await operation.is_cancelled():
            await self.stop()   # must actually halt the arm — see §1
            return
        _, angles = await asyncio.to_thread(self._arm.get_servo_angle, is_radian=False)
        if all(abs(a - t) <= 0.05 for a, t in zip(angles, target_deg)):
            return
        await asyncio.sleep(0.05)
        elapsed += 0.05
    raise TimeoutError("move did not reach target in time")
```

`[verified]` Confirmed against a real `asyncio.Task.cancel()` (not
merely a mock returning early): cancelling an in-flight
`@run_with_operation`-wrapped call correctly propagates through
`asyncio.shield` (`run_with_operation`'s own implementation shields the
inner call so it can finish cleanly rather than being abruptly killed),
the outer awaiter receives `CancelledError` immediately, and the
still-running inner coroutine observes `operation.is_cancelled() == True`
on its next poll and calls `stop()` — verified by asserting the mock
hardware's stop sequence actually ran, not just that the call returned.

**This generalizes past xArm.** Any vendor SDK that is synchronous and
offers a blocking "wait for completion" convenience parameter has this
same trap. The fix is the same shape regardless of vendor: non-blocking
dispatch, then poll-with-cancellation-check in the coroutine that's
actually being awaited.

---

## 7. `viam.spatialmath` — a real capability nothing points at

Non-Viam arm vendor SDKs routinely report orientation as Euler angles
(roll/pitch/yaw) — `xarm-python-sdk` does, Universal Robots' does too.
Viam's `Pose` reports orientation as an orientation vector
(`o_x`, `o_y`, `o_z`) plus `theta`. Converting between the two is not a
unit conversion — it's a different rotational representation entirely,
and getting it wrong produces a plausible-looking, silently incorrect
pose.

`[verified]` `viam.spatialmath` (ships with the Python SDK, FFI-backed —
`viam/spatialmath/_ffi.py`) has exactly this conversion, correctly:
`EulerAngles(roll_rad, pitch_rad, yaw_rad).to_quaternion()` then
`Quaternion.to_pose(x, y, z)` for vendor→Viam, and
`Quaternion.from_pose(pose).to_euler_angles()` for the reverse.

**Units**: `EulerAngles`'s constructor takes **radians**, confirmed
empirically —
`EulerAngles(0, 0, math.pi/2).to_quaternion().to_pose(1,2,3)` gives
`theta=89.999...`  (correct, a 90° yaw), while passing `90.0` as a literal
degree value produces `theta=116.6...` (wrong — it was interpreted as
~90 radians). `xarm-python-sdk`'s roll/pitch/yaw are in degrees whenever
called with `is_radian=False` (this reference's convention throughout,
explicit rather than relying on the SDK default) — convert with
`math.radians`/`math.degrees` at the boundary.

**Round-trip verified**: converting degrees → `Pose` → degrees through
this path reproduces the original values to floating-point precision —
`(30.0, -20.0, 15.0)` in, `(30.000000000000004, -20.00000000000001,
15.000000000000021)` out. This confirms internal consistency of the
conversion pipeline; it does **not** confirm the result matches any
specific vendor's physical RPY convention against real hardware feedback
— that needs an arm to check against.

### Caveat: `RotationMatrix.elements` changed layout between SDK versions

`[verified]` — tested directly against both versions, not taken on
report. Constructing a pure 90°-about-Y rotation
(`EulerAngles(0, math.pi/2, 0)`) and reading `.to_quaternion().to_rotation_matrix().elements`:

| SDK version | `elements` | Layout |
|---|---|---|
| 0.79.2 | `[0, 0, -1, 0, 1, 0, 1, 0, 0]` | **column-major** |
| 0.80.0 | `[0, 0, 1, 0, 1, 0, -1, 0, 0]` | **row-major** |

The two arrays are exact transposes of each other for the same rotation —
this is a genuine, silent breaking change in the raw buffer layout `.elements`
exposes, not floating-point noise. Code that reads `RotationMatrix.elements`
directly and assumes one layout will silently produce a transposed (wrong)
rotation on whichever version it didn't test against.

**The fix, also verified**: don't read `.elements`. Use `Quaternion`'s
scalar accessors (`.w`, `.i`, `.j`, `.k`) instead — layout-independent by
construction, since they're individual scalars, not a flattened buffer.
Confirmed identical `w`/`i`/`j`/`k` values and identical `to_pose()` output
on both 0.79.2 and 0.80.0 for the same input Euler angles.

---

## 8. The three unit boundaries

`[verified]` — confirmed by inspection of a URDF file, the installed
`xarm-python-sdk`, and the installed `viam-sdk`'s proto definitions
together in one driver build.

| Boundary | Units | Whose job the conversion is |
|---|---|---|
| URDF kinematics file | metres, radians (URDF spec default) | Nobody's, in a Python driver — the file is read as opaque bytes and handed to `GetKinematics`; only RDK (Go) ever parses it into a kinematic model |
| Vendor SDK (e.g. `xarm-python-sdk`) | mm, degrees (with `is_radian=False`) | The driver, at the point it calls into the vendor SDK |
| Viam arm API (`Pose`, `JointPositions`) | mm, degrees (`Pose.theta`, `JointPositions.values` both documented "degrees"; `Pose.x/y/z` "millimeters") | The driver, at the point it returns a value to the RPC layer |
| `viam.spatialmath` (`EulerAngles`, `OrientationVector`) | **radians**, despite the Viam wire format being degrees | The driver, at the one or two call sites that touch orientation conversion (§7) |

The useful consequence: `xarm-python-sdk`'s native units (mm, degrees) and
Viam's own Python wire units (mm, degrees) already agree. A driver built
this way can stay in degrees for joint positions, speed, and acceleration
end-to-end, and only reach for radians in the narrow slice of code that
calls into `viam.spatialmath` for orientation conversion. Contrast the Go
module, which converts mm/degrees to radians immediately on every call
(`utils.DegToRad`), because RDK's `referenceframe` package is
radians-internally throughout — a Python driver doesn't inherit that
constraint and shouldn't manufacture it.

---

## 9. C++ coverage: what's `[source]`, and where it stops

No C++ arm port was built to produce this reference — everything in this
section is read from `viam-cpp-sdk` and a real production C++ arm module
(`universal-robots`), cited by file:line, and marked `[source]`
accordingly. Treat it as a map of where to look, not a substitute for
building and running a C++ driver.

- **Method surface**: see §2's table — C++ requires the most (`get_status`,
  `do_command`, `get_geometries`, `get_3d_models` are all pure virtual;
  Python treats three of the four as optional and lacks the fourth
  entirely).
- **Cancellation has no SDK-provided operation manager in C++** — unlike
  Go's `operation.SingleOperationManager`. `[source]` The `universal-robots`
  module hand-rolls cancellation via a gRPC context observer:
  `async_cancellation_monitor` lambdas built from
  `GrpcContextObserver::current()` (`ur_arm.cpp:751,816`), checked inside
  `stop_()` (`ur_arm.cpp:1138-1140`, `cancel_future->get()`). This is real
  cancellation, but it is bespoke per-module, not a library feature — a
  new C++ arm driver needs to build (or copy) this mechanism, not assume
  the SDK provides one.
- **Mesh channel**: `KinematicsDataURDF::meshes_by_urdf_filepath`
  (`viam-cpp-sdk/src/viam/sdk/common/kinematics.hpp:52-55`) is the same
  proto field as Python's `meshes_by_urdf_filepath` and Go's
  `MeshesByUrdfFilepath` — one wire format, three SDKs reading it. Since
  `get_3d_models` is pure virtual in C++, a C++ arm module cannot defer
  the `.glb`-channel decision the way a Python one currently can (§5) —
  it has to return *something* (`universal-robots`'s `URArm::get_3d_models`
  exists and is called, `ur_arm.hpp:145`) even if that something is an
  empty map.
- **What's genuinely unknown from here**: whether C++'s blocking-SDK/
  cancellation story has the same `asyncio.to_thread`-shaped trap Python
  does (§6). C++ has no single dominant async runtime the way Python has
  `asyncio`, so the answer is almost certainly "it depends on the
  concurrency primitives the specific module chose" rather than one
  universal pattern — but that's a plausible-sounding guess, not something
  read or run, so it isn't asserted as fact here. A C++ port would need to
  answer this for itself.

---

## 10. Cross-references

- `SKILL.md` — the phase machine, the FK ladder (ranked), `armkit`
  mechanics, the two known `armkit`/RDK divergences. This document is the
  depth; `SKILL.md` is the front door.
- `viam-python` skill — Python SDK mechanics generally (async connection
  patterns, `EasyResource`, module registration boilerplate) not specific
  to arms.
- `viam-cpp` skill — C++ SDK mechanics generally (module registration,
  CMake, threading) not specific to arms.
- `viam-go-motion-vision` — motion planning, frame systems, `PlanRequest`,
  and motion-service delegation (§3's rung 3) once the arm exists.
