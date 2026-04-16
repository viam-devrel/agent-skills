# Viam Skill Suite Design

A suite of Claude Code skills providing deep, source-verified expertise across the full
Viam robotics platform: SDKs (Go, Python, TypeScript, C++), module development, fleet
management, and ML pipelines.

## Skill Inventory

| Skill | Scope | Status |
|-------|-------|--------|
| `viam-go-motion-vision` | Arm, camera, vision, motion planning, frame system, spatial math — Go SDK | Existing (rename from `viam-expert`) |
| `viam-go-platform` | Non-manipulation components/services, resource model — Go SDK | New |
| `viam-modules-fleet` | Module lifecycle, CLI, fleet management, robot config | New |
| `viam-python` | Python SDK: all components/services, async patterns, ecosystem integration | New |
| `viam-ml` | Data capture, training scripts, model deployment, ML framework integration | New |
| `viam-cpp` | C++ SDK: performance-critical drivers, CMake toolchain | New |
| `viam-typescript` | TS SDK: web app integration, Viam Applications, HMI patterns | New |

## Skill Boundaries

### `viam-go-motion-vision` (rename of `viam-expert`)

**Triggers on:** Arm, camera, vision service, motion planning, frame system, coordinate
transforms, point clouds, spatial math, WorldState, PlanRequest, IK, obstacle detection,
any vision-guided manipulation pipeline — in Go.

**Does NOT trigger on:** Non-manipulation components, fleet ops, ML training, other
languages.

**Changes:** Rename only. Content stays as-is with the fixes applied 2026-04-16. Update
frontmatter `name` and `description`.

### `viam-go-platform`

**Triggers on:** Go SDK usage for base, board, motor, servo, sensor, encoder, gripper,
gantry, input controller, power sensor, movement sensor, audio in/out, button, switch,
pose tracker, navigation service, SLAM service, data manager service, discovery service,
world state store service, base remote control service, generic component/service patterns,
Go-specific resource API patterns (`resource.Resource`, `resource.Shaped`, `Reconfigure`,
dependency injection).

**Does NOT trigger on:** Manipulation/vision/motion (-> `viam-go-motion-vision`), module
lifecycle (-> `viam-modules-fleet`), ML training (-> `viam-ml`).

### `viam-modules-fleet`

**Triggers on:** `viam` CLI commands, `viam module generate`, `meta.json` authoring,
module building/packaging/uploading, registry operations, robot configuration (JSON/app
UI), fleet provisioning, machine management, fragment configs, `viam server` operation,
agent/deployment workflows.

**Does NOT trigger on:** SDK-specific code (-> language skills), ML model training
(-> `viam-ml`).

**Agentic focus:** This skill guides CLI command sequences, not just explains concepts.
Includes exact commands, expected outputs, and error recovery for workflows like:
scaffold -> build -> package -> upload -> deploy -> verify.

### `viam-python`

**Triggers on:** Python code importing `viam` packages, async robot connection patterns
(`viam.robot.client`), Python component/service implementations, Python module
development, ecosystem integration (OpenCV, NumPy, PyTorch, etc. with Viam), `asyncio`
patterns specific to Viam's Python SDK.

**Does NOT trigger on:** Go SDK (-> Go skills), module registry/fleet ops
(-> `viam-modules-fleet`), ML training pipeline (-> `viam-ml`).

### `viam-ml`

**Triggers on:** Viam data capture/sync, dataset management, ML training scripts (custom
or Viam-provided), model deployment to robots, vision service ML model configuration, ML
model service, `viam dataset`, `viam train`.

**Framework coverage:** TFLite, ONNX, PyTorch, TensorFlow, Triton Server, Keras,
Ultralytics.

**Does NOT trigger on:** General vision service usage without ML context
(-> motion-vision or language skills), fleet deployment mechanics
(-> `viam-modules-fleet`).

### `viam-cpp`

**Triggers on:** C++ code importing Viam SDK headers, CMake build configuration for Viam
modules, performance-critical driver development, depth camera implementations (RealSense,
Orbbec), arm driver implementations (UR), gRPC service patterns in C++, memory management
and threading concerns in Viam context.

**Does NOT trigger on:** Other languages, fleet ops, ML training.

### `viam-typescript`

**Triggers on:** TypeScript/JavaScript code importing `@viamrobotics/sdk`, web application
integration with robots, Viam Applications development, HMI/dashboard patterns,
browser-based robot control, `viam-typescript-sdk` usage.

**Node.js:** Support technically exists (see `examples/node` in the SDK repo) but may
have unsupported features compared to client-side usage. Note limitations when relevant.

**Does NOT trigger on:** Other languages, fleet ops, ML training.

## Cross-Skill References

Skills reference each other rather than duplicating domain knowledge.

| Concept | Primary skill | Referenced by |
|---------|--------------|---------------|
| Frame system, transforms, motion planning architecture | `viam-go-motion-vision` | `viam-python`, `viam-cpp` |
| Component/service resource model | `viam-go-platform` | All language skills |
| Module lifecycle (scaffold, meta.json, build, upload, deploy) | `viam-modules-fleet` | All language skills |
| ML model integration | `viam-ml` | `viam-go-motion-vision`, `viam-cpp`, `viam-python` |
| Robot config schema | `viam-modules-fleet` | All skills |

When a skill hits a topic owned by another skill, it should:
1. Give a brief contextual answer (1-2 sentences)
2. Note which skill has the deep reference
3. Never reproduce the other skill's reference material

Language skills cover their SDK's manipulation/vision APIs (method signatures, async
patterns, type differences) but reference `viam-go-motion-vision` for underlying
architecture and mental models.

## Reference Material Sources

### `viam-go-motion-vision`
- **Status:** Done. Has `rdk-vision-manipulation-reference.md` and `cheatsheet.md`.

### `viam-go-platform`
- **Source:** `~/src/rdk/` — `components/base`, `components/board`, `components/motor`,
  `components/servo`, `components/sensor`, `components/movementsensor`,
  `components/encoder`, `components/gripper`, `components/gantry`,
  `components/inputcontroller`, `components/powersensor`, `components/audioin`,
  `components/audioout`, `components/button`, `components/switch`,
  `components/posetracker`, `services/navigation`, `services/slam`,
  `services/datamanager`, `services/discovery`, `services/worldstatestore`,
  `services/baseremotecontrol`, `resource/` package
- **Reference captures:** Interface definitions, config patterns, resource lifecycle
  (Reconfigure, Close, Dependencies), common error patterns, data manager sync behavior,
  SLAM map types, navigation modes
- **Cheatsheet captures:** Component interface hierarchy, config field tables, unit
  conventions per sensor type

### `viam-modules-fleet`
- **Source:** `~/src/rdk/cli/` for CLI internals, `viam --help` tree for user-facing
  commands
- **Reference captures:** Full CLI command tree with flags and expected output, `meta.json`
  schema with field explanations, robot config JSON schema, fragment structure, module
  packaging formats (tar.gz, appimage), registry API patterns, provisioning workflows
- **Cheatsheet captures:** CLI command quick-reference (scaffold -> build -> upload ->
  deploy), config snippet templates, common error messages and fixes
- **Agentic recipes:** Step-by-step command sequences for common workflows (create new
  module, deploy to fleet, update module version, rollback)

### `viam-python`
- **Source:** `viamrobotics/viam-python-sdk` (GitHub)
- **Reference captures:** Async client connection pattern, component/service interfaces
  (differences from Go: async/await, Pythonic naming), module server implementation, type
  mappings (Go types -> Python equivalents), ecosystem integration patterns (OpenCV
  ndarray <-> Viam image, NumPy <-> point cloud)
- **Cheatsheet captures:** Import paths, interface method signatures, async patterns,
  common type conversions

### `viam-ml`
- **Source:** RDK `services/mlmodel`, `services/datamanager`, `data/`; training script
  repos: `viam-modules/classification-tflite`, `viam-modules/detection-tflite`,
  `viam-modules/tabular-data-tensorflow`, `viam-devrel/yolo-training` (Ultralytics)
- **Reference captures:** Data capture pipeline (collector -> sync -> cloud), dataset
  management, training script structure (Keras patterns and Ultralytics patterns), model
  deployment config for each runtime (TFLite, ONNX, PyTorch, TensorFlow, Triton Server),
  ML model service interface and implementation patterns, vision service <-> ML model
  service integration
- **Cheatsheet captures:** Supported model formats per runtime, config snippets for each
  ML framework, training script template, data query patterns

### `viam-cpp`
- **Source:** `viamrobotics/viam-cpp-sdk` (GitHub); `viam-modules/` org repos:
  `universal-robots`, `orbbec`, `mlmodel-tflite`, `viam-camera-realsense`,
  `viam-mlmodelservice-triton`, `system-audio`
- **Reference captures:** SDK architecture (gRPC stubs, component/service base classes),
  CMake integration patterns, module registration, threading model, memory management
  patterns, driver implementation patterns extracted from real modules
- **Cheatsheet captures:** CMake boilerplate, component interface signatures, gRPC
  patterns, build/link flags

### `viam-typescript`
- **Source:** `viamrobotics/viam-typescript-sdk` (GitHub, including `examples/node`);
  `viam-devrel/cube-sorter-webapp`
- **Reference captures:** Browser client connection pattern, streaming APIs (camera, sensor
  data in browser), Viam Applications deployment model, component/service client
  interfaces, Node.js support status and limitations
- **Cheatsheet captures:** Import paths, client setup boilerplate, streaming patterns,
  Viam Application config

## Phased Build Order

### Phase 1: Foundation
1. **Rename `viam-expert` -> `viam-go-motion-vision`** — Frontmatter and directory name
   only.
2. **`viam-modules-fleet`** — Core workflow, agentic CLI support. Source: `~/src/rdk/cli/`.

### Phase 2: Language Coverage
3. **`viam-python`** — Broadest remaining language use case. Source:
   `viamrobotics/viam-python-sdk`.
4. **`viam-go-platform`** — Completes Go coverage. Source: `~/src/rdk/` components and
   services.

### Phase 3: Specialized
5. **`viam-ml`** — Training + deployment pipeline. Source: RDK ML services + training
   script repos.
6. **`viam-cpp`** — Performance-critical drivers. Source: `viam-cpp-sdk` + `viam-modules/`
   repos.

### Phase 4: Web
7. **`viam-typescript`** — Web/HMI integration. Source: `viam-typescript-sdk` +
   `cube-sorter-webapp`.

## Deliverables Per Skill

Each skill delivery includes:
- `SKILL.md` — Frontmatter, domain guidance, gotcha library, code examples, cross-skill
  references
- `references/<topic>-reference.md` — Deep reference from source analysis
- `references/cheatsheet.md` — Quick-reference tables (interfaces, types, units, patterns)
- Installed to `~/.claude/skills/<skill-name>/`
- Packaged as `.skill` file in `~/Downloads/`

## Prerequisites

Each skill requires source analysis to build reference material. For skills sourced from
`~/src/rdk/` this is ready. External repos need to be cloned locally:
- `viamrobotics/viam-python-sdk`
- `viamrobotics/viam-cpp-sdk`
- `viamrobotics/viam-typescript-sdk`
- `viam-modules/universal-robots`
- `viam-modules/orbbec`
- `viam-modules/mlmodel-tflite`
- `viam-modules/viam-camera-realsense`
- `viam-modules/viam-mlmodelservice-triton`
- `viam-modules/system-audio`
- `viam-modules/classification-tflite`
- `viam-modules/detection-tflite`
- `viam-modules/tabular-data-tensorflow`
- `viam-devrel/yolo-training`
- `viam-devrel/cube-sorter-webapp`
