# Viam Skill Suite Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build 7 Claude Code skills providing deep, source-verified expertise across the full Viam robotics platform.

**Architecture:** Each skill follows the same structure (SKILL.md + references/) with deep reference material built from source analysis. Skills cross-reference each other rather than duplicating domain knowledge. Each is installed to `~/.claude/skills/` and packaged as a `.skill` zip.

**Tech Stack:** Claude Code skills (Markdown), source analysis of Go/Python/C++/TypeScript codebases, Viam RDK and SDKs.

**Design doc:** `docs/plans/2026-04-16-viam-skill-suite-design.md`

---

## Phase 1: Foundation

### Task 1: Rename `viam-expert` to `viam-go-motion-vision`

**Files:**
- Modify: `~/.claude/skills/viam-expert/SKILL.md` (frontmatter only)
- Rename: `~/.claude/skills/viam-expert/` -> `~/.claude/skills/viam-go-motion-vision/`
- Modify: `~/Downloads/viam-expert.skill` -> rebuild as `viam-go-motion-vision.skill`

**Step 1: Update SKILL.md frontmatter**

Change the `name` and `description` fields:

```yaml
---
name: viam-go-motion-vision
description: >
  Deep expert on building robotic manipulation pipelines with the Viam platform
  and Go SDK. Use this skill whenever a developer asks about: Viam RDK arm/camera/vision
  components, motion planning, frame systems, coordinate transforms, point clouds, spatial
  math, WorldState, PlanRequest, IK solvers, obstacle detection, or any vision-guided
  manipulation pipeline in Go. Trigger even for general questions like "how do I move my
  arm to a pose?" or "why is my motion plan failing?" — if Viam + Go + manipulation is
  in scope, use this skill. Also trigger when the user shares Go code that imports
  viam motion/vision/arm packages and wants help debugging, extending, or designing
  around it. For other Viam topics see: viam-go-platform (non-manipulation Go components),
  viam-modules-fleet (CLI, modules, fleet), viam-python, viam-cpp, viam-typescript,
  viam-ml.
---
```

**Step 2: Update the skill heading**

Change `# Viam Expert Skill` to `# Viam Go Motion & Vision Skill`.

**Step 3: Rename the skill directory**

```bash
mv ~/.claude/skills/viam-expert ~/.claude/skills/viam-go-motion-vision
```

**Step 4: Rebuild the .skill package**

```bash
cd ~/.claude/skills
zip -r ~/Downloads/viam-go-motion-vision.skill viam-go-motion-vision/
```

**Step 5: Verify**

Confirm the skill appears in the Claude Code skill list with the new name and description.
Start a new conversation and verify the trigger description is clear.

---

### Task 2: Clone missing repos

**Step 1: Clone all needed repos to `~/src/`**

```bash
cd ~/src
git clone https://github.com/viamrobotics/viam-python-sdk.git
git clone https://github.com/viamrobotics/viam-typescript-sdk.git
git clone https://github.com/viam-modules/universal-robots.git
git clone https://github.com/viam-modules/mlmodel-tflite.git
git clone https://github.com/viam-modules/viam-mlmodelservice-triton.git
git clone https://github.com/viam-modules/classification-tflite.git
git clone https://github.com/viam-modules/detection-tflite.git
git clone https://github.com/viam-modules/tabular-data-tensorflow.git
git clone https://github.com/viam-devrel/yolo-training.git
```

**Step 2: Verify all repos are present**

```bash
ls ~/src/{rdk,viam-python-sdk,viam-typescript-sdk,viam-cpp-sdk,viam-camera-orbbec,viam-camera-realsense,universal-robots,mlmodel-tflite,viam-mlmodelservice-triton,system-audio,classification-tflite,detection-tflite,tabular-data-tensorflow,yolo-training,cube-sorter-webapp}
```

Expected: all directories exist with content.

---

### Task 3: Build `viam-modules-fleet` skill

This is the highest-priority new skill — core workflow with agentic CLI support.

**Files:**
- Create: `~/.claude/skills/viam-modules-fleet/SKILL.md`
- Create: `~/.claude/skills/viam-modules-fleet/references/cli-reference.md`
- Create: `~/.claude/skills/viam-modules-fleet/references/config-schema-reference.md`
- Create: `~/.claude/skills/viam-modules-fleet/references/cheatsheet.md`
- Create: `~/Downloads/viam-modules-fleet.skill`

**Step 1: Analyze CLI source and help output**

Read and analyze the following to build reference material:
- `~/src/rdk/cli/` — all `.go` files for CLI command implementations
- Run `viam --help` and `viam <subcommand> --help` for every subcommand to capture the
  full command tree, flags, and descriptions
- Focus on: `module`, `machines`, `data`, `train`, `dataset`, `packages`, `organizations`,
  `locations`, `robots`, `login`, `logout`, `version`

**Step 2: Analyze config and meta.json schemas**

Read and analyze:
- `~/src/rdk/cli/module_generate.go` (or equivalent) for `meta.json` schema
- `~/src/rdk/config/` for robot config JSON schema
- Fragment structure from config or CLI source

**Step 3: Write `references/cli-reference.md`**

Content:
- Full CLI command tree organized by subcommand group
- For each command: syntax, all flags with types/defaults, description, example usage,
  expected output
- Error messages and their causes
- Auth flow (login, API keys, org context)

**Step 4: Write `references/config-schema-reference.md`**

Content:
- `meta.json` schema: every field, types, required vs optional, examples
- Robot config JSON structure: components, services, modules, remotes, fragments
- Fragment structure and inheritance
- Common config patterns (add a module, configure a component from a module, set up
  data capture)

**Step 5: Write `references/cheatsheet.md`**

Content:
- CLI command quick-reference table (one-liner per command)
- `meta.json` minimal template
- Robot config snippet templates (component, service, module, fragment)
- Module packaging commands (build, package, upload)
- Common error messages -> fixes table

**Step 6: Write `SKILL.md`**

Structure (following `viam-go-motion-vision` as template):
- Frontmatter with trigger description (CLI, module lifecycle, fleet management, config)
- Knowledge Sources section pointing to references, with version awareness caveat
- Out of Scope section (SDK-specific code -> language skills, ML training -> viam-ml)
- Detecting Developer Level table
- Domain Guidance sections:
  1. Module Lifecycle (generate -> build -> package -> upload -> deploy -> update)
  2. Fleet Management (provisioning, fragments, machine groups)
  3. Robot Configuration (config structure, component/service/module config patterns)
  4. Agentic Workflows (step-by-step CLI recipes with exact commands and expected output)
- Gotcha Library (common CLI errors, config mistakes, registry pitfalls)
- Code/Config Example Patterns (complete, working examples)
- Cross-references to other Viam skills

**Step 7: Package and install**

```bash
cd ~/.claude/skills
zip -r ~/Downloads/viam-modules-fleet.skill viam-modules-fleet/
```

**Step 8: Verify**

Start a new conversation, ask "how do I create and deploy a new Viam module?" and confirm
the skill triggers and provides useful guidance with correct CLI commands.

---

## Phase 2: Language Coverage

### Task 4: Build `viam-python` skill

**Files:**
- Create: `~/.claude/skills/viam-python/SKILL.md`
- Create: `~/.claude/skills/viam-python/references/python-sdk-reference.md`
- Create: `~/.claude/skills/viam-python/references/cheatsheet.md`
- Create: `~/Downloads/viam-python.skill`

**Step 1: Analyze Python SDK source**

Read and analyze `~/src/viam-python-sdk/`:
- `src/viam/robot/client.py` — async robot connection pattern
- `src/viam/components/` — all component interfaces (compare with Go equivalents)
- `src/viam/services/` — all service interfaces
- `src/viam/module/` — module server implementation
- `src/viam/proto/` — protobuf bindings, type mappings
- Focus on: async patterns, naming conventions (snake_case vs Go's CamelCase), type
  differences, ecosystem integration points

**Step 2: Write `references/python-sdk-reference.md`**

Content:
- SDK architecture (async client, gRPC, protobuf bindings)
- Component/service interface definitions with full method signatures
- Module server implementation patterns (how to build a Python module)
- Type mappings: Go types -> Python equivalents (Pose, PoseInFrame, WorldState, etc.)
- Async patterns: connection lifecycle, context management, concurrent operations
- Ecosystem integration: OpenCV ndarray <-> Viam image, NumPy <-> point cloud,
  PIL/Pillow image handling
- Differences from Go SDK (what's missing, what's different, what's Pythonic)

**Step 3: Write `references/cheatsheet.md`**

Content:
- Import paths for all components and services
- Async connection boilerplate
- Component/service method signature quick-reference
- Common type conversions (image, point cloud, pose)
- Python module server template

**Step 4: Write `SKILL.md`**

Structure:
- Frontmatter with trigger description
- Knowledge Sources with version awareness
- Out of Scope (Go -> Go skills, fleet -> viam-modules-fleet, ML training -> viam-ml)
- Detecting Developer Level
- Domain Guidance:
  1. Async Connection Patterns
  2. Component/Service Usage
  3. Module Development in Python
  4. Ecosystem Integration (OpenCV, NumPy, PyTorch, etc.)
- Gotcha Library (async pitfalls, type conversion traps, import path differences)
- Code Example Patterns
- Cross-references (manipulation concepts -> viam-go-motion-vision, module deployment
  -> viam-modules-fleet)

**Step 5: Package, install, verify**

Same as Task 3 Steps 7-8.

---

### Task 5: Build `viam-go-platform` skill

**Files:**
- Create: `~/.claude/skills/viam-go-platform/SKILL.md`
- Create: `~/.claude/skills/viam-go-platform/references/components-services-reference.md`
- Create: `~/.claude/skills/viam-go-platform/references/resource-api-reference.md`
- Create: `~/.claude/skills/viam-go-platform/references/cheatsheet.md`
- Create: `~/Downloads/viam-go-platform.skill`

**Step 1: Analyze component sources**

Read and analyze `~/src/rdk/` for each component:
- `components/base/base.go` — interface, config
- `components/board/board.go` — interface, GPIO, analog, SPI, I2C patterns
- `components/motor/motor.go` — interface, power/position/velocity modes
- `components/servo/servo.go` — interface, angle control
- `components/sensor/sensor.go` — generic sensor interface
- `components/movementsensor/movementsensor.go` — GPS, IMU, position, orientation
- `components/encoder/encoder.go` — tick counting, position
- `components/gripper/gripper.go` — open/close/grab
- `components/gantry/gantry.go` — multi-axis linear motion
- `components/inputcontroller/input_controller.go` — gamepad/joystick events
- `components/powersensor/powersensor.go` — voltage, current, power
- `components/audioin/audioin.go` — audio input interface
- `components/audioout/audioout.go` — audio output interface
- `components/button/button.go` — button interface
- `components/switch/switch.go` — switch interface
- `components/posetracker/pose_tracker.go` — pose tracking interface

**Step 2: Analyze service sources**

- `services/navigation/navigation.go` — waypoint navigation, modes
- `services/slam/slam.go` — SLAM map types, position
- `services/datamanager/data_manager.go` — data capture, sync
- `services/discovery/discovery.go` — component discovery
- `services/worldstatestore/worldstatestore.go` — world state persistence
- `services/baseremotecontrol/base_remote_control.go` — joystick -> base control

**Step 3: Analyze resource package**

- `resource/resource.go` — Resource interface, lifecycle
- `resource/api.go` — API registration
- `resource/config.go` — config patterns
- `resource/dependencies.go` — dependency injection
- Focus on patterns that module developers need to understand

**Step 4: Write `references/components-services-reference.md`**

Content:
- Interface definitions for every component and service listed above
- Config patterns for each (what goes in the robot config JSON)
- Common usage patterns and integration points between components
- Data manager sync behavior, capture configuration
- Navigation modes, waypoint patterns
- SLAM map types and usage

**Step 5: Write `references/resource-api-reference.md`**

Content:
- Resource lifecycle (construction, Reconfigure, Close)
- Dependency injection patterns
- API registration for custom components/services
- Config validation patterns
- How the resource graph works
- Common patterns for module developers

**Step 6: Write `references/cheatsheet.md`**

Content:
- Component interface hierarchy table (every component, key methods)
- Service interface hierarchy table
- Resource lifecycle method signatures
- Config field tables per component type
- Unit conventions per sensor type

**Step 7: Write `SKILL.md`**

Structure:
- Frontmatter with trigger description
- Knowledge Sources with version awareness
- Out of Scope (manipulation -> viam-go-motion-vision, module deployment ->
  viam-modules-fleet, ML -> viam-ml)
- Domain Guidance:
  1. Component Patterns (base, motor, sensor families)
  2. Service Patterns (navigation, SLAM, data manager)
  3. Resource API (lifecycle, dependencies, config)
  4. Board & GPIO (pins, analog, SPI, I2C — common hardware integration)
- Gotcha Library
- Code Example Patterns
- Cross-references

**Step 8: Package, install, verify**

---

## Phase 3: Specialized

### Task 6: Build `viam-ml` skill

**Files:**
- Create: `~/.claude/skills/viam-ml/SKILL.md`
- Create: `~/.claude/skills/viam-ml/references/ml-pipeline-reference.md`
- Create: `~/.claude/skills/viam-ml/references/training-scripts-reference.md`
- Create: `~/.claude/skills/viam-ml/references/cheatsheet.md`
- Create: `~/Downloads/viam-ml.skill`

**Step 1: Analyze RDK ML service sources**

Read and analyze `~/src/rdk/`:
- `services/mlmodel/` — ML model service interface, implementations
- `services/datamanager/` — data capture, sync pipeline
- `data/` — data management internals
- Vision service ML integration points (from `viam-go-motion-vision` reference)

**Step 2: Analyze training script repos**

Read and analyze:
- `~/src/classification-tflite/` — Keras-based classification training
- `~/src/detection-tflite/` — Keras-based detection training
- `~/src/tabular-data-tensorflow/` — TensorFlow tabular data training
- `~/src/yolo-training/` — Ultralytics YOLO training (non-Keras pattern)
- Extract: training script structure, required entry points, Viam data loading
  patterns, model export formats, registration with Viam

**Step 3: Analyze ML runtime integration**

Read and analyze:
- `~/src/mlmodel-tflite/` — TFLite model service (C++ module)
- `~/src/viam-mlmodelservice-triton/` — Triton inference server integration
- Framework docs for: ONNX Runtime, PyTorch serving, TensorFlow Serving
- Focus on: model formats accepted, config patterns, inference API

**Step 4: Write `references/ml-pipeline-reference.md`**

Content:
- End-to-end ML pipeline: data capture -> sync -> dataset -> train -> deploy -> infer
- Data capture configuration (collectors, sync intervals, cloud storage)
- Dataset management (creation, querying, filtering)
- ML model service interface and implementations
- Model deployment config for each runtime (TFLite, ONNX, PyTorch, TensorFlow, Triton)
- Vision service <-> ML model service integration
- Framework-specific details (input/output tensor formats, preprocessing)

**Step 5: Write `references/training-scripts-reference.md`**

Content:
- Training script structure and required interfaces
- Keras patterns (from classification-tflite, detection-tflite)
- Ultralytics patterns (from yolo-training)
- TensorFlow patterns (from tabular-data-tensorflow)
- Data loading from Viam datasets
- Model export and format conversion
- Registration and deployment

**Step 6: Write `references/cheatsheet.md`**

Content:
- Supported model formats per runtime table
- Config snippets for each ML framework deployment
- Training script template (Keras, Ultralytics)
- Data capture config template
- CLI commands for data/dataset/train operations
- Common error patterns and fixes

**Step 7: Write `SKILL.md`**

Structure:
- Frontmatter with trigger description
- Knowledge Sources with version awareness
- Out of Scope (general vision -> motion-vision, fleet ops -> modules-fleet)
- Domain Guidance:
  1. Data Pipeline (capture, sync, datasets)
  2. Training Scripts (Keras, Ultralytics, TensorFlow patterns)
  3. Model Deployment (TFLite, ONNX, PyTorch, TF, Triton)
  4. Vision + ML Integration
- Gotcha Library (format mismatches, tensor shape issues, deployment config errors)
- Code Example Patterns
- Cross-references

**Step 8: Package, install, verify**

---

### Task 7: Build `viam-cpp` skill

**Files:**
- Create: `~/.claude/skills/viam-cpp/SKILL.md`
- Create: `~/.claude/skills/viam-cpp/references/cpp-sdk-reference.md`
- Create: `~/.claude/skills/viam-cpp/references/driver-patterns-reference.md`
- Create: `~/.claude/skills/viam-cpp/references/cheatsheet.md`
- Create: `~/Downloads/viam-cpp.skill`

**Step 1: Analyze C++ SDK source**

Read and analyze `~/src/viam-cpp-sdk/`:
- SDK architecture: gRPC stubs, component/service base classes
- Module registration and lifecycle
- CMake integration (how to build a module)
- Threading model, error handling patterns
- Component/service interface definitions

**Step 2: Analyze driver module repos**

Read and analyze:
- `~/src/universal-robots/` — UR arm driver (complex real-time control)
- `~/src/viam-camera-orbbec/` — Orbbec depth camera (point cloud generation)
- `~/src/viam-camera-realsense/` — RealSense depth camera
- `~/src/mlmodel-tflite/` — TFLite inference (C++ ML integration)
- `~/src/viam-mlmodelservice-triton/` — Triton server integration
- `~/src/system-audio/` — Audio I/O (simpler module pattern)
- Extract: common patterns, CMake structure, dependency management, testing patterns

**Step 3: Write `references/cpp-sdk-reference.md`**

Content:
- SDK architecture (gRPC, protobuf, base classes)
- Component/service interface definitions (C++ signatures)
- Module registration and lifecycle (main.cpp patterns)
- CMake integration (FindViam, linking, dependencies)
- Threading model (gRPC threads, component threads, synchronization)
- Memory management patterns (smart pointers, RAII in Viam context)
- Error handling (grpc::Status, exceptions)

**Step 4: Write `references/driver-patterns-reference.md`**

Content:
- Patterns extracted from real modules:
  - Camera driver pattern (RealSense, Orbbec): streaming, point cloud, configuration
  - Arm driver pattern (UR): real-time control, kinematics integration
  - ML model service pattern (TFLite, Triton): inference, tensor handling
  - Simple module pattern (system-audio): audio I/O, straightforward lifecycle
- CMake boilerplate per pattern
- Dependency management strategies
- Testing patterns from real modules

**Step 5: Write `references/cheatsheet.md`**

Content:
- Component/service interface signatures table
- CMake template (minimal module)
- Module registration boilerplate
- gRPC patterns (status codes, streaming)
- Build/link flags reference
- Common compiler errors and fixes

**Step 6: Write `SKILL.md`**

Structure:
- Frontmatter with trigger description
- Knowledge Sources with version awareness
- Out of Scope
- Domain Guidance:
  1. Module Development (CMake, registration, lifecycle)
  2. Driver Implementation (camera, arm, sensor, ML patterns)
  3. Performance Patterns (threading, memory, real-time constraints)
  4. gRPC Integration
- Gotcha Library (linking errors, threading deadlocks, protobuf version mismatches)
- Code Example Patterns
- Cross-references (manipulation concepts -> viam-go-motion-vision, module deployment
  -> viam-modules-fleet, ML models -> viam-ml)

**Step 7: Package, install, verify**

---

## Phase 4: Web

### Task 8: Build `viam-typescript` skill

**Files:**
- Create: `~/.claude/skills/viam-typescript/SKILL.md`
- Create: `~/.claude/skills/viam-typescript/references/ts-sdk-reference.md`
- Create: `~/.claude/skills/viam-typescript/references/cheatsheet.md`
- Create: `~/Downloads/viam-typescript.skill`

**Step 1: Analyze TypeScript SDK source**

Read and analyze `~/src/viam-typescript-sdk/`:
- `src/` — client architecture, component/service interfaces
- `examples/` — browser and Node.js usage patterns
- Connection patterns (WebRTC, direct gRPC)
- Streaming APIs (camera streams in browser, sensor data)
- Type definitions (compare with Go/Python equivalents)

**Step 2: Analyze Viam Application example**

Read and analyze `~/src/cube-sorter-webapp/`:
- Application structure and deployment model
- How it connects to robots
- UI patterns for robot control
- Streaming camera data to browser

**Step 3: Research Viam Applications deployment model**

Web fetch `https://docs.viam.com/operate/control/viam-applications/` for current
deployment docs. Extract: how apps are deployed, configuration, authentication,
hosting model.

**Step 4: Write `references/ts-sdk-reference.md`**

Content:
- SDK architecture (gRPC-web, WebRTC, client construction)
- Component/service client interfaces (TypeScript signatures)
- Connection patterns (browser vs Node.js, authentication)
- Streaming APIs (camera, sensor data)
- Viam Applications model (deployment, config, auth)
- Node.js support and limitations (what works, what doesn't)
- Type differences from Go/Python SDKs

**Step 5: Write `references/cheatsheet.md`**

Content:
- Import paths (`@viamrobotics/sdk`)
- Client connection boilerplate (browser, Node.js)
- Component/service method signatures table
- Camera streaming pattern
- Viam Application config template
- Common browser-specific gotchas

**Step 6: Write `SKILL.md`**

Structure:
- Frontmatter with trigger description
- Knowledge Sources with version awareness
- Out of Scope
- Domain Guidance:
  1. Browser Client Patterns (connection, auth, streaming)
  2. Viam Applications (deployment model, config, hosting)
  3. HMI/Dashboard Patterns (camera display, control interfaces, sensor visualization)
  4. Node.js Usage (what works, limitations)
- Gotcha Library (WebRTC issues, CORS, streaming buffering, auth token handling)
- Code Example Patterns
- Cross-references

**Step 7: Package, install, verify**

---

## Execution Notes

### Source analysis approach

Each "Analyze" step should use the Explore agent or parallel subagents to read source
code, extract interface definitions, identify patterns, and build the reference material.
The reference docs should be written in the same style as `viam-go-motion-vision`'s
existing references: thorough, with exact type signatures, code examples, and
architecture diagrams.

### Verification criteria per skill

Each skill is verified by:
1. Skill appears in Claude Code skill list with correct name and description
2. Trigger logic activates on relevant questions (test with 2-3 sample prompts)
3. Code examples in the skill compile/run against the current SDK version
4. Cross-references to other skills are accurate
5. .skill package extracts correctly

### Dependency order

Tasks 1-2 have no dependencies and can run in parallel.
Task 3 depends on Task 2 (needs RDK cli/ source).
Tasks 4-5 can run in parallel after Task 2.
Tasks 6-7 depend on Task 2 (needs cloned repos).
Task 8 depends on Task 2.

Within each task, the steps are sequential (analyze before writing references, write
references before SKILL.md).
