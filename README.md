# Viam Agent Skills

A suite of Claude Code skills for the Viam robotics platform. The SDK skills provide deep, source-verified expertise built from direct analysis of SDK source code and real module repositories — not documentation summaries. The machine-tooling skills bundle ready-to-run scripts that drive live Viam machines through the Viam app API.

**This library of skills is a work-in-progress. If you have feedback about how to improve any of the skills or ones to add, please file an issue or submit a PR.**

## Skills

### SDK & pipeline skills

Deep, source-verified reference skills — each pairs a `SKILL.md` with reference files extracted from real codebases.

| Skill | Scope | Lines |
|-------|-------|------:|
| [viam-go-motion-vision](skills/viam-go-motion-vision/) | Arm, camera, vision, motion planning, frame system, spatial math — Go SDK | 1,310 |
| [viam-go-platform](skills/viam-go-platform/) | All other components (17), services (7), and the resource API — Go SDK | 3,109 |
| [viam-modules-fleet](skills/viam-modules-fleet/) | `viam` CLI, module lifecycle, fleet management, robot configuration | 2,150 |
| [viam-python](skills/viam-python/) | Python SDK: async patterns, all components/services, module dev, ecosystem integration | 2,221 |
| [viam-ml](skills/viam-ml/) | Data capture, training scripts (Keras, Ultralytics), model deployment (TFLite, Triton) | 2,004 |
| [viam-cpp](skills/viam-cpp/) | C++ SDK: driver patterns from 6 production modules (UR, RealSense, Orbbec, TFLite, Triton, audio) | 1,876 |
| [viam-typescript](skills/viam-typescript/) | TypeScript SDK: browser robot control, Viam Applications, HMI/dashboard patterns | 1,831 |

**Total: ~14,500 lines of reference material across 25 files.**

### Machine tooling skills

Operational skills — each bundles an executable Python script for working with live Viam machines from the command line, in places the `viam` CLI itself can't.

| Skill | Scope |
|-------|-------|
| [local-viam-server](skills/local-viam-server/) | Create a Viam cloud machine and bring up a local `viam-server` connected to it — fetches the machine part secret the `viam` CLI does not expose, and picks a free port |
| [viam-machine-config](skills/viam-machine-config/) | Push or replace a machine's robot config (modules, components, services) via the Viam app API — the CLI can read config history but cannot set it |

## How Skills Work Together

The skills cross-reference each other rather than duplicating knowledge:

```
                    viam-modules-fleet
                   (CLI, deploy, config)
                    /    |    |    \
                   /     |    |     \
  viam-go-motion-vision  |    |  viam-typescript
  (manipulation, Go)     |    |  (web/HMI)
                         |    |
  viam-go-platform       |    viam-cpp
  (components, Go)       |    (drivers, C++)
                         |
                     viam-python ──── viam-ml
                     (Python SDK)     (training, deployment)

       local-viam-server ──── viam-machine-config
      (create machine +       (push robot config)
       run viam-server)
      └──── operate live machines (Viam app API) ────┘
```

- **Manipulation concepts** (frame system, motion planning, WorldState) live in `viam-go-motion-vision` — language skills reference it for architecture
- **Module lifecycle** (scaffold, build, upload, deploy) lives in `viam-modules-fleet` — all language skills reference it for deployment
- **ML pipeline** (data capture, training, model deployment) lives in `viam-ml` — referenced by vision and language skills
- **Machine operations** (create a machine, run `viam-server`, push a robot config) live in `local-viam-server` and `viam-machine-config` — they drive live machines via the Viam app API, complementing the build/upload workflows in `viam-modules-fleet`

## Installation

This repo is published as a **Claude Code plugin marketplace**. You can install the full bundle in one go, or pick individual plugins.

### 1. Add the marketplace

Inside Claude Code, run:

```
/plugin marketplace add viam-devrel/agent-skills
```

This registers the marketplace from the GitHub repo. You can also point at a local clone:

```
/plugin marketplace add /path/to/agent-skills
```

### 2. Browse and install

Open the interactive picker:

```
/plugin
```

You'll see the `viam-agent-skills` marketplace with these plugins:

| Plugin | What it installs |
|--------|------------------|
| `viam-skills` | **Bundled** — all 11 skills below |
| `viam-go-motion-vision` | Arm, camera, vision, motion planning (Go) |
| `viam-go-platform` | Non-manipulation Go components, services, resource API |
| `viam-modules-fleet` | Viam CLI, module lifecycle, fleet/robot config |
| `viam-python` | Python SDK |
| `viam-ml` | Data capture, training, model deployment |
| `viam-cpp` | C++ SDK driver patterns |
| `viam-typescript` | Browser robot control, HMIs, Viam Applications |
| `local-viam-server` | Create a cloud machine + run a local viam-server |
| `viam-machine-config` | Push a machine's robot config via the app API |
| `viam-doc-draft` | Draft Viam docs pages from learning objectives (Diátaxis, house style) |
| `viam-doc-review` | Review Viam docs pages against house style before a PR |

### 3. Install directly (non-interactive)

```
/plugin install viam-skills@viam-agent-skills
/plugin install viam-python@viam-agent-skills
/plugin install viam-cpp@viam-agent-skills
/plugin install local-viam-server@viam-agent-skills
```

The `@viam-agent-skills` suffix selects this marketplace.

### Picking skills at runtime

Once installed, skills activate automatically based on triggers in their frontmatter (for example, touching a file that imports `@viamrobotics/sdk` activates `viam-typescript`). To invoke a skill manually, use the `Skill` tool or mention the skill by name.

### Updating and removing

```
/plugin marketplace update viam-agent-skills
/plugin uninstall viam-python@viam-agent-skills
```

### Repository layout

```
.claude-plugin/
  plugin.json          # bundled "viam-skills" plugin manifest
  marketplace.json     # marketplace listing (bundle + per-skill)
skills/                # canonical skill sources
  <skill>/
plugins/               # per-skill plugin wrappers (symlink into skills/)
  <skill>/
    .claude-plugin/plugin.json
    skills/<skill>  -> ../../../skills/<skill>
```

The per-skill plugins are thin wrappers that symlink into `skills/` — there is only one source of truth for each skill.

## Skill Structure

**SDK & pipeline skills** pair a `SKILL.md` with reference files:

```
skills/viam-<name>/
  SKILL.md                          # Main skill definition
  references/
    <topic>-reference.md            # Deep reference (interfaces, architecture)
    cheatsheet.md                   # Quick-reference tables and templates
```

**Machine tooling skills** pair a `SKILL.md` with an executable script:

```
skills/<skill-name>/
  SKILL.md                          # When to use the skill, usage, options, gotchas
  <script>.py                       # The script the skill drives (run via python3)
```

### An SDK skill's SKILL.md contains:

- **Frontmatter** — trigger description (when the skill activates)
- **Knowledge Sources** — which reference files to consult, with version awareness caveats
- **Out of Scope** — what belongs to other skills, with cross-references
- **Detecting Developer Level** — adjusts response depth based on user signals
- **Domain Guidance** — structured advice for the skill's core topics
- **Gotcha Library** — proactive warnings for common mistakes
- **Code Example Patterns** — complete, working examples

### Reference files contain:

Source-verified interface definitions, method signatures, type systems, architecture diagrams, and patterns extracted from real codebases. These are not documentation summaries — they are built from reading the actual source code.

## Source Material

The SDK & pipeline skills were built from analysis of these repositories:

| Source | Used by |
|--------|---------|
| [viamrobotics/rdk](https://github.com/viamrobotics/rdk) | viam-go-motion-vision, viam-go-platform, viam-modules-fleet, viam-ml |
| [viamrobotics/viam-python-sdk](https://github.com/viamrobotics/viam-python-sdk) | viam-python |
| [viamrobotics/viam-cpp-sdk](https://github.com/viamrobotics/viam-cpp-sdk) | viam-cpp |
| [viamrobotics/viam-typescript-sdk](https://github.com/viamrobotics/viam-typescript-sdk) | viam-typescript |
| [viam-modules/universal-robots](https://github.com/viam-modules/universal-robots) | viam-cpp |
| [viam-modules/orbbec](https://github.com/viam-modules/orbbec) | viam-cpp |
| [viam-modules/viam-camera-realsense](https://github.com/viam-modules/viam-camera-realsense) | viam-cpp |
| [viam-modules/mlmodel-tflite](https://github.com/viam-modules/mlmodel-tflite) | viam-cpp, viam-ml |
| [viam-modules/viam-mlmodelservice-triton](https://github.com/viam-modules/viam-mlmodelservice-triton) | viam-cpp, viam-ml |
| [viam-modules/system-audio](https://github.com/viam-modules/system-audio) | viam-cpp |
| [viam-modules/classification-tflite](https://github.com/viam-modules/classification-tflite) | viam-ml |
| [viam-modules/detection-tflite](https://github.com/viam-modules/detection-tflite) | viam-ml |
| [viam-modules/tabular-data-tensorflow](https://github.com/viam-modules/tabular-data-tensorflow) | viam-ml |
| [viam-devrel/yolo-training](https://github.com/viam-devrel/yolo-training) | viam-ml |
| [viam-devrel/cube-sorter-webapp](https://github.com/viam-devrel/cube-sorter-webapp) | viam-typescript |

## Version Awareness

The SDK & pipeline skills were built from source analysis circa April 2026. The Viam platform evolves rapidly. Each of those skills includes version awareness guidance instructing the LLM to:

- Check the user's `go.mod` / `requirements.txt` / `package.json` for their SDK version
- Prefer grepping local source over trusting the reference blindly
- Acknowledge gaps rather than fabricating API signatures
- Recommend `pkg.go.dev` or SDK docs for canonical API references
