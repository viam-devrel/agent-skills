# Viam Agent Skills

A suite of Claude Code skills providing deep, source-verified expertise across the Viam robotics platform. Each skill is built from direct analysis of SDK source code and real module repositories — not documentation summaries.

## Skills

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
```

- **Manipulation concepts** (frame system, motion planning, WorldState) live in `viam-go-motion-vision` — language skills reference it for architecture
- **Module lifecycle** (scaffold, build, upload, deploy) lives in `viam-modules-fleet` — all language skills reference it for deployment
- **ML pipeline** (data capture, training, model deployment) lives in `viam-ml` — referenced by vision and language skills

## Installation

### Install all skills to Claude Code

```bash
# Copy all skills to your Claude Code skills directory
cp -r skills/viam-* ~/.claude/skills/
```

### Install a single skill

```bash
cp -r skills/viam-python ~/.claude/skills/
```

### Package as .skill files (for sharing)

```bash
cd skills
for skill in viam-*/; do
  zip -r "${skill%/}.skill" "$skill"
done
```

## Skill Structure

Each skill follows the same structure:

```
skills/viam-<name>/
  SKILL.md                          # Main skill definition
  references/
    <topic>-reference.md            # Deep reference (interfaces, architecture)
    cheatsheet.md                   # Quick-reference tables and templates
```

### SKILL.md contains:

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

Skills were built from analysis of these repositories:

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

These skills were built from source analysis circa April 2026. The Viam platform evolves rapidly. Each skill includes version awareness guidance instructing the LLM to:

- Check the user's `go.mod` / `requirements.txt` / `package.json` for their SDK version
- Prefer grepping local source over trusting the reference blindly
- Acknowledge gaps rather than fabricating API signatures
- Recommend `pkg.go.dev` or SDK docs for canonical API references

## Design Documents

The design and implementation plans are in [`docs/plans/`](docs/plans/):

- `2026-04-16-viam-skill-suite-design.md` — skill boundaries, cross-references, source map
- `2026-04-16-viam-skill-suite-implementation.md` — phased build plan with task breakdown
