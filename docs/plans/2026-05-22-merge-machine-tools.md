# Merge: viam-machine-claude-tools → agent-skills

**Date:** 2026-05-22

## Goal

Fold the two operational skills from the standalone `viam-machine-claude-tools`
prototype into this marketplace so they install, version, and update alongside
the SDK skills instead of being symlinked in by hand.

## What moved

| From `viam-machine-claude-tools` | To `agent-skills` |
|---|---|
| `viam-machine-up/{SKILL.md, machine_up.py}` | `skills/local-viam-server/` |
| `viam-machine-config/{SKILL.md, update_config.py}` | `skills/viam-machine-config/` |

The `viam-machine-up` skill was renamed **`local-viam-server`** on the way in
(directory, plugin, and `name:` frontmatter); its bundled script keeps the
`machine_up.py` filename. The Python scripts (`machine_up.py`,
`update_config.py`) were copied verbatim. Each `SKILL.md` gained a "Related
skills" section cross-referencing the rest of the suite; the `local-viam-server`
/ `viam-machine-config` descriptions gained a cross-reference tail in the house
style.

## How they fit the marketplace

- Each skill ships as a **1:1 per-skill plugin wrapper** under `plugins/`,
  matching the existing convention: a `.claude-plugin/plugin.json` plus a
  `skills/<name>` symlink into the canonical `skills/` source. The root
  `viam-skills` bundle plugin picks them up automatically — **9 skills total**.
- These are *operational* skills: a `SKILL.md` paired with an executable script,
  rather than a `SKILL.md` paired with a `references/` directory. The README
  "Skill Structure" section now documents both shapes, and the "Skills" section
  splits into "SDK & pipeline skills" and "Machine tooling skills".

## Versioning

Marketplace metadata and the `viam-skills` bundle plugin bumped 0.1.0 → 0.2.0
(two new skills is more than a patch). The two new per-skill plugins start at
0.1.0.

## Scope notes

- `viam-modules-fleet` is the only existing skill with real topical overlap
  (machine / fleet management). Its description, "Out of Scope", and
  "Cross-skill handoff patterns" now hand off operational machine work to the
  new skills. The other six skills' cross-reference lists were left unchanged —
  a C++ or TypeScript question does not route to a machine-tooling skill, and
  expanding every list risks the Claude Desktop description-length limit.
- `local-viam-server`'s generated `viam-server.json` configs hold machine
  secrets; they are written outside the repo (`~/viam-local-machines/`) and a
  repo `.gitignore` was added as a backstop.
