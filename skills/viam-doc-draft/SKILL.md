---
name: viam-doc-draft
description: >
  Turn a set of learning objectives (LOs) plus source links into a great Viam docs
  page. Use this skill whenever you are drafting or substantially rewriting a Viam
  docs page from learning objectives, a page outline, or reviewer feedback —
  especially conceptual ("explanation") and how-to pages under docs/. It encodes
  the LO-to-page framework: Diátaxis classification, concrete-first openings,
  positive role-assignment instead of negation, no personification of services,
  API precision, vocabulary discipline, and a prose-level vale gate. Trigger on
  requests like "build a page from these LOs", "draft a docs page for X",
  "rewrite this page to match house style", or "the reviewer didn't like the
  style, fix it". Pair with viam-doc-review to check the result.
---

# Viam doc drafting skill (LO → page)

You turn learning objectives into docs pages that read the way Viam's best pages
read: concrete first, precise about APIs, positively phrased, and free of jargon
and personification. Learning objectives are **author scaffolding** — they shape
the page and then disappear from it. A reader must be able to *do* each objective
after reading, but must never see the objective's Bloom verb ("Understand X",
"Analyze Y") in the prose.

This framework is synthesized from Shannon's review feedback on PR #5098, the
repo's own vale rules, Diátaxis, the Google and Microsoft style guides, and how
Drake, Unity, and Unreal document spatial/frame/scene concepts. See
`references/framework.md` for the full rationale and citations, and
`references/exemplars.md` for model openings.

## Source of truth: defer to Shannon's playbooks and code map

The authoritative standards live in **`/home/shrews/viam/viam-code-map`** (the `playbook-*.md`
files, `vocabulary.md`, and the `*-xref.md` / `flows.md` code map). Defer to it for process,
vocabulary, and code accuracy. This skill's own value-add is the LO-to-page craft below,
`references/exemplars.md`, and the SVG/diagram reference (`references/diagrams.md`) — none of
which the playbooks cover.

- **Writing process and Diátaxis** → `playbook-writing.md` and `playbook-diataxis.md`. To
  create or substantially rewrite a page, follow `playbooks.md` (Playbook 2: gather code
  refs → read the code → write → validate); when responding to review comments, follow
  `playbook-reviewer-feedback.md`.
- **Vocabulary** → `vocabulary.md` (preferred Viam terms + Adopt / Bridge / Acknowledge). When
  the disposition is **Adopt**, use the practitioner term in titles and headings for
  discoverability; when **Bridge**, name both. `references/banned-words.md` is a supplement.
- **Code accuracy** → the code map's `*-xref.md` files and `flows.md` (proto is the source of
  truth). Gather and read code refs *before* writing, as Playbook 2 requires.

## The eight moves

Work through these in order when drafting.

### 1. Classify the page with Diátaxis — and don't mix modes

Decide which one mode dominates:

- **Explanation** (concept): answers "Can you tell me about X?" Understanding-oriented. Discussion, not instruction.
- **How-to**: answers "How do I X?" Goal-oriented steps for a competent user.
- **Tutorial**: a guided learning lesson for a newcomer.
- **Reference**: dry, structured lookup for someone already working.

Diátaxis's core rule: **explanation must stay closely bounded and resist absorbing instruction.** A page that is "concept + a runnable code example" is two modes conflated. Either split it into an explanation page and a linked how-to, or — if it must stay one page — keep the concept and the steps in clearly separated sections that cross-link, so neither dilutes the other.

**LOs appear in prose only as a tutorial's `{{< alert title="Learning Goals" color="info" >}}` callout** (an established Viam tutorial pattern). On explanation and how-to pages, LOs never appear — they only drive what the page must teach.

### 2. Map each LO to a section, then throw the LO wording away

One numbered LO → one section. The heading is a concrete user question or task ("Target a frame with `Move`"), never the Bloom verb. If "Understanding…", "Analyze…", or "Learn about…" survives into a heading or opening sentence, you skipped this step. That exact leak is what Shannon flagged ("Rather than the sentence beginning 'Understanding…', make the abstract concrete").

### 3. Open concrete, then abstract — concept → short form → API

The first sentence states the true, mechanical thing that happens, in words a newcomer already owns. Then name the API. Models to imitate:

- **Drake (pose):** "A spatial pose, more commonly just pose, provides the location and orientation of a frame B with respect to another frame A." *Then* "In Drake this concept is provided by the `RigidTransform` class." Concept → short form → class.
- **Unity (transform):** "The Transform stores a GameObject's Position, Rotation, Scale and parenting state." List what it *stores* in sentence one; state the invariant next.
- **Shannon's own rewrite:** "To move an arm using the Viam motion service, you specify a desired pose for a specific frame defined within the frame system. The motion planner solves for joint angles that place that frame's origin in the destination pose."

Never pre-announce ("In this section we will…", "This page covers…" as the *first* move — a one-line scope sentence is fine after the concrete opening). Get to the point fast.

### 4. Second person; name the real actor; never personify the service

**Voice: second person, never first person.** Address the reader as "you". Do not use
"we", "us", "our", or "let's" — even for friendly transitions. Write "Next, add a new
frame", not "Let's add a new frame"; "you pass two parameters", not "we pass two
parameters". This keeps the warmth of a walkthrough without the narrator. (Shannon never
used first person in any of his suggested rewrites; the repo's `AvoidFirstPerson` vale
rule enforces it.)

**Actor: name the thing that acts.** The subject of a sentence is the planner, the method,
or the module — never an anthropomorphized service that "wants", "targets", "tries to", or
"knows". Use active voice and present tense. "The motion planner solves for joint angles" —
not "the motion service targets the frame it wants." This was Shannon's first and most
emphatic note.

### 5. Prefer positive statements — assign each thing a noun and a job

Say what a thing **is** and **does**. The repo's `PositiveStatements` vale rule flags "is not / does not / won't" for a reason. For two confusable things (a visual vs. a collision geometry; `WorldState` vs. the world state store service), **give each a noun and a job** the way Unity and Unreal do, instead of negating:

- Avoid: "A visual in the scene is **not** an obstacle. The planner **does not** read it."
- Prefer: "The renderer draws the visual so you can see it. The motion planner collision-checks a separate geometry — the frame system's `frame.geometry` and the `WorldState` you pass to `Move`. Each object can have both, and they serve different jobs."

Reserve at most **one** explicit contrast per page, and only when distinguishing the two things *is itself* the learning objective. State it once, positively ("these are two separate paths"), never as a drumbeat of disclaimers.

### 6. Be API-precise and term-consistent

Name the exact method, the API it belongs to, its parameters, what it returns, and what a value *is*. "`Move` is a method in the motion service API; its destination is a `PoseInFrame` giving the goal pose for a named frame's origin." Use the **same term every time** for the same concept (Google's consistency rule) — don't alternate "world state", "WorldState", and "obstacles" for the same thing. Avoid the overloaded word "configuration" when you mean something specific (Shannon's note); say "frame", "geometry", or "machine config" precisely.

**Verify every API symbol against the Viam SDK skills before raw source.** Those expert
skills already encode accurate, current API knowledge — use them as the first source of
truth, not your memory and not a sub-agent's summary:

- Go motion / frame system / spatial math / WorldState / vision → `viam-go-motion-vision/references/` (`MoveReq` with `ComponentName string`, `NewLinkInFrame`, `NewPose`, `OrientationVectorDegrees`, `NewBox`, `PoseInFrame`).
- Python SDK → `viam-python/references/` (`Transform` fields incl. `physical_object`, `component_name` as `str`, pose/orientation).
- Other Go components/services → `viam-go-platform`; config and `frame` attributes → `viam-machine-config` / `viam-modules-fleet`; TypeScript → `viam-typescript`; C++ → `viam-cpp`.

Fall back to the raw repos (`/home/shrews/viam/rdk`, the SDKs, `motion-tools`) only for
symbols the references do not cover. **High-risk claims to always check** — every one of
these was wrong in an early draft of the end-effector page: trailing/optional parameters
("what is the 4th arg?"), exact field names (`physical_object`, `pose_in_observer_frame`),
the *type* of an identifier (a plain string vs a `resource.Name`), read-only vs writable
services (the world state store service is read-only from a client), and any "behind the
scenes" / automatic-behavior platform claim.

Give snippets enough context to be true without being full programs: a one-line note on
where a client comes from ("these snippets assume a connected `motion_service`; see
[Access the motion service in your code](...)"), and a trailing comment on any variable
you reference but do not define.

### 7. Budget vocabulary; define terms once and link

Don't stack unfamiliar terms in the intro (Shannon: "explain what this page is without so many terms that might not be familiar"). For each term a newcomer might not know, either:

- replace it with plain language, or
- define it once in the canonical place (the section overview or glossary) and link with `{{< glossary_tooltip term_id="..." >}}` or a direct link.

Put **conditions before instructions** (Google). Add a `## Prerequisites` bullet list (linked, not inline-explained) when the reader needs prior setup or concepts. Define pose, frame, and world state in the section overview and link from leaf pages, rather than redefining them on every page.

### 8. Gate on prose, then re-check the LOs

Before handing the page back, run vale at **suggestion** level, not just the error gate that CLAUDE.md's pre-PR check uses:

```bash
vale --minAlertLevel suggestion docs/<section>/<page>.md
```

The prose-quality rules (`PositiveStatements`, `Simplicity`, `SentenceLength`, `Readability`, `AvoidObscure`, `GlobalAudienceJargon`, `AvoidFirstPerson`) are suggestion/warning level, so the error-only gate hides them. Treat them as required for prose. Targets: sentences under 25 words, reading grade 8 or below, no "simply/easy/handy", no "via/utilize/e.g.", second person ("you"), positive statements.

Also check for **banned idioms, banned words, and em dashes** (`references/banned-words.md`, sourced from the viam-training `worksheet-style-guide.md`). No em dashes (`—`) as punctuation, no idioms or colloquialisms, and none of the banned vague words ("shape" as a vague noun, "spin up", "surface" as a verb, "ships", "load-bearing", and so on) — unless the word is literally the topic (a page about geometric shapes may say "shape"). Quick grep:

```bash
grep -nE "—|–|spin up|stand up|\bships\b|load-bearing|earns its keep|on the wire" docs/<section>/<page>.md
```

Then close the loop: confirm a reader can now *do* each numbered LO — without the LO text appearing anywhere on the page.

## Followability patterns (what makes a page easy to follow)

These come from a rewrite that readers found markedly easier to follow. They are the
"how" behind moves 1-8 — apply them especially on explanation+how-to pages.

- **Name the synonyms up front, then pick one.** When a concept has several industry
  names, list them in the opening and state which one the page uses: "the end effector
  frame (also called the tool control point, or TCP)… this page calls it the end effector
  frame." Readers arrive with different vocabularies; this meets all of them, then commits
  to one term (move 6). Mirrors Drake's "a spatial pose, more commonly just pose".
- **Build in layers, simple case first.** Order sections so each builds on the last: the
  static/default case → the dynamic/code case → adding a geometry → visualizing it. The
  reader earns each new idea on top of one they already have.
- **Annotate non-obvious parameters inline in the code.** Add a short trailing comment on
  each argument a reader could not guess: `reference_frame="my-arm",  # the parent frame`.
  Never leave a parameter unexplained (no `nil, // what is this?`) — look it up in source
  and say what it is. Annotated code is the highest-leverage followability win.
- **Ground the concept in concrete hardware variety.** Show the idea across real end
  effectors (a parallel gripper, a vacuum gripper, a screwdriver) so the reader sees what
  changes and what stays the same, before the abstraction.
- **Use labeled coordinate-frame diagrams for spatial concepts.** A frame, pose, or offset
  is far clearer as a diagram with labeled RGB axes (X red, Y green, Z blue — the
  Unity/Unreal/Drake convention) than as prose. Mark the origin and the offset distance.
  `references/diagrams.md` has the reusable SVG design system (metallic arm, triads, iso
  boxes), the Hugo embedding rules, and the **render-and-verify loop** — author diagrams
  by rendering a PNG and reading it, never blind.
- **Pick an example that genuinely requires the feature.** For a code-defined frame, choose
  a runtime scenario a static config could not handle (a grasped object whose dimensions
  are unknown until pickup), not one that a `frame` attribute already covers. When the
  scenario is a real fork (tool changer vs held object), ask the author to choose.
- **Say what happens "behind the scenes" plainly, and verify it.** When the platform does
  something implicit (an arm gets a built-in end effector frame from its kinematics file),
  state it in one plain sentence — and confirm the mechanism against source before
  claiming it.

## House conventions (don't reinvent these)

- **Frontmatter:** `linkTitle`, `title`, `weight`, `layout: "docs"`, `type: "docs"`, `description`. The `description` is action-oriented and **under ~155 characters** (Hugo warns above 158). `no_list: true` on section indexes, `aliases` for moved pages.
- **Shortcodes:** `{{< cards >}}`/`{{% card link=... noimage="true" %}}` for "What's next"; `{{< tabs >}}`/`{{% tab %}}` for multi-language code; `{{% alert color="tip|caution|note|info" %}}` for callouts; `{{< glossary_tooltip >}}` for term definitions; `{{< imgproc >}}` for images (alt text required).
- **Page shape:** concrete opening paragraph → one-line scope sentence → sections (one per LO) → optional `## Prerequisites` near the top when needed → `## What's next` cards. End how-to pages with linked cards.
- **Code examples:** complete enough to orient, real package paths, the actual function names from the RDK/SDK source. Verify against source before writing — don't invent method names or parameters.

## Workflow when invoked

1. Confirm the LOs, the Diátaxis mode, and the example scenario.
2. Ground every API claim in the **Viam SDK skills' references first** (move 6), falling
   back to raw repos only for what they miss. Read 1-2 neighboring pages to match local
   structure and term definitions.
3. Draft following the eight moves. Keep `references/exemplars.md` open as a style anchor.
   For any spatial concept, build the figures per `references/diagrams.md` (render and read
   each one).
4. Run prettier, markdownlint, and `vale --minAlertLevel suggestion`; fix prose findings.
5. Re-check each LO is satisfied by the prose. Report what each section teaches, mapped back to its LO, so the author can verify coverage without the LOs being on the page.

Hand off to **viam-doc-review** for an independent pass before the PR.
