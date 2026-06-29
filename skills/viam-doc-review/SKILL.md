---
name: viam-doc-review
description: >
  Review a Viam docs page against house style the way a senior reviewer (Shannon)
  would, before it reaches a PR. Use this skill whenever asked to review, critique,
  or style-check a Viam docs page, or after drafting one with viam-doc-draft.
  It checks for the failure modes that draw review comments: personified services,
  learning-objective wording leaking into prose, negation-heavy "X is not Y"
  framing, undefined jargon stacked in intros, imprecise API references, overloaded
  terms, and prose that fails the suggestion-level vale rules the error gate skips.
  Reports findings as file:line with a concrete rewrite for each, grouped by
  severity. Trigger on "review this page", "does this match house style?",
  "style-check docs/...", or "would Shannon flag anything here?".
---

# Viam doc review skill

You review a docs page the way Shannon reviewed PR #5098: concrete, specific, and
focused on tone and clarity rather than typos. For every finding you give the
`file:line`, name the failure mode, and provide a **concrete rewrite** — never just
"this is too abstract." You are catching the comments before the human reviewer does.

The companion drafting framework lives in the `viam-doc-draft` skill
(`references/framework.md` and `references/exemplars.md`). This skill checks the
same eight moves from the reader's side.

## Source of truth: defer to Shannon's playbooks and code map

The authoritative standards live in **`/home/shrews/viam/viam-code-map`** (Shannon's
`viam-code-map` repo: the `playbook-*.md` files, `vocabulary.md`, and the `*-xref.md` /
`flows.md` code map). Defer to it. This skill adds the two things the playbooks lack: a
**runnable prose-lint** (the greps below) and the **diagram/SVG design system**.

- **House style and vocabulary** → `playbook-writing.md` (prose craft, Williams-based) and
  `vocabulary.md` (preferred Viam terms with Adopt / Bridge / Acknowledge dispositions).
  `viam-doc-draft/references/banned-words.md` is a supplement, not the authority.
- **Code accuracy** → the cross-reference files (`component-api-xref.md`,
  `service-api-xref.md`, `config-xref.md`, `cli-xref.md`) and `flows.md`. **Proto is the
  single source of truth** for API signatures; verify behavioral claims against RDK code,
  not proto. Cite `code:<repo>/<path>:<line>` and label each claim *verified* or *inferred*.
- **Reviewer-feedback workflow** → `playbook-reviewer-feedback.md` (Intake → Verify → Plan →
  Draft → PR, with a verification-log table and acceptance-criteria mapping: every diff
  change traces to a reviewer comment, nothing out of scope).
- **Page / section assessment** → `playbook-1-assess-existing-page.md` (Steps 0–13 + rule
  ledger) and `playbook-section-review.md` (4-phase, with phase gates).

## How to run a review

1. **Read the page in full**, plus its frontmatter and one or two neighboring pages
   for term-consistency context.
2. **Run the prose gate** (the part CLAUDE.md's error-only check skips):

   ```bash
   vale --minAlertLevel suggestion docs/<section>/<page>.md
   ```

   Also run `npx markdownlint-cli --config .markdownlint.yaml` on the page.

   **Grep for em dashes, banned idioms, and banned words** (canonical list in
   `viam-doc-draft/references/banned-words.md`, sourced from viam-training's
   `worksheet-style-guide.md` — read it before reviewing):

   ```bash
   grep -nE "—|–|spin up|stand up|drops? in(to)?|\bships\b|\bsurface[ds]?\b|load-bearing|earns its keep|pays off|on the wire|just works|\bmagic\b" docs/<section>/<page>.md
   ```

   **Also grep for soft and contrastive negatives that vale's `PositiveStatements` rule
   misses.** That rule only flags "is not / does not / won't", so negative *definitions*
   and define-by-contrast framing slip through (an opening like "JSON tells you nothing…
   visualization renders it" passed the gate; so did "expressed in world coords that is
   awkward… expressed in the frame…" and "rather than translating it by hand" — both Shannon
   review hits):

   ```bash
   grep -niE "\binvisible\b|\bnothing\b|\bcannot\b|\bcan't\b|\bnever\b|tells you nothing|\bno (single|way|need)\b|rather than|instead of|\bby hand\b|\bwithout\b" docs/<section>/<page>.md
   ```

   Each hit is a candidate to flip positive (say what the thing *is* or *does*), not an
   automatic defect — "cannot" in a genuine limitation may be fine. **Watch for the
   "explain-the-wrong-way-first" structure**: a sentence that sets up the awkward/wrong
   approach before the right one ("in world coords this is messy; in the frame it is clean")
   is the negation drumbeat in disguise. State only what the reader *does*.

   **Grep for overclaiming words** that read as correctness or safety guarantees when the
   intended meaning is "faster / less likely to fail" (Shannon flagged "reliable" reading as
   "the planner sends the end effector to the wrong pose"):

   ```bash
   grep -niE "\breliabl[ey]\b|\brobust\b|\bsafe(ly|r)?\b|\baccurate(ly)?\b|\bguarantee" docs/<section>/<page>.md
   ```

   Keep these only when you mean a true correctness/safety property; otherwise say the
   concrete benefit (faster, fewer failures, larger solution set).

   **Triage the vale output** — not every rule is load-bearing. Prioritize the
   `Viam.*` prose rules that encode house style: `PositiveStatements`,
   `SentenceLength`, `Readability`, `Simplicity`, `AvoidObscure`,
   `GlobalAudienceJargon`, `AvoidFirstPerson`. Treat `write-good.E-Prime`
   ("avoid is/are/be") and most `Viam.Careful`/`Viam.Contractions` hits as advisory
   noise — mention them only if a sentence is genuinely weak. A high
   `PositiveStatements` count on one page (e.g. 5 on a single page) is a strong
   signal of the negation-drumbeat failure, not five separate nits.
3. **Walk the checklist below**, line by line.
4. **Report** grouped by severity, each finding as `path:line — <failure mode>` with
   a one or two line rewrite. End with a short "what reads well" note so the author
   knows what to keep.

## The checklist

### Blocking (a reviewer will comment on these)

1. **Personified service or first-person voice.** Subject of a sentence is a service
   that "wants", "targets", "tries to", "knows", or "decides" → rewrite with the real
   actor (the planner, the method, the module) in active, present-tense voice. Or the
   prose uses first person — "we", "us", "our", "let's" → rewrite in **second person**
   ("you") or make the thing the subject. "Next, add a frame", not "Let's add a frame".
   (Confirmed style decision; the repo's `AvoidFirstPerson` vale rule flags it too.)
2. **Learning-objective leak.** Prose opens with or contains "Understanding…",
   "Learn about…", "Analyze…", "This page will help you understand…", or any Bloom
   verb describing the reader's cognition. (Exception: a tutorial's
   `{{< alert title="Learning Goals" >}}` callout.) → Replace with a concrete claim
   about what the thing *is* or *does*.
3. **Abstract opening / pre-announcement.** First sentence is throat-clearing ("X is
   an important concept", "This page covers…", "In this section we will…") instead
   of stating the concrete thing. → Lead with what the thing is/stores/does, then
   name the API. See exemplars.
4. **Negation drumbeat.** Two or more "is not / does not / doesn't / won't" sentences,
   or a concept defined mainly by what it *isn't*. **Watch the soft negatives vale does
   not flag** — "invisible", "tells you nothing", "cannot", "never", "there is no X" —
   especially in openings that motivate a feature by what something else *fails* to do.
   → Convert to noun-plus-job: give each confusable thing its own role in parallel
   positive sentences, and lead with what the thing *is* or *does*. Allow at most one
   explicit contrast, and only when the distinction is the page's point.
5. **Imprecise API reference.** A method named without its API, or a vague verb
   ("targets", "handles", "deals with") standing in for what it actually does, or a
   value referenced without saying what type/shape it is. → Name the method, its API,
   its parameters, its return, and what the value *is*.
6. **Em dash, idiom, or banned word.** Any em/en dash (`—` / `–`) used as punctuation;
   any idiom or colloquialism; any banned vague word from
   `viam-doc-draft/references/banned-words.md` ("shape" as a vague noun, "spin up",
   "surface" as a verb, "ships", "load-bearing", "on the wire", "magic", "just works",
   filler lead-ins). → Replace the dash with a period/comma/colon; rewrite the idiom or
   banned word as the plain mechanism. **Exception:** the word is allowed when it is
   literally the topic (a geometric "shape", a 3D "surface", a kinematic "model").

### Important (fix before PR)

7. **Jargon stacked in the intro.** Three or more terms a newcomer might not know in
   the opening paragraph, none defined or linked. → Define once (glossary tooltip or
   link to the canonical overview) or replace with plain words.
8. **Overloaded term used loosely.** "Configuration" when a precise word (frame,
   geometry, machine config) is meant; inconsistent terms for one concept
   ("world state" vs "WorldState" vs "obstacles" interchangeably). → Pick one term
   and use it consistently with consistent capitalization.
9. **Prose gate failures** from `vale --minAlertLevel suggestion`: sentences over 25
   words (`SentenceLength`), reading grade above 8 (`Readability`), "easy/simply/
   handy" (`Simplicity`), "via/utilize/e.g." (`AvoidObscure`), "I/we/us/our"
   (`AvoidFirstPerson`), jargon swaps (`GlobalAudienceJargon`). → Apply the suggested
   fix; split long sentences.
10. **Missing conditions-first / prerequisites.** Instructions appear before the
    conditions they depend on, or the page assumes setup with no `## Prerequisites`
    block or links. → Move conditions ahead of instructions; add a linked prerequisites
    list.

### Polish

11. **Frontmatter:** `description` over ~155 characters (Hugo warns at 158), missing
    `linkTitle`/`weight`/`layout`/`type`, or a non-action-oriented description.
12. **Structure:** no concrete opening before sections; missing `## What's next`
    cards on a how-to page; a wall of prose where a parallel bullet list (Unity's
    pose-component pattern) would read better.
13. **Code fidelity (high priority — accuracy outranks style).** Re-verify every API symbol
    against the **code map** at `/home/shrews/viam/viam-code-map`: the `*-xref.md` files for
    signatures (proto is the source of truth) and `flows.md` for behavior (verify behavioral
    claims against RDK code, not proto). Cite `code:<repo>/<path>:<line>` and label each
    claim *verified* or *inferred*; for a list, enumerate from source then reconcile the
    count. Fall back to the SDK skill references (`viam-go-motion-vision/references/`,
    `viam-python/references/`) only for symbols the map lacks, and to raw repos last. Flag any invented method, wrong field
    name, or wrong identifier type, and check the high-risk cases: optional/trailing params,
    exact field names (`physical_object`, `pose_in_observer_frame`), a plain string vs a
    `resource.Name`, read-only vs writable services (the world state store service is
    read-only from a client), and "behind the scenes" platform claims. Also flag code blocks
    missing a language tag, and a variable used but never defined or given a context note.
14. **Figure / alt / code consistency.** The `alt` text must describe the *current* figure
    (re-check after every diagram edit — stale alt slipped through twice on the
    end-effector page). A value shown in a figure (an offset, pose, or shape) must match the
    code, and the same named entity (e.g. an `object-tip` frame) must carry one definition
    across every code block on the page.
15. **Figure readability and triads (render it, don't trust the XML).** Render every changed
    SVG and Read the PNG. Flag: text clipped at an edge, labels overlapping a shape/leader/
    other label, low-contrast labels (text buried on the gray arm or a colored shape). For
    coordinate triads: every frame must be **right-handed**; arm/tool frames that point down
    are **Z-down** (DH), the world frame is Z-up; the red **X** arrow must layer **on top of**
    green Y; and only **one** triad per image carries x/y/z labels (the rest are color-coded).
    See `viam-doc-draft/references/diagrams.md` ("Coordinate triads", "Captions").
16. **Figure caption stands on its own.** Read the one-line caption cold: it must parse
    without the page's context, name only things actually drawn, and state the figure's
    real point. (Dan flagged "…relative to the arm frame at the flange and the world frame
    at the base" as ambiguous — the second clause did not parse alone.)
17. **Every paragraph earns its place (reader value / learning objective).** For each
    paragraph ask: *what does the reader learn here, and which page LO does it serve?* Flag a
    paragraph that is **dense/encyclopedic** (a mechanism derivation the reader does not need
    to act), **redundant** with earlier text, or that no LO needs. (Shannon: "this paragraph
    is super dense and its value is unclear — what LO?" and "I'd cut this, it's redundant.")
    **Depth rule:** advanced mechanism detail belongs as **one reader-facing sentence plus a
    link to a reference/concept page**, not an inline derivation. The orientation-vector
    unit-sphere derivation on the pose-clouds page is the cautionary example — accurate, but
    wrong depth for a how-to.
18. **One running example per page.** Examples named early must be the ones used later. A page
    that opens with a soup can but works a cup-of-water example reads as two unrelated ideas.
    → Pick one example (the worked one) and use it in every list, figure, and snippet. Also
    keep one idea per bullet: do not blend, e.g., placing and picking in a single bullet.
19. **A figure must teach its concept, not just render cleanly.** Beyond item 15's legibility
    checks: would a reader who has not read the prose grasp the section's *subject* from the
    figure? The subject must be the visually dominant thing. (Shannon: "we're talking about
    the end-effector frame but I just see a box.") If the alt text claims something the eye
    does not land on first, the figure fails — redraw so the subject is salient.
20. **Overclaiming words.** "reliable / robust / safe / accurate / guarantee" read as
    correctness or safety properties. Keep them only when you mean exactly that; when you mean
    "faster / fewer failures / larger solution set," say the concrete benefit. After fixing
    such a word in the body, grep the frontmatter `description` for the same wording.

## Report format

```
## Review: docs/<section>/<page>.md  (Diátaxis: explanation | how-to | …)

### Blocking
- L10 — Personified service. "the motion service targets the frame" → "The motion
  planner solves for the joint angles that place the frame's origin at the goal pose."
- L13 — LO leak. Opens "Understanding frames…" → lead concrete: "The frame you
  specify is typically the arm's end effector."

### Important
- L7 — Description is 159 chars; trim below 155.
- L84 — Negation drumbeat (3× "does not"); convert to noun-plus-job (see rewrite).

### Polish
- L48 — Code block missing `go` language tag.

### Reads well
- The "Anatomy of a transform" bullet list uses parallel grammar — keep it.
```

Always close with the "Reads well" note. Reviews that only list faults are demoralizing
and hide which patterns the author should repeat.
