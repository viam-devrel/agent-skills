# The LO-to-page framework: rationale and sources

This file records *why* each move in the framework exists, traced to a source, so
the conventions can be defended and updated rather than treated as arbitrary.

## Origin: Shannon's review of PR #5098

Nine inline comments on `end-effector-frames.md` collapsed into seven patterns.
They are the seed of this framework:

| Shannon's comment | Move it maps to |
|---|---|
| "This personifies the motion service. That's the wrong tone for docs." | Move 4 — name the real actor |
| "Rather than the sentence beginning 'Understanding…' make the abstract concrete." | Moves 2 + 3 — no LO leak, concrete first |
| "'targets frames' — what does that mean?" | Move 6 — API precision |
| "Explain what this page is without so many terms that might not be familiar." | Move 7 — vocabulary budget |
| "Be precise. `Move` is a method in the motion service API. … what the destination is exactly." | Move 6 — API precision |
| "We don't want to say what something is not… The component name resolves to a defined frame." | Move 5 — positive statements |
| "Problematic to use the word 'configuration' here given how much we talk about configuration." | Move 6 — term precision |
| "Do we need to summarize what a pose is? … define pose and frame clearly in a place we can reference." | Move 7 — canonical definitions |

## The false-confidence gap

CLAUDE.md's pre-PR check runs `vale` but only **fails on `error`**. The prose-quality
rules that encode Viam house style are `suggestion`/`warning` level, so "vale: 0
findings" passed while the prose still violated house style. Shannon was hand-enforcing
the suggestion-level rules the gate skips. Move 8 closes this by running
`vale --minAlertLevel suggestion` on prose.

Relevant repo rules (in `.github/vale/styles/Viam/`):

- `PositiveStatements.yml` — flags "is not / does not / won't / can't" → Move 5.
- `AvoidObscure.yml` (error) — "via"→through/by/with, "e.g."→for example, "utilize"→use.
- `Simplicity.yml` — flags "easy/easily/simple/simply/handy/useful/obviously".
- `SentenceLength.yml` — under 25 words per sentence.
- `Readability.yml` — Flesch-Kincaid / Gunning Fog grade 8 max.
- `GlobalAudienceJargon.yml` — "execute"→run, "launch"→start, "kill"→stop, etc.
- `AvoidFirstPerson.yml` — no "I/we/us/our"; second person is the house voice.

## Diátaxis (diataxis.fr)

- Four modes on two axes (action↔cognition, acquisition↔application): tutorial,
  how-to, reference, explanation.
- Explanation is "understanding-oriented… a discursive treatment that permits reflection."
- The no-mixing rule (quoted): "One risk of explanation is that it tends to absorb
  other things." Mixing "is bad for the reference, interrupted by digressions. But
  it's bad for the explanation too, because it's not allowed to develop appropriately."
- A "concept + how-to code" page is two modes conflated → split or cleanly section.
- Sources: https://diataxis.fr/explanation/ , /how-to-guides/ , /reference-explanation/ , /compass/

## Google developer documentation style guide

- "Don't pre-announce anything." (no "in this section we will…") → Move 3.
- Use second person ("you", not "we"); use "user" only for the reader's own end users.
- Present tense for general behavior; avoid hypothetical "would". → Move 4.
- Active voice: make clear who performs the action. → Move 4 (anti-personification).
- Put conditions before instructions. → Move 7.
- Use the same term consistently, same capitalization. → Move 6.
- Sources: developers.google.com/style/highlights, /tense, /person, /translation

## Microsoft Writing Style Guide

- "Get to the point fast. Lead with what's most important." → Move 3.
- "Start each statement with a verb" / "Prune every excess word."
- Voice: warm and relaxed, crisp and clear, "scanning first, reading second."
- "Shun jargon and acronyms." → Move 7.
- Sources: learn.microsoft.com/style-guide/top-10-tips-style-voice, /brand-voice-above-all-simple-human

## Drake (drake.mit.edu) — robotics/spatial exemplar

- Concept → short form → API: "A spatial pose, more commonly just pose, provides the
  location and orientation of a frame B with respect to another frame A." Only then the
  `RigidTransform` class. → Move 3.
- Reference-target-expressed-in grammar, math form beside code form beside English
  meaning in one table. Good template for a frame/pose reference page.
- Justify a convention by the bug it prevents ("verification by rote pattern matching").
- Anti-patterns to avoid: abstract throat-clearing openings ("translating … is a
  difficult process and requires careful discipline"), undscaffolded symbol soup,
  first-person hedging ("I don't love it myself"), fragmenting concept/notation/example
  across separate pages.
- Sources: drake.mit.edu/doxygen_cxx/group__multibody__notation.html (and …__basics,
  …__frames__and__bodies, …__spatial__pose, …__quantities), manipulation.mit.edu/pick.html

## Unity & Unreal — 3D scene / transform / visualization exemplar

- Open a Transform page by stating what it **stores**, then the invariant: "The
  Transform stores a GameObject's Position, Rotation, Scale and parenting state… you
  can't create a GameObject without a Transform." → Move 3.
- Define the three pose components in strictly parallel grammar with a unit anchor each
  ("Rotation… around the x/y/z-axis, measured in degrees").
- Teach parenting via inherited motion verbs: "A child GameObject moves, rotates, and
  scales exactly as its parent does."
- Behavior first, label second for local/world space.
- **Render-vs-simulation without negation** (the key fix for the visualization pages):
  give each artifact a noun and a job. "the physics solver will use the corresponding
  shape for scene queries and collision tests" — render mesh and collision shape are a
  pair, each with a stated job, no "X is not Y". → Move 5.
- Anti-pattern: a page full of disclaimers ("this isn't rendered", "won't affect
  physics") reads as a list of negations. Assign roles instead.
- Sources: docs.unity3d.com/Manual/class-Transform.html, /gizmos-and-handles.html,
  /UsingTheSceneView.html ; dev.epicgames.com/documentation/unreal-engine/
  coordinate-system-and-spaces, /transforming-actors, /simple-versus-complex-collision

## Write the Docs / Docs for Developers

- "Write like a newspaper instead of a novel"; begin paragraphs and list items with the
  identifiable concept as early as possible. → Move 3.
- "Structure content to help readers identify and skip concepts they already understand."
  → Move 7 (prerequisites, scannability).
- Developers read non-linearly — often code first, concept second — which is another
  argument for concrete-first openings even on explanation pages.
- Sources: writethedocs.org/guide/writing/docs-principles/ ; Bhatti et al., *Docs for
  Developers*.
