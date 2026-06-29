# Banned idioms, words, and em dashes

Source of truth: the viam-training curriculum's `worksheet-style-guide.md` ("Banned
idioms and prose"). These have been flagged repeatedly in review. **Do not use them in
docs prose unless the word is literally the topic** — e.g. "shape" is fine on a page
about geometric shapes, "surface" is fine when discussing a 3D surface or NURBS, "model"
is fine when it means an ML or kinematic model. The ban is on the *vague/evocative* use,
not the technical one.

## No em dashes

Never use em dashes (`—`) or en dashes (`–`) as sentence punctuation. Rewrite with a
period, a comma, a colon, or parentheses. (The repo's vale `Dashes`/`DashesSpaces` rules
also enforce this.) Hyphens in compound words (`end-effector`, `world-frame`) are fine.

## No idioms

No idioms or colloquialisms (Google: "avoid colloquialisms, idioms, or slang" — they
don't translate and slow non-native readers). Say the mechanism plainly.

## Banned words and phrases

The test for each: **does the sentence describe what the thing _does_ (good) or how it
_feels_ (bad)?** If the latter, rewrite as a mechanism.

| Banned (vague use) | Use instead |
|---|---|
| **"shape"** as a vague noun ("the same shape", "the shape of the problem") | say what is actually common between the things |
| **"dance"** for a coordination or sequence | name the actual steps |
| **"stand up" / "spin up"** for creating/starting | "create", "start", "run", "bring up" |
| **"surface"** as a verb ("surface an error") | "report", "show", "return" |
| **"lands" / "drops in" / "drops into"** for code/content added | "is added in", "joins", "starts being used in" |
| **"ships"** for content/features | describe what it is, or "is delivered in" / "is available in" |
| **"earns its keep" / "pays off" / "buys" / "the payoff is"** | lead with the content, not the value-framing |
| **"load-bearing"** | "essential", or say what depends on it |
| **"the cheap part is X" / "the expensive part is X"** | describe the actual work or cost |
| **"the classic <bug>" / "the classic <pattern>"** | drop the chatty appeal to familiarity |
| **"from the operator's seat"** and perspective-shift phrases | state it directly |
| **"wrist gymnastics"** and evocative non-technical mechanism descriptions | describe the actual motion/mechanism |
| **"on the wire"** for serialized form | "the response", "the request", "what you get back" |
| **"magic" / "just works" / "beautifully"** and intensifiers | describe the behavior |
| filler lead-ins: **"Now, the fun part…", "you'll meet…", "we'll explore…"** | delete; get to the content |

This list also overlaps the repo's vale rules: `Simplicity` ("easy/simply/handy"),
`GlobalAudienceJargon` ("execute/launch/kill/spin up"), and `AvoidObscure` ("via/utilize").
When the training list and a vale rule disagree, prefer whichever is stricter.

## Keeping the list current

The canonical list lives in viam-training `worksheet-style-guide.md`. When that guide adds
a banned term, mirror it here so the docs skills stay in sync. The grep check in the review
skill reads from this table's left column.
