# Diagrams for Viam docs (spatial / frame / scene figures)

Spatial concepts (frames, poses, offsets, collision geometry, scene-vs-plan) are far
clearer as a labeled diagram than as prose, and a stock photo cannot show a coordinate
frame. For these, author a **schematic SVG with labeled RGB axes** (the Drake / Unity /
Unreal convention), not a photo. This file is the design system and the workflow that
produced the end-effector-frames diagrams.

## When to make one

- A frame, pose, offset, or coordinate convention → a labeled coordinate-frame diagram.
- "What renders vs what the planner uses" → give each artifact a noun and a job in the
  figure (a translucent dashed box for a collision geometry, a solid shape for hardware).
- Hardware variety (gripper vs drill vs held object) → one figure per panel, same arm.

## Design system

Keep every figure visually consistent by reusing the same symbols, palette, and type.

- **Palette**: arm links `#c2cad4` fill with `#444b54` outline; joints `#aab2bd`; base
  `#6b7480`. Axes: X `#e23b3b`, Y `#2fa84f`, Z `#2f6fe0`; origin dot `#1f2937`. Grasped
  object amber `#d9a441`/`#9c6f1d`. Workpiece gold `#e0bd6b`/`#c99c42`/`#ad8836` (top/
  front/side). Collision geometry translucent `#f59e0b` at `fill-opacity:0.20`, dashed
  `#d97706` outline. Captions/labels `#41506a`; bold labels `#1f2937`.
- **Type**: titles 15 bold; bold labels 12.5; captions 12; axis letters 11.5 italic in
  the axis color.
- **Triads**: `triadL` (length 24, x/y/z letters) for the focus frame; `triadS`
  (length 16, no letters) for context frames (world, arm).
- **Always** put `width` and `height` on the root `<svg>` (not just `viewBox`) so Hugo's
  image processing can read dimensions.

Reusable `<defs>` block — paste into a new figure, then place symbols with
`<use href="#armP" x=".." y=".."/>`. The arm's flange origin lands at
`(use.x + 196, use.y - 114)`; hang a tool from there. World/arm frames are `triadS`,
the end effector frame is `triadL`.

```svg
<defs>
  <marker id="aR" markerWidth="10" markerHeight="10" refX="7" refY="3.2" orient="auto"><path d="M0,0 L8,3.2 L0,6.4 z" fill="#e23b3b"/></marker>
  <marker id="aG" markerWidth="10" markerHeight="10" refX="7" refY="3.2" orient="auto"><path d="M0,0 L8,3.2 L0,6.4 z" fill="#2fa84f"/></marker>
  <marker id="aB" markerWidth="10" markerHeight="10" refX="7" refY="3.2" orient="auto"><path d="M0,0 L8,3.2 L0,6.4 z" fill="#2f6fe0"/></marker>
  <marker id="arw" markerWidth="11" markerHeight="11" refX="8" refY="3.6" orient="auto"><path d="M0,0 L9,3.6 L0,7.2 z" fill="#475569"/></marker>
  <symbol id="triadL" overflow="visible">
    <line x1="0" y1="0" x2="24" y2="0" stroke="#e23b3b" stroke-width="2.4" marker-end="url(#aR)"/><text x="28" y="4" fill="#e23b3b" font-size="11.5" font-style="italic">x</text>
    <line x1="0" y1="0" x2="0" y2="-24" stroke="#2f6fe0" stroke-width="2.4" marker-end="url(#aB)"/><text x="4" y="-25" fill="#2f6fe0" font-size="11.5" font-style="italic">z</text>
    <line x1="0" y1="0" x2="-16" y2="11" stroke="#2fa84f" stroke-width="2.4" marker-end="url(#aG)"/><text x="-26" y="19" fill="#2fa84f" font-size="11.5" font-style="italic">y</text>
    <circle cx="0" cy="0" r="3" fill="#1f2937"/>
  </symbol>
  <symbol id="triadS" overflow="visible">
    <line x1="0" y1="0" x2="16" y2="0" stroke="#e23b3b" stroke-width="2" marker-end="url(#aR)"/>
    <line x1="0" y1="0" x2="0" y2="-16" stroke="#2f6fe0" stroke-width="2" marker-end="url(#aB)"/>
    <line x1="0" y1="0" x2="-11" y2="8" stroke="#2fa84f" stroke-width="2" marker-end="url(#aG)"/>
    <circle cx="0" cy="0" r="2.6" fill="#1f2937"/>
  </symbol>
  <symbol id="armP" overflow="visible">
    <ellipse cx="0" cy="5" rx="44" ry="7" fill="#000000" opacity="0.08"/>
    <rect x="-30" y="-20" width="60" height="22" rx="5" fill="#6b7480" stroke="#444b54" stroke-width="1.5"/>
    <rect x="-11" y="-31" width="22" height="13" rx="3" fill="#7a828d" stroke="#444b54" stroke-width="1.2"/>
    <line x1="0" y1="-30" x2="64" y2="-150" stroke="#444b54" stroke-width="20" stroke-linecap="round"/>
    <line x1="0" y1="-30" x2="64" y2="-150" stroke="#c2cad4" stroke-width="15" stroke-linecap="round"/>
    <line x1="64" y1="-150" x2="196" y2="-150" stroke="#444b54" stroke-width="20" stroke-linecap="round"/>
    <line x1="64" y1="-150" x2="196" y2="-150" stroke="#c2cad4" stroke-width="15" stroke-linecap="round"/>
    <line x1="196" y1="-150" x2="196" y2="-122" stroke="#444b54" stroke-width="18" stroke-linecap="round"/>
    <line x1="196" y1="-150" x2="196" y2="-122" stroke="#c2cad4" stroke-width="13" stroke-linecap="round"/>
    <circle cx="0" cy="-30" r="11" fill="#aab2bd" stroke="#444b54" stroke-width="1.5"/><circle cx="0" cy="-30" r="4" fill="#7a828d"/>
    <circle cx="64" cy="-150" r="11" fill="#aab2bd" stroke="#444b54" stroke-width="1.5"/><circle cx="64" cy="-150" r="4" fill="#7a828d"/>
    <circle cx="196" cy="-150" r="10" fill="#aab2bd" stroke="#444b54" stroke-width="1.5"/><circle cx="196" cy="-150" r="3.5" fill="#7a828d"/>
    <rect x="180" y="-124" width="32" height="10" rx="2" fill="#b7bfca" stroke="#444b54" stroke-width="1.2"/>
  </symbol>
  <symbol id="gripperP" overflow="visible">
    <rect x="-18" y="0" width="36" height="13" rx="3" fill="#9aa3af" stroke="#444b54" stroke-width="1.2"/>
    <rect x="-15" y="13" width="8" height="34" rx="2" fill="#b7bfca" stroke="#444b54" stroke-width="1.1"/>
    <rect x="7" y="13" width="8" height="34" rx="2" fill="#b7bfca" stroke="#444b54" stroke-width="1.1"/>
  </symbol>
  <symbol id="drillP" overflow="visible">
    <rect x="-16" y="0" width="32" height="30" rx="7" fill="#5f6873" stroke="#3c424a" stroke-width="1.2"/>
    <path d="M-8,30 L8,30 L5,41 L-5,41 Z" fill="#9aa3af" stroke="#444b54" stroke-width="1"/>
    <line x1="0" y1="41" x2="0" y2="64" stroke="#5b626c" stroke-width="3.5" stroke-linecap="round"/>
  </symbol>
  <symbol id="cup" overflow="visible">
    <path d="M-13,-34 L13,-34 L10,0 L-10,0 Z" fill="#eef4fb" stroke="#6f96b8" stroke-width="1.3"/>
    <path d="M-11.5,-26 L11.5,-26 L10,0 L-10,0 Z" fill="#4a90d9"/>
    <ellipse cx="0" cy="-34" rx="13" ry="3.4" fill="#cfe0f0" stroke="#6f96b8" stroke-width="1.1"/>
    <ellipse cx="0" cy="-26" rx="11.5" ry="2.6" fill="#6fb0e8"/>
  </symbol>
</defs>
```

Inline patterns (not symbols, copy and translate):

- **Isometric workpiece** (origin = top-center, place a `triadL` on top for a target):
  `<path d="M515,236 L645,236 L677,210 L547,210 Z" fill="#e0bd6b" stroke="#7a5e22"/>`
  (top), then a front rect and a right parallelogram in `#c99c42` / `#ad8836`.
- **Collision geometry**: a `rect` with `fill="#f59e0b" fill-opacity="0.20"
  stroke="#d97706" stroke-width="1.6" stroke-dasharray="6 4"` wrapping the object.
- **Action arrow**: `<path ... fill="none" stroke="#475569" stroke-width="1.6"
  stroke-dasharray="5 4" marker-end="url(#arw)"/>`.
- **Isometric work surface (table)**: a two-face block — a top parallelogram `fill="#d9c9a8"`
  then a front rect `fill="#c2ad82"`, both `stroke="#9c8a63"`. Flatter and larger than the
  workpiece; objects and regions sit on its top face.
- **Region on a surface (a pose cloud)**: a rounded `rect` skewed onto the iso top face with
  `transform="matrix(1,0,-0.46,0.89,0,0)"`, `fill="#0000ff" fill-opacity="0.21"`, dashed
  outline. The clean way to draw "a set of acceptable poses" lying flat on a table.
- **Example instance**: convey a region's meaning with one labeled instance of the moved
  object placed inside it (`<use href="#cup" transform="translate(dx,dy)"/>` labelled
  "valid destination"), rather than abstract ghost copies.
- **Numbered waypoint**: `<circle r="12" fill="#2f6fe0" stroke="#1e4fa0" stroke-width="1.5"/>`
  with a white centered number on top.
- **Obstacle**: the isometric-box pattern in grays (`#aab2bd`/`#8893a3`/`#6b7480`), next to
  the gold workpiece (`#e0bd6b`/`#c99c42`/`#ad8836`).

## Reference figures (the quality bar — study these first)

These shipped figures are the canonical examples. Open and render one before starting a new
figure, and match its composition, weight, spacing, and label placement:

- `assets/motion-planning/move-an-arm/pose-cloud-cup.svg` — an arm holds an object, a dashed
  arrow leads to a flat translucent **region** on an iso table, with the target frame and one
  labeled example placement. The model for "a region of acceptable poses".
- `assets/motion-planning/move-an-arm/waypoint-trajectory.svg` — two contrasting dashed paths
  (each labeled) through a scene with an obstacle and numbered waypoints. The model for
  "this approach versus that approach".
- `assets/motion-planning/frame-system/end-effector-types.svg` and `arm-vs-gripper-frame.svg`
  — arms holding tools, with labeled coordinate frames on the flange and tool point.

When these and your draft disagree, the shipped figures win.

## Render-and-verify loop (do not author SVGs blind)

You cannot judge a diagram from XML. After every change, render and Read the PNG:

```bash
google-chrome --headless --disable-gpu --no-sandbox --force-device-scale-factor=1 \
  --hide-scrollbars --default-background-color=FFFFFFFF \
  --screenshot=/tmp/fig.png --window-size=900,700 \
  "file:///abs/path/to/figure.svg"
```

Then Read `/tmp/fig.png`. Use a window **taller and wider than the SVG** (headless Chrome
crops ~15% off the bottom otherwise; size the window to the SVG plus ~40 px each way).

After every render, scan specifically for these failures — they recur constantly:

1. text clipped at any edge (especially bottom captions and corner labels),
2. text overlapping a shape, an axis, a leader, or another label — and check **contrast**
   (a label sitting on the gray arm or a colored shape must still read; nudge it to clear
   space or onto white),
3. a colored or dashed element with no label or legend,
4. a leader line crossing through a shape,
5. large empty regions (the canvas is bigger than the content),
6. **triad layer order**: the red **X** arrow must draw *on top of* the green **Y** arrow
   where they overlap, and the origin dot on top of both. Draw order in the symbol is
   Y, then Z, then X, then the dot,
7. **triad handedness**: every frame must be right-handed. Confirm with the rendered
   arrows, not the XML (see "Coordinate triads" below),
8. **more than one labeled triad**: only one triad per image carries x/y/z text (the
   legend); every other frame is the bare color-coded arrows,
9. **caption mismatch**: the caption names something not drawn, omits the figure's actual
   point, or only parses with outside context (read it cold — see "Captions" below),
10. **concept not taught**: the hardest test. Cover the prose and look at the figure cold —
    would a reader grasp the section's *subject* from it? The subject must be the visually
    dominant element, not an incidental box or arrow. If the section is about the end-effector
    frame but the eye lands on a region box, the figure fails (a real Shannon review hit). The
    `alt` text's first claim must match what you see first. Redraw so the subject is salient,
    not just legible.

Fix and re-render until all pass. Do not embed a figure you have not eyeballed clean.

## Embedding in a page

- Store the SVG in `assets/<section>/<subsection>/name.svg`. Reference it as
  `src="/<section>/<subsection>/name.svg"` (the `assets/` prefix is stripped).
- Use the `imgproc` shortcode with `declaredimensions=true`, an `alt`, a `style`
  max-width, and `class="aligncenter"`. No `resize` on SVG.

```text
{{<imgproc src="/motion-planning/frame-system/name.svg" declaredimensions=true alt="..." style="max-width:760px" class="aligncenter">}}
```

## Text and label placement (the #1 source of rework)

SVG has no auto-layout: text never reflows or avoids shapes, so every label is positioned
by hand and clipping/overlap is the *default* failure. The defects that recur are clipped
labels, labels sitting on shapes, floating labels with no referent, and unlabeled color
encodings. Rules that prevent them:

- **Estimate text size before placing it.** A label is about `0.58 × fontSize × charCount`
  px wide and `fontSize` px tall (cap height ~0.7 × fontSize above the baseline, descender
  ~0.2 below). Use that estimate to keep labels off shapes and inside the canvas — you
  cannot eyeball it from the XML.
- **Anchor by side.** Label to the *left* of its referent: `text-anchor="end"` with
  `x − estWidth ≥ 14`. To the *right*: default anchor with `x + estWidth ≤ width − 14`.
  Title/caption: `text-anchor="middle"`. This is what stops right-side labels running off
  the canvas and left-side labels colliding with the figure.
- **Keep a 14 px margin from every edge**, and put caption baselines **≥ 18 px above the
  bottom** (descenders plus Chrome's bottom crop clip anything closer — this is why "world
  frame" labels kept disappearing).
- **Leader or adjacency, never floating.** A label that does not touch its referent gets a
  thin leader (`stroke="#9aa3af" stroke-width="1"`) from the label to the thing it names.
  Route leaders through empty space, not across other shapes.
- **Label every encoded element.** When color or dash style carries meaning (a red path vs
  a blue path), each one needs its own label or a small legend. An unlabeled colored line
  is a defect, not a clean minimalism.
- **One label per ~80 px of region.** When an area needs several labels (a frame origin
  with axes, a cone, a rotation arc), space them out or stack them as a leadered callout;
  do not pile them onto the busy spot.
- **Sentence case** for all labels and titles, matching house prose style.

## Layout and shapes

- **Fit the canvas to the content.** Find the content bounding box, add margins (~16 px
  sides, ~28 px under the title, ~40 px for a bottom caption), and set `width`/`height`/
  `viewBox` to that. Large empty regions mean the canvas is too big — crop it.
- **Ground the arm** (the `armP` shadow ellipse) and put the world frame **on the ground
  beside the base**, not inside the base block (it gets buried and its label clips).
- The focus frame is `triadL` with a bold label; context frames are small `triadS`.
- **Rotation/arc arrows: use a `<path>` arc with `marker-end`.** SVG markers do not render
  on `<ellipse>` or `<circle>`, so an ellipse "twist" arrow shows no arrowhead.
- **Re-check the `alt` text whenever the figure changes** (a bent object, a new frame).
  Stale alt text slipped through twice.

## Coordinate triads (frames)

Axis colors are fixed: **X red `#e23b3b`, Y green `#2fa84f`, Z blue `#2f6fe0`**, arrowheads
via `marker-end` with `orient="auto"` so they rotate with the line.

- **Right-hand rule, always.** Every frame must be right-handed; verify from the rendered
  arrows, not the XML. The catch with a downward tool: you **cannot** have Z-down with
  X-right *and* Y-down-left — that combination is left-handed. The two right-handed Z-down
  options are X-right/Y-up-right (rotate the Z-up triad 180° about X) or X-left/Y-down-left
  (180° about Y). Do not invent a third that looks symmetric but is left-handed.
  **Decided convention for these docs:** Z-down tool/end-effector frames use
  **X-left, Y-down-left** (180° about Y), so the green Y stays on the same lower-left side
  as the Z-up world frame's Y. Keeping all the greens on one side is what reads as correct;
  give Y a slightly longer line than X/Z so the green reads clearly.
- **Z direction follows the frame.** The world/base frame is Z-up (X-right, Y-down-left).
  An end-effector or tool frame that points down (a drill bit, a gripper reaching down)
  is Z-down per the DH convention most cobots use. When a diagram shows arm/tool frames
  relative to the world, the reviewer expects their Z to point *down*. Confirm each frame's
  intended Z before drawing.
- **One legend per image.** Because the axes are color-coded, only **one** triad shows
  x/y/z text labels (the legend); every other frame is the bare colored arrows. Put the
  legend on a frame with open space, and add the labels as **standalone `<text>` at full
  font size** — do not rely on a triad symbol's own labels, which shrink with the symbol's
  scale and become unreadable. A Z-down legend's `z` lands in the tool body; that is fine
  if it still reads (blue on gray is legible; nudge if not).
- **Layer order is X over Y.** In the symbol, draw the lines in order Y, Z, X, then the
  origin dot, so the red X arrow sits on top of green Y where the arrowheads overlap.

## Captions

The one-line caption under a figure is read cold, by someone who has not read the page.

- **Self-contained.** It must parse on its own. "Defined relative to the arm frame at the
  flange and the world frame at the base" drew a review comment because the second clause
  is ambiguous without context — say one clear thing, or split into two clear statements.
- **Matches the drawing.** Every noun in the caption must appear in the figure, and the
  figure's actual point must be the caption's point. Do not caption a frame, offset, or
  path that is not drawn, and do not leave the figure's main idea unstated.
- **Re-read after edits.** When you move or remove an element (a frame, a clause Dan
  flagged), re-read the caption against the new picture.

## Inkscape workflow gotcha

When an author edits an SVG in Inkscape, Hugo's dev-server file watcher misses the save
(Inkscape writes via a temp-file swap), so the page keeps serving the old figure. To
refresh: `pkill -9 -f "hugo server"; rm -rf public/ resources/_gen/; hugo server`.
Inkscape also bloats the file with `sodipodi:`/`inkscape:` metadata and duplicate
markers — harmless to the build; run `svgo` before committing if you want it lean.
