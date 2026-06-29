# Model openings and before/after rewrites

Keep this open while drafting. Imitate the *shape* of these openings, not the words.

## Model openings to imitate

**Concept → short form → API (Drake, pose):**

> A spatial pose, more commonly just pose, provides the location and orientation of a
> frame B with respect to another frame A. … In Drake this concept is provided by the
> `RigidTransform` class.

**Stores X, then the invariant (Unity, transform):**

> The Transform stores a GameObject's Position, Rotation, Scale and parenting state. A
> GameObject always has a Transform component attached: you can't remove a Transform or
> create a GameObject without a Transform component.

**Concrete robot setup, then the gap it creates (Viam, frame-system overview):**

> A robot with an arm, a camera, and a gripper has three components, and each one reports
> positions in its own local coordinate system. The camera sees an object at pixel
> (320, 240), but the arm needs that object's position in three-dimensional space relative
> to its own base. Without a unified spatial model, you cannot translate between
> coordinate systems.

**Mechanical statement of what happens (Viam, how-planning-works):**

> Motion planning finds a safe, joint-level path from one arm configuration to another.
> The planner takes the arm's kinematic model, the target pose, the world state, and any
> motion constraints, and it returns a sequence of joint configurations the arm can
> execute without colliding with itself or its environment.

**Shannon's rewrite of the end-effector opening (the target for that page):**

> To move an arm using the Viam motion service, you specify a desired pose for a specific
> frame defined within the frame system. The motion planner solves for joint angles that
> place that frame's origin in the destination pose. The frame you specify is typically
> the frame of the end effector for the arm.

## Before / after (drawn from the visualization pages)

### Personification → real actor

- Before: "The world state store service streams these transforms to the scene."
  (fine) but "the motion service targets the frame it wants" (not fine).
- After: "The motion planner solves for the joint angles that place the frame's origin
  at the goal pose."

### LO leak → concrete

- Before: "Understanding how geometry attaches to a transform is important because…"
- After: "A transform carries a geometry — the box, sphere, or mesh the scene draws —
  positioned by a pose in a parent frame."

### Negation drumbeat → noun + job (the biggest visualization rework)

- Before: "A visual is not an obstacle. The motion planner does not read the world state
  store. Publishing a box to the scene does not add an obstacle the arm will avoid."
- After: "The renderer draws a world state store transform so you can see it in the 3D
  scene. The motion planner collision-checks a separate geometry — each component's
  `frame.geometry` and the `WorldState` you pass to `Move`. A shape can appear in both,
  and each path has its own job: one is for seeing, one is for planning."

### Vague verb → API precision

- Before: "`Move` targets a frame."
- After: "`Move` is a method in the motion service API. Its goal is a `PoseInFrame`: the
  destination pose for a named frame's origin, expressed in a reference frame."

### Overloaded term → precise term

- Before: "The component's configuration resolves to a frame."
- After: "The component name resolves to a defined frame in the frame system."

## The "two confusable things" pattern (reusable)

When a page must distinguish two things readers conflate, use this positive structure
instead of a list of "X is not Y" sentences:

1. Name thing A and its **job** in one sentence (active, present tense).
2. Name thing B and its **job** in one sentence, parallel grammar.
3. One sentence on how they relate ("a shape can be both") and that each has its own path.

Unreal's collision docs are the model: render mesh and collision shape are presented as a
pair the engine maintains together, each "used for" a stated purpose — the reader infers
the distinction from the two roles, with no negation.
