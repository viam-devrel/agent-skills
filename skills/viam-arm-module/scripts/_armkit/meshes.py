"""Mesh geometry inspection for an already-parsed KinematicModel.

Why this exists: `checks.py`'s `mesh-references` finding can only COUNT
how many mesh files a model references -- it never touches disk, so it
can't say whether any of them actually exist. An acceptance review of 84
real vendor URDFs found mesh resolution to be the single most common
reason RDK rejects a file armkit itself passes (confirmed on
`cr5_robot.urdf` and both mycobot gripper URDFs): "armkit PASS, then RDK
rejects on meshes" is the most likely bad first experience this toolkit
produces. This module closes that gap -- resolve each mesh reference
against disk, load it, and report what's actually there.

Nothing here raises. Like checks.py, an unresolvable or unloadable mesh
is exactly the finding this module exists to produce, not a reason to
abort -- inspect_meshes reports every mesh reference in the model,
whatever state it's in, and keeps going. This module does NOT wire into
checks.py or add a CLI subcommand -- that's A8's job, after it first
extracts report.py; this is the module and its tests only.

**Path resolution.** Two forms appear in real URDFs:
  - `package://some_pkg/meshes/base.dae` -- resolved by walking UP from
    the URDF's own directory, checking at each ancestor level whether a
    child directory named `some_pkg` exists (mimicking ROS package
    resolution without a real ROS environment). Verified against a real
    mycobot URDF (`package://mycobot_description/urdf/.../base.dae`,
    resolves two levels up) and shown to correctly NOT resolve a UR
    vendor URDF's `package://urdfs/...` (no directory named `urdfs`
    exists anywhere above it -- see this task's real-vendor-URDF report).
  - `meshes/link1.stl` -- relative to the URDF's own directory.
An unresolvable path is reported with `resolved=False`, not raised.

**Loading and the three-way state.** A mesh reference is in one of three
states, not two:
  1. `resolved=False` -- no file found at the resolved path at all.
  2. `resolved=True, load_error=<str>` -- a file exists there, but it was
     never turned into a loaded mesh, for one of two reasons: trimesh
     either raised loading it (e.g. malformed COLLADA XML) or loaded it
     "successfully" into a mesh with zero triangles (trimesh's own
     behavior for e.g. plain-text content given an .stl extension -- it
     does NOT raise for that, it silently returns an empty scene, so this
     module checks explicitly rather than relying on exceptions alone) --
     OR loading was never attempted at all, because `inspect_meshes` was
     called with `load=False` (see below); `load_error` is then set to a
     fixed "not loaded" message rather than left None, specifically so
     this state reads uniformly regardless of WHY the mesh never became
     available, and so `MeshReport.loaded` (below) is False for all of
     it. `triangles`/`bbox_mm` stay None in every sub-case; `bytes` (a
     plain disk stat, no trimesh needed) and `origin_offset_mm` (from the
     URDF, not the mesh file) are still populated.
  3. `resolved=True, load_error=None` -- loaded cleanly; every field is
     populated.
This third state matters because it is a DIFFERENT failure than a
missing file -- "the path is right but the asset is broken" needs a
different fix than "the path is wrong" -- and collapsing it into either
of the other two would lose that distinction for whoever reads the
report (A8). Use `MeshReport.loaded` to test for state 3 specifically;
`resolved` alone does not imply `triangles is not None`.

**The `load` parameter -- resolution is orders of magnitude cheaper than
loading.** Measured on a real 14-reference mycobot URDF
(mycobot_adaptive_gripper.urdf): parsing the file is ~0.3 ms, resolving
all 14 mesh paths against disk (this function with `load=False`) is
~0.3 ms, and loading all 14 through trimesh (`load=True`, with the
caching below already applied) is ~110 ms on this machine with the mesh
files already in the OS page cache from earlier corpus-wide scans --
call it 300-1500x depending on disk cache state, not a fixed constant.
`unresolved-mesh` -- the finding this module exists to enable, and the
single biggest PASS-vs-RDK-loads gap per the acceptance review that
motivated it -- needs only resolution. `inspect_meshes(model,
load=False)` skips trimesh entirely: every mesh reference is still
resolved against disk (so `resolved`/`resolved_path`/`bytes`/
`origin_offset_mm` are exactly as if `load=True`), but nothing is
loaded, so `triangles`/`bbox_mm` stay None and `load_error` carries the
fixed "not loaded" message described above. `validate` (a hot path an
agent may run repeatedly) should call this with `load=False`; only a
`meshes`/`heavy-mesh`-style consumer that actually needs geometry should
pay the loading cost.

**Caching.** Vendors commonly reference the SAME mesh file from both
<visual> and <collision> on a link (or reuse one mesh across several
links). Measured on that same 14-reference URDF: only 7 distinct files;
loading each of the 14 references independently (no caching) took
~183 ms, loading the 7 distinct files once each and reusing the result
(this function's cache, keyed on resolved path, scoped to a single call)
took ~108 ms -- a real reduction, though not exactly half, since loading
cost isn't uniform across files. This lives here, not in a caller,
because it is purely an artifact of how this function walks the model,
not a policy choice A8 should own.

**Units.** Internal values are always mm (see model.py's module
docstring) -- but mesh files are not uniformly one unit. Measured, not
guessed:
  - STL/OBJ/PLY carry no unit metadata; trimesh reports `.units is None`
    for them. Verified against a real UR5e mesh (`base.stl`): raw extents
    0.151 x 0.151 x 0.099, matching the UR5e's published ~150 mm base
    flange diameter -- i.e. the raw file coordinates are in METERS,
    matching the ROS/URDF convention parse_urdf already assumes for
    `<origin xyz>` (transforms.py's M_TO_MM). Applied by scaling the
    merged mesh directly when `.units` is None.
  - COLLADA (.dae) declares its own unit via `<unit meter="...">`, and
    trimesh surfaces it as a `"{factor} * meters"` string on `.units`
    (e.g. `"0.001 * meters"` for a file authored in millimeters).
    Verified against a real mycobot mesh (`base.dae`, unit 0.001 ->
    raw coordinates already in mm) and a trimesh-exported synthetic one
    (unit 1.0 -> raw coordinates in meters, needing the same x1000 as
    STL). Handled by calling trimesh's own `.convert_units("millimeters")`
    on the loaded Scene/Trimesh -- NOT by re-deriving the factor by hand
    -- because the unit-conversion resource table lives in trimesh, and
    reimplementing it would be a second, independently-wrong copy.
    Must run BEFORE merging a Scene's multiple geometries into one
    Trimesh (`force='mesh'` or `Scene.to_geometry()` after the fact both
    silently discard the `.units` metadata a Scene carries -- verified;
    converting first and merging second is required, not stylistic).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .model import KinematicModel
from .transforms import M_TO_MM


# Fixed load_error text for the "resolved but never attempted" sub-state
# (inspect_meshes(..., load=False)) -- distinct from any real trimesh/
# pycollada failure message, but deliberately still a non-None
# load_error, so MeshReport.loaded is False for it exactly like a real
# load failure: both leave triangles/bbox_mm unavailable, for a consumer
# that only cares whether it can read those fields, not why it can't.
NOT_LOADED = "not loaded (inspect_meshes called with load=False)"


@dataclass
class MeshReport:
    link: str
    path: str                          # exactly as written in the URDF
    kind: str                          # "visual" | "collision"
    resolved: bool
    resolved_path: str | None = None   # absolute path on disk, when resolved
    # None when resolved=False (not applicable) or state 3 (loaded clean);
    # a string otherwise -- a real trimesh/pycollada failure, OR the fixed
    # NOT_LOADED text when load=False skipped loading entirely. Use
    # `loaded`, not this field directly, to test for state 3.
    load_error: str | None = None
    triangles: int | None = None
    # (min_xyz, max_xyz), each an (x, y, z) tuple, mesh-local frame, mm.
    bbox_mm: tuple[tuple[float, float, float], tuple[float, float, float]] | None = None
    # Translation of the <visual>/<collision> element's own <origin> (mm);
    # (0, 0, 0) when that element declared none, per _origin()'s identity
    # default -- NOT derived from the mesh geometry itself.
    origin_offset_mm: tuple[float, float, float] | None = None
    bytes: int | None = None           # file size on disk; independent of trimesh

    @property
    def loaded(self) -> bool:
        """True only when the file was found AND actually turned into a
        loaded mesh (state 3) -- `triangles`/`bbox_mm` are non-None
        exactly when this is True. False for an unresolved path, a real
        load failure, and a resolved-but-not-attempted report
        (load=False) alike: all three leave triangles/bbox_mm at None,
        and this is the safe way to guard access to them without having
        to separately check `resolved` and `load_error` (which, taken
        alone, do not imply `triangles is not None`) or risk a TypeError
        comparing None against a threshold.
        """
        return self.resolved and self.load_error is None


def _resolve_package_uri(urdf_dir: Path, uri: str) -> Path | None:
    """`package://<pkg_name>/<sub_path>` -> an existing file, or None.

    Walks up from urdf_dir; at each ancestor, checks whether
    `ancestor / pkg_name / sub_path` exists. This mimics ROS package
    resolution (find a directory named pkg_name somewhere above the URDF,
    then resolve the rest of the URI under it) without needing a real ROS
    environment or package index -- good enough for real package layouts
    (verified against mycobot_ros2) and correctly gives up on ones that
    don't actually have a directory by that name anywhere above the URDF
    (verified against a UR vendor URDF's `package://urdfs/...`, which
    doesn't).
    """
    rest = uri[len("package://"):]
    pkg_name, _, sub_path = rest.partition("/")
    if not pkg_name or not sub_path:
        return None
    for ancestor in (urdf_dir, *urdf_dir.parents):
        candidate = ancestor / pkg_name / sub_path
        if candidate.is_file():
            return candidate
    return None


def _resolve_mesh_path(urdf_dir: Path, mesh_path: str) -> Path | None:
    if mesh_path.startswith("package://"):
        return _resolve_package_uri(urdf_dir, mesh_path)
    candidate = urdf_dir / mesh_path
    return candidate if candidate.is_file() else None


def _load_geometry_mm(path: Path):
    """Load `path` via trimesh, normalized to a single Trimesh in mm.

    Returns the merged Trimesh with >0 triangles. Raises ValueError (with
    `path` named) for anything that goes wrong -- a raised trimesh/
    pycollada exception, or a "successfully" loaded but empty (zero-
    triangle) result -- so the caller can turn either into a MeshReport's
    load_error rather than letting one bad mesh file abort the whole
    report.
    """
    import trimesh  # lazy: callers that only parse pay nothing for this

    try:
        loaded = trimesh.load(path)
    except Exception as e:
        raise ValueError(f"{path}: failed to load mesh ({type(e).__name__}: {e})") from e

    # .units is None for STL/OBJ/PLY (no unit metadata in those formats);
    # a "{factor} * meters"-style string for COLLADA, which always
    # declares one. Must convert BEFORE merging a Scene down to one
    # Trimesh -- both force='mesh' and Scene.to_geometry() silently drop
    # this metadata, so a merge-first order would leave nothing to
    # convert.
    units = getattr(loaded, "units", None)
    if units is not None:
        try:
            loaded = loaded.convert_units("millimeters")
        except Exception as e:
            raise ValueError(
                f"{path}: failed to convert mesh units {units!r} to millimeters "
                f"({type(e).__name__}: {e})"
            ) from e

    try:
        mesh = loaded.to_geometry() if isinstance(loaded, trimesh.Scene) else loaded
    except Exception as e:
        raise ValueError(f"{path}: failed to merge mesh geometry ({type(e).__name__}: {e})") from e

    if units is None:
        # No unit metadata: ROS/URDF meshes are conventionally authored in
        # meters (see module docstring -- measured against a real UR5e
        # mesh, not guessed), matching parse_urdf's own M_TO_MM handling
        # of <origin xyz>.
        mesh.apply_scale(M_TO_MM)

    if len(mesh.faces) == 0:
        # trimesh does not raise for this -- e.g. plain text given an
        # .stl extension parses "successfully" as an empty scene. Treated
        # as a load failure, not a legitimately empty mesh.
        raise ValueError(f"{path}: mesh loaded with zero triangles -- not a valid mesh file")

    return mesh


_CacheEntry = tuple[int, tuple, str | None]  # (triangles, bbox_mm, load_error)


def _load_geometry_cached(cache: dict[Path, _CacheEntry], resolved_path: Path) -> _CacheEntry:
    """`_load_geometry_mm`, memoized per resolved path within one
    `inspect_meshes` call -- see the module docstring's Caching section
    for why (the same file is commonly referenced by both <visual> and
    <collision>, or reused across links, and trimesh loading is the
    expensive part of this module by ~3 orders of magnitude).
    """
    if resolved_path in cache:
        return cache[resolved_path]
    try:
        mesh = _load_geometry_mm(resolved_path)
        lo, hi = mesh.bounds
        entry: _CacheEntry = (
            len(mesh.faces),
            (tuple(float(v) for v in lo), tuple(float(v) for v in hi)),
            None,
        )
    except ValueError as e:
        entry = (None, None, str(e))
    cache[resolved_path] = entry
    return entry


def inspect_meshes(model: KinematicModel, *, load: bool = True) -> list[MeshReport]:
    """Every mesh reference in `model`, resolved and (by default) loaded.

    Reads only Link.visual_meshes/collision_meshes/visual_mesh_origins/
    collision_mesh_origins (populated by urdf.py at parse time) -- this
    function never touches the URDF's XML itself. Order is: for each link
    (dict iteration order, i.e. declaration order in the file), visual
    references before collision, each in file order -- deterministic, not
    load-bearing.

    `load=False` skips trimesh entirely (see the module docstring's `load`
    section for the measured cost gap -- three orders of magnitude on
    this machine, more with a cold disk cache): every reference is still
    resolved against disk, but `triangles`/`bbox_mm` stay None and
    `load_error` is set to the fixed NOT_LOADED text. Use this for
    anything that only needs `unresolved-mesh` (e.g. `validate`); loading
    is for a consumer that actually needs geometry (e.g. `heavy-mesh`).
    """
    urdf_dir = Path(model.source_path).resolve().parent
    reports: list[MeshReport] = []
    cache: dict[Path, _CacheEntry] = {}

    for link in model.links.values():
        for kind, paths, origins in (
            ("visual", link.visual_meshes, link.visual_mesh_origins),
            ("collision", link.collision_meshes, link.collision_mesh_origins),
        ):
            # paths/origins are populated together, in lockstep, by
            # urdf.py's _mesh_refs -- this assert is cheap insurance
            # against a future desync silently truncating one of them via
            # zip() below, rather than a case expected to fire today.
            assert len(paths) == len(origins), (
                f"link {link.name!r}: {kind} mesh paths ({len(paths)}) and "
                f"origins ({len(origins)}) count mismatch"
            )
            for mesh_path, origin in zip(paths, origins):
                resolved_path = _resolve_mesh_path(urdf_dir, mesh_path)
                if resolved_path is None:
                    reports.append(MeshReport(link=link.name, path=mesh_path, kind=kind, resolved=False))
                    continue

                offset = (float(origin[0, 3]), float(origin[1, 3]), float(origin[2, 3]))
                size_bytes = resolved_path.stat().st_size

                if not load:
                    reports.append(MeshReport(
                        link=link.name, path=mesh_path, kind=kind, resolved=True,
                        resolved_path=str(resolved_path), load_error=NOT_LOADED,
                        origin_offset_mm=offset, bytes=size_bytes,
                    ))
                    continue

                triangles, bbox_mm, load_error = _load_geometry_cached(cache, resolved_path)
                reports.append(MeshReport(
                    link=link.name, path=mesh_path, kind=kind, resolved=True,
                    resolved_path=str(resolved_path),
                    triangles=triangles, bbox_mm=bbox_mm, load_error=load_error,
                    origin_offset_mm=offset, bytes=size_bytes,
                ))

    return reports
