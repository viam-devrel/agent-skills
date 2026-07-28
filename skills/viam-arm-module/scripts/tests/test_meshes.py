"""Tests for _armkit/meshes.py: inspect_meshes.

Covers path resolution (package:// and relative), the meters -> mm
conversion (STL/OBJ/PLY have no unit metadata so meters is assumed, per
ROS/URDF convention; COLLADA's own declared unit is honored instead), the
three-way resolved / unresolved / exists-but-unloadable state, and
origin_offset_mm (the <visual>/<collision> element's own <origin>, not
anything derived from the mesh geometry) -- see meshes.py's module
docstring for the overall design and why each of these is the way it is.
"""
from __future__ import annotations

from pathlib import Path

from _armkit.meshes import NOT_LOADED, inspect_meshes
from _armkit.urdf import parse_urdf


def _write_urdf(workspace: Path, body: str) -> Path:
    urdf_path = workspace / "urdf" / "robot.urdf"
    urdf_path.write_text(f'<robot name="mesh_test">{body}</robot>')
    return urdf_path


def _bbox_size(report) -> tuple[float, float, float]:
    lo, hi = report.bbox_mm
    return tuple(round(h - l, 2) for l, h in zip(lo, hi))


def test_ur20_yields_empty_report(fixtures):
    # ur20.urdf uses only <box> collision primitives -- no <mesh>
    # references at all, so inspect_meshes has nothing to report.
    model = parse_urdf(fixtures / "ur20.urdf")
    assert inspect_meshes(model) == []


def test_meshed_urdf_yields_three_unresolved(fixtures):
    # meshed.urdf's mesh references are deliberately unresolvable (none of
    # the files exist on disk) -- the whole point of that fixture. base has
    # one visual mesh; link1 has one visual and one collision mesh (the
    # visual/collision pairing case).
    model = parse_urdf(fixtures / "meshed.urdf")
    reports = inspect_meshes(model)
    assert len(reports) == 3
    assert all(not r.resolved for r in reports)
    assert all(r.triangles is None and r.bbox_mm is None and r.bytes is None for r in reports)

    base_visual = [r for r in reports if r.link == "base"]
    assert len(base_visual) == 1
    assert base_visual[0].kind == "visual"
    assert base_visual[0].path == "package://some_pkg/meshes/base.dae"

    link1 = {r.kind: r for r in reports if r.link == "link1"}
    assert link1["visual"].path == "meshes/link1.dae"
    assert link1["collision"].path == "meshes/link1.stl"


def test_resolves_package_uri_and_reports_geometry(mesh_workspace):
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual><geometry><mesh filename="package://some_pkg/meshes/cube.stl"/></geometry></visual>
      </link>
    """)
    model = parse_urdf(urdf_path)
    [report] = inspect_meshes(model)
    assert report.link == "base"
    assert report.kind == "visual"
    assert report.resolved
    assert report.load_error is None
    assert report.triangles == 12  # a box triangulates to 12 triangles
    assert report.bytes and report.bytes > 0
    assert _bbox_size(report) == (10.0, 20.0, 30.0)
    assert report.origin_offset_mm == (0.0, 0.0, 0.0)  # no <origin> declared


def test_resolves_relative_path(mesh_workspace):
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="gripper">
        <collision><geometry><mesh filename="meshes/gripper.stl"/></geometry></collision>
      </link>
    """)
    model = parse_urdf(urdf_path)
    [report] = inspect_meshes(model)
    assert report.kind == "collision"
    assert report.resolved
    assert _bbox_size(report) == (5.0, 5.0, 5.0)


def test_resolves_collada_mesh_honoring_its_own_unit_tag(mesh_workspace):
    # part.dae carries its own <unit> declaration (COLLADA does this,
    # unlike STL/OBJ/PLY) -- inspect_meshes must honor that instead of
    # assuming meters the way it does for unit-less formats.
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual><geometry><mesh filename="meshes/part.dae"/></geometry></visual>
      </link>
    """)
    model = parse_urdf(urdf_path)
    [report] = inspect_meshes(model)
    assert report.resolved
    assert report.load_error is None
    assert report.triangles == 12
    assert _bbox_size(report) == (10.0, 20.0, 30.0)


def test_origin_offset_mm_reflects_declared_visual_origin(mesh_workspace):
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual>
          <origin xyz="0.01 0.02 0.03" rpy="0 0 0"/>
          <geometry><mesh filename="package://some_pkg/meshes/cube.stl"/></geometry>
        </visual>
      </link>
    """)
    model = parse_urdf(urdf_path)
    [report] = inspect_meshes(model)
    assert report.origin_offset_mm == (10.0, 20.0, 30.0)


def test_link_with_both_visual_and_collision_mesh_pairs_correctly(mesh_workspace):
    # A pairing bug (e.g. reading collision's path against visual's
    # origin) would misassign kind, path, or origin_offset_mm here.
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual>
          <origin xyz="0.001 0 0" rpy="0 0 0"/>
          <geometry><mesh filename="package://some_pkg/meshes/cube.stl"/></geometry>
        </visual>
        <collision>
          <origin xyz="0.002 0 0" rpy="0 0 0"/>
          <geometry><mesh filename="meshes/gripper.stl"/></geometry>
        </collision>
      </link>
    """)
    model = parse_urdf(urdf_path)
    reports = {r.kind: r for r in inspect_meshes(model)}
    assert len(reports) == 2
    assert reports["visual"].path == "package://some_pkg/meshes/cube.stl"
    assert reports["visual"].origin_offset_mm == (1.0, 0.0, 0.0)
    assert _bbox_size(reports["visual"]) == (10.0, 20.0, 30.0)
    assert reports["collision"].path == "meshes/gripper.stl"
    assert reports["collision"].origin_offset_mm == (2.0, 0.0, 0.0)
    assert _bbox_size(reports["collision"]) == (5.0, 5.0, 5.0)


def test_unresolvable_package_uri_reports_unresolved_not_raise(mesh_workspace):
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual><geometry><mesh filename="package://nonexistent_pkg/meshes/cube.stl"/></geometry></visual>
      </link>
    """)
    model = parse_urdf(urdf_path)
    [report] = inspect_meshes(model)
    assert report.resolved is False
    assert report.load_error is None
    assert report.path == "package://nonexistent_pkg/meshes/cube.stl"


def test_unresolvable_relative_path_reports_unresolved_not_raise(mesh_workspace):
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="base">
        <collision><geometry><mesh filename="meshes/does_not_exist.stl"/></geometry></collision>
      </link>
    """)
    model = parse_urdf(urdf_path)
    [report] = inspect_meshes(model)
    assert report.resolved is False


def test_exists_but_empty_reports_load_error_not_raise(mesh_workspace):
    # garbage.stl is plain text, not a mesh -- trimesh parses it as an
    # empty (zero-triangle) scene rather than raising, so this exercises
    # the explicit zero-triangle check in meshes.py, not just exception
    # handling.
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual><geometry><mesh filename="meshes/garbage.stl"/></geometry></visual>
      </link>
    """)
    model = parse_urdf(urdf_path)
    [report] = inspect_meshes(model)
    assert report.resolved is True
    assert report.load_error is not None
    assert report.triangles is None
    assert report.bbox_mm is None
    assert report.bytes is not None and report.bytes > 0  # file size, no trimesh needed


def test_exists_but_malformed_reports_load_error_not_raise(mesh_workspace):
    # garbage.dae is invalid XML -- pycollada raises loading it, unlike
    # garbage.stl above; this exercises the caught-exception path.
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual><geometry><mesh filename="meshes/garbage.dae"/></geometry></visual>
      </link>
    """)
    model = parse_urdf(urdf_path)
    [report] = inspect_meshes(model)
    assert report.resolved is True
    assert report.load_error is not None
    assert report.triangles is None
    assert report.bbox_mm is None
    assert report.loaded is False


# ---------------------------------------------------------------------------
# load=False: resolution-only, no trimesh -- and the `loaded` predicate.
# ---------------------------------------------------------------------------

def test_load_false_still_resolves_but_skips_loading(mesh_workspace):
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual>
          <origin xyz="0.01 0.02 0.03" rpy="0 0 0"/>
          <geometry><mesh filename="package://some_pkg/meshes/cube.stl"/></geometry>
        </visual>
      </link>
    """)
    model = parse_urdf(urdf_path)
    [report] = inspect_meshes(model, load=False)
    assert report.resolved is True
    assert report.resolved_path is not None
    assert report.bytes and report.bytes > 0          # cheap: a stat, no trimesh
    assert report.origin_offset_mm == (10.0, 20.0, 30.0)  # cheap: from the URDF, not the mesh
    assert report.triangles is None
    assert report.bbox_mm is None
    assert report.load_error == NOT_LOADED
    assert report.loaded is False


def test_load_false_still_reports_unresolved_paths(mesh_workspace):
    # load=False changes nothing about resolution itself.
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual><geometry><mesh filename="package://nonexistent_pkg/meshes/cube.stl"/></geometry></visual>
      </link>
    """)
    model = parse_urdf(urdf_path)
    [report] = inspect_meshes(model, load=False)
    assert report.resolved is False
    assert report.load_error is None  # NOT_LOADED only applies once resolved


def test_loaded_property_across_all_states(fixtures, mesh_workspace):
    # unresolved (state 1)
    unresolved_reports = inspect_meshes(parse_urdf(fixtures / "meshed.urdf"))
    assert all(not r.resolved and r.loaded is False for r in unresolved_reports)

    # resolved but broken (state 2a)
    broken_urdf = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual><geometry><mesh filename="meshes/garbage.stl"/></geometry></visual>
      </link>
    """)
    [broken] = inspect_meshes(parse_urdf(broken_urdf))
    assert broken.resolved and broken.load_error and broken.loaded is False

    # resolved but not attempted (state 2b, load=False)
    ok_urdf = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual><geometry><mesh filename="package://some_pkg/meshes/cube.stl"/></geometry></visual>
      </link>
    """)
    [not_attempted] = inspect_meshes(parse_urdf(ok_urdf), load=False)
    assert not_attempted.resolved and not_attempted.load_error and not_attempted.loaded is False

    # fully loaded (state 3)
    [loaded] = inspect_meshes(parse_urdf(ok_urdf))
    assert loaded.resolved and loaded.load_error is None and loaded.loaded is True
    assert loaded.triangles is not None and loaded.bbox_mm is not None


# ---------------------------------------------------------------------------
# Caching: a mesh file referenced twice (visual + collision) is loaded once.
# ---------------------------------------------------------------------------

def test_shared_mesh_file_is_loaded_once_and_values_match(mesh_workspace, monkeypatch):
    urdf_path = _write_urdf(mesh_workspace, """
      <link name="base">
        <visual><geometry><mesh filename="package://some_pkg/meshes/cube.stl"/></geometry></visual>
        <collision><geometry><mesh filename="package://some_pkg/meshes/cube.stl"/></geometry></collision>
      </link>
    """)
    model = parse_urdf(urdf_path)

    import trimesh

    calls: list = []
    real_load = trimesh.load

    def counting_load(*args, **kwargs):
        calls.append(args[0] if args else kwargs.get("file_obj"))
        return real_load(*args, **kwargs)

    monkeypatch.setattr(trimesh, "load", counting_load)

    reports = inspect_meshes(model)
    assert len(reports) == 2
    assert len(calls) == 1, "the second reference should reuse the cached load, not call trimesh again"

    visual = next(r for r in reports if r.kind == "visual")
    collision = next(r for r in reports if r.kind == "collision")
    assert visual.loaded and collision.loaded
    # Caching must not change what's reported, only how many times it's computed.
    assert visual.triangles == collision.triangles == 12
    assert visual.bbox_mm == collision.bbox_mm
