from pathlib import Path

import pytest

@pytest.fixture
def fixtures() -> Path:
    return Path(__file__).parent / "fixtures"


@pytest.fixture
def mesh_workspace(tmp_path: Path) -> Path:
    """A small on-disk package layout with REAL, loadable mesh files --
    generated with trimesh rather than committed as binary fixture assets
    (A6's scope explicitly calls for this) -- for inspect_meshes tests that
    need meshes actually resolvable and loadable on disk, unlike
    meshed.urdf's fixtures, which are deliberately unresolvable.

    Layout (all paths relative to the returned tmp_path):
        some_pkg/meshes/cube.stl   -- 10x20x30 mm box; a `package://
                                       some_pkg/...` target
        urdf/meshes/gripper.stl    -- 5x5x5 mm box; a relative-path target
        urdf/meshes/part.dae       -- the same 10x20x30 mm box, exported as
                                       COLLADA (declares its own <unit>,
                                       unlike STL -- see meshes.py)
        urdf/meshes/garbage.stl    -- ASCII text, not a mesh at all: trimesh
                                       parses this as an EMPTY scene rather
                                       than raising, so it exercises the
                                       zero-triangle load-failure path
        urdf/meshes/garbage.dae    -- invalid XML: exercises the exception-
                                       raising load-failure path (pycollada
                                       does raise on this one)

    Each test writes its own urdf/robot.urdf referencing whichever of
    these it needs -- the interesting variation per test is the URDF
    (origin, pairing, which mesh), not the mesh files themselves.

    Box dimensions are authored in METERS -- trimesh's raw, unitless STL
    coordinates are conventionally meters for ROS/URDF meshes (verified
    against a real UR5e mesh: base.stl's raw extents are 0.151 x 0.151 x
    0.099, matching the UR5e's published ~150 mm base flange diameter) --
    so the millimeter values named above are what inspect_meshes should
    report after its meters -> mm conversion.
    """
    import trimesh

    pkg_dir = tmp_path / "some_pkg" / "meshes"
    pkg_dir.mkdir(parents=True)
    trimesh.creation.box(extents=[0.010, 0.020, 0.030]).export(pkg_dir / "cube.stl")

    urdf_meshes = tmp_path / "urdf" / "meshes"
    urdf_meshes.mkdir(parents=True)
    trimesh.creation.box(extents=[0.005, 0.005, 0.005]).export(urdf_meshes / "gripper.stl")
    trimesh.creation.box(extents=[0.010, 0.020, 0.030]).export(urdf_meshes / "part.dae")
    (urdf_meshes / "garbage.stl").write_text("not a mesh file at all, just text")
    (urdf_meshes / "garbage.dae").write_text("this is not valid collada xml <<<>>>")

    return tmp_path
