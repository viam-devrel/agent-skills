import numpy as np
import pytest
from _armkit.transforms import rpy_to_matrix
from _armkit.urdf import parse_urdf


def _write(tmp_path, xml, name="test.urdf"):
    p = tmp_path / name
    p.write_text(xml)
    return p


def test_parses_two_link(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    assert m.name == "two_link"
    assert m.dof == 2
    assert [j.name for j in m.chain()] == ["j1", "j2"]
    assert m.base_link == "base"
    assert m.tip_link == "tip"


def test_source_format_and_path_are_recorded(fixtures):
    # A later task routes on source_format; nothing previously pinned it
    # or source_path.
    m = parse_urdf(fixtures / "two_link.urdf")
    assert m.source_format == "urdf"
    assert m.source_path == str(fixtures / "two_link.urdf")


def test_links_are_all_present(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    assert set(m.links) == {"base", "link1", "tip"}


def test_converts_meters_to_millimeters(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    # URDF says xyz="1 0 0"; internal representation is mm.
    assert np.isclose(m.chain()[0].origin[0, 3], 1000.0)


def test_origin_scales_translation_and_preserves_rotation(tmp_path):
    # A joint with BOTH non-zero xyz and non-zero rpy pins three things
    # at once: translation is scaled by 1000, the rotation block is
    # exactly rpy_to_matrix(...) (unscaled), and the bottom row stays
    # [0, 0, 0, 1]. two_link.urdf alone can't cover this: its rpy is
    # always "0 0 0", so a bug that only corrupted the rotation block
    # would slip past a translation-only assertion.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    m = parse_urdf(path)
    origin = m.chain()[0].origin
    assert np.allclose(origin[:3, 3], [1000.0, 2000.0, 3000.0])
    assert np.allclose(origin[:3, :3], rpy_to_matrix(0.1, 0.2, 0.3))
    assert np.allclose(origin[3, :], [0, 0, 0, 1])


def test_limits_stay_in_radians(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    j = m.chain()[0]
    assert np.isclose(j.lower, -3.14159)
    assert np.isclose(j.upper, 3.14159)


def test_parses_real_ur20(fixtures):
    m = parse_urdf(fixtures / "ur20.urdf")
    assert m.dof == 6


def test_continuous_joint_gets_infinite_limits(tmp_path):
    # Covers: `jtype == "continuous"` -> lower/upper forced to +/-inf,
    # even when a <limit> element is present (RDK converts continuous
    # joints to revolute-with-infinite-limits, so this must hold).
    path = _write(tmp_path, """
    <robot name="cont">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="continuous">
        <parent link="base"/><child link="tip"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    m = parse_urdf(path)
    j = m.chain()[0]
    assert j.lower == -np.inf
    assert j.upper == np.inf


def test_prismatic_limits_scaled_to_millimeters(tmp_path):
    # Covers: `if jtype == "prismatic"` -> limits scaled by M_TO_MM,
    # unlike revolute/continuous limits which stay in radians.
    path = _write(tmp_path, """
    <robot name="prism">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="prismatic">
        <parent link="base"/><child link="tip"/>
        <axis xyz="0 0 1"/>
        <limit lower="-0.5" upper="0.5" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    m = parse_urdf(path)
    j = m.chain()[0]
    assert np.isclose(j.lower, -500.0)
    assert np.isclose(j.upper, 500.0)


def test_actuated_joint_without_limit_element_has_none_limits(tmp_path):
    # Covers: actuated joint with no <limit> child at all -- lower/upper
    # stay None. This is NOT asserted to be an error here; a later task
    # reports missing limits as a "missing-limits" finding. This test
    # pins current behavior so that task can rely on it.
    path = _write(tmp_path, """
    <robot name="nolimit">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <axis xyz="0 0 1"/>
      </joint>
    </robot>
    """)
    m = parse_urdf(path)
    j = m.chain()[0]
    assert j.lower is None and j.upper is None


def test_zero_length_axis_raises(tmp_path):
    # Covers: `if norm == 0` -> ValueError on a degenerate axis vector.
    path = _write(tmp_path, """
    <robot name="badaxis">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <axis xyz="0 0 0"/>
        <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="zero-length axis"):
        parse_urdf(path)


def test_four_component_axis_raises(tmp_path):
    # CRITICAL regression covered: `xyz="0 0 1 5"` used to normalize in
    # its own (4-component) dimension -- axis=[0, 0, 0.1961, 0.9806] --
    # which parses cleanly, reports a correct dof count, and would PASS
    # validation while silently handing forward kinematics a non-3-vector
    # axis (confirmed before this fix: np.linalg.norm of that result is
    # 1.0, so the old zero-length check never caught it). Wrong-length
    # input must be rejected before normalization, not after.
    path = _write(tmp_path, """
    <robot name="badaxislen">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <axis xyz="0 0 1 5"/>
        <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="4-component axis"):
        parse_urdf(path)


def test_two_component_axis_raises(tmp_path):
    # Covers the other side of the same length check: too FEW
    # components ("0 1") previously produced a 2-vector axis that would
    # only surface as an IndexError far downstream, from FK code that
    # assumes axis[2] exists.
    path = _write(tmp_path, """
    <robot name="shortaxis">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <axis xyz="0 1"/>
        <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="2-component axis"):
        parse_urdf(path)


def test_missing_parent_raises(tmp_path):
    # Covers: joint with no <parent> element -- previously an
    # AttributeError from `None.get("link")`.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="fixed">
        <child link="tip"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="parent"):
        parse_urdf(path)


def test_missing_child_raises(tmp_path):
    # Covers: joint with no <child> element -- same failure mode as
    # missing <parent>, mirrored.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="fixed">
        <parent link="base"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="child"):
        parse_urdf(path)


def test_parent_with_no_link_attribute_raises(tmp_path):
    # Covers: <parent/> present but without a link="..." attribute --
    # previously produced a joint whose parent is None.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="fixed">
        <parent/><child link="tip"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="parent"):
        parse_urdf(path)


def test_rpy_with_wrong_component_count_raises(tmp_path):
    # Covers: `rpy="0 0"` -- previously a TypeError from
    # rpy_to_matrix() missing its third positional argument.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <origin xyz="0 0 0" rpy="0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="rpy"):
        parse_urdf(path)


def test_unsupported_joint_type_raises(tmp_path):
    # Covers: `type="floating"` -- URDF-legal but unhandled by this
    # toolkit. Previously parsed to dof=0 and would report PASS on a
    # model armkit cannot actually validate.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="floating">
        <parent link="base"/><child link="tip"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="unsupported type"):
        parse_urdf(path)


def test_missing_joint_type_raises(tmp_path):
    # Covers: joint with no type="..." attribute at all -- same silent
    # dof=0 failure mode as an unsupported type.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="tip"/>
      <joint name="j1">
        <parent link="base"/><child link="tip"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="unsupported type"):
        parse_urdf(path)


def test_joint_missing_name_raises(tmp_path):
    # Covers: <joint> with no name="..." attribute.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="tip"/>
      <joint type="fixed">
        <parent link="base"/><child link="tip"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="missing a 'name'"):
        parse_urdf(path)


def test_link_missing_name_raises(tmp_path):
    # Covers: <link> with no name="..." attribute -- previously became
    # a link keyed under None.
    path = _write(tmp_path, """
    <robot name="t">
      <link/>
      <link name="tip"/>
      <joint name="j1" type="fixed">
        <parent link="base"/><child link="tip"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="<link> element missing"):
        parse_urdf(path)


def test_root_element_not_robot_raises(tmp_path):
    # Covers: well-formed XML whose root tag isn't <robot> -- previously
    # parsed "successfully" to zero joints and then failed later with
    # the unrelated-looking "no root link (cycle?)".
    path = _write(tmp_path, """<not_a_robot name="t"><link name="base"/></not_a_robot>""")
    with pytest.raises(ValueError, match="not <robot>"):
        parse_urdf(path)


def test_malformed_xml_raises_valueerror(tmp_path):
    # Covers: unparsable XML -- previously xml.etree.ElementTree.ParseError,
    # whose MRO is SyntaxError, NOT ValueError.
    path = _write(tmp_path, "<robot name='t'><link name='base'></robot>")
    with pytest.raises(ValueError, match="malformed XML"):
        parse_urdf(path)


def test_empty_file_raises_valueerror(tmp_path):
    # Covers: an empty file -- also an ET.ParseError ("no element
    # found"), same MRO problem as malformed XML.
    path = _write(tmp_path, "")
    with pytest.raises(ValueError, match="malformed XML"):
        parse_urdf(path)


def test_nonexistent_path_raises_valueerror(tmp_path):
    # Covers: a path that doesn't exist -- previously FileNotFoundError.
    with pytest.raises(ValueError, match="not found"):
        parse_urdf(tmp_path / "does_not_exist.urdf")


def test_directory_path_raises_valueerror(tmp_path):
    # Covers: a directory passed instead of a file -- previously
    # IsADirectoryError.
    with pytest.raises(ValueError, match="directory"):
        parse_urdf(tmp_path)
