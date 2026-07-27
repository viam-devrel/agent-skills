import numpy as np
import pytest
from _armkit.urdf import parse_urdf


def _write(tmp_path, xml):
    p = tmp_path / "test.urdf"
    p.write_text(xml)
    return p


def test_parses_two_link(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    assert m.name == "two_link"
    assert m.dof == 2
    assert [j.name for j in m.chain()] == ["j1", "j2"]
    assert m.base_link == "base"
    assert m.tip_link == "tip"


def test_converts_meters_to_millimeters(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    # URDF says xyz="1 0 0"; internal representation is mm.
    assert np.isclose(m.chain()[0].origin[0, 3], 1000.0)


def test_limits_stay_in_radians(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    assert np.isclose(m.chain()[0].lower, -3.14159)


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
