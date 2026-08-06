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


def test_non_numeric_axis_component_raises(tmp_path):
    # Covers: `<axis xyz="a b c"/>` -- previously a bare
    # ValueError("could not convert string to float: 'a'") with no file
    # path or joint name attached, re-raised untouched by parse_urdf's
    # `except ValueError: raise`. Now wrapped with context at the point
    # of failure, same pattern as _origin's non-numeric-component guard.
    path = _write(tmp_path, """
    <robot name="badaxisval">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <axis xyz="a b c"/>
        <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="non-numeric axis component") as exc_info:
        parse_urdf(path)
    assert str(path) in str(exc_info.value)
    assert "j1" in str(exc_info.value)


def test_non_numeric_limit_lower_raises(tmp_path):
    # Covers: `<limit lower="abc" .../>` -- same untouched-message
    # failure mode as the axis case, but for a limit attribute instead
    # of an axis component; the message must say "lower", not "axis",
    # so the two failure modes don't read identically.
    path = _write(tmp_path, """
    <robot name="badlower">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <axis xyz="0 0 1"/>
        <limit lower="abc" upper="1.0" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="non-numeric limit 'lower'") as exc_info:
        parse_urdf(path)
    assert str(path) in str(exc_info.value)
    assert "j1" in str(exc_info.value)


def test_non_numeric_limit_upper_raises(tmp_path):
    # Covers: `<limit upper="xyz" .../>` -- mirrors the lower case;
    # message must say "upper", distinguishing it from both the axis
    # and the lower-limit failure modes.
    path = _write(tmp_path, """
    <robot name="badupper">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.0" upper="xyz" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="non-numeric limit 'upper'") as exc_info:
        parse_urdf(path)
    assert str(path) in str(exc_info.value)
    assert "j1" in str(exc_info.value)


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


def test_joint_referencing_undeclared_parent_link_raises(tmp_path):
    # Covers the referential-integrity check added after Fix 1: a joint
    # whose parent names a link that was never declared with <link
    # name="..."/>. This is the exact shape of the real-world corpus
    # hit (mycobot_with_vision_copy.urdf, an incomplete xacro fragment
    # whose joint's parent, "joint1", is a link defined only in an
    # <xacro:include>d file this parser never sees) -- previously this
    # parsed cleanly and would report a misleadingly clean bill of
    # health.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="tip"/>
      <joint name="j1" type="fixed">
        <parent link="ghost_link"/><child link="tip"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="parent link 'ghost_link', which is not declared"):
        parse_urdf(path)


def test_joint_referencing_undeclared_child_link_raises(tmp_path):
    # Mirrors the parent case for <child>.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/>
      <joint name="j1" type="fixed">
        <parent link="base"/><child link="ghost_link"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="child link 'ghost_link', which is not declared"):
        parse_urdf(path)


def test_mimic_joint_excluded_from_dof(fixtures):
    # Measured against RDK v1.0.0 via the Go probe (referenceframe.
    # KinematicModelFromFile): this file gives DoF=2, because joint3
    # mimics joint1 and RDK derives its value at runtime rather than
    # giving it its own input slot. armkit previously reported DoF=3,
    # a live correctness bug -- mimic joints are still real joints in
    # the tree (they still contribute a transform) but must not consume
    # an input slot.
    m = parse_urdf(fixtures / "test_mimic_serial.urdf")
    assert m.dof == 2
    assert [j.name for j in m.actuated_joints] == ["joint1", "joint2"]
    # The mimic joint is still present in the full joint list/tree...
    assert [j.name for j in m.chain()] == ["base_joint", "joint1", "joint2", "joint3"]
    joint3 = next(j for j in m.chain() if j.name == "joint3")
    assert joint3.mimic is not None
    assert joint3.mimic.source == "joint1"
    assert joint3.mimic.multiplier == -1.0
    assert joint3.mimic.offset == 0.0
    # ...but RDK zeroes a mimic joint's own limits (model_urdf.go:191)
    # since the source joint's limits govern; armkit matches that.
    assert joint3.lower == 0.0
    assert joint3.upper == 0.0


def test_mimic_defaults_multiplier_and_offset(tmp_path):
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="mid"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="mid"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="j2" type="revolute">
        <parent link="mid"/><child link="tip"/>
        <axis xyz="0 0 1"/>
        <mimic joint="j1"/>
      </joint>
    </robot>
    """)
    m = parse_urdf(path)
    j2 = next(j for j in m.chain() if j.name == "j2")
    assert j2.mimic.multiplier == 1.0
    assert j2.mimic.offset == 0.0


def test_mimic_referencing_undeclared_joint_raises(tmp_path):
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <axis xyz="0 0 1"/>
        <mimic joint="ghost_joint"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="mimics 'ghost_joint', which is not declared"):
        parse_urdf(path)


def test_mimic_missing_joint_attribute_raises(tmp_path):
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <axis xyz="0 0 1"/>
        <mimic multiplier="1.0"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="<mimic> element missing a 'joint' attribute"):
        parse_urdf(path)


def test_mimic_non_numeric_multiplier_raises(tmp_path):
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="mid"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="mid"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="j2" type="revolute">
        <parent link="mid"/><child link="tip"/>
        <axis xyz="0 0 1"/>
        <mimic joint="j1" multiplier="abc"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="non-numeric mimic 'multiplier'"):
        parse_urdf(path)


def test_mimic_of_mimic_composes_transitively(fixtures):
    # RDK ACCEPTS a mimic-of-a-mimic (DoF=1), composing the multiplier/
    # offset transitively rather than rejecting the chain -- verified
    # against RDK v1.0.0 via the probe. j2 mimics q1 (multiplier=2.0,
    # offset=0.1); j3 mimics j2 (multiplier=3.0, offset=0.2). Composing
    # gives j3 = 6.0*q1 + 0.5 (offset composed using the OLD multiplier,
    # before it's updated -- RDK model_json.go:194/208). Cross-checked
    # numerically against the probe: feeding q1=0.3 through this file
    # and feeding [0.3, 0.7, 2.3] (the hand-expanded, non-mimic
    # equivalent) through an otherwise-identical 3-independent-joint
    # URDF produce IDENTICAL poses on RDK v1.0.0; feeding the
    # multiplier-before-offset (wrong-order) result [0.3, 0.7, 2.6]
    # produces a visibly different pose.
    m = parse_urdf(fixtures / "mimic_of_mimic.urdf")
    assert m.dof == 1
    j3 = next(j for j in m.chain() if j.name == "j3")
    assert j3.mimic.source == "q1"
    assert j3.mimic.multiplier == 6.0
    assert j3.mimic.offset == 0.5


def test_mimic_of_mimic_preserves_declared_triple_alongside_composed(fixtures):
    # Fix (post-A3b-approval, Minor #1): in-place composition is correct
    # for joint_values()'s computation but destroys what the file
    # actually said -- a future `validate` finding about a mimic joint
    # must be able to report the source/multiplier/offset the user
    # actually wrote, not the collapsed form (and not float noise from
    # composing that the file never contained).
    #
    # j2 mimics j1 (multiplier=-2.0, offset=0.1); j3 mimics j2
    # (multiplier=3.0, offset=-0.3) -- a mixed-sign three-deep chain
    # chosen because it produces exactly the float noise a naive
    # "just round it" fix would be tempted to paper over: composing
    # gives multiplier=-6.0 (exact) but offset=5.551115123125783e-17
    # (IEEE-754 noise from 3.0*0.1 + -0.3, arithmetically correct, but
    # not a number that belongs in a report -- and not what the file
    # said, which is exactly why declared_offset exists).
    m = parse_urdf(fixtures / "mimic_of_mimic_mixed_sign.urdf")
    j3 = next(j for j in m.chain() if j.name == "j3")

    # Declared: exactly what <mimic joint="j2" multiplier="3.0"
    # offset="-0.3"/> said, untouched by composition.
    assert j3.mimic.declared_source == "j2"
    assert j3.mimic.declared_multiplier == 3.0
    assert j3.mimic.declared_offset == -0.3

    # Composed: collapsed to the ultimate non-mimic source (j1), for
    # joint_values() to use directly.
    assert j3.mimic.source == "j1"
    assert j3.mimic.multiplier == -6.0
    assert j3.mimic.offset == pytest.approx(0.0, abs=1e-15)

    # A joint that directly mimics a non-mimic joint (the common,
    # single-hop case) has an identical declared/composed triple --
    # composition is a no-op there.
    j2 = next(j for j in m.chain() if j.name == "j2")
    assert (j2.mimic.declared_source, j2.mimic.declared_multiplier, j2.mimic.declared_offset) == \
        (j2.mimic.source, j2.mimic.multiplier, j2.mimic.offset) == ("j1", -2.0, 0.1)


@pytest.mark.timeout(5)
def test_mimic_cycle_raises(tmp_path):
    # RDK REJECTS this (verified via the probe against RDK v1.0.0):
    #   circular mimic joint reference detected: joint "j2"
    # A mimic cycle makes value derivation non-terminating -- the same
    # failure mode as chain()'s cycle guard, one layer up. Time-bounded
    # deliberately: a regression here hangs rather than fails.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="l1"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="j2" type="revolute">
        <parent link="l1"/><child link="l2"/>
        <axis xyz="0 0 1"/>
        <mimic joint="j3"/>
      </joint>
      <joint name="j3" type="revolute">
        <parent link="l2"/><child link="l3"/>
        <axis xyz="0 0 1"/>
        <mimic joint="j2"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match=r'circular mimic joint reference detected: joint "j2"'):
        parse_urdf(path)


def test_mimic_self_reference_raises(tmp_path):
    # RDK REJECTS this (verified via the probe against RDK v1.0.0):
    #   circular mimic joint reference detected: joint "j2"
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="l1"/><link name="l2"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="l1"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="j2" type="revolute">
        <parent link="l1"/><child link="l2"/>
        <axis xyz="0 0 1"/>
        <mimic joint="j2"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match=r'circular mimic joint reference detected: joint "j2"'):
        parse_urdf(path)


def test_mimic_of_fixed_joint_raises(tmp_path):
    # RDK REJECTS this (verified via the probe against RDK v1.0.0):
    #   mimic joint references non-existent source joint: joint "j3"
    #   references source "j2" which has no DoF
    # A fixed joint has no position to derive a mimic value from.
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="l1"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="j2" type="fixed">
        <parent link="l1"/><child link="l2"/>
      </joint>
      <joint name="j3" type="revolute">
        <parent link="l2"/><child link="l3"/>
        <axis xyz="0 0 1"/>
        <mimic joint="j2"/>
      </joint>
    </robot>
    """)
    with pytest.raises(
        ValueError,
        match=r'mimic joint references non-existent source joint: joint "j3" references source "j2" which has no DoF',
    ):
        parse_urdf(path)


def test_declared_tip_selects_branch(tmp_path):
    # Part 3: parse_urdf(path, tip=...) stores the declared tip on the
    # model; branching is legal once a tip is declared (Part 2).
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="palm"/>
      <link name="finger_l"/><link name="finger_r"/>
      <joint name="j0" type="revolute">
        <parent link="base"/><child link="palm"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="jl" type="revolute">
        <parent link="palm"/><child link="finger_l"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="jr" type="revolute">
        <parent link="palm"/><child link="finger_r"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    m = parse_urdf(path, tip="finger_l")
    assert m.primary_output_frame == "finger_l"
    assert [j.name for j in m.chain()] == ["j0", "jl"]
    assert m.tip_link == "finger_l"
    # Verified against RDK v1.0.0 via the probe on an equivalent
    # branching SVA fixture: DoF counts joints on BOTH branches, not
    # just the ones on the root->tip path (model.go:150 -- BFS over
    # the whole frame system, not the tip chain).
    assert m.dof == 3


def test_declared_tip_not_in_model_raises(tmp_path):
    path = _write(tmp_path, """
    <robot name="t">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="tip"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>
    """)
    with pytest.raises(ValueError, match="declared tip 'nonexistent' is not a link"):
        parse_urdf(path, tip="nonexistent")


def test_namespaced_root_parses_successfully(tmp_path):
    # Covers Fix 4: a default xmlns on <robot> makes ElementTree report
    # root.tag as "{uri}robot". Before stripping the namespace prefix,
    # this was misreported as "not a URDF file" even though the file
    # genuinely is one. No corpus file was found doing this, but the
    # message was actively misleading, so it's pinned here.
    path = _write(tmp_path, """
    <robot xmlns="http://www.ros.org/urdf" name="t">
      <link name="base"/><link name="tip"/>
      <joint name="j1" type="fixed">
        <parent link="base"/><child link="tip"/>
      </joint>
    </robot>
    """)
    m = parse_urdf(path)
    assert m.name == "t"
    assert [j.name for j in m.chain()] == ["j1"]


def test_records_visual_and_collision_mesh_references(fixtures):
    # Fix 2 (A7 review): armkit needs to COUNT mesh references (not resolve
    # or inspect them -- that's A6's job) so validate can warn that RDK
    # hard-fails on a missing mesh file, a risk armkit itself cannot see.
    # meshed.urdf has one visual mesh on "base" and one collision mesh on
    # "link1"; Link.visual_meshes/collision_meshes already existed in
    # model.py but were never populated by the parser.
    m = parse_urdf(fixtures / "meshed.urdf")
    assert m.links["base"].visual_meshes == ["package://some_pkg/meshes/base.dae"]
    assert m.links["link1"].collision_meshes == ["meshes/link1.stl"]


def test_box_primitives_are_not_counted_as_mesh_references(fixtures):
    # ur20.urdf uses <box> collision primitives and references no meshes at
    # all -- test_parity.py already relies on this to justify "no
    # unresolved-mesh findings expected here". Only <mesh> elements count.
    m = parse_urdf(fixtures / "ur20.urdf")
    assert all(not link.visual_meshes and not link.collision_meshes for link in m.links.values())


def test_joint_with_origin_is_marked_declared(fixtures):
    m = parse_urdf(fixtures / "two_link.urdf")
    assert all(j.has_declared_origin for j in m.chain())


def test_joint_without_origin_is_marked_not_declared(fixtures):
    # no_origin.urdf's joint has no <origin> element at all. armkit
    # defaults it to identity (correct, per the URDF spec) but must still
    # be able to tell a caller that the default was applied -- RDK v1.0.0
    # panics on exactly this case (see test_parity.py), and validate's
    # Fix 2 needs to warn about it per-joint.
    m = parse_urdf(fixtures / "no_origin.urdf")
    j = m.chain()[0]
    assert j.has_declared_origin is False
