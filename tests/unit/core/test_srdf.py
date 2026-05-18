"""Unit tests for SRDF Models, Parser, and Generator to achieve 100% coverage."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from linkforge.core import (
    Chain,
    CollisionPair,
    EndEffector,
    GroupState,
    JointProperty,
    Link,
    LinkSphereApproximation,
    PassiveJoint,
    PlanningGroup,
    Robot,
    RobotGeneratorError,
    RobotParserIOError,
    RobotValidationError,
    SemanticRobotDescription,
    SRDFGenerator,
    SRDFParser,
    SrdfSphere,
    ValidationErrorCode,
    VirtualJoint,
)
from linkforge.core.exceptions import (
    RobotParserUnexpectedError,
    RobotParserXMLRootError,
)


@pytest.fixture
def parser() -> SRDFParser:
    return SRDFParser()


@pytest.fixture
def generator() -> SRDFGenerator:
    return SRDFGenerator()


# ==========================================
# 1. SRDF Models and Validations
# ==========================================


class TestSRDFModels:
    def test_srdf_model_creation(self) -> None:
        """Test creating a basic SRDF model."""
        srdf = SemanticRobotDescription(robot_name="test_robot")
        assert srdf.robot_name == "test_robot"
        assert len(srdf.groups) == 0

    def test_virtual_joint_validations(self) -> None:
        """Test VirtualJoint constructor validations."""
        with pytest.raises(RobotValidationError) as exc:
            VirtualJoint(name="", type="fixed", parent_frame="world", child_link="base_link")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

        with pytest.raises(RobotValidationError) as exc:
            VirtualJoint(
                name="vj", type="invalid_type", parent_frame="world", child_link="base_link"
            )
        assert exc.value.code == ValidationErrorCode.INVALID_VALUE

        vj = VirtualJoint(name="vj", type="fixed", parent_frame="world", child_link="base_link")
        assert vj.name == "vj"

    def test_group_state_validations(self) -> None:
        """Test GroupState constructor validations and normalization."""
        with pytest.raises(RobotValidationError) as exc:
            GroupState(name="", group="arm")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

        with pytest.raises(RobotValidationError) as exc:
            GroupState(name="home", group="")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

        # Normalization checks
        gs1 = GroupState(
            name="home", group="arm", joint_values={"j1": [1.0, 2.0], "j2": 1.5, "j3": "string_val"}
        )
        assert gs1.joint_values["j1"] == (1.0, 2.0)
        assert gs1.joint_values["j2"] == (1.5,)
        assert gs1.joint_values["j3"] == ("string_val",)

    def test_end_effector_validations(self) -> None:
        """Test EndEffector constructor validations."""
        with pytest.raises(RobotValidationError) as exc:
            EndEffector(name="", group="hand", parent_link="wrist")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

        with pytest.raises(RobotValidationError) as exc:
            EndEffector(name="ee", group="", parent_link="wrist")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

    def test_passive_joint_validations(self) -> None:
        """Test PassiveJoint constructor validations."""
        with pytest.raises(RobotValidationError) as exc:
            PassiveJoint(name="")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

    def test_collision_pair_validations(self) -> None:
        """Test CollisionPair constructor validations."""
        with pytest.raises(RobotValidationError) as exc:
            CollisionPair(link1="", link2="link2")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

        with pytest.raises(RobotValidationError) as exc:
            CollisionPair(link1="link1", link2="")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

        with pytest.raises(RobotValidationError) as exc:
            CollisionPair(link1="link1", link2="link1")
        assert exc.value.code == ValidationErrorCode.INVALID_VALUE

    def test_chain_validations(self) -> None:
        """Test Chain constructor validations."""
        with pytest.raises(RobotValidationError) as exc:
            Chain(base_link="", tip_link="tip")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

        with pytest.raises(RobotValidationError) as exc:
            Chain(base_link="base", tip_link="")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

    def test_planning_group_validations(self) -> None:
        """Test PlanningGroup constructor validations."""
        with pytest.raises(RobotValidationError) as exc:
            PlanningGroup(name="", links=["l1"])
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

        with pytest.raises(RobotValidationError) as exc:
            PlanningGroup(name="arm")
        assert exc.value.code == ValidationErrorCode.VALUE_EMPTY

        # Test list conversion to tuple
        pg = PlanningGroup(name="arm", links=["l1", "l2"])
        assert isinstance(pg.links, tuple)
        assert pg.links == ("l1", "l2")

    def test_srdf_sphere_validations(self) -> None:
        """Test SrdfSphere constructor validations."""
        with pytest.raises(RobotValidationError) as exc:
            SrdfSphere(center_x=0.0, center_y=0.0, center_z=0.0, radius=-1.0)
        assert exc.value.code == ValidationErrorCode.INVALID_VALUE

    def test_link_sphere_approximation_validations(self) -> None:
        """Test LinkSphereApproximation constructor validations."""
        with pytest.raises(RobotValidationError) as exc:
            LinkSphereApproximation(link="")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

    def test_joint_property_validations(self) -> None:
        """Test JointProperty constructor validations."""
        with pytest.raises(RobotValidationError) as exc:
            JointProperty(joint_name="", property_name="p", value="v")
        assert exc.value.code == ValidationErrorCode.VALUE_EMPTY

    # ==========================================
    # 2. Namespace Prefixing
    # ==========================================

    def test_with_prefix(self) -> None:
        """Test prefixing semantic descriptions and all nested components."""
        vj = VirtualJoint(name="vj", type="fixed", parent_frame="world", child_link="base_link")
        chain = Chain(base_link="base", tip_link="tip")
        group = PlanningGroup(
            name="arm", links=["l1"], joints=["j1"], chains=[chain], subgroups=["sg"]
        )
        gs = GroupState(name="home", group="arm", joint_values={"j1": 1.0})
        ee = EndEffector(name="ee", group="arm", parent_link="wrist", parent_group="parent_sg")
        pj = PassiveJoint(name="pj")
        cp_dis = CollisionPair(link1="l1", link2="l2", reason="adjacent")
        cp_en = CollisionPair(link1="l3", link2="l4", reason="never")
        lsa = LinkSphereApproximation(link="l1", spheres=[SrdfSphere(0, 0, 0, 0.1)])
        jp = JointProperty(joint_name="j1", property_name="p", value="v")

        desc = SemanticRobotDescription(
            robot_name="my_robot",
            virtual_joints=[vj],
            groups=[group],
            group_states=[gs],
            end_effectors=[ee],
            passive_joints=[pj],
            disabled_collisions=[cp_dis],
            enabled_collisions=[cp_en],
            no_default_collision_links=["l1"],
            link_sphere_approximations=[lsa],
            joint_properties=[jp],
        )

        prefixed = desc.with_prefix("prefix_")
        assert prefixed.robot_name == "prefix_my_robot"
        assert prefixed.virtual_joints[0].name == "prefix_vj"
        assert prefixed.virtual_joints[0].child_link == "prefix_base_link"
        assert prefixed.groups[0].name == "prefix_arm"
        assert prefixed.groups[0].links == ("prefix_l1",)
        assert prefixed.groups[0].joints == ("prefix_j1",)
        assert prefixed.groups[0].chains[0].base_link == "prefix_base"
        assert prefixed.groups[0].chains[0].tip_link == "prefix_tip"
        assert prefixed.groups[0].subgroups == ("prefix_sg",)
        assert prefixed.group_states[0].name == "prefix_home"
        assert prefixed.group_states[0].group == "prefix_arm"
        assert "prefix_j1" in prefixed.group_states[0].joint_values
        assert prefixed.end_effectors[0].name == "prefix_ee"
        assert prefixed.end_effectors[0].group == "prefix_arm"
        assert prefixed.end_effectors[0].parent_link == "prefix_wrist"
        assert prefixed.end_effectors[0].parent_group == "prefix_parent_sg"
        assert prefixed.passive_joints[0].name == "prefix_pj"
        assert prefixed.disabled_collisions[0].link1 == "prefix_l1"
        assert prefixed.disabled_collisions[0].link2 == "prefix_l2"
        assert prefixed.enabled_collisions[0].link1 == "prefix_l3"
        assert prefixed.enabled_collisions[0].link2 == "prefix_l4"
        assert prefixed.no_default_collision_links == ("prefix_l1",)
        assert prefixed.link_sphere_approximations[0].link == "prefix_l1"
        assert prefixed.joint_properties[0].joint_name == "prefix_j1"

    # ==========================================
    # 3. Merging and Deduplication
    # ==========================================

    def test_merge_with(self) -> None:
        """Test merge_with and deduplication logic."""
        vj1 = VirtualJoint(name="vj", type="fixed", parent_frame="world", child_link="base_link")
        vj2 = VirtualJoint(name="vj", type="fixed", parent_frame="world2", child_link="base_link2")
        vj3 = VirtualJoint(name="vj2", type="fixed", parent_frame="world", child_link="base_link")

        g1 = PlanningGroup(name="arm", links=["l1"])
        g2 = PlanningGroup(name="arm", links=["l2"])
        g3 = PlanningGroup(name="hand", links=["l3"])

        cp1 = CollisionPair(link1="l1", link2="l2", reason="adj")
        cp2 = CollisionPair(link1="l2", link2="l1", reason="adj")  # Symmetric duplicate
        cp3 = CollisionPair(link1="l1", link2="l3", reason="never")

        lsa1 = LinkSphereApproximation(link="l1", spheres=[SrdfSphere(0, 0, 0, 0.1)])
        lsa2 = LinkSphereApproximation(link="l1", spheres=[SrdfSphere(0, 0, 0, 0.2)])
        lsa3 = LinkSphereApproximation(link="l2", spheres=[SrdfSphere(0, 0, 0, 0.1)])

        jp1 = JointProperty(joint_name="j1", property_name="p", value="v1")
        jp2 = JointProperty(joint_name="j1", property_name="p", value="v2")
        jp3 = JointProperty(joint_name="j1", property_name="p2", value="v3")

        desc1 = SemanticRobotDescription(
            virtual_joints=[vj1],
            groups=[g1],
            disabled_collisions=[cp1],
            no_default_collision_links=["l1"],
            link_sphere_approximations=[lsa1],
            joint_properties=[jp1],
        )

        desc2 = SemanticRobotDescription(
            virtual_joints=[vj2, vj3],
            groups=[g2, g3],
            disabled_collisions=[cp2, cp3],
            no_default_collision_links=["l1", "l2"],
            link_sphere_approximations=[lsa2, lsa3],
            joint_properties=[jp2, jp3],
        )

        merged = desc1.merge_with(desc2)
        assert len(merged.virtual_joints) == 2
        assert merged.virtual_joints[0].parent_frame == "world"  # Kept original
        assert merged.virtual_joints[1].name == "vj2"

        assert len(merged.groups) == 2
        assert merged.groups[0].links == ("l1",)
        assert merged.groups[1].name == "hand"

        # Deduplication of symmetric collision pairs
        assert len(merged.disabled_collisions) == 2
        assert merged.disabled_collisions[0].link1 == "l1"
        assert merged.disabled_collisions[0].link2 == "l2"
        assert merged.disabled_collisions[1].link1 == "l1"
        assert merged.disabled_collisions[1].link2 == "l3"

        assert merged.no_default_collision_links == ("l1", "l2")

        assert len(merged.link_sphere_approximations) == 2
        assert merged.link_sphere_approximations[0].link == "l1"
        assert merged.link_sphere_approximations[1].link == "l2"

        assert len(merged.joint_properties) == 2
        assert merged.joint_properties[0].property_name == "p"
        assert merged.joint_properties[0].value == "v1"
        assert merged.joint_properties[1].property_name == "p2"

    def test_normalized(self) -> None:
        """Test normalized() sorting stability."""
        desc = SemanticRobotDescription(
            virtual_joints=[
                VirtualJoint("vj2", "fixed", "w", "c"),
                VirtualJoint("vj1", "fixed", "w", "c"),
            ],
            groups=[PlanningGroup("g2", links=["l1"]), PlanningGroup("g1", links=["l1"])],
            group_states=[GroupState("gs", "g2"), GroupState("gs", "g1")],
            passive_joints=[PassiveJoint("pj2"), PassiveJoint("pj1")],
            disabled_collisions=[CollisionPair("l2", "l1"), CollisionPair("l1", "l3")],
            no_default_collision_links=["l2", "l1"],
            link_sphere_approximations=[
                LinkSphereApproximation("l2"),
                LinkSphereApproximation("l1"),
            ],
            joint_properties=[JointProperty("j2", "p", "v"), JointProperty("j1", "p", "v")],
        )

        norm = desc.normalized()
        assert norm.virtual_joints[0].name == "vj1"
        assert norm.groups[0].name == "g1"
        assert norm.group_states[0].group == "g1"
        assert norm.passive_joints[0].name == "pj1"
        # sort by: sorted([l1, l2])
        assert norm.disabled_collisions[0].link1 == "l2"  # (l1, l2) sorted
        assert norm.disabled_collisions[1].link1 == "l1"  # (l1, l3) sorted
        assert norm.no_default_collision_links == ("l1", "l2")
        assert norm.link_sphere_approximations[0].link == "l1"
        assert norm.joint_properties[0].joint_name == "j1"


# ==========================================
# 4. SRDF Parser Edge Cases
# ==========================================


class TestSRDFParserCoverage:
    def test_detect_xacro_content(self, parser) -> None:
        """Test parser detects unexpanded XACRO content and raises error."""
        xml_ns = """<?xml version="1.0"?>
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:macro name="m"/>
        </robot>
        """
        with pytest.raises(RobotParserUnexpectedError) as exc:
            parser.parse_string(xml_ns)
        assert "Unexpanded XACRO detected in SRDF" in str(exc.value)

        xml_attrib = """<?xml version="1.0"?>
        <robot>
            <group name="${group_name}"/>
        </robot>
        """
        with pytest.raises(RobotParserUnexpectedError) as exc:
            parser.parse_string(xml_attrib)
        assert "Unexpanded XACRO detected in SRDF" in str(exc.value)

    def test_empty_or_truncated_xml(self, parser) -> None:
        """Test parsing empty/truncated strings."""
        with pytest.raises(RobotParserUnexpectedError) as exc:
            parser.parse_string("")
        assert "SRDF parse" in str(exc.value)

        with pytest.raises(RobotParserUnexpectedError) as exc:
            parser.parse_string("<robot>")
        assert "SRDF parse" in str(exc.value)

    def test_incorrect_root_tag(self, parser) -> None:
        """Test parsing XML with incorrect root tag."""
        xml = "<semantic_robot></semantic_robot>"
        with pytest.raises(RobotParserXMLRootError):
            parser.parse_string(xml)

    def test_xml_depth_protection(self, parser) -> None:
        """Test parser catches overly nested XML exceeding depth limit."""
        nested = "<a>" * 2005 + "</a>" * 2005
        # Prepend valid robot tag
        xml = f"<robot>{nested}</robot>"
        with pytest.raises(RobotParserUnexpectedError) as exc:
            parser.parse_string(xml)
        assert "nesting" in str(exc.value)

    def test_skip_malformed_subelements(self, parser) -> None:
        """Test parser skips sub-elements that fail validation or are missing attributes."""
        xml = """<?xml version="1.0"?>
        <robot name="skip_test">
            <!-- planning group invalid -->
            <group/>
            <!-- virtual joint invalid -->
            <virtual_joint type="fixed"/>
            <!-- group state invalid -->
            <group_state name="home"/>
            <group_state name="home" group="arm">
                <joint/>
                <joint name="j1"/> <!-- missing value -->
                <joint name="j2" value="invalid_float"/>
            </group_state>
            <!-- end effector invalid -->
            <end_effector name="ee"/>
            <!-- collision pair invalid -->
            <disable_collisions link1="l1"/>
            <enable_collisions link2="l2"/>
            <disable_collisions link1="l1" link2="l1"/> <!-- self-collision error -->
            <!-- passive joint skipped if no name -->
            <passive_joint/>
            <!-- disable default collisions skipped if no link -->
            <disable_default_collisions/>
            <!-- link sphere approximation skipped if no link, or sphere has missing/invalid values -->
            <link_sphere_approximation/>
            <link_sphere_approximation link="l1">
                <sphere/>
                <sphere center="0 0" radius="0.1"/>
                <sphere center="0 0 0" radius="-0.1"/>
            </link_sphere_approximation>
            <!-- joint property invalid -->
            <joint_property joint_name="j1"/>
        </robot>
        """
        srdf = parser.parse_string(xml)
        assert srdf.robot_name == "skip_test"
        assert len(srdf.groups) == 0
        assert len(srdf.virtual_joints) == 0
        assert len(srdf.group_states) == 1
        assert len(srdf.group_states[0].joint_values) == 0
        assert len(srdf.end_effectors) == 0
        assert len(srdf.disabled_collisions) == 0
        assert len(srdf.passive_joints) == 0
        assert len(srdf.no_default_collision_links) == 0
        assert len(srdf.link_sphere_approximations) == 1
        assert len(srdf.link_sphere_approximations[0].spheres) == 0
        assert len(srdf.joint_properties) == 0

    def test_parse_from_file_exceptions(self, parser, tmp_path) -> None:
        """Test file parsing exceptions and XML root checks."""
        f_missing = tmp_path / "missing.srdf"
        with pytest.raises(RobotParserIOError):
            parser.parse(f_missing)

        f_invalid_root = tmp_path / "bad_root.srdf"
        f_invalid_root.write_text("<bad_root/>", encoding="utf-8")
        with pytest.raises(RobotParserXMLRootError):
            parser.parse(f_invalid_root)

        f_empty = tmp_path / "empty.srdf"
        f_empty.write_text("", encoding="utf-8")
        with pytest.raises(RobotParserUnexpectedError) as exc:
            parser.parse(f_empty)
        assert "SRDF file parse" in str(exc.value)

        f_malformed = tmp_path / "malformed.srdf"
        f_malformed.write_text("<robot><unclosed>", encoding="utf-8")
        with pytest.raises(RobotParserUnexpectedError) as exc:
            parser.parse(f_malformed)
        assert "SRDF file parse" in str(exc.value)

    def test_parser_generator_coverage(self, parser) -> None:
        """Cover remaining parser and generator logic to hit 100%."""
        xml = """<?xml version="1.0"?>
        <robot name="valid_robot">
            <virtual_joint name="vj" type="fixed" parent_frame="world" child_link="base_link"/>
            <group name="arm">
                <link name="link1"/>
                <joint name="joint1"/>
                <chain base_link="base" tip_link="tip"/>
                <group name="sub_arm"/>
            </group>
            <group name="sub_arm">
                <link name="link2"/>
            </group>
            <group_state name="home" group="arm">
                <joint name="joint1" value="0.5"/>
            </group_state>
            <end_effector name="ee" group="sub_arm" parent_link="link2" parent_group="arm"/>
            <disable_collisions link1="link1" link2="link2" reason="adjacent"/>
            <enable_collisions link1="link1" link2="link2" reason="never"/>
            <passive_joint name="joint1"/>
            <disable_default_collisions link="link1"/>
            <link_sphere_approximation link="link1">
                <sphere center="0 0 0" radius="0.1"/>
            </link_sphere_approximation>
            <joint_property joint_name="joint1" property_name="p" value="v"/>
        </robot>
        """
        srdf = parser.parse_string(xml)
        assert srdf.robot_name == "valid_robot"
        assert len(srdf.groups) == 2
        assert len(srdf.virtual_joints) == 1
        assert len(srdf.group_states) == 1
        assert len(srdf.end_effectors) == 1
        assert len(srdf.disabled_collisions) == 1
        assert len(srdf.enabled_collisions) == 1
        assert len(srdf.passive_joints) == 1
        assert len(srdf.no_default_collision_links) == 1
        assert len(srdf.link_sphere_approximations) == 1
        assert len(srdf.joint_properties) == 1

        # Test cross reference warning for end effector and group state
        xml_warnings = """<?xml version="1.0"?>
        <robot name="warn_robot">
            <group_state name="gs" group="unknown_group"/>
            <end_effector name="ee" group="unknown_group" parent_link="l"/>
        </robot>
        """
        parser.parse_string(xml_warnings)

        # Mock constructors to raise exceptions during parser iteration
        import unittest.mock as mock

        with mock.patch(
            "linkforge.core.parsers.srdf_parser.VirtualJoint", side_effect=Exception("vj error")
        ):
            xml_vj_err = """<?xml version="1.0"?>
            <robot name="vj_err">
                <virtual_joint name="vj" type="fixed" parent_frame="w" child_link="c"/>
            </robot>
            """
            res = parser.parse_string(xml_vj_err)
            assert len(res.virtual_joints) == 0

        with mock.patch(
            "linkforge.core.parsers.srdf_parser.EndEffector", side_effect=Exception("ee error")
        ):
            xml_ee_err = """<?xml version="1.0"?>
            <robot name="ee_err">
                <end_effector name="ee" group="g" parent_link="l"/>
            </robot>
            """
            res = parser.parse_string(xml_ee_err)
            assert len(res.end_effectors) == 0

        with mock.patch(
            "linkforge.core.parsers.srdf_parser.LinkSphereApproximation",
            side_effect=Exception("lsa error"),
        ):
            xml_lsa_err = """<?xml version="1.0"?>
            <robot name="lsa_err">
                <link_sphere_approximation link="l"/>
            </robot>
            """
            res = parser.parse_string(xml_lsa_err)
            assert len(res.link_sphere_approximations) == 0

        with mock.patch(
            "linkforge.core.parsers.srdf_parser.JointProperty", side_effect=Exception("jp error")
        ):
            xml_jp_err = """<?xml version="1.0"?>
            <robot name="jp_err">
                <joint_property joint_name="j" property_name="p" value="v"/>
            </robot>
            """
            res = parser.parse_string(xml_jp_err)
            assert len(res.joint_properties) == 0

        with mock.patch(
            "linkforge.core.parsers.srdf_parser.PlanningGroup", side_effect=Exception("pg error")
        ):
            xml_pg_err = """<?xml version="1.0"?>
            <robot name="pg_err">
                <group name="arm"><link name="l1"/></group>
            </robot>
            """
            res = parser.parse_string(xml_pg_err)
            assert len(res.groups) == 0

        with mock.patch(
            "linkforge.core.parsers.srdf_parser.GroupState", side_effect=Exception("gs error")
        ):
            xml_gs_err = """<?xml version="1.0"?>
            <robot name="gs_err">
                <group name="arm"><link name="l1"/></group>
                <group_state name="home" group="arm">
                    <joint name="j1" value="0.5"/>
                </group_state>
            </robot>
            """
            res = parser.parse_string(xml_gs_err)
            assert len(res.group_states) == 0

        # Mock iterparse exceptions in parse_string
        with mock.patch("xml.etree.ElementTree.iterparse", side_effect=StopIteration):
            with pytest.raises(RobotParserUnexpectedError) as exc:
                parser.parse_string("<robot/>")
            assert "Empty or truncated XML" in str(exc.value)

        with mock.patch("xml.etree.ElementTree.iterparse", side_effect=Exception("custom error")):
            with pytest.raises(RobotParserUnexpectedError) as exc:
                parser.parse_string("<robot/>")
            assert "custom error" in str(exc.value)

        with mock.patch.object(
            parser, "_parse_from_context", side_effect=ET.ParseError("parse error")
        ):
            with pytest.raises(RobotParserUnexpectedError) as exc:
                parser.parse_string("<robot/>")
            assert "parse error" in str(exc.value)

        with mock.patch.object(
            parser, "_parse_from_context", side_effect=Exception("unexpected error")
        ):
            with pytest.raises(RobotParserUnexpectedError) as exc:
                parser.parse_string("<robot/>")
            assert "unexpected error" in str(exc.value)

        # Mock parse file exceptions
        with mock.patch.object(parser, "_validate_file"):
            with mock.patch("xml.etree.ElementTree.iterparse", side_effect=StopIteration):
                with pytest.raises(RobotParserUnexpectedError) as exc:
                    parser.parse(Path("some_file.srdf"))
                assert "Empty or truncated XML" in str(exc.value)

            with mock.patch(
                "xml.etree.ElementTree.iterparse", side_effect=Exception("file exception")
            ):
                with pytest.raises(RobotParserIOError) as exc:
                    parser.parse(Path("some_file.srdf"))
                assert "file exception" in str(exc.value)


# ==========================================
# 5. SRDF Generator Edge Cases
# ==========================================


class TestSRDFGeneratorCoverage:
    def test_generate_validation_failure(self, generator) -> None:
        """Test generator raises RobotGeneratorError on validation failures."""
        robot = Robot(name="invalid_robot")
        # Creating invalid kinematics (link loop or similar issue) to trigger validation failure
        # For simplicity, let's inject an issue that validator fails on.
        # But we can also mock validation result to fail.
        import unittest.mock as mock

        from linkforge.core.validation import Severity, ValidationIssue, ValidationResult

        with mock.patch("linkforge.core.validation.RobotValidator.validate") as mock_val:
            mock_val.return_value = ValidationResult(
                issues=[
                    ValidationIssue(
                        severity=Severity.ERROR, title="SemanticCheck", message="Invalid structure"
                    )
                ]
            )
            with pytest.raises(RobotGeneratorError) as exc:
                generator.generate(robot, validate=True)
            assert "Robot validation failed" in str(exc.value)

    def test_generate_validation_success(self, generator) -> None:
        """Test generator succeeds on validation check."""
        robot = Robot(name="valid_robot")
        import unittest.mock as mock

        from linkforge.core.validation import ValidationResult

        with mock.patch("linkforge.core.validation.RobotValidator.validate") as mock_val:
            mock_val.return_value = ValidationResult(issues=[])
            xml_str = generator.generate(robot, validate=True)
            root = ET.fromstring(xml_str)
            assert root.tag == "robot"

    def test_empty_semantic_description_warning(self, generator) -> None:
        """Test generating SRDF when robot has no semantic description."""
        robot = Robot(name="no_semantic")
        robot.semantic = None
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)
        assert root.tag == "robot"
        assert root.get("name") == "no_semantic"
        assert len(list(root)) == 0

    def test_full_generation_suite(self, generator) -> None:
        """Test generating all optional elements inside the SRDF generator."""
        robot = Robot(name="full_robot")
        robot.add_link(Link(name="l1"))
        robot.add_link(Link(name="l2"))

        semantic = SemanticRobotDescription(
            robot_name="full_robot_semantic",
            virtual_joints=[VirtualJoint("vj", "fixed", "world", "l1")],
            groups=[
                PlanningGroup(
                    "arm",
                    links=["l1"],
                    joints=["j1"],
                    chains=[Chain("l1", "l2")],
                    subgroups=["sub1"],
                )
            ],
            group_states=[GroupState("home", "arm", joint_values={"j1": 0.5})],
            end_effectors=[EndEffector("ee", "arm", "l2", "sub1")],
            passive_joints=[PassiveJoint("pj1")],
            disabled_collisions=[CollisionPair("l1", "l2", "adj")],
            enabled_collisions=[CollisionPair("l1", "l2", "never")],
            no_default_collision_links=["l1"],
            link_sphere_approximations=[
                LinkSphereApproximation("l1", spheres=[SrdfSphere(0, 0, 0, 0.1)])
            ],
            joint_properties=[JointProperty("j1", "p", "v")],
        )
        robot.semantic = semantic

        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)
        assert root.tag == "robot"
        assert root.get("name") == "full_robot_semantic"

        assert root.find("virtual_joint") is not None
        assert root.find("group") is not None
        assert root.find("group_state") is not None
        assert root.find("end_effector") is not None
        assert root.find("passive_joint") is not None
        assert root.find("disable_collisions") is not None
        assert root.find("enable_collisions") is not None
        assert root.find("disable_default_collisions") is not None
        assert root.find("link_sphere_approximation") is not None
        assert root.find("joint_property") is not None


def test_srdf_parser_remaining_coverage(parser) -> None:
    """Verify all remaining branch paths in srdf_parser.py."""
    # 1. Missing attributes inside group (link, joint, chain, subgroup)
    xml_groups = """<?xml version="1.0"?>
    <robot name="r">
        <group name="arm">
            <link name="valid_link"/>
            <link/> <!-- missing name attribute -->
            <joint/> <!-- missing name attribute -->
            <chain base_link="base"/> <!-- missing tip_link attribute -->
            <chain tip_link="tip"/> <!-- missing base_link attribute -->
            <group/> <!-- subgroup missing name attribute -->
        </group>
    </robot>
    """
    srdf = parser.parse_string(xml_groups)
    assert len(srdf.groups) == 1
    assert len(srdf.groups[0].links) == 1
    assert srdf.groups[0].links[0] == "valid_link"
    assert len(srdf.groups[0].joints) == 0
    assert len(srdf.groups[0].chains) == 0
    assert len(srdf.groups[0].subgroups) == 0

    # 2. Joint state with whitespace/empty values in j_val_str
    xml_joint_val = """<?xml version="1.0"?>
    <robot name="r">
        <group name="arm"/>
        <group_state name="home" group="arm">
            <joint name="j" value="   "/> <!-- splits to empty list -->
        </group_state>
    </robot>
    """
    srdf2 = parser.parse_string(xml_joint_val)
    assert len(srdf2.group_states) == 1
    assert len(srdf2.group_states[0].joint_values) == 0

    # 3. Unknown root-level elements skipped
    xml_unknown = """<?xml version="1.0"?>
    <robot name="r">
        <unknown_tag attribute="val"/>
    </robot>
    """
    srdf3 = parser.parse_string(xml_unknown)
    assert srdf3.robot_name == "r"
