"""Unit tests for URDF Models, Parser, and Generator."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest
from linkforge_core.base import XacroDetectedError
from linkforge_core.generators.urdf_generator import URDFGenerator
from linkforge_core.models import (
    Box,
    Color,
    Cylinder,
    Joint,
    JointType,
    Link,
    Material,
    Mesh,
    Robot,
    Vector3,
    Visual,
)
from linkforge_core.parsers.urdf_parser import URDFParser


@pytest.fixture
def parser() -> URDFParser:
    return URDFParser()


@pytest.fixture
def generator() -> URDFGenerator:
    return URDFGenerator(pretty_print=False)


# URDF Parser Unit Tests


class TestURDFParserInternal:
    """Tests for individual element parsing methods in URDFParser."""

    def test_parse_geometry_box(self, parser) -> None:
        """Test parsing box geometry."""
        xml = '<geometry><box size="1 2 3"/></geometry>'
        elem = ET.fromstring(xml)
        geom = parser._parse_geometry_element(elem)
        assert isinstance(geom, Box)
        assert geom.size.x == 1.0

    def test_parse_geometry_cylinder(self, parser) -> None:
        """Test parsing cylinder geometry."""
        xml = '<geometry><cylinder radius="0.5" length="2.0"/></geometry>'
        elem = ET.fromstring(xml)
        geom = parser._parse_geometry_element(elem)
        assert isinstance(geom, Cylinder)
        assert geom.radius == 0.5

    def test_parse_geometry_mesh(self, parser) -> None:
        """Test parsing mesh geometry."""
        xml = '<geometry><mesh filename="mesh.stl" scale="0.1 0.1 0.1"/></geometry>'
        elem = ET.fromstring(xml)
        geom = parser._parse_geometry_element(elem)
        assert isinstance(geom, Mesh)
        assert geom.resource == "mesh.stl"
        assert geom.scale.x == 0.1

    def test_parse_material_color(self, parser) -> None:
        """Test parsing material with color."""
        xml = '<material name="blue"><color rgba="0 0 1 1"/></material>'
        elem = ET.fromstring(xml)
        mat = parser._parse_material_element(elem, {})
        assert mat.name == "blue"
        assert mat.color.b == 1.0

    def test_parse_joint_comprehensive(self, parser) -> None:
        """Test parsing joint with limits, dynamics, mimic, etc."""
        xml = """
        <joint name="j1" type="revolute">
            <parent link="base"/><child link="link1"/>
            <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
            <dynamics damping="0.5" friction="0.1"/>
            <mimic joint="j0" multiplier="2.0" offset="0.5"/>
            <safety_controller soft_lower_limit="-1" soft_upper_limit="1" k_position="15" k_velocity="10"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)
        assert joint.name == "j1"
        assert joint.limits.lower == -1.57
        assert joint.dynamics.damping == 0.5
        assert joint.mimic.multiplier == 2.0
        assert joint.safety_controller.k_position == 15.0


# URDF Generator Unit Tests


class TestURDFGeneratorInternal:
    """Tests for generating URDF XML from Robot model."""

    def test_generate_basic_robot(self, generator) -> None:
        """Test generating a minimal robot."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))
        robot.add_link(Link(name="child"))
        robot.add_joint(Joint(name="j1", parent="base_link", child="child", type=JointType.FIXED))

        xml_str = generator.generate(robot)
        root = ET.fromstring(xml_str)
        assert root.get("name") == "test_robot"
        assert len(root.findall("link")) == 2

    def test_generate_materials_deduplication(self, generator) -> None:
        """Test material deduplication logic."""
        robot = Robot(name="mat_robot")
        mat = Material(name="red", color=Color(1, 0, 0, 1))
        robot.add_link(
            Link(name="l1", visuals=[Visual(geometry=Box(Vector3(1, 1, 1)), material=mat)])
        )
        robot.add_link(
            Link(name="l2", visuals=[Visual(geometry=Box(Vector3(1, 1, 1)), material=mat)])
        )

        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)
        # Should have 1 global material definition
        assert len(root.findall("material")) == 1


# Robustness and Security Unit Tests


class TestURDFRobustness:
    """Tests for edge cases, sanitization, and security."""

    def test_inertia_sanitization(self, parser) -> None:
        """Ensure inertia tensor violations fall back to safe minimal values."""
        # Violating triangle inequality should fallback to minimal valid tensor
        elem = ET.fromstring(
            '<inertial><mass value="1"/><inertia ixx="10" iyy="1" izz="1"/></inertial>'
        )
        inertial = parser._parse_inertial_element(elem)
        assert inertial.inertia.ixx == 1e-6

    def test_mesh_path_security(self, parser, tmp_path) -> None:
        """Verify mesh path security checks."""
        elem = ET.fromstring('<geometry><mesh filename="/etc/passwd"/></geometry>')
        # Should return None and log warning for absolute paths or traversal
        assert parser._parse_geometry_element(elem, base_directory=tmp_path) is None

    def test_xacro_detection(self, parser) -> None:
        """Verify XACRO artifacts trigger detection error."""
        xml = '<robot xmlns:xacro="http://ros.org/wiki/xacro"><xacro:macro name="m"/></robot>'
        with pytest.raises(XacroDetectedError):
            parser.parse_string(xml)

    def test_duplicate_names_handling(self, parser) -> None:
        """Verify that duplicate link/joint names are skipped."""
        xml = """
        <robot name="dupe_bot">
            <link name="l1"/>
            <link name="l1"/>
        </robot>
        """
        robot = parser.parse_string(xml)
        assert len(robot.links) == 1


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
