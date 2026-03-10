"""Tests for URDF XML parser."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from linkforge_core.base import RobotParserError
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models import (
    Box,
    Color,
    Cylinder,
    JointType,
    Material,
    Mesh,
    Sphere,
)
from linkforge_core.parsers.urdf_parser import URDFParser
from linkforge_core.utils.xml_utils import parse_vector3


class TestParseVector3:
    """Tests for parse_vector3 function."""

    def test_parse_simple_vector(self):
        """Test parsing simple space-separated vector."""
        vec = parse_vector3("1.0 2.0 3.0")
        assert vec.x == pytest.approx(1.0)
        assert vec.y == pytest.approx(2.0)
        assert vec.z == pytest.approx(3.0)

    def test_parse_vector_with_extra_spaces(self):
        """Test parsing vector with extra spaces."""
        vec = parse_vector3("  1.5   2.5   3.5  ")
        assert vec.x == pytest.approx(1.5)
        assert vec.y == pytest.approx(2.5)
        assert vec.z == pytest.approx(3.5)

    def test_parse_negative_values(self):
        """Test parsing vector with negative values."""
        vec = parse_vector3("-1.0 -2.0 -3.0")
        assert vec.x == pytest.approx(-1.0)
        assert vec.y == pytest.approx(-2.0)
        assert vec.z == pytest.approx(-3.0)

    def test_parse_zeros(self):
        """Test parsing zero vector."""
        vec = parse_vector3("0 0 0")
        assert vec.x == pytest.approx(0.0)
        assert vec.y == pytest.approx(0.0)
        assert vec.z == pytest.approx(0.0)

    def test_parse_invalid_too_few_values(self):
        """Test that parsing with too few values raises RobotModelError."""
        with pytest.raises(RobotModelError, match="Expected 3 values"):
            parse_vector3("1.0 2.0")

    def test_parse_invalid_too_many_values(self):
        """Test that parsing with too many values raises RobotModelError."""
        with pytest.raises(RobotModelError, match="Expected 3 values"):
            parse_vector3("1.0 2.0 3.0 4.0")

    def test_parse_invalid_non_numeric(self):
        """Test that parsing non-numeric values raises RobotModelError."""
        with pytest.raises(RobotModelError, match="Invalid Vector3 format"):
            parse_vector3("1.0 abc 3.0")

    def test_parse_empty_string(self):
        """Test that parsing empty string raises RobotModelError."""
        with pytest.raises(RobotModelError, match="Expected 3 values"):
            parse_vector3("")


class TestParseOrigin:
    """Tests for _parse_origin_element method."""

    @pytest.fixture
    def parser(self):
        return URDFParser()

    def test_parse_origin_with_xyz_and_rpy(self, parser):
        """Test parsing origin with both xyz and rpy."""
        elem = ET.fromstring('<origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>')
        transform = parser._parse_origin_element(elem)

        assert transform.xyz.x == pytest.approx(1.0)
        assert transform.xyz.y == pytest.approx(2.0)
        assert transform.xyz.z == pytest.approx(3.0)
        assert transform.rpy.x == pytest.approx(0.1)
        assert transform.rpy.y == pytest.approx(0.2)
        assert transform.rpy.z == pytest.approx(0.3)

    def test_parse_origin_xyz_only(self, parser):
        """Test parsing origin with only xyz."""
        elem = ET.fromstring('<origin xyz="1 2 3"/>')
        transform = parser._parse_origin_element(elem)

        assert transform.xyz.x == pytest.approx(1.0)
        assert transform.xyz.y == pytest.approx(2.0)
        assert transform.xyz.z == pytest.approx(3.0)
        assert transform.rpy.x == pytest.approx(0.0)
        assert transform.rpy.y == pytest.approx(0.0)
        assert transform.rpy.z == pytest.approx(0.0)

    def test_parse_origin_rpy_only(self, parser):
        """Test parsing origin with only rpy."""
        elem = ET.fromstring('<origin rpy="1.57 0 0"/>')
        transform = parser._parse_origin_element(elem)

        assert transform.xyz.x == pytest.approx(0.0)
        assert transform.xyz.y == pytest.approx(0.0)
        assert transform.xyz.z == pytest.approx(0.0)
        assert transform.rpy.x == pytest.approx(1.57)
        assert transform.rpy.y == pytest.approx(0.0)
        assert transform.rpy.z == pytest.approx(0.0)

    def test_parse_origin_none(self, parser):
        """Test parsing None origin returns identity."""
        transform = parser._parse_origin_element(None)
        assert transform.xyz.x == pytest.approx(0.0)
        assert transform.xyz.y == pytest.approx(0.0)
        assert transform.xyz.z == pytest.approx(0.0)
        assert transform.rpy.x == pytest.approx(0.0)
        assert transform.rpy.y == pytest.approx(0.0)
        assert transform.rpy.z == pytest.approx(0.0)

    def test_parse_origin_empty(self, parser):
        """Test parsing empty origin element."""
        elem = ET.fromstring("<origin/>")
        transform = parser._parse_origin_element(elem)

        # Should use defaults (0 0 0)
        assert transform.xyz.x == pytest.approx(0.0)
        assert transform.xyz.y == pytest.approx(0.0)
        assert transform.xyz.z == pytest.approx(0.0)


class TestParseGeometry:
    """Tests for _parse_geometry_element method."""

    @pytest.fixture
    def parser(self):
        return URDFParser()

    def test_parse_box(self, parser):
        """Test parsing box geometry."""
        elem = ET.fromstring('<geometry><box size="1 2 3"/></geometry>')
        geom = parser._parse_geometry_element(elem)

        assert isinstance(geom, Box)
        assert geom.size.x == pytest.approx(1.0)
        assert geom.size.y == pytest.approx(2.0)
        assert geom.size.z == pytest.approx(3.0)

    def test_parse_cylinder(self, parser):
        """Test parsing cylinder geometry."""
        elem = ET.fromstring('<geometry><cylinder radius="0.5" length="2.0"/></geometry>')
        geom = parser._parse_geometry_element(elem)

        assert isinstance(geom, Cylinder)
        assert geom.radius == pytest.approx(0.5)
        assert geom.length == pytest.approx(2.0)

    def test_parse_sphere(self, parser):
        """Test parsing sphere geometry."""
        elem = ET.fromstring('<geometry><sphere radius="1.5"/></geometry>')
        geom = parser._parse_geometry_element(elem)

        assert isinstance(geom, Sphere)
        assert geom.radius == pytest.approx(1.5)

    def test_parse_mesh(self, parser):
        """Test parsing mesh geometry."""
        elem = ET.fromstring('<geometry><mesh filename="robot.stl" scale="1 1 1"/></geometry>')
        geom = parser._parse_geometry_element(elem)

        assert isinstance(geom, Mesh)
        assert geom.resource == "robot.stl"
        assert geom.scale.x == pytest.approx(1.0)
        assert geom.scale.y == pytest.approx(1.0)
        assert geom.scale.z == pytest.approx(1.0)

    def test_parse_mesh_with_scale(self, parser):
        """Test parsing mesh with custom scale."""
        elem = ET.fromstring(
            '<geometry><mesh filename="model.dae" scale="0.001 0.001 0.001"/></geometry>'
        )
        geom = parser._parse_geometry_element(elem)

        assert isinstance(geom, Mesh)
        assert geom.scale.x == pytest.approx(0.001)

    def test_parse_empty_geometry(self, parser):
        """Test parsing empty geometry element."""
        elem = ET.fromstring("<geometry></geometry>")
        geom = parser._parse_geometry_element(elem)

        assert geom is None

    def test_parse_box_missing_size(self, parser, caplog):
        """Test that box without size attribute returns None and logs warning."""
        elem = ET.fromstring("<geometry><box/></geometry>")
        geom = parser._parse_geometry_element(elem)

        # Parser is resilient - returns None and logs warning instead of crashing
        assert geom is None
        assert "Invalid box geometry ignored" in caplog.text

    def test_parse_box_negative_dimension(self, parser, caplog):
        """Test that box with negative dimension returns None and logs warning."""
        elem = ET.fromstring('<geometry><box size="1 -2 3"/></geometry>')
        geom = parser._parse_geometry_element(elem)
        assert geom is None
        assert "Invalid box geometry ignored" in caplog.text

    def test_parse_cylinder_negative_radius(self, parser, caplog):
        """Test that cylinder with negative radius returns None and logs warning."""
        elem = ET.fromstring('<geometry><cylinder radius="-0.5" length="2.0"/></geometry>')
        geom = parser._parse_geometry_element(elem)
        assert geom is None
        assert "Invalid cylinder geometry ignored" in caplog.text

    def test_parse_sphere_zero_radius(self, parser, caplog):
        """Test that sphere with zero radius returns None and logs warning."""
        elem = ET.fromstring('<geometry><sphere radius="0"/></geometry>')
        geom = parser._parse_geometry_element(elem)
        assert geom is None
        assert "Invalid sphere geometry ignored" in caplog.text

    def test_parse_mesh_missing_filename(self, parser, caplog):
        """Test that mesh without filename returns None and logs warning."""
        elem = ET.fromstring("<geometry><mesh/></geometry>")
        geom = parser._parse_geometry_element(elem)
        assert geom is None
        assert "Invalid mesh geometry ignored" in caplog.text

    def test_parse_mesh_negative_scale(self, parser, caplog):
        """Test that mesh with negative scale returns None and logs warning."""
        elem = ET.fromstring('<geometry><mesh filename="test.stl" scale="1 -1 1"/></geometry>')
        geom = parser._parse_geometry_element(elem)
        assert geom is None
        assert "Invalid mesh geometry ignored" in caplog.text

    def test_parse_cylinder_invalid_radius(self, parser, caplog):
        """Test that cylinder with invalid radius format returns None and logs warning."""
        elem = ET.fromstring('<geometry><cylinder radius="abc" length="2.0"/></geometry>')
        geom = parser._parse_geometry_element(elem)
        assert geom is None
        assert "Invalid cylinder geometry ignored" in caplog.text


class TestParseMaterial:
    """Tests for _parse_material_element method."""

    @pytest.fixture
    def parser(self):
        return URDFParser()

    def test_parse_material_with_color(self, parser):
        """Test parsing material with RGBA color."""
        elem = ET.fromstring('<material name="red"><color rgba="1 0 0 1"/></material>')
        materials = {}
        mat = parser._parse_material_element(elem, materials)

        assert mat is not None
        assert mat.name == "red"
        assert mat.color.r == pytest.approx(1.0)
        assert mat.color.g == pytest.approx(0.0)
        assert mat.color.b == pytest.approx(0.0)
        assert mat.color.a == pytest.approx(1.0)

    def test_parse_material_rgb_only(self, parser):
        """Test parsing material with RGB (no alpha)."""
        elem = ET.fromstring('<material name="blue"><color rgba="0 0 1"/></material>')
        materials = {}
        mat = parser._parse_material_element(elem, materials)

        assert mat is not None
        assert mat.color.r == pytest.approx(0.0)
        assert mat.color.g == pytest.approx(0.0)
        assert mat.color.b == pytest.approx(1.0)
        assert mat.color.a == pytest.approx(1.0)  # Default alpha

    def test_parse_material_reference(self, parser):
        """Test parsing material reference to existing material."""
        existing = Material(name="gray", color=Color(0.5, 0.5, 0.5, 1.0))
        materials = {"gray": existing}
        elem = ET.fromstring('<material name="gray"/>')
        mat = parser._parse_material_element(elem, materials)

        assert mat is existing  # Should return the same object

    def test_parse_material_none(self, parser):
        """Test parsing None material element."""
        materials = {}
        mat = parser._parse_material_element(None, materials)

        assert mat is None

    def test_parse_material_no_color(self, parser):
        """Test parsing material without color element."""
        elem = ET.fromstring('<material name="no_color"/>')
        materials = {}
        mat = parser._parse_material_element(elem, materials)

        assert mat is None


class TestParseLink:
    """Tests for _parse_link method."""

    @pytest.fixture
    def parser(self):
        return URDFParser()

    def test_parse_simple_link(self, parser):
        """Test parsing simple link with visual."""
        xml = """
        <link name="test_link">
            <visual>
                <geometry><box size="1 1 1"/></geometry>
            </visual>
        </link>
        """
        elem = ET.fromstring(xml)
        materials = {}
        link = parser._parse_link(elem, materials)

        assert link.name == "test_link"
        assert len(link.visuals) == 1
        assert isinstance(link.visuals[0].geometry, Box)

    def test_parse_link_with_collision(self, parser):
        """Test parsing link with collision."""
        xml = """
        <link name="coll_link">
            <collision>
                <geometry><cylinder radius="0.5" length="2.0"/></geometry>
            </collision>
        </link>
        """
        elem = ET.fromstring(xml)
        materials = {}
        link = parser._parse_link(elem, materials)

        assert len(link.collisions) == 1
        assert isinstance(link.collisions[0].geometry, Cylinder)

    def test_parse_link_with_inertial(self, parser):
        """Test parsing link with inertial properties."""
        xml = """
        <link name="inert_link">
            <inertial>
                <mass value="5.0"/>
                <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
            </inertial>
        </link>
        """
        elem = ET.fromstring(xml)
        materials = {}
        link = parser._parse_link(elem, materials)

        assert link.inertial is not None
        assert link.inertial.mass == pytest.approx(5.0)
        assert link.inertial.inertia.ixx == pytest.approx(1.0)

    def test_parse_link_with_material(self, parser):
        """Test parsing link with material."""
        xml = """
        <link name="mat_link">
            <visual>
                <geometry><box size="1 1 1"/></geometry>
                <material name="green"><color rgba="0 1 0 1"/></material>
            </visual>
        </link>
        """
        elem = ET.fromstring(xml)
        materials = {}
        link = parser._parse_link(elem, materials)

        assert len(link.visuals) == 1
        assert link.visuals[0].material is not None
        assert link.visuals[0].material.name == "green"

    def test_parse_link_with_origin(self, parser):
        """Test parsing link with origin offset."""
        xml = """
        <link name="origin_link">
            <visual>
                <origin xyz="1 0 0" rpy="0 0 1.57"/>
                <geometry><box size="1 1 1"/></geometry>
            </visual>
        </link>
        """
        elem = ET.fromstring(xml)
        materials = {}
        link = parser._parse_link(elem, materials)

        assert len(link.visuals) == 1
        assert link.visuals[0].origin.xyz.x == pytest.approx(1.0)
        assert link.visuals[0].origin.rpy.z == pytest.approx(1.57)

    def test_parse_empty_link(self, parser):
        """Test parsing link without visual, collision, or inertial."""
        xml = '<link name="empty_link"/>'
        elem = ET.fromstring(xml)
        materials = {}
        link = parser._parse_link(elem, materials)

        assert link.name == "empty_link"
        assert not link.visuals
        assert not link.collisions
        assert link.inertial is None


class TestParseJoint:
    """Tests for _parse_joint method."""

    @pytest.fixture
    def parser(self):
        return URDFParser()

    def test_parse_revolute_joint(self, parser):
        """Test parsing revolute joint."""
        xml = """
        <joint name="joint1" type="revolute">
            <parent link="link1"/>
            <child link="link2"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.name == "joint1"
        assert joint.type == JointType.REVOLUTE
        assert joint.parent == "link1"
        assert joint.child == "link2"
        assert joint.axis.z == pytest.approx(1.0)
        assert joint.limits is not None
        assert joint.limits.lower == pytest.approx(-1.57)
        assert joint.limits.upper == pytest.approx(1.57)

    def test_parse_continuous_joint(self, parser):
        """Test parsing continuous joint."""
        xml = """
        <joint name="joint2" type="continuous">
            <parent link="link1"/>
            <child link="link2"/>
            <axis xyz="0 1 0"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.type == JointType.CONTINUOUS
        assert joint.axis.y == pytest.approx(1.0)

    def test_parse_fixed_joint(self, parser):
        """Test parsing fixed joint."""
        xml = """
        <joint name="joint3" type="fixed">
            <parent link="link1"/>
            <child link="link2"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.type == JointType.FIXED

    def test_parse_prismatic_joint(self, parser):
        """Test parsing prismatic joint."""
        xml = """
        <joint name="joint4" type="prismatic">
            <parent link="link1"/>
            <child link="link2"/>
            <axis xyz="1 0 0"/>
            <limit lower="0" upper="1" effort="10" velocity="1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.type == JointType.PRISMATIC
        assert joint.axis.x == pytest.approx(1.0)

    def test_parse_joint_with_origin(self, parser):
        """Test parsing joint with origin."""
        xml = """
        <joint name="joint_origin" type="fixed">
            <parent link="link1"/>
            <child link="link2"/>
            <origin xyz="0.5 0 0" rpy="0 1.57 0"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.origin.xyz.x == pytest.approx(0.5)
        assert joint.origin.rpy.y == pytest.approx(1.57)

    def test_parse_joint_with_dynamics(self, parser):
        """Test parsing joint with dynamics."""
        xml = """
        <joint name="joint_dyn" type="revolute">
            <parent link="link1"/>
            <child link="link2"/>
            <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
            <dynamics damping="0.5" friction="0.1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.dynamics is not None
        assert joint.dynamics.damping == pytest.approx(0.5)
        assert joint.dynamics.friction == pytest.approx(0.1)

    def test_parse_joint_with_mimic(self, parser):
        """Test parsing joint with mimic."""
        xml = """
        <joint name="follower_joint" type="revolute">
            <parent link="link1"/>
            <child link="link2"/>
            <limit lower="-1" upper="1" effort="10" velocity="1"/>
            <mimic joint="leader_joint" multiplier="2.0" offset="0.1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.mimic is not None
        assert joint.mimic.joint == "leader_joint"
        assert joint.mimic.multiplier == pytest.approx(2.0)
        assert joint.mimic.offset == pytest.approx(0.1)

    def test_parse_floating_joint(self, parser):
        """Test parsing floating joint type."""
        xml = """
        <joint name="joint_float" type="floating">
            <parent link="link1"/>
            <child link="link2"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.type == JointType.FLOATING

    def test_parse_planar_joint(self, parser):
        """Test parsing planar joint type."""
        xml = """
        <joint name="joint_planar" type="planar">
            <parent link="link1"/>
            <child link="link2"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.type == JointType.PLANAR


class TestParseURDF:
    """Tests for parse_urdf function (integration tests)."""

    def test_parse_simple_robot(self, tmp_path: Path):
        """Test parsing simple robot with one link."""
        urdf_content = """<?xml version="1.0"?>
<robot name="simple_robot">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
        </visual>
    </link>
</robot>
"""
        urdf_file = tmp_path / "simple.urdf"
        urdf_file.write_text(urdf_content)

        robot = URDFParser().parse(urdf_file)

        assert robot.name == "simple_robot"
        assert len(robot.links) == 1
        assert robot.links[0].name == "base_link"

    def test_parse_robot_with_joint(self, tmp_path: Path):
        """Test parsing robot with links and joints."""
        urdf_content = """<?xml version="1.0"?>
<robot name="two_link_robot">
    <link name="link1">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.5"/>
            </geometry>
        </visual>
    </link>
    <link name="link2">
        <visual>
            <geometry>
                <box size="0.2 0.2 0.2"/>
            </geometry>
        </visual>
    </link>
    <joint name="joint1" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>
</robot>
"""
        urdf_file = tmp_path / "two_link.urdf"
        urdf_file.write_text(urdf_content)

        robot = URDFParser().parse(urdf_file)

        assert robot.name == "two_link_robot"
        assert len(robot.links) == 2
        assert len(robot.joints) == 1
        assert robot.joints[0].name == "joint1"
        assert robot.joints[0].type == JointType.REVOLUTE

    def test_parse_robot_with_global_materials(self, tmp_path: Path):
        """Test parsing robot with global material definitions."""
        urdf_content = """<?xml version="1.0"?>
<robot name="colored_robot">
    <material name="red">
        <color rgba="1 0 0 1"/>
    </material>
    <material name="blue">
        <color rgba="0 0 1 1"/>
    </material>
    <link name="link1">
        <visual>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
            <material name="red"/>
        </visual>
    </link>
    <link name="link2">
        <visual>
            <geometry>
                <sphere radius="0.5"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>
    <joint name="j1" type="fixed">
        <parent link="link1"/>
        <child link="link2"/>
    </joint>
</robot>
"""
        urdf_file = tmp_path / "colored.urdf"
        urdf_file.write_text(urdf_content)

        robot = URDFParser().parse(urdf_file)

        assert len(robot.links) == 2
        # Both links should reference the global materials
        assert robot.links[0].visuals[0].material is not None
        assert robot.links[1].visuals[0].material is not None

    def test_parse_file_not_found(self, tmp_path: Path):
        """Test that parsing nonexistent file raises FileNotFoundError."""
        nonexistent = tmp_path / "nonexistent.urdf"

        with pytest.raises(FileNotFoundError):
            URDFParser().parse(nonexistent)

    def test_parse_invalid_xml(self, tmp_path: Path):
        """Test that invalid XML raises ParseError."""
        urdf_file = tmp_path / "invalid.urdf"
        urdf_file.write_text("<robot><link></robot>")  # Malformed XML

        with pytest.raises(RobotParserError):
            URDFParser().parse(urdf_file)

    def test_parse_non_robot_root(self, tmp_path: Path):
        """Test that non-robot root element raises RobotModelError."""
        urdf_content = """<?xml version="1.0"?>
<notarobot name="invalid">
    <link name="link1"/>
</notarobot>
"""
        urdf_file = tmp_path / "invalid_root.urdf"
        urdf_file.write_text(urdf_content)

        with pytest.raises(RobotParserError, match="Root element must be <robot>"):
            URDFParser().parse(urdf_file)

    def test_parse_complex_robot(self, tmp_path: Path):
        """Test parsing complex robot with multiple features."""
        urdf_content = """<?xml version="1.0"?>
<robot name="complex_robot">
    <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
    </material>

    <link name="base_link">
        <visual>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <geometry>
                <box size="0.5 0.3 0.1"/>
            </geometry>
            <material name="gray"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.5 0.3 0.1"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>

    <link name="wheel_left">
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.02"/>
            </geometry>
            <material name="gray"/>
        </visual>
    </link>

    <link name="wheel_right">
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.02"/>
            </geometry>
            <material name="gray"/>
        </visual>
    </link>

    <joint name="wheel_left_joint" type="continuous">
        <parent link="base_link"/>
        <child link="wheel_left"/>
        <origin xyz="0.15 0.2 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="wheel_right_joint" type="continuous">
        <parent link="base_link"/>
        <child link="wheel_right"/>
        <origin xyz="0.15 -0.2 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>
</robot>
"""
        urdf_file = tmp_path / "complex.urdf"
        urdf_file.write_text(urdf_content)

        robot = URDFParser().parse(urdf_file)

        assert robot.name == "complex_robot"
        assert len(robot.links) == 3
        assert len(robot.joints) == 2

        # Check base link has all properties
        base_link = robot.links[0]
        assert base_link.visuals[0] is not None
        assert base_link.collisions[0] is not None
        assert base_link.inertial is not None
        assert base_link.inertial.mass == pytest.approx(1.0)

        # Check joints are continuous type
        assert all(j.type == JointType.CONTINUOUS for j in robot.joints)


class TestXACRODetection:
    """Tests for XACRO file detection."""

    def test_detect_xacro_namespace(self, tmp_path: Path):
        """Test detection of XACRO namespace."""
        xacro_content = """<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
        </visual>
    </link>
</robot>
"""
        xacro_file = tmp_path / "test.xacro"
        xacro_file.write_text(xacro_content)

        with pytest.raises(RobotParserError, match="XACRO file detected"):
            URDFParser().parse(xacro_file)

    def test_detect_xacro_properties(self, tmp_path: Path):
        """Test detection of XACRO property elements."""
        xacro_content = """<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="width" value="0.2" />
    <link name="base_link">
        <visual>
            <geometry>
                <box size="${width} ${width} 0.1"/>
            </geometry>
        </visual>
    </link>
</robot>
"""
        xacro_file = tmp_path / "test_props.xacro"
        xacro_file.write_text(xacro_content)

        with pytest.raises(RobotParserError, match="XACRO file detected"):
            URDFParser().parse(xacro_file)

    def test_detect_xacro_substitutions(self, tmp_path: Path):
        """Test detection of XACRO variable substitutions."""
        xacro_content = """<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="length" value="0.5" />
    <link name="base_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="${length}"/>
            </geometry>
        </visual>
    </link>
</robot>
"""
        xacro_file = tmp_path / "test_sub.xacro"
        xacro_file.write_text(xacro_content)

        with pytest.raises(RobotParserError, match="XACRO file detected"):
            URDFParser().parse(xacro_file)

    def test_detect_xacro_macros(self, tmp_path: Path):
        """Test detection of XACRO macro definitions."""
        xacro_content = """<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:macro name="wheel" params="prefix">
        <link name="${prefix}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="0.1" length="0.05"/>
                </geometry>
            </visual>
        </link>
    </xacro:macro>
</robot>
"""
        xacro_file = tmp_path / "test_macro.xacro"
        xacro_file.write_text(xacro_content)

        with pytest.raises(RobotParserError, match="XACRO file detected"):
            URDFParser().parse(xacro_file)

    def test_error_message_includes_conversion_instructions(self, tmp_path: Path):
        """Test that error message includes helpful conversion instructions."""
        xacro_content = """<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <link name="base_link"/>
</robot>
"""
        xacro_file = tmp_path / "robot.urdf.xacro"
        xacro_file.write_text(xacro_content)

        with pytest.raises(RobotParserError) as exc_info:
            URDFParser().parse(xacro_file)

        error_msg = str(exc_info.value)
        assert "convert" in error_msg.lower()
        assert "xacro" in error_msg.lower()
        assert "robot.urdf.xacro" in error_msg
        assert "robot.urdf" in error_msg  # Shows expected output filename

    def test_valid_urdf_not_detected_as_xacro(self, tmp_path: Path):
        """Test that valid URDF files are not incorrectly detected as XACRO."""
        urdf_content = """<?xml version="1.0"?>
<robot name="test_robot">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
        </visual>
    </link>
</robot>
"""
        urdf_file = tmp_path / "test.urdf"
        urdf_file.write_text(urdf_content)

        # Should not raise any error
        robot = URDFParser().parse(urdf_file)
        assert robot.name == "test_robot"

    def test_detect_package_substitution(self, tmp_path: Path):
        """Test detection of ROS package substitution syntax."""
        xacro_content = """<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <link name="base_link">
        <visual>
            <geometry>
                <mesh filename="$(find my_package)/meshes/base.stl"/>
            </geometry>
        </visual>
    </link>
</robot>
"""
        xacro_file = tmp_path / "test_package.xacro"
        xacro_file.write_text(xacro_content)

        with pytest.raises(RobotParserError, match="XACRO file detected"):
            URDFParser().parse(xacro_file)


class TestURDFParserErrorHandling:
    """Tests for URDF parser error handling and malformed XML."""

    def test_missing_robot_name(self, tmp_path: Path):
        """Test that missing robot name defaults to 'imported_robot'."""
        urdf_content = """<?xml version="1.0"?>
<robot>
    <link name="base_link"/>
</robot>
"""
        urdf_file = tmp_path / "no_name.urdf"
        urdf_file.write_text(urdf_content)

        # Parser uses filename "no_name" when name missing
        robot = URDFParser().parse(urdf_file)
        assert robot.name == "no_name"

    def test_malformed_xml(self, tmp_path: Path):
        """Test that malformed XML raises appropriate error."""
        urdf_content = """<?xml version="1.0"?>
<robot name="test">
    <link name="base"
</robot>
"""
        urdf_file = tmp_path / "malformed.urdf"
        urdf_file.write_text(urdf_content)

        with pytest.raises(RobotParserError):
            URDFParser().parse(urdf_file)

    def test_missing_joint_parent(self, tmp_path: Path, caplog):
        """Test that joint without parent link is skipped gracefully."""
        urdf_content = """<?xml version="1.0"?>
<robot name="test">
    <link name="link1"/>
    <link name="link2"/>
    <joint name="joint1" type="revolute">
        <child link="link2"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="10" velocity="1"/>
    </joint>
</robot>
"""
        urdf_file = tmp_path / "no_parent.urdf"
        urdf_file.write_text(urdf_content)

        # Joint validation skips with warning
        robot = URDFParser().parse(urdf_file)
        assert len(robot.joints) == 0
        assert "Skipping invalid joint 'joint1'" in caplog.text
        assert "Parent link name cannot be empty" in caplog.text

    def test_missing_joint_child(self, tmp_path: Path, caplog):
        """Test that joint without child link is skipped gracefully."""
        urdf_content = """<?xml version="1.0"?>
<robot name="test">
    <link name="link1"/>
    <link name="link2"/>
    <joint name="joint1" type="revolute">
        <parent link="link1"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="10" velocity="1"/>
    </joint>
</robot>
"""
        urdf_file = tmp_path / "no_child.urdf"
        urdf_file.write_text(urdf_content)

        # Joint validation skips with warning
        robot = URDFParser().parse(urdf_file)
        assert len(robot.joints) == 0
        assert "Skipping invalid joint 'joint1'" in caplog.text
        assert "Child link name cannot be empty" in caplog.text

    def test_invalid_geometry_values(self, tmp_path: Path):
        """Test that invalid geometry dimensions are handled gracefully."""
        urdf_content = """<?xml version="1.0"?>
<robot name="test">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="-1 2 3"/>
            </geometry>
        </visual>
    </link>
</robot>
"""
        urdf_file = tmp_path / "bad_geometry.urdf"
        urdf_file.write_text(urdf_content)

        # Parser is resilient - invalid geometry is skipped with warning
        robot = URDFParser().parse(urdf_file)
        assert robot.name == "test"
        assert len(robot.links) == 1
        # Visual with invalid geometry should be skipped
        assert len(robot.links[0].visuals) == 0

    def test_missing_required_joint_attributes(self, tmp_path: Path):
        """Test that joints without limits are handled gracefully (defaults used)."""
        urdf_content = """<?xml version="1.0"?>
<robot name="test">
    <link name="link1"/>
    <link name="link2"/>
    <joint name="joint1" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <axis xyz="0 0 1"/>
        <limit effort="10.0" velocity="1.0"/>
    </joint>
</robot>
"""
        urdf_file = tmp_path / "no_limits.urdf"
        urdf_file.write_text(urdf_content)

        # Revolute joint without limits should now succeed with defaults
        robot = URDFParser().parse(urdf_file)
        joint = robot.joints[0]
        assert joint.limits is not None
        # Should default to None (which implies 0 in physics/export logic usually)
        # or explicit 0 if we parsed it that way.
        # Looking at code: lower/upper become None if attribute missing.
        assert joint.limits.lower is None
        assert joint.limits.upper is None

    def test_massless_link_parsing(self, tmp_path: Path):
        """Test that a link with no inertial tag is parsed as inertial=None."""
        urdf_content = """<?xml version="1.0"?>
<robot name="massless_bot">
    <link name="root" />
    <link name="child">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
    </link>
    <joint name="fixed_base" type="fixed">
        <parent link="root"/>
        <child link="child"/>
    </joint>
</robot>
"""
        urdf_file = tmp_path / "massless.urdf"
        urdf_file.write_text(urdf_content)

        robot = URDFParser().parse(urdf_file)

        # Verify root has no inertial
        root = robot.get_link("root")
        assert root is not None
        assert root.inertial is None
        assert root.mass == 0.0

        # Verify child has inertial
        child = robot.get_link("child")
        assert child is not None
        assert child.inertial is not None
        assert child.mass == 1.0
