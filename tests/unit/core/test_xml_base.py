"""Tests for the base XML generator."""

import xml.etree.ElementTree as ET
from pathlib import Path

from linkforge_core.generators.xml_base import RobotXMLGenerator
from linkforge_core.models.geometry import Box, Cylinder, Mesh, Sphere, Transform, Vector3
from linkforge_core.models.link import Inertial, InertiaTensor
from linkforge_core.models.robot import Robot


class MockXMLGenerator(RobotXMLGenerator):
    """Minimal implementation of abstract RobotXMLGenerator for testing base functionality."""

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    def generate(self, robot: Robot, **kwargs) -> str:
        return "<robot></robot>"


def test_format_value() -> None:
    """Test value formatting hook."""
    gen = MockXMLGenerator()
    assert gen._format_value(1.0) == "1"
    assert gen._format_value(1.12345678) == "1.123457"  # Assuming math_utils.format_float behavior
    assert gen._format_value(42) == "42"
    assert gen._format_value("test") == "test"


def test_format_vector() -> None:
    """Test vector formatting hook."""
    gen = MockXMLGenerator()
    assert gen._format_vector(1.0, 2.0, 3.0) == "1 2 3"
    assert gen._format_vector(1.12345678, 0.0, -1.0) == "1.123457 0 -1"


def test_add_origin_element() -> None:
    """Test origin element generation."""
    gen = MockXMLGenerator()
    parent = ET.Element("parent")

    # Test identity transform (should not add an element)
    gen._add_origin_element(parent, Transform.identity())
    assert parent.find("origin") is None

    # Test custom tag
    transform = Transform(xyz=Vector3(1, 2, 3), rpy=Vector3(0.1, 0.2, 0.3))
    gen._add_origin_element(parent, transform, tag="custom_origin")

    elem = parent.find("custom_origin")
    assert elem is not None
    assert elem.get("xyz") == "1 2 3"
    assert elem.get("rpy") == "0.1 0.2 0.3"


def test_add_inertial_element() -> None:
    """Test inertial element generation."""
    gen = MockXMLGenerator()
    parent = ET.Element("parent")

    inertial = Inertial(
        mass=5.5,
        origin=Transform(xyz=Vector3(0, 0, 1)),
        inertia=InertiaTensor(
            ixx=1.0,
            iyy=2.0,
            izz=3.0,
            ixy=0.1,
            ixz=0.2,
            iyz=0.3,
        ),
    )

    gen._add_inertial_element(parent, inertial)
    elem = parent.find("inertial")
    assert elem is not None

    # Check mass
    mass_elem = elem.find("mass")
    assert mass_elem is not None
    assert mass_elem.get("value") == "5.5"

    # Check origin
    origin_elem = elem.find("origin")
    assert origin_elem is not None
    assert origin_elem.get("xyz") == "0 0 1"

    # Check inertia
    inertia_elem = elem.find("inertia")
    assert inertia_elem is not None
    assert inertia_elem.get("ixx") == "1"
    assert inertia_elem.get("ixy") == "0.1"
    assert inertia_elem.get("ixz") == "0.2"
    assert inertia_elem.get("iyy") == "2"
    assert inertia_elem.get("iyz") == "0.3"
    assert inertia_elem.get("izz") == "3"


def test_add_geometry_element_box() -> None:
    """Test geometry element generation for Box."""
    gen = MockXMLGenerator()
    parent = ET.Element("parent")
    box = Box(size=Vector3(1, 2, 3))

    gen._add_geometry_element(box, parent)
    elem = parent.find("geometry")
    assert elem is not None

    box_elem = elem.find("box")
    assert box_elem is not None
    assert box_elem.get("size") == "1 2 3"


def test_add_geometry_element_cylinder() -> None:
    """Test geometry element generation for Cylinder."""
    gen = MockXMLGenerator()
    parent = ET.Element("parent")
    cylinder = Cylinder(radius=0.5, length=2.0)

    gen._add_geometry_element(cylinder, parent)
    elem = parent.find("geometry")
    assert elem is not None

    cyl_elem = elem.find("cylinder")
    assert cyl_elem is not None
    assert cyl_elem.get("radius") == "0.5"
    assert cyl_elem.get("length") == "2"


def test_add_geometry_element_sphere() -> None:
    """Test geometry element generation for Sphere."""
    gen = MockXMLGenerator()
    parent = ET.Element("parent")
    sphere = Sphere(radius=1.5)

    gen._add_geometry_element(sphere, parent)
    elem = parent.find("geometry")
    assert elem is not None

    sphere_elem = elem.find("sphere")
    assert sphere_elem is not None
    assert sphere_elem.get("radius") == "1.5"


def test_add_geometry_element_mesh() -> None:
    """Test geometry element generation for Mesh."""
    # Test with output_path set (relative path)
    gen = MockXMLGenerator(output_path=Path("/tmp/robot/robot.urdf"))
    parent = ET.Element("parent")
    mesh = Mesh(resource="package://my_robot/meshes/part.stl", scale=Vector3(2, 2, 2))

    gen._add_geometry_element(mesh, parent)
    elem = parent.find("geometry")
    assert elem is not None

    mesh_elem = elem.find("mesh")
    assert mesh_elem is not None
    assert mesh_elem.get("filename") == "package://my_robot/meshes/part.stl"
    assert mesh_elem.get("scale") == "2 2 2"


def test_add_geometry_element_unsupported() -> None:
    """Test that unsupported geometry type creates an empty container as fallback."""
    gen = MockXMLGenerator()
    parent = ET.Element("parent")

    class UnknownGeometry:
        pass

    gen._add_geometry_element(UnknownGeometry(), parent)
    elem = parent.find("geometry")
    assert elem is not None
    assert len(elem) == 0  # No specific geometry child created


def test_geometry_parsing_unsupported_mesh_warning() -> None:
    """Verify that malformed mesh geometry triggers a warning during base XML parsing."""
    from unittest.mock import patch

    from linkforge_core.parsers.xml_base import RobotXMLParser

    class MockParser(RobotXMLParser):
        def parse(self, *args, **kwargs):
            pass

    parser = MockParser()
    elem = ET.Element("geometry")
    # Add a mesh with invalid scale to trigger the float conversion error
    ET.SubElement(elem, "mesh", filename="model.stl", scale="invalid_scale_string")

    with patch("linkforge_core.parsers.xml_base.logger") as mock_logger:
        res = parser._parse_geometry_element(elem)
        assert res is None
        mock_logger.warning.assert_called()
