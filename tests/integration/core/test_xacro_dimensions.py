"""Test XACRO dimension extraction feature."""

from __future__ import annotations

import xml.etree.ElementTree as ET

from linkforge_core import XACROGenerator
from linkforge_core.models import Color, Joint, JointType, Link, Material, Robot, Visual
from linkforge_core.models.geometry import Box, Cylinder, Sphere, Transform, Vector3


def test_extract_dimensions_cylinders() -> None:
    """Test that repeated cylinder dimensions are extracted as properties."""
    # Create robot with 4 identical wheels
    base = Link(name="base_link")
    wheels = []
    joints = []

    for i, name in enumerate(["fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"]):
        wheel = Link(
            name=name, initial_visuals=[Visual(geometry=Cylinder(radius=0.05, length=0.02))]
        )
        wheels.append(wheel)
        joint = Joint(
            name=f"{name}_joint",
            type=JointType.CONTINUOUS,
            parent="base_link",
            child=name,
            axis=Vector3(1.0, 0.0, 0.0),
            origin=Transform(xyz=Vector3(i, 0, 0)),
        )
        joints.append(joint)

    robot = Robot(name="test_robot", initial_links=[base] + wheels, initial_joints=joints)

    # Generate XACRO with extract_dimensions=True
    gen = XACROGenerator(extract_dimensions=True, extract_materials=False)
    xml_str = gen.generate(robot, validate=False)
    root = ET.fromstring(xml_str)

    # Verify properties were created
    properties = root.findall(".//{http://www.ros.org/wiki/xacro}property")
    prop_names = [p.get("name") for p in properties]

    assert "wheel_radius" in prop_names, "Expected wheel_radius property"
    assert "wheel_length" in prop_names, "Expected wheel_length property"

    # Verify property values
    radius_prop = next(p for p in properties if p.get("name") == "wheel_radius")
    length_prop = next(p for p in properties if p.get("name") == "wheel_length")

    assert radius_prop.get("value") == "0.05"
    assert length_prop.get("value") == "0.02"

    # Verify dimensions are substituted in geometry
    cylinders = root.findall(".//cylinder")
    assert len(cylinders) == 4

    for cyl in cylinders:
        assert cyl.get("radius") == "${wheel_radius}", "Radius should use property"
        assert cyl.get("length") == "${wheel_length}", "Length should use property"


def test_extract_dimensions_boxes() -> None:
    """Test that repeated box dimensions are extracted as properties."""
    # Create robot with 2 identical legs
    base = Link(name="base_link")
    left_leg = Link(
        name="left_leg", initial_visuals=[Visual(geometry=Box(size=Vector3(0.1, 0.1, 0.5)))]
    )
    right_leg = Link(
        name="right_leg", initial_visuals=[Visual(geometry=Box(size=Vector3(0.1, 0.1, 0.5)))]
    )

    j1 = Joint("j1", JointType.FIXED, "base_link", "left_leg")
    j2 = Joint("j2", JointType.FIXED, "base_link", "right_leg")

    robot = Robot(
        name="test_robot", initial_links=[base, left_leg, right_leg], initial_joints=[j1, j2]
    )

    # Generate XACRO
    gen = XACROGenerator(extract_dimensions=True, extract_materials=False)
    xml_str = gen.generate(robot, validate=False)
    root = ET.fromstring(xml_str)

    # Verify properties
    properties = root.findall(".//{http://www.ros.org/wiki/xacro}property")
    prop_names = [p.get("name") for p in properties]

    assert "leg_width" in prop_names
    assert "leg_depth" in prop_names
    assert "leg_height" in prop_names

    # Verify substitution
    boxes = root.findall(".//box")
    assert len(boxes) == 2

    for box in boxes:
        assert box.get("size") == "${leg_width} ${leg_depth} ${leg_height}"


def test_extract_dimensions_spheres() -> None:
    """Test that repeated sphere dimensions are extracted as properties."""
    # Create robot with 3 identical balls
    base = Link(name="base_link")
    balls = []
    joints = []

    for i in range(3):
        ball = Link(name=f"ball{i}", initial_visuals=[Visual(geometry=Sphere(radius=0.03))])
        balls.append(ball)
        joint = Joint(f"j{i}", JointType.FIXED, "base_link", f"ball{i}")
        joints.append(joint)

    robot = Robot(name="test_robot", initial_links=[base] + balls, initial_joints=joints)

    # Generate XACRO
    gen = XACROGenerator(extract_dimensions=True, extract_materials=False)
    xml_str = gen.generate(robot, validate=False)
    root = ET.fromstring(xml_str)

    # Verify property
    properties = root.findall(".//{http://www.ros.org/wiki/xacro}property")
    prop_names = [p.get("name") for p in properties]

    assert "ball_radius" in prop_names

    # Verify substitution
    spheres = root.findall(".//sphere")
    assert len(spheres) == 3

    for sphere in spheres:
        assert sphere.get("radius") == "${ball_radius}"


def test_no_extract_unique_dimensions() -> None:
    """Test that unique dimensions are NOT extracted as properties."""
    # Create robot with unique dimensions
    base = Link(name="base_link")
    wheel1 = Link(
        name="wheel1", initial_visuals=[Visual(geometry=Cylinder(radius=0.05, length=0.02))]
    )
    wheel2 = Link(
        name="wheel2", initial_visuals=[Visual(geometry=Cylinder(radius=0.07, length=0.03))]
    )

    j1 = Joint("j1", JointType.FIXED, "base_link", "wheel1")
    j2 = Joint("j2", JointType.FIXED, "base_link", "wheel2")

    robot = Robot(name="test_robot", initial_links=[base, wheel1, wheel2], initial_joints=[j1, j2])

    # Generate XACRO
    gen = XACROGenerator(extract_dimensions=True, extract_materials=False)
    xml_str = gen.generate(robot, validate=False)
    root = ET.fromstring(xml_str)

    # Verify NO properties created (all dimensions are unique)
    properties = root.findall(".//{http://www.ros.org/wiki/xacro}property")
    assert len(properties) == 0, "No properties should be created for unique dimensions"

    # Verify hardcoded values
    cylinders = root.findall(".//cylinder")
    assert len(cylinders) == 2

    assert cylinders[0].get("radius") == "0.05"
    assert cylinders[0].get("length") == "0.02"
    assert cylinders[1].get("radius") == "0.07"
    assert cylinders[1].get("length") == "0.03"


def test_extract_dimensions_mixed_geometries() -> None:
    """Test dimension extraction with multiple geometry types."""
    # Create robot with cylinders + boxes
    base = Link(name="base_link")

    # 2 identical wheels
    wheel1 = Link(
        name="wheel1", initial_visuals=[Visual(geometry=Cylinder(radius=0.05, length=0.02))]
    )
    wheel2 = Link(
        name="wheel2", initial_visuals=[Visual(geometry=Cylinder(radius=0.05, length=0.02))]
    )

    # 2 identical legs
    leg1 = Link(name="leg1", initial_visuals=[Visual(geometry=Box(size=Vector3(0.1, 0.1, 0.5)))])
    leg2 = Link(name="leg2", initial_visuals=[Visual(geometry=Box(size=Vector3(0.1, 0.1, 0.5)))])

    joints = [
        Joint("j1", JointType.FIXED, "base_link", "wheel1"),
        Joint("j2", JointType.FIXED, "base_link", "wheel2"),
        Joint("j3", JointType.FIXED, "base_link", "leg1"),
        Joint("j4", JointType.FIXED, "base_link", "leg2"),
    ]

    robot = Robot(
        name="test_robot", initial_links=[base, wheel1, wheel2, leg1, leg2], initial_joints=joints
    )

    # Generate XACRO
    gen = XACROGenerator(extract_dimensions=True, extract_materials=False)
    xml_str = gen.generate(robot, validate=False)
    root = ET.fromstring(xml_str)

    # Verify properties for both geometry types
    properties = root.findall(".//{http://www.ros.org/wiki/xacro}property")
    prop_names = [p.get("name") for p in properties]

    # Cylinder properties
    assert "wheel_radius" in prop_names
    assert "wheel_length" in prop_names

    # Box properties
    assert "leg_width" in prop_names
    assert "leg_depth" in prop_names
    assert "leg_height" in prop_names


def test_dimension_floating_point_tolerance() -> None:
    """Test that similar dimensions are grouped with tolerance."""
    # Create wheels with slightly different dimensions (within tolerance)
    base = Link(name="base_link")
    wheel1 = Link(
        name="wheel1", initial_visuals=[Visual(geometry=Cylinder(radius=0.05, length=0.02))]
    )
    wheel2 = Link(
        name="wheel2", initial_visuals=[Visual(geometry=Cylinder(radius=0.050001, length=0.02))]
    )  # Slightly different

    j1 = Joint("j1", JointType.FIXED, "base_link", "wheel1")
    j2 = Joint("j2", JointType.FIXED, "base_link", "wheel2")

    robot = Robot(name="test_robot", initial_links=[base, wheel1, wheel2], initial_joints=[j1, j2])

    # Generate XACRO
    gen = XACROGenerator(extract_dimensions=True, extract_materials=False)
    xml_str = gen.generate(robot, validate=False)
    root = ET.fromstring(xml_str)

    # Verify properties created (dimensions are within tolerance)
    properties = root.findall(".//{http://www.ros.org/wiki/xacro}property")
    prop_names = [p.get("name") for p in properties]

    assert "wheel_radius" in prop_names, "Similar dimensions should be grouped"
    assert "wheel_length" in prop_names


def test_dimension_property_naming() -> None:
    """Test that property names are descriptive."""
    # Test various naming scenarios
    base = Link(name="base_link")

    # Scenario 1: Common suffix (fl_wheel, fr_wheel → wheel)
    fl_wheel = Link(
        name="fl_wheel", initial_visuals=[Visual(geometry=Cylinder(radius=0.05, length=0.02))]
    )
    fr_wheel = Link(
        name="fr_wheel", initial_visuals=[Visual(geometry=Cylinder(radius=0.05, length=0.02))]
    )

    joints = [
        Joint("j1", JointType.FIXED, "base_link", "fl_wheel"),
        Joint("j2", JointType.FIXED, "base_link", "fr_wheel"),
    ]

    robot = Robot(
        name="test_robot", initial_links=[base, fl_wheel, fr_wheel], initial_joints=joints
    )

    gen = XACROGenerator(extract_dimensions=True, extract_materials=False)
    xml_str = gen.generate(robot, validate=False)
    root = ET.fromstring(xml_str)

    properties = root.findall(".//{http://www.ros.org/wiki/xacro}property")
    prop_names = [p.get("name") for p in properties]

    # Should extract common suffix "wheel"
    assert "wheel_radius" in prop_names
    assert "wheel_length" in prop_names


def test_extract_dimensions_disabled() -> None:
    """Test that dimensions are NOT extracted when extract_dimensions=False."""
    # Create robot with repeated dimensions
    base = Link(name="base_link")
    wheel1 = Link(
        name="wheel1", initial_visuals=[Visual(geometry=Cylinder(radius=0.05, length=0.02))]
    )
    wheel2 = Link(
        name="wheel2", initial_visuals=[Visual(geometry=Cylinder(radius=0.05, length=0.02))]
    )

    j1 = Joint("j1", JointType.FIXED, "base_link", "wheel1")
    j2 = Joint("j2", JointType.FIXED, "base_link", "wheel2")

    robot = Robot(name="test_robot", initial_links=[base, wheel1, wheel2], initial_joints=[j1, j2])

    # Generate XACRO with extract_dimensions=False
    gen = XACROGenerator(extract_dimensions=False, extract_materials=False)
    xml_str = gen.generate(robot, validate=False)
    root = ET.fromstring(xml_str)

    # Verify NO properties created
    properties = root.findall(".//{http://www.ros.org/wiki/xacro}property")
    assert len(properties) == 0

    # Verify hardcoded values
    cylinders = root.findall(".//cylinder")
    for cyl in cylinders:
        assert cyl.get("radius") == "0.05"
        assert cyl.get("length") == "0.02"


def test_extract_dimensions_with_materials() -> None:
    """Test that dimension and material extraction work together."""
    # Create robot with repeated dimensions AND materials
    base = Link(name="base_link")
    mat = Material(name="black", color=Color(0, 0, 0, 1))

    wheel1 = Link(
        name="wheel1",
        initial_visuals=[Visual(geometry=Cylinder(radius=0.05, length=0.02), material=mat)],
    )
    wheel2 = Link(
        name="wheel2",
        initial_visuals=[Visual(geometry=Cylinder(radius=0.05, length=0.02), material=mat)],
    )

    j1 = Joint("j1", JointType.FIXED, "base_link", "wheel1")
    j2 = Joint("j2", JointType.FIXED, "base_link", "wheel2")

    robot = Robot(name="test_robot", initial_links=[base, wheel1, wheel2], initial_joints=[j1, j2])

    # Generate XACRO with both features enabled
    gen = XACROGenerator(extract_dimensions=True, extract_materials=True)
    xml_str = gen.generate(robot, validate=False)
    root = ET.fromstring(xml_str)

    # Verify both material and dimension properties
    properties = root.findall(".//{http://www.ros.org/wiki/xacro}property")
    prop_names = [p.get("name") for p in properties]

    # Material property
    assert "black" in prop_names

    # Dimension properties
    assert "wheel_radius" in prop_names
    assert "wheel_length" in prop_names

    # Verify substitution in geometry
    cylinders = root.findall(".//cylinder")
    for cyl in cylinders:
        assert cyl.get("radius") == "${wheel_radius}"
        assert cyl.get("length") == "${wheel_length}"

    # Verify material substitution
    materials = root.findall(".//material")
    for mat_elem in materials:
        if mat_elem.get("name") == "black":
            color = mat_elem.find("color")
            if color is not None:
                assert color.get("rgba") == "${black}"
