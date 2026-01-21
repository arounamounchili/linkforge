"""Test XACRO macro generation."""

from __future__ import annotations

import xml.etree.ElementTree as ET

from linkforge.core import XACROGenerator
from linkforge.core.models import Color, Joint, JointType, Link, Material, Robot, Visual
from linkforge.core.models.geometry import Box, Cylinder, Transform, Vector3


def test_macro_generation_wheels():
    """Test that 4 identical wheels are converted to a macro."""
    # Create base link
    base_link = Link(name="base_link")

    # Create 4 identical wheels
    wheel_geom = Cylinder(radius=0.1, length=0.05)
    wheel_mat = Material(name="black", color=Color(0, 0, 0, 1))

    links = [base_link]
    joints = []

    for i, name in enumerate(["front_left", "front_right", "rear_left", "rear_right"]):
        wheel_link = Link(
            name=f"{name}_wheel", visuals=[Visual(geometry=wheel_geom, material=wheel_mat)]
        )
        links.append(wheel_link)

        # Joint connecting to base
        joint = Joint(
            name=f"{name}_wheel_joint",
            type=JointType.CONTINUOUS,
            parent="base_link",
            child=f"{name}_wheel",
            origin=Transform(xyz=Vector3(i, 0, 0)),  # Different positions
        )
        joints.append(joint)

    robot = Robot(name="car_robot", links=links, joints=joints)

    # Generate XACRO
    gen = XACROGenerator(generate_macros=True)
    xml_str = gen.generate(robot, validate=False)

    # Verify
    root = ET.fromstring(xml_str)

    # 1. Check for macro definition
    macros = root.findall(".//{http://www.ros.org/wiki/xacro}macro")
    assert len(macros) == 1, f"Expected 1 macro, found {len(macros)}"
    macro = macros[0]
    assert "cyl" in macro.get("name")
    assert macro.get("params") == "name parent xyz rpy"

    # 2. Check for macro calls
    # The macro name will be something like "cyl_0.100_0.050_black_macro"
    macro_name = macro.get("name")
    calls = root.findall(f".//{{http://www.ros.org/wiki/xacro}}{macro_name}")
    assert len(calls) == 4, f"Expected 4 macro calls, found {len(calls)}"

    # 3. Check call parameters
    for call in calls:
        assert call.get("parent") == "base_link"
        assert "wheel" in call.get("name")
        assert call.get("xyz") is not None

    # 4. Check that original links are GONE (except base_link)
    links_in_xml = root.findall("link")
    link_names = [link.get("name") for link in links_in_xml]
    assert "base_link" in link_names
    assert "front_left_wheel" not in link_names, "Original link should be replaced by macro call"


def test_macro_generation_mixed():
    """Test that unique links are NOT converted to macros."""
    # Base + 2 identical wheels + 1 unique arm
    base = Link(name="base")

    wheel_geom = Cylinder(radius=0.1, length=0.05)
    wheel1 = Link(name="w1", visuals=[Visual(geometry=wheel_geom)])
    wheel2 = Link(name="w2", visuals=[Visual(geometry=wheel_geom)])

    arm_geom = Box(size=Vector3(1, 0.1, 0.1))
    arm = Link(name="arm", visuals=[Visual(geometry=arm_geom)])

    j1 = Joint("j1", JointType.FIXED, "base", "w1")
    j2 = Joint("j2", JointType.FIXED, "base", "w2")
    j3 = Joint("j3", JointType.FIXED, "base", "arm")

    robot = Robot("test", links=[base, wheel1, wheel2, arm], joints=[j1, j2, j3])

    gen = XACROGenerator(generate_macros=True)
    xml_str = gen.generate(robot, validate=False)
    root = ET.fromstring(xml_str)

    # Should have 1 macro for wheels
    macros = root.findall(".//{http://www.ros.org/wiki/xacro}macro")
    assert len(macros) == 1

    # Should have 2 calls for wheels
    macro_name = macros[0].get("name")
    calls = root.findall(f".//{{http://www.ros.org/wiki/xacro}}{macro_name}")
    assert len(calls) == 2

    # Arm should still be a regular link
    links = [link.get("name") for link in root.findall("link")]
    assert "arm" in links
