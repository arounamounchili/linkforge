"""Test for bug fix: base_link at non-origin positions should preserve child link relative positions.

This test verifies the fix for the issue where:
- User creates base_link at world position (5, 0, 2)
- User creates child links at various positions
- Export to URDF moves base_link to origin (0, 0, 0) - correct per URDF spec
- Re-import should place child links at their correct relative positions

Before fix: child links stayed at original world positions (wrong)
After fix: child links shift to maintain relative positions to base_link (correct)
"""

from __future__ import annotations

import math

from linkforge_core import URDFGenerator
from linkforge_core.models import (
    Box,
    Inertial,
    InertiaTensor,
    Joint,
    JointType,
    Link,
    Robot,
    Transform,
    Vector3,
    Visual,
)
from linkforge_core.parsers.urdf_parser import URDFParser


def test_base_link_at_non_origin_preserves_relative_positions() -> None:
    """Test that child links maintain relative positions when base_link is not at origin."""
    # Create a robot where base_link is at (5, 0, 2) and child links are offset
    robot = Robot(name="offset_test_robot")

    # Simulate base_link at world position (5, 0, 2)
    # In URDF export, the visual origin will capture this offset
    base_visual_origin = Transform(xyz=Vector3(5.0, 0.0, 2.0), rpy=Vector3(0.0, 0.0, 0.0))
    base_link = Link(
        name="base_link",
        initial_visuals=[
            Visual(geometry=Box(size=Vector3(1.0, 1.0, 1.0)), origin=base_visual_origin)
        ],
        inertial=Inertial(mass=1.0, inertia=InertiaTensor(0.1, 0, 0, 0.1, 0, 0.1)),
    )

    # Simulate link1 at world position (5, 0, 4) - which is 2 units above base_link
    # Joint origin should be (0, 0, 2) relative to base_link
    link1 = Link(
        name="link1",
        initial_visuals=[Visual(geometry=Box(size=Vector3(0.5, 0.5, 0.5)))],
        inertial=Inertial(mass=0.5, inertia=InertiaTensor(0.05, 0, 0, 0.05, 0, 0.05)),
    )

    # Simulate link2 at world position (5, 3, 2) - which is 3 units to the side of base_link
    # Joint origin should be (0, 3, 0) relative to base_link
    link2 = Link(
        name="link2",
        initial_visuals=[Visual(geometry=Box(size=Vector3(0.5, 0.5, 0.5)))],
        inertial=Inertial(mass=0.5, inertia=InertiaTensor(0.05, 0, 0, 0.05, 0, 0.05)),
    )

    robot.add_link(base_link)
    robot.add_link(link1)
    robot.add_link(link2)

    # Joint origins calculated as: child_world_pos - base_world_pos
    # link1: (5,0,4) - (5,0,2) = (0,0,2)
    # link2: (5,3,2) - (5,0,2) = (0,3,0)
    joint1 = Joint(
        name="joint1",
        type=JointType.FIXED,
        parent="base_link",
        child="link1",
        origin=Transform(xyz=Vector3(0.0, 0.0, 2.0), rpy=Vector3(0.0, 0.0, 0.0)),
    )

    joint2 = Joint(
        name="joint2",
        type=JointType.FIXED,
        parent="base_link",
        child="link2",
        origin=Transform(xyz=Vector3(0.0, 3.0, 0.0), rpy=Vector3(0.0, 0.0, 0.0)),
    )

    robot.add_joint(joint1)
    robot.add_joint(joint2)

    # Export to URDF string
    generator = URDFGenerator(pretty_print=False)
    urdf_string = generator.generate(robot)

    # Re-parse URDF
    robot2 = URDFParser().parse_string(urdf_string)

    # Verify structure preserved
    assert len(robot2.links) == 3
    assert len(robot2.joints) == 2

    # Verify joint origins are preserved correctly
    # These should reflect the RELATIVE positions between links
    joint1_reimported = next(j for j in robot2.joints if j.name == "joint1")
    joint2_reimported = next(j for j in robot2.joints if j.name == "joint2")

    # Joint1 should be at (0, 0, 2) - 2 units above base_link
    assert math.isclose(joint1_reimported.origin.xyz.x, 0.0, abs_tol=1e-6)
    assert math.isclose(joint1_reimported.origin.xyz.y, 0.0, abs_tol=1e-6)
    assert math.isclose(joint1_reimported.origin.xyz.z, 2.0, abs_tol=1e-6)

    # Joint2 should be at (0, 3, 0) - 3 units to the side of base_link
    assert math.isclose(joint2_reimported.origin.xyz.x, 0.0, abs_tol=1e-6)
    assert math.isclose(joint2_reimported.origin.xyz.y, 3.0, abs_tol=1e-6)
    assert math.isclose(joint2_reimported.origin.xyz.z, 0.0, abs_tol=1e-6)

    # Verify base_link visual origin is preserved (captures world offset)
    base_link_reimported = next(link for link in robot2.links if link.name == "base_link")
    assert len(base_link_reimported.visuals) == 1
    visual_origin = base_link_reimported.visuals[0].origin

    # Base visual should be at (5, 0, 2) to account for Blender world position
    assert math.isclose(visual_origin.xyz.x, 5.0, abs_tol=1e-6)
    assert math.isclose(visual_origin.xyz.y, 0.0, abs_tol=1e-6)
    assert math.isclose(visual_origin.xyz.z, 2.0, abs_tol=1e-6)


def test_base_link_with_rotation_preserves_child_orientations() -> None:
    """Test that child links maintain relative orientations when base_link is rotated."""
    robot = Robot(name="rotated_base_test")

    # Base link rotated 45 degrees around Z axis (pi/4 radians)
    rotation_z = math.pi / 4
    base_visual_origin = Transform(xyz=Vector3(0.0, 0.0, 0.0), rpy=Vector3(0.0, 0.0, rotation_z))

    base_link = Link(
        name="base_link",
        initial_visuals=[
            Visual(geometry=Box(size=Vector3(1.0, 1.0, 1.0)), origin=base_visual_origin)
        ],
        inertial=Inertial(mass=1.0, inertia=InertiaTensor(0.1, 0, 0, 0.1, 0, 0.1)),
    )

    child_link = Link(
        name="child_link",
        initial_visuals=[Visual(geometry=Box(size=Vector3(0.5, 0.5, 0.5)))],
        inertial=Inertial(mass=0.5, inertia=InertiaTensor(0.05, 0, 0, 0.05, 0, 0.05)),
    )

    robot.add_link(base_link)
    robot.add_link(child_link)

    # Joint with no rotation (child aligned with parent)
    joint = Joint(
        name="joint1",
        type=JointType.FIXED,
        parent="base_link",
        child="child_link",
        origin=Transform(xyz=Vector3(1.0, 0.0, 0.0), rpy=Vector3(0.0, 0.0, 0.0)),
    )
    robot.add_joint(joint)

    # Export and re-import
    generator = URDFGenerator(pretty_print=False)
    urdf_string = generator.generate(robot)
    robot2 = URDFParser().parse_string(urdf_string)

    # Verify base link rotation is preserved
    base_link2 = next(link for link in robot2.links if link.name == "base_link")
    assert len(base_link2.visuals) == 1
    visual_rpy = base_link2.visuals[0].origin.rpy
    assert math.isclose(visual_rpy.z, rotation_z, abs_tol=1e-6)

    # Verify joint origin is preserved
    joint2 = next(j for j in robot2.joints if j.name == "joint1")
    assert math.isclose(joint2.origin.xyz.x, 1.0, abs_tol=1e-6)
    assert math.isclose(joint2.origin.rpy.x, 0.0, abs_tol=1e-6)
    assert math.isclose(joint2.origin.rpy.y, 0.0, abs_tol=1e-6)
    assert math.isclose(joint2.origin.rpy.z, 0.0, abs_tol=1e-6)
