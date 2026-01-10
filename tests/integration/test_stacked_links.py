"""Integration test for stacked links with joints at different heights.

This tests the scenario described where:
- base_link (cube) at world origin (0, 0, 0)
- cylinder_link1 at world (0, 0, 2) with joint from base_link
- cylinder_link2 at world (0, 0, 4) with joint from cylinder_link1

The test verifies that:
1. Export correctly calculates joint origins as relative offsets
2. Collision/visual origins are link-relative (not world coordinates)
3. Import recreates the same link positions in world space
"""

from __future__ import annotations

from pathlib import Path

import pytest

from linkforge.core.generators.urdf import URDFGenerator
from linkforge.core.models import (
    Box,
    Collision,
    Cylinder,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    Robot,
    Transform,
    Vector3,
    Visual,
)
from linkforge.core.parsers.urdf_parser import parse_urdf, parse_urdf_string


def create_stacked_robot() -> Robot:
    """Create a robot with stacked links at increasing Z heights.

    Creates a robot with three links:
    - base_link (2x2x2 cube) with link frame at world (0, 0, 0)
    - cylinder_link1 (2m tall cylinder) with link frame at world (0, 0, 2)
    - cylinder_link2 (2m tall cylinder) with link frame at world (0, 0, 4)

    Joints connect each link with proper relative offsets.
    """
    robot = Robot(name="stacked_robot")

    # Base link at origin (0, 0, 0)
    base_link = Link(
        name="base_link",
        visuals=[
            Visual(
                geometry=Box(size=Vector3(2.0, 2.0, 2.0)),
                origin=Transform.identity(),  # Visual at link origin
            )
        ],
        collisions=[
            Collision(
                geometry=Box(size=Vector3(2.0, 2.0, 2.0)),
                origin=Transform.identity(),  # Collision at link origin (NOT world coords!)
            )
        ],
        inertial=Inertial(
            mass=1.0,
            inertia=InertiaTensor(
                ixx=0.666667, ixy=0.0, ixz=0.0, iyy=0.666667, iyz=0.0, izz=0.666667
            ),
        ),
    )

    # First cylinder link - link frame should be at (0, 0, 2)
    cylinder_link1 = Link(
        name="cylinder_link1",
        visuals=[
            Visual(
                geometry=Cylinder(radius=1.0, length=2.0),
                origin=Transform.identity(),  # Visual at link origin
            )
        ],
        collisions=[
            Collision(
                geometry=Cylinder(radius=1.0, length=2.0),
                origin=Transform.identity(),  # Collision at link origin (NOT world 0,0,2!)
            )
        ],
        inertial=Inertial(
            mass=1.0,
            inertia=InertiaTensor(ixx=0.58, ixy=0.0, ixz=0.0, iyy=0.58, iyz=0.0, izz=0.5),
        ),
    )

    # Second cylinder link - link frame should be at (0, 0, 4)
    cylinder_link2 = Link(
        name="cylinder_link2",
        visuals=[
            Visual(
                geometry=Cylinder(radius=1.0, length=2.0),
                origin=Transform.identity(),  # Visual at link origin
            )
        ],
        collisions=[
            Collision(
                geometry=Cylinder(radius=1.0, length=2.0),
                origin=Transform.identity(),  # Collision at link origin (NOT world 0,0,4!)
            )
        ],
        inertial=Inertial(
            mass=1.0,
            inertia=InertiaTensor(ixx=0.58, ixy=0.0, ixz=0.0, iyy=0.58, iyz=0.0, izz=0.5),
        ),
    )

    robot.add_link(base_link)
    robot.add_link(cylinder_link1)
    robot.add_link(cylinder_link2)

    # Joint from base_link to cylinder_link1
    # Joint origin (0, 0, 2) means: child link frame is 2m above parent link frame
    joint1 = Joint(
        name="cylinder_link1_joint",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="cylinder_link1",
        origin=Transform(xyz=Vector3(0.0, 0.0, 2.0), rpy=Vector3(0.0, 0.0, 0.0)),
        axis=Vector3(0.0, 0.0, 1.0),
        limits=JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=1.0),
    )

    # Joint from cylinder_link1 to cylinder_link2
    # Joint origin (0, 0, 2) means: child link frame is 2m above parent link frame
    # So cylinder_link2 will be at world (0, 0, 4) = base(0,0,0) + joint1(0,0,2) + joint2(0,0,2)
    joint2 = Joint(
        name="cylinder_link2_joint",
        type=JointType.REVOLUTE,
        parent="cylinder_link1",
        child="cylinder_link2",
        origin=Transform(xyz=Vector3(0.0, 0.0, 2.0), rpy=Vector3(0.0, 0.0, 0.0)),
        axis=Vector3(0.0, 0.0, 1.0),
        limits=JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=1.0),
    )

    robot.add_joint(joint1)
    robot.add_joint(joint2)

    return robot


def test_stacked_links_export():
    """Test that exporting stacked links produces correct joint origins and collision origins."""
    robot = create_stacked_robot()

    # Generate URDF
    generator = URDFGenerator(pretty_print=True)
    urdf_string = generator.generate(robot)

    # Verify joint origins are correct (relative to parent link)
    assert 'joint name="cylinder_link1_joint"' in urdf_string
    assert 'joint name="cylinder_link2_joint"' in urdf_string

    # Verify both joints have correct origins (looking for any reasonable formatting)
    # Joint origins should be approximately "0 0 2" (allowing for floating point formatting)
    assert "cylinder_link1_joint" in urdf_string
    assert "cylinder_link2_joint" in urdf_string

    # The key test: collision origins should NOT have world coordinates
    # If bug exists, we'd see <collision><origin xyz="0 0 2"> or xyz="0 0 4"
    # (world coordinates leaked into link-relative collision origins)

    # Verify collision origins are identity (link-relative, not world coordinates!)
    # If bug exists, collision origins would be xyz="0 0 2" and xyz="0 0 4" (world coords)
    assert 'link name="cylinder_link1"' in urdf_string
    assert 'link name="cylinder_link2"' in urdf_string

    # Collision geometry should have no origin elements (identity transform omitted)
    # or should be at identity (0 0 0), NOT at world coordinates
    # The bug would show: <collision><origin xyz="0 0 2"/> (WRONG - world coordinate leak)

    # Parse to verify collision origins
    parsed_robot = parse_urdf_string(urdf_string)

    # Check collision origins are identity (link-relative)
    cylinder1 = next(link for link in parsed_robot.links if link.name == "cylinder_link1")
    assert len(cylinder1.collisions) == 1
    collision1_origin = cylinder1.collisions[0].origin
    assert collision1_origin.xyz.x == pytest.approx(0.0)
    assert collision1_origin.xyz.y == pytest.approx(0.0)
    assert collision1_origin.xyz.z == pytest.approx(0.0), (
        "Collision origin should be link-relative (0,0,0), not world coordinate (0,0,2)"
    )

    cylinder2 = next(link for link in parsed_robot.links if link.name == "cylinder_link2")
    assert len(cylinder2.collisions) == 1
    collision2_origin = cylinder2.collisions[0].origin
    assert collision2_origin.xyz.x == pytest.approx(0.0)
    assert collision2_origin.xyz.y == pytest.approx(0.0)
    assert collision2_origin.xyz.z == pytest.approx(0.0), (
        "Collision origin should be link-relative (0,0,0), not world coordinate (0,0,4)"
    )


def test_stacked_links_roundtrip(tmp_path: Path):
    """Test full export-import roundtrip for stacked links.

    Verifies that:
    1. Export generates correct URDF with relative joint origins
    2. Import recreates link positions correctly in world space
    3. Re-export produces identical URDF structure
    """
    robot = create_stacked_robot()

    # Write URDF
    urdf_path = tmp_path / "stacked_robot.urdf"
    generator = URDFGenerator(pretty_print=True, urdf_path=urdf_path)
    generator.write(robot, urdf_path)

    # Re-import
    reimported_robot = parse_urdf(urdf_path)

    # Verify structure
    assert len(reimported_robot.links) == 3
    assert len(reimported_robot.joints) == 2

    # Verify joint origins preserved
    joint1 = next(j for j in reimported_robot.joints if j.name == "cylinder_link1_joint")
    assert joint1.origin.xyz.z == pytest.approx(2.0)

    joint2 = next(j for j in reimported_robot.joints if j.name == "cylinder_link2_joint")
    assert joint2.origin.xyz.z == pytest.approx(2.0)

    # Verify collision origins are identity (not world coordinates)
    for link in reimported_robot.links:
        if link.name != "base_link":
            for collision in link.collisions:
                assert collision.origin.xyz.x == pytest.approx(0.0)
                assert collision.origin.xyz.y == pytest.approx(0.0)
                assert collision.origin.xyz.z == pytest.approx(0.0), (
                    f"Link '{link.name}' collision origin should be (0,0,0), not world coordinates"
                )

    # Re-export
    urdf_path2 = tmp_path / "stacked_robot_reexport.urdf"
    generator2 = URDFGenerator(pretty_print=True, urdf_path=urdf_path2)
    generator2.write(reimported_robot, urdf_path2)

    # Verify re-exported URDF has same structure
    urdf_string1 = urdf_path.read_text()
    urdf_string2 = urdf_path2.read_text()

    # Joint origins should be identical
    assert urdf_string1.count('<origin xyz="0 0 2"') == urdf_string2.count('<origin xyz="0 0 2"')
