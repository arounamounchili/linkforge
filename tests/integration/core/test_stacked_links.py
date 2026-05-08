"""Integration test for stacked links with joints at different heights."""

from __future__ import annotations

import pytest
from linkforge_core.models import (
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

from tests.core_test_utils import assert_robots_equal, perform_urdf_roundtrip


def create_stacked_robot() -> Robot:
    """Create a robot with stacked links at increasing Z heights."""
    robot = Robot(name="stacked_robot")

    # Base link at origin (0, 0, 0)
    base_link = Link(
        name="base_link",
        initial_visuals=[Visual(geometry=Box(size=Vector3(2, 2, 2)), origin=Transform.identity())],
        initial_collisions=[
            Collision(geometry=Box(size=Vector3(2, 2, 2)), origin=Transform.identity())
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
        initial_visuals=[
            Visual(geometry=Cylinder(radius=1, length=2), origin=Transform.identity())
        ],
        initial_collisions=[
            Collision(geometry=Cylinder(radius=1, length=2), origin=Transform.identity())
        ],
        inertial=Inertial(
            mass=1.0,
            inertia=InertiaTensor(ixx=0.58, ixy=0.0, ixz=0.0, iyy=0.58, iyz=0.0, izz=0.5),
        ),
    )

    # Second cylinder link - link frame should be at (0, 0, 4)
    cylinder_link2 = Link(
        name="cylinder_link2",
        initial_visuals=[
            Visual(geometry=Cylinder(radius=1, length=2), origin=Transform.identity())
        ],
        initial_collisions=[
            Collision(geometry=Cylinder(radius=1, length=2), origin=Transform.identity())
        ],
        inertial=Inertial(
            mass=1.0,
            inertia=InertiaTensor(ixx=0.58, ixy=0.0, ixz=0.0, iyy=0.58, iyz=0.0, izz=0.5),
        ),
    )

    robot.add_link(base_link)
    robot.add_link(cylinder_link1)
    robot.add_link(cylinder_link2)

    robot.add_joint(
        Joint(
            name="cylinder_link1_joint",
            type=JointType.REVOLUTE,
            parent="base_link",
            child="cylinder_link1",
            origin=Transform(xyz=Vector3(0.0, 0.0, 2.0)),
            axis=Vector3(0.0, 0.0, 1.0),
            limits=JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=1.0),
        )
    )

    robot.add_joint(
        Joint(
            name="cylinder_link2_joint",
            type=JointType.REVOLUTE,
            parent="cylinder_link1",
            child="cylinder_link2",
            origin=Transform(xyz=Vector3(0.0, 0.0, 2.0)),
            axis=Vector3(0.0, 0.0, 1.0),
            limits=JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=1.0),
        )
    )

    return robot


def test_stacked_links_roundtrip() -> None:
    """Test full export-import roundtrip for stacked links."""
    robot = create_stacked_robot()

    # Roundtrip
    reimported_robot = perform_urdf_roundtrip(robot)

    # Verify comprehensive equality
    assert_robots_equal(robot, reimported_robot)

    # Verify specific coordinate leak protection
    for link in reimported_robot.links:
        if link.name != "base_link":
            for collision in link.collisions:
                assert collision.origin.xyz.z == pytest.approx(0.0), (
                    f"Link '{link.name}' collision origin should be (0,0,0), not world coordinates"
                )
