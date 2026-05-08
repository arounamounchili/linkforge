"""Perfect roundtrip test - Verify exact preservation of all URDF elements.

This test performs a deep comparison between original and round-tripped URDFs
to identify any data loss or transformation issues.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from linkforge_core.models import (
    Box,
    Collision,
    Color,
    Cylinder,
    Joint,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointType,
    Link,
    Material,
    Robot,
    Sphere,
    Transform,
    Vector3,
    Visual,
)
from linkforge_core.parsers.urdf_parser import URDFParser

from tests.core_test_utils import assert_robots_equal, perform_urdf_roundtrip


def test_perfect_roundtrip_comprehensive_robot(examples_dir: Path) -> None:
    """Test that comprehensive test robot survives perfect roundtrip."""
    # Load original
    original_path = examples_dir / "urdf" / "roundtrip_test_robot.urdf"
    robot1 = URDFParser().parse(original_path)

    # Export and re-import
    robot2 = perform_urdf_roundtrip(robot1)

    # Assert equality
    assert_robots_equal(robot1, robot2)


def test_geometry_types_roundtrip() -> None:
    """Test that all geometry types survive roundtrip."""
    robot = Robot(name="geometry_test")

    # Add base link
    robot.add_link(Link(name="base"))

    # Box
    robot.add_link(
        Link(
            name="box_link",
            initial_visuals=[
                Visual(
                    geometry=Box(size=Vector3(1.0, 2.0, 3.0)),
                    origin=Transform(xyz=Vector3(0.1, 0.2, 0.3)),
                )
            ],
        )
    )
    robot.add_joint(
        Joint(name="base_to_box", type=JointType.FIXED, parent="base", child="box_link")
    )

    # Cylinder
    robot.add_link(
        Link(
            name="cylinder_link",
            initial_visuals=[
                Visual(
                    geometry=Cylinder(radius=0.5, length=2.0),
                    origin=Transform(xyz=Vector3(0.0, 0.0, 1.0)),
                )
            ],
        )
    )
    robot.add_joint(
        Joint(name="base_to_cylinder", type=JointType.FIXED, parent="base", child="cylinder_link")
    )

    # Sphere
    robot.add_link(
        Link(
            name="sphere_link",
            initial_visuals=[
                Visual(
                    geometry=Sphere(radius=0.75),
                    origin=Transform(xyz=Vector3(1.0, 1.0, 1.0)),
                )
            ],
        )
    )
    robot.add_joint(
        Joint(name="base_to_sphere", type=JointType.FIXED, parent="base", child="sphere_link")
    )

    # Roundtrip and compare
    robot2 = perform_urdf_roundtrip(robot)
    assert_robots_equal(robot, robot2)


def test_joint_types_roundtrip() -> None:
    """Test that all joint types survive roundtrip."""
    robot = Robot(name="joint_test")
    robot.add_link(Link(name="base"))
    robot.add_link(Link(name="revolute_link"))
    robot.add_link(Link(name="prismatic_link"))
    robot.add_link(Link(name="continuous_link"))
    robot.add_link(Link(name="fixed_link"))

    # Revolute
    robot.add_joint(
        Joint(
            name="revolute_joint",
            type=JointType.REVOLUTE,
            parent="base",
            child="revolute_link",
            origin=Transform(xyz=Vector3(1.0, 0.0, 0.0), rpy=Vector3(0.0, 0.0, 1.57)),
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=2.0),
            dynamics=JointDynamics(damping=0.5, friction=0.1),
        )
    )

    # Prismatic
    robot.add_joint(
        Joint(
            name="prismatic_joint",
            type=JointType.PRISMATIC,
            parent="base",
            child="prismatic_link",
            origin=Transform(xyz=Vector3(0.0, 1.0, 0.0)),
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=0.0, upper=1.0, effort=5.0, velocity=1.0),
        )
    )

    # Continuous
    robot.add_joint(
        Joint(
            name="continuous_joint",
            type=JointType.CONTINUOUS,
            parent="base",
            child="continuous_link",
            origin=Transform(xyz=Vector3(0.0, 0.0, 1.0)),
            axis=Vector3(1, 0, 0),
            dynamics=JointDynamics(damping=0.2, friction=0.05),
        )
    )

    # Fixed
    robot.add_joint(
        Joint(
            name="fixed_joint",
            type=JointType.FIXED,
            parent="base",
            child="fixed_link",
            origin=Transform(xyz=Vector3(-1.0, 0.0, 0.0)),
        )
    )

    # Roundtrip and compare
    robot2 = perform_urdf_roundtrip(robot)
    assert_robots_equal(robot, robot2)


def test_mimic_joint_roundtrip() -> None:
    """Test that mimic joints survive roundtrip."""
    robot = Robot(name="mimic_test")
    robot.add_link(Link(name="base"))
    robot.add_link(Link(name="master_link"))
    robot.add_link(Link(name="follower_link"))

    robot.add_joint(
        Joint(
            name="master_joint",
            type=JointType.PRISMATIC,
            parent="base",
            child="master_link",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=0.0, upper=0.1, effort=1.0, velocity=0.5),
        )
    )

    robot.add_joint(
        Joint(
            name="follower_joint",
            type=JointType.PRISMATIC,
            parent="base",
            child="follower_link",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-0.1, upper=0.0, effort=1.0, velocity=0.5),
            mimic=JointMimic(joint="master_joint", multiplier=-1.0, offset=0.0),
        )
    )

    # Roundtrip and compare
    robot2 = perform_urdf_roundtrip(robot)
    assert_robots_equal(robot, robot2)

    # Specifically verify mimic
    follower = next(j for j in robot2.joints if j.name == "follower_joint")
    assert follower.mimic is not None
    assert follower.mimic.joint == "master_joint"
    assert follower.mimic.multiplier == -1.0
    assert follower.mimic.offset == 0.0


def test_material_preservation_roundtrip() -> None:
    """Test that materials with colors survive roundtrip."""
    robot = Robot(name="material_test")

    red_material = Material(name="red", color=Color(1.0, 0.0, 0.0, 1.0))
    blue_material = Material(name="blue", color=Color(0.0, 0.0, 1.0, 0.5))

    robot.add_link(Link(name="base"))

    robot.add_link(
        Link(
            name="red_link",
            initial_visuals=[Visual(geometry=Box(size=Vector3(1, 1, 1)), material=red_material)],
        )
    )
    robot.add_joint(
        Joint(name="base_to_red", type=JointType.FIXED, parent="base", child="red_link")
    )

    robot.add_link(
        Link(
            name="blue_link",
            initial_visuals=[Visual(geometry=Sphere(radius=0.5), material=blue_material)],
        )
    )
    robot.add_joint(
        Joint(name="base_to_blue", type=JointType.FIXED, parent="base", child="blue_link")
    )

    # Roundtrip and compare
    robot2 = perform_urdf_roundtrip(robot)
    assert_robots_equal(robot, robot2)


def test_collision_geometry_roundtrip() -> None:
    """Test that collision geometries survive roundtrip."""
    robot = Robot(name="collision_test")

    robot.add_link(Link(name="base"))

    robot.add_link(
        Link(
            name="test_link",
            initial_visuals=[Visual(geometry=Box(size=Vector3(1, 1, 1)))],
            initial_collisions=[
                Collision(
                    geometry=Box(size=Vector3(1.1, 1.1, 1.1)),
                    origin=Transform(xyz=Vector3(0, 0, 0.05)),
                )
            ],
        )
    )
    robot.add_joint(
        Joint(name="base_to_test", type=JointType.FIXED, parent="base", child="test_link")
    )

    # Roundtrip and compare
    robot2 = perform_urdf_roundtrip(robot)
    assert_robots_equal(robot, robot2)


if __name__ == "__main__":
    import pytest

    pytest.main([__file__, "-v", "-s"])
