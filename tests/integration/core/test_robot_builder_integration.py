"""RobotBuilder integration tests.

This module verifies complex robot building scenarios, including stacked links,
non-origin base links, and multi-visual/collision configurations.
"""

from __future__ import annotations

import pytest
from linkforge_core.composer.robot_builder import RobotBuilder
from linkforge_core.models import (
    Box,
    Collision,
    Joint,
    JointType,
    Link,
    Transform,
    Vector3,
    Visual,
)


def test_stacked_links_building() -> None:
    """Test building a robot with many stacked links."""
    builder = RobotBuilder(name="stacked_robot")
    builder.robot.add_link(Link(name="base_link"))

    for i in range(1, 11):
        parent = f"link_{i - 1}" if i > 1 else "base_link"
        child = f"link_{i}"
        builder.robot.add_link(Link(name=child))
        builder.robot.add_joint(
            Joint(
                name=f"joint_{i}",
                type=JointType.FIXED,
                parent=parent,
                child=child,
                origin=Transform(xyz=Vector3(0, 0, 0.1)),
            )
        )

    robot = builder.build()
    assert len(robot.links) == 11
    assert len(robot.joints) == 10


def test_non_origin_base_link() -> None:
    """Test building a robot where the base link has an offset origin."""
    builder = RobotBuilder(name="offset_base")
    builder.robot.add_link(
        Link(
            name="base_link",
            initial_visuals=[
                Visual(
                    geometry=Box(size=Vector3(1, 1, 1)), origin=Transform(xyz=Vector3(10, 10, 10))
                )
            ],
        )
    )
    robot = builder.build()
    assert robot.link("base_link").visuals[0].origin.xyz.x == 10.0


def test_multi_visual_collision_building() -> None:
    """Test building a link with multiple visual and collision elements."""
    builder = RobotBuilder(name="multi_element_robot")
    builder.robot.add_link(Link(name="base_link"))

    # Add complex link
    link = Link(name="complex_link")
    for i in range(3):
        link.add_visual(Visual(geometry=Box(size=Vector3(i + 1, i + 1, i + 1))))
        link.add_collision(Collision(geometry=Box(size=Vector3(i + 1, i + 1, i + 1))))

    builder.robot.add_link(link)
    builder.robot.add_joint(
        Joint(
            name="j1",
            type=JointType.FIXED,
            parent="base_link",
            child="complex_link",
        )
    )

    robot = builder.build()
    complex_link = robot.link("complex_link")
    assert len(complex_link.visuals) == 3
    assert len(complex_link.collisions) == 3


def test_robot_builder_high_level_api() -> None:
    """Test high-level RobotBuilder API for rapid prototyping."""
    # The RobotBuilder.link() API is the intended way to use it
    builder = RobotBuilder(name="proto")
    (
        builder.link("base")
        .visual(Box(size=Vector3(1, 1, 1)))
        .commit()
        .link("arm", parent="base")
        .visual(Box(size=Vector3(0.1, 0.1, 1.0)))
        .revolute(axis=(0, 0, 1), limits=(-1.57, 1.57), xyz=(0, 0, 0.5))
        .commit()
    )
    robot = builder.build()
    assert robot.name == "proto"
    assert len(robot.links) == 2
    assert len(robot.joints) == 1


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
