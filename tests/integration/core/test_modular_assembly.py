"""Modular assembly and namespacing integration tests.

This module verifies that multiple robots can be merged into a single
unified model using the 'attach' API, ensuring that namespacing (prefixes)
correctly prevents collisions and maintains kinematic integrity.
"""

from __future__ import annotations

from linkforge.core import Box, RobotBuilder, RobotValidator, Vector3


def create_arm_component() -> RobotBuilder:
    """Create a simple 2-DOF arm sub-assembly."""
    builder = RobotBuilder("arm")
    (
        builder.link("arm_base")
        .mass(1.0)
        .visual(Box(size=Vector3(0.1, 0.1, 0.1)))
        .commit()
        .link("link_1", parent="arm_base")
        .mass(1.0)
        .revolute(axis=(0, 0, 1), limits=(-1.57, 1.57), xyz=(0, 0, 0.1))
        .visual(Box(size=Vector3(0.05, 0.05, 0.2)))
        .commit()
        .link("wrist", parent="link_1")
        .mass(1.0)
        .revolute(axis=(0, 1, 0), limits=(-1.57, 1.57), xyz=(0, 0, 0.2))
        .visual(Box(size=Vector3(0.05, 0.05, 0.05)))
        .commit()
    )
    return builder


def test_modular_assembly_with_prefixes() -> None:
    """Verify merging two identical arms with unique prefixes."""
    # 1. Create the main robot (base)
    main = RobotBuilder("modular_robot")
    main.link("chassis").mass(1.0).visual(Box(size=Vector3(0.5, 0.5, 0.2))).commit()

    # 2. Create the arm component
    arm_comp = create_arm_component()

    # 3. Attach Left Arm
    main.attach(
        arm_comp,
        at_link="chassis",
        prefix="left_",
        xyz=(0.2, 0, 0.1),
        joint_name="chassis_to_left_arm",
    )

    # 4. Attach Right Arm (identical component, different prefix)
    main.attach(
        arm_comp,
        at_link="chassis",
        prefix="right_",
        xyz=(-0.2, 0, 0.1),
        joint_name="chassis_to_right_arm",
    )

    robot = main.build()

    # --- Verification ---

    # Total links: 1 (chassis) + 3 (left arm) + 3 (right arm) = 7
    assert len(robot.links) == 7

    # Verify Left Arm Namespacing
    assert robot.has_link("left_arm_base")
    assert robot.has_link("left_link_1")
    assert robot.has_link("left_wrist")

    # Verify Right Arm Namespacing
    assert robot.has_link("right_arm_base")
    assert robot.has_link("right_link_1")
    assert robot.has_link("right_wrist")

    # Verify Joints
    assert robot.has_joint("chassis_to_left_arm")
    assert robot.has_joint("chassis_to_right_arm")
    assert robot.has_joint("left_arm_base_to_link_1")
    assert robot.has_joint("right_arm_base_to_link_1")

    # Verify Kinematic Connectivity
    # left_wrist should have left_link_1 as parent in the final graph
    left_wrist_joint = robot.joint("left_link_1_to_wrist")
    assert left_wrist_joint.parent == "left_link_1"
    assert left_wrist_joint.child == "left_wrist"

    # Verify Robot Structure is Valid (no cycles, single root)

    result = RobotValidator().validate(robot)
    assert result.is_valid, f"Validation failed: {result.errors}"
    assert robot.root_link.name == "chassis"


def test_modular_assembly_collision_disabling() -> None:
    """Verify that collision disabling works across modular interfaces."""
    main = RobotBuilder("mobile_base")
    main.link("base_link").commit()

    wheel = RobotBuilder("wheel")
    wheel.link("rim").commit()

    # Attach wheel and disable collision with base_link
    main.attach(
        wheel, at_link="base_link", prefix="front_left_", disable_collision=True, reason="Adjacent"
    )

    robot = main.build()

    # Check SRDF (Semantic Description) for disabled collisions
    found = False
    for pair in robot.semantic.disabled_collisions:
        if (pair.link1 == "base_link" and pair.link2 == "front_left_rim") or (
            pair.link1 == "front_left_rim" and pair.link2 == "base_link"
        ):
            found = True
            assert pair.reason == "Adjacent"
            break

    assert found, "Collision pair was not found in SRDF"
