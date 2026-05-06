from linkforge_core.composer import RobotBuilder, box, cylinder
from linkforge_core.models.joint import JointType


def test_full_robot_build_and_export():
    """Integration test: Build a complex robot and verify its components."""
    builder = RobotBuilder("complex_bot")

    # 1. Setup materials
    builder.material("red", color=(1, 0, 0, 1))

    # 2. Setup control
    builder.ros2_control("arm_control", "fake_hw/RobotSystem")

    # 3. Build tree
    (
        builder.link("base")
        .visual(box(0.5, 0.5, 0.2), material="red")
        .collision()  # Clones visual
        .mass(5.0)
        .child("arm_1")
        .revolute(axis=(0, 0, 1), limits=(-3.14, 3.14))
        .visual(cylinder(0.05, 0.4), xyz=(0, 0, 0.2))
        .mass(1.0)
        .transmission(actuator="actuator_1", reduction=50.0)
        .ros2_control(["position"], ["position", "velocity"])
        .child("hand")
        .fixed(xyz=(0, 0, 0.4))
        .visual(box(0.1, 0.1, 0.1))
        .camera("palm_cam")
        .commit()
    )

    # 3. Add Semantic info
    builder.semantic.group("arm_group", links=["base", "arm_1"], joints=["base_to_arm_1"])
    builder.semantic.end_effector("my_hand", group="arm_group", parent_link="hand")
    builder.semantic.virtual_joint("world_fix", child_link="base")

    robot = builder.build()

    # 4. Verify IR Integrity
    assert robot.name == "complex_bot"
    assert len(robot.links) == 3
    assert len(robot.joints) == 2
    assert len(robot.sensors) == 1
    assert len(robot._ros2_controls) == 1
    assert len(robot.semantic.groups) == 1

    # 5. Verify Physics
    arm = robot.link("arm_1")
    assert arm.inertial is not None
    assert arm.inertial.mass == 1.0
    # Cylinder inertia should be non-zero
    assert arm.inertial.inertia.ixx > 0

    # 6. Verify Export (Pseudo-integration)
    # Since we are in core, we test if the model is ready for export
    assert robot.joint("base_to_arm_1").type == JointType.REVOLUTE
    assert robot.joint("arm_1_to_hand").type == JointType.FIXED
