"""Integration tests for example URDF files."""

from pathlib import Path

from linkforge.core.parsers.urdf_parser import parse_urdf


def get_examples_dir() -> Path:
    """Get path to examples directory."""
    return Path(__file__).parent.parent.parent / "examples"


def test_mobile_robot_structure():
    """Test mobile_robot.urdf has correct structure."""
    robot = parse_urdf(get_examples_dir() / "mobile_robot.urdf")

    assert robot.name == "mobile_robot"
    assert len(robot.links) == 6
    assert len(robot.joints) == 5

    # Check root link
    all_children = {j.child for j in robot.joints}
    root_links = [link.name for link in robot.links if link.name not in all_children]
    assert root_links == ["base_link"]

    # Verify all wheels are connected to base
    wheel_joints = [j for j in robot.joints if "wheel" in j.name]
    assert len(wheel_joints) == 4
    for joint in wheel_joints:
        assert joint.parent == "base_link"


def test_diff_drive_robot_structure():
    """Test diff_drive_robot.urdf has correct structure and advanced features."""
    robot = parse_urdf(get_examples_dir() / "diff_drive_robot.urdf")

    assert robot.name == "diff_drive_robot"
    assert len(robot.links) == 4
    assert len(robot.joints) == 3
    assert len(robot.transmissions) == 0
    assert len(robot.sensors) == 1
    assert len(robot.gazebo_elements) == 1
    assert len(robot.ros2_controls) == 1

    # Check root link
    all_children = {j.child for j in robot.joints}
    root_links = [link.name for link in robot.links if link.name not in all_children]
    assert root_links == ["base_link"]

    # Check ros2_control joints
    rc = robot.ros2_controls[0]
    rc_joints = {j.name for j in rc.joints}
    assert "left_wheel_joint" in rc_joints
    assert "right_wheel_joint" in rc_joints


def test_roundtrip_test_robot_structure():
    """Test roundtrip_test_robot.urdf has correct structure and all features."""
    robot = parse_urdf(get_examples_dir() / "roundtrip_test_robot.urdf")

    assert robot.name == "comprehensive_test_robot"
    assert len(robot.links) == 15  # Added planar_platform and floating_sensor
    assert len(robot.joints) == 14  # Added planar_joint and floating_joint
    assert len(robot.transmissions) == 0
    assert len(robot.sensors) == 3
    assert len(robot.ros2_controls) == 1
    assert len(robot.gazebo_elements) == 1

    # Check root link
    all_children = {j.child for j in robot.joints}
    root_links = [link.name for link in robot.links if link.name not in all_children]
    assert root_links == ["base_link"]

    # Check joint types are present (now includes ALL 6 types)
    from linkforge.core.models.joint import JointType

    joint_types = {j.type for j in robot.joints}
    assert JointType.FIXED in joint_types
    assert JointType.REVOLUTE in joint_types
    assert JointType.CONTINUOUS in joint_types
    assert JointType.PRISMATIC in joint_types
    assert JointType.PLANAR in joint_types  # NEW
    assert JointType.FLOATING in joint_types  # NEW

    # Check arm kinematic chain
    joint_map = {j.name: (j.parent, j.child) for j in robot.joints}
    assert joint_map["arm_base_joint"] == ("base_link", "arm_base")
    assert joint_map["shoulder_joint"] == ("arm_base", "upper_arm")
    assert joint_map["elbow_joint"] == ("upper_arm", "forearm")
    assert joint_map["wrist_joint"] == ("forearm", "gripper_base")

    # Check gripper with mimic joint
    assert joint_map["left_finger_joint"] == ("gripper_base", "left_finger")
    assert joint_map["right_finger_joint"] == ("gripper_base", "right_finger")

    right_finger = [j for j in robot.joints if j.name == "right_finger_joint"][0]
    assert right_finger.mimic is not None
    assert right_finger.mimic.joint == "left_finger_joint"
    assert right_finger.mimic.multiplier == -1.0

    # Check ros2_control
    rc = robot.ros2_controls[0]
    rc_joints = {j.name for j in rc.joints}
    assert "arm_base_joint" in rc_joints
    assert "shoulder_joint" in rc_joints

    # Verify interfaces
    arm_joint = next(j for j in rc.joints if j.name == "arm_base_joint")
    assert "effort" in arm_joint.command_interfaces
    assert "position" in arm_joint.state_interfaces

    # Check sensors
    sensor_types = {s.type.value for s in robot.sensors}
    assert "camera" in sensor_types
    assert "lidar" in sensor_types
    assert "imu" in sensor_types

    # Check geometry types
    from linkforge.core.models.geometry import Box, Cylinder, Sphere

    geom_types = set()
    for link in robot.links:
        for visual in link.visuals:
            if visual.geometry:
                geom_types.add(type(visual.geometry))
    assert Box in geom_types
    assert Cylinder in geom_types
    assert Sphere in geom_types

    # Check visual geometry with origin offset
    upper_arm = [link for link in robot.links if link.name == "upper_arm"][0]
    assert len(upper_arm.visuals) == 1
    visual = upper_arm.visuals[0]
    assert visual.origin.xyz.z == 0.2  # Visual offset from link frame


def test_roundtrip_robot_has_all_joint_types():
    """Verify roundtrip_test_robot.urdf includes ALL 6 URDF joint types."""
    robot = parse_urdf(get_examples_dir() / "roundtrip_test_robot.urdf")

    from linkforge.core.models.joint import JointType

    joint_types = {j.type for j in robot.joints}

    # Verify all 6 joint types are present
    assert JointType.FIXED in joint_types, "Missing FIXED joint"
    assert JointType.REVOLUTE in joint_types, "Missing REVOLUTE joint"
    assert JointType.CONTINUOUS in joint_types, "Missing CONTINUOUS joint"
    assert JointType.PRISMATIC in joint_types, "Missing PRISMATIC joint"
    assert JointType.PLANAR in joint_types, "Missing PLANAR joint"
    assert JointType.FLOATING in joint_types, "Missing FLOATING joint"


def test_all_examples_have_inertia():
    """Verify all links in all examples have inertial properties."""
    examples = [
        "mobile_robot.urdf",
        "diff_drive_robot.urdf",
        "roundtrip_test_robot.urdf",
        "quadruped_robot.urdf",
    ]

    for example_file in examples:
        robot = parse_urdf(get_examples_dir() / example_file)
        for link in robot.links:
            assert link.inertial is not None, f"Link {link.name} in {example_file} missing inertial"
            assert link.inertial.mass > 0, f"Link {link.name} has zero or negative mass"


def test_all_examples_parse_without_errors():
    """Ensure all example files can be parsed without exceptions."""
    examples = [
        "mobile_robot.urdf",
        "diff_drive_robot.urdf",
        "roundtrip_test_robot.urdf",
        "quadruped_robot.urdf",
    ]

    for example_file in examples:
        robot = parse_urdf(get_examples_dir() / example_file)
        assert robot is not None
        assert robot.name
        assert len(robot.links) > 0
