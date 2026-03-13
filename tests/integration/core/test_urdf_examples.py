"""Integration tests for example URDF files."""

from pathlib import Path

from linkforge_core.parsers.urdf_parser import URDFParser


def test_mobile_robot_structure(examples_dir: Path) -> None:
    """Test mobile_robot.urdf has correct structure."""
    robot = URDFParser().parse(examples_dir / "urdf" / "mobile_robot.urdf")

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


def test_diff_drive_robot_structure(examples_dir: Path) -> None:
    """Test diff_drive_robot.urdf has correct structure and advanced features."""
    robot = URDFParser().parse(examples_dir / "urdf" / "diff_drive_robot.urdf")

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


def test_roundtrip_test_robot_structure(examples_dir: Path) -> None:
    """Test roundtrip_test_robot.urdf has correct structure and all features."""
    robot = URDFParser().parse(examples_dir / "urdf" / "roundtrip_test_robot.urdf")

    assert robot.name == "comprehensive_test_robot"
    assert len(robot.links) == 15
    assert len(robot.joints) == 14
    assert len(robot.transmissions) == 0
    assert len(robot.sensors) == 3
    assert len(robot.ros2_controls) == 1
    assert len(robot.gazebo_elements) == 1

    # Check root link
    all_children = {j.child for j in robot.joints}
    root_links = [link.name for link in robot.links if link.name not in all_children]
    assert root_links == ["base_link"]

    # Check joint types
    from linkforge_core.models.joint import JointType

    joint_types = {j.type for j in robot.joints}
    assert JointType.FIXED in joint_types
    assert JointType.REVOLUTE in joint_types
    assert JointType.CONTINUOUS in joint_types
    assert JointType.PRISMATIC in joint_types
    assert JointType.PLANAR in joint_types
    assert JointType.FLOATING in joint_types


def test_roundtrip_robot_has_all_joint_types(examples_dir: Path) -> None:
    """Verify roundtrip_test_robot.urdf includes ALL 6 URDF joint types."""
    robot = URDFParser().parse(examples_dir / "urdf" / "roundtrip_test_robot.urdf")

    from linkforge_core.models.joint import JointType

    joint_types = {j.type for j in robot.joints}

    assert JointType.FIXED in joint_types
    assert JointType.REVOLUTE in joint_types
    assert JointType.CONTINUOUS in joint_types
    assert JointType.PRISMATIC in joint_types
    assert JointType.PLANAR in joint_types
    assert JointType.FLOATING in joint_types


def test_all_examples_have_inertia(examples_dir: Path) -> None:
    """Verify all links in all examples have inertial properties."""
    examples = [
        "mobile_robot.urdf",
        "diff_drive_robot.urdf",
        "roundtrip_test_robot.urdf",
        "quadruped_robot.urdf",
    ]

    for example_file in examples:
        robot = URDFParser().parse(examples_dir / "urdf" / example_file)
        for link in robot.links:
            assert link.inertial is not None
            assert link.inertial.mass > 0


def test_all_examples_parse_without_errors(examples_dir: Path) -> None:
    """Ensure all example files can be parsed without exceptions."""
    examples = [
        "mobile_robot.urdf",
        "diff_drive_robot.urdf",
        "roundtrip_test_robot.urdf",
        "quadruped_robot.urdf",
    ]

    for example_file in examples:
        robot = URDFParser().parse(examples_dir / "urdf" / example_file)
        assert robot is not None
        assert robot.name
        assert len(robot.links) > 0
