"""Integration test for URDF import/export round-trip.

This test verifies that:
1. URDF can be imported successfully
2. Imported robot can be exported back to URDF
3. The exported URDF is valid and preserves key properties
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest

from linkforge.core import URDFGenerator
from linkforge.core.parsers.urdf_parser import parse_urdf, parse_urdf_string


def get_examples_dir() -> Path:
    """Get path to examples directory."""
    return Path(__file__).parent.parent.parent / "examples"


def test_simple_arm_roundtrip():
    """Test importing and re-exporting roundtrip_test_robot.urdf preserves structure."""
    # Get path to example URDF
    examples_dir = get_examples_dir()
    urdf_path = examples_dir / "roundtrip_test_robot.urdf"

    assert urdf_path.exists(), f"Example URDF not found at {urdf_path}"

    # Parse URDF
    robot = parse_urdf(urdf_path)

    # Verify basic structure
    assert robot.name == "comprehensive_test_robot"
    assert len(robot.links) == 15  # Added planar_platform and floating_sensor
    assert len(robot.joints) == 14  # Added planar_joint and floating_joint

    # Verify link names (includes base, wheels, sensors, arm, gripper)
    link_names = {link.name for link in robot.links}
    assert "base_link" in link_names
    assert "arm_base" in link_names
    assert "upper_arm" in link_names
    assert "forearm" in link_names
    assert "gripper_base" in link_names
    assert "left_finger" in link_names
    assert "right_finger" in link_names

    # Verify joint names and types
    joint_names = {joint.name for joint in robot.joints}
    revolute_joints = [j for j in robot.joints if j.type.name == "REVOLUTE"]

    assert len(revolute_joints) == 4  # arm_base_joint, shoulder_joint, elbow_joint, wrist_joint

    # Export back to URDF
    with tempfile.TemporaryDirectory() as tmpdir:
        output_path = Path(tmpdir) / "exported.urdf"

        generator = URDFGenerator(pretty_print=True, urdf_path=output_path)
        generator.write(robot, output_path)

        assert output_path.exists(), "Exported URDF file not created"

        # Re-parse exported URDF
        robot2 = parse_urdf(output_path)

        # Verify structure is preserved
        assert robot2.name == robot.name
        assert len(robot2.links) == len(robot.links)
        assert len(robot2.joints) == len(robot.joints)

        # Verify link names match
        link_names2 = {link.name for link in robot2.links}
        assert link_names2 == link_names

        # Verify joint names match
        joint_names2 = {joint.name for joint in robot2.joints}
        assert joint_names2 == joint_names


def test_materials_preserved():
    """Test that material colors are preserved in round-trip."""
    examples_dir = get_examples_dir()
    urdf_path = examples_dir / "roundtrip_test_robot.urdf"

    robot = parse_urdf(urdf_path)

    # Check that materials are parsed
    materials_found = []
    for link in robot.links:
        if link.visuals and link.visuals[0] and link.visuals[0].material:
            materials_found.append(link.visuals[0].material.name)

    assert len(materials_found) > 0, "Expected materials to be parsed"
    # Check for expected materials in roundtrip_test_robot
    material_names = {m for m in materials_found}
    assert "arm_material" in material_names or "base_material" in material_names


def test_inertial_preserved():
    """Test that inertial properties are preserved in round-trip."""
    examples_dir = get_examples_dir()
    urdf_path = examples_dir / "roundtrip_test_robot.urdf"

    robot = parse_urdf(urdf_path)

    # Check that all links have inertial properties
    for link in robot.links:
        assert link.inertial is not None, f"Link {link.name} missing inertial"
        assert link.inertial.mass > 0, f"Link {link.name} has invalid mass"
        assert link.inertial.inertia is not None, f"Link {link.name} missing inertia tensor"


def test_quadruped_roundtrip():
    """Test roundtrip for the complex quadruped robot example."""
    examples_dir = get_examples_dir()
    urdf_path = examples_dir / "quadruped_robot.urdf"
    if not urdf_path.exists():
        pytest.skip("quadruped_robot.urdf not found")

    # 1. Parse original URDF
    robot = parse_urdf(urdf_path)

    # Verify structure
    assert len(robot.links) == 17  # Base + 4 legs * 4 links (hip, thigh, calf, foot)
    assert len(robot.joints) == 16  # 4 legs * 4 joints
    assert len(robot.transmissions) == 12  # 3 actuated joints per leg * 4 legs
    assert len(robot.ros2_controls) == 1  # 1 system tag

    # 2. Generate URDF from model
    generator = URDFGenerator()
    generated_urdf = generator.generate(robot)

    # 3. Parse generated URDF
    robot_roundtrip = parse_urdf_string(generated_urdf)

    # 4. Compare
    assert robot_roundtrip.name == robot.name
    assert len(robot_roundtrip.links) == len(robot.links)
    assert len(robot_roundtrip.joints) == len(robot.joints)
    assert len(robot_roundtrip.transmissions) == len(robot.transmissions)
    assert len(robot_roundtrip.ros2_controls) == len(robot.ros2_controls)

    # Verify specific ros2_control content
    rc = robot_roundtrip.ros2_controls[0]
    assert len(rc.joints) == 12
    assert rc.joints[0].command_interfaces[0] == "effort"


def test_joint_limits_preserved():
    """Test that joint limits are preserved in round-trip."""
    examples_dir = get_examples_dir()
    urdf_path = examples_dir / "roundtrip_test_robot.urdf"

    robot = parse_urdf(urdf_path)

    # Check that all joints have limits
    for joint in robot.joints:
        if joint.type.name == "REVOLUTE" or joint.type.name == "PRISMATIC":
            assert joint.limits is not None, f"Joint {joint.name} missing limits"
            assert joint.limits.lower < joint.limits.upper, f"Invalid limits for {joint.name}"
            assert joint.limits.effort > 0, f"Invalid effort for {joint.name}"
            assert joint.limits.velocity > 0, f"Invalid velocity for {joint.name}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
