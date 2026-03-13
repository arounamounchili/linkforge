"""Tests for the enhanced validation system."""

from linkforge_core.models.joint import Joint, JointLimits, JointType, Vector3
from linkforge_core.models.link import Inertial, Link
from linkforge_core.models.robot import Robot
from linkforge_core.validation import RobotValidator, ValidationResult


def test_validation_result_creation() -> None:
    """Test creating a validation result."""
    result = ValidationResult(robot_name="test_robot")

    assert result.is_valid
    assert not result.has_warnings
    assert result.error_count == 0
    assert result.warning_count == 0


def test_validation_result_add_error() -> None:
    """Test adding an error to validation result."""
    result = ValidationResult()

    result.add_error(
        title="Test Error",
        message="This is a test error",
        affected_objects=["link1", "link2"],
        suggestion="Fix the error",
    )

    assert not result.is_valid
    assert result.error_count == 1
    assert result.warning_count == 0
    assert len(result.errors) == 1

    error = result.errors[0]
    assert error.title == "Test Error"
    assert error.message == "This is a test error"
    assert error.affected_objects == ["link1", "link2"]
    assert error.suggestion == "Fix the error"
    assert error.is_error
    assert not error.is_warning


def test_validation_result_add_warning() -> None:
    """Test adding a warning to validation result."""
    result = ValidationResult()

    result.add_warning(
        title="Test Warning",
        message="This is a test warning",
        suggestion="Consider fixing",
    )

    assert result.is_valid  # Warnings don't block validity
    assert result.has_warnings
    assert result.error_count == 0
    assert result.warning_count == 1
    assert len(result.warnings) == 1

    warning = result.warnings[0]
    assert warning.title == "Test Warning"
    assert warning.is_warning
    assert not warning.is_error


def test_valid_robot() -> None:
    """Test validation of a valid robot."""
    robot = Robot(name="valid_robot")

    # Create valid robot structure
    base = Link(name="base", inertial=Inertial(mass=1.0))
    link1 = Link(name="link1", inertial=Inertial(mass=0.5))

    robot.add_link(base)
    robot.add_link(link1)
    robot.add_joint(
        Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="base",
            child="link1",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
        )
    )

    # Validate
    validator = RobotValidator()
    result = validator.validate(robot)

    assert result.is_valid
    assert result.error_count == 0
    # Note: Will have warnings about missing geometry


def test_robot_with_no_links() -> None:
    """Test validation of robot with no links."""
    robot = Robot(name="empty_robot")

    validator = RobotValidator()
    result = validator.validate(robot)

    assert not result.is_valid
    assert result.error_count == 1
    assert result.errors[0].title == "No links"


def test_duplicate_link_names() -> None:
    """Test validation of robot with duplicate link names."""
    robot = Robot(name="duplicate_links")

    # Bypass add_link to create invalid structure (duplicate names)
    robot._links.append(Link(name="duplicate", inertial=Inertial(mass=1.0)))
    robot._links.append(Link(name="duplicate", inertial=Inertial(mass=1.0)))

    validator = RobotValidator()
    result = validator.validate(robot)

    assert not result.is_valid
    errors = [e for e in result.errors if e.title == "Duplicate link name"]
    assert len(errors) == 1


def test_duplicate_joint_names() -> None:
    """Test validation of robot with duplicate joint names."""
    robot = Robot(name="duplicate_joints")

    base = Link(name="base", inertial=Inertial(mass=1.0))
    link1 = Link(name="link1", inertial=Inertial(mass=0.5))
    link2 = Link(name="link2", inertial=Inertial(mass=0.5))

    robot.add_link(base)
    robot.add_link(link1)
    robot.add_link(link2)

    # Add duplicate joints by manipulating internal list directly
    robot._joints.append(
        Joint(name="duplicate", type=JointType.FIXED, parent="base", child="link1")
    )
    robot._joints.append(
        Joint(name="duplicate", type=JointType.FIXED, parent="base", child="link2")
    )

    validator = RobotValidator()
    result = validator.validate(robot)

    assert not result.is_valid
    errors = [e for e in result.errors if e.title == "Duplicate joint name"]
    assert len(errors) == 1


def test_missing_parent_link() -> None:
    """Test validation of joint with missing parent link."""
    robot = Robot(name="missing_parent")

    link1 = Link(name="link1", inertial=Inertial(mass=1.0))
    robot.add_link(link1)

    # Add joint with invalid parent (bypass add_joint validation)
    robot._joints.append(
        Joint(name="joint1", type=JointType.FIXED, parent="nonexistent", child="link1")
    )

    validator = RobotValidator()
    result = validator.validate(robot)

    assert not result.is_valid
    errors = [e for e in result.errors if e.title == "Missing parent link"]
    assert len(errors) == 1
    assert "nonexistent" in errors[0].message


def test_missing_child_link() -> None:
    """Test validation of joint with missing child link."""
    robot = Robot(name="missing_child")

    base = Link(name="base", inertial=Inertial(mass=1.0))
    robot.add_link(base)

    # Add joint with invalid child (bypass add_joint validation)
    robot._joints.append(
        Joint(name="joint1", type=JointType.FIXED, parent="base", child="nonexistent")
    )

    validator = RobotValidator()
    result = validator.validate(robot)

    assert not result.is_valid
    errors = [e for e in result.errors if e.title == "Missing child link"]
    assert len(errors) == 1


def test_disconnected_link() -> None:
    """Test validation of disconnected link (shows as multiple root links)."""
    robot = Robot(name="disconnected")

    base = Link(name="base", inertial=Inertial(mass=1.0))
    link1 = Link(name="link1", inertial=Inertial(mass=0.5))
    link2 = Link(name="link2", inertial=Inertial(mass=0.5))  # Disconnected

    robot.add_link(base)
    robot.add_link(link1)
    robot.add_link(link2)

    # Only connect link1 to base
    robot.add_joint(Joint(name="joint1", type=JointType.FIXED, parent="base", child="link1"))

    validator = RobotValidator()
    result = validator.validate(robot)

    assert not result.is_valid
    # Disconnected link creates multiple root links
    errors = [e for e in result.errors if e.title == "Multiple root links"]
    assert len(errors) == 1
    assert "link2" in errors[0].message or "base" in errors[0].message


def test_multiple_parent_joints() -> None:
    """Test validation of link with multiple parent joints.

    URDF spec: No <link> element can serve as a child node in more than one <joint> element.
    Put another way, no <link> element can have more than one parent element in the model's
    connectivity graph. Only the root link can have zero parents.
    """
    robot = Robot(name="multiple_parents")

    base = Link(name="base", inertial=Inertial(mass=1.0))
    link1 = Link(name="link1", inertial=Inertial(mass=0.5))
    link2 = Link(name="link2", inertial=Inertial(mass=0.5))

    robot.add_link(base)
    robot.add_link(link1)
    robot.add_link(link2)

    # Add first joint normally
    robot.add_joint(Joint(name="joint1", type=JointType.FIXED, parent="base", child="link1"))

    # Bypass add_joint validation to create invalid structure
    # Both joints have link2 as child (link2 has multiple parents)
    robot._joints.append(Joint(name="joint2", type=JointType.FIXED, parent="base", child="link2"))
    robot._joints.append(Joint(name="joint3", type=JointType.FIXED, parent="link1", child="link2"))

    validator = RobotValidator()
    result = validator.validate(robot)

    assert not result.is_valid
    errors = [e for e in result.errors if e.title == "Multiple parent joints"]
    assert len(errors) == 1
    assert "link2" in errors[0].message
    assert "2 parent joints" in errors[0].message
    assert errors[0].affected_objects == ["link2"]


def test_multiple_parent_joints_complex() -> None:
    """Test validation with multiple links having multiple parents.

    This tests a more complex invalid structure where multiple links violate the single-parent rule.
    """
    robot = Robot(name="complex_invalid")

    # Create a structure that violates the tree constraint
    base = Link(name="base", inertial=Inertial(mass=1.0))
    link1 = Link(name="link1", inertial=Inertial(mass=0.5))
    link2 = Link(name="link2", inertial=Inertial(mass=0.5))
    link3 = Link(name="link3", inertial=Inertial(mass=0.5))
    link4 = Link(name="link4", inertial=Inertial(mass=0.5))

    robot.add_link(base)
    robot.add_link(link1)
    robot.add_link(link2)
    robot.add_link(link3)
    robot.add_link(link4)

    # Create valid part of tree
    robot.add_joint(Joint(name="joint1", type=JointType.FIXED, parent="base", child="link1"))

    # Bypass validation to create invalid structure:
    # link3 has two parents (link1 and link2)
    robot._joints.append(Joint(name="joint2", type=JointType.FIXED, parent="base", child="link2"))
    robot._joints.append(Joint(name="joint3a", type=JointType.FIXED, parent="link1", child="link3"))
    robot._joints.append(
        Joint(
            name="joint3b",
            type=JointType.REVOLUTE,
            parent="link2",
            child="link3",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
        )
    )

    # link4 also has two parents (both link2 and link3)
    robot._joints.append(Joint(name="joint4a", type=JointType.FIXED, parent="link2", child="link4"))
    robot._joints.append(Joint(name="joint4b", type=JointType.FIXED, parent="link3", child="link4"))

    validator = RobotValidator()
    result = validator.validate(robot)

    assert not result.is_valid
    errors = [e for e in result.errors if e.title == "Multiple parent joints"]

    # Should detect both link3 and link4 as having multiple parents
    assert len(errors) == 2
    affected_links = set()
    for error in errors:
        assert len(error.affected_objects) == 1
        affected_links.update(error.affected_objects)

    assert "link3" in affected_links
    assert "link4" in affected_links


def test_low_mass_warning() -> None:
    """Test warning for very low mass link."""
    robot = Robot(name="low_mass")

    # Link with very low mass
    base = Link(name="base", inertial=Inertial(mass=0.001))  # 1 gram
    robot.add_link(base)

    validator = RobotValidator()
    result = validator.validate(robot)

    assert result.is_valid  # Not an error
    assert result.has_warnings
    warnings = [w for w in result.warnings if w.title == "Very low mass"]
    assert len(warnings) == 1
    assert "0.001" in warnings[0].message or "base" in warnings[0].message


def test_missing_inertia_warning() -> None:
    """Test warning for missing inertia."""
    robot = Robot(name="no_inertia")

    # Link without inertia
    base = Link(name="base", inertial=None)
    robot.add_link(base)

    validator = RobotValidator()
    result = validator.validate(robot)

    assert result.is_valid
    assert result.has_warnings
    warnings = [w for w in result.warnings if w.title == "Missing inertia"]
    assert len(warnings) == 1


def test_missing_geometry_warnings() -> None:
    """Test warnings for missing visual and collision geometry."""
    robot = Robot(name="no_geometry")

    # Link with no geometry
    base = Link(name="base", inertial=Inertial(mass=1.0))
    robot.add_link(base)

    validator = RobotValidator()
    result = validator.validate(robot)

    assert result.is_valid
    assert result.has_warnings

    # Should have warnings for both visual and collision
    visual_warnings = [w for w in result.warnings if w.title == "No visual geometry"]
    collision_warnings = [w for w in result.warnings if w.title == "No collision geometry"]

    assert len(visual_warnings) == 1
    assert len(collision_warnings) == 1


def test_validation_result_string_representation() -> None:
    """Test string representation of validation results."""
    result = ValidationResult(robot_name="my_robot")

    # Valid with no warnings
    assert "valid" in str(result).lower()
    assert "my_robot" in str(result)

    # Valid with warnings
    result.add_warning("Test", "Test warning")
    assert "warning" in str(result).lower()
    assert "my_robot" in str(result)

    # Invalid with errors
    result.add_error("Test", "Test error")
    assert "error" in str(result).lower()
    assert "my_robot" in str(result)


def test_validator_cycle_detection() -> None:
    """Test that validator detects cycles in kinematic tree."""
    from linkforge_core.validation.validator import RobotValidator

    robot = Robot(name="test_robot")
    robot.add_link(Link(name="link1"))
    robot.add_link(Link(name="link2"))

    # Create cycle
    robot._joints.append(Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2"))
    robot._joints.append(Joint(name="joint2", type=JointType.FIXED, parent="link2", child="link1"))

    validator = RobotValidator()
    result = validator.validate(robot)

    assert not result.is_valid
    assert any(
        "cycle" in issue.title.lower() or "cycle" in issue.message.lower()
        for issue in result.issues
    )


def test_validator_no_root_link() -> None:
    """Test that validator detects when no root link exists."""
    from linkforge_core.validation.validator import RobotValidator

    robot = Robot(name="test_robot")
    robot.add_link(Link(name="link1"))
    robot.add_link(Link(name="link2"))

    # Create cycle (no root)
    robot._joints.append(Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2"))
    robot._joints.append(Joint(name="joint2", type=JointType.FIXED, parent="link2", child="link1"))

    validator = RobotValidator()
    result = validator.validate(robot)

    assert not result.is_valid
    # Should detect either cycle or no root
    assert len(result.issues) > 0


def test_validator_missing_collision() -> None:
    """Test validator warns on missing collision geometry."""
    from linkforge_core.models.geometry import Box, Vector3
    from linkforge_core.models.link import Visual

    robot = Robot(name="test_bot")
    # Link with visual but no collision
    l1 = Link(name="l1", initial_visuals=[Visual(geometry=Box(Vector3(1, 1, 1)))])
    robot.add_link(l1)

    validator = RobotValidator()
    result = validator.validate(robot)

    assert result.is_valid
    # Check for warning about missing collision
    assert any("no collision geometry" in w.message for w in result.warnings)


def test_validator_unreachable_code_mocks() -> None:
    """Test unreachable code paths via mocking (for 100% coverage)."""
    from unittest.mock import patch

    from linkforge_core.models.link import Inertial, Link
    from linkforge_core.models.robot import Robot
    from linkforge_core.validation import RobotValidator

    # Case 1: get_root_link returns None even with links
    robot = Robot(name="mock_bot")
    robot.add_link(Link(name="base", inertial=Inertial(mass=1.0)))

    with patch.object(robot, "get_root_link", return_value=None):
        validator = RobotValidator()
        result = validator.validate(robot)
        # Should trigger "No root link" error
        # Note: logic says if root is None, add error "No root link found"
        # The message is "No root link found. A robot must have exactly one link..."
        assert any("No root link found" in e.message for e in result.errors)

    # Case 2: Disconnected link check
    # logic: if root and link != root and count == 0:
    # Normally get_root_link would raise if there are multiple roots.
    # We need to mock get_root_link to return one root, while another exists.

    robot2 = Robot(name="disconnected_mock")
    base = Link(name="base", inertial=Inertial(mass=1.0))
    # Disconnected link
    disc = Link(name="disconnected", inertial=Inertial(mass=1.0))

    robot2.add_link(base)
    robot2.add_link(disc)

    # Mock get_root_link to return 'base' explicitly, ignoring the fact that 'disconnected' is also a root
    with patch.object(robot2, "get_root_link", return_value=base):
        validator2 = RobotValidator()
        result2 = validator2.validate(robot2)

        # Should detect 'disconnected' as a disconnected link
        # Because count for 'disconnected' is 0, and it != base.
        # if root and link != root and count == 0:
        # result.add_error(title="Disconnected link", ...)
        assert any("Disconnected link" in e.title for e in result2.errors)
        assert any("disconnected" in e.message for e in result2.errors)
