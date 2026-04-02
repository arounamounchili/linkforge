"""Comprehensive tests for Joint model and related classes."""

from __future__ import annotations

import math

import pytest
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models import Transform, Vector3
from linkforge_core.models.joint import (
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafetyController,
    JointType,
)


class TestJointLimits:
    """Tests for JointLimits validation."""

    def test_valid_limits(self) -> None:
        """Test creating valid joint limits."""
        limits = JointLimits(lower=-math.pi, upper=math.pi, effort=100.0, velocity=10.0)
        assert limits.lower == -math.pi
        assert limits.upper == math.pi
        assert limits.effort == 100.0
        assert limits.velocity == 10.0

    def test_lower_greater_than_upper(self) -> None:
        """Test that lower > upper raises error."""
        with pytest.raises(RobotModelError):
            JointLimits(lower=2.0, upper=1.0)

    def test_negative_effort(self) -> None:
        """Test that negative effort raises error."""
        with pytest.raises(RobotModelError):
            JointLimits(lower=0.0, upper=1.0, effort=-10.0)

    def test_negative_velocity(self) -> None:
        """Test that negative velocity raises error."""
        with pytest.raises(RobotModelError):
            JointLimits(lower=0.0, upper=1.0, velocity=-5.0)

    def test_equal_limits(self) -> None:
        """Test that equal lower and upper limits is valid."""
        limits = JointLimits(lower=0.0, upper=0.0)
        assert limits.lower == limits.upper == 0.0

    def test_default_effort_velocity(self) -> None:
        """Test default values for effort and velocity."""
        limits = JointLimits(lower=-1.0, upper=1.0)
        assert limits.effort == 0.0
        assert limits.velocity == 0.0


class TestJointDynamics:
    """Tests for JointDynamics validation."""

    def test_valid_dynamics(self) -> None:
        """Test creating valid dynamics."""
        dynamics = JointDynamics(damping=0.5, friction=0.3)
        assert dynamics.damping == 0.5
        assert dynamics.friction == 0.3

    def test_negative_damping(self) -> None:
        """Test that negative damping raises error."""
        with pytest.raises(RobotModelError):
            JointDynamics(damping=-0.5)

    def test_negative_friction(self) -> None:
        """Test that negative friction raises error."""
        with pytest.raises(RobotModelError):
            JointDynamics(friction=-0.3)

    def test_default_values(self) -> None:
        """Test default dynamics values."""
        dynamics = JointDynamics()
        assert dynamics.damping == 0.0
        assert dynamics.friction == 0.0


class TestJointMimic:
    """Tests for JointMimic."""

    def test_creation(self) -> None:
        """Test creating mimic configuration."""
        mimic = JointMimic(joint="other_joint", multiplier=2.0, offset=0.5)
        assert mimic.joint == "other_joint"
        assert mimic.multiplier == 2.0
        assert mimic.offset == 0.5

    def test_default_values(self) -> None:
        """Test default multiplier and offset."""
        mimic = JointMimic(joint="other_joint")
        assert mimic.multiplier == 1.0
        assert mimic.offset == 0.0


class TestJointSafetyController:
    """Tests for JointSafetyController."""

    def test_creation(self) -> None:
        """Test creating safety controller configuration."""
        safety = JointSafetyController(
            soft_lower_limit=-1.0,
            soft_upper_limit=1.0,
            k_position=10.0,
            k_velocity=5.0,
        )
        assert safety.soft_lower_limit == -1.0
        assert safety.soft_upper_limit == 1.0
        assert safety.k_position == 10.0
        assert safety.k_velocity == 5.0

    def test_default_values(self) -> None:
        """Test default values for safety controller."""
        safety = JointSafetyController()
        assert safety.soft_lower_limit == 0.0
        assert safety.soft_upper_limit == 0.0
        assert safety.k_position == 0.0
        assert safety.k_velocity == 0.0


class TestJointCalibration:
    """Tests for JointCalibration."""

    def test_creation(self) -> None:
        """Test creating calibration configuration."""
        calib = JointCalibration(rising=0.1, falling=0.2)
        assert calib.rising == 0.1
        assert calib.falling == 0.2

    def test_default_values(self) -> None:
        """Test default values for calibration."""
        calib = JointCalibration()
        assert calib.rising is None
        assert calib.falling is None


class TestJoint:
    """Tests for Joint model."""

    def test_fixed_joint(self) -> None:
        """Test creating a fixed joint."""
        joint = Joint(
            name="fixed_joint",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        assert joint.name == "fixed_joint"
        assert joint.type == JointType.FIXED
        assert joint.degrees_of_freedom == 0

    def test_revolute_joint(self) -> None:
        """Test creating a revolute joint."""
        joint = Joint(
            name="revolute_joint",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-math.pi, upper=math.pi),
        )
        assert joint.type == JointType.REVOLUTE
        assert joint.degrees_of_freedom == 1

    def test_continuous_joint(self) -> None:
        """Test creating a continuous joint."""
        joint = Joint(
            name="continuous_joint",
            type=JointType.CONTINUOUS,
            parent="link1",
            child="link2",
            axis=Vector3(1.0, 0.0, 0.0),
        )
        assert joint.type == JointType.CONTINUOUS
        assert joint.degrees_of_freedom == 1

    def test_prismatic_joint(self) -> None:
        """Test creating a prismatic joint."""
        joint = Joint(
            name="prismatic_joint",
            type=JointType.PRISMATIC,
            parent="link1",
            child="link2",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=0.0, upper=1.0),
        )
        assert joint.type == JointType.PRISMATIC
        assert joint.degrees_of_freedom == 1

    def test_planar_joint(self) -> None:
        """Test creating a planar joint."""
        joint = Joint(
            name="planar_joint",
            type=JointType.PLANAR,
            parent="link1",
            child="link2",
            axis=Vector3(1.0, 0.0, 0.0),
        )
        assert joint.type == JointType.PLANAR
        assert joint.degrees_of_freedom == 2

    def test_floating_joint(self) -> None:
        """Test creating a floating joint."""
        joint = Joint(
            name="floating_joint",
            type=JointType.FLOATING,
            parent="link1",
            child="link2",
        )
        assert joint.type == JointType.FLOATING
        assert joint.degrees_of_freedom == 6

    def test_empty_name(self) -> None:
        """Test that empty name raises error."""
        with pytest.raises(RobotModelError, match="cannot be empty"):
            Joint(
                name="",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
            )

    def test_invalid_name_characters(self) -> None:
        """Test that invalid characters in name raise error."""
        with pytest.raises(RobotModelError):
            Joint(
                name="joint with spaces!",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
            )

    def test_valid_name_with_underscore(self) -> None:
        """Test that underscores in names are valid."""
        joint = Joint(
            name="joint_1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        assert joint.name == "joint_1"

    def test_valid_name_with_hyphen(self) -> None:
        """Test that hyphens in names are valid."""
        joint = Joint(
            name="joint-1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        assert joint.name == "joint-1"

    def test_empty_parent(self) -> None:
        """Test that empty parent name raises error."""
        with pytest.raises(RobotModelError, match="cannot be empty"):
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="",
                child="link2",
            )

    def test_empty_child(self) -> None:
        """Test that empty child name raises error."""
        with pytest.raises(RobotModelError, match="cannot be empty"):
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="",
            )

    def test_same_parent_and_child(self) -> None:
        """Test that same parent and child raises error."""
        with pytest.raises(RobotModelError, match="cannot be the same"):
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="link1",
            )

    def test_revolute_without_limits(self) -> None:
        """Test that revolute joint without limits raises error."""
        with pytest.raises(RobotModelError):
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                axis=Vector3(1.0, 0.0, 0.0),
            )

    def test_prismatic_without_limits(self) -> None:
        """Test that prismatic joint without limits raises error."""
        with pytest.raises(RobotModelError):
            Joint(
                name="joint1",
                type=JointType.PRISMATIC,
                parent="link1",
                child="link2",
                axis=Vector3(1.0, 0.0, 0.0),
            )

    def test_fixed_with_limits(self) -> None:
        """Test that fixed joint with limits raises error."""
        with pytest.raises(RobotModelError):
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=0.0, upper=1.0),
            )

    def test_zero_axis(self) -> None:
        """Test that zero axis vector raises error."""
        with pytest.raises(RobotModelError):
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                axis=Vector3(0.0, 0.0, 0.0),
                limits=JointLimits(lower=0.0, upper=1.0),
            )

    def test_revolute_without_axis_error(self) -> None:
        """Test that revolute joint without axis raises error in strict model."""
        with pytest.raises(RobotModelError):
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=0.0, upper=1.0),
            )

    def test_fixed_joint_has_no_axis(self) -> None:
        """Test that fixed joints have no axis."""
        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        assert joint.axis is None

    def test_custom_axis(self) -> None:
        """Test custom axis."""
        joint = Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            axis=Vector3(0.0, 0.0, 1.0),
            limits=JointLimits(lower=0.0, upper=1.0),
        )
        assert joint.axis == Vector3(0.0, 0.0, 1.0)

    def test_default_origin(self) -> None:
        """Test default origin is identity."""
        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        assert joint.origin.xyz == Vector3(0.0, 0.0, 0.0)
        assert joint.origin.rpy == Vector3(0.0, 0.0, 0.0)

    def test_custom_origin(self) -> None:
        """Test custom origin."""
        origin = Transform(
            xyz=Vector3(1.0, 2.0, 3.0),
            rpy=Vector3(0.1, 0.2, 0.3),
        )
        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
            origin=origin,
        )
        assert joint.origin == origin

    def test_with_dynamics(self) -> None:
        """Test joint with dynamics."""
        dynamics = JointDynamics(damping=0.5, friction=0.3)
        joint = Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-math.pi, upper=math.pi),
            dynamics=dynamics,
        )
        assert joint.dynamics == dynamics

    def test_with_mimic(self) -> None:
        """Test joint with mimic."""
        mimic = JointMimic(joint="other_joint", multiplier=2.0)
        joint = Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-math.pi, upper=math.pi),
            mimic=mimic,
        )
        assert joint.mimic == mimic

    def test_with_safety_controller(self) -> None:
        """Test joint with safety controller."""
        safety = JointSafetyController(soft_lower_limit=-1.0, soft_upper_limit=1.0)
        joint = Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-math.pi, upper=math.pi),
            safety_controller=safety,
        )
        assert joint.safety_controller == safety

    def test_with_calibration(self) -> None:
        """Test joint with calibration."""
        calib = JointCalibration(rising=0.1)
        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
            calibration=calib,
        )
        assert joint.calibration == calib

    def test_continuous_joint_no_limits_required(self) -> None:
        """Test that continuous joints don't require limits."""
        joint = Joint(
            name="joint1",
            type=JointType.CONTINUOUS,
            parent="link1",
            child="link2",
            axis=Vector3(1.0, 0.0, 0.0),
        )
        assert joint.limits is None

    def test_continuous_joint_can_have_optional_limits(self) -> None:
        """Test that continuous joints CAN have optional limits (for effort/velocity)."""
        joint = Joint(
            name="joint1",
            type=JointType.CONTINUOUS,
            parent="link1",
            child="link2",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=0.0, upper=0.0, effort=10.0, velocity=1.0),
        )
        assert joint.limits is not None
        assert joint.limits.effort == 10.0
        assert joint.limits.velocity == 1.0

    def test_floating_joint_no_limits(self) -> None:
        """Test that floating joints don't require limits."""
        joint = Joint(
            name="joint1",
            type=JointType.FLOATING,
            parent="link1",
            child="link2",
        )
        assert joint.limits is None

    def test_planar_joint_no_limits(self) -> None:
        """Test that planar joints don't require limits."""
        joint = Joint(
            name="joint1",
            type=JointType.PLANAR,
            parent="link1",
            child="link2",
            axis=Vector3(1.0, 0.0, 0.0),
        )
        assert joint.limits is None


class TestJointType:
    """Tests for JointType enum."""

    def test_all_joint_types(self) -> None:
        """Test that all joint types are defined."""
        assert JointType.REVOLUTE.value == "revolute"
        assert JointType.CONTINUOUS.value == "continuous"
        assert JointType.PRISMATIC.value == "prismatic"
        assert JointType.FIXED.value == "fixed"
        assert JointType.FLOATING.value == "floating"
        assert JointType.PLANAR.value == "planar"

    def test_joint_type_count(self) -> None:
        """Test that we have exactly 6 joint types."""
        assert len(JointType) == 6


class TestJointAxisNormalization:
    """Tests for joint axis normalization."""

    def test_axis_normalization_error(self) -> None:
        """Test that non-unit axis vectors raise error in strict model."""
        with pytest.raises(RobotModelError, match="unit vector"):
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                axis=Vector3(2.0, 0.0, 0.0),  # Non-unit vector
                limits=JointLimits(lower=0.0, upper=1.0),
            )

    def test_axis_normalization_complex_error(self) -> None:
        """Test that non-unit complex axis vectors raise error in strict model."""
        with pytest.raises(RobotModelError, match="unit vector"):
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                axis=Vector3(3.0, 4.0, 0.0),  # Magnitude = 5.0
                limits=JointLimits(lower=0.0, upper=1.0),
            )

    def test_axis_already_normalized(self) -> None:
        """Test that already-normalized axis is not modified."""
        joint = Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            axis=Vector3(1.0, 0.0, 0.0),  # Already unit vector
            limits=JointLimits(lower=0.0, upper=1.0),
        )
        # Should remain unchanged
        assert joint.axis.x == 1.0
        assert joint.axis.y == 0.0
        assert joint.axis.z == 0.0


class TestJointAxisWarnings:
    """Tests for joint axis validation warnings."""

    def test_fixed_joint_with_axis_error(self) -> None:
        """Test that fixed joint with axis raises error."""
        with pytest.raises(RobotModelError):
            Joint(
                name="fixed_joint",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
                axis=Vector3(1.0, 0.0, 0.0),  # Fixed joints shouldn't have axis
            )

    def test_floating_joint_with_axis_error(self) -> None:
        """Test that floating joint with axis raises error."""
        with pytest.raises(RobotModelError):
            Joint(
                name="floating_joint",
                type=JointType.FLOATING,
                parent="link1",
                child="link2",
                axis=Vector3(0.0, 1.0, 0.0),  # Floating joints shouldn't have axis
            )
