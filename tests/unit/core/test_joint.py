"""Comprehensive tests for Joint model and related classes."""

from __future__ import annotations

import math

import pytest

from linkforge.core.models import (
    Joint,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointType,
    Transform,
    Vector3,
)


class TestJointLimits:
    """Tests for JointLimits validation."""

    def test_valid_limits(self):
        """Test creating valid joint limits."""
        limits = JointLimits(lower=-math.pi, upper=math.pi, effort=100.0, velocity=10.0)
        assert limits.lower == -math.pi
        assert limits.upper == math.pi
        assert limits.effort == 100.0
        assert limits.velocity == 10.0

    def test_lower_greater_than_upper(self):
        """Test that lower > upper raises error."""
        with pytest.raises(ValueError, match="Lower limit.*must be <= upper limit"):
            JointLimits(lower=2.0, upper=1.0)

    def test_negative_effort(self):
        """Test that negative effort raises error."""
        with pytest.raises(ValueError, match="Effort must be non-negative"):
            JointLimits(lower=0.0, upper=1.0, effort=-10.0)

    def test_negative_velocity(self):
        """Test that negative velocity raises error."""
        with pytest.raises(ValueError, match="Velocity must be non-negative"):
            JointLimits(lower=0.0, upper=1.0, velocity=-5.0)

    def test_equal_limits(self):
        """Test that equal lower and upper limits is valid."""
        limits = JointLimits(lower=0.0, upper=0.0)
        assert limits.lower == limits.upper == 0.0

    def test_default_effort_velocity(self):
        """Test default values for effort and velocity."""
        limits = JointLimits(lower=-1.0, upper=1.0)
        assert limits.effort == 0.0
        assert limits.velocity == 0.0


class TestJointDynamics:
    """Tests for JointDynamics validation."""

    def test_valid_dynamics(self):
        """Test creating valid dynamics."""
        dynamics = JointDynamics(damping=0.5, friction=0.3)
        assert dynamics.damping == 0.5
        assert dynamics.friction == 0.3

    def test_negative_damping(self):
        """Test that negative damping raises error."""
        with pytest.raises(ValueError, match="Damping must be non-negative"):
            JointDynamics(damping=-0.5)

    def test_negative_friction(self):
        """Test that negative friction raises error."""
        with pytest.raises(ValueError, match="Friction must be non-negative"):
            JointDynamics(friction=-0.3)

    def test_default_values(self):
        """Test default dynamics values."""
        dynamics = JointDynamics()
        assert dynamics.damping == 0.0
        assert dynamics.friction == 0.0


class TestJointMimic:
    """Tests for JointMimic."""

    def test_creation(self):
        """Test creating mimic configuration."""
        mimic = JointMimic(joint="other_joint", multiplier=2.0, offset=0.5)
        assert mimic.joint == "other_joint"
        assert mimic.multiplier == 2.0
        assert mimic.offset == 0.5

    def test_default_values(self):
        """Test default multiplier and offset."""
        mimic = JointMimic(joint="other_joint")
        assert mimic.multiplier == 1.0
        assert mimic.offset == 0.0


class TestJoint:
    """Tests for Joint model."""

    def test_fixed_joint(self):
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

    def test_revolute_joint(self):
        """Test creating a revolute joint."""
        joint = Joint(
            name="revolute_joint",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            limits=JointLimits(lower=-math.pi, upper=math.pi),
        )
        assert joint.type == JointType.REVOLUTE
        assert joint.degrees_of_freedom == 1

    def test_continuous_joint(self):
        """Test creating a continuous joint."""
        joint = Joint(
            name="continuous_joint",
            type=JointType.CONTINUOUS,
            parent="link1",
            child="link2",
        )
        assert joint.type == JointType.CONTINUOUS
        assert joint.degrees_of_freedom == 1

    def test_prismatic_joint(self):
        """Test creating a prismatic joint."""
        joint = Joint(
            name="prismatic_joint",
            type=JointType.PRISMATIC,
            parent="link1",
            child="link2",
            limits=JointLimits(lower=0.0, upper=1.0),
        )
        assert joint.type == JointType.PRISMATIC
        assert joint.degrees_of_freedom == 1

    def test_planar_joint(self):
        """Test creating a planar joint."""
        joint = Joint(
            name="planar_joint",
            type=JointType.PLANAR,
            parent="link1",
            child="link2",
        )
        assert joint.type == JointType.PLANAR
        assert joint.degrees_of_freedom == 2

    def test_floating_joint(self):
        """Test creating a floating joint."""
        joint = Joint(
            name="floating_joint",
            type=JointType.FLOATING,
            parent="link1",
            child="link2",
        )
        assert joint.type == JointType.FLOATING
        assert joint.degrees_of_freedom == 6

    def test_empty_name(self):
        """Test that empty name raises error."""
        with pytest.raises(ValueError, match="name cannot be empty"):
            Joint(
                name="",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
            )

    def test_invalid_name_characters(self):
        """Test that invalid characters in name raise error."""
        with pytest.raises(ValueError, match="invalid characters"):
            Joint(
                name="joint with spaces!",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
            )

    def test_valid_name_with_underscore(self):
        """Test that underscores in names are valid."""
        joint = Joint(
            name="joint_1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        assert joint.name == "joint_1"

    def test_valid_name_with_hyphen(self):
        """Test that hyphens in names are valid."""
        joint = Joint(
            name="joint-1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        assert joint.name == "joint-1"

    def test_empty_parent(self):
        """Test that empty parent name raises error."""
        with pytest.raises(ValueError, match="Parent link name cannot be empty"):
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="",
                child="link2",
            )

    def test_empty_child(self):
        """Test that empty child name raises error."""
        with pytest.raises(ValueError, match="Child link name cannot be empty"):
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="",
            )

    def test_same_parent_and_child(self):
        """Test that same parent and child raises error."""
        with pytest.raises(ValueError, match="Parent and child cannot be the same"):
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="link1",
            )

    def test_revolute_without_limits(self):
        """Test that revolute joint without limits raises error."""
        with pytest.raises(ValueError, match="revolute joints require limits"):
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
            )

    def test_prismatic_without_limits(self):
        """Test that prismatic joint without limits raises error."""
        with pytest.raises(ValueError, match="prismatic joints require limits"):
            Joint(
                name="joint1",
                type=JointType.PRISMATIC,
                parent="link1",
                child="link2",
            )

    def test_fixed_with_limits(self):
        """Test that fixed joint with limits raises error."""
        with pytest.raises(ValueError, match="Fixed joints cannot have limits"):
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=0.0, upper=1.0),
            )

    def test_zero_axis(self):
        """Test that zero axis vector raises error."""
        with pytest.raises(ValueError, match="axis magnitude must be"):
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                axis=Vector3(0.0, 0.0, 0.0),
                limits=JointLimits(lower=0.0, upper=1.0),
            )

    def test_default_axis_revolute(self):
        """Test default axis for revolute is X-axis."""
        joint = Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            limits=JointLimits(lower=0.0, upper=1.0),
        )
        assert joint.axis == Vector3(1.0, 0.0, 0.0)

    def test_fixed_joint_has_no_axis(self):
        """Test that fixed joints have no axis."""
        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        assert joint.axis is None

    def test_custom_axis(self):
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

    def test_default_origin(self):
        """Test default origin is identity."""
        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        assert joint.origin.xyz == Vector3(0.0, 0.0, 0.0)
        assert joint.origin.rpy == Vector3(0.0, 0.0, 0.0)

    def test_custom_origin(self):
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

    def test_with_dynamics(self):
        """Test joint with dynamics."""
        dynamics = JointDynamics(damping=0.5, friction=0.3)
        joint = Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            limits=JointLimits(lower=-math.pi, upper=math.pi),
            dynamics=dynamics,
        )
        assert joint.dynamics == dynamics

    def test_with_mimic(self):
        """Test joint with mimic."""
        mimic = JointMimic(joint="other_joint", multiplier=2.0)
        joint = Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            limits=JointLimits(lower=-math.pi, upper=math.pi),
            mimic=mimic,
        )
        assert joint.mimic == mimic

    def test_continuous_joint_no_limits_required(self):
        """Test that continuous joints don't require limits."""
        joint = Joint(
            name="joint1",
            type=JointType.CONTINUOUS,
            parent="link1",
            child="link2",
        )
        assert joint.limits is None

    def test_continuous_joint_can_have_optional_limits(self):
        """Test that continuous joints CAN have optional limits (for effort/velocity)."""
        joint = Joint(
            name="joint1",
            type=JointType.CONTINUOUS,
            parent="link1",
            child="link2",
            limits=JointLimits(lower=0.0, upper=0.0, effort=10.0, velocity=1.0),
        )
        assert joint.limits is not None
        assert joint.limits.effort == 10.0
        assert joint.limits.velocity == 1.0

    def test_floating_joint_no_limits(self):
        """Test that floating joints don't require limits."""
        joint = Joint(
            name="joint1",
            type=JointType.FLOATING,
            parent="link1",
            child="link2",
        )
        assert joint.limits is None

    def test_planar_joint_no_limits(self):
        """Test that planar joints don't require limits."""
        joint = Joint(
            name="joint1",
            type=JointType.PLANAR,
            parent="link1",
            child="link2",
        )
        assert joint.limits is None


class TestJointType:
    """Tests for JointType enum."""

    def test_all_joint_types(self):
        """Test that all joint types are defined."""
        assert JointType.REVOLUTE.value == "revolute"
        assert JointType.CONTINUOUS.value == "continuous"
        assert JointType.PRISMATIC.value == "prismatic"
        assert JointType.FIXED.value == "fixed"
        assert JointType.FLOATING.value == "floating"
        assert JointType.PLANAR.value == "planar"

    def test_joint_type_count(self):
        """Test that we have exactly 6 joint types."""
        assert len(JointType) == 6
