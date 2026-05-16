import pytest
from linkforge.core import JointType, RobotBuilder, RobotValidationError


class TestLinkBuilderAdvanced:
    def test_axis_normalization(self) -> None:
        """Test that LinkBuilder automatically normalizes joint axes."""
        builder = RobotBuilder("norm_test")

        # Revolute with non-unit axis (0, 0, 2)
        builder.link("base").root()
        builder.link("link1", parent="base").revolute(axis=(0, 0, 2), limits=(-1, 1)).commit()

        joint = builder.robot.get_joint("base_to_link1")
        assert joint is not None
        assert joint.axis is not None
        # Should be normalized to (0, 0, 1)
        assert joint.axis.x == 0.0
        assert joint.axis.y == 0.0
        assert joint.axis.z == 1.0

    def test_continuous_axis_normalization(self) -> None:
        """Test auto-normalization for continuous joints."""
        builder = RobotBuilder("cont_norm_test")
        builder.link("base").root()
        builder.link("link1", parent="base").continuous(axis=(5, 0, 0)).commit()

        joint = builder.robot.get_joint("base_to_link1")
        assert joint is not None
        assert joint.axis is not None
        assert joint.axis.x == 1.0
        assert joint.axis.y == 0.0
        assert joint.axis.z == 0.0

    def test_prismatic_axis_normalization(self) -> None:
        """Test auto-normalization for prismatic joints."""
        builder = RobotBuilder("prism_norm_test")
        builder.link("base").root()
        builder.link("link1", parent="base").prismatic(axis=(0, 3, 0), limits=(0, 1)).commit()

        joint = builder.robot.get_joint("base_to_link1")
        assert joint is not None
        assert joint.axis is not None
        assert joint.axis.x == 0.0
        assert joint.axis.y == 1.0
        assert joint.axis.z == 0.0

    def test_zero_axis_error(self) -> None:
        """Test that LinkBuilder catches zero-magnitude axis immediately."""
        builder = RobotBuilder("zero_axis_test")
        builder.link("base").root()

        with pytest.raises(RobotValidationError, match="Joint axis magnitude is too small"):
            builder.link("link1", parent="base").revolute(axis=(0, 0, 0), limits=(-1, 1))

    def test_internal_methods_docstrings(self) -> None:
        """Verify internal methods have docstrings (smoke test via __doc__)."""
        builder = RobotBuilder("doc_test")
        lb = builder.link("test")

        assert lb._finalize_joint.__doc__ is not None
        assert lb._finalize_link.__doc__ is not None
        assert lb._finalize_inertial.__doc__ is not None
        assert lb._finalize_transmission.__doc__ is not None
        assert lb._finalize_ros2_control.__doc__ is not None

    def test_attach_with_axis_and_limits(self) -> None:
        """Test attaching a component with explicit axis and limits."""
        # Base robot
        builder = RobotBuilder("base_bot")
        builder.link("base").root()

        # Component to attach
        comp_builder = RobotBuilder("arm")
        comp_builder.link("link1").root()

        # Attach with revolute joint

        builder.attach(
            comp_builder,
            at_link="base",
            prefix="arm_",
            joint_type=JointType.REVOLUTE,
            axis=(0, 0, 2),  # Should normalize
            limits=(-3.14, 3.14),
        )

        joint = builder.robot.get_joint("base_to_arm_link1")
        assert joint is not None
        assert joint.type == JointType.REVOLUTE
        assert joint.axis is not None
        assert joint.axis.z == 1.0  # Normalized
        assert joint.limits is not None
        assert joint.limits.lower == -3.14
