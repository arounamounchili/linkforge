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

    def test_builder_missing_branches(self) -> None:
        """Test builder cases for 100% coverage of link_builder.py."""
        builder = RobotBuilder("missing_branches_bot")
        builder.link("base").root()

        # 1. Test committed exception (Line 124)
        lb = builder.link("link1", parent="base")
        lb.commit()
        with pytest.raises(RuntimeError, match="already committed"):
            lb.visual(geometry=None)  # type: ignore

        # 2. Test continuous joint with only effort or only velocity provided (Line 355-359 branch)
        builder.link("link2", parent="base").continuous(
            axis=(0, 0, 1), effort=5.0, velocity=None
        ).commit()
        j2 = builder.robot.get_joint("base_to_link2")
        assert j2 is not None
        assert j2.limits is not None
        assert j2.limits.effort == 5.0
        assert j2.limits.velocity == 0.0

        builder.link("link3", parent="base").continuous(
            axis=(0, 0, 1), effort=None, velocity=10.0
        ).commit()
        j3 = builder.robot.get_joint("base_to_link3")
        assert j3 is not None
        assert j3.limits is not None
        assert j3.limits.effort == 0.0
        assert j3.limits.velocity == 10.0

        # Test continuous joint with both effort and velocity as None (covers False branch of effort or velocity)
        builder.link("link2b", parent="base").continuous(
            axis=(0, 0, 1), effort=None, velocity=None
        ).commit()
        j2b = builder.robot.get_joint("base_to_link2b")
        assert j2b is not None
        assert j2b.limits is None

        # 3. Test floating joint (Line 409-411)
        builder.link("link4", parent="base").floating(name="float_joint").commit()
        j4 = builder.robot.get_joint("float_joint")
        assert j4 is not None
        assert j4.type == JointType.FLOATING

        # 4. Test planar joint (Line 431-434)
        builder.link("link5", parent="base").planar(axis=(0, 0, 1), name="planar_joint").commit()
        j5 = builder.robot.get_joint("planar_joint")
        assert j5 is not None
        assert j5.type == JointType.PLANAR
        assert j5.axis is not None
        assert j5.axis.z == 1.0

        # 5. Test physics with raw gazebo params (Line 525->529, 531, 967-968)
        builder.link("link6", parent="base").physics(
            mu=0.8, kp=1e6, material="Gazebo/Red", static=True
        ).commit()
        gz = None
        for element in builder.robot.gazebo_elements:
            if element.reference == "link6":
                gz = element
                break
        assert gz is not None
        assert gz.material == "Gazebo/Red"
        assert gz.static is True

        # Test physics with ONLY raw gazebo params (covers False branch of phys_updates)
        builder.link("link6b", parent="base").physics(material="Gazebo/Blue").commit()
        gzb = None
        for element in builder.robot.gazebo_elements:
            if element.reference == "link6b":
                gzb = element
                break
        assert gzb is not None
        assert gzb.material == "Gazebo/Blue"

        # 6. Test gpu_lidar (Line 707-717)
        builder.link("link7", parent="base").gpu_lidar(name="mylidar").commit()
        sensor = None
        for s in builder.robot.sensors:
            if s.name == "mylidar":
                sensor = s
                break
        assert sensor is not None
        assert sensor.type.name == "GPU_LIDAR"
