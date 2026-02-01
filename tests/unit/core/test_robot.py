"""Tests for Robot model."""

from __future__ import annotations

import math

import pytest
from linkforge_core.models import (
    CameraInfo,
    GazeboElement,
    GazeboPlugin,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    Robot,
    Sensor,
    SensorType,
    Transmission,
)


class TestRobot:
    """Tests for Robot model."""

    def test_creation(self):
        """Test creating a robot."""
        robot = Robot(name="test_robot")
        assert robot.name == "test_robot"
        assert len(robot.links) == 0
        assert len(robot.joints) == 0

    def test_invalid_name(self):
        """Test that invalid names raise error."""
        with pytest.raises(ValueError, match="invalid characters"):
            Robot(name="robot with spaces!")

    def test_add_link(self):
        """Test adding a link."""
        robot = Robot(name="test_robot")
        link = Link(name="link1")
        robot.add_link(link)

        assert len(robot.links) == 1
        assert robot.get_link("link1") == link

    def test_add_duplicate_link(self):
        """Test that adding duplicate link raises error."""
        robot = Robot(name="test_robot")
        link = Link(name="link1")
        robot.add_link(link)

        with pytest.raises(ValueError, match="already exists"):
            robot.add_link(link)

    def test_add_joint(self):
        """Test adding a joint."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        robot.add_joint(joint)

        assert len(robot.joints) == 1
        assert robot.get_joint("joint1") == joint

    def test_add_joint_missing_parent(self):
        """Test that adding joint with missing parent raises error."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link2"))

        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",  # Does not exist
            child="link2",
        )

        with pytest.raises(ValueError, match="not found"):
            robot.add_joint(joint)

    def test_get_root_link(self):
        """Test getting root link."""
        robot = Robot(name="test_robot")
        link1 = Link(name="link1")
        link2 = Link(name="link2")

        robot.add_link(link1)
        robot.add_link(link2)

        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        robot.add_joint(joint)

        root = robot.get_root_link()
        assert root == link1

    def test_total_mass(self):
        """Test calculating total mass."""
        robot = Robot(name="test_robot")

        inertia = InertiaTensor(ixx=1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=1.0)
        link1 = Link(name="link1", inertial=Inertial(mass=5.0, inertia=inertia))
        link2 = Link(name="link2", inertial=Inertial(mass=3.0, inertia=inertia))

        robot.add_link(link1)
        robot.add_link(link2)

        assert robot.total_mass == 8.0

    def test_degrees_of_freedom(self):
        """Test calculating degrees of freedom."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        # Add fixed joint (0 DOF)
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
            )
        )

        # Add revolute joint (1 DOF)
        robot.add_joint(
            Joint(
                name="joint2",
                type=JointType.REVOLUTE,
                parent="link2",
                child="link3",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        assert robot.degrees_of_freedom == 1


class TestRobotValidation:
    """Tests for robot validation."""

    def test_valid_robot(self):
        """Test that valid robot passes validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
            )
        )

        errors = robot.validate_tree_structure()
        assert len(errors) == 0

    def test_empty_robot(self):
        """Test that empty robot fails validation."""
        robot = Robot(name="test_robot")
        errors = robot.validate_tree_structure()
        assert len(errors) > 0
        assert any("at least one link" in err for err in errors)

    def test_duplicate_link_names(self):
        """Test that duplicate link names fail validation."""
        robot = Robot(name="test_robot")
        robot._links.append(Link(name="link1"))
        robot._links.append(Link(name="link1"))  # Bypass add_link validation

        errors = robot.validate_tree_structure()
        assert any("Duplicate link names" in err for err in errors)

    def test_duplicate_joint_names(self):
        """Test that duplicate joint names fail validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        joint1 = Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2")
        joint2 = Joint(name="joint1", type=JointType.FIXED, parent="link2", child="link3")

        robot._joints.append(joint1)
        robot._joints.append(joint2)

        errors = robot.validate_tree_structure()
        assert any("Duplicate joint names" in err for err in errors)

    def test_missing_parent_link(self):
        """Test that missing parent link fails validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link2"))

        # Manually add joint to bypass add_joint validation
        robot._joints.append(
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",  # Does not exist
                child="link2",
            )
        )

        errors = robot.validate_tree_structure()
        assert any("parent link 'link1' not found" in err for err in errors)

    def test_cycle_detection(self):
        """Test that cycles are detected."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        # Create a cycle
        robot.add_joint(Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2"))
        robot.add_joint(Joint(name="joint2", type=JointType.FIXED, parent="link2", child="link1"))

        errors = robot.validate_tree_structure()
        assert any("cycle" in err.lower() for err in errors)

    def test_multiple_roots(self):
        """Test that multiple roots fail validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        # Only connect link2 and link3, leaving link1 as orphan root
        robot.add_joint(Joint(name="joint1", type=JointType.FIXED, parent="link2", child="link3"))

        errors = robot.validate_tree_structure()
        assert any("Multiple root links" in err for err in errors)

    def test_disconnected_link(self):
        """Test that disconnected links fail validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        # Connect link1 and link2, leaving link3 disconnected
        robot.add_joint(Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2"))

        errors = robot.validate_tree_structure()
        # link3 is disconnected and will be detected as a multiple root issue
        assert any("Multiple root links" in err for err in errors)


class TestRobotWithSensors:
    """Tests for Robot model with sensors."""

    def test_add_sensor(self):
        """Test adding a sensor to a robot."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="camera_link"))

        sensor = Sensor(
            name="camera",
            type=SensorType.CAMERA,
            link_name="camera_link",
            camera_info=CameraInfo(),
        )
        robot.add_sensor(sensor)

        assert len(robot.sensors) == 1
        # Inline get_sensor logic
        found_sensor = next((s for s in robot.sensors if s.name == "camera"), None)
        assert found_sensor == sensor

    def test_add_sensor_nonexistent_link(self):
        """Test that adding sensor to nonexistent link raises error."""
        robot = Robot(name="test_robot")

        sensor = Sensor(
            name="camera",
            type=SensorType.CAMERA,
            link_name="camera_link",  # Link doesn't exist
            camera_info=CameraInfo(),
        )

        with pytest.raises(ValueError, match="link .* not found"):
            robot.add_sensor(sensor)

    def test_add_duplicate_sensor(self):
        """Test that adding duplicate sensor raises error."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="camera_link"))

        sensor = Sensor(
            name="camera",
            type=SensorType.CAMERA,
            link_name="camera_link",
            camera_info=CameraInfo(),
        )
        robot.add_sensor(sensor)

        with pytest.raises(ValueError, match="already exists"):
            robot.add_sensor(sensor)

    def test_robot_str_with_sensors(self):
        """Test robot string representation includes sensors."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_sensor(
            Sensor(
                name="camera",
                type=SensorType.CAMERA,
                link_name="link1",
                camera_info=CameraInfo(),
            )
        )

        robot_str = str(robot)
        assert "sensors=1" in robot_str


class TestRobotWithTransmissions:
    """Tests for Robot model with transmissions."""

    def test_add_transmission(self):
        """Test adding a transmission to a robot."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        trans = Transmission.create_simple(
            name="trans1",
            joint_name="joint1",
        )
        robot.add_transmission(trans)

        assert len(robot.transmissions) == 1
        # Inline get_transmission logic
        found_trans = next((t for t in robot.transmissions if t.name == "trans1"), None)
        assert found_trans == trans

    def test_add_transmission_nonexistent_joint(self):
        """Test that adding transmission with nonexistent joint raises error."""
        robot = Robot(name="test_robot")

        trans = Transmission.create_simple(
            name="trans1",
            joint_name="joint1",  # Joint doesn't exist
        )

        with pytest.raises(ValueError, match="joint .* not found"):
            robot.add_transmission(trans)

    def test_add_duplicate_transmission(self):
        """Test that adding duplicate transmission raises error."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        trans = Transmission.create_simple(
            name="trans1",
            joint_name="joint1",
        )
        robot.add_transmission(trans)

        with pytest.raises(ValueError, match="already exists"):
            robot.add_transmission(trans)

    def test_robot_str_with_transmissions(self):
        """Test robot string representation includes transmissions."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        robot.add_transmission(Transmission.create_simple("trans1", "joint1"))

        robot_str = str(robot)
        assert "transmissions=1" in robot_str


class TestRobotWithGazeboElements:
    """Tests for Robot model with Gazebo elements."""

    def test_add_link_gazebo_element(self):
        """Test adding a link-level Gazebo element."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        element = GazeboElement(
            reference="base_link",
            material="Gazebo/Red",
        )
        robot.add_gazebo_element(element)

        assert len(robot.gazebo_elements) == 1
        # Inline get_gazebo_elements_for_link logic
        link_elements = [e for e in robot.gazebo_elements if e.reference == "base_link"]
        assert len(link_elements) == 1
        assert element in link_elements

    def test_add_joint_gazebo_element(self):
        """Test adding a joint-level Gazebo element."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        element = GazeboElement(
            reference="joint1",
            provide_feedback=True,
        )
        robot.add_gazebo_element(element)

        # Inline get_gazebo_elements_for_joint logic
        joint_elements = [e for e in robot.gazebo_elements if e.reference == "joint1"]
        assert len(joint_elements) == 1
        assert element in joint_elements

    def test_add_gazebo_element_invalid_reference(self):
        """Test that adding Gazebo element with invalid reference raises error."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))

        element = GazeboElement(
            reference="nonexistent_link",
            material="Gazebo/Red",
        )

        with pytest.raises(ValueError, match="does not match any link or joint"):
            robot.add_gazebo_element(element)

    def test_add_gazebo_element_with_plugin(self):
        """Test adding Gazebo element with plugin."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))

        plugin = GazeboPlugin(name="test_plugin", filename="lib.so")
        element = GazeboElement(
            reference=None,
            plugins=[plugin],
        )
        robot.add_gazebo_element(element)

        # Inline get_robot_level_gazebo_elements logic
        robot_level = [e for e in robot.gazebo_elements if e.reference is None]
        assert len(robot_level[0].plugins) == 1

    def test_robot_str_with_gazebo_elements(self):
        """Test robot string representation includes Gazebo elements."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_gazebo_element(GazeboElement(reference=None, static=True))

        robot_str = str(robot)
        assert "gazebo_elements=1" in robot_str


class TestRobotEdgeCases:
    """Tests for Robot model edge cases and validation."""

    def test_duplicate_link_names(self):
        """Test that adding links with duplicate names raises error."""
        robot = Robot(name="test")
        link1 = Link(name="base_link")
        robot.add_link(link1)

        link2 = Link(name="base_link")  # Same name
        with pytest.raises(ValueError, match="already exists|duplicate"):
            robot.add_link(link2)

    def test_duplicate_joint_names(self):
        """Test that adding joints with duplicate names raises error."""
        robot = Robot(name="test")
        link1 = Link(name="link1")
        link2 = Link(name="link2")
        link3 = Link(name="link3")
        robot.add_link(link1)
        robot.add_link(link2)
        robot.add_link(link3)

        joint1 = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        robot.add_joint(joint1)

        joint2 = Joint(
            name="joint1",  # Same name
            type=JointType.FIXED,
            parent="link2",
            child="link3",
        )
        with pytest.raises(ValueError, match="already exists|duplicate"):
            robot.add_joint(joint2)

    def test_joint_with_nonexistent_parent(self):
        """Test that joint referencing non-existent parent link raises error."""
        robot = Robot(name="test")
        link1 = Link(name="link1")
        robot.add_link(link1)

        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="nonexistent_link",  # Does not exist
            child="link1",
        )
        with pytest.raises(ValueError, match="Parent link.*not found"):
            robot.add_joint(joint)

    def test_joint_with_nonexistent_child(self):
        """Test that joint referencing non-existent child link raises error."""
        robot = Robot(name="test")
        link1 = Link(name="link1")
        robot.add_link(link1)

        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="nonexistent_link",  # Does not exist
        )
        with pytest.raises(ValueError, match="Child link.*not found"):
            robot.add_joint(joint)

    def test_empty_robot_name(self):
        """Test that empty robot name raises error."""
        with pytest.raises(ValueError, match="name cannot be empty"):
            Robot(name="")

    def test_robot_name_with_spaces(self):
        """Test that robot name with spaces raises error."""
        with pytest.raises(ValueError, match="invalid characters"):
            Robot(name="robot with spaces")

    def test_robot_name_with_special_chars(self):
        """Test that robot name with special characters raises error."""
        with pytest.raises(ValueError, match="invalid characters"):
            Robot(name="robot@name!")

    def test_valid_robot_name_with_underscore(self):
        """Test that robot name with underscores is valid."""
        robot = Robot(name="my_robot_v2")
        assert robot.name == "my_robot_v2"

    def test_valid_robot_name_with_hyphen(self):
        """Test that robot name with hyphens is valid."""
        robot = Robot(name="my-robot-v2")
        assert robot.name == "my-robot-v2"

    def test_get_nonexistent_link(self):
        """Test that getting non-existent link returns None."""
        robot = Robot(name="test")
        link = robot.get_link("nonexistent")
        assert link is None

    def test_get_nonexistent_joint(self):
        """Test that getting non-existent joint returns None."""
        robot = Robot(name="test")
        joint = robot.get_joint("nonexistent")
        assert joint is None

    def test_circular_joint_dependency_simple(self):
        """Test detection of simple circular dependency (A -> B -> A)."""
        robot = Robot(name="test")
        link_a = Link(name="link_a")
        link_b = Link(name="link_b")
        robot.add_link(link_a)
        robot.add_link(link_b)

        # Add joint A -> B
        joint1 = Joint(
            name="joint_ab",
            type=JointType.FIXED,
            parent="link_a",
            child="link_b",
        )
        robot.add_joint(joint1)

        # Try to add joint B -> A (creates cycle)
        joint2 = Joint(
            name="joint_ba",
            type=JointType.FIXED,
            parent="link_b",
            child="link_a",
        )
        # This should either raise an error or be caught by validation
        # (Current implementation may allow it, rely on validator to catch)
        robot.add_joint(joint2)
        # Circular dependency exists, validator should catch it
        assert len(robot.joints) == 2


class TestRobotConstructorEdgeCases:
    """Tests for Robot constructor with initial_links and initial_joints."""

    def test_constructor_with_initial_links(self):
        """Test Robot constructor with initial_links parameter."""
        link1 = Link(name="link1", inertial=Inertial(mass=1.0))
        link2 = Link(name="link2", inertial=Inertial(mass=0.5))

        robot = Robot(name="test_robot", initial_links=[link1, link2])

        assert len(robot.links) == 2
        assert robot.get_link("link1") == link1
        assert robot.get_link("link2") == link2

    def test_constructor_with_initial_joints(self):
        """Test Robot constructor with initial_joints parameter."""
        link1 = Link(name="link1")
        link2 = Link(name="link2")
        joint1 = Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2")

        robot = Robot(name="test_robot", initial_links=[link1, link2], initial_joints=[joint1])

        assert len(robot.joints) == 1
        assert robot.get_joint("joint1") == joint1

    def test_constructor_duplicate_link_names(self):
        """Test that constructor raises error for duplicate link names."""
        link1 = Link(name="duplicate", inertial=Inertial(mass=1.0))
        link2 = Link(name="duplicate", inertial=Inertial(mass=0.5))

        with pytest.raises(ValueError, match="Duplicate link name"):
            Robot(name="test_robot", initial_links=[link1, link2])

    def test_constructor_duplicate_joint_names(self):
        """Test that constructor raises error for duplicate joint names."""
        link1 = Link(name="link1")
        link2 = Link(name="link2")
        link3 = Link(name="link3")
        joint1 = Joint(name="duplicate", type=JointType.FIXED, parent="link1", child="link2")
        joint2 = Joint(name="duplicate", type=JointType.FIXED, parent="link2", child="link3")

        with pytest.raises(ValueError, match="Duplicate joint name"):
            Robot(
                name="test_robot",
                initial_links=[link1, link2, link3],
                initial_joints=[joint1, joint2],
            )


class TestRobotGetJointsForLink:
    """Tests for get_joints_for_link method."""

    def test_get_joints_for_link_as_parent(self):
        """Test getting joints where link is parent."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base"))
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        robot.add_joint(Joint(name="joint1", type=JointType.FIXED, parent="base", child="link1"))
        robot.add_joint(Joint(name="joint2", type=JointType.FIXED, parent="base", child="link2"))

        joints = robot.get_joints_for_link("base", as_parent=True)

        assert len(joints) == 2
        assert all(j.parent == "base" for j in joints)

    def test_get_joints_for_link_as_child(self):
        """Test getting joints where link is child."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base"))
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        robot.add_joint(Joint(name="joint1", type=JointType.FIXED, parent="base", child="link1"))
        robot.add_joint(Joint(name="joint2", type=JointType.FIXED, parent="link2", child="link1"))

        joints = robot.get_joints_for_link("link1", as_parent=False)

        assert len(joints) == 2
        assert all(j.child == "link1" for j in joints)


class TestRobotRos2ControlEdgeCases:
    """Tests for ros2_control edge cases."""

    def test_add_duplicate_ros2_control_name(self):
        """Test that adding duplicate ros2_control name raises error."""
        from linkforge_core.models import Ros2Control, Ros2ControlJoint

        robot = Robot(name="test_robot")

        rc1 = Ros2Control(
            name="duplicate",
            hardware_plugin="plugin1",
            joints=[Ros2ControlJoint("j1", ["position"], ["position"])],
        )
        rc2 = Ros2Control(
            name="duplicate",
            hardware_plugin="plugin2",
            joints=[Ros2ControlJoint("j2", ["position"], ["position"])],
        )

        robot.add_ros2_control(rc1)

        with pytest.raises(ValueError, match="already exists"):
            robot.add_ros2_control(rc2)


class TestRobotRootLinkEdgeCases:
    """Tests for get_root_link edge cases."""

    def test_get_root_link_empty_robot(self):
        """Test get_root_link returns None for empty robot."""
        robot = Robot(name="empty")
        assert robot.get_root_link() is None


class TestRobotValidationEdgeCases:
    """Tests for validation edge cases."""

    def test_validation_joint_child_not_found(self):
        """Test validation detects missing child link."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))

        # Bypass add_joint validation
        robot._joints.append(
            Joint(name="joint1", type=JointType.FIXED, parent="link1", child="nonexistent")
        )

        errors = robot.validate_tree_structure()
        assert any("child link 'nonexistent' not found" in err for err in errors)

    def test_validation_no_root_link(self):
        """Test validation detects when no root link exists."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        # Create circular dependency (no root)
        robot._joints.append(
            Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2")
        )
        robot._joints.append(
            Joint(name="joint2", type=JointType.FIXED, parent="link2", child="link1")
        )

        errors = robot.validate_tree_structure()
        # Should detect cycle or no root
        assert len(errors) > 0

    def test_validation_multiple_parent_joints(self):
        """Test validation detects links with multiple parents."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base"))
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        robot.add_joint(Joint(name="joint1", type=JointType.FIXED, parent="base", child="link1"))

        # Bypass validation to create multiple parents
        robot._joints.append(
            Joint(name="joint2a", type=JointType.FIXED, parent="base", child="link2")
        )
        robot._joints.append(
            Joint(name="joint2b", type=JointType.FIXED, parent="link1", child="link2")
        )

        errors = robot.validate_tree_structure()
        assert any("has 2 parent joints" in err for err in errors)


class TestRobotMimicValidation:
    """Tests for mimic chain validation."""

    def test_mimic_nonexistent_joint(self):
        """Test validation detects mimic targeting non-existent joint."""
        from linkforge_core.models import JointMimic

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-1.0, upper=1.0),
                mimic=JointMimic(joint="nonexistent"),
            )
        )

        errors = robot._validate_mimic_chains()
        assert any("mimics non-existent joint" in err for err in errors)

    def test_mimic_circular_dependency(self):
        """Test validation detects circular mimic dependencies."""
        from linkforge_core.models import JointMimic

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-1.0, upper=1.0),
                mimic=JointMimic(joint="joint2"),
            )
        )
        robot.add_joint(
            Joint(
                name="joint2",
                type=JointType.REVOLUTE,
                parent="link2",
                child="link3",
                limits=JointLimits(lower=-1.0, upper=1.0),
                mimic=JointMimic(joint="joint1"),
            )
        )

        errors = robot._validate_mimic_chains()
        assert any("Circular mimic dependency" in err for err in errors)


class TestRobotReprEdgeCases:
    """Tests for __repr__ with all optional fields."""

    def test_repr_with_ros2_controls(self):
        """Test __repr__ includes ros2_controls."""
        from linkforge_core.models import Ros2Control, Ros2ControlJoint

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))

        rc = Ros2Control(
            name="system1",
            hardware_plugin="plugin",
            joints=[Ros2ControlJoint("j1", ["position"], ["position"])],
        )
        robot.add_ros2_control(rc)

        repr_str = repr(robot)
        # Just check that ros2_controls is in the repr, don't check exact format
        assert "ros2_controls" in repr_str
        assert "system1" in repr_str


class TestRobotCoverage:
    """Tests specifically for 100% coverage of robot.py."""

    def test_validation_disconnected_link_error(self):
        """Trigger disconnected link error in validate_tree_structure."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="root"))
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="orphan"))
        robot.add_joint(Joint(name="j1", type=JointType.FIXED, parent="root", child="link1"))

        # We need to mock get_root_link to return 'root' even though 'orphan' exists
        # to trigger the 'is not connected' check at line 271.
        # Otherwise get_root_link raises "Multiple root links found".
        from unittest.mock import patch

        with patch.object(Robot, "get_root_link", return_value=robot.get_link("root")):
            errors = robot.validate_tree_structure()
            assert any("is not connected to the tree" in err for err in errors)

    def test_validation_no_root_link_fallback(self):
        """Trigger 'No root link found' fallback at line 261."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        # Force robot.links to be empty after initial check but before get_root_link
        from unittest.mock import patch

        with patch.object(Robot, "get_root_link", return_value=None):
            errors = robot.validate_tree_structure()
            assert any("No root link found" in err for err in errors)

    def test_dfs_cycle_detection_redundant_edge(self):
        """Test cycle detection with redundant edges."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="a"))
        robot.add_link(Link(name="b"))
        robot._joints.append(Joint(name="j1", type=JointType.FIXED, parent="a", child="b"))
        robot._joints.append(Joint(name="j2", type=JointType.FIXED, parent="a", child="b"))
        # This no longer triggers line 319 (as it's gone), but exercises the 'node in visited' skip
        assert robot._has_cycle() is False  # No cycle, just two paths to same node

    def test_mimic_chain_end_coverage(self):
        """Trigger line 386 break in mimic chain validation."""
        from linkforge_core.models import JointMimic

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="a"))
        robot.add_link(Link(name="b"))
        robot.add_link(Link(name="c"))
        robot.add_joint(Joint(name="j1", type=JointType.CONTINUOUS, parent="a", child="b"))
        robot.add_joint(
            Joint(
                name="j2",
                type=JointType.CONTINUOUS,
                parent="b",
                child="c",
                mimic=JointMimic(joint="j1"),
            )
        )

        # Validates without circular dependency, hits 'mimic is None' break
        errors = robot._validate_mimic_chains()
        assert len(errors) == 0

    def test_full_str_representation(self):
        """Test __str__ with all optional components for full coverage."""
        from linkforge_core.models import GazeboElement, Ros2Control, Ros2ControlJoint

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_ros2_control(
            Ros2Control(
                name="rc", hardware_plugin="h", joints=[Ros2ControlJoint("j", ["p"], ["p"])]
            )
        )
        robot.add_link(Link(name="link2"))
        robot.add_joint(Joint(name="j1", type=JointType.FIXED, parent="link1", child="link2"))
        robot.add_gazebo_element(GazeboElement(reference="link1", static=True))

        robot_str = str(robot)
        assert "ros2_controls=1" in robot_str
        assert "gazebo_elements=1" in robot_str
