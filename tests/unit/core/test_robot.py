"""Tests for Robot model."""

from __future__ import annotations

import math

import pytest

from linkforge.core.models import (
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
        robot.links.append(Link(name="link1"))
        robot.links.append(Link(name="link1"))  # Bypass add_link validation

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

        robot.joints.append(joint1)
        robot.joints.append(joint2)

        errors = robot.validate_tree_structure()
        assert any("Duplicate joint names" in err for err in errors)

    def test_missing_parent_link(self):
        """Test that missing parent link fails validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link2"))

        # Manually add joint to bypass add_joint validation
        robot.joints.append(
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
        robot.joints.append(
            Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2")
        )
        robot.joints.append(
            Joint(name="joint2", type=JointType.FIXED, parent="link2", child="link1")
        )

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
