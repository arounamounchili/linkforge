from unittest.mock import patch

import pytest
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models import (
    CameraInfo,
    GazeboElement,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointMimic,
    JointType,
    Link,
    Robot,
    Ros2Control,
    SensorType,
    Vector3,
)
from linkforge_core.models.sensor import Sensor
from linkforge_core.models.transmission import Transmission, TransmissionJoint
from linkforge_core.validation import RobotValidator


class TestRobot:
    def test_robot_initialization(self):
        """Test basic robot initialization and index creation."""
        base = Link(name="base_link")
        link1 = Link(name="link1")

        joint1 = Joint(
            name="joint1",
            parent="base_link",
            child="link1",
            type=JointType.REVOLUTE,
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-1.0, upper=1.0, effort=10.0, velocity=1.0),
        )

        robot = Robot(name="test_robot", initial_links=[base, link1], initial_joints=[joint1])

        assert robot.name == "test_robot"
        assert len(robot.links) == 2
        assert len(robot.joints) == 1

        assert robot.get_link("base_link") is base
        assert robot.get_link("link1") is link1
        assert robot.get_joint("joint1") is joint1
        assert robot.get_link("non_existent") is None

    def test_invalid_names(self):
        """Test validation of robot names."""
        with pytest.raises(RobotModelError, match="cannot be empty"):
            Robot(name="")

        with pytest.raises(RobotModelError, match="invalid characters"):
            Robot(name="invalid name with spaces")

    def test_duplicate_components(self):
        """Test detection of duplicate links and joints."""
        link1 = Link(name="link1")
        link2 = Link(name="link1")

        with pytest.raises(RobotModelError, match="already exists"):
            Robot(name="test", initial_links=[link1, link2])

        joint1 = Joint(name="joint1", parent="base", child="link1", type=JointType.FIXED)
        joint2 = Joint(name="joint1", parent="base", child="link1", type=JointType.FIXED)

        with pytest.raises(RobotModelError, match="already exists"):
            Robot(
                name="test",
                initial_links=[Link(name="base"), Link(name="link1")],
                initial_joints=[joint1, joint2],
            )

    def test_graph_operations(self):
        """Test adding links and joints dynamically."""
        robot = Robot(name="test")

        base = Link(name="base")
        child = Link(name="child")
        robot.add_link(base)
        robot.add_link(child)

        assert len(robot.links) == 2
        assert robot.get_link("base") is base

        joint = Joint(name="joint1", parent="base", child="child", type=JointType.FIXED)
        robot.add_joint(joint)

        assert len(robot.joints) == 1
        assert robot.get_joint("joint1") is joint

        with pytest.raises(RobotModelError, match="already exists"):
            robot.add_link(Link(name="base"))

        with pytest.raises(RobotModelError, match="already exists"):
            robot.add_joint(
                Joint(name="joint1", parent="base", child="child", type=JointType.FIXED)
            )

        with pytest.raises(RobotModelError, match="not found"):
            robot.add_joint(
                Joint(name="j2", parent="base", child="missing_link", type=JointType.FIXED)
            )

    def test_cycle_detection(self):
        """Test detection of kinematic loops."""
        links = [Link(name="A"), Link(name="B"), Link(name="C")]
        joints = [
            Joint(name="j1", parent="A", child="B", type=JointType.FIXED),
            Joint(name="j2", parent="B", child="C", type=JointType.FIXED),
            Joint(name="j3", parent="C", child="A", type=JointType.FIXED),
        ]

        robot = Robot(name="cyclic_robot", initial_links=links, initial_joints=joints)
        result = RobotValidator().validate(robot)

        assert any("cycle" in e.message.lower() for e in result.errors)
        assert robot._has_cycle() is True

    def test_mimic_cycle_detection(self):
        """Test detection of circular mimic dependencies."""
        links = [Link(name="A"), Link(name="B")]
        limits = JointLimits(lower=-1.0, upper=1.0)
        j1 = Joint(
            name="j1",
            parent="A",
            child="B",
            type=JointType.REVOLUTE,
            axis=Vector3(1.0, 0.0, 0.0),
            limits=limits,
            mimic=JointMimic(joint="j2"),
        )
        j2 = Joint(
            name="j2",
            parent="B",
            child="A",
            type=JointType.REVOLUTE,
            axis=Vector3(1.0, 0.0, 0.0),
            limits=limits,
            mimic=JointMimic(joint="j1"),
        )

        robot = Robot(name="mimic_cycle", initial_links=links, initial_joints=[j1, j2])
        result = RobotValidator().validate(robot)

        assert any("Circular mimic dependency" in e.message for e in result.errors)

    def test_mimic_missing_joint(self):
        """Test validation of mimic pointing to non-existent joint."""
        links = [Link(name="A"), Link(name="B")]

        j1 = Joint(
            name="j1",
            parent="A",
            child="B",
            type=JointType.REVOLUTE,
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-1.0, upper=1.0),
            mimic=JointMimic(joint="missing_joint"),
        )

        robot = Robot(name="mimic_missing", initial_links=links, initial_joints=[j1])
        result = RobotValidator().validate(robot)

        assert any("mimics non-existent joint" in e.message for e in result.errors)

    def test_root_link_identification(self):
        """Test identification of the root link."""
        links = [Link(name="base"), Link(name="mid"), Link(name="tip")]
        joints = [
            Joint(name="j1", parent="base", child="mid", type=JointType.FIXED),
            Joint(name="j2", parent="mid", child="tip", type=JointType.FIXED),
        ]

        robot = Robot(name="arm", initial_links=links, initial_joints=joints)
        assert robot.get_root_link().name == "base"

        # Test validation error when no root exists (all links are children)
        j3 = Joint(name="j3", parent="mid", child="base", type=JointType.FIXED)
        robot_cycle = Robot(
            name="cyclic",
            initial_links=[Link(name="base"), Link(name="mid")],
            initial_joints=[j3, joints[0]],
        )

        with pytest.raises(RobotModelError, match="No root link found"):
            robot_cycle.get_root_link()

    def test_disconnected_component(self):
        """Test detection of disconnected parts of the graph."""
        links = [Link(name="root"), Link(name="connected"), Link(name="floating")]
        joints = [Joint(name="j1", parent="root", child="connected", type=JointType.FIXED)]

        robot = Robot(name="disconnected", initial_links=links, initial_joints=joints)
        result = RobotValidator().validate(robot)

        assert any("Multiple root links found" in e.message for e in result.errors)

    def test_add_sensor_validation(self):
        """Test validation when adding sensors."""
        robot = Robot(name="test", initial_links=[Link(name="base")])
        sensor = Sensor(
            name="cam1", link_name="base", type=SensorType.CAMERA, camera_info=CameraInfo()
        )
        robot.add_sensor(sensor)
        assert robot.sensors[0] == sensor

        with pytest.raises(RobotModelError, match="link 'missing' not found"):
            robot.add_sensor(
                Sensor(
                    name="cam2",
                    link_name="missing",
                    type=SensorType.CAMERA,
                    camera_info=CameraInfo(),
                )
            )

        with pytest.raises(RobotModelError, match="already exists"):
            robot.add_sensor(
                Sensor(
                    name="cam1", link_name="base", type=SensorType.CAMERA, camera_info=CameraInfo()
                )
            )

    def test_add_transmission_validation(self):
        """Test validation when adding transmissions."""
        robot = Robot(
            name="test",
            initial_links=[Link(name="base"), Link(name="child")],
            initial_joints=[Joint(name="j1", parent="base", child="child", type=JointType.FIXED)],
        )

        trans = Transmission(
            name="t1",
            type="SimpleTransmission",
            joints=[TransmissionJoint(name="j1", hardware_interfaces=["position"])],
        )
        robot.add_transmission(trans)
        assert robot.transmissions[0] == trans

        bad_trans = Transmission(
            name="t2",
            type="SimpleTransmission",
            joints=[TransmissionJoint(name="missing_joint", hardware_interfaces=["position"])],
        )

        with pytest.raises(RobotModelError, match="joint 'missing_joint' not found"):
            robot.add_transmission(bad_trans)

    def test_robot_properties(self):
        """Test robot properties like mass and DOF."""
        # Create a robot with 2 links and 1 joint
        # Fix: Inertial takes float mass, not Mass object

        base = Link(name="base", inertial=None)  # Mass 0

        inertial = Inertial(
            mass=5.0, inertia=InertiaTensor(ixx=1, ixy=0, ixz=0, iyy=1, iyz=0, izz=1)
        )
        child = Link(name="child", inertial=inertial)

        from linkforge_core.models import JointLimits

        # Revolute joint (1 DOF)
        j1 = Joint(
            name="j1",
            parent="base",
            child="child",
            type=JointType.REVOLUTE,
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-1, upper=1),
        )

        robot = Robot(name="test_props", initial_links=[base, child], initial_joints=[j1])

        assert robot.total_mass == 5.0
        assert robot.degrees_of_freedom == 1

        # Add fixed joint (0 DOF)
        tip = Link(name="tip")
        j2 = Joint(name="j2", parent="child", child="tip", type=JointType.FIXED)
        robot.add_link(tip)
        robot.add_joint(j2)

        assert robot.degrees_of_freedom == 1  # Still 1

        # Verify read-only views
        assert len(robot.links) == 3
        assert len(robot.joints) == 2
        assert isinstance(robot.links, tuple)
        assert isinstance(robot.joints, tuple)

    def test_add_gazebo_element(self):
        """Test adding Gazebo-specific elements."""
        robot = Robot(name="test", initial_links=[Link(name="base")])

        # Valid element without reference
        elem1 = GazeboElement(material="Gazebo/Blue")
        robot.add_gazebo_element(elem1)
        assert len(robot.gazebo_elements) == 1

        # Valid element with reference
        elem2 = GazeboElement(reference="base", material="Gazebo/Red")
        robot.add_gazebo_element(elem2)
        assert len(robot.gazebo_elements) == 2

        # Invalid reference
        with pytest.raises(RobotModelError, match="does not match any link or joint"):
            robot.add_gazebo_element(GazeboElement(reference="missing"))

    def test_add_ros2_control(self):
        """Test adding ROS2 Control configurations."""
        robot = Robot(name="test")
        ros2_ctrl = Ros2Control(
            name="System", type="system", hardware_plugin="mock_plugin", joints=[]
        )
        robot.add_ros2_control(ros2_ctrl)
        assert len(robot.ros2_controls) == 1

        # Duplicate name check
        with pytest.raises(RobotModelError, match="already exists"):
            robot.add_ros2_control(ros2_ctrl)

    def test_string_representation(self):
        """Test __str__ method completeness."""
        robot = Robot(name="full_bot", initial_links=[Link(name="base")])

        # Basic
        assert "Robot(name=full_bot" in str(robot)
        assert "links=1" in str(robot)

        robot.add_sensor(
            Sensor(name="cam", link_name="base", type=SensorType.CAMERA, camera_info=CameraInfo())
        )

        # Fix: Transmission requires joints, and add_transmission checks existence
        j1 = Joint(name="j1", parent="base", child="l2", type=JointType.FIXED)
        l2 = Link(name="l2")
        robot.add_link(l2)
        robot.add_joint(j1)

        trans = Transmission(
            name="t1",
            type="Simple",
            joints=[TransmissionJoint(name="j1", hardware_interfaces=["position"])],
        )

        robot.add_transmission(trans)

        robot.add_ros2_control(
            Ros2Control(name="ctrl", type="system", hardware_plugin="mock", joints=[])
        )
        robot.add_gazebo_element(GazeboElement(reference="base"))

        s = str(robot)
        assert "sensors=1" in s
        assert "transmissions=1" in s
        assert "ros2_controls=1" in s


class TestRobotCoverage:
    """Additional tests to ensure 100% coverage of edge cases."""

    def test_add_joint_parent_not_found(self):
        robot = Robot(name="test")
        l1 = Link(name="l1")
        robot.add_link(l1)

        # Parent "missing" does not exist
        j1 = Joint(name="j1", type=JointType.FIXED, parent="missing", child="l1")
        with pytest.raises(RobotModelError, match="Parent link 'missing' not found"):
            robot.add_joint(j1)

    def test_add_joint_child_not_found(self):
        robot = Robot(name="test")
        l1 = Link(name="l1")
        robot.add_link(l1)

        # Child "missing" does not exist
        j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="missing")
        with pytest.raises(RobotModelError, match="Child link 'missing' not found"):
            robot.add_joint(j1)

    def test_get_joints_for_link(self):
        robot = Robot(name="test")
        l1 = Link(name="l1")
        l2 = Link(name="l2")
        l3 = Link(name="l3")
        robot.add_link(l1)
        robot.add_link(l2)
        robot.add_link(l3)

        j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="l2")
        j2 = Joint(name="j2", type=JointType.FIXED, parent="l2", child="l3")
        robot.add_joint(j1)
        robot.add_joint(j2)

        # l1 is parent in j1
        assert robot.get_joints_for_link("l1", as_parent=True) == [j1]
        assert robot.get_joints_for_link("l1", as_parent=False) == []

        # l2 is child in j1, parent in j2
        assert robot.get_joints_for_link("l2", as_parent=True) == [j2]
        assert robot.get_joints_for_link("l2", as_parent=False) == [j1]

    def test_add_transmission_duplicate(self):
        robot = Robot(name="test")
        # Need existing joints for transmission
        l1 = Link(name="l1")
        l2 = Link(name="l2")
        robot.add_link(l1)
        robot.add_link(l2)
        j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="l2")
        robot.add_joint(j1)

        t1 = Transmission(name="t1", type="SimpleTransmission", joints=[j1])
        robot.add_transmission(t1)

        with pytest.raises(RobotModelError, match="Transmission 't1' already exists"):
            robot.add_transmission(t1)

    def test_get_root_link_empty(self):
        robot = Robot(name="test")
        assert robot.get_root_link() is None

    def test_validate_tree_structure_duplicate_names_mock(self):
        # To test duplicate names logic in validate_tree_structure,
        # we need to bypass add_link/add_joint checks which prevent duplicates.
        robot = Robot(name="test")
        l1 = Link(name="l1")
        robot._links.append(l1)
        robot._links.append(l1)  # Duplicate!

        result = RobotValidator().validate(robot)
        assert any("Duplicate link name" in e.title for e in result.errors)

        robot = Robot(name="test2")
        l1 = Link(name="l1")
        l2 = Link(name="l2")
        robot.add_link(l1)
        robot.add_link(l2)

        j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="l2")
        robot._joints.append(j1)
        robot._joints.append(j1)  # Duplicate!

        result = RobotValidator().validate(robot)
        assert any("Duplicate joint name" in e.title for e in result.errors)

    def test_validate_tree_structure_missing_child_mock(self):
        # Bypass add_joint check
        robot = Robot(name="test")
        l1 = Link(name="l1")
        robot.add_link(l1)

        j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="missing")
        robot._joints.append(j1)

        result = RobotValidator().validate(robot)
        assert any("Missing child link" in e.title for e in result.errors)

    def test_validate_tree_structure_root_none_mock(self):
        # Mock get_root_link to return None even if links exist
        robot = Robot(name="test")
        l1 = Link(name="l1")
        robot.add_link(l1)

        with patch.object(robot, "get_root_link", return_value=None):
            result = RobotValidator().validate(robot)
            assert any("No root link found" in e.message for e in result.errors)

    def test_validate_tree_structure_graph_error_mock(self):
        # Test the 'except RobotModelError' block in RobotValidator
        robot = Robot(name="test")
        robot.add_link(Link(name="l1"))

        with patch.object(
            robot, "_has_cycle", side_effect=RobotModelError("Unexpected graph error")
        ):
            result = RobotValidator().validate(robot)
            assert any("Kinematic graph error" in e.title for e in result.errors)
            assert any("Unexpected graph error" in e.message for e in result.errors)

    def test_validate_tree_structure_disconnected_and_multi_parent(self):
        robot = Robot(name="test")
        l1 = Link(name="l1")  # Root
        l2 = Link(name="l2")  # Disconnected (another root effectively)
        l3 = Link(name="l3")  # Multi-parent
        l4 = Link(name="l4")

        robot.add_link(l1)
        robot.add_link(l2)
        robot.add_link(l3)
        robot.add_link(l4)

        # l1 -> l4
        j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="l4")
        robot.add_joint(j1)

        # l3 has 2 parents: l1->l3 and l4->l3
        j2 = Joint(name="j2", type=JointType.FIXED, parent="l1", child="l3")
        j3 = Joint(name="j3", type=JointType.FIXED, parent="l4", child="l3")
        robot.add_joint(j2)
        robot.add_joint(j3)

        # Mock get_root_link up to return l1 (ignoring l2 as second root)
        with patch.object(robot, "get_root_link", return_value=l1):
            result = RobotValidator().validate(robot)

            # l2 is disconnected (count=0, != root)
            assert any("Link 'l2' is not connected" in e.message for e in result.errors)

            # l3 has 2 parents
            assert any("Link 'l3' has 2 parent joints" in e.message for e in result.errors)

    def test_mimic_chain_valid_break(self):
        # Test a mimic chain that ends properly (hitting break)

        robot = Robot(name="test")
        l1 = Link(name="l1")
        l2 = Link(name="l2")
        l3 = Link(name="l3")
        robot.add_link(l1)
        robot.add_link(l2)
        robot.add_link(l3)

        # j1 mimics j2. j2 mimics nothing.
        j2 = Joint(
            name="j2",
            type=JointType.REVOLUTE,
            parent="l1",
            child="l2",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=0, upper=1),
        )
        j1 = Joint(
            name="j1",
            type=JointType.REVOLUTE,
            parent="l2",
            child="l3",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=0, upper=1),
            mimic=JointMimic(joint="j2"),
        )

        robot.add_joint(j2)
        robot.add_joint(j1)

        result = RobotValidator().validate(robot)
        assert result.is_valid

    def test_kinematic_graph_caching(self):
        """Test that the kinematic graph is cached and correctly invalidated."""
        robot = Robot(name="cache_test")
        robot.add_link(Link(name="base"))
        robot.add_link(Link(name="child"))
        robot.add_joint(Joint(name="j1", parent="base", child="child", type=JointType.FIXED))

        # First access builds the graph
        graph1 = robot.graph
        assert graph1 is not None

        # Second access should return the same instance (cached)
        graph2 = robot.graph
        assert graph2 is graph1

        # Adding a link should invalidate the cache
        robot.add_link(Link(name="new_link"))
        graph3 = robot.graph
        assert graph3 is not graph1

        # Accessing again should be cached again
        graph4 = robot.graph
        assert graph4 is graph3

        # Adding a joint should invalidate the cache
        robot.add_joint(Joint(name="j2", parent="child", child="new_link", type=JointType.FIXED))
        graph5 = robot.graph
        assert graph5 is not graph4

    def test_robot_encapsulation(self):
        """Test that internal collections are protected and read-only."""
        robot = Robot(name="encap_test")

        # Test links collection
        robot.add_link(Link(name="l1"))
        links = robot.links
        assert isinstance(links, tuple)

        # Attempting to modify the tuple should raise TypeError
        with pytest.raises(TypeError):
            links[0] = Link(name="cheat")  # type: ignore

        # Verify that robot.links doesn't change if we try to modify the returned tuple
        assert len(robot.links) == 1
        assert robot.links[0].name == "l1"

        # Check other collections
        assert isinstance(robot.sensors, tuple)
        assert isinstance(robot.transmissions, tuple)
        assert isinstance(robot.ros2_controls, tuple)
        assert isinstance(robot.gazebo_elements, tuple)

    def test_robot_duplicate_initial_components_gap_fill(self):
        """Test detection of duplicate initial components in post-init."""
        robot = Robot(name="test")
        link1 = Link(name="l1")
        robot._links = [link1, link1]
        with pytest.raises(RobotModelError, match="Duplicate link name"):
            robot.__post_init__(None, None)

        robot = Robot(name="test")
        joint1 = Joint(name="j1", parent="a", child="b", type=JointType.FIXED)
        robot._joints = [joint1, joint1]
        with pytest.raises(RobotModelError, match="Duplicate joint name"):
            robot.__post_init__(None, None)
