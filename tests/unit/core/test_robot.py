from unittest.mock import PropertyMock, patch

import pytest
from linkforge_core.exceptions import RobotModelError, RobotValidationError
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
from linkforge_core.models.transmission import (
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    TransmissionType,
)
from linkforge_core.validation import RobotValidator


class TestRobot:
    def test_robot_initialization(self) -> None:
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

    def test_invalid_names(self) -> None:
        """Test validation of robot names."""
        with pytest.raises(RobotModelError):
            Robot(name="")

        with pytest.raises(RobotModelError, match="Invalid name format"):
            Robot(name="invalid name with spaces")

    def test_duplicate_components(self) -> None:
        """Test detection of duplicate links and joints."""
        link1 = Link(name="link1")
        link2 = Link(name="link1")

        with pytest.raises(RobotModelError, match="Already exists"):
            Robot(name="test", initial_links=[link1, link2])

        joint1 = Joint(name="joint1", parent="base", child="link1", type=JointType.FIXED)
        joint2 = Joint(name="joint1", parent="base", child="link1", type=JointType.FIXED)

        with pytest.raises(RobotModelError, match="Already exists"):
            Robot(
                name="test",
                initial_links=[Link(name="base"), Link(name="link1")],
                initial_joints=[joint1, joint2],
            )

    def test_graph_operations(self) -> None:
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

        with pytest.raises(RobotModelError, match="Already exists"):
            robot.add_link(Link(name="base"))

        with pytest.raises(RobotModelError, match="Already exists"):
            robot.add_joint(
                Joint(name="joint1", parent="base", child="child", type=JointType.FIXED)
            )

        with pytest.raises(RobotModelError, match="Not found"):
            robot.add_joint(
                Joint(name="j2", parent="base", child="missing_link", type=JointType.FIXED)
            )

    def test_cycle_detection(self) -> None:
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
        assert robot.has_cycle is True

    def test_mimic_cycle_detection(self) -> None:
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

    def test_mimic_missing_joint(self) -> None:
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

    def test_root_link_identification(self) -> None:
        """Test identification of the root link."""
        links = [Link(name="base"), Link(name="mid"), Link(name="tip")]
        joints = [
            Joint(name="j1", parent="base", child="mid", type=JointType.FIXED),
            Joint(name="j2", parent="mid", child="tip", type=JointType.FIXED),
        ]

        robot = Robot(name="arm", initial_links=links, initial_joints=joints)
        assert robot.root_link.name == "base"

        # Test validation error when no root exists (all links are children)
        j3 = Joint(name="j3", parent="mid", child="base", type=JointType.FIXED)
        robot_cycle = Robot(
            name="cyclic",
            initial_links=[Link(name="base"), Link(name="mid")],
            initial_joints=[j3, joints[0]],
        )

        with pytest.raises(RobotModelError, match="No root link found"):
            _ = robot_cycle.root_link

    def test_disconnected_component(self) -> None:
        """Test detection of disconnected parts of the graph."""
        links = [Link(name="root"), Link(name="connected"), Link(name="floating")]
        joints = [Joint(name="j1", parent="root", child="connected", type=JointType.FIXED)]

        robot = Robot(name="disconnected", initial_links=links, initial_joints=joints)
        result = RobotValidator().validate(robot)

        assert any("Multiple root links found" in e.message for e in result.errors)

    def test_add_sensor_validation(self) -> None:
        """Test validation when adding sensors."""
        robot = Robot(name="test", initial_links=[Link(name="base")])
        sensor = Sensor(
            name="cam1", link_name="base", type=SensorType.CAMERA, camera_info=CameraInfo()
        )
        robot.add_sensor(sensor)
        assert robot.sensors[0] == sensor

        with pytest.raises(RobotModelError, match="Not found"):
            robot.add_sensor(
                Sensor(
                    name="cam2",
                    link_name="missing",
                    type=SensorType.CAMERA,
                    camera_info=CameraInfo(),
                )
            )

        with pytest.raises(RobotModelError, match="Already exists"):
            robot.add_sensor(
                Sensor(
                    name="cam1", link_name="base", type=SensorType.CAMERA, camera_info=CameraInfo()
                )
            )

    def test_add_transmission_validation(self) -> None:
        """Test validation when adding transmissions."""
        robot = Robot(
            name="test",
            initial_links=[Link(name="base"), Link(name="child")],
            initial_joints=[Joint(name="j1", parent="base", child="child", type=JointType.FIXED)],
        )

        trans = Transmission(
            name="t1",
            type=TransmissionType.SIMPLE,
            joints=[TransmissionJoint(name="j1", hardware_interfaces=["position"])],
            actuators=[TransmissionActuator(name="a1")],
        )
        robot.add_transmission(trans)
        assert robot.transmissions[0] == trans

        bad_trans = Transmission(
            name="t2",
            type=TransmissionType.SIMPLE,
            joints=[TransmissionJoint(name="missing_joint", hardware_interfaces=["position"])],
            actuators=[TransmissionActuator(name="a1")],
        )

        with pytest.raises(RobotModelError):
            robot.add_transmission(bad_trans)

    def test_robot_properties(self) -> None:
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

    def test_add_gazebo_element(self) -> None:
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
        with pytest.raises(RobotModelError):
            robot.add_gazebo_element(GazeboElement(reference="missing"))

    def test_add_ros2_control(self) -> None:
        """Test adding ROS2 Control configurations."""
        robot = Robot(name="test")
        ros2_ctrl = Ros2Control(
            name="System", type="system", hardware_plugin="mock_plugin", joints=[]
        )
        robot.add_ros2_control(ros2_ctrl)
        assert len(robot.ros2_controls) == 1

        # Duplicate name check
        with pytest.raises(RobotModelError, match="Already exists"):
            robot.add_ros2_control(ros2_ctrl)

    def test_string_representation(self) -> None:
        """Test the simplified, lightweight __str__ method."""
        robot = Robot(name="full_bot", initial_links=[Link(name="base")])
        s = str(robot)

        assert "Robot(name=full_bot" in s
        assert "links=1" in s
        assert "joints=0" in s
        assert "dof=0" in s

        # Verify it doesn't contain heavy diagnostic info anymore
        assert "sensors=" not in s
        assert "root=" not in s

    def test_robot_summary(self) -> None:
        """Test the detailed architectural summary() method."""
        robot = Robot(name="full_bot", initial_links=[Link(name="base")])

        # Add components to make the summary interesting
        robot.add_sensor(
            Sensor(name="cam", link_name="base", type=SensorType.CAMERA, camera_info=CameraInfo())
        )

        j1 = Joint(name="j1", parent="base", child="l2", type=JointType.FIXED)
        l2 = Link(name="l2")
        robot.add_link(l2)
        robot.add_joint(j1)

        summary = robot.summary()

        assert "Robot Summary: full_bot" in summary
        assert "Status: VALID" in summary
        assert "Root: base" in summary
        assert "Topology: 2 links, 1 joints" in summary
        assert "Functional: 1 sensors" in summary

        # Test invalid state summary
        # Add a cycle to make it invalid
        j2 = Joint(name="j2", parent="l2", child="base", type=JointType.FIXED)
        robot._joints.append(j2)  # Bypass adder validation for testing
        robot._reindex()

        invalid_summary = robot.summary()
        assert "Status: INVALID" in invalid_summary

    # Edge Cases and Structural Validation

    def test_add_joint_parent_not_found(self) -> None:
        robot = Robot(name="test")
        l1 = Link(name="l1")
        robot.add_link(l1)

        # Parent "missing" does not exist
        j1 = Joint(name="j1", type=JointType.FIXED, parent="missing", child="l1")
        with pytest.raises(RobotModelError, match="Not found"):
            robot.add_joint(j1)

    def test_add_joint_child_not_found(self) -> None:
        robot = Robot(name="test")
        l1 = Link(name="l1")
        robot.add_link(l1)

        # Child "missing" does not exist
        j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="missing")
        with pytest.raises(RobotModelError, match="Not found"):
            robot.add_joint(j1)

    def test_get_joints_for_link(self) -> None:
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

    def test_add_transmission_duplicate(self) -> None:
        robot = Robot(name="test")
        # Need existing joints for transmission
        l1 = Link(name="l1")
        l2 = Link(name="l2")
        robot.add_link(l1)
        robot.add_link(l2)
        j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="l2")
        robot.add_joint(j1)

        t1 = Transmission(
            name="t1",
            type=TransmissionType.SIMPLE,
            joints=[TransmissionJoint(name=j1.name)],
            actuators=[TransmissionActuator(name="a1")],
        )
        robot.add_transmission(t1)

        with pytest.raises(RobotModelError, match="Already exists"):
            robot.add_transmission(t1)

    def test_root_link_empty(self) -> None:
        robot = Robot(name="test")
        from linkforge_core.exceptions import RobotValidationError, ValidationErrorCode

        with pytest.raises(RobotValidationError) as exc:
            _ = robot.root_link
        assert exc.value.code == ValidationErrorCode.NO_ROOT

    def test_validate_tree_structure_duplicate_names_mock(self) -> None:
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

    def test_validate_tree_structure_missing_child_mock(self) -> None:
        # Bypass add_joint check
        robot = Robot(name="test")
        l1 = Link(name="l1")
        robot.add_link(l1)

        j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="missing")
        robot._joints.append(j1)

        result = RobotValidator().validate(robot)
        assert any("Missing child link" in e.title for e in result.errors)

    def test_validate_tree_structure_root_none_mock(self) -> None:
        # Mock get_root_link to raise NO_ROOT error even if links exist
        robot = Robot(name="test")
        l1 = Link(name="l1")
        robot.add_link(l1)

        from linkforge_core.exceptions import RobotValidationError, ValidationErrorCode

        error = RobotValidationError(ValidationErrorCode.NO_ROOT, "No root link found")
        with patch.object(Robot, "root_link", new_callable=PropertyMock, side_effect=error):
            result = RobotValidator().validate(robot)
            assert any("No root link found" in e.message for e in result.errors)

    def test_validate_tree_structure_graph_error_mock(self) -> None:
        # Test the 'except RobotModelError' block in RobotValidator
        robot = Robot(name="test")
        robot.add_link(Link(name="l1"))

        with patch.object(
            Robot,
            "has_cycle",
            new_callable=PropertyMock,
            side_effect=RobotModelError("Unexpected graph error"),
        ):
            result = RobotValidator().validate(robot)
            assert any("Kinematic graph error" in e.title for e in result.errors)
            assert any("Unexpected graph error" in e.message for e in result.errors)

    def test_validate_tree_structure_disconnected_and_multi_parent(self) -> None:
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
        with patch.object(Robot, "root_link", new_callable=PropertyMock, return_value=l1):
            result = RobotValidator().validate(robot)

            # l2 is disconnected (count=0, != root)
            assert any("Link 'l2' is not connected" in e.message for e in result.errors)

            # l3 has 2 parents
            assert any("Link 'l3' has 2 parent joints" in e.message for e in result.errors)

    def test_mimic_chain_valid_break(self) -> None:
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

    def test_kinematic_graph_caching(self) -> None:
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

    def test_robot_encapsulation(self) -> None:
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

    def test_robot_duplicate_initial_components_gap_fill(self) -> None:
        """Test detection of duplicate initial components during initialization."""
        link1 = Link(name="l1")
        # add_link inside __post_init__ will catch duplicates
        with pytest.raises(RobotModelError, match="Already exists"):
            Robot(name="test", initial_links=[link1, link1])

        joint1 = Joint(name="j1", parent="a", child="b", type=JointType.FIXED)
        with pytest.raises(RobotModelError, match="Already exists"):
            Robot(
                name="test",
                initial_links=[Link(name="a"), Link(name="b")],
                initial_joints=[joint1, joint1],
            )

    def test_sensor_accessors(self) -> None:
        """Test the new sensor accessor methods (get_sensor, sensor, has_sensor)."""
        robot = Robot(name="test", initial_links=[Link(name="base")])
        sensor = Sensor(
            name="cam1", link_name="base", type=SensorType.CAMERA, camera_info=CameraInfo()
        )
        robot.add_sensor(sensor)

        assert robot.has_sensor("cam1") is True
        assert robot.has_sensor("missing") is False
        assert robot.get_sensor("cam1") is sensor
        assert robot.get_sensor("missing") is None
        assert robot.sensor("cam1") is sensor
        with pytest.raises(RobotModelError, match="not found"):
            robot.sensor("missing")

    def test_prefix_all_identity_sync(self) -> None:
        """Test that prefix_all correctly namespaces the robot name and semantic data."""
        robot = Robot(name="ur5", initial_links=[Link(name="base")])
        robot.prefix_all("left_")

        assert robot.name == "left_ur5"
        assert robot.links[0].name == "left_base"
        assert robot._semantic.robot_name == "left_ur5"

    def test_reindex_integrity(self) -> None:
        """Test that _reindex correctly rebuilds maps after manual mutation."""
        robot = Robot(name="test", initial_links=[Link(name="base")])
        sensor = Sensor(
            name="s1", link_name="base", type=SensorType.CAMERA, camera_info=CameraInfo()
        )
        # Bypass add_sensor and verify reindex picks it up
        robot._sensors.append(sensor)
        assert not robot.has_sensor("s1")
        robot._reindex()
        assert robot.has_sensor("s1")
        assert robot.sensor("s1") is sensor

    def test_traversal_helpers(self) -> None:
        """Test the high-level kinematic traversal helper methods."""
        # Setup: root -> j1 -> mid -> j2 -> tip
        root = Link(name="root")
        mid = Link(name="mid")
        tip = Link(name="tip")
        j1 = Joint(name="j1", parent="root", child="mid", type=JointType.FIXED)
        j2 = Joint(name="j2", parent="mid", child="tip", type=JointType.FIXED)

        robot = Robot(name="test", initial_links=[root, mid, tip], initial_joints=[j1, j2])

        # Test Parent Joint lookups
        assert robot.get_parent_joint("root") is None
        assert robot.get_parent_joint("mid") == j1
        assert robot.get_parent_joint("tip") == j2

        # Test Child Joints lookups
        assert robot.get_child_joints("root") == [j1]
        assert robot.get_child_joints("mid") == [j2]
        assert robot.get_child_joints("tip") == []

        # Test Parent Link lookups
        assert robot.get_parent_link("root") is None
        assert robot.get_parent_link("mid") == root
        assert robot.get_parent_link("tip") == mid

        # Test Child Links lookups
        assert robot.get_child_links("root") == [mid]
        assert robot.get_child_links("mid") == [tip]
        assert robot.get_child_links("tip") == []

    def test_transmission_and_ros2_control_accessors(self) -> None:
        """Test high-performance accessors for transmissions and ROS2 control."""
        robot = Robot(name="test")

        # Setup dependencies (links and joints)
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        # Add joints that will be referenced by transmission and ros2_control
        robot.add_joint(
            Joint(
                name="joint1",
                parent="link1",
                child="link2",
                type=JointType.REVOLUTE,
                axis=Vector3(1, 0, 0),
                limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
            )
        )
        robot.add_joint(Joint(name="joint2", parent="link2", child="link1", type=JointType.FIXED))

        # Setup Transmission and Ros2Control
        from linkforge_core.models.ros2_control import Ros2Control
        from linkforge_core.models.transmission import Transmission

        # Use valid simple transmission
        trans = Transmission.create_simple(
            name="trans1", joint_name="joint1", actuator_name="motor1"
        )

        # Use valid Ros2Control
        rc = Ros2Control(name="ctrl1", hardware_plugin="fake_hardware", type="system")

        robot.add_transmission(trans)
        robot.add_ros2_control(rc)

        # Test Accessors
        assert robot.has_transmission("trans1") is True
        assert robot.get_transmission("trans1") == trans
        assert robot.transmission("trans1") == trans

        assert robot.has_ros2_control("ctrl1") is True
        assert robot.get_ros2_control("ctrl1") == rc
        assert robot.ros2_control("ctrl1") == rc

        # Test Non-existent
        assert robot.has_transmission("ghost") is False
        assert robot.get_transmission("ghost") is None
        with pytest.raises(RobotValidationError, match="Transmission 'ghost' not found"):
            robot.transmission("ghost")

        # Test Duplicate Prevention
        with pytest.raises(RobotValidationError, match="Already exists: Transmission"):
            robot.add_transmission(trans)

        with pytest.raises(RobotValidationError, match="Already exists: ROS2 control"):
            robot.add_ros2_control(rc)

        # Test Joint Existence Validation in ROS2 control
        from linkforge_core.models.ros2_control import Ros2ControlJoint

        rc_invalid = Ros2Control(
            name="invalid_ctrl",
            hardware_plugin="fake",
            joints=[Ros2ControlJoint(name="missing_joint", state_interfaces=["position"])],
        )
        with pytest.raises(RobotValidationError, match="Not found: Joint 'missing_joint'"):
            robot.add_ros2_control(rc_invalid)

    def test_gazebo_element_filtering(self) -> None:
        """Test filtering Gazebo elements by reference."""
        robot = Robot(name="test")
        robot.add_link(Link(name="link1"))

        ge1 = GazeboElement(reference="link1", mu1=0.2)
        ge2 = GazeboElement(reference="link1", mu2=0.2)
        ge3 = GazeboElement(reference=None, static=True)

        robot.add_gazebo_element(ge1)
        robot.add_gazebo_element(ge2)
        robot.add_gazebo_element(ge3)

        # Test Global Retrieval
        assert len(robot.get_gazebo_elements()) == 3

        # Test Filtered Retrieval
        link1_elements = robot.get_gazebo_elements("link1")
        assert len(link1_elements) == 2
        assert ge1 in link1_elements
        assert ge2 in link1_elements

        # Test Global Only (None reference)
        global_elements = [ge for ge in robot.get_gazebo_elements() if ge.reference is None]
        assert len(global_elements) == 1
        assert global_elements[0] == ge3
