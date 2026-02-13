import pytest
from linkforge_core.models import Joint, JointMimic, JointType, Link, Robot
from linkforge_core.models.sensor import Sensor
from linkforge_core.models.transmission import Transmission, TransmissionJoint


class TestRobot:
    def test_robot_initialization(self):
        """Test basic robot initialization and index creation."""
        base = Link(name="base_link")
        link1 = Link(name="link1")

        from linkforge_core.models import JointLimits

        joint1 = Joint(
            name="joint1",
            parent="base_link",
            child="link1",
            type=JointType.REVOLUTE,
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
        with pytest.raises(ValueError, match="cannot be empty"):
            Robot(name="")

        with pytest.raises(ValueError, match="invalid characters"):
            Robot(name="invalid name with spaces")

    def test_duplicate_components(self):
        """Test detection of duplicate links and joints."""
        link1 = Link(name="link1")
        link2 = Link(name="link1")

        with pytest.raises(ValueError, match="Duplicate link name"):
            Robot(name="test", initial_links=[link1, link2])

        joint1 = Joint(name="joint1", parent="base", child="link1", type=JointType.FIXED)
        joint2 = Joint(name="joint1", parent="base", child="link1", type=JointType.FIXED)

        with pytest.raises(ValueError, match="Duplicate joint name"):
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

        with pytest.raises(ValueError, match="already exists"):
            robot.add_link(Link(name="base"))

        with pytest.raises(ValueError, match="not found"):
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
        errors = robot.validate_tree_structure()

        assert any("cycle" in e for e in errors)
        assert robot._has_cycle() is True

    def test_mimic_cycle_detection(self):
        """Test detection of circular mimic dependencies."""
        links = [Link(name="A"), Link(name="B")]

        from linkforge_core.models import JointLimits

        limits = JointLimits(lower=-1.0, upper=1.0)
        j1 = Joint(
            name="j1",
            parent="A",
            child="B",
            type=JointType.REVOLUTE,
            limits=limits,
            mimic=JointMimic(joint="j2"),
        )
        j2 = Joint(
            name="j2",
            parent="B",
            child="A",
            type=JointType.REVOLUTE,
            limits=limits,
            mimic=JointMimic(joint="j1"),
        )

        robot = Robot(name="mimic_cycle", initial_links=links, initial_joints=[j1, j2])
        errors = robot.validate_tree_structure()

        assert any("Circular mimic dependency" in e for e in errors)

    def test_mimic_missing_joint(self):
        """Test validation of mimic pointing to non-existent joint."""
        links = [Link(name="A"), Link(name="B")]
        from linkforge_core.models import JointLimits

        j1 = Joint(
            name="j1",
            parent="A",
            child="B",
            type=JointType.REVOLUTE,
            limits=JointLimits(lower=-1.0, upper=1.0),
            mimic=JointMimic(joint="missing_joint"),
        )

        robot = Robot(name="mimic_missing", initial_links=links, initial_joints=[j1])
        errors = robot.validate_tree_structure()

        assert any("mimics non-existent joint" in e for e in errors)

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

        with pytest.raises(ValueError, match="No root link found"):
            robot_cycle.get_root_link()

    def test_disconnected_component(self):
        """Test detection of disconnected parts of the graph."""
        links = [Link(name="root"), Link(name="connected"), Link(name="floating")]
        joints = [Joint(name="j1", parent="root", child="connected", type=JointType.FIXED)]

        robot = Robot(name="disconnected", initial_links=links, initial_joints=joints)
        errors = robot.validate_tree_structure()

        assert any("Multiple root links found" in e for e in errors)

    def test_add_sensor_validation(self):
        """Test validation when adding sensors."""
        robot = Robot(name="test", initial_links=[Link(name="base")])

        from linkforge_core.models import CameraInfo, SensorType

        sensor = Sensor(
            name="cam1", link_name="base", type=SensorType.CAMERA, camera_info=CameraInfo()
        )
        robot.add_sensor(sensor)
        assert robot.sensors[0] == sensor

        with pytest.raises(ValueError, match="link 'missing' not found"):
            robot.add_sensor(
                Sensor(
                    name="cam2",
                    link_name="missing",
                    type=SensorType.CAMERA,
                    camera_info=CameraInfo(),
                )
            )

        with pytest.raises(ValueError, match="already exists"):
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

        with pytest.raises(ValueError, match="joint 'missing_joint' not found"):
            robot.add_transmission(bad_trans)

    def test_robot_properties(self):
        """Test robot properties like mass and DOF."""
        # Create a robot with 2 links and 1 joint
        # Fix: Inertial takes float mass, not Mass object
        from linkforge_core.models import Inertial, InertiaTensor

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
        from linkforge_core.models import GazeboElement

        # Valid element without reference
        elem1 = GazeboElement(material="Gazebo/Blue")
        robot.add_gazebo_element(elem1)
        assert len(robot.gazebo_elements) == 1

        # Valid element with reference
        elem2 = GazeboElement(reference="base", material="Gazebo/Red")
        robot.add_gazebo_element(elem2)
        assert len(robot.gazebo_elements) == 2

        # Invalid reference
        with pytest.raises(ValueError, match="does not match any link or joint"):
            robot.add_gazebo_element(GazeboElement(reference="missing"))

    def test_add_ros2_control(self):
        """Test adding ROS2 Control configurations."""
        robot = Robot(name="test")
        # Fix: Ros2Control takes hardware_plugin string, not HardwareInterface object
        from linkforge_core.models import Ros2Control

        ros2_ctrl = Ros2Control(
            name="System", type="system", hardware_plugin="mock_plugin", joints=[]
        )
        robot.add_ros2_control(ros2_ctrl)
        assert len(robot.ros2_controls) == 1

        # Duplicate name check
        with pytest.raises(ValueError, match="already exists"):
            robot.add_ros2_control(ros2_ctrl)

    def test_string_representation(self):
        """Test __str__ method completeness."""
        robot = Robot(name="full_bot", initial_links=[Link(name="base")])

        # Basic
        assert "Robot(name=full_bot" in str(robot)
        assert "links=1" in str(robot)

        # With all optional components
        from linkforge_core.models import CameraInfo, GazeboElement, Ros2Control, SensorType

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
        assert "gazebo_elements=1" in s
