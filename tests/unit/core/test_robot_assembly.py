import pytest
from linkforge_core.composer.robot_assembly import RobotAssembly
from linkforge_core.exceptions import RobotValidationError
from linkforge_core.models.gazebo import GazeboElement
from linkforge_core.models.geometry import Vector3
from linkforge_core.models.joint import Joint, JointLimits, JointType
from linkforge_core.models.link import Link
from linkforge_core.models.robot import Robot
from linkforge_core.models.ros2_control import Ros2Control, Ros2ControlJoint
from linkforge_core.models.sensor import LidarInfo, Sensor, SensorType
from linkforge_core.models.srdf import (
    EndEffector,
    GroupState,
    PlanningGroup,
    SemanticRobotDescription,
)
from linkforge_core.models.transmission import (
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    TransmissionType,
)


class TestRobotAssembly:
    def test_assembly_creation(self) -> None:
        """Test basic assembly creation."""
        assembly = RobotAssembly.create("my_robot")
        assert assembly.robot.name == "my_robot"
        assert len(assembly.robot.links) == 0
        assert assembly.srdf is not None

    def test_micro_construction_fluent(self) -> None:
        """Test building a robot link-by-link using the fluent API."""
        assembly = RobotAssembly.create("fluent_bot")

        # Build base
        assembly.robot.add_link(Link(name="base_link"))

        # Build arm link using fluent API with full validation parameters
        assembly.add_link("link1").with_mass(1.5).connect_to(
            parent="base_link",
            joint_name="joint1",
            joint_type=JointType.REVOLUTE,
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-1, upper=1),
        )

        assert len(assembly.robot.links) == 2
        assert len(assembly.robot.joints) == 1
        assert assembly.robot.get_link("link1").mass == 1.5
        assert assembly.robot.get_joint("joint1").parent == "base_link"
        assert assembly.robot.get_joint("joint1").axis.z == 1.0

    def test_macro_assembly_attach(self) -> None:
        """Test attaching a sub-robot component."""
        # Create a simple gripper component
        gripper = Robot(name="gripper")
        gripper.add_link(Link(name="palm"))
        gripper.add_link(Link(name="finger"))
        gripper.add_joint(
            Joint(
                name="f_joint",
                parent="palm",
                child="finger",
                type=JointType.REVOLUTE,
                axis=Vector3(0, 0, 1),
                limits=JointLimits(lower=0, upper=0.5),
            )
        )

        # Create base robot
        assembly = RobotAssembly.create("robot_arm")
        assembly.robot.add_link(Link(name="tool0"))

        # Attach gripper
        assembly.attach(
            component=gripper, at_link="tool0", joint_name="gripper_fix", prefix="left_"
        )

        # Verify names are prefixed
        assert assembly.robot.get_link("left_palm") is not None
        assert assembly.robot.get_link("left_finger") is not None
        assert assembly.robot.get_joint("left_f_joint") is not None
        assert assembly.robot.get_joint("left_gripper_fix") is not None

        # Verify connectivity
        fix_joint = assembly.robot.get_joint("left_gripper_fix")
        assert fix_joint.parent == "tool0"
        assert fix_joint.child == "left_palm"

        # Verify isolation (original gripper should not be modified)
        assert gripper.get_link("left_palm") is None
        assert gripper.get_link("palm") is not None

    def test_srdf_helpers(self) -> None:
        """Test SRDF semantic data helpers."""
        assembly = RobotAssembly.create("semantic_bot")
        assembly.robot.add_link(Link(name="link_a"))
        assembly.robot.add_link(Link(name="link_b"))

        assembly.add_group("arm", links=["link_a", "link_b"])
        assembly.disable_collisions("link_a", "link_b", reason="Never")

        assert len(assembly.srdf.groups) == 1
        assert assembly.srdf.groups[0].name == "arm"
        assert len(assembly.srdf.disabled_collisions) == 1
        assert assembly.srdf.disabled_collisions[0].reason == "Never"

    def test_attach_duplicate_protection(self) -> None:
        """Test that attaching twice with different prefixes works perfectly."""
        wheel = Robot(name="wheel")
        wheel.add_link(Link(name="rim"))

        assembly = RobotAssembly.create("car")
        assembly.robot.add_link(Link(name="chassis"))

        # Attach two identical wheels
        assembly.attach(wheel, at_link="chassis", joint_name="w_joint", prefix="fr_")
        assembly.attach(wheel, at_link="chassis", joint_name="w_joint", prefix="fl_")

        assert len(assembly.robot.links) == 3  # chassis + 2 rims
        assert assembly.robot.get_link("fr_rim") is not None
        assert assembly.robot.get_link("fl_rim") is not None

    def test_validation_error_on_attach(self) -> None:
        """Test that assembly re-validates and catches errors."""
        assembly = RobotAssembly.create("error_bot")
        assembly.robot.add_link(Link(name="base"))

        # 1. Test missing parent link in assembly
        bad_component = Robot(name="comp")
        bad_component.add_link(Link(name="l1"))
        with pytest.raises(RobotValidationError, match="Attachment link not found"):
            assembly.attach(bad_component, at_link="non_existent", joint_name="j")

        # 2. Test component with no root (empty)
        empty_comp = Robot(name="empty")
        with pytest.raises(RobotValidationError, match="No root link found"):
            assembly.attach(empty_comp, at_link="base", joint_name="j")

    def test_complex_component_merge(self) -> None:
        """Test merging a component with sensors, gazebo, and ros2_control."""
        # Create a complex sub-robot
        comp = Robot(name="sub")
        comp.add_link(Link(name="sub_base"))
        comp.add_link(Link(name="sub_link"))
        comp.add_joint(
            Joint(
                name="sub_joint",
                parent="sub_base",
                child="sub_link",
                type=JointType.REVOLUTE,
                axis=Vector3(0, 0, 1),
                limits=JointLimits(lower=-1, upper=1),
            )
        )

        # Add a sensor
        comp.add_sensor(
            Sensor(
                name="lidar",
                type=SensorType.LIDAR,
                link_name="sub_link",
                lidar_info=LidarInfo(),
            )
        )

        # Add a gazebo element
        comp.add_gazebo_element(GazeboElement(reference="sub_link"))

        # Add a transmission
        trans = Transmission(
            name="trans1",
            type=TransmissionType.SIMPLE,
            joints=[TransmissionJoint(name="sub_joint")],
            actuators=[TransmissionActuator(name="act1")],
        )
        comp.add_transmission(trans)

        # Add ROS2 Control
        rc = Ros2Control(name="sub_ctrl", hardware_plugin="mock_hw")
        rc.joints.append(Ros2ControlJoint(name="sub_joint", state_interfaces=["position"]))
        comp.add_ros2_control(rc)

        # Create assembly
        assembly = RobotAssembly.create("main")
        assembly.robot.add_link(Link(name="root"))

        # Attach (using sub_base as the root of the component)
        assembly.attach(comp, at_link="root", joint_name="conn", prefix="p_")

        # Verify
        assert assembly.robot._sensor_index.get("p_lidar") is not None
        assert assembly.robot._sensor_index.get("p_lidar").link_name == "p_sub_link"
        assert len(assembly.robot.gazebo_elements) == 1
        assert assembly.robot.gazebo_elements[0].reference == "p_sub_link"
        assert len(assembly.robot.transmissions) == 1
        assert assembly.robot.transmissions[0].name == "p_trans1"
        assert assembly.robot.transmissions[0].joints[0].name == "p_sub_joint"
        assert len(assembly.robot.ros2_controls) == 1
        assert assembly.robot.ros2_controls[0].name == "p_sub_ctrl"
        assert assembly.robot.ros2_controls[0].joints[0].name == "p_sub_joint"

    def test_prefix_all_semantic_merging(self) -> None:
        """Test that SRDF groups and states are correctly prefixed and merged."""
        comp = Robot(name="arm")
        comp.add_link(Link(name="base"))
        comp.add_link(Link(name="tip"))
        comp.add_joint(Joint(name="j1", parent="base", child="tip", type=JointType.FIXED))

        srdf = SemanticRobotDescription()
        srdf.groups.append(PlanningGroup(name="grp", links=["base", "tip"]))
        srdf.group_states.append(GroupState(name="folded", group="grp", joint_values={"j1": 0.0}))
        srdf.end_effectors.append(EndEffector(name="ee", group="grp", parent_link="tip"))
        comp.semantic = srdf

        assembly = RobotAssembly.create("full")
        assembly.robot.add_link(Link(name="world"))

        assembly.attach(comp, at_link="world", joint_name="mount", prefix="robot1_")

        # Check SRDF
        assert len(assembly.srdf.groups) == 1
        assert assembly.srdf.groups[0].name == "robot1_grp"
        assert "robot1_base" in assembly.srdf.groups[0].links

        assert len(assembly.srdf.group_states) == 1
        assert assembly.srdf.group_states[0].name == "robot1_folded"
        assert assembly.srdf.group_states[0].group == "robot1_grp"
        assert "robot1_j1" in assembly.srdf.group_states[0].joint_values

        assert len(assembly.srdf.end_effectors) == 1
        assert assembly.srdf.end_effectors[0].name == "robot1_ee"
        assert assembly.srdf.end_effectors[0].parent_link == "robot1_tip"
