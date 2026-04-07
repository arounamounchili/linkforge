from dataclasses import replace

import pytest
from linkforge_core.composer import RobotAssembly, fixed_joint, origin, revolute_joint
from linkforge_core.exceptions import RobotValidationError
from linkforge_core.models.gazebo import GazeboElement
from linkforge_core.models.geometry import Transform, Vector3
from linkforge_core.models.joint import Joint, JointLimits, JointType
from linkforge_core.models.link import Link
from linkforge_core.models.robot import Robot
from linkforge_core.models.ros2_control import Ros2Control, Ros2ControlJoint
from linkforge_core.models.sensor import LidarInfo, Sensor, SensorType
from linkforge_core.models.srdf import (
    GroupState,
    PassiveJoint,
    PlanningGroup,
    SemanticRobotDescription,
    VirtualJoint,
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
        ).as_revolute(
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

    def test_assembly_init_with_existing_semantic(self) -> None:
        """Test that assembly syncs with existing semantic data in the robot."""
        robot = Robot(name="test")
        semantic = SemanticRobotDescription(virtual_joints=[])
        robot.semantic = semantic
        assembly = RobotAssembly(robot=robot)
        assert assembly.srdf is semantic

    def test_link_builder_with_existing_inertial(self) -> None:
        """Test LinkBuilder.with_mass on a link that already has inertial data."""
        from linkforge_core.models.link import Inertial, InertiaTensor

        assembly = RobotAssembly.create("test")
        init_inertial = Inertial(mass=1.0, inertia=InertiaTensor.zero())
        builder = assembly.add_link("link1")
        builder._link = replace(builder._link, inertial=init_inertial)

        builder.with_mass(5.0)
        assert builder._link.inertial.mass == 5.0

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
        with pytest.raises(
            RobotValidationError, match=r"\[NOT_FOUND\] Attachment link 'non_existent' not found"
        ):
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
        sub = Robot(name="sub")
        sub.add_link(Link(name="l1"))  # Add root link
        sub._semantic = SemanticRobotDescription(
            groups=[PlanningGroup(name="grp", links=["l1"])],
            group_states=[GroupState(name="st", group="grp", joint_values={"j1": 0.0})],
            virtual_joints=[
                VirtualJoint(name="vj", child_link="l1", parent_frame="world", type="fixed")
            ],
            passive_joints=[PassiveJoint(name="pj")],
        )

        assembly = RobotAssembly.create("main")
        assembly.robot.add_link(Link(name="root"))
        assembly.attach(sub, at_link="root", joint_name="conn", prefix="p_")

        assert len(assembly.srdf.groups) == 1
        assert assembly.srdf.groups[0].name == "p_grp"
        assert assembly.srdf.groups[0].links == ["p_l1"]
        assert assembly.srdf.group_states[0].name == "p_st"
        assert assembly.srdf.group_states[0].group == "p_grp"
        assert assembly.srdf.group_states[0].joint_values == {"p_j1": 0.0}
        assert assembly.srdf.virtual_joints[0].name == "p_vj"
        assert assembly.srdf.virtual_joints[0].child_link == "p_l1"
        assert assembly.srdf.passive_joints[0].name == "p_pj"

    def test_export_shortcuts(self) -> None:
        """Test the shortcut methods for URDF and SRDF export."""
        assembly = RobotAssembly.create("export_bot")
        assembly.robot.add_link(Link(name="base"))
        assembly.add_link("root").connect_to("base", "joint").as_fixed()

        urdf = assembly.export_urdf(validate=True)
        assert '<robot name="export_bot"' in urdf
        assert '<link name="root"' in urdf

        srdf = assembly.export_srdf(validate=False)
        assert '<robot name="export_bot"' in srdf

    def test_batch_collision_disable(self) -> None:
        """Test the batch collision disable helper."""
        assembly = RobotAssembly.create("coll_bot")
        links = ["l1", "l2", "l3"]
        for link_name in links:
            assembly.robot.add_link(Link(name=link_name))

        assembly.disable_all_collisions(links, reason="BatchDisable")

        # Combinations of 3 are (l1,l2), (l1,l3), (l2,l3)
        assert len(assembly.srdf.disabled_collisions) == 3
        found_pairs = {(dc.link1, dc.link2) for dc in assembly.srdf.disabled_collisions}
        assert ("l1", "l2") in found_pairs
        assert ("l1", "l3") in found_pairs
        assert ("l2", "l3") in found_pairs

    def test_factory_aliases(self) -> None:
        """Test the ergonomic factory aliases from the composer package."""
        assert fixed_joint() == JointType.FIXED
        assert revolute_joint() == JointType.REVOLUTE

        o = origin(xyz=(1, 2, 3), rpy=(0.1, 0.2, 0.3))
        assert isinstance(o, Transform)
        assert o.xyz.x == 1.0
        assert o.rpy.z == 0.3

    def test_fluent_joint_builders(self) -> None:
        """Test as_fixed and as_revolute fluent builders."""
        assembly = RobotAssembly.create("fluent_bot")
        assembly.robot.add_link(Link(name="parent"))

        # Test as_fixed
        assembly.add_link("child_fixed").connect_to("parent", "j_fixed").as_fixed()
        assert assembly.robot.get_joint("j_fixed").type == JointType.FIXED

        # Test as_revolute
        axis = Vector3(0, 0, 1)
        limits = JointLimits(effort=10, velocity=1)
        assembly.add_link("child_rev").connect_to("parent", "j_rev").as_revolute(
            axis=axis, limits=limits
        )
        joint = assembly.robot.get_joint("j_rev")
        assert joint.type == JointType.REVOLUTE
        assert joint.axis == axis

    def test_at_origin_sets_joint_transform(self) -> None:
        """Verify that at_origin() correctly sets the joint transform."""
        assembly = RobotAssembly.create("origin_test")
        assembly.robot.add_link(Link(name="base"))

        assembly.add_link("l1").at_origin(xyz=(1, 2, 3), rpy=(0, 0.5, 0)).connect_to(
            "base", "j1"
        ).as_fixed()

        joint = assembly.robot.get_joint("j1")
        assert joint.origin.xyz.x == 1.0
        assert joint.origin.xyz.y == 2.0
        assert joint.origin.xyz.z == 3.0
        assert joint.origin.rpy.y == 0.5

    def test_explicit_origin_overrides_at_origin(self) -> None:
        """Verify that explicit origin takes precedence over at_origin()."""
        assembly = RobotAssembly.create("override_test")
        assembly.robot.add_link(Link(name="base"))

        explicit_origin = Transform(xyz=Vector3(10, 0, 0))

        assembly.add_link("l1").at_origin(xyz=(1, 1, 1)).connect_to("base", "j1").as_fixed(
            origin=explicit_origin
        )

        joint = assembly.robot.get_joint("j1")
        assert joint.origin.xyz.x == 10.0
        assert joint.origin.xyz.y == 0.0

    def test_as_prismatic_creates_correct_joint(self) -> None:
        """Verify the prismatic joint shortcut."""
        assembly = RobotAssembly.create("prismatic_test")
        assembly.robot.add_link(Link(name="base"))

        axis = Vector3(1, 0, 0)
        limits = JointLimits(lower=0, upper=1, effort=10, velocity=1)
        assembly.add_link("l1").connect_to("base", "j1").as_prismatic(axis=axis, limits=limits)

        joint = assembly.robot.get_joint("j1")
        assert joint.type == JointType.PRISMATIC
        assert joint.axis == axis
        assert joint.limits == limits

    def test_as_continuous_creates_correct_joint(self) -> None:
        """Verify the continuous joint shortcut."""
        assembly = RobotAssembly.create("continuous_test")
        assembly.robot.add_link(Link(name="base"))

        axis = Vector3(0, 0, 1)
        assembly.add_link("l1").connect_to("base", "j1").as_continuous(axis=axis)

        joint = assembly.robot.get_joint("j1")
        assert joint.type == JointType.CONTINUOUS
        assert joint.axis == axis
        assert joint.limits is None

    def test_missing_connect_to_raises_error(self) -> None:
        """Verify that finalizing without connect_to raises an error."""
        assembly = RobotAssembly.create("error_test")
        builder = assembly.add_link("l1")

        with pytest.raises(RobotValidationError, match=r"connect_to\(\) must be called"):
            builder.as_fixed()
