import pytest
from linkforge_core.composer.robot_assembly import RobotAssembly
from linkforge_core.exceptions import RobotValidationError
from linkforge_core.models.joint import Joint, JointType
from linkforge_core.models.link import Link
from linkforge_core.models.robot import Robot


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

        # Build arm link using fluent API
        assembly.add_link("link1").with_mass(1.5).connect_to(
            parent="base_link", joint_name="joint1", type=JointType.REVOLUTE
        )

        assert len(assembly.robot.links) == 2
        assert len(assembly.robot.joints) == 1
        assert assembly.robot.get_link("link1").mass == 1.5
        assert assembly.robot.get_joint("joint1").parent == "base_link"

    def test_macro_assembly_attach(self) -> None:
        """Test attaching a sub-robot component."""
        # Create a simple gripper component
        gripper = Robot(name="gripper")
        gripper.add_link(Link(name="palm"))
        gripper.add_link(Link(name="finger"))
        gripper.add_joint(
            Joint(name="f_joint", parent="palm", child="finger", type=JointType.REVOLUTE)
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

        bad_component = Robot(name="bad")
        bad_component.add_link(Link(name="only_link"))
        # This will create a cycle if we connect base -> only_link -> base

        # We'll mock a cycle or just test a basic validation failure
        with pytest.raises(RobotValidationError):
            assembly.attach(bad_component, at_link="non_existent", joint_name="j")
