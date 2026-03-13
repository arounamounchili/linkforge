"""Targeted unit tests for individual ValidationCheck classes.

These tests exercise each check in isolation by calling check.run() directly,
independent of the full RobotValidator pipeline. This ensures that focused
validation rules can be reused in different parts of the application.
"""

import pytest
from linkforge_core.models.joint import Joint, JointLimits, JointMimic, JointType, Vector3
from linkforge_core.models.link import Inertial, Link
from linkforge_core.models.robot import Robot
from linkforge_core.validation import (
    DuplicateNameCheck,
    GeometryCheck,
    HasLinksCheck,
    JointReferenceCheck,
    MassPropertiesCheck,
    MimicChainCheck,
    Ros2ControlCheck,
    TreeStructureCheck,
    ValidationCheck,
)
from linkforge_core.validation.result import ValidationResult

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_result() -> ValidationResult:
    return ValidationResult(robot_name="test")


def _empty_robot() -> Robot:
    return Robot(name="test")


def _simple_robot() -> Robot:
    """Two-link, one-joint valid robot."""
    robot = Robot(name="simple")
    robot.add_link(Link(name="base", inertial=Inertial(mass=1.0)))
    robot.add_link(Link(name="link1", inertial=Inertial(mass=0.5)))
    robot.add_joint(
        Joint(
            name="j1",
            type=JointType.REVOLUTE,
            parent="base",
            child="link1",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
        )
    )
    return robot


# ---------------------------------------------------------------------------
# ValidationCheck ABC
# ---------------------------------------------------------------------------


class TestValidationCheckABC:
    def test_is_abstract(self) -> None:
        """ValidationCheck cannot be instantiated directly."""
        with pytest.raises(TypeError):
            ValidationCheck()  # type: ignore[abstract]

    def test_concrete_check_is_instance(self) -> None:
        assert isinstance(HasLinksCheck(), ValidationCheck)


# ---------------------------------------------------------------------------
# HasLinksCheck
# ---------------------------------------------------------------------------


class TestHasLinksCheck:
    def test_no_links_produces_error(self) -> None:
        robot = _empty_robot()
        result = _make_result()
        HasLinksCheck().run(robot, result)
        assert not result.is_valid
        assert result.errors[0].title == "No links"

    def test_with_links_passes(self) -> None:
        robot = _empty_robot()
        robot.add_link(Link(name="base"))
        result = _make_result()
        HasLinksCheck().run(robot, result)
        assert result.is_valid


# ---------------------------------------------------------------------------
# DuplicateNameCheck
# ---------------------------------------------------------------------------


class TestDuplicateNameCheck:
    def test_duplicate_link_names(self) -> None:
        robot = _empty_robot()
        robot._links.append(Link(name="dup", inertial=Inertial(mass=1.0)))
        robot._links.append(Link(name="dup", inertial=Inertial(mass=1.0)))
        result = _make_result()
        DuplicateNameCheck().run(robot, result)
        errors = [e for e in result.errors if e.title == "Duplicate link name"]
        assert len(errors) == 1
        assert "dup" in errors[0].message

    def test_duplicate_joint_names(self) -> None:
        robot = _simple_robot()
        robot.add_link(Link(name="link2", inertial=Inertial(mass=0.5)))
        robot._joints.append(Joint(name="j1", type=JointType.FIXED, parent="base", child="link2"))
        result = _make_result()
        DuplicateNameCheck().run(robot, result)
        errors = [e for e in result.errors if e.title == "Duplicate joint name"]
        assert len(errors) == 1

    def test_unique_names_pass(self) -> None:
        result = _make_result()
        DuplicateNameCheck().run(_simple_robot(), result)
        assert result.is_valid


# ---------------------------------------------------------------------------
# JointReferenceCheck
# ---------------------------------------------------------------------------


class TestJointReferenceCheck:
    def test_missing_parent(self) -> None:
        robot = _empty_robot()
        robot.add_link(Link(name="link1"))
        robot._joints.append(Joint(name="j1", type=JointType.FIXED, parent="ghost", child="link1"))
        result = _make_result()
        JointReferenceCheck().run(robot, result)
        assert any(e.title == "Missing parent link" for e in result.errors)

    def test_missing_child(self) -> None:
        robot = _empty_robot()
        robot.add_link(Link(name="base"))
        robot._joints.append(Joint(name="j1", type=JointType.FIXED, parent="base", child="ghost"))
        result = _make_result()
        JointReferenceCheck().run(robot, result)
        assert any(e.title == "Missing child link" for e in result.errors)

    def test_valid_references_pass(self) -> None:
        result = _make_result()
        JointReferenceCheck().run(_simple_robot(), result)
        assert result.is_valid


# ---------------------------------------------------------------------------
# TreeStructureCheck
# ---------------------------------------------------------------------------


class TestTreeStructureCheck:
    def test_skips_when_no_links(self) -> None:
        result = _make_result()
        TreeStructureCheck().run(_empty_robot(), result)
        assert result.is_valid  # HasLinksCheck responsibility

    def test_cycle_detection(self) -> None:
        robot = _empty_robot()
        robot.add_link(Link(name="a"))
        robot.add_link(Link(name="b"))
        robot._joints.append(Joint(name="j1", type=JointType.FIXED, parent="a", child="b"))
        robot._joints.append(Joint(name="j2", type=JointType.FIXED, parent="b", child="a"))
        result = _make_result()
        TreeStructureCheck().run(robot, result)
        assert not result.is_valid

    def test_disconnected_link_via_mock(self) -> None:
        from unittest.mock import patch

        robot = _empty_robot()
        base = Link(name="base", inertial=Inertial(mass=1.0))
        disc = Link(name="disc", inertial=Inertial(mass=1.0))
        robot.add_link(base)
        robot.add_link(disc)
        # Mock get_root_link so TreeStructureCheck sees 'base' as root
        # while 'disc' has no parent joint → triggers "Disconnected link"
        with patch.object(robot, "get_root_link", return_value=base):
            result = _make_result()
            TreeStructureCheck().run(robot, result)
        assert any(e.title == "Disconnected link" for e in result.errors)

    def test_valid_tree_passes(self) -> None:
        result = _make_result()
        TreeStructureCheck().run(_simple_robot(), result)
        assert result.is_valid


# ---------------------------------------------------------------------------
# MassPropertiesCheck
# ---------------------------------------------------------------------------


class TestMassPropertiesCheck:
    def test_very_low_mass_warning(self) -> None:
        robot = _empty_robot()
        robot.add_link(Link(name="base", inertial=Inertial(mass=0.001)))
        result = _make_result()
        MassPropertiesCheck().run(robot, result)
        assert result.is_valid
        assert any(w.title == "Very low mass" for w in result.warnings)

    def test_missing_inertia_warning(self) -> None:
        robot = _empty_robot()
        robot.add_link(Link(name="base", inertial=None))
        result = _make_result()
        MassPropertiesCheck().run(robot, result)
        assert result.is_valid
        assert any(w.title == "Missing inertia" for w in result.warnings)

    def test_healthy_link_no_warnings(self) -> None:
        robot = _empty_robot()
        robot.add_link(Link(name="base", inertial=Inertial(mass=1.0)))
        result = _make_result()
        MassPropertiesCheck().run(robot, result)
        assert not result.has_warnings


# ---------------------------------------------------------------------------
# GeometryCheck
# ---------------------------------------------------------------------------


class TestGeometryCheck:
    def test_no_visual_or_collision_warnings(self) -> None:
        robot = _empty_robot()
        robot.add_link(Link(name="base", inertial=Inertial(mass=1.0)))
        result = _make_result()
        GeometryCheck().run(robot, result)
        titles = {w.title for w in result.warnings}
        assert "No visual geometry" in titles
        assert "No collision geometry" in titles


# ---------------------------------------------------------------------------
# Ros2ControlCheck
# ---------------------------------------------------------------------------


class TestRos2ControlCheck:
    def test_skips_when_no_ros2_controls(self) -> None:
        result = _make_result()
        Ros2ControlCheck().run(_simple_robot(), result)
        assert result.is_valid

    def test_invalid_control_joint_reference(self) -> None:
        from linkforge_core.models.ros2_control import Ros2Control, Ros2ControlJoint

        robot = _simple_robot()
        control = Ros2Control(
            name="ctrl",
            type="system",
            hardware_plugin="mock_hw/MockSystem",
            joints=[
                Ros2ControlJoint(
                    name="ghost_joint",
                    command_interfaces=["position"],
                )
            ],
        )
        robot.add_ros2_control(control)
        result = _make_result()
        Ros2ControlCheck().run(robot, result)
        assert any(e.title == "Invalid ros2_control joint" for e in result.errors)


# ---------------------------------------------------------------------------
# MimicChainCheck
# ---------------------------------------------------------------------------


class TestMimicChainCheck:
    def test_invalid_mimic_target(self) -> None:
        robot = _simple_robot()
        robot._joints.append(
            Joint(
                name="mimic_j",
                type=JointType.REVOLUTE,
                parent="base",
                child="link1",
                axis=Vector3(1.0, 0.0, 0.0),
                limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
                mimic=JointMimic(joint="nonexistent"),
            )
        )
        result = _make_result()
        MimicChainCheck().run(robot, result)
        assert any(e.title == "Invalid mimic target" for e in result.errors)

    def test_circular_mimic_chain(self) -> None:
        from linkforge_core.models.link import Inertial

        robot = Robot(name="mimic_cycle")
        robot.add_link(Link(name="base", inertial=Inertial(mass=1.0)))
        robot.add_link(Link(name="a", inertial=Inertial(mass=0.5)))
        robot.add_link(Link(name="b", inertial=Inertial(mass=0.5)))

        robot._joints.append(
            Joint(
                name="ja",
                type=JointType.REVOLUTE,
                parent="base",
                child="a",
                axis=Vector3(1.0, 0.0, 0.0),
                limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
                mimic=JointMimic(joint="jb"),
            )
        )
        robot._joints.append(
            Joint(
                name="jb",
                type=JointType.REVOLUTE,
                parent="a",
                child="b",
                axis=Vector3(1.0, 0.0, 0.0),
                limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
                mimic=JointMimic(joint="ja"),
            )
        )
        result = _make_result()
        MimicChainCheck().run(robot, result)
        assert any(e.title == "Circular mimic dependency" for e in result.errors)

    def test_no_mimic_passes(self) -> None:
        result = _make_result()
        MimicChainCheck().run(_simple_robot(), result)
        assert result.is_valid


# ---------------------------------------------------------------------------
# RobotValidator custom registry
# ---------------------------------------------------------------------------


class TestRobotValidatorRegistry:
    def test_custom_checks_only_runs_those_checks(self) -> None:
        """Passing a custom check list runs only those checks, not all defaults."""
        from linkforge_core.validation import RobotValidator

        # Robot with no links AND low mass — only HasLinksCheck should fire
        robot = _empty_robot()
        result = RobotValidator(checks=[HasLinksCheck()]).validate(robot)
        assert not result.is_valid
        assert len(result.errors) == 1
        assert result.errors[0].title == "No links"
        # No warnings from MassPropertiesCheck or GeometryCheck
        assert not result.has_warnings

    def test_default_registry_runs_all_checks(self) -> None:
        from linkforge_core.validation import RobotValidator

        result = RobotValidator().validate(_simple_robot())
        assert result.is_valid  # No errors
        assert result.has_warnings  # Geometry warnings expected
