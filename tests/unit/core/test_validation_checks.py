"""Unit tests for modular validation checks."""

import pytest
from linkforge.core import (
    Joint,
    JointMimic,
    JointType,
    Link,
    Robot,
    ValidationErrorCode,
    Vector3,
)
from linkforge.core.io import read_srdf
from linkforge.core.models.link import Inertial
from linkforge.core.models.ros2_control import Ros2Control, Ros2ControlJoint
from linkforge.core.validation.checks import (
    DuplicateNameCheck,
    GeometryCheck,
    HasLinksCheck,
    JointReferenceCheck,
    MassPropertiesCheck,
    MimicChainCheck,
    Ros2ControlCheck,
    SemanticCheck,
    TreeStructureCheck,
)
from linkforge.core.validation.result import ValidationResult


@pytest.fixture
def empty_robot():
    return Robot(name="test_robot")


@pytest.fixture
def result():
    return ValidationResult()


def test_has_links_check(empty_robot, result):
    check = HasLinksCheck()
    check.run(empty_robot, result)
    assert not result.is_valid
    assert result.errors[0].code == ValidationErrorCode.VALUE_EMPTY


def test_duplicate_name_check(empty_robot, result):
    # Link duplicates
    empty_robot.links = (Link(name="link1"), Link(name="link1"))
    check = DuplicateNameCheck()
    check.run(empty_robot, result)
    assert any(
        err.code == ValidationErrorCode.DUPLICATE_NAME and "link" in err.message
        for err in result.errors
    )

    # Joint duplicates (clear results first)
    result.issues = []
    empty_robot.links = (Link(name="base"), Link(name="l1"))
    empty_robot.joints = (
        Joint(name="j1", type=JointType.FIXED, parent="base", child="l1"),
        Joint(name="j1", type=JointType.FIXED, parent="base", child="l1"),
    )
    check.run(empty_robot, result)
    assert any(
        err.code == ValidationErrorCode.DUPLICATE_NAME and "joint" in err.message
        for err in result.errors
    )


def test_joint_reference_check(empty_robot, result):
    # Missing child
    empty_robot.add_link(Link(name="base"))
    empty_robot.joints = (
        Joint(name="j1", type=JointType.FIXED, parent="base", child="non_existent"),
    )
    check = JointReferenceCheck()
    check.run(empty_robot, result)
    assert any("child" in err.message.lower() for err in result.errors)

    # Missing parent
    result.issues = []
    empty_robot.links = (Link(name="l1"),)
    empty_robot.joints = (
        Joint(name="j2", type=JointType.FIXED, parent="non_existent", child="l1"),
    )
    check.run(empty_robot, result)
    assert any("parent" in err.message.lower() for err in result.errors)


def test_tree_structure_check_cycle(empty_robot, result):
    empty_robot.add_link(Link(name="l1"))
    empty_robot.add_link(Link(name="l2"))
    # Manual bypass to create cycle
    empty_robot.joints = (
        Joint(name="j1", type=JointType.FIXED, parent="l1", child="l2"),
        Joint(name="j2", type=JointType.FIXED, parent="l2", child="l1"),
    )

    check = TreeStructureCheck()
    check.run(empty_robot, result)
    assert any(err.code == ValidationErrorCode.HAS_CYCLE for err in result.errors)


def test_tree_structure_check_multiple_parents(empty_robot, result):
    empty_robot.add_link(Link(name="base"))
    empty_robot.add_link(Link(name="l1"))
    empty_robot.add_link(Link(name="l2"))
    # l1 has two parents: base and l2.
    # We also connect l2 to base to ensure a single root exists,
    # so that the connectivity check is triggered.
    empty_robot.joints = (
        Joint(name="j1", type=JointType.FIXED, parent="base", child="l1"),
        Joint(name="j2", type=JointType.FIXED, parent="l2", child="l1"),
        Joint(name="j3", type=JointType.FIXED, parent="base", child="l2"),
    )
    # Reindex to build indices for connectivity check
    empty_robot._reindex()

    check = TreeStructureCheck()
    check.run(empty_robot, result)
    # Now that there is a unique root (base), connectivity check runs and finds multiple parents for l1
    assert any("Multiple parent joints" in err.title for err in result.errors)


def test_tree_structure_check_disconnected(empty_robot, result):
    empty_robot.add_link(Link(name="base"))
    empty_robot.add_link(Link(name="island"))
    empty_robot._reindex()

    check = TreeStructureCheck()
    check.run(empty_robot, result)
    # Reports MULTIPLE_ROOTS because 'island' has no parent
    assert any(err.code == ValidationErrorCode.MULTIPLE_ROOTS for err in result.errors)


def test_mass_properties_check(empty_robot, result):
    # Link with near-zero mass
    link = Link(name="light", inertial=Inertial(mass=1e-12))
    empty_robot.add_link(link)

    # Link with missing inertia (should trigger warning)
    link2 = Link(name="no_inertia", inertial=None)
    empty_robot.add_link(link2)

    check = MassPropertiesCheck()
    check.run(empty_robot, result)
    assert any(err.code == ValidationErrorCode.PHYSICS_VIOLATION for err in result.errors)
    assert any(warn.code == ValidationErrorCode.NOT_FOUND for warn in result.warnings)


def test_geometry_check_warnings(empty_robot, result):
    empty_robot.add_link(Link(name="ghost"))

    check = GeometryCheck()
    check.run(empty_robot, result)
    assert len(result.warnings) >= 2
    assert "visual" in result.warnings[0].message
    assert "collision" in result.warnings[1].message


def test_ros2_control_check(empty_robot, result):
    empty_robot.add_link(Link(name="base"))
    # Bypass validation in add_ros2_control
    empty_robot.ros2_controls = (
        Ros2Control(
            name="hw",
            hardware_plugin="mock",
            joints=(Ros2ControlJoint(name="ghost_joint", command_interfaces=("position",)),),
        ),
    )

    check = Ros2ControlCheck()
    check.run(empty_robot, result)
    assert any(err.code == ValidationErrorCode.NOT_FOUND for err in result.errors)


def test_mimic_chain_circular(empty_robot, result):
    axis = Vector3(0, 0, 1)
    # Initialize with mimic to avoid FrozenInstanceError
    j1 = Joint(
        name="j1",
        type=JointType.CONTINUOUS,
        parent="a",
        child="b",
        axis=axis,
        mimic=JointMimic(joint="j2"),
    )
    j2 = Joint(
        name="j2",
        type=JointType.CONTINUOUS,
        parent="b",
        child="c",
        axis=axis,
        mimic=JointMimic(joint="j1"),
    )
    # Bypass validation in add_joint
    empty_robot.joints = (j1, j2)

    check = MimicChainCheck()
    check.run(empty_robot, result)
    assert any(err.code == ValidationErrorCode.HAS_CYCLE for err in result.errors)


def test_semantic_check_subgroup_cycle(empty_robot, result):
    # Create SRDF with circular subgroup dependencies
    srdf_xml = """<?xml version="1.0" encoding="UTF-8"?>
    <robot name="test">
        <group name="g1">
            <group name="g2"/>
        </group>
        <group name="g2">
            <group name="g1"/>
        </group>
    </robot>
    """
    empty_robot.semantic = read_srdf(srdf_xml)

    check = SemanticCheck()
    check.run(empty_robot, result)
    assert any("Circular subgroup dependency" in err.title for err in result.errors)


def test_semantic_check_invalid_end_effector(empty_robot, result):
    srdf_xml = """<?xml version="1.0" encoding="UTF-8"?>
    <robot name="test">
        <end_effector name="hand" group="non_existent" parent_link="base"/>
    </robot>
    """
    empty_robot.add_link(Link(name="base"))
    empty_robot.semantic = read_srdf(srdf_xml)

    check = SemanticCheck()
    check.run(empty_robot, result)
    assert any("Invalid end effector group" in err.title for err in result.errors)


def test_robot_validator_integration(empty_robot):
    # Just ensure it runs without crashing and collects issues
    empty_robot.add_link(Link(name="base", inertial=Inertial(mass=1.0)))
    # Trigger a warning (no visual/collision)
    from linkforge.core.io import validate_robot

    result = validate_robot(empty_robot)
    if not result.is_valid:
        for issue in result.issues:
            print(f"Issue: {issue}")
    assert result.has_warnings
    assert any("visual" in w.message for w in result.warnings)
