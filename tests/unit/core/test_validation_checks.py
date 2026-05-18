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
    # Link with near-zero mass (PHYSICS_VIOLATION error)
    link = Link(name="light", inertial=Inertial(mass=1e-12))
    empty_robot.add_link(link)

    # Link with missing inertia (should trigger warning)
    link2 = Link(name="no_inertia", inertial=None)
    empty_robot.add_link(link2)

    # Link with low mass (Very low mass warning)
    link3 = Link(name="low_mass_warn", inertial=Inertial(mass=0.005))
    empty_robot.add_link(link3)

    # Link with near-zero inertia (Critical low inertia error)
    from linkforge.core.models.link import InertiaTensor

    tiny_tensor = InertiaTensor(ixx=1e-12, ixy=0, ixz=0, iyy=1e-12, iyz=0, izz=1e-12)
    link4 = Link(name="low_inertia", inertial=Inertial(mass=1.0, inertia=tiny_tensor))
    empty_robot.add_link(link4)

    check = MassPropertiesCheck()
    check.run(empty_robot, result)
    assert any(err.title == "Critical low mass" for err in result.errors)
    assert any(err.title == "Critical low inertia" for err in result.errors)
    assert any(warn.title == "Very low mass" for warn in result.warnings)
    assert any(warn.title == "Missing inertia" for warn in result.warnings)


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


def test_geometry_check_with_geom(empty_robot, result):
    from linkforge.core import Collision, Visual
    from linkforge.core.models.geometry import Box

    geom = Box(size=Vector3(1, 1, 1))

    link = Link(
        name="l1",
        visuals=[Visual(geometry=geom)],
        collisions=[Collision(geometry=geom)],
    )
    empty_robot.add_link(link)
    check = GeometryCheck()
    check.run(empty_robot, result)
    assert not result.has_warnings


def test_ros2_control_check_valid_joint(empty_robot, result):
    empty_robot.add_link(Link(name="base"))
    empty_robot.add_link(Link(name="l1"))
    empty_robot.add_joint(Joint(name="j1", type=JointType.FIXED, parent="base", child="l1"))
    empty_robot.ros2_controls = (
        Ros2Control(
            name="hw",
            hardware_plugin="mock",
            joints=(Ros2ControlJoint(name="j1", command_interfaces=("position",)),),
        ),
    )
    check = Ros2ControlCheck()
    check.run(empty_robot, result)
    assert result.is_valid


def test_mimic_chain_valid(empty_robot, result):
    empty_robot.add_link(Link(name="base"))
    empty_robot.add_link(Link(name="l1"))
    empty_robot.add_link(Link(name="l2"))
    axis = Vector3(0, 0, 1)
    empty_robot.add_joint(
        Joint(name="j1", type=JointType.CONTINUOUS, parent="base", child="l1", axis=axis)
    )
    empty_robot.add_joint(
        Joint(
            name="j2",
            type=JointType.CONTINUOUS,
            parent="l1",
            child="l2",
            axis=axis,
            mimic=JointMimic(joint="j1"),
        )
    )
    check = MimicChainCheck()
    check.run(empty_robot, result)
    assert result.is_valid


def test_semantic_check_comprehensive(empty_robot, result) -> None:
    srdf_xml = """<?xml version="1.0" encoding="UTF-8"?>
    <robot name="test">
        <group name="g1">
            <link name="non_existent_link"/>
            <joint name="non_existent_joint"/>
            <group name="non_existent_subgroup"/>
        </group>
        <group_state name="s1" group="non_existent_group"/>
        <end_effector name="ee1" group="g1" parent_link="non_existent_link" parent_group="non_existent_parent_group"/>
        <passive_joint name="non_existent_passive"/>
    </robot>
    """
    empty_robot.semantic = read_srdf(srdf_xml)
    check = SemanticCheck()
    check.run(empty_robot, result)

    assert any("Invalid planning group link" in err.title for err in result.errors)
    assert any("Invalid planning group joint" in err.title for err in result.errors)
    assert any("Invalid group state reference" in err.title for err in result.errors)
    assert any("Invalid end effector parent link" in err.title for err in result.errors)
    assert any("Invalid end effector parent group" in err.title for err in result.errors)
    assert any("Invalid passive joint" in err.title for err in result.errors)
    assert any("Invalid subgroup reference" in err.title for err in result.errors)


def test_semantic_check_dfs_not_found(result) -> None:
    from linkforge.core.models.srdf import PlanningGroup

    check = SemanticCheck()
    group = PlanningGroup(name="ghost", links=("link1",))
    # This directly triggers "if not current_group: return False"
    check._check_subgroup_cycles(group, [], result)
    assert not result.errors


def test_tree_structure_check_connectivity_disconnected(empty_robot, result, mocker):
    empty_robot.add_link(Link(name="base"))
    empty_robot.add_link(Link(name="island"))
    mocker.patch.object(Robot, "root_link", return_value=empty_robot.links[0])
    check = TreeStructureCheck()
    check.run(empty_robot, result)
    assert any("Disconnected link" in err.title for err in result.errors)


def test_tree_structure_check_exceptions(empty_robot, result, mocker):
    from linkforge.core import RobotModelError, RobotValidationError
    from linkforge.core.models.graph import KinematicGraph

    # 1. TreeStructureCheck on robot without links (line 140)
    check = TreeStructureCheck()
    check.run(empty_robot, result)
    assert not result.errors  # already reported by HasLinksCheck

    # Add a link so subsequent checks can run
    empty_robot.add_link(Link(name="base"))

    # 2. robot.has_cycle raises RobotModelError (lines 160-165)
    mocker.patch.object(
        KinematicGraph, "has_cycle", side_effect=RobotModelError("mocked cycle error")
    )

    check.run(empty_robot, result)
    assert any("Kinematic graph error" in err.title for err in result.errors)

    # 2b. robot.has_cycle raises RobotModelError and has_ref_errors is True
    result.issues = []
    result.add_error(
        title="Missing parent link", message="some msg", code=ValidationErrorCode.NOT_FOUND
    )
    check.run(empty_robot, result)
    assert not any("Kinematic graph error" in err.title for err in result.errors)

    # 3. robot.root_link raises unexpected RobotValidationError (line 196)
    result.issues = []
    mocker.patch.object(
        Robot,
        "get_root_link",
        side_effect=RobotValidationError(
            ValidationErrorCode.INVALID_VALUE, "mocked root validation error"
        ),
    )
    check.run(empty_robot, result)
    assert any("Root link error" in err.title for err in result.errors)

    # 4. robot.root_link raises RobotModelError (lines 202-208)
    result.issues = []
    mocker.patch.object(
        Robot, "get_root_link", side_effect=RobotModelError("mocked root model error")
    )
    check.run(empty_robot, result)
    assert any("Kinematic error" in err.title for err in result.errors)


def test_semantic_check_no_semantic(empty_robot, result):
    empty_robot.semantic = None
    check = SemanticCheck()
    check.run(empty_robot, result)
    assert not result.errors


def test_mimic_chain_check_non_existent_mimic(empty_robot, result):
    empty_robot.add_link(Link(name="base"))
    empty_robot.add_link(Link(name="l1"))
    empty_robot.add_joint(
        Joint(
            name="j1",
            type=JointType.CONTINUOUS,
            parent="base",
            child="l1",
            axis=Vector3(0, 0, 1),
            mimic=JointMimic(joint="ghost"),
        )
    )
    check = MimicChainCheck()
    check.run(empty_robot, result)
    assert any("Invalid mimic target" in err.title for err in result.errors)


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
