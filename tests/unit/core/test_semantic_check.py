from linkforge_core.composer.robot_builder import RobotBuilder
from linkforge_core.validation.checks import SemanticCheck
from linkforge_core.validation.result import ValidationResult


def test_semantic_check_group_cycles():
    builder = RobotBuilder("cycle_robot")
    builder.link("base").root()

    # Create circular subgroup dependency
    # Group A -> Group B -> Group A
    semantic = builder.semantic
    semantic.group("group_a", subgroups=["group_b"])
    semantic.group("group_b", subgroups=["group_a"])

    robot = builder.build(validate=False)
    check = SemanticCheck()
    result = ValidationResult()
    check.run(robot, result)

    assert any("Circular subgroup dependency" in str(issue) for issue in result.errors)


def test_semantic_check_invalid_references():
    builder = RobotBuilder("ref_robot")
    builder.link("base").root()

    # Reference non-existent links/joints in group
    semantic = builder.semantic
    semantic.group("bad_group", links=["non_existent_link"], joints=["non_existent_joint"])

    robot = builder.build(validate=False)
    check = SemanticCheck()
    result = ValidationResult()
    check.run(robot, result)

    assert any(
        "Group 'bad_group' references non-existent link 'non_existent_link'" in str(issue)
        for issue in result.errors
    )
    assert any(
        "Group 'bad_group' references non-existent joint 'non_existent_joint'" in str(issue)
        for issue in result.errors
    )


def test_semantic_check_end_effector_invalid():
    builder = RobotBuilder("ee_robot")
    builder.link("base").root()

    semantic = builder.semantic
    # EE group doesn't exist
    semantic.end_effector("gripper", group="non_existent_group", parent_link="base")

    robot = builder.build(validate=False)
    check = SemanticCheck()
    result = ValidationResult()
    check.run(robot, result)

    assert any(
        "End effector 'gripper' references non-existent group 'non_existent_group'" in str(issue)
        for issue in result.errors
    )
