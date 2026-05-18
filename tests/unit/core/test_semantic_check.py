from linkforge.core import RobotBuilder, ValidationResult
from linkforge.core.validation import SemanticCheck


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


def test_semantic_check_valid_elements():
    builder = RobotBuilder("valid_robot")
    builder.link("base").root()
    builder.link("tip", parent="base", joint_name="j1")

    # Access the semantic builder to add valid elements
    semantic = builder.semantic
    # Group with valid link and joint
    semantic.group("hand", links=["tip"])
    semantic.group("arm", links=["base", "tip"], joints=["j1"], subgroups=["hand"])

    # Group state referencing valid group
    semantic.group_state("home", group="arm", values={"j1": 0.5})

    # End effector referencing valid group, parent link, and parent group
    semantic.end_effector("gripper", group="hand", parent_link="tip", parent_group="arm")

    # Passive joint referencing valid joint
    semantic.passive_joint("j1")

    robot = builder.build(validate=False)
    check = SemanticCheck()
    result = ValidationResult()
    check.run(robot, result)

    # Everything should be 100% valid!
    assert not result.errors
