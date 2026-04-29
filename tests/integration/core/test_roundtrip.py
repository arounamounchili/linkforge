from linkforge_core.composer import RobotBuilder, box, cylinder
from linkforge_core.generators.srdf_generator import SRDFGenerator
from linkforge_core.generators.urdf_generator import URDFGenerator
from linkforge_core.models.geometry import Vector3
from linkforge_core.parsers.srdf_parser import SRDFParser
from linkforge_core.parsers.urdf_parser import URDFParser


def test_urdf_roundtrip():
    # 1. Build a complex robot
    builder = RobotBuilder("test_robot")
    builder.link("base_link").visual(box(1, 1, 1)).collision().mass(1.0).root()
    builder.link("link1", parent="base_link").visual(cylinder(0.1, 0.5)).collision().mass(
        0.5
    ).revolute(
        axis=(0, 0, 1), limits=(0, 3.14), effort=10, velocity=1.0, name="base_link_to_link1"
    ).at_origin(xyz=(0, 0, 0.5)).commit()

    robot = builder.build()

    # 2. Generate URDF
    generator = URDFGenerator(pretty_print=True)
    urdf_str = generator.generate(robot)

    # 3. Parse URDF back
    parser = URDFParser()
    robot_parsed = parser.parse_string(urdf_str)

    # 4. Verify equality
    assert robot_parsed.name == robot.name
    assert len(robot_parsed.links) == len(robot.links)
    assert len(robot_parsed.joints) == len(robot.joints)

    # Check specific joint properties
    joint = robot_parsed.get_joint("base_link_to_link1")
    assert joint is not None
    assert joint.axis == Vector3(0.0, 0.0, 1.0)
    assert joint.limits is not None
    assert joint.limits.lower == 0.0
    assert joint.limits.upper == 3.14


def test_srdf_roundtrip():
    # 1. Build a robot with semantic description
    builder = RobotBuilder("test_robot")
    builder.link("base_link").visual(box(1, 1, 1)).collision().mass(1.0).root()
    builder.link("link1", parent="base_link").visual(cylinder(0.1, 0.5)).collision().mass(
        0.5
    ).fixed(name="base_link_to_link1").commit()

    semantic = builder.semantic
    semantic.group("arm", links=["base_link", "link1"], joints=["base_link_to_link1"])
    semantic.group("hand", subgroups=["arm"])
    semantic.group_state("home", group="arm", values={"base_link_to_link1": 0.0})
    semantic.end_effector("gripper", group="arm", parent_link="link1")

    robot = builder.build()

    # 2. Generate SRDF
    generator = SRDFGenerator(pretty_print=True)
    srdf_str = generator.generate(robot)

    # 3. Parse SRDF back
    parser = SRDFParser()
    semantic_parsed = parser.parse_string(srdf_str)

    # 4. Verify equality
    assert len(semantic_parsed.groups) == 2

    group_arm = next(g for g in semantic_parsed.groups if g.name == "arm")
    assert "base_link" in group_arm.links
    assert "base_link_to_link1" in group_arm.joints

    group_hand = next(g for g in semantic_parsed.groups if g.name == "hand")
    assert "arm" in group_hand.subgroups

    state_home = next(s for s in semantic_parsed.group_states if s.name == "home")
    assert state_home.group == "arm"
    assert state_home.joint_values["base_link_to_link1"] == 0.0
