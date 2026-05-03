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

    # New MoveIt 2 features
    from linkforge_core.models.srdf import SrdfSphere

    semantic.virtual_joint("vj1", "base_link", "world", "fixed")
    semantic.enable_collisions("base_link", "link1", reason="Testing")
    semantic.disable_default_collisions("base_link")
    semantic.approximate_link_collision("link1", [SrdfSphere(0, 0, 0, 0.1)])
    semantic.joint_property("base_link_to_link1", "p1", "v1")

    robot = builder.build()

    # 2. Generate SRDF
    generator = SRDFGenerator(pretty_print=True)
    srdf_str = generator.generate(robot)

    # 3. Parse SRDF back
    parser = SRDFParser()
    semantic_parsed = parser.parse_string(srdf_str)

    # 4. Verify equality
    assert len(semantic_parsed.groups) == 2
    assert len(semantic_parsed.virtual_joints) == 1
    assert len(semantic_parsed.enabled_collisions) == 1
    assert len(semantic_parsed.no_default_collision_links) == 1
    assert len(semantic_parsed.link_sphere_approximations) == 1
    assert len(semantic_parsed.joint_properties) == 1

    group_arm = next(g for g in semantic_parsed.groups if g.name == "arm")
    assert "base_link" in group_arm.links
    assert "base_link_to_link1" in group_arm.joints

    state_home = next(s for s in semantic_parsed.group_states if s.name == "home")
    assert state_home.joint_values["base_link_to_link1"] == (0.0,)

    vj = semantic_parsed.virtual_joints[0]
    assert vj.name == "vj1"
    assert vj.parent_frame == "world"

    ec = semantic_parsed.enabled_collisions[0]
    assert ec.link1 == "base_link"
    assert ec.reason == "Testing"

    lsa = semantic_parsed.link_sphere_approximations[0]
    assert lsa.link == "link1"
    assert len(lsa.spheres) == 1
    assert lsa.spheres[0].radius == 0.1

    jp = semantic_parsed.joint_properties[0]
    assert jp.joint_name == "base_link_to_link1"
    assert jp.property_name == "p1"
    assert jp.value == "v1"
