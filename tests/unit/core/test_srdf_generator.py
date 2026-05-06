from linkforge_core.generators.srdf_generator import SRDFGenerator
from linkforge_core.models.robot import Robot
from linkforge_core.models.srdf import (
    Chain,
    CollisionPair,
    EndEffector,
    GroupState,
    JointProperty,
    LinkSphereApproximation,
    PassiveJoint,
    PlanningGroup,
    SemanticRobotDescription,
    SrdfSphere,
    VirtualJoint,
)
from linkforge_core.parsers.srdf_parser import SRDFParser

SAMPLE_SRDF = """<?xml version="1.0"?>
<robot name="test_robot">
  <virtual_joint name="world_joint" type="fixed" parent_frame="world" child_link="base_link"/>
  <group name="arm">
    <link name="link1"/>
    <joint name="joint1"/>
    <chain base_link="base_link" tip_link="tool0"/>
    <group name="hand"/>
  </group>
  <group_state name="home" group="arm">
    <joint name="joint1" value="0.0"/>
    <joint name="multi_joint" value="0.0 1.0"/>
  </group_state>
  <end_effector name="hand" group="hand_group" parent_link="link4" parent_group="arm"/>
  <passive_joint name="passive_1"/>
  <disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
  <enable_collisions link1="link1" link2="link3" reason="Testing"/>
  <disable_default_collisions link="link1"/>
  <link_sphere_approximation link="link1">
    <sphere center="0.1 0.2 0.3" radius="0.5"/>
  </link_sphere_approximation>
  <joint_property joint_name="joint1" property_name="prop1" value="val1"/>
</robot>
"""


def test_srdf_generator_basic():
    """Test generating SRDF from a manually constructed Robot model."""
    robot = Robot(name="gen_robot")
    robot.semantic = SemanticRobotDescription(
        virtual_joints=[VirtualJoint("vj", "fixed", "world", "base")],
        groups=[
            PlanningGroup(
                "arm",
                links=["l1"],
                joints=["j1"],
                chains=[Chain("base", "tool")],
                subgroups=["grp1"],
            )
        ],
        group_states=[GroupState("pose", "arm", {"j1": (1.57,)})],
        end_effectors=[EndEffector("ee", "grp1", "l4")],
        passive_joints=[PassiveJoint("pj1")],
        disabled_collisions=[CollisionPair("l1", "l2", "reason1")],
        enabled_collisions=[CollisionPair("l1", "l3")],
        no_default_collision_links=["l1"],
        link_sphere_approximations=[
            LinkSphereApproximation("l1", [SrdfSphere(0.1, 0.2, 0.3, 0.5)])
        ],
        joint_properties=[JointProperty("j1", "p1", "v1")],
    )

    generator = SRDFGenerator(pretty_print=True)
    xml_out = generator.generate(robot, validate=False)

    assert 'name="gen_robot"' in xml_out
    assert "<virtual_joint" in xml_out
    assert 'name="vj"' in xml_out
    assert 'type="fixed"' in xml_out
    assert 'value="1.57"' in xml_out
    assert 'reason="reason1"' in xml_out
    assert "<enable_collisions" in xml_out
    assert '<disable_default_collisions link="l1"' in xml_out
    assert '<link_sphere_approximation link="l1"' in xml_out
    assert "<joint_property" in xml_out


def test_srdf_generator_round_trip():
    """Test the idempotency of Parse -> Generate -> Parse."""
    parser = SRDFParser()
    semantic_1 = parser.parse_string(SAMPLE_SRDF)
    robot_1 = Robot(name="test_robot")
    robot_1.semantic = semantic_1

    generator = SRDFGenerator(pretty_print=True)
    xml_generated = generator.generate(robot_1, validate=False)

    # Parse the generated XML
    semantic_2 = parser.parse_string(xml_generated)

    # Compare semantic models
    assert semantic_1 == semantic_2

    # Check specific details
    assert len(semantic_2.groups) == 1
    assert semantic_2.groups[0].name == "arm"
    assert semantic_2.groups[0].chains[0].base_link == "base_link"
    assert semantic_2.group_states[0].joint_values["joint1"] == (0.0,)
    assert semantic_2.group_states[0].joint_values["multi_joint"] == (0.0, 1.0)
    assert len(semantic_2.enabled_collisions) == 1
    assert len(semantic_2.no_default_collision_links) == 1
    assert len(semantic_2.link_sphere_approximations) == 1
    assert len(semantic_2.joint_properties) == 1


def test_srdf_generator_empty_semantic():
    """Test generating SRDF for a robot without semantic data."""
    robot = Robot(name="empty_robot")
    generator = SRDFGenerator()
    xml_out = generator.generate(robot, validate=False)
    assert '<robot name="empty_robot"' in xml_out
    assert "<virtual_joint" not in xml_out


def test_srdf_generator_no_reason_collision():
    """Test generating SRDF with a disabled collision that has no reason."""
    robot = Robot(name="test")
    robot.semantic = SemanticRobotDescription(disabled_collisions=[CollisionPair("l1", "l2")])
    generator = SRDFGenerator()
    xml_out = generator.generate(robot, validate=False)
    assert 'link1="l1"' in xml_out
    assert 'link2="l2"' in xml_out
    assert "reason" not in xml_out
