from linkforge_core.generators.srdf_generator import SRDFGenerator
from linkforge_core.models.robot import Robot
from linkforge_core.models.srdf import (
    DisabledCollision,
    EndEffector,
    GroupState,
    PassiveJoint,
    PlanningGroup,
    SemanticRobotDescription,
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
  </group_state>
  <end_effector name="hand" group="hand_group" parent_link="link4" parent_group="arm"/>
  <passive_joint name="passive_1"/>
  <disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
</robot>
"""


def test_srdf_generator_basic():
    """Test generating SRDF from a manually constructed Robot model."""
    robot = Robot(name="gen_robot")
    robot.semantic = SemanticRobotDescription(
        virtual_joints=[VirtualJoint("vj", "fixed", "world", "base")],
        groups=[
            PlanningGroup(
                "arm", links=["l1"], joints=["j1"], chains=[("base", "tool")], subgroups=["grp1"]
            )
        ],
        group_states=[GroupState("pose", "arm", {"j1": 1.57})],
        end_effectors=[EndEffector("ee", "grp1", "l4")],
        passive_joints=[PassiveJoint("pj1")],
        disabled_collisions=[DisabledCollision("l1", "l2", "reason1")],
    )

    generator = SRDFGenerator(pretty_print=True)
    xml_out = generator.generate(robot)

    assert 'name="gen_robot"' in xml_out
    assert "<virtual_joint" in xml_out
    assert 'name="vj"' in xml_out
    assert 'type="fixed"' in xml_out
    assert 'value="1.57"' in xml_out
    assert 'reason="reason1"' in xml_out


def test_srdf_generator_round_trip():
    """Test the idempotency of Parse -> Generate -> Parse."""
    parser = SRDFParser()
    robot_1 = parser.parse_string(SAMPLE_SRDF)

    generator = SRDFGenerator(pretty_print=True)
    xml_generated = generator.generate(robot_1)

    # Parse the generated XML
    robot_2 = parser.parse_string(xml_generated)

    # Compare semantic models
    assert robot_1.semantic == robot_2.semantic
    assert robot_2.name == "test_robot"

    # Check specific details
    assert len(robot_2.semantic.groups) == 1
    assert robot_2.semantic.groups[0].name == "arm"
    assert ("base_link", "tool0") in robot_2.semantic.groups[0].chains
    assert robot_2.semantic.group_states[0].joint_values["joint1"] == 0.0


def test_srdf_generator_empty_semantic():
    """Test generating SRDF for a robot without semantic data."""
    robot = Robot(name="empty_robot")
    generator = SRDFGenerator()
    xml_out = generator.generate(robot)
    assert '<robot name="empty_robot"' in xml_out
    assert "<virtual_joint" not in xml_out


def test_srdf_generator_no_reason_collision():
    """Test generating SRDF with a disabled collision that has no reason."""
    robot = Robot(name="test")
    robot.semantic = SemanticRobotDescription(disabled_collisions=[DisabledCollision("l1", "l2")])
    generator = SRDFGenerator()
    xml_out = generator.generate(robot)
    assert 'link1="l1"' in xml_out
    assert 'link2="l2"' in xml_out
    assert "reason" not in xml_out
