from linkforge_core.generators.srdf_generator import SRDFGenerator
from linkforge_core.models.robot import Robot
from linkforge_core.parsers.srdf_parser import SRDFParser
from linkforge_core.parsers.urdf_parser import URDFParser

# Simplified URDF for integration testing
SAMPLE_URDF = """<?xml version="1.0"?>
<robot name="panda">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="87" lower="-2.8973" upper="2.8973" velocity="2.1750"/>
  </joint>
</robot>
"""

# Corresponding SRDF
SAMPLE_SRDF = """<?xml version="1.0"?>
<robot name="panda">
  <group name="panda_arm">
    <link name="base_link"/>
    <link name="link1"/>
    <joint name="joint1"/>
  </group>
  <group_state name="home" group="panda_arm">
    <joint name="joint1" value="0.0"/>
  </group_state>
  <virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_link"/>
  <disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
</robot>
"""


def test_full_robot_description_integration():
    """Verify that URDF and SRDF can be combined into a single Robot model."""
    # 1. Parse URDF
    urdf_parser = URDFParser()
    robot = urdf_parser.parse_string(SAMPLE_URDF)

    assert robot.name == "panda"
    assert len(robot.links) == 2
    assert robot.semantic is None  # Initially empty

    # 2. Parse SRDF and update the same robot model
    srdf_parser = SRDFParser()
    srdf_parser.parse_string(SAMPLE_SRDF, robot=robot)

    # 3. Verify Integration
    assert robot.semantic is not None
    assert robot.semantic.virtual_joints[0].name == "virtual_joint"
    assert robot.semantic.groups[0].name == "panda_arm"
    assert "base_link" in robot.semantic.groups[0].links

    # Check joint consistency (SRDF joint exists in URDF robot)
    srdf_joint_name = robot.semantic.groups[0].joints[0]
    assert any(j.name == srdf_joint_name for j in robot.joints)

    # 4. Generate SRDF from the unified model
    generator = SRDFGenerator()
    generated_srdf = generator.generate(robot)

    assert 'name="panda"' in generated_srdf
    assert "panda_arm" in generated_srdf
    assert "virtual_joint" in generated_srdf
    assert 'reason="Adjacent"' in generated_srdf


def test_srdf_parser_replaces_semantic_in_existing_robot():
    """Ensure parsing multiple SRDFs correctly updates/overwrites the semantic model."""
    robot = Robot(name="test")
    parser = SRDFParser()

    # First SRDF
    xml1 = '<robot name="test"><passive_joint name="pj1"/></robot>'
    parser.parse_string(xml1, robot=robot)
    assert len(robot.semantic.passive_joints) == 1

    # Second SRDF should overwrite the SemanticRobotDescription object
    xml2 = '<robot name="test"><virtual_joint name="vj1" type="fixed" parent_frame="w" child_link="l"/></robot>'
    parser.parse_string(xml2, robot=robot)

    assert len(robot.semantic.passive_joints) == 0
    assert len(robot.semantic.virtual_joints) == 1
