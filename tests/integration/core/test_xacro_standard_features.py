import tempfile
from pathlib import Path

from linkforge_core.parsers.xacro_parser import XACROParser


def test_xacro_standard_parities() -> None:
    """Verify standard ROS XACRO parity including $(eval) and recursive dot-access.

    Checks compliance using synthetic XACRO and YAML structures without
    relying on external file dependencies.
    """
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)
        config_dir = tmp_path / "config"
        urdf_dir = tmp_path / "urdf"
        config_dir.mkdir()
        urdf_dir.mkdir()

        # 1. Create a nested YAML config
        yaml_content = """
kinematics:
  base:
    x: 0.1
    y: 0.2
    z: 0.3
    roll: 1.5707
  arm:
    link1:
      length: 0.5
      mass: 2.0
"""
        yaml_file = config_dir / "kinematics.yaml"
        yaml_file.write_text(yaml_content)

        # 2. Create a XACRO file using dot-access and $(eval)
        xacro_content = f"""<?xml version="1.0"?>
<robot name="compat_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="use_eval" default="true"/>
  <xacro:property name="kin" value="${{load_yaml('{yaml_file}')}}"/>

  <link name="base_link">
    <visual>
      <origin xyz="$(eval kin.kinematics.base.x) ${{kin.kinematics.base.y}} 0" rpy="${{kin.kinematics.base.roll}} 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <xacro:if value="$(arg use_eval)">
    <link name="arm_link">
      <inertial>
        <mass value="${{kin.kinematics.arm.link1.mass}}"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="base_to_arm" type="fixed">
      <parent link="base_link"/>
      <child link="arm_link"/>
      <origin xyz="0 0 $(eval kin.kinematics.arm.link1.length / 2.0)"/>
    </joint>
  </xacro:if>
</robot>
"""
        xacro_file = urdf_dir / "robot.xacro"
        xacro_file.write_text(xacro_content)

        # 3. Resolve and verify
        parser = XACROParser()
        robot = parser.parse(xacro_file)

        assert robot.name == "compat_robot"
        assert len(robot.links) == 2
        assert len(robot.joints) == 1

        # Verify dot-access resolution in attributes
        next(link for link in robot.links if link.name == "base_link")

        arm_link = next(link for link in robot.links if link.name == "arm_link")
        assert arm_link.inertial.mass == 2.0

        joint = robot.joints[0]
        # origin z: "$(eval kin.kinematics.arm.link1.length / 2.0)" -> 0.5 / 2.0 = 0.25
        assert joint.origin.xyz.z == 0.25
        assert joint.origin.xyz.x == 0.0
