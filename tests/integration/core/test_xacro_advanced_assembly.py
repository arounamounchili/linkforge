"""Advanced Xacro integration tests.

This module verifies complex assembly scenarios, including nested namespaces,
conditional model generation, and data-driven configuration.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from linkforge_core.models import Box, Robot
from linkforge_core.parsers.urdf_parser import URDFParser
from linkforge_core.parsers.xacro_parser import RobotXacroError, XACROParser


@pytest.fixture
def xacro_to_robot():
    """Helper to resolve xacro and parse as robot."""

    def _parse(path: Path, **kwargs) -> Robot:
        xml_str = XACROParser().resolve(path, **kwargs)
        return URDFParser().parse_string(xml_str, source_directory=path.parent)

    return _parse


def test_nested_namespaces_and_property_isolation(tmp_path: Path, xacro_to_robot) -> None:
    """Test deeply nested namespaces and property access."""
    # Level 2 (deepest)
    file_c = tmp_path / "sensor.xacro"
    file_c.write_text("""
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:property name="mass" value="0.1"/>
      <xacro:macro name="sensor_link" params="suffix">
        <link name="sensor_${suffix}">
          <inertial>
            <mass value="${mass}"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
          </inertial>
        </link>
      </xacro:macro>
    </robot>
    """)

    # Level 1
    file_b = tmp_path / "arm.xacro"
    file_b.write_text(f'''
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{file_c}" ns="s"/>
      <xacro:property name="mass" value="1.0"/>
      <xacro:macro name="arm_link">
        <link name="arm_link">
          <inertial>
            <mass value="${{mass}}"/>
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
          </inertial>
        </link>
        <xacro:s.sensor_link suffix="arm"/>
      </xacro:macro>
    </robot>
    ''')

    # Level 0 (main)
    file_a = tmp_path / "main.xacro"
    file_a.write_text(f'''
    <robot name="nested_bot" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{file_b}" ns="a"/>
      <xacro:a.arm_link/>

      <!-- Test property access across namespaces -->
      <link name="info_link">
        <note text="Arm mass: ${{a.mass}}, Sensor mass: ${{a.s.mass}}"/>
      </link>
    </robot>
    ''')

    robot = xacro_to_robot(file_a)

    assert len(robot.links) == 3  # arm_link, sensor_arm, info_link

    arm_link = next(link for link in robot.links if link.name == "arm_link")
    sensor_link = next(link for link in robot.links if link.name == "sensor_arm")

    assert arm_link.inertial.mass == 1.0
    assert sensor_link.inertial.mass == 0.1


def test_conditional_model_generation(tmp_path: Path, xacro_to_robot) -> None:
    """Test xacro:if and xacro:unless for toggling robot features."""
    xacro_content = """
    <robot name="cond_bot" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:arg name="use_gpu" default="false"/>
      <xacro:arg name="with_gripper" default="true"/>

      <link name="base"/>

      <xacro:if value="$(arg with_gripper)">
        <link name="gripper"/>
        <joint name="base_to_gripper" type="fixed">
          <parent link="base"/>
          <child link="gripper"/>
        </joint>
      </xacro:if>

      <xacro:unless value="$(arg use_gpu)">
        <link name="cpu_sensor"/>
      </xacro:unless>

      <xacro:if value="$(arg use_gpu)">
        <link name="gpu_sensor"/>
      </xacro:if>
    </robot>
    """
    xacro_file = tmp_path / "cond.xacro"
    xacro_file.write_text(xacro_content)

    # Case 1: Default (gripper=true, gpu=false)
    robot = xacro_to_robot(xacro_file)
    assert any(link.name == "gripper" for link in robot.links)
    assert any(link.name == "cpu_sensor" for link in robot.links)
    assert not any(link.name == "gpu_sensor" for link in robot.links)

    # Case 2: Custom (gripper=false, gpu=true)
    robot = xacro_to_robot(xacro_file, with_gripper=False, use_gpu=True)
    assert not any(link.name == "gripper" for link in robot.links)
    assert not any(link.name == "cpu_sensor" for link in robot.links)
    assert any(link.name == "gpu_sensor" for link in robot.links)


def test_yaml_driven_assembly(tmp_path: Path, xacro_to_robot) -> None:
    """Test using xacro:load_yaml to drive model dimensions."""
    config_file = tmp_path / "config.yaml"
    config_file.write_text("""
    dimensions:
      width: 0.5
      length: 1.2
    material:
      name: "Silver"
      color: "0.7 0.7 0.7 1.0"
    """)

    xacro_content = f"""
    <robot name="yaml_bot" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:property name="config" value="${{load_yaml('{config_file}')}}"/>

      <link name="base">
        <visual>
          <geometry>
            <box size="${{config.dimensions.width}} ${{config.dimensions.length}} 0.1"/>
          </geometry>
          <material name="${{config.material.name}}">
            <color rgba="${{config.material.color}}"/>
          </material>
        </visual>
      </link>
    </robot>
    """
    xacro_file = tmp_path / "yaml.xacro"
    xacro_file.write_text(xacro_content)

    robot = xacro_to_robot(xacro_file)

    link = robot.links[0]
    geom = link.visuals[0].geometry
    assert isinstance(geom, Box)
    assert geom.size.x == 0.5
    assert geom.size.y == 1.2
    assert link.visuals[0].material.name == "Silver"


def test_xacro_error_propagation_to_parser(tmp_path: Path) -> None:
    """Test that xacro errors correctly bubble up through the resolution pipeline."""
    bad_xacro = tmp_path / "bad.xacro"
    bad_xacro.write_text("""
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:macro name="fail">
        <link name="${undefined_var}"/>
      </xacro:macro>
      <xacro:fail/>
    </robot>
    """)

    # Resolving should raise RobotXacroError
    with pytest.raises(RobotXacroError) as excinfo:
        XACROParser().resolve(bad_xacro)
    assert "undefined_var" in str(excinfo.value)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
