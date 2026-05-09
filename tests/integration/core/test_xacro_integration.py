"""Xacro integration tests.

This module verifies Xacro features like macros, includes, and parameter evaluation
within a robotic model context.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from linkforge_core.models import Robot
from linkforge_core.parsers.urdf_parser import URDFParser
from linkforge_core.parsers.xacro_parser import XACROParser


@pytest.fixture
def xacro_to_robot():
    """Helper to resolve xacro and parse as robot."""

    def _parse(path: Path) -> Robot:
        xml_str = XACROParser().resolve(path)
        return URDFParser().parse_string(xml_str, source_directory=path.parent)

    return _parse


def test_xacro_includes_and_macros(tmp_path: Path, xacro_to_robot) -> None:
    """Test Xacro includes and macros with complex dependencies."""
    # Create macro file
    macro_file = tmp_path / "macros.xacro"
    macro_file.write_text("""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_link" params="name color">
    <link name="${name}">
      <visual>
        <material name="${color}"/>
      </visual>
    </link>
  </xacro:macro>
</robot>
""")

    # Create main file
    main_file = tmp_path / "main.xacro"
    main_file.write_text(f"""<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{macro_file}"/>
  <xacro:test_link name="base_link" color="red"/>
  <xacro:test_link name="arm_link" color="blue"/>
</robot>
""")

    robot = xacro_to_robot(main_file)

    assert len(robot.links) == 2
    assert robot.links[0].name == "base_link"
    assert robot.links[1].name == "arm_link"


def test_xacro_dimensions_and_math(tmp_path: Path, xacro_to_robot) -> None:
    """Test Xacro math evaluation and dimension passing."""
    xacro_content = """<?xml version="1.0"?>
<robot name="math_bot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="width" value="2.0"/>
  <xacro:property name="height" value="5.0"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="${width} ${width * 2} ${height + 1}"/>
      </geometry>
    </visual>
  </link>
</robot>
"""
    xacro_file = tmp_path / "math.xacro"
    xacro_file.write_text(xacro_content)

    robot = xacro_to_robot(xacro_file)

    from linkforge_core.models import Box

    geom = robot.links[0].visuals[0].geometry
    assert isinstance(geom, Box)
    assert geom.size.x == 2.0
    assert geom.size.y == 4.0
    assert geom.size.z == 6.0


def test_empty_xacro_handling(tmp_path: Path, xacro_to_robot) -> None:
    """Test that nearly empty Xacro files are handled gracefully."""
    xacro_content = """<?xml version="1.0"?>
<robot name="empty" xmlns:xacro="http://www.ros.org/wiki/xacro">
</robot>
"""
    xacro_file = tmp_path / "empty.xacro"
    xacro_file.write_text(xacro_content)

    robot = xacro_to_robot(xacro_file)
    assert robot.name == "empty"
    assert len(robot.links) == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
