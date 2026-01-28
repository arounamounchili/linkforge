"""Tests for XACRO files with nested includes.

This module tests that LinkForge correctly handles XACRO files that use
<xacro:include> directives with relative paths.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest
from linkforge_core.parsers.urdf_parser import URDFParser


def test_xacro_with_relative_includes():
    """Test that XACRO files with relative includes are processed correctly."""
    from linkforge_core.parsers import XacroResolver

    # Create temporary directory structure
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)
        urdf_dir = tmp_path / "urdf"
        common_dir = tmp_path / "common"
        urdf_dir.mkdir()
        common_dir.mkdir()

        # Create included file with properties
        materials_xacro = common_dir / "materials.xacro"
        materials_xacro.write_text(
            """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_length" value="0.02"/>

  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
</robot>
"""
        )

        # Create main XACRO file with relative include
        robot_xacro = urdf_dir / "robot.xacro"
        robot_xacro.write_text(
            """<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="../common/materials.xacro"/>

  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
"""
        )

        # Test: Process XACRO with CWD change (correct behavior)
        import os

        old_cwd = os.getcwd()
        try:
            resolver = XacroResolver(search_paths=[urdf_dir])
            urdf_string = resolver.resolve_file(robot_xacro)
        finally:
            os.chdir(old_cwd)

        # Parse the generated URDF
        robot = URDFParser().parse_string(urdf_string)

        # Verify robot structure
        assert robot.name == "test_robot"
        assert len(robot.links) == 2
        assert len(robot.joints) == 1

        # Verify properties were substituted
        wheel_link = next(link for link in robot.links if link.name == "wheel")
        assert len(wheel_link.visuals) == 1

        from linkforge_core.models import Cylinder

        geometry = wheel_link.visuals[0].geometry
        assert isinstance(geometry, Cylinder)
        assert geometry.radius == pytest.approx(0.05)
        assert geometry.length == pytest.approx(0.02)

        # Verify material from included file is present
        assert wheel_link.visuals[0].material is not None
        assert wheel_link.visuals[0].material.name == "black"
        assert wheel_link.visuals[0].material.color.r == pytest.approx(0.0)


def test_xacro_nested_includes():
    """Test XACRO files with multiple levels of includes."""
    from linkforge_core.parsers import XacroResolver

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)
        urdf_dir = tmp_path / "urdf"
        common_dir = tmp_path / "common"
        macros_dir = common_dir / "macros"
        urdf_dir.mkdir()
        common_dir.mkdir()
        macros_dir.mkdir()

        # Level 3: Base constants
        constants_xacro = macros_dir / "constants.xacro"
        constants_xacro.write_text(
            """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="base_mass" value="10.0"/>
</robot>
"""
        )

        # Level 2: Include constants
        materials_xacro = common_dir / "materials.xacro"
        materials_xacro.write_text(
            """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="macros/constants.xacro"/>
</robot>
"""
        )

        # Level 1: Main file includes materials
        robot_xacro = urdf_dir / "robot.xacro"
        robot_xacro.write_text(
            """<?xml version="1.0"?>
<robot name="nested_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="../common/materials.xacro"/>

  <link name="base_link">
    <inertial>
      <mass value="${base_mass}"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
</robot>
"""
        )

        # Process with correct CWD handling
        import os

        old_cwd = os.getcwd()
        try:
            resolver = XacroResolver(search_paths=[urdf_dir])
            urdf_string = resolver.resolve_file(robot_xacro)
        finally:
            os.chdir(old_cwd)

        # Parse and verify
        robot = URDFParser().parse_string(urdf_string)
        assert robot.name == "nested_robot"
        assert len(robot.links) == 1

        base_link = robot.links[0]
        assert base_link.inertial is not None
        assert base_link.inertial.mass == pytest.approx(10.0)


def test_xacro_absolute_path_includes():
    """Test that absolute path includes still work."""
    from linkforge_core.parsers import XacroResolver

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)

        # Create included file at absolute path
        materials_xacro = tmp_path / "materials.xacro"
        materials_xacro.write_text(
            """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="test_value" value="42.0"/>
</robot>
"""
        )

        # Create main XACRO with absolute include path
        robot_xacro = tmp_path / "robot.xacro"
        robot_xacro.write_text(
            f"""<?xml version="1.0"?>
<robot name="abs_path_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{materials_xacro}"/>

  <link name="base_link">
    <inertial>
      <mass value="${{test_value}}"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
</robot>
"""
        )

        # Process with CWD change
        import os

        old_cwd = os.getcwd()
        try:
            resolver = XacroResolver()
            urdf_string = resolver.resolve_file(robot_xacro)
        finally:
            os.chdir(old_cwd)

        # Parse and verify
        robot = URDFParser().parse_string(urdf_string)
        assert robot.links[0].inertial.mass == pytest.approx(42.0)
