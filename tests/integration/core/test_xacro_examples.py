"""Integration tests for XACRO example files."""

from pathlib import Path

from linkforge_core.parsers.xacro_parser import XACROParser


def test_xacro_roundtrip_master_structure(examples_dir: Path):
    """Test roundtrip.xacro parses into a valid robot with expected features."""
    parser = XACROParser()
    # The XACROParser.parse() handles search path setup
    robot = parser.parse(examples_dir / "xacro" / "roundtrip.xacro")

    assert robot.name == "xacro_roundtrip_robot"
    # base_link + arm_link (if use_arm=true)
    assert len(robot.links) == 2
    assert len(robot.joints) == 1

    # Check link names
    link_names = {link.name for link in robot.links}
    assert "base_link" in link_names
    assert "arm_link" in link_names

    # Verify math resolution (from macros/inertials.xacro)
    # base_mass 5.0, width 0.5, height 0.2, depth 0.5
    # ixx = (5/12) * (0.2^2 + 0.5^2) = (5/12) * (0.04 + 0.25) = (5/12) * 0.29 = 0.120833
    base_link = next(link for link in robot.links if link.name == "base_link")
    assert round(base_link.inertial.inertia.ixx, 6) == 0.120833


def test_xacro_roundtrip_with_disabled_arm(examples_dir: Path):
    """Test roundtrip.xacro with argument-driven conditional logic."""
    parser = XACROParser()
    # Arguments are now passed directly to parse()
    robot = parser.parse(examples_dir / "xacro" / "roundtrip.xacro", use_arm="false")

    assert robot.name == "xacro_roundtrip_robot"
    # base_link + no_arm_payload
    link_names = {link.name for link in robot.links}
    assert "base_link" in link_names
    assert "no_arm_payload" in link_names
    assert "arm_link" not in link_names
