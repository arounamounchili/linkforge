"""Tests for XACRO namespace prefix verification."""

from linkforge_core import XACROGenerator
from linkforge_core.models import (
    Box,
    Link,
    Robot,
    Vector3,
    Visual,
)


def test_xacro_namespace_prefix():
    """Verify that XACRO generator uses the 'xacro:' prefix instead of 'ns0:'."""
    from linkforge_core.models import Color, Material

    robot = Robot(name="test_robot")

    # Create a link with a material to trigger property extraction
    mat = Material(name="3d_printed", color=Color(1, 0.82, 0.12, 1))
    link = Link(
        name="base_link", visuals=[Visual(geometry=Box(size=Vector3(1.0, 1.0, 1.0)), material=mat)]
    )
    robot.add_link(link)

    # Generate XACRO with advanced mode
    generator = XACROGenerator(advanced_mode=True, extract_materials=True)
    xacro_str = generator.generate(robot)

    # 1. Check for the standard namespace declaration
    assert 'xmlns:xacro="http://www.ros.org/wiki/xacro"' in xacro_str

    # 2. Check that we DON'T have the generic ns0 declaration
    assert "xmlns:ns0" not in xacro_str

    # 3. Check for the xacro prefix in tags
    assert "<xacro:property" in xacro_str
    assert "<ns0:property" not in xacro_str
