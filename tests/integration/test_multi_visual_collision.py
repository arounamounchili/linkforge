"""Tests for multiple visual/collision elements per link (URDF spec compliance)."""

from __future__ import annotations

from linkforge.core.generators.urdf import URDFGenerator
from linkforge.core.models import Box, Collision, Cylinder, Link, Sphere, Vector3, Visual
from linkforge.core.parsers.urdf_parser import parse_urdf_string


class TestMultipleVisualElements:
    """Test multiple visual elements per link."""

    def test_link_with_multiple_visuals(self):
        """Test creating link with multiple visual elements."""
        geom1 = Box(size=Vector3(1.0, 1.0, 1.0))
        geom2 = Cylinder(radius=0.5, length=1.0)

        v1 = Visual(geometry=geom1, name="box_visual")
        v2 = Visual(geometry=geom2, name="cylinder_visual")

        link = Link(name="multi_visual_link", visuals=[v1, v2])

        assert len(link.visuals) == 2
        assert link.visuals[0].name == "box_visual"
        assert link.visuals[1].name == "cylinder_visual"

    def test_link_with_single_visual(self):
        """Test link with single visual element (list with one item)."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=geom)

        link = Link(name="single_visual_link", visuals=[visual])

        assert len(link.visuals) == 1
        assert link.visuals[0] == visual

    def test_link_with_no_visuals(self):
        """Test link with empty visuals list."""
        link = Link(name="no_visual_link")

        assert len(link.visuals) == 0


class TestMultipleCollisionElements:
    """Test multiple collision elements per link."""

    def test_link_with_multiple_collisions(self):
        """Test creating link with multiple collision elements."""
        geom1 = Box(size=Vector3(1.0, 1.0, 1.0))
        geom2 = Sphere(radius=0.5)

        c1 = Collision(geometry=geom1, name="box_collision")
        c2 = Collision(geometry=geom2, name="sphere_collision")

        link = Link(name="multi_collision_link", collisions=[c1, c2])

        assert len(link.collisions) == 2
        assert link.collisions[0].name == "box_collision"
        assert link.collisions[1].name == "sphere_collision"

    def test_link_with_single_collision(self):
        """Test link with single collision element."""
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        collision = Collision(geometry=geom)

        link = Link(name="single_collision_link", collisions=[collision])

        assert len(link.collisions) == 1
        assert link.collisions[0] == collision

    def test_link_with_no_collisions(self):
        """Test link with empty collisions list."""
        link = Link(name="no_collision_link")

        assert len(link.collisions) == 0


class TestURDFParserMultipleElements:
    """Test URDF parser with multiple visual/collision elements."""

    def test_parse_link_with_multiple_visuals(self):
        """Test parsing link with multiple visual elements."""
        urdf_xml = """
        <robot name="test">
            <link name="multi_link">
                <visual name="visual1">
                    <geometry><box size="1 1 1"/></geometry>
                </visual>
                <visual name="visual2">
                    <geometry><cylinder radius="0.5" length="1.0"/></geometry>
                </visual>
            </link>
        </robot>
        """
        robot = parse_urdf_string(urdf_xml)
        link = robot.links[0]

        assert len(link.visuals) == 2
        assert link.visuals[0].name == "visual1"
        assert link.visuals[1].name == "visual2"
        assert isinstance(link.visuals[0].geometry, Box)
        assert isinstance(link.visuals[1].geometry, Cylinder)

    def test_parse_link_with_multiple_collisions(self):
        """Test parsing link with multiple collision elements."""
        urdf_xml = """
        <robot name="test">
            <link name="multi_link">
                <collision name="collision1">
                    <geometry><box size="1 1 1"/></geometry>
                </collision>
                <collision name="collision2">
                    <geometry><sphere radius="0.5"/></geometry>
                </collision>
            </link>
        </robot>
        """
        robot = parse_urdf_string(urdf_xml)
        link = robot.links[0]

        assert len(link.collisions) == 2
        assert link.collisions[0].name == "collision1"
        assert link.collisions[1].name == "collision2"

    def test_parse_link_with_unnamed_elements(self):
        """Test parsing visual/collision without name attributes."""
        urdf_xml = """
        <robot name="test">
            <link name="test_link">
                <visual>
                    <geometry><box size="1 1 1"/></geometry>
                </visual>
                <visual>
                    <geometry><box size="2 2 2"/></geometry>
                </visual>
            </link>
        </robot>
        """
        robot = parse_urdf_string(urdf_xml)
        link = robot.links[0]

        assert len(link.visuals) == 2
        assert link.visuals[0].name is None
        assert link.visuals[1].name is None


class TestURDFGeneratorMultipleElements:
    """Test URDF generator with multiple visual/collision elements."""

    def test_generate_multiple_visuals(self):
        """Test generating URDF with multiple visual elements."""
        from linkforge.core.models import Robot

        robot = Robot(name="test_robot")

        geom1 = Box(size=Vector3(1.0, 1.0, 1.0))
        geom2 = Cylinder(radius=0.5, length=1.0)

        v1 = Visual(geometry=geom1, name="visual1")
        v2 = Visual(geometry=geom2, name="visual2")

        link = Link(name="test_link", visuals=[v1, v2])
        robot.add_link(link)

        generator = URDFGenerator()
        urdf_xml = generator.generate(robot)

        # Verify both visuals are in the output
        assert '<visual name="visual1">' in urdf_xml
        assert '<visual name="visual2">' in urdf_xml
        assert urdf_xml.count("<visual") == 2

    def test_generate_multiple_collisions(self):
        """Test generating URDF with multiple collision elements."""
        from linkforge.core.models import Robot

        robot = Robot(name="test_robot")

        geom1 = Box(size=Vector3(1.0, 1.0, 1.0))
        geom2 = Sphere(radius=0.5)

        c1 = Collision(geometry=geom1, name="collision1")
        c2 = Collision(geometry=geom2, name="collision2")

        link = Link(name="test_link", collisions=[c1, c2])
        robot.add_link(link)

        generator = URDFGenerator()
        urdf_xml = generator.generate(robot)

        # Verify both collisions are in the output
        assert '<collision name="collision1">' in urdf_xml
        assert '<collision name="collision2">' in urdf_xml
        assert urdf_xml.count("<collision") == 2


class TestRoundTripMultipleElements:
    """Test round-trip fidelity for multiple visual/collision elements."""

    def test_roundtrip_multiple_visuals(self):
        """Test import -> export -> import preserves multiple visuals."""
        urdf_xml = """
        <robot name="test">
            <link name="test_link">
                <visual name="v1">
                    <geometry><box size="1 1 1"/></geometry>
                </visual>
                <visual name="v2">
                    <geometry><cylinder radius="0.5" length="1.0"/></geometry>
                </visual>
                <visual name="v3">
                    <geometry><sphere radius="0.3"/></geometry>
                </visual>
            </link>
        </robot>
        """

        # Parse
        robot1 = parse_urdf_string(urdf_xml)
        assert len(robot1.links[0].visuals) == 3

        # Generate
        generator = URDFGenerator()
        urdf_xml2 = generator.generate(robot1)

        # Parse again
        robot2 = parse_urdf_string(urdf_xml2)
        assert len(robot2.links[0].visuals) == 3

        # Verify names preserved
        assert robot2.links[0].visuals[0].name == "v1"
        assert robot2.links[0].visuals[1].name == "v2"
        assert robot2.links[0].visuals[2].name == "v3"

    def test_roundtrip_multiple_collisions(self):
        """Test import -> export -> import preserves multiple collisions."""
        urdf_xml = """
        <robot name="test">
            <link name="test_link">
                <collision name="c1">
                    <geometry><box size="1 1 1"/></geometry>
                </collision>
                <collision name="c2">
                    <geometry><box size="2 2 2"/></geometry>
                </collision>
            </link>
        </robot>
        """

        robot1 = parse_urdf_string(urdf_xml)
        assert len(robot1.links[0].collisions) == 2

        generator = URDFGenerator()
        urdf_xml2 = generator.generate(robot1)

        robot2 = parse_urdf_string(urdf_xml2)
        assert len(robot2.links[0].collisions) == 2
        assert robot2.links[0].collisions[0].name == "c1"
        assert robot2.links[0].collisions[1].name == "c2"


class TestExportCheckboxBehavior:
    """Test that export checkboxes control visual/collision export.

    These tests verify the expected behavior when use_visual_geometry=False
    or export_collision=False in Blender properties. The actual checkbox
    logic is in converters.py lines 413 and 450.
    """

    def test_link_with_no_visuals_exports_correctly(self):
        """Test that link with empty visuals list generates valid URDF."""
        from linkforge.core.models import Robot

        robot = Robot(name="test_robot")
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        collision = Collision(geometry=geom)

        # Link with collision but NO visuals (use_visual_geometry=False case)
        link = Link(name="test_link", visuals=[], collisions=[collision])
        robot.add_link(link)

        generator = URDFGenerator()
        urdf_xml = generator.generate(robot)

        # Should have collision but no visual elements
        assert "<collision>" in urdf_xml
        assert "<visual>" not in urdf_xml

        # Verify round-trip
        robot2 = parse_urdf_string(urdf_xml)
        assert len(robot2.links[0].visuals) == 0
        assert len(robot2.links[0].collisions) == 1

    def test_link_with_no_collisions_exports_correctly(self):
        """Test that link with empty collisions list generates valid URDF."""
        from linkforge.core.models import Robot

        robot = Robot(name="test_robot")
        geom = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=geom)

        # Link with visual but NO collisions (export_collision=False case)
        link = Link(name="test_link", visuals=[visual], collisions=[])
        robot.add_link(link)

        generator = URDFGenerator()
        urdf_xml = generator.generate(robot)

        # Should have visual but no collision elements
        assert "<visual>" in urdf_xml
        assert "<collision>" not in urdf_xml

        # Verify round-trip
        robot2 = parse_urdf_string(urdf_xml)
        assert len(robot2.links[0].visuals) == 1
        assert len(robot2.links[0].collisions) == 0

    def test_link_with_neither_visual_nor_collision(self):
        """Test that link with no geometry at all is valid."""
        from linkforge.core.models import Robot

        robot = Robot(name="test_robot")

        # Link with NO visuals and NO collisions (both checkboxes unchecked)
        link = Link(name="test_link", visuals=[], collisions=[])
        robot.add_link(link)

        generator = URDFGenerator()
        urdf_xml = generator.generate(robot)

        # Should have neither visual nor collision elements
        assert "<visual>" not in urdf_xml
        assert "<collision>" not in urdf_xml

        # Verify round-trip
        robot2 = parse_urdf_string(urdf_xml)
        assert len(robot2.links[0].visuals) == 0
        assert len(robot2.links[0].collisions) == 0
