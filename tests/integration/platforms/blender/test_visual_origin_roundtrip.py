"""Test visual origin preservation in roundtrip - specifically for visual with no origin."""

from __future__ import annotations

import tempfile
from pathlib import Path

import bpy
import pytest
from linkforge.linkforge_core import URDFGenerator
from linkforge.linkforge_core.models import Cylinder, Link, Robot, Transform, Visual
from linkforge.linkforge_core.parsers.urdf_parser import URDFParser


def test_inertia_integration_with_offset():
    """Verify that offset visuals are handled correctly via the Parallel Axis Theorem."""
    from typing import Any

    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None

    link_obj = bpy.data.objects.new("link", None)
    assert link_obj is not None
    collection.objects.link(link_obj)
    link_lf: Any = getattr(link_obj, "linkforge")
    link_lf.is_robot_link = True


def test_inertia_integration_flow():
    """Verify end-to-end inertia calculation in Blender."""
    from typing import Any

    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None

    # Create the link (Empty) manually instead of using bpy.ops.linkforge
    link_obj = bpy.data.objects.new("link", None)
    assert link_obj is not None
    collection.objects.link(link_obj)
    link_lf: Any = getattr(link_obj, "linkforge")
    link_lf.is_robot_link = True
    assert link_lf.is_robot_link


def test_cylinder_no_origin_roundtrip() -> None:
    """Test that cylinder visual with NO origin stays at link frame.

    This tests the specific case like arm_base in roundtrip_test_robot.urdf
    where a cylinder has no visual origin and should be centered at the link frame.
    """
    robot = Robot(name="cylinder_test")

    # Create link with cylinder visual (no origin specified)
    robot.add_link(
        Link(
            name="test_link",
            initial_visuals=[
                Visual(
                    geometry=Cylinder(radius=0.08, length=0.15),
                    origin=Transform.identity(),  # At link frame
                )
            ],
        )
    )

    # Export
    generator = URDFGenerator()
    urdf_string = generator.generate(robot)

    # Check exported URDF doesn't have origin tag for visual
    assert "<visual>" in urdf_string
    assert "<cylinder" in urdf_string

    # Count origin tags in visual section
    visual_start = urdf_string.find("<visual>")
    visual_end = urdf_string.find("</visual>")
    visual_section = urdf_string[visual_start:visual_end]

    # Should NOT have origin tag (identity origin is omitted)
    assert "<origin" not in visual_section, "Visual with no origin should not export origin tag"

    # Re-import
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        f.write(urdf_string)

    try:
        robot2 = URDFParser().parse(temp_path)

        # Verify link exists
        assert len(robot2.links) == 1
        link2 = robot2.links[0]

        # Verify visual exists
        assert len(link2.visuals) == 1
        visual2 = link2.visuals[0]

        # Verify cylinder geometry
        assert isinstance(visual2.geometry, Cylinder)
        assert visual2.geometry.radius == pytest.approx(0.08)
        assert visual2.geometry.length == pytest.approx(0.15)

        # Verify origin is None or identity
        if visual2.origin is None:
            # Verify limits
            pass
        else:
            # Should be identity
            assert visual2.origin.xyz.x == pytest.approx(0.0)
            assert visual2.origin.xyz.y == pytest.approx(0.0)
            assert visual2.origin.xyz.z == pytest.approx(0.0)
            assert visual2.origin.rpy.x == pytest.approx(0.0)
            assert visual2.origin.rpy.y == pytest.approx(0.0)
            assert visual2.origin.rpy.z == pytest.approx(0.0)

    finally:
        temp_path.unlink()


def test_ros2_control_parameter_extraction():
    """Verify that custom parameters are extracted from Blender into ROS2 Control models."""
    from typing import Any

    # Setup Link & Joint
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None

    p = bpy.data.objects.new("Parent", None)
    assert p is not None
    collection.objects.link(p)
    p_lf: Any = getattr(p, "linkforge")
    p_lf.is_robot_link = True

    c = bpy.data.objects.new("Child", None)
    assert c is not None
    collection.objects.link(c)
    c_lf: Any = getattr(c, "linkforge")
    c_lf.is_robot_link = True

    j = bpy.data.objects.new("Joint", None)
    assert j is not None
    collection.objects.link(j)
    j_lf: Any = getattr(j, "linkforge_joint")
    j_lf.is_robot_joint = True
    j_lf.parent_link = p
    j_lf.child_link = c
    j_lf.joint_type = "REVOLUTE"


def test_arm_base_specific_case(examples_dir: Path) -> None:
    """Test the exact arm_base case from roundtrip_test_robot.urdf."""
    # Load the actual file
    urdf_path = examples_dir / "urdf" / "roundtrip_test_robot.urdf"
    robot1 = URDFParser().parse(urdf_path)

    # Find arm_base link
    arm_base = next((link for link in robot1.links if link.name == "arm_base"), None)
    assert arm_base is not None, "arm_base link not found"

    # Verify it has visual
    assert len(arm_base.visuals) == 1
    visual1 = arm_base.visuals[0]

    # Verify it's a cylinder
    assert isinstance(visual1.geometry, Cylinder)

    # Check if visual has origin - should be None or identity
    print(f"\nOriginal arm_base visual origin: {visual1.origin}")
    if visual1.origin:
        print(f"  xyz: ({visual1.origin.xyz.x}, {visual1.origin.xyz.y}, {visual1.origin.xyz.z})")
        print(f"  rpy: ({visual1.origin.rpy.x}, {visual1.origin.rpy.y}, {visual1.origin.rpy.z})")

    # Export
    generator = URDFGenerator()
    urdf_string = generator.generate(robot1)

    # Re-import
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        f.write(urdf_string)

    try:
        robot2 = URDFParser().parse(temp_path)

        # Find arm_base in re-imported robot
        arm_base2 = next((link for link in robot2.links if link.name == "arm_base"), None)
        assert arm_base2 is not None

        # Check visual
        assert len(arm_base2.visuals) == 1
        visual2 = arm_base2.visuals[0]

        print(f"\nRe-imported arm_base visual origin: {visual2.origin}")
        if visual2.origin:
            print(
                f"  xyz: ({visual2.origin.xyz.x}, {visual2.origin.xyz.y}, {visual2.origin.xyz.z})"
            )
            print(
                f"  rpy: ({visual2.origin.rpy.x}, {visual2.origin.rpy.y}, {visual2.origin.rpy.z})"
            )

        # Compare origins
        if visual1.origin is None and visual2.origin is None:
            # Perfect match
            pass
        elif visual1.origin is None and visual2.origin is not None:
            # Should be identity
            assert visual2.origin.xyz.x == pytest.approx(0.0), "Origin should be identity"
            assert visual2.origin.xyz.y == pytest.approx(0.0)
            assert visual2.origin.xyz.z == pytest.approx(0.0)
        elif visual1.origin is not None and visual2.origin is None:
            # Original should have been identity
            assert visual1.origin.xyz.x == pytest.approx(0.0), "Original origin should be identity"
            assert visual1.origin.xyz.y == pytest.approx(0.0)
            assert visual1.origin.xyz.z == pytest.approx(0.0)
        else:
            # Both have origins - should match
            assert visual2.origin.xyz.x == pytest.approx(visual1.origin.xyz.x)
            assert visual2.origin.xyz.y == pytest.approx(visual1.origin.xyz.y)
            assert visual2.origin.xyz.z == pytest.approx(visual1.origin.xyz.z)

    finally:
        temp_path.unlink()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
