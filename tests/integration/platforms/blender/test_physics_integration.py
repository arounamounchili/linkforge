from __future__ import annotations

from typing import Any

import bpy
import pytest
from linkforge.blender.operators.link_ops import calculate_inertia_for_link


def test_inertia_integration_flow() -> None:
    """Verify end-to-end inertia calculation in Blender.

    1. Create a link with a cube visual.
    2. Trigger automated inertia calculation.
    3. Verify properties match expected physical values.
    """
    # Create the link (Empty) manually instead of using bpy.ops.linkforge
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None

    link_obj = bpy.data.objects.new("link", None)
    assert link_obj is not None
    collection.objects.link(link_obj)
    link_lf: Any = getattr(link_obj, "linkforge")
    link_lf.is_robot_link = True
    assert link_lf.is_robot_link

    # Create a visual cube child
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0, 0, 0))
    visual_obj = bpy.context.active_object
    assert visual_obj is not None
    visual_obj.name = "cube_visual"
    visual_obj.parent = link_obj

    # Force LinkForge link name
    link_lf.link_name = "test_link"
    link_lf.mass = 1.0  # 1kg

    # Trigger inertia calculation (which uses our NumPy optimization)
    success = calculate_inertia_for_link(link_obj)
    assert success is True

    # Check results (1m cube, 1kg => Ixx=1/6 = 0.1666...)
    expected = 1.0 / 6.0
    assert pytest.approx(link_lf.inertia_ixx, abs=1e-5) == expected
    assert pytest.approx(link_lf.inertia_iyy, abs=1e-5) == expected
    assert pytest.approx(link_lf.inertia_izz, abs=1e-5) == expected
    # Off-diagonals should be zero
    assert pytest.approx(link_lf.inertia_ixy, abs=1e-5) == 0.0


def test_inertia_integration_with_offset() -> None:
    """Verify that offset visuals are handled correctly via the Parallel Axis Theorem."""
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None

    link_obj = bpy.data.objects.new("link_offset", None)
    assert link_obj is not None
    collection.objects.link(link_obj)
    link_lf: Any = getattr(link_obj, "linkforge")
    link_lf.is_robot_link = True

    # Visual cube offset from link origin
    # Link at (0,0,0), visual at (10, 0, 0)
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(10, 0, 0))
    visual_obj = bpy.context.active_object
    assert visual_obj is not None
    visual_obj.name = "offset_visual"
    visual_obj.parent = link_obj

    link_lf.mass = 2.0  # 2kg

    # Trigger
    success = calculate_inertia_for_link(link_obj)
    assert success is True

    # Ixx for a 2kg cube about its COM is 2 * (1/6) = 1/3 = 0.333...
    # The world-to-local transform in extract_mesh_triangles should shift the geometry
    # relative to the link frame, then our NumPy code shifts it back to COM.
    # Therefore the inertia about COM should remain 1/3 regardless of the visual offset.
    expected = 2.0 / 6.0
    assert pytest.approx(link_lf.inertia_ixx, abs=1e-5) == expected
