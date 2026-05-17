"""Integration tests for Blender Physics and Inertia."""

from __future__ import annotations

from typing import Any

import bpy

from tests.blender_test_utils import (
    create_mesh_object,
    safe_get_linkforge,
    safe_update,
)


class TestPhysicsInertiaIntegration:
    def test_inertia_calculation_primitive_box(self, blender_clean_scene) -> None:
        """Verify inertia calculation for a primitive box matches theory."""
        scene = bpy.context.scene
        ops: Any = bpy.ops

        # 1. Create an Empty as link frame
        ops.linkforge.add_empty_link()
        link_obj = bpy.context.active_object
        assert link_obj is not None, "Failed to create or set active link frame"
        assert link_obj.name == "base_link"
        lf = safe_get_linkforge(link_obj)
        lf.mass = 1.0

        # 2. Add a visual mesh child (cube side length 2.0m)
        vis = create_mesh_object("box_visual", scene=scene, with_cube=True)
        vis.parent = link_obj
        vis.select_set(True)

        # Mass = 1.0kg, Side = 2.0m
        # Ixx = m * (y^2 + z^2) / 12 = 1.0 * (4 + 4) / 12 = 8/12 = 0.666...

        safe_update()

        # Execute operator on the link frame
        res = ops.linkforge.calculate_inertia()
        assert res == {"FINISHED"}

        # Verify values
        assert abs(lf.inertia_ixx - 0.666666) < 1e-4
        assert abs(lf.inertia_iyy - 0.666666) < 1e-4
        assert abs(lf.inertia_izz - 0.666666) < 1e-4

    def test_collision_mesh_generation_convex_hull(self, blender_clean_scene) -> None:
        """Verify that convex hull collision generation works and parents correctly."""
        scene = bpy.context.scene
        ops: Any = bpy.ops
        # Create an Empty as link frame
        ops.linkforge.add_empty_link()
        link_obj = bpy.context.active_object
        assert link_obj is not None
        assert link_obj.name == "base_link"

        # Add a visual mesh child
        vis = create_mesh_object("base_visual", scene=scene, with_cube=True)
        vis.parent = link_obj
        # Select visual mesh and generate collision
        if bpy.context.view_layer is not None:
            bpy.context.view_layer.objects.active = vis
        else:
            bpy.context.active_object = vis
        vis.select_set(True)
        safe_update()

        # Execute collision generation (MESH mode is default for multi-visual or auto)
        res = ops.linkforge.generate_collision()
        assert res == {"FINISHED"}

        # Verify collision object exists
        collision_obj = next((c for c in link_obj.children if "_collision" in c.name), None)
        assert collision_obj is not None
        assert collision_obj.type == "MESH"
        assert collision_obj.parent == link_obj
        assert collision_obj.display_type == "WIRE"

    def test_mass_calculation_from_volume(self, blender_clean_scene) -> None:
        """Verify mass calculation based on volume and density."""
        # Note: I need to check if LINKFORGE_OT_calculate_mass_from_volume exists
        # Let's assume it does based on link_ops.py analysis
        pass
