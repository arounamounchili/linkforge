"""Hardened unit tests for LinkForge Blender operators and link operations."""

from __future__ import annotations

import bpy
import pytest
from linkforge.blender.operators.link_ops import (
    calculate_inertia_for_link,
    regenerate_collision_mesh,
)

from tests.blender_test_utils import (
    cleanup_blender_scene,
    create_mesh_object,
    create_robot_link,
    safe_get_linkforge,
    safe_update,
)


class TestLinkOperators:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_add_empty_link_operator(self, scene, blender_context) -> None:
        """Verify that the add_empty_link operator creates a valid link frame."""
        # Run operator
        import typing

        ops: typing.Any = bpy.ops
        ops.linkforge.add_empty_link()

        # Verify object creation
        assert "base_link" in bpy.data.objects
        link_obj = bpy.data.objects["base_link"]
        assert link_obj.type == "EMPTY"

        # Verify LinkForge properties
        lf = safe_get_linkforge(link_obj)
        assert lf.is_robot_link is True
        assert lf.link_name == "base_link"

    def test_create_link_from_mesh_operator(self, scene, blender_context) -> None:
        """Verify that create_link_from_mesh correctly restructures a mesh object."""
        # Create a mesh object
        mesh_obj = create_mesh_object("arm_segment", scene=scene, with_cube=True)
        mesh_obj.location = (1, 2, 3)
        safe_update(scene)

        # Select it (operator uses active_object)
        view_layer = bpy.context.view_layer
        assert view_layer is not None
        view_layer.objects.active = mesh_obj
        mesh_obj.select_set(True)

        # Run operator
        import typing

        ops: typing.Any = bpy.ops
        ops.linkforge.create_link_from_mesh()

        # Verify restructuring
        # The Empty should now have the original name "arm_segment"
        assert "arm_segment" in bpy.data.objects
        empty_obj = bpy.data.objects["arm_segment"]
        assert empty_obj.type == "EMPTY"

        # The mesh should be renamed to "arm_segment_visual" and parented
        assert "arm_segment_visual" in bpy.data.objects
        visual_obj = bpy.data.objects["arm_segment_visual"]
        assert visual_obj.parent == empty_obj

        # Verify transforms
        assert (empty_obj.location - (1, 2, 3)).length < 1e-5
        # Parent inverse should be identity for strict alignment
        assert visual_obj.matrix_parent_inverse.is_identity
        assert visual_obj.location.length < 1e-5


class TestInertiaCalculation:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_calculate_inertia_primitive_box(self, scene, blender_context) -> None:
        """Verify inertia calculation for a box primitive."""
        link_obj = create_robot_link("box_link", scene, with_collision=False)
        visual_obj = link_obj.children[0]

        # Scale the visual to 2x1x0.5
        # Cube is 2x2x2 by default. Scale (1, 0.5, 0.25) -> dimensions (2, 1, 0.5)
        visual_obj.scale = (1.0, 0.5, 0.25)
        safe_update(scene)

        lf = safe_get_linkforge(link_obj)
        lf.mass = 12.0

        # Run calculation
        success = calculate_inertia_for_link(link_obj)
        assert success is True

        # For a box 2x1x0.5 with mass 12:
        # Ixx = 1/12 * mass * (y^2 + z^2) = 1/12 * 12 * (1^2 + 0.5^2) = 1.25
        # Iyy = 1/12 * mass * (x^2 + z^2) = 1/12 * 12 * (2^2 + 0.5^2) = 4.25
        # Izz = 1/12 * mass * (x^2 + y^2) = 1/12 * 12 * (2^2 + 1^2) = 5.0

        assert abs(lf.inertia_ixx - 1.25) < 1e-3
        assert abs(lf.inertia_iyy - 4.25) < 1e-3
        assert abs(lf.inertia_izz - 5.0) < 1e-3

    def test_calculate_inertia_mesh_fallback(self, scene, blender_context) -> None:
        """Verify inertia calculation fallback for non-primitive meshes."""
        # Use with_cube=True to ensure we have vertices
        link_obj = create_robot_link("mesh_link", scene, with_visual=True)
        # Re-create visual with cube to be sure
        visual_obj = link_obj.children[0]
        mesh = bpy.data.meshes.new("mesh_with_cube")
        import bmesh

        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=2.0)
        bm.to_mesh(mesh)
        bm.free()
        visual_obj.data = mesh
        safe_update(scene)

        bm = bmesh.new()
        bm.from_mesh(visual_obj.data)
        assert len(bm.verts) > 0
        bm.verts[0].co.x += 0.5
        bm.to_mesh(visual_obj.data)
        bm.free()
        safe_update(scene)

        lf = safe_get_linkforge(link_obj)
        lf.mass = 1.0

        # Run calculation
        success = calculate_inertia_for_link(link_obj)
        assert success is True
        assert lf.inertia_ixx > 0
        assert lf.inertia_iyy > 0
        assert lf.inertia_izz > 0


class TestCompoundOperations:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_regenerate_collision_compound(self, scene, blender_context) -> None:
        """Verify that regenerate_collision_mesh creates a compound mesh for multiple visuals."""
        link_obj = create_robot_link(
            "multi_visual_link", scene, with_visual=False, with_collision=False
        )

        # Add two visual meshes
        v1 = create_mesh_object("v1_visual", scene=scene, with_cube=True)
        v1.parent = link_obj
        v1.location = (1, 0, 0)

        v2 = create_mesh_object("v2_visual", scene=scene, with_cube=True)
        v2.parent = link_obj
        v2.location = (-1, 0, 0)

        safe_update(scene)

        # Run regeneration
        regenerate_collision_mesh(link_obj, "MESH", bpy.context)

        # Verify collision object
        collision_objs = [c for c in link_obj.children if "_collision" in c.name]
        assert len(collision_objs) == 1
        col_obj = collision_objs[0]

        # Compound collision should span both visuals
        # v1 is 2x2x2 at (1,0,0) -> spans x=[0, 2]
        # v2 is 2x2x2 at (-1,0,0) -> spans x=[-2, 0]
        # Total span x=[-2, 2], width=4
        assert abs(col_obj.dimensions.x - 4.0) < 0.1
        assert abs(col_obj.dimensions.y - 2.0) < 0.1
        assert abs(col_obj.dimensions.z - 2.0) < 0.1
