from unittest.mock import patch

import bpy
import pytest
from linkforge.blender.operators.link_ops import (
    create_collision_for_link,
    execute_collision_preview_update,
    regenerate_collision_mesh,
)


def test_execute_collision_preview_update_branches(clean_scene):
    """Hit lines 62-81 in link_ops.py."""
    link_obj = bpy.data.objects.new("Link", None)
    bpy.context.collection.objects.link(link_obj)
    link_obj.linkforge.is_robot_link = True

    col_mesh = bpy.data.meshes.new("col")
    col_obj = bpy.data.objects.new("Link_collision", col_mesh)
    bpy.context.collection.objects.link(col_obj)
    col_obj.parent = link_obj

    # Line 62: No view_layer
    with patch("linkforge.blender.operators.link_ops.bpy") as mock_bpy:
        mock_bpy.data.objects = {"Link": link_obj}
        mock_bpy.context.view_layer = None
        # We need to set the global _preview_pending_object
        import linkforge.blender.operators.link_ops as link_ops

        link_ops._preview_pending_object = link_obj
        assert execute_collision_preview_update() is None

    # Line 73: imported_from_urdf
    col_obj["imported_from_urdf"] = True
    link_ops._preview_pending_object = link_obj
    assert execute_collision_preview_update() is None
    col_obj["imported_from_urdf"] = False


def test_regenerate_collision_mesh_validation(clean_scene):
    """Hit lines 528-533 (validation in regenerate)."""
    # Passing None or non-link object
    regenerate_collision_mesh(None, "AUTO", bpy.context)

    o = bpy.data.objects.new("NotLink", None)
    regenerate_collision_mesh(o, "AUTO", bpy.context)


def test_create_collision_failure_branches(clean_scene):
    """Hit lines 553-555 (collision creation failure)."""
    link_obj = bpy.data.objects.new("Link", None)
    bpy.context.collection.objects.link(link_obj)
    link_obj.linkforge.is_robot_link = True

    # Force _create_primitive_collision to fail (return None)
    with patch(
        "linkforge.blender.operators.link_ops._create_primitive_collision",
        return_value=(None, (0, 0, 0)),
    ):
        assert create_collision_for_link(link_obj, "BOX", bpy.context) is None


def test_generate_collision_all_skip(clean_scene):
    """Hit lines 591-592 (skipping links with no visuals)."""
    link_obj = bpy.data.objects.new("EmptyLink", None)
    bpy.context.collection.objects.link(link_obj)
    link_obj.linkforge.is_robot_link = True

    # link_obj has no children, so generate_collision_all should skip it
    bpy.ops.linkforge.generate_collision_all()
    assert not any("_collision" in obj.name for obj in bpy.data.objects)


def test_add_material_slot_skip(clean_scene):
    """Hit lines 1140-1141 (skipping if no visual)."""
    link_obj = bpy.data.objects.new("MatLink", None)
    bpy.context.collection.objects.link(link_obj)
    link_obj.linkforge.is_robot_link = True

    bpy.context.view_layer.objects.active = link_obj
    # add_material_slot should do nothing/return if no visual found
    with pytest.raises(RuntimeError, match="No visual mesh found"):
        bpy.ops.linkforge.add_material_slot()
