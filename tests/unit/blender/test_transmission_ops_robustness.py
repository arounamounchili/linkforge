from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.operators.transmission_ops import (
    LINKFORGE_OT_create_transmission,
    LINKFORGE_OT_delete_transmission,
)


def test_create_transmission_axes(clean_scene):
    """Test transmission creation with all joint axis types."""
    # Setup Joint
    j = bpy.data.objects.new("Joint", None)
    bpy.context.collection.objects.link(j)
    j.linkforge_joint.is_robot_joint = True

    bpy.context.view_layer.objects.active = j
    j.select_set(True)

    axes = [
        ("X", (1, 0, 0)),
        ("Y", (0, 1, 0)),
        ("Z", (0, 0, 1)),
        ("CUSTOM", (1, 1, 0)),
    ]

    for axis_type, _ in axes:
        # ALWAYS RE-SELECT JOINT (the created transmission would become active otherwise)
        bpy.ops.object.select_all(action="DESELECT")
        j.select_set(True)
        bpy.context.view_layer.objects.active = j

        j.linkforge_joint.axis = axis_type
        if axis_type == "CUSTOM":
            j.linkforge_joint.custom_axis_x = 1.0
            j.linkforge_joint.custom_axis_y = 1.0
            j.linkforge_joint.custom_axis_z = 0.0

        bpy.ops.linkforge.create_transmission()
        trans = bpy.context.active_object
        assert trans.name == f"{j.name}_trans"
        assert trans.parent == j
        # Cleanup for next iteration
        bpy.data.objects.remove(trans, do_unlink=True)


def test_create_transmission_preferences(clean_scene):
    """Verify transmission size from preferences."""
    with patch("linkforge.blender.preferences.get_addon_prefs") as mock_prefs:
        prefs = MagicMock()
        prefs.transmission_empty_size = 0.123
        mock_prefs.return_value = prefs

        j = bpy.data.objects.new("Joint", None)
        bpy.context.collection.objects.link(j)
        j.linkforge_joint.is_robot_joint = True
        bpy.context.view_layer.objects.active = j
        j.select_set(True)

        bpy.ops.linkforge.create_transmission()
        trans = bpy.data.objects.get(f"{j.name}_trans")
        assert trans.empty_display_size == pytest.approx(0.123)


def test_create_transmission_collection_sync(clean_scene):
    """Verify transmission is in joint's collection."""
    custom_coll = bpy.data.collections.new("MechColl")
    bpy.context.scene.collection.children.link(custom_coll)

    j = bpy.data.objects.new("Joint", None)
    custom_coll.objects.link(j)
    j.linkforge_joint.is_robot_joint = True

    bpy.context.view_layer.objects.active = j
    j.select_set(True)

    bpy.ops.linkforge.create_transmission()
    trans = bpy.data.objects.get(f"{j.name}_trans")
    assert trans in custom_coll.objects.values()


def test_delete_transmission(clean_scene):
    """Test deletion of transmission."""
    j = bpy.data.objects.new("Joint", None)
    bpy.context.collection.objects.link(j)
    j.linkforge_joint.is_robot_joint = True
    bpy.context.view_layer.objects.active = j
    j.select_set(True)
    bpy.ops.linkforge.create_transmission()
    trans = bpy.context.active_object

    assert trans.linkforge_transmission.is_robot_transmission is True

    # Poll failure: not selected or no active object
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.view_layer.objects.active = None
    assert LINKFORGE_OT_delete_transmission.poll(bpy.context) is False

    # Re-select for deletion
    trans.select_set(True)
    bpy.context.view_layer.objects.active = trans
    bpy.ops.linkforge.delete_transmission()


def test_transmission_logic_gaps(clean_scene):
    """Hit remaining logic gaps in transmission_ops."""
    # 1. Non-EMPTY object poll failure
    bpy.ops.mesh.primitive_cube_add()
    cube = bpy.context.active_object
    assert LINKFORGE_OT_create_transmission.poll(bpy.context) is False

    # 2. Selected but not joint poll failure
    cube.select_set(True)
    assert LINKFORGE_OT_create_transmission.poll(bpy.context) is False


def test_create_transmission_no_axis_fallback(clean_scene):
    """Hit transmission fallback when no axis vec is detectable."""
    j = bpy.data.objects.new("Joint", None)
    bpy.context.collection.objects.link(j)
    j.linkforge_joint.is_robot_joint = True

    # Custom axis without setting actual values (0,0,0) - length will be 0
    j.linkforge_joint.axis = "CUSTOM"
    j.linkforge_joint.custom_axis_x = 0.0
    j.linkforge_joint.custom_axis_y = 0.0
    j.linkforge_joint.custom_axis_z = 0.0

    bpy.context.view_layer.objects.active = j
    j.select_set(True)

    bpy.ops.linkforge.create_transmission()
    trans = bpy.context.active_object
    # Should have identity rotation (0,0,0) as fallback
    assert trans.rotation_euler.x == 0
    assert trans.rotation_euler.y == 0
    assert trans.rotation_euler.z == 0
