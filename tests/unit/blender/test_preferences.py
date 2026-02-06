from unittest.mock import MagicMock

import bpy
import linkforge.blender.preferences as prefs_mod
import pytest


def test_get_addon_id_traditional(mocker):
    """Test addon ID detection for traditional addons."""
    mocker.patch.object(prefs_mod, "__package__", "linkforge.blender")
    assert prefs_mod.get_addon_id() == "linkforge"


def test_get_addon_id_extension(mocker):
    """Test addon ID detection for Blender Extensions."""
    mocker.patch.object(prefs_mod, "__package__", "bl_ext.user_default.linkforge")
    assert prefs_mod.get_addon_id() == "bl_ext.user_default.linkforge"


def test_get_addon_id_basic(mocker):
    """Test addon ID detection for simple package names."""
    mocker.patch.object(prefs_mod, "__package__", "linkforge")
    assert prefs_mod.get_addon_id() == "linkforge"


def test_get_addon_id_empty(mocker):
    """Test fallback when package is missing."""
    mocker.patch.object(prefs_mod, "__package__", None)
    assert prefs_mod.get_addon_id() == "linkforge"


def test_update_sizing_callbacks():
    """Test that sizing update functions correctly iterate and update objects."""
    bpy.ops.object.select_all(action="DESELECT")

    # Create test objects
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True

    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.linkforge_joint.is_robot_joint = True

    bpy.ops.object.empty_add()
    sensor_obj = bpy.context.active_object
    sensor_obj.linkforge_sensor.is_robot_sensor = True

    # Link update
    fake_prefs = MagicMock()
    fake_prefs.link_empty_size = 0.55
    prefs_mod.update_link_empty_size(fake_prefs, bpy.context)
    assert link_obj.empty_display_size == pytest.approx(0.55)

    # Joint update
    fake_prefs.joint_empty_size = 0.77
    prefs_mod.update_joint_empty_size(fake_prefs, bpy.context)
    assert joint_obj.empty_display_size == pytest.approx(0.77)

    # Sensor update
    fake_prefs.sensor_empty_size = 0.33
    prefs_mod.update_sensor_empty_size(fake_prefs, bpy.context)
    assert sensor_obj.empty_display_size == pytest.approx(0.33)


def test_inertia_viz_toggle_callback(mocker):
    """Test that inertia visualization toggle callback triggers the right utilities."""
    mock_ensure = mocker.patch("linkforge.blender.utils.inertia_gizmos.ensure_inertia_handler")
    mock_tag = mocker.patch("linkforge.blender.utils.inertia_gizmos.tag_redraw")

    fake_prefs = MagicMock()
    fake_prefs.show_inertia_gizmos = True
    prefs_mod.update_inertia_visibility(fake_prefs, bpy.context)

    mock_ensure.assert_called_once()
    mock_tag.assert_called_once()
