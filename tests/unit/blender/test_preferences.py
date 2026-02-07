from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.preferences import (
    get_addon_id,
    get_addon_prefs,
    update_inertia_size,
    update_inertia_visibility,
    update_joint_axes_visibility,
    update_joint_empty_size,
    update_link_empty_size,
    update_sensor_empty_size,
)


def test_get_addon_id():
    """Test that addon ID is correctly derived from package name."""
    # We can't easily mock __package__ globally, but we can verify current state
    # In tests, it usually defaults to 'linkforge' or similar
    addon_id = get_addon_id()
    assert isinstance(addon_id, str)
    assert len(addon_id) > 0


def test_update_joint_empty_size():
    """Test that updating joint size in prefs affects scene objects."""
    # Create a joint
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.linkforge_joint.is_robot_joint = True
    obj.empty_display_size = 0.1

    # Mock self (preferences)
    mock_prefs = MagicMock()
    mock_prefs.joint_empty_size = 0.5

    # Call update
    with patch("linkforge.blender.visualization.joint_gizmos.update_viz_handle") as mock_viz:
        update_joint_empty_size(mock_prefs, bpy.context)
        mock_viz.assert_called_once_with(bpy.context)

    # Verify object was updated
    assert obj.empty_display_size == pytest.approx(0.5)


def test_update_sensor_empty_size():
    """Test that updating sensor size in prefs affects scene objects."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.linkforge_sensor.is_robot_sensor = True
    obj.empty_display_size = 0.1

    mock_prefs = MagicMock()
    mock_prefs.sensor_empty_size = 0.3

    update_sensor_empty_size(mock_prefs, bpy.context)

    assert obj.empty_display_size == pytest.approx(0.3)


def test_update_link_empty_size():
    """Test that updating link size in prefs affects scene objects."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.linkforge.is_robot_link = True
    obj.empty_display_size = 0.1

    mock_prefs = MagicMock()
    mock_prefs.link_empty_size = 0.4

    update_link_empty_size(mock_prefs, bpy.context)

    assert obj.empty_display_size == pytest.approx(0.4)


def test_get_addon_prefs_missing():
    """Test that get_addon_prefs returns None if addon not found."""
    mock_context = MagicMock()
    mock_context.preferences.addons.get.return_value = None

    with patch("linkforge.blender.preferences.bpy.context", mock_context):
        prefs = get_addon_prefs()
        assert prefs is None


def test_update_joint_axes_visibility():
    """Test that toggling joint axes visibility calls update_viz_handle."""
    with patch("linkforge.blender.visualization.joint_gizmos.update_viz_handle") as mock_viz:
        update_joint_axes_visibility(MagicMock(), bpy.context)
        mock_viz.assert_called_once_with(bpy.context)


def test_update_inertia_visibility():
    """Test that toggling inertia visibility calls tag_redraw and ensure_handler."""
    mock_prefs = MagicMock()
    mock_prefs.show_inertia_gizmos = True

    with (
        patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw") as mock_redraw,
        patch(
            "linkforge.blender.visualization.inertia_gizmos.ensure_inertia_handler"
        ) as mock_ensure,
    ):
        update_inertia_visibility(mock_prefs, bpy.context)
        mock_redraw.assert_called_once()
        mock_ensure.assert_called_once()


def test_update_inertia_size():
    """Test that changing inertia size calls tag_redraw."""
    with patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw") as mock_redraw:
        update_inertia_size(MagicMock(), bpy.context)
        mock_redraw.assert_called_once()
