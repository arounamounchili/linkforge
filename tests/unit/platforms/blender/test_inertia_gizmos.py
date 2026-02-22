"""Tests for inertia_gizmos visualization module."""

from unittest.mock import MagicMock

import bpy


def test_generate_inertia_axes_geometry():
    """Test generating inertia axes geometry for a link."""
    from linkforge.blender.visualization.inertia_gizmos import generate_inertia_axes_geometry

    # Clean scene
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create link with manual inertia
    bpy.ops.object.empty_add(location=(1, 2, 3))
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.use_auto_inertia = False
    link_obj.linkforge.inertia_origin_xyz = (0.1, 0.2, 0.3)

    # Generate geometry
    geometry = generate_inertia_axes_geometry(link_obj, axis_length=0.2)

    # Verify geometry was generated
    assert "lines" in geometry
    assert "line_colors" in geometry
    assert len(geometry["lines"]) > 0
    assert len(geometry["line_colors"]) == len(geometry["lines"])


def test_generate_inertia_axes_geometry_none_object():
    """Test geometry generation with None object."""
    from linkforge.blender.visualization.inertia_gizmos import generate_inertia_axes_geometry

    geometry = generate_inertia_axes_geometry(None)

    assert geometry["lines"] == []
    assert geometry["line_colors"] == []


def test_generate_inertia_axes_geometry_with_rotation():
    """Test geometry generation with rotated inertia frame."""
    from linkforge.blender.visualization.inertia_gizmos import generate_inertia_axes_geometry

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create link with rotated inertia
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.use_auto_inertia = False
    link_obj.linkforge.inertia_origin_xyz = (0, 0, 0)
    link_obj.linkforge.inertia_origin_rpy = (1.57, 0, 0)  # 90 degrees X rotation

    geometry = generate_inertia_axes_geometry(link_obj)

    # Should have lines for axes and spheres
    assert len(geometry["lines"]) > 0


def test_draw_inertia_gizmos(mocker):
    """Test the main draw function with GPU mocking."""
    # Mock preferences
    mock_prefs = MagicMock()
    mock_prefs.show_inertia_gizmos = True
    mock_prefs.inertia_gizmo_size = 0.15
    mocker.patch(
        "linkforge.blender.visualization.inertia_gizmos.get_addon_prefs", return_value=mock_prefs
    )

    # Mock GPU module
    mock_shader = MagicMock()
    mocker.patch("gpu.shader.from_builtin", return_value=mock_shader)
    mock_batch = mocker.patch("linkforge.blender.visualization.inertia_gizmos.batch_for_shader")
    mocker.patch("gpu.matrix.get_projection_matrix", return_value=MagicMock())
    mocker.patch("gpu.matrix.get_model_view_matrix", return_value=MagicMock())
    mocker.patch("gpu.state.line_width_set")
    mocker.patch("gpu.state.depth_test_set")
    mocker.patch("gpu.state.blend_set")

    # Create link with manual inertia
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.use_auto_inertia = False

    # Call draw function
    from linkforge.blender.visualization.inertia_gizmos import draw_inertia_gizmos

    draw_inertia_gizmos()

    # Verify GPU calls were made
    mock_batch.assert_called_once()
    mock_shader.bind.assert_called_once()


def test_draw_inertia_gizmos_disabled(mocker):
    """Test draw function when gizmos are disabled."""
    # Mock preferences with gizmos disabled
    mock_prefs = MagicMock()
    mock_prefs.show_inertia_gizmos = False
    mocker.patch(
        "linkforge.blender.visualization.inertia_gizmos.get_addon_prefs", return_value=mock_prefs
    )

    mock_batch = mocker.patch("linkforge.blender.visualization.inertia_gizmos.batch_for_shader")

    from linkforge.blender.visualization.inertia_gizmos import draw_inertia_gizmos

    draw_inertia_gizmos()

    # Should not create batch when disabled
    mock_batch.assert_not_called()


def test_draw_inertia_gizmos_auto_inertia(mocker):
    """Test that auto-inertia links are skipped."""
    mock_prefs = MagicMock()
    mock_prefs.show_inertia_gizmos = True
    mock_prefs.inertia_gizmo_size = 0.1
    mocker.patch(
        "linkforge.blender.visualization.inertia_gizmos.get_addon_prefs", return_value=mock_prefs
    )

    mock_batch = mocker.patch("linkforge.blender.visualization.inertia_gizmos.batch_for_shader")

    # Create link with AUTO inertia (should be skipped)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.use_auto_inertia = True  # AUTO - should skip

    from linkforge.blender.visualization.inertia_gizmos import draw_inertia_gizmos

    draw_inertia_gizmos()

    # Should not draw anything for auto-inertia links
    mock_batch.assert_not_called()


def test_draw_inertia_gizmos_no_objects(mocker):
    """Test draw with no objects in scene."""
    mock_prefs = MagicMock()
    mock_prefs.show_inertia_gizmos = True
    mocker.patch(
        "linkforge.blender.visualization.inertia_gizmos.get_addon_prefs", return_value=mock_prefs
    )

    mock_batch = mocker.patch("linkforge.blender.visualization.inertia_gizmos.batch_for_shader")

    # Clean scene
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    from linkforge.blender.visualization.inertia_gizmos import draw_inertia_gizmos

    draw_inertia_gizmos()

    # Should handle empty scene gracefully
    mock_batch.assert_not_called()


def test_ensure_inertia_handler(mocker):
    """Test handler registration."""
    mock_add = mocker.patch("bpy.types.SpaceView3D.draw_handler_add")
    mocker.patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw")

    import linkforge.blender.visualization.inertia_gizmos as ig_module
    from linkforge.blender.visualization.inertia_gizmos import ensure_inertia_handler

    # Reset global handle
    ig_module._draw_handle = None

    # Register handler
    ensure_inertia_handler()

    # Verify handler was registered
    mock_add.assert_called_once()


def test_ensure_inertia_handler_already_registered(mocker):
    """Test that handler registration is idempotent."""
    mock_add = mocker.patch("bpy.types.SpaceView3D.draw_handler_add")
    mocker.patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw")

    import linkforge.blender.visualization.inertia_gizmos as ig_module
    from linkforge.blender.visualization.inertia_gizmos import ensure_inertia_handler

    # Set handler to simulate already registered
    ig_module._draw_handle = MagicMock()

    ensure_inertia_handler()

    # Should not register again
    mock_add.assert_not_called()

    # Reset for other tests
    ig_module._draw_handle = None


def test_check_manual_inertia_on_load():
    """Test checking for manual inertia on file load."""
    # Create link with manual inertia
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.use_auto_inertia = False  # Manual inertia

    from linkforge.blender.visualization.inertia_gizmos import check_manual_inertia_on_load

    result = check_manual_inertia_on_load()

    # Should return None for timer compliance
    assert result is None


def test_check_manual_inertia_on_load_no_manual():
    """Test when no manual inertia links exist."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create link with auto inertia
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.use_auto_inertia = True  # Auto

    from linkforge.blender.visualization.inertia_gizmos import check_manual_inertia_on_load

    result = check_manual_inertia_on_load()

    assert result is None


def test_unregister(mocker):
    """Test unregistration cleans up handlers."""
    mock_remove = mocker.patch("bpy.types.SpaceView3D.draw_handler_remove")
    mocker.patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw")

    import linkforge.blender.visualization.inertia_gizmos as ig_module
    from linkforge.blender.visualization.inertia_gizmos import unregister

    # Set up a mock handler
    mock_handle = MagicMock()
    ig_module._draw_handle = mock_handle

    # Unregister
    unregister()

    # Verify handler was removed
    mock_remove.assert_called_once_with(mock_handle, "WINDOW")
    assert ig_module._draw_handle is None
