from unittest.mock import MagicMock

import bpy
import pytest
from linkforge.blender.visualization.joint_gizmos import (
    fix_existing_joints,
    generate_arrow_cone_vertices,
    generate_axis_geometry,
    update_viz_handle,
)
from mathutils import Vector


def test_generate_arrow_cone_vertices():
    """Test the math behind arrow cone vertex generation."""
    origin = Vector((0, 0, 0))
    direction = Vector((0, 0, 1))  # Z-axis
    length = 1.0

    positions, indices = generate_arrow_cone_vertices(origin, direction, length)

    # Tip should be at (0, 0, 1)
    assert positions[0] == pytest.approx((0, 0, 1))

    # Base center should be at (0, 0, 0.8) if default ratio is 0.2
    assert positions[-1] == pytest.approx((0, 0, 0.8))

    # Should have tip + 8 circle segments + base center = 10 vertices
    assert len(positions) == 10

    # Circle vertices should have constant Z and radius
    # cone_radius = cone_length (0.2) * 0.3 = 0.06
    for i in range(1, 9):
        assert positions[i][2] == pytest.approx(0.8)
        xy_dist = (positions[i][0] ** 2 + positions[i][1] ** 2) ** 0.5
        assert xy_dist == pytest.approx(0.06)


def test_generate_axis_geometry():
    """Test the orchestration of RGB axis geometry generation."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.linkforge_joint.is_robot_joint = True

    # Move and rotate object to ensure world space transformation is tested
    obj.location = (1.0, 2.0, 3.0)
    obj.rotation_euler = (0, 0, 0)  # No rotation for easy check

    # CRITICAL: Update view layer so matrix_world reflects the new location
    bpy.context.view_layer.update()

    data = generate_axis_geometry(obj, axis_length=0.2)

    assert "lines" in data
    assert "tris" in data
    assert len(data["lines"]) == 6  # 3 axes * 2 points per line
    assert len(data["line_colors"]) == 6

    # First line should be X axis (Red)
    # Origin is (1, 2, 3). End is origin + (X * length * 0.8)
    # Origin
    assert data["lines"][0] == pytest.approx((1.0, 2.0, 3.0))
    # Shaft End
    assert data["lines"][1] == pytest.approx((1.16, 2.0, 3.0))  # 1 + 0.2 * 0.8
    assert data["line_colors"][0] == (1.0, 0.0, 0.0, 1.0)


def test_fix_existing_joints():
    """Test the iteration logic that forces PLAIN_AXES on joints."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.linkforge_joint.is_robot_joint = True
    obj.empty_display_type = "CUBE"

    fix_existing_joints()

    assert obj.empty_display_type == "PLAIN_AXES"


def test_update_viz_handle_switching(mocker):
    """Test registering and unregistering the draw handler based on prefs."""
    # Mock bpy.types.SpaceView3D to avoid real handler registration
    mock_add = mocker.patch("bpy.types.SpaceView3D.draw_handler_add", return_value="handle_123")
    mock_remove = mocker.patch("bpy.types.SpaceView3D.draw_handler_remove")

    # Mock preferences
    mock_prefs = mocker.patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs")

    # 1. Test ENABLE
    mock_prefs.return_value = type("Prefs", (), {"show_joint_axes": True})()
    update_viz_handle(bpy.context)
    mock_add.assert_called_once()
    assert bpy.app.driver_namespace["linkforge_joint_gizmo_handler"] == "handle_123"

    # 2. Test DISABLE
    mock_prefs.return_value.show_joint_axes = False
    update_viz_handle(bpy.context)
    mock_remove.assert_called()
    assert "linkforge_joint_gizmo_handler" not in bpy.app.driver_namespace


def test_draw_internal(mocker):
    """Test the drawing logic by mocking the GPU module."""
    # Create a joint object in the scene BEFORE mocking context
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.linkforge_joint.is_robot_joint = True

    # Mock bpy.context locally for the joint_gizmos module
    mock_context = MagicMock()
    mock_context.scene = bpy.context.scene
    mock_context.window_manager = bpy.context.window_manager
    mock_context.region_data = MagicMock()
    mocker.patch("linkforge.blender.visualization.joint_gizmos.bpy.context", mock_context)

    # Mock GPU module
    mock_shader_obj = MagicMock()
    mocker.patch("gpu.shader.from_builtin", return_value=mock_shader_obj)
    mocker.patch("gpu.state.depth_test_set")
    mocker.patch("gpu.state.blend_set")
    mocker.patch("gpu.state.line_width_set")
    mocker.patch("gpu.matrix.get_projection_matrix", return_value=MagicMock())
    mocker.patch("gpu.matrix.get_model_view_matrix", return_value=MagicMock())
    mock_batch = mocker.patch("linkforge.blender.visualization.joint_gizmos.batch_for_shader")

    from linkforge.blender.visualization.joint_gizmos import _draw_internal

    _draw_internal()

    # Verify GPU calls were made
    assert mock_batch.called
    assert mock_shader_obj.bind.called
    assert mock_shader_obj.uniform_float.called


def test_fix_current_scene(mocker):
    """Test the timer callback for fixing joints."""
    mock_fix = mocker.patch("linkforge.blender.visualization.joint_gizmos.fix_existing_joints")
    mock_update = mocker.patch("linkforge.blender.visualization.joint_gizmos.update_viz_handle")

    from linkforge.blender.visualization.joint_gizmos import fix_current_scene

    result = fix_current_scene()

    assert result is None  # Timer should not repeat
    mock_fix.assert_called_once()
    mock_update.assert_called_once()


def test_shader_fallback(mocker):
    """Test get_shader fallback for older Blender versions."""
    from linkforge.blender.visualization import joint_gizmos

    joint_gizmos._builtin_shader_name = None  # Reset

    mock_from_builtin = mocker.patch("gpu.shader.from_builtin")
    # First call fails, second call succeeds with fallback
    mock_from_builtin.side_effect = [Exception("Shader not found"), "fallback_shader"]

    shader = joint_gizmos.get_shader()
    assert shader == "fallback_shader"
    assert joint_gizmos._builtin_shader_name == "3D_FLAT_COLOR"


def test_generate_axis_geometry_invalid():
    """Test generating geometry for invalid objects."""
    # None object
    data = generate_axis_geometry(None)
    assert len(data["lines"]) == 0

    # Non-empty object (e.g., Mesh)
    bpy.ops.mesh.primitive_cube_add()
    cube = bpy.context.active_object
    data = generate_axis_geometry(cube)
    assert len(data["lines"]) == 0


def test_draw_internal_no_objects(mocker):
    """Test that drawing returns early if no joints are present."""
    mocker.patch(
        "linkforge.blender.visualization.joint_gizmos.get_addon_prefs",
        return_value=type("Prefs", (), {"show_joint_axes": True})(),
    )

    # Mock context locally
    mock_context = MagicMock()
    mock_context.scene = bpy.context.scene
    mock_context.region_data = MagicMock()
    mocker.patch("linkforge.blender.visualization.joint_gizmos.bpy.context", mock_context)

    mock_gpu_state = mocker.patch("gpu.state.depth_test_set")

    # Delete all objects in real scene
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    from linkforge.blender.visualization.joint_gizmos import _draw_internal

    _draw_internal()

    # Should have returned BEFORE setting GPU state
    assert not mock_gpu_state.called


def test_draw_internal_disabled(mocker):
    """Test that drawing returns early if disabled in prefs."""
    mocker.patch(
        "linkforge.blender.visualization.joint_gizmos.get_addon_prefs",
        return_value=type("Prefs", (), {"show_joint_axes": False})(),
    )

    mock_gpu_state = mocker.patch("gpu.state.depth_test_set")

    from linkforge.blender.visualization.joint_gizmos import _draw_internal

    _draw_internal()

    assert not mock_gpu_state.called


def test_unregister(mocker):
    """Test the unregistration cleanup."""
    mock_handler = MagicMock()
    bpy.app.driver_namespace["linkforge_joint_gizmo_handler"] = mock_handler
    mock_remove = mocker.patch("bpy.types.SpaceView3D.draw_handler_remove")

    from linkforge.blender.visualization import joint_gizmos

    joint_gizmos.unregister()

    mock_remove.assert_called_once_with(mock_handler, "WINDOW")
    assert "linkforge_joint_gizmo_handler" not in bpy.app.driver_namespace


def test_draw_joint_axes(mocker):
    """Test the main entry point for joint axis drawing."""
    mock_draw_internal = mocker.patch("linkforge.blender.visualization.joint_gizmos._draw_internal")
    from linkforge.blender.visualization.joint_gizmos import draw_joint_axes

    draw_joint_axes()
    mock_draw_internal.assert_called_once()


def test_draw_internal_no_region_data(mocker):
    """Test early return if region_data is missing."""
    mocker.patch(
        "linkforge.blender.visualization.joint_gizmos.get_addon_prefs",
        return_value=type("Prefs", (), {"show_joint_axes": True})(),
    )

    # Mock context with NO region_data
    mock_context = MagicMock(spec=bpy.types.Context)
    mock_context.scene = bpy.context.scene
    mocker.patch("linkforge.blender.visualization.joint_gizmos.bpy.context", mock_context)

    mock_gpu_state = mocker.patch("gpu.state.depth_test_set")

    from linkforge.blender.visualization.joint_gizmos import _draw_internal

    _draw_internal()

    assert not mock_gpu_state.called


def test_fix_existing_joints_exception(mocker):
    """Test exception handling in fix_existing_joints."""
    mock_context = MagicMock()
    type(mock_context).scene = mocker.PropertyMock(side_effect=AttributeError("Mock Error"))
    mocker.patch("linkforge.blender.visualization.joint_gizmos.bpy.context", mock_context)

    from linkforge.blender.visualization.joint_gizmos import fix_existing_joints

    # Should not raise exception
    fix_existing_joints()
