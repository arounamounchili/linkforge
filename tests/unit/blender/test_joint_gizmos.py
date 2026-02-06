import bpy
import pytest
from linkforge.blender.utils.joint_gizmos import (
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
    mock_prefs = mocker.patch("linkforge.blender.utils.joint_gizmos.get_addon_prefs")

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
