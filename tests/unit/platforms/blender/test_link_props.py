from unittest.mock import patch

import bpy


def test_link_name_getter_setter() -> None:
    """Test that link_name getter/setter work and sanitize names."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.name = "Original Name"
    obj.linkforge.is_robot_link = True

    # Getter should return sanitized name (preserves case, replaces spaces)
    assert obj.linkforge.link_name == "Original_Name"

    # Setter should update object name
    obj.linkforge.link_name = "New-Link-Name!"
    assert obj.name == "New-Link-Name_"
    assert obj.linkforge.link_name == "New-Link-Name_"


def test_automatic_child_renaming() -> None:
    """Test that renaming a link object also renames its visual/collision children."""
    bpy.ops.object.select_all(action="DESELECT")

    # Create parent link
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "base_link"
    link_obj.linkforge.is_robot_link = True

    # Create visual child
    bpy.ops.mesh.primitive_cube_add()
    vis_obj = bpy.context.active_object
    vis_obj.name = "base_link_visual"
    vis_obj.parent = link_obj

    # Create collision child with suffix
    bpy.ops.mesh.primitive_cube_add()
    col_obj = bpy.context.active_object
    col_obj.name = "base_link_collision_01"
    col_obj.parent = link_obj

    # Rename the link
    link_obj.linkforge.link_name = "chassis"

    # Assert children are renamed
    assert link_obj.name == "chassis"
    assert vis_obj.name == "chassis_visual"
    assert col_obj.name == "chassis_collision_01"


def test_collision_quality_update_trigger() -> None:
    """Test that changing collision quality schedules a preview update."""
    # We need to mock the update_collision_quality_realtime to avoid background timer issues
    with patch(
        "linkforge.blender.operators.link_ops.update_collision_quality_realtime"
    ) as mock_update:
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.object.empty_add()
        obj = bpy.context.active_object
        obj.linkforge.is_robot_link = True

        # Add a collision child (needed for the trigger logic inside update_collision_quality)
        bpy.ops.mesh.primitive_cube_add()
        col_obj = bpy.context.active_object
        col_obj.name = "test_collision"
        col_obj.parent = obj

        # Change quality
        obj.linkforge.collision_quality = 75.0

        # Verify mock was called
        mock_update.assert_called_once()


def test_collision_quality_skip_imported() -> None:
    """Test that quality update is skipped for imported collision meshes."""
    import linkforge.blender.operators.link_ops as link_ops

    # Stub schedule function to ensure it's NOT called
    original_schedule = link_ops.schedule_collision_preview_update
    call_count = 0

    def stub_schedule(obj):
        nonlocal call_count
        call_count += 1

    link_ops.schedule_collision_preview_update = stub_schedule

    try:
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.object.empty_add()
        obj = bpy.context.active_object
        obj.linkforge.is_robot_link = True

        # Add a collision child marked as imported
        bpy.ops.mesh.primitive_cube_add()
        col_obj = bpy.context.active_object
        col_obj.name = "imported_collision"
        col_obj.parent = obj
        col_obj["imported_from_urdf"] = True

        # Change quality
        obj.linkforge.collision_quality = 20.0

        # Should NOT have been called
        assert call_count == 0
    finally:
        link_ops.schedule_collision_preview_update = original_schedule


def test_auto_inertia_toggle() -> None:
    """Test that validating auto-inertia ensures the inertia handler is running."""
    with patch(
        "linkforge.blender.visualization.inertia_gizmos.ensure_inertia_handler"
    ) as mock_ensure:
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.object.empty_add()
        obj = bpy.context.active_object
        obj.linkforge.is_robot_link = True

        # Turn off auto-inertia
        obj.linkforge.use_auto_inertia = False

        # Verify mock was called
        mock_ensure.assert_called_once()
