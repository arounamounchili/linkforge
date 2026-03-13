import bpy
from linkforge.blender.utils.property_helpers import find_property_owner
from linkforge.blender.utils.transform_utils import clear_parent_keep_transform


def test_find_property_owner_strategies() -> None:
    """Test find_property_owner with various strategies."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.name = "owner_obj"
    obj.linkforge.is_robot_link = True

    props = obj.linkforge

    # Strategy 1: id_data (should work immediately)
    found = find_property_owner(bpy.context, props, "linkforge")
    assert found == obj

    # Strategy 2: Active object
    # Force strategy 2 by mocking/circumventing Strategy 1 if possible
    # Actually Strategy 1 is built-in to Blender PropertyGroups
    assert props.id_data == obj

    # Check with non-existent property
    assert find_property_owner(bpy.context, props, "invalid_attr") is None


def test_clear_parent_keep_transform() -> None:
    """Test unparenting while preserving world transform."""
    bpy.ops.object.select_all(action="DESELECT")

    # Create parent with transform
    bpy.ops.object.empty_add()
    parent = bpy.context.active_object
    parent.location = (1, 2, 3)
    parent.rotation_euler = (0.5, 0, 0)

    # Create child
    bpy.ops.object.empty_add()
    child = bpy.context.active_object
    child.parent = parent
    child.location = (0, 1, 0)

    bpy.context.view_layer.update()
    world_matrix_before = child.matrix_world.copy()

    # Clear parent
    clear_parent_keep_transform(child)

    # Assert world matrix is preserved (within tolerance)
    bpy.context.view_layer.update()
    world_matrix_after = child.matrix_world

    for i in range(4):
        for j in range(4):
            assert abs(world_matrix_before[i][j] - world_matrix_after[i][j]) < 1e-6

    assert child.parent is None
