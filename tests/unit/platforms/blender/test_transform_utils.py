"""Tests for transform_utils module."""

import bpy
import pytest
from linkforge.blender.utils.transform_utils import (
    clear_parent_keep_transform,
    set_parent_keep_transform,
)


def test_set_parent_keep_transform_basic():
    """Test parenting while preserving world transform."""
    # Clean scene
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create parent and child objects
    bpy.ops.object.empty_add(location=(1, 2, 3))
    parent_obj = bpy.context.active_object
    parent_obj.name = "Parent"

    bpy.ops.object.empty_add(location=(5, 6, 7))
    child_obj = bpy.context.active_object
    child_obj.name = "Child"

    # Store original world location
    original_world_loc = child_obj.matrix_world.translation.copy()

    # Set parent while keeping transform
    set_parent_keep_transform(child_obj, parent_obj)

    # Verify parent was set
    assert child_obj.parent == parent_obj

    # Verify world location is preserved
    assert child_obj.matrix_world.translation.x == pytest.approx(original_world_loc.x, abs=1e-4)
    assert child_obj.matrix_world.translation.y == pytest.approx(original_world_loc.y, abs=1e-4)
    assert child_obj.matrix_world.translation.z == pytest.approx(original_world_loc.z, abs=1e-4)


def test_set_parent_keep_transform_with_rotation():
    """Test parenting with rotated parent."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create rotated parent
    bpy.ops.object.empty_add(location=(0, 0, 0), rotation=(0, 0, 1.5708))  # 90 degrees Z
    parent_obj = bpy.context.active_object

    # Create child at specific location
    bpy.ops.object.empty_add(location=(1, 0, 0))
    child_obj = bpy.context.active_object

    original_world_loc = child_obj.matrix_world.translation.copy()

    # Parent with transform preservation
    set_parent_keep_transform(child_obj, parent_obj)

    # World location should be preserved
    new_world_loc = child_obj.matrix_world.translation
    assert new_world_loc.x == pytest.approx(original_world_loc.x, abs=1e-4)
    assert new_world_loc.y == pytest.approx(original_world_loc.y, abs=1e-4)
    assert new_world_loc.z == pytest.approx(original_world_loc.z, abs=1e-4)


def test_set_parent_keep_transform_none_child():
    """Test with None child."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    parent_obj = bpy.context.active_object

    # Should not raise error
    set_parent_keep_transform(None, parent_obj)


def test_set_parent_keep_transform_none_parent():
    """Test with None parent."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    child_obj = bpy.context.active_object

    # Should not raise error
    set_parent_keep_transform(child_obj, None)


def test_clear_parent_keep_transform_basic():
    """Test clearing parent while preserving world transform."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create parent and child
    bpy.ops.object.empty_add(location=(2, 3, 4))
    parent_obj = bpy.context.active_object

    bpy.ops.object.empty_add(location=(5, 6, 7))
    child_obj = bpy.context.active_object

    # Set parent first
    child_obj.parent = parent_obj

    # Store world location
    original_world_loc = child_obj.matrix_world.translation.copy()

    # Clear parent while keeping transform
    clear_parent_keep_transform(child_obj)

    # Verify parent was cleared
    assert child_obj.parent is None

    # Verify world location is preserved
    assert child_obj.matrix_world.translation.x == pytest.approx(original_world_loc.x, abs=1e-4)
    assert child_obj.matrix_world.translation.y == pytest.approx(original_world_loc.y, abs=1e-4)
    assert child_obj.matrix_world.translation.z == pytest.approx(original_world_loc.z, abs=1e-4)


def test_clear_parent_keep_transform_none():
    """Test with None object."""
    clear_parent_keep_transform(None)
    # Should not raise error


def test_clear_parent_keep_transform_no_parent():
    """Test clearing parent on object without parent."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add(location=(1, 2, 3))
    obj = bpy.context.active_object

    # Store location
    original_loc = obj.matrix_world.translation.copy()

    # Clear parent (should be a no-op)
    clear_parent_keep_transform(obj)

    # Location should be unchanged
    assert obj.matrix_world.translation.x == pytest.approx(original_loc.x, abs=1e-4)
    assert obj.matrix_world.translation.y == pytest.approx(original_loc.y, abs=1e-4)
    assert obj.matrix_world.translation.z == pytest.approx(original_loc.z, abs=1e-4)


def test_set_parent_with_scale():
    """Test that parenting preserves transform even with scaled parent."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create scaled parent
    bpy.ops.object.empty_add(location=(0, 0, 0))
    parent_obj = bpy.context.active_object
    parent_obj.scale = (2.0, 2.0, 2.0)

    # Create child
    bpy.ops.object.empty_add(location=(4, 0, 0))
    child_obj = bpy.context.active_object

    original_world_loc = child_obj.matrix_world.translation.copy()

    # Parent with transform preservation
    set_parent_keep_transform(child_obj, parent_obj)

    # World location should still be at (4, 0, 0)
    assert child_obj.matrix_world.translation.x == pytest.approx(original_world_loc.x, abs=1e-4)
    assert child_obj.matrix_world.translation.y == pytest.approx(original_world_loc.y, abs=1e-4)
    assert child_obj.matrix_world.translation.z == pytest.approx(original_world_loc.z, abs=1e-4)
