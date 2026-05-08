"""Tests for transform_utils module."""

import bpy
import pytest
from linkforge.blender.utils.transform_utils import (
    clear_parent_keep_transform,
    set_parent_keep_transform,
)

from tests.blender_test_utils import create_test_object


def test_set_parent_keep_transform_basic(scene) -> None:
    """Test parenting while preserving world transform."""
    # Clean scene
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create parent and child objects
    parent_obj = create_test_object("Parent", None, scene)
    parent_obj.location = (1, 2, 3)

    child_obj = create_test_object("Child", None, scene)
    child_obj.location = (5, 6, 7)

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


def test_set_parent_keep_transform_with_rotation(scene) -> None:
    """Test parenting with rotated parent."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create rotated parent
    parent_obj = create_test_object("ParentRot", None, scene)
    parent_obj.location = (0, 0, 0)
    parent_obj.rotation_euler = (0, 0, 1.5708)  # 90 degrees Z

    # Create child at specific location
    child_obj = create_test_object("ChildAtLoc", None, scene)
    child_obj.location = (1, 0, 0)

    original_world_loc = child_obj.matrix_world.translation.copy()

    # Parent with transform preservation
    set_parent_keep_transform(child_obj, parent_obj)

    # World location should be preserved
    new_world_loc = child_obj.matrix_world.translation
    assert new_world_loc.x == pytest.approx(original_world_loc.x, abs=1e-4)
    assert new_world_loc.y == pytest.approx(original_world_loc.y, abs=1e-4)
    assert new_world_loc.z == pytest.approx(original_world_loc.z, abs=1e-4)


def test_set_parent_keep_transform_none_child(scene) -> None:
    """Test with None child."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    parent_obj = create_test_object("ParentNoneChild", None, scene)

    # Should not raise error
    set_parent_keep_transform(None, parent_obj)


def test_set_parent_keep_transform_none_parent(scene) -> None:
    """Test with None parent."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    child_obj = create_test_object("ChildNoneParent", None, scene)

    # Should not raise error
    set_parent_keep_transform(child_obj, None)


def test_clear_parent_keep_transform_basic(scene) -> None:
    """Test clearing parent while preserving world transform."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create parent and child
    parent_obj = create_test_object("ParentClear", None, scene)
    parent_obj.location = (2, 3, 4)

    child_obj = create_test_object("ChildClear", None, scene)
    child_obj.location = (5, 6, 7)

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


def test_clear_parent_keep_transform_none(scene) -> None:
    """Test with None object."""
    clear_parent_keep_transform(None)
    # Should not raise error


def test_clear_parent_keep_transform_no_parent(scene) -> None:
    """Test clearing parent on object without parent."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    obj = create_test_object("NoParent", None, scene)
    obj.location = (1, 2, 3)

    # Store location
    original_loc = obj.matrix_world.translation.copy()

    # Clear parent (should be a no-op)
    clear_parent_keep_transform(obj)

    # Location should be unchanged
    assert obj.matrix_world.translation.x == pytest.approx(original_loc.x, abs=1e-4)
    assert obj.matrix_world.translation.y == pytest.approx(original_loc.y, abs=1e-4)
    assert obj.matrix_world.translation.z == pytest.approx(original_loc.z, abs=1e-4)


def test_set_parent_with_scale(scene) -> None:
    """Test that parenting preserves transform even with scaled parent."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create scaled parent
    parent_obj = create_test_object("ScaledParent", None, scene)
    parent_obj.scale = (2.0, 2.0, 2.0)

    # Create child
    child_obj = create_test_object("ScaledChild", None, scene)
    child_obj.location = (4, 0, 0)

    original_world_loc = child_obj.matrix_world.translation.copy()

    # Parent with transform preservation
    set_parent_keep_transform(child_obj, parent_obj)

    # World location should still be at (4, 0, 0)
    assert child_obj.matrix_world.translation.x == pytest.approx(original_world_loc.x, abs=1e-4)
    assert child_obj.matrix_world.translation.y == pytest.approx(original_world_loc.y, abs=1e-4)
    assert child_obj.matrix_world.translation.z == pytest.approx(original_world_loc.z, abs=1e-4)
