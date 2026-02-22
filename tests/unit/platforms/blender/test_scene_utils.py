"""Tests for scene_utils module."""

import bpy
from linkforge.blender.utils.scene_utils import move_to_collection


def test_move_to_collection_basic():
    """Test moving an object to a new collection."""
    # Clean scene
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create a cube
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Create a new collection
    target_collection = bpy.data.collections.new("TestCollection")
    bpy.context.scene.collection.children.link(target_collection)

    # Move object to new collection
    move_to_collection(obj, target_collection)

    # Verify object is in target collection
    assert obj in target_collection.objects[:]
    # Verify object is not in scene root collection
    assert obj not in bpy.context.scene.collection.objects[:]


def test_move_to_collection_already_in_target():
    """Test moving an object to its current collection."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create collection and object
    target_collection = bpy.data.collections.new("TestCollection")
    bpy.context.scene.collection.children.link(target_collection)

    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Move to target once
    move_to_collection(obj, target_collection)

    # Move again (should be idempotent)
    move_to_collection(obj, target_collection)

    # Should still be in target collection only once
    assert obj in target_collection.objects[:]
    assert list(obj.users_collection) == [target_collection]


def test_move_to_collection_none_object():
    """Test with None object."""
    target_collection = bpy.data.collections.new("TestCollection")
    bpy.context.scene.collection.children.link(target_collection)

    # Should not raise error
    move_to_collection(None, target_collection)


def test_move_to_collection_none_collection():
    """Test with None collection."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Should not raise error
    move_to_collection(obj, None)


def test_move_to_collection_both_none():
    """Test with both None."""
    move_to_collection(None, None)
    # Should not raise error


def test_move_to_collection_multiple_collections():
    """Test moving object that exists in multiple collections."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create object
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Create multiple collections and link object
    coll1 = bpy.data.collections.new("Collection1")
    coll2 = bpy.data.collections.new("Collection2")
    target_coll = bpy.data.collections.new("TargetCollection")

    bpy.context.scene.collection.children.link(coll1)
    bpy.context.scene.collection.children.link(coll2)
    bpy.context.scene.collection.children.link(target_coll)

    coll1.objects.link(obj)
    coll2.objects.link(obj)

    # Move to target
    move_to_collection(obj, target_coll)

    # Should only be in target collection now
    assert obj in target_coll.objects[:]
    assert obj not in coll1.objects[:]
    assert obj not in coll2.objects[:]
