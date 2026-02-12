"""General Blender utility functions for object and collection management."""

from __future__ import annotations

import contextlib

import bpy


def move_to_collection(obj: bpy.types.Object, collection: bpy.types.Collection) -> None:
    """Safely move an object to a specific collection.

    This unlinks the object from all existing collections and links it to
     the target collection.

    Args:
        obj: Blender object to move
        collection: Target Blender collection
    """
    if not obj or not collection:
        return

    # Unlink from all current collections
    for coll in list(obj.users_collection):
        if coll != collection:
            coll.objects.unlink(obj)

    # Link to target if not already there
    if obj.name not in collection.objects:
        # Object might be already linked but not showing in collection.objects lookup yet
        with contextlib.suppress(RuntimeError):
            collection.objects.link(obj)
