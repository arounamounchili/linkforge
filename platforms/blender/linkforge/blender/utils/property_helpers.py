"""Helper utilities for Blender property groups.

This module provides optimized helper functions for property update callbacks.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import bpy
from bpy.types import Context

if TYPE_CHECKING:
    pass


def find_property_owner(context: Context, property_group: Any, property_attr: str) -> Any | None:
    """Find the Blender object that owns a given property group instance.

    This is an optimized helper for property update callbacks that need to find
    their owning object. It tries multiple strategies from fastest to slowest:
    1. Check context.object (active object) first
    2. Check context.selected_objects
    3. Fall back to full scene search as last resort

    Args:
        context: Blender context
        property_group: The property group instance (self in update callback)
        property_attr: The attribute name on objects (e.g., "linkforge_sensor")

    Returns:
        The object that owns this property group, or None if not found

    Example:
        >>> def update_sensor_name(self, context):
        >>>     obj = find_property_owner(context, self, "linkforge_sensor")
        >>>     if obj:
        >>>         obj.name = self.sensor_name
    """
    # Strategy 1: Check id_data (most reliable and fastest)
    if (
        hasattr(property_group, "id_data")
        and property_group.id_data
        and isinstance(property_group.id_data, bpy.types.Object)
        and hasattr(property_group.id_data, property_attr)
        and getattr(property_group.id_data, property_attr) == property_group
    ):
        return property_group.id_data

    # Strategy 2: Check active object (fast fallback)
    if (
        hasattr(context, "object")
        and context.object
        and hasattr(context.object, property_attr)
        and getattr(context.object, property_attr) == property_group
    ):
        return context.object

    # Strategy 2: Check selected objects (faster than full scene search)
    if hasattr(context, "selected_objects"):
        for obj in context.selected_objects:
            if hasattr(obj, property_attr) and getattr(obj, property_attr) == property_group:
                return obj

    # Strategy 3: Fall back to full scene search (slowest)
    if hasattr(context, "scene") and context.scene:
        for obj in context.scene.objects:
            if hasattr(obj, property_attr) and getattr(obj, property_attr) == property_group:
                return obj

    return None
