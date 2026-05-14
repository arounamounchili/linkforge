"""Helper utilities for Blender property groups.

This module provides optimized helper functions for property update callbacks.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, cast

import bpy
from bpy.types import Context


def find_property_owner(context: Context, property_group: Any, property_attr: str) -> Any | None:
    """Find the Blender object that owns a given property group instance.

    This is an optimized helper for property update callbacks that need to find
    their owning object. It tries multiple strategies from fastest to slowest:
    1. Check id_data (most reliable and fastest)
    2. Check context.object (active object) first
    3. Check context.selected_objects
    4. Fall back to full scene search as last resort

    Args:
        context: Blender context
        property_group: The property group instance (self in update callback)
        property_attr: The attribute name on objects (e.g., "linkforge_sensor")

    Returns:
        The object that owns this property group, or None if not found
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

    # Strategy 3: Check selected objects (faster than full scene search)
    if hasattr(context, "selected_objects"):
        for obj in context.selected_objects:
            if hasattr(obj, property_attr) and getattr(obj, property_attr) == property_group:
                return obj

    # Strategy 4: Fall back to full scene search (slowest)
    if hasattr(context, "scene") and context.scene:
        for obj in context.scene.objects:
            if hasattr(obj, property_attr) and getattr(obj, property_attr) == property_group:
                return obj

    return None


if TYPE_CHECKING:
    from linkforge.blender.properties.joint_props import JointPropertyGroup
    from linkforge.blender.properties.link_props import LinkPropertyGroup
    from linkforge.blender.properties.robot_props import RobotPropertyGroup
    from linkforge.blender.properties.sensor_props import SensorPropertyGroup
    from linkforge.blender.properties.transmission_props import (
        TransmissionPropertyGroup,
    )


def get_link_props(obj: bpy.types.Object | None) -> LinkPropertyGroup | None:
    """Type-safe access to LinkForge link properties on a Blender object."""
    if obj is None:
        return None
    return cast("LinkPropertyGroup | None", getattr(obj, "linkforge", None))


def get_joint_props(obj: bpy.types.Object | None) -> JointPropertyGroup | None:
    """Type-safe access to LinkForge joint properties on a Blender object."""
    if obj is None:
        return None
    return cast("JointPropertyGroup | None", getattr(obj, "linkforge_joint", None))


def get_sensor_props(obj: bpy.types.Object | None) -> SensorPropertyGroup | None:
    """Type-safe access to LinkForge sensor properties on a Blender object."""
    if obj is None:
        return None
    return cast("SensorPropertyGroup | None", getattr(obj, "linkforge_sensor", None))


def get_transmission_props(
    obj: bpy.types.Object | None,
) -> TransmissionPropertyGroup | None:
    """Type-safe access to LinkForge transmission properties on a Blender object."""
    if obj is None:
        return None
    return cast(
        "TransmissionPropertyGroup | None",
        getattr(obj, "linkforge_transmission", None),
    )


def get_robot_props(scene: bpy.types.Scene | None) -> RobotPropertyGroup | None:
    """Type-safe access to LinkForge robot properties on a Blender scene."""
    if scene is None:
        return None
    return cast("RobotPropertyGroup | None", getattr(scene, "linkforge", None))
