"""Helper utilities for Blender property groups.

This module provides optimized helper functions for property update callbacks.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, cast

import bpy
from bpy.types import Context

from ..constants import (
    PROP_JOINT,
    PROP_LINK,
    PROP_ROBOT,
    PROP_SENSOR,
    PROP_TRANSMISSION,
)

if TYPE_CHECKING:
    from ..properties.joint_props import JointPropertyGroup
    from ..properties.link_props import LinkPropertyGroup
    from ..properties.robot_props import RobotPropertyGroup
    from ..properties.sensor_props import SensorPropertyGroup
    from ..properties.transmission_props import (
        TransmissionPropertyGroup,
    )


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
    # Check id_data (most reliable and fastest)
    if (
        hasattr(property_group, "id_data")
        and property_group.id_data
        and isinstance(property_group.id_data, bpy.types.Object)
        and hasattr(property_group.id_data, property_attr)
        and getattr(property_group.id_data, property_attr) == property_group
    ):
        return property_group.id_data

    # Check active object (fast fallback)
    if (
        hasattr(context, "object")
        and context.object
        and hasattr(context.object, property_attr)
        and getattr(context.object, property_attr) == property_group
    ):
        return context.object

    # Check selected objects (faster than full scene search)
    if hasattr(context, "selected_objects"):
        for obj in context.selected_objects:
            if hasattr(obj, property_attr) and getattr(obj, property_attr) == property_group:
                return obj

    # Fall back to full scene search (slowest)
    if hasattr(context, "scene") and context.scene:
        for obj in context.scene.objects:
            if hasattr(obj, property_attr) and getattr(obj, property_attr) == property_group:
                return obj

    return None


def get_link_props(obj: bpy.types.Object | None) -> LinkPropertyGroup | None:
    """Type-safe access to LinkForge link properties on a Blender object."""
    if obj is None:
        return None
    return cast("LinkPropertyGroup | None", getattr(obj, PROP_LINK, None))


def get_joint_props(obj: bpy.types.Object | None) -> JointPropertyGroup | None:
    """Type-safe access to LinkForge joint properties on a Blender object."""
    if obj is None:
        return None
    return cast("JointPropertyGroup | None", getattr(obj, PROP_JOINT, None))


def get_sensor_props(obj: bpy.types.Object | None) -> SensorPropertyGroup | None:
    """Type-safe access to LinkForge sensor properties on a Blender object."""
    if obj is None:
        return None
    return cast("SensorPropertyGroup | None", getattr(obj, PROP_SENSOR, None))


def get_transmission_props(
    obj: bpy.types.Object | None,
) -> TransmissionPropertyGroup | None:
    """Type-safe access to LinkForge transmission properties on a Blender object."""
    if obj is None:
        return None
    return cast(
        "TransmissionPropertyGroup | None",
        getattr(obj, PROP_TRANSMISSION, None),
    )


def get_robot_props(scene: bpy.types.Scene | None) -> RobotPropertyGroup | None:
    """Type-safe access to LinkForge robot properties on a Blender scene."""
    if scene is None:
        return None
    return cast("RobotPropertyGroup | None", getattr(scene, PROP_ROBOT, None))


# Global queue for deferred datablock renames (used when RNA is locked or in background mode)
PENDING_RENAMES: list[tuple[Any, str]] = []


def flush_deferred_renames() -> None:
    """Execute all pending datablock renames in the queue.

    Used primarily in background mode or during tests to ensure synchronization
    is complete after depsgraph evaluation.
    """
    global PENDING_RENAMES
    remaining = []
    while PENDING_RENAMES:
        obj, new_name = PENDING_RENAMES.pop(0)
        try:
            if obj and hasattr(obj, "name"):
                obj.name = new_name
        except Exception:
            # If it fails (likely read-only / RNA still locked), keep it for next flush
            remaining.append((obj, new_name))

    PENDING_RENAMES.extend(remaining)


def safe_set_id_name(id_data: Any, sanitized_name: str) -> None:
    """Safely update a Blender datablock's name, deferring if RNA is locked.

    During property updates or depsgraph evaluation, writing directly to
    ID datablocks can raise RuntimeError or AttributeError.
    In GUI mode, updates are deferred via bpy.app.timers.
    In background mode (or headless environments where timers do not run),
    updates are queued into PENDING_RENAMES and flushed on depsgraph updates.

    Note:
        Blender automatically handles duplicate object name collisions by
        appending numeric suffixes (e.g. '.001', '.002'). The LinkForge
        model identity is preserved in `source_name_stored`.
    """
    if not id_data or not hasattr(id_data, "name") or id_data.name == sanitized_name:
        return

    try:
        id_data.name = sanitized_name
    except (AttributeError, RuntimeError):
        if not getattr(bpy.app, "background", False) and hasattr(bpy.app, "timers"):

            def deferred_rename() -> None:
                import contextlib

                if id_data and hasattr(id_data, "name"):
                    with contextlib.suppress(Exception):
                        id_data.name = sanitized_name
                return None

            bpy.app.timers.register(deferred_rename, first_interval=0.01)
        else:
            PENDING_RENAMES.append((id_data, sanitized_name))
