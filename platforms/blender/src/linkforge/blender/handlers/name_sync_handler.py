"""Handler for synchronizing LinkForge names with Blender object names.

This ensures that renaming an object in the Outliner or duplicating it
automatically updates the corresponding LinkForge robot model identity.
"""

from __future__ import annotations

import typing

import bpy

from ..core._utils.string_utils import sanitize_name
from ..utils.property_helpers import (
    PENDING_RENAMES,
    flush_deferred_renames,
    get_joint_props,
    get_link_props,
    get_sensor_props,
)

__all__ = [
    "PENDING_RENAMES",
    "flush_deferred_renames",
    "on_depsgraph_update_post",
    "register",
    "sync_object_identity",
    "sync_scene_identities",
    "unregister",
]

try:
    from bpy.app.handlers import persistent
except (ImportError, AttributeError):
    F = typing.TypeVar("F", bound=typing.Callable[..., typing.Any])

    def persistent(func: F) -> F:
        """Dummy decorator for environments without real Blender handlers."""
        return func


def sync_object_identity(obj: typing.Any) -> None:
    """Synchronize a single object's LinkForge identity with its datablock name.

    Adheres to Blender depsgraph handler guidelines:
    1. Unwraps evaluated proxy objects to their persistent original datablock (`update.id.original`).
    2. Guards against ReferenceError when objects are being deleted.
    3. Exits early if properties already match to prevent recursive update cycles.
    4. Only operates on objects marked as LinkForge robot components.
    """
    try:
        real_obj = getattr(obj, "original", obj)
        if not getattr(real_obj, "type", None):
            return

        # Sync Link identities
        if (lf := get_link_props(real_obj)) and lf.is_robot_link:
            sanitized = sanitize_name(real_obj.name)
            if sanitized != lf.link_name or real_obj.name != sanitized:
                lf.link_name = sanitized

        # Sync Joint identities
        if (jf := get_joint_props(real_obj)) and jf.is_robot_joint:
            sanitized = sanitize_name(real_obj.name)
            if sanitized != jf.joint_name or real_obj.name != sanitized:
                jf.joint_name = sanitized

        # Sync Sensor identities
        if (sf := get_sensor_props(real_obj)) and sf.is_robot_sensor:
            sanitized = sanitize_name(real_obj.name)
            if sanitized != sf.sensor_name or real_obj.name != sanitized:
                sf.sensor_name = sanitized
    except (ReferenceError, AttributeError):
        pass


def sync_scene_identities(scene: typing.Any) -> None:
    """Synchronize all robot components in the scene with their datablock names."""
    flush_deferred_renames()
    if not scene or not hasattr(scene, "objects"):
        return
    for obj in scene.objects:
        sync_object_identity(obj)


@persistent  # type: ignore[untyped-decorator]
def on_depsgraph_update_post(_scene: typing.Any, _depsgraph: typing.Any) -> None:
    """Synchronize LinkForge identities when objects are renamed in the Outliner.

    This handler detects renames in the depsgraph and updates the corresponding
    LinkForge property groups. We only perform synchronization for robot components,
    avoiding overhead on standard Blender objects.
    """
    flush_deferred_renames()

    for update in _depsgraph.updates:
        sync_object_identity(update.id)


def register() -> None:
    """Register name synchronization handlers."""
    if on_depsgraph_update_post not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(on_depsgraph_update_post)


def unregister() -> None:
    """Unregister name sync handler."""
    if on_depsgraph_update_post in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(on_depsgraph_update_post)
