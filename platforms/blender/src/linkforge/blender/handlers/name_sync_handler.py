"""Handler for synchronizing LinkForge names with Blender object names.

This ensures that renaming an object in the Outliner or duplicating it
automatically updates the corresponding LinkForge robot model identity.
"""

from __future__ import annotations

import typing

if typing.TYPE_CHECKING:
    pass
from linkforge.core._utils.string_utils import sanitize_name

from ..utils.property_helpers import (
    get_joint_props,
    get_link_props,
    get_sensor_props,
    get_transmission_props,
)

try:
    from bpy.app.handlers import persistent
except (ImportError, AttributeError):
    F = typing.TypeVar("F", bound=typing.Callable[..., typing.Any])

    def persistent(func: F) -> F:
        """Dummy decorator for environments without real Blender handlers."""
        return func


# Global queue for deferred renames (used primarily in background mode where timers don't run)
PENDING_RENAMES: list[tuple[typing.Any, str]] = []


def flush_deferred_renames() -> None:
    """Execute all pending renames in the queue.

    Used primarily in background mode or during tests to ensure synchronization
    is complete after a depsgraph update.
    """
    global PENDING_RENAMES
    remaining = []
    while PENDING_RENAMES:
        obj, new_name = PENDING_RENAMES.pop(0)
        try:
            if obj and hasattr(obj, "name"):
                obj.name = new_name
        except Exception:
            # If it fails (likely read-only), keep it for next flush
            remaining.append((obj, new_name))

    PENDING_RENAMES.extend(remaining)


@persistent  # type: ignore[untyped-decorator]
def on_depsgraph_update_post(_scene: typing.Any, _depsgraph: typing.Any) -> None:
    """Synchronize LinkForge identities when objects are renamed in the Outliner.

    This handler detects renames in the depsgraph and updates the corresponding
    LinkForge property groups. We only perform synchronization for robot components,
    avoiding overhead on standard Blender objects.
    """
    for update in _depsgraph.updates:
        obj = update.id

        # 1. Sync Link identities
        if (lf := get_link_props(obj)) and lf.is_robot_link:
            sanitized = sanitize_name(obj.name)
            if sanitized != lf.link_name or obj.name != sanitized:
                lf.link_name = sanitized

        # 2. Sync Joint identities
        if (jf := get_joint_props(obj)) and jf.is_robot_joint:
            sanitized = sanitize_name(obj.name)
            if sanitized != jf.joint_name or obj.name != sanitized:
                jf.joint_name = sanitized

        # 3. Sync Sensor identities
        if (sf := get_sensor_props(obj)) and sf.is_robot_sensor:
            sanitized = sanitize_name(obj.name)
            if sanitized != sf.sensor_name:
                sf.sensor_name = sanitized

        # 4. Sync Transmission identities
        if (tf := get_transmission_props(obj)) and tf.is_robot_transmission:
            sanitized = sanitize_name(obj.name)
            if sanitized != tf.transmission_name:
                tf.transmission_name = sanitized


def register() -> None:
    """Register name synchronization handlers."""
    import bpy

    if on_depsgraph_update_post not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(on_depsgraph_update_post)


def unregister() -> None:
    """Unregister name sync handler."""
    import bpy

    if on_depsgraph_update_post in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(on_depsgraph_update_post)
