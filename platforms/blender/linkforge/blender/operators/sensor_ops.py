"""Operators for managing robot sensors."""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.types import Context, Operator

from ..properties.link_props import sanitize_urdf_name
from ..utils.context import context_and_mode_guard
from ..utils.decorators import safe_execute
from ..utils.scene_utils import clear_stats_cache


class LINKFORGE_OT_create_sensor(Operator):
    """Create a new robot sensor at selected link's location.

    This operator initializes a sensor (Blender Empty with a sphere display)
    at the world origin of the currently selected link object, setting up
    parent-child relationships and default sensor properties.
    """

    bl_idname = "linkforge.create_sensor"
    bl_label = "Create Sensor"
    bl_description = "Create a new robot sensor at the selected link's location and orientation"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False

        # Only allow if object is selected
        if not obj.select_get():
            return False

        # Allow if object is a link (not a joint!)
        if hasattr(obj, "linkforge") and typing.cast(typing.Any, obj).linkforge.is_robot_link:
            return True

        return bool(
            obj.parent
            and hasattr(obj.parent, "linkforge")
            and typing.cast(typing.Any, obj.parent).linkforge.is_robot_link
        )

    @safe_execute
    def execute(self, context: Context) -> set[str]:
        """Execute the operator."""
        obj = context.active_object
        if not obj or not (
            hasattr(obj, "linkforge") or (obj.parent and hasattr(obj.parent, "linkforge"))
        ):
            return {"CANCELLED"}

        # Get the link object (either selected directly or parent of selected visual)
        link_obj = (
            obj
            if obj
            and hasattr(obj, "linkforge")
            and typing.cast(typing.Any, obj).linkforge.is_robot_link
            else (obj.parent if obj else None)
        )
        if not link_obj:
            return {"CANCELLED"}

        # Get preferred empty size from addon preferences
        empty_size = 0.1  # Default fallback
        from ..preferences import get_addon_prefs

        addon_prefs = get_addon_prefs(context)
        if addon_prefs:
            empty_size = getattr(addon_prefs, "sensor_empty_size", empty_size)

        # Create Empty at 0,0,0 initially (we will snap it)
        with context_and_mode_guard(context):
            bpy.ops.object.empty_add(type="SPHERE", location=(0, 0, 0))
            sensor_empty = context.active_object

        # Ensure unique name
        link_name = typing.cast(typing.Any, link_obj).linkforge.link_name if link_obj else "unknown"
        if sensor_empty:
            sensor_empty.name = f"{link_name}_sensor"

        if sensor_empty:
            # STRICT ALIGNMENT PARENTING
            # Instead of calculating world transforms, we explicitly want the sensor
            # to be at the exact origin of the parent link (0,0,0 relative).
            sensor_empty.parent = link_obj
            sensor_empty.matrix_parent_inverse.identity()  # Remove hidden Blender offset
            sensor_empty.rotation_mode = "XYZ"
            sensor_empty.location = (0, 0, 0)
            sensor_empty.rotation_euler = (0, 0, 0)
            sensor_empty.scale = (1, 1, 1)

            # Move to parent's collection
            for coll in list(sensor_empty.users_collection):
                coll.objects.unlink(sensor_empty)
            if link_obj and link_obj.users_collection:
                link_obj.users_collection[0].objects.link(sensor_empty)

            # Set display size from preferences
            sensor_empty.empty_display_size = empty_size

        # Enable sensor properties
        if sensor_empty:
            sensor_props = typing.cast(typing.Any, sensor_empty).linkforge_sensor
            sensor_props.is_robot_sensor = True
            sensor_props.sensor_name = sanitize_urdf_name(sensor_empty.name)

            # Set default sensor type
            sensor_props.sensor_type = "CAMERA"

            # Auto-set attached link to the link object
            sensor_props.attached_link = link_obj

        name = sensor_empty.name if sensor_empty else "sensor"
        parent_name = link_obj.name if link_obj else "unknown"
        self.report({"INFO"}, f"Created sensor '{name}' attached to '{parent_name}'")
        clear_stats_cache()
        return {"FINISHED"}


class LINKFORGE_OT_delete_sensor(Operator):
    """Delete the selected sensor Empty.

    This operator removes the selected sensor object from the scene and
    cleans up its references in the LinkForge hierarchy.
    """

    bl_idname = "linkforge.delete_sensor"
    bl_label = "Remove Sensor"
    bl_description = "Remove the selected sensor from the robot"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False
        return bool(
            obj.type == "EMPTY"
            and hasattr(obj, "linkforge_sensor")
            and typing.cast(typing.Any, obj).linkforge_sensor.is_robot_sensor
        )

    @safe_execute
    def execute(self, context: Context) -> set[str]:
        """Execute the operator."""
        obj = context.active_object
        if not obj:
            return {"CANCELLED"}

        sensor_name = typing.cast(typing.Any, obj).linkforge_sensor.sensor_name or obj.name

        # Delete the object
        with context_and_mode_guard(context):
            bpy.data.objects.remove(obj, do_unlink=True)

        self.report({"INFO"}, f"Deleted sensor '{sensor_name}'")
        clear_stats_cache()
        return {"FINISHED"}


# Registration
classes = [
    LINKFORGE_OT_create_sensor,
    LINKFORGE_OT_delete_sensor,
]


def register() -> None:
    """Register operators."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister operators."""
    for cls in reversed(classes):
        with contextlib.suppress(RuntimeError):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
