"""Operators for managing robot sensors."""

from __future__ import annotations

import bpy
from bpy.types import Operator

from ..properties.link_props import sanitize_urdf_name


class LINKFORGE_OT_create_sensor(Operator):
    """Create a new robot sensor at selected link's location"""

    bl_idname = "linkforge.create_sensor"
    bl_label = "Create Sensor"
    bl_description = "Create a new robot sensor at the selected link's location and orientation"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False

        # Only allow if object is selected
        if not obj.select_get():
            return False

        # Allow if object is a link (not a joint!)
        if hasattr(obj, "linkforge") and obj.linkforge.is_robot_link:
            return True

        # Allow if object is a child of a link (visual/collision mesh)
        if obj.parent and hasattr(obj.parent, "linkforge"):
            # Make sure parent is a LINK, not a joint
            return obj.parent.linkforge.is_robot_link

        return False

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object

        # Get the link object (either selected directly or parent of selected visual)
        link_obj = obj if obj.linkforge.is_robot_link else obj.parent

        # Get preferred empty size from addon preferences
        empty_size = 0.1  # Default fallback
        try:
            addon_prefs = context.preferences.addons.get("bl_ext.user_default.linkforge")
            if addon_prefs and hasattr(addon_prefs, "preferences"):
                prefs = addon_prefs.preferences
                if hasattr(prefs, "sensor_empty_size"):
                    empty_size = prefs.sensor_empty_size
        except (AttributeError, KeyError):
            pass

        # Create Empty at 0,0,0 initially (we will snap it)
        bpy.ops.object.empty_add(type="SPHERE", location=(0, 0, 0))
        sensor_empty = context.active_object

        # Ensure unique name
        sensor_empty.name = f"{link_obj.linkforge.link_name}_sensor"

        # STRICT ALIGNMENT PARENTING
        # Instead of calculating world transforms, we explicitly want the sensor
        # to be at the exact origin of the parent link (0,0,0 relative).
        sensor_empty.parent = link_obj
        sensor_empty.matrix_parent_inverse.identity()  # Remove hidden Blender offset
        sensor_empty.location = (0, 0, 0)
        sensor_empty.rotation_euler = (0, 0, 0)
        sensor_empty.scale = (1, 1, 1)

        # Move to parent's collection
        for coll in list(sensor_empty.users_collection):
            coll.objects.unlink(sensor_empty)
        if link_obj.users_collection:
            link_obj.users_collection[0].objects.link(sensor_empty)

        # Set display size from preferences
        sensor_empty.empty_display_size = empty_size

        # Enable sensor properties
        sensor_empty.linkforge_sensor.is_robot_sensor = True
        sensor_empty.linkforge_sensor.sensor_name = sanitize_urdf_name(sensor_empty.name)

        # Set default sensor type
        sensor_empty.linkforge_sensor.sensor_type = "CAMERA"

        # Auto-set attached link to the link object
        sensor_empty.linkforge_sensor.attached_link = link_obj

        self.report({"INFO"}, f"Created sensor '{sensor_empty.name}' attached to '{link_obj.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_delete_sensor(Operator):
    """Delete the selected sensor"""

    bl_idname = "linkforge.delete_sensor"
    bl_label = "Remove Sensor"
    bl_description = "Remove the selected sensor from the robot"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False
        return obj.type == "EMPTY" and obj.linkforge_sensor.is_robot_sensor

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object
        sensor_name = obj.linkforge_sensor.sensor_name or obj.name

        # Delete the object
        bpy.data.objects.remove(obj, do_unlink=True)

        self.report({"INFO"}, f"Deleted sensor '{sensor_name}'")
        return {"FINISHED"}


# Registration
def register():
    """Register operators."""
    bpy.utils.register_class(LINKFORGE_OT_create_sensor)
    bpy.utils.register_class(LINKFORGE_OT_delete_sensor)


def unregister():
    """Unregister operators."""
    bpy.utils.unregister_class(LINKFORGE_OT_delete_sensor)
    bpy.utils.unregister_class(LINKFORGE_OT_create_sensor)


if __name__ == "__main__":
    register()
