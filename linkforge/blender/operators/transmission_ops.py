"""Operators for managing robot transmissions."""

from __future__ import annotations

import bpy
from bpy.types import Operator

from ..properties.link_props import sanitize_urdf_name


class LINKFORGE_OT_create_transmission(Operator):
    """Create a new robot transmission"""

    bl_idname = "linkforge.create_transmission"
    bl_label = "Create Transmission"
    bl_description = "Create a new robot transmission at the selected joint's location"
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
        # Require a joint to be selected
        return obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object

        # Get preferred empty size from addon preferences
        empty_size = 0.05  # Default fallback (matches TRANSMISSION_EMPTY_DISPLAY_SIZE)
        from ..preferences import get_addon_prefs

        addon_prefs = get_addon_prefs(context)
        if addon_prefs:
            empty_size = getattr(addon_prefs, "transmission_empty_size", empty_size)

        # Get selected joint (guaranteed by poll())
        joint_obj = obj
        joint_name = obj.linkforge_joint.joint_name
        location = obj.matrix_world.translation.copy()

        # Create Empty at joint's location
        # Use SINGLE_ARROW to represent actuation vector (matches importer)
        bpy.ops.object.empty_add(type="SINGLE_ARROW", location=location)
        transmission_empty = context.active_object
        transmission_empty.name = f"{joint_name}_trans"

        # Parent transmission to joint (matches import behavior)
        transmission_empty.parent = joint_obj
        # STRICT ALIGNMENT:
        # We want the transmission to be exactly at the joint origin.
        # Identity inverse means Local (0,0,0) -> World (Joint Location).
        transmission_empty.matrix_parent_inverse.identity()

        # Reset local position to be at joint origin
        transmission_empty.location = (0, 0, 0)

        # Ensure matrices are up to date before applying rotation logic
        context.view_layer.update()

        # ALIGNMENT: Point arrow along Joint Axis
        if hasattr(joint_obj, "linkforge_joint"):
            jp = joint_obj.linkforge_joint
            axis_vec = None
            if jp.axis == "X":
                axis_vec = (1, 0, 0)
            elif jp.axis == "Y":
                axis_vec = (0, 1, 0)
            elif jp.axis == "Z":
                axis_vec = (0, 0, 1)
            elif jp.axis == "CUSTOM":
                axis_vec = (jp.custom_axis_x, jp.custom_axis_y, jp.custom_axis_z)

            # Note: NEG_X/Y/Z are handled via CUSTOM in joint_props, so no need to check here

            if axis_vec:
                from mathutils import Vector

                vec = Vector(axis_vec)
                if vec.length > 0:
                    # 'TRACK' aligns Z axis (Arrow default) to vector
                    rot_quat = Vector((0, 0, 1)).rotation_difference(vec)
                    transmission_empty.rotation_euler = rot_quat.to_euler()
            else:
                transmission_empty.rotation_euler = (0, 0, 0)
        else:
            transmission_empty.rotation_euler = (0, 0, 0)

        # Move transmission to same collection as parent joint (for clean organization)
        # Remove from all current collections
        for coll in list(transmission_empty.users_collection):
            coll.objects.unlink(transmission_empty)
        # Add to parent's collection
        if joint_obj.users_collection:
            parent_collection = joint_obj.users_collection[0]
            parent_collection.objects.link(transmission_empty)

        # Set display size from preferences
        transmission_empty.empty_display_size = empty_size

        # Enable transmission properties
        transmission_empty.linkforge_transmission.is_robot_transmission = True
        transmission_empty.linkforge_transmission.transmission_name = sanitize_urdf_name(
            transmission_empty.name
        )

        # Set default transmission type
        transmission_empty.linkforge_transmission.transmission_type = "SIMPLE"

        # Auto-set joint (guaranteed by poll())
        transmission_empty.linkforge_transmission.joint_name = joint_obj

        self.report(
            {"INFO"}, f"Created transmission '{transmission_empty.name}' for joint '{joint_name}'"
        )
        return {"FINISHED"}


class LINKFORGE_OT_delete_transmission(Operator):
    """Delete the selected transmission"""

    bl_idname = "linkforge.delete_transmission"
    bl_label = "Remove Transmission"
    bl_description = "Remove the selected transmission from the robot"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False
        return obj.type == "EMPTY" and obj.linkforge_transmission.is_robot_transmission

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object
        transmission_name = obj.linkforge_transmission.transmission_name or obj.name

        # Delete the object
        bpy.data.objects.remove(obj, do_unlink=True)

        self.report({"INFO"}, f"Deleted transmission '{transmission_name}'")
        return {"FINISHED"}


# Registration
def register():
    """Register operators."""
    bpy.utils.register_class(LINKFORGE_OT_create_transmission)
    bpy.utils.register_class(LINKFORGE_OT_delete_transmission)


def unregister():
    """Unregister operators."""
    bpy.utils.unregister_class(LINKFORGE_OT_delete_transmission)
    bpy.utils.unregister_class(LINKFORGE_OT_create_transmission)


if __name__ == "__main__":
    register()
