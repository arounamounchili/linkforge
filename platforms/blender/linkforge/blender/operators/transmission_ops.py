"""Operators for managing robot transmissions."""

from __future__ import annotations

import contextlib
import typing

import bpy

from ..properties.link_props import sanitize_robot_name
from ..utils.decorators import OperatorReturn, safe_execute
from ..utils.property_helpers import get_joint_props, get_transmission_props
from ..utils.scene_utils import clear_stats_cache

if typing.TYPE_CHECKING:
    from bpy.types import Context, Operator

else:
    # Runtime fallback for mock environments where bpy.types might be partially loaded.
    Context = typing.Any
    Operator = getattr(getattr(bpy, "types", object), "Operator", object)


class LINKFORGE_OT_create_transmission(Operator):
    """Create a new robot transmission.

    This operator initializes a transmission (Blender Empty with a single
    arrow display) at the world origin of the currently selected joint,
    setting up parent-child relationships and default transmission properties.
    """

    bl_idname = "linkforge.create_transmission"
    bl_label = "Create Transmission"
    bl_description = "Create a new robot transmission at the selected joint's location"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if a joint object is selected.
        """
        obj = context.active_object
        if obj is None:
            return False
        # Only allow if object is selected
        if not obj.select_get():
            return False
        # Require a joint to be selected
        return bool(obj.type == "EMPTY" and (jp := get_joint_props(obj)) and jp.is_robot_joint)

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state.
        """
        obj = context.active_object
        if not obj:
            return {"CANCELLED"}

        # Use the standalone logic function
        success = create_transmission_for_joint(obj, context)
        return {"FINISHED"} if success else {"CANCELLED"}


def create_transmission_for_joint(joint_obj: typing.Any, context: Context) -> bool:
    """Logic for creating a transmission for a specific joint.

    Args:
        joint_obj: The Blender object representing the joint.
        context: The current Blender context (or adapter).

    Returns:
        True if successful.
    """
    # Get preferred empty size from addon preferences
    empty_size = 0.05  # Default fallback
    from ..preferences import get_addon_prefs

    addon_prefs = get_addon_prefs(context)
    if addon_prefs:
        empty_size = getattr(addon_prefs, "transmission_empty_size", empty_size)

    # Get selected joint
    if not (joint_props := get_joint_props(joint_obj)):
        return False
    joint_name = joint_props.joint_name
    location = joint_obj.matrix_world.translation.copy()

    # Create Empty at joint's location
    ops = getattr(context, "ops", bpy.ops)
    ops.object.empty_add(type="SINGLE_ARROW", location=location)
    transmission_empty = getattr(context, "active_object", bpy.context.active_object)

    if not transmission_empty:
        return False

    # Use joint_name property if set, otherwise fallback to object name
    final_joint_name = joint_name if joint_name else joint_obj.name
    transmission_empty.name = f"{final_joint_name}_trans"

    # Parent transmission to joint
    transmission_empty.parent = joint_obj
    transmission_empty.matrix_parent_inverse.identity()
    transmission_empty.location = (0, 0, 0)

    # Update view layer to ensure matrices are ready
    view_layer = context.view_layer
    if view_layer is not None:
        view_layer.update()

    # ALIGNMENT: Point arrow along Joint Axis
    if jp := get_joint_props(joint_obj):
        axis_vec = None
        if jp.axis == "X":
            axis_vec = (1, 0, 0)
        elif jp.axis == "Y":
            axis_vec = (0, 1, 0)
        elif jp.axis == "Z":
            axis_vec = (0, 0, 1)
        elif jp.axis == "CUSTOM":
            axis_vec = (jp.custom_axis_x, jp.custom_axis_y, jp.custom_axis_z)

        if axis_vec:
            from mathutils import Vector

            vec = Vector(axis_vec)
            if vec.length > 0:
                rot_quat = Vector((0, 0, 1)).rotation_difference(vec)
                transmission_empty.rotation_euler = rot_quat.to_euler("XYZ")
        else:
            transmission_empty.rotation_euler = (0, 0, 0)
    else:
        transmission_empty.rotation_euler = (0, 0, 0)

    # Move to same collection as parent
    for coll in list(transmission_empty.users_collection):
        coll.objects.unlink(transmission_empty)
    if joint_obj.users_collection:
        parent_collection = joint_obj.users_collection[0]
        parent_collection.objects.link(transmission_empty)

    # Set display size and properties
    transmission_empty.empty_display_size = empty_size
    if trans_props := get_transmission_props(transmission_empty):
        trans_props.is_robot_transmission = True

        trans_props.transmission_name = sanitize_robot_name(transmission_empty.name)
        trans_props.transmission_type = "SIMPLE"
        trans_props.joint_name = joint_obj

    clear_stats_cache()
    return True


class LINKFORGE_OT_delete_transmission(Operator):
    """Delete the selected transmission Empty.

    This operator removes the selected transmission object from the scene and
    cleans up its references in the LinkForge hierarchy.
    """

    bl_idname = "linkforge.delete_transmission"
    bl_label = "Remove Transmission"
    bl_description = "Remove the selected transmission from the robot"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if a transmission object is selected.
        """
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False
        return bool(
            obj.type == "EMPTY" and (tp := get_transmission_props(obj)) and tp.is_robot_transmission
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state.
        """
        obj = context.active_object
        if not obj:
            return {"CANCELLED"}

        # Use the standalone logic function
        success = delete_transmission_for_object(obj, context)
        return {"FINISHED"} if success else {"CANCELLED"}


def delete_transmission_for_object(obj: typing.Any, context: Context) -> bool:
    """Logic for deleting a transmission object.

    Args:
        obj: The transmission object to delete.
        context: The current Blender context (or adapter).

    Returns:
        True if successful.
    """
    # Delete the object
    data = getattr(context, "data", bpy.data)
    data.objects.remove(obj, do_unlink=True)

    clear_stats_cache()
    return True


# Registration
classes = [
    LINKFORGE_OT_create_transmission,
    LINKFORGE_OT_delete_transmission,
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
