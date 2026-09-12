"""Operators for viewport selection and tree navigation."""

from __future__ import annotations

import contextlib

import bpy
from bpy.props import StringProperty
from bpy.types import Context, Operator

from ..utils.decorators import OperatorReturn, safe_execute
from ..utils.scene_utils import build_tree_from_stats, get_robot_statistics


class LINKFORGE_OT_select_tree_object(Operator):
    """Select object from kinematic tree."""

    bl_idname = "linkforge.select_tree_object"
    bl_label = "Select Object"
    bl_description = "Select this object in the 3D viewport"
    bl_options = {"REGISTER", "UNDO"}

    object_name: StringProperty(  # type: ignore
        name="Object Name", description="Name of the object to select"
    )
    object_type: StringProperty(  # type: ignore
        name="Object Type", description="Type of object (link, joint, sensor)", default=""
    )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene:
            return {"CANCELLED"}
        obj = scene.objects.get(self.object_name)
        if not obj:
            self.report({"WARNING"}, f"Object '{self.object_name}' not found")
            return {"CANCELLED"}

        # Deselect all
        bpy.ops.object.select_all(action="DESELECT")

        # Select and activate the object
        obj.select_set(True)
        vl = context.view_layer
        if vl:
            vl.objects.active = obj

        return {"FINISHED"}


class LINKFORGE_OT_select_root_link(Operator):
    """Select the root link of the robot."""

    bl_idname = "linkforge.select_root_link"
    bl_label = "Select Root Link"
    bl_description = "Select the root link of the robot in the 3D viewport"
    bl_options = {"REGISTER", "UNDO"}

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene:
            return {"CANCELLED"}

        stats = get_robot_statistics(scene)
        _, root_link, _, _ = build_tree_from_stats(stats)

        # Select the root link
        if root_link:
            root_obj = scene.objects.get(root_link)
            if root_obj:
                bpy.ops.object.select_all(action="DESELECT")
                root_obj.select_set(True)
                vl = context.view_layer
                if vl:
                    vl.objects.active = root_obj

            return {"FINISHED"}
        else:
            self.report({"WARNING"}, "No root link found for the robot.")
            return {"CANCELLED"}


# Registration
classes = [
    LINKFORGE_OT_select_tree_object,
    LINKFORGE_OT_select_root_link,
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
