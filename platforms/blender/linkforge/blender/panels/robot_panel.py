"""UI Panel for robot-level properties and validation."""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.props import StringProperty
from bpy.types import Context, Operator

from ..utils.decorators import safe_execute
from ..utils.scene_utils import build_tree_from_stats, get_robot_statistics


class LINKFORGE_OT_select_tree_object(Operator):
    """Select object from kinematic tree."""

    bl_idname = "linkforge.select_tree_object"
    bl_label = "Select Object"
    bl_description = "Select this object in the 3D viewport"
    bl_options = {"REGISTER", "UNDO"}

    object_name: bpy.props.StringProperty(  # type: ignore
        name="Object Name", description="Name of the object to select"
    )
    object_type: bpy.props.StringProperty(  # type: ignore
        name="Object Type", description="Type of object (link/joint)", default="link"
    )
    joint_name = StringProperty(name="Joint Name")  # type: ignore[func-returns-value]
    parent_link = StringProperty(name="Parent Link")  # type: ignore[func-returns-value]
    child_link = StringProperty(name="Child Name")  # type: ignore[func-returns-value]

    @safe_execute
    def execute(self, context: Context) -> set[str]:
        """Execute the operator.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        # Find the object
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
    def execute(self, context: Context) -> set[str]:
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


class LINKFORGE_OT_clear_component_search(Operator):
    """Clear component browser search filter."""

    bl_idname = "linkforge.clear_component_search"
    bl_label = "Clear Search"
    bl_description = "Clear component browser search filter"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Only enable operator when search text exists.

        Args:
            context: The execution context.

        Returns:
            True if the operator can be executed, False otherwise.
        """
        if not hasattr(context.scene, "linkforge"):
            return False
        props = typing.cast(typing.Any, context.scene).linkforge
        return bool(props.component_browser_search)

    @safe_execute
    def execute(self, context: Context) -> set[str]:
        """Clear the component browser search field.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene:
            return {"CANCELLED"}
        props = typing.cast(typing.Any, scene).linkforge
        props.component_browser_search = ""
        return {"FINISHED"}


# Registration
classes = [
    LINKFORGE_OT_select_tree_object,
    LINKFORGE_OT_select_root_link,
    LINKFORGE_OT_clear_component_search,
]


def register() -> None:
    """Register panel."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister panel."""
    for cls in reversed(classes):
        with contextlib.suppress(RuntimeError):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
