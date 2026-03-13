"""Operators for managing robot links."""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.types import Context, Operator

from ..properties.link_props import sanitize_urdf_name
from ..utils.decorators import safe_execute
from ..utils.scene_utils import clear_stats_cache

if typing.TYPE_CHECKING:
    from ..properties.link_props import LinkPropertyGroup


class LINKFORGE_OT_create_link(Operator):
    """Mark the selected object as a robot link.

    This operator initializes robot link properties for the selected object,
    setting up default URDF-compatible naming and enabling collision/inertial
    property management.
    """

    bl_idname = "linkforge.create_link"
    bl_label = "Create Link"
    bl_description = "Initialize the selected object as a robot link"
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

        # Only allow if object is not already a link
        return not bool(
            hasattr(obj, "linkforge")
            and typing.cast("LinkPropertyGroup", obj.linkforge).is_robot_link
        )

    @safe_execute
    def execute(self, context: Context) -> set[str]:
        """Execute the operator."""
        obj = context.active_object
        if not obj:
            return {"CANCELLED"}

        # Enable link properties
        props = typing.cast("LinkPropertyGroup", obj.linkforge)
        props.is_robot_link = True
        props.link_name = sanitize_urdf_name(obj.name)

        # Set default collision quality
        props.collision_quality = 100

        self.report({"INFO"}, f"Initialized link '{props.link_name}'")
        clear_stats_cache()
        return {"FINISHED"}


class LINKFORGE_OT_delete_link(Operator):
    """Remove robot link status from the selected object.

    This operator removes LinkForge metadata from the object while
    leaving the visual and physical mesh data intact.
    """

    bl_idname = "linkforge.delete_link"
    bl_label = "Remove Link"
    bl_description = "Remove robot link status from the selected object"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        obj = context.active_object
        return bool(
            obj
            and hasattr(obj, "linkforge")
            and typing.cast("LinkPropertyGroup", obj.linkforge).is_robot_link
        )

    @safe_execute
    def execute(self, context: Context) -> set[str]:
        """Execute the operator."""
        obj = context.active_object
        if not obj:
            return {"CANCELLED"}

        # Disable link properties (preserves other object data)
        typing.cast("LinkPropertyGroup", obj.linkforge).is_robot_link = False

        self.report({"INFO"}, f"Removed robot link status from '{obj.name}'")
        clear_stats_cache()
        return {"FINISHED"}


def update_collision_quality_realtime(
    obj: bpy.types.Object, collision_obj: bpy.types.Object
) -> None:
    """Update collision quality ratio in realtime via Decimate modifier.

    Args:
        obj: The main link object.
        collision_obj: The generated collision object.
    """
    if not collision_obj or not obj:
        return

    # FAST PATH: If we have a Decimate modifier, just update the ratio
    # This provides instant feedback without expensive mesh regeneration
    lf = typing.cast("LinkPropertyGroup", obj.linkforge)
    quality_ratio = lf.collision_quality / 100.0

    decimate_mod = next((m for m in collision_obj.modifiers if m.type == "DECIMATE"), None)
    if decimate_mod and isinstance(decimate_mod, bpy.types.DecimateModifier):
        decimate_mod.ratio = quality_ratio


# Registration
classes = [
    LINKFORGE_OT_create_link,
    LINKFORGE_OT_delete_link,
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
