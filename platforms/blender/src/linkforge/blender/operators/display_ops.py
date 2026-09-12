"""Operators for viewport display, gizmo fitting, and visibility toggles."""

from __future__ import annotations

import contextlib

import bpy
from bpy.types import Context, Operator

from ..constants import PROP_ROBOT
from ..utils.decorators import OperatorReturn, safe_execute
from ..utils.scene_utils import auto_fit_robot_gizmos


class LINKFORGE_OT_auto_fit_gizmo_sizes(Operator):
    """Automatically fit gizmo and empty display sizes to the current robot's physical bounds."""

    bl_idname = "linkforge.auto_fit_gizmo_sizes"
    bl_label = "Fit Gizmos to Robot"
    bl_description = (
        "Auto-scale joint, link, sensor, and inertia display markers to match robot size"
    )
    bl_options = {"REGISTER", "UNDO"}

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the auto-fit calculation and update preferences."""
        scene = context.scene
        if not scene:
            self.report({"WARNING"}, "No active scene found")
            return {"CANCELLED"}

        result = auto_fit_robot_gizmos(scene, context)
        if not result:
            self.report({"WARNING"}, "No robot components detected to fit display sizes to")
            return {"CANCELLED"}

        recommended_size, diagonal = result
        self.report(
            {"INFO"},
            f"Fitted display size to {recommended_size * 100:.1f} cm (robot bounds: {diagonal:.2f} m)",
        )
        return {"FINISHED"}


class LINKFORGE_OT_toggle_collisions(Operator):
    """Toggle viewport visibility for all collision meshes in the robot."""

    bl_idname = "linkforge.toggle_collisions"
    bl_label = "Toggle Collisions"
    bl_description = "Show or hide all collision meshes in the 3D viewport"
    bl_options = {"REGISTER", "UNDO"}

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Toggle the scene show_collisions property."""
        scene = context.scene
        if not scene or not hasattr(scene, PROP_ROBOT):
            self.report({"WARNING"}, "No robot scene properties found")
            return {"CANCELLED"}

        robot_props = getattr(scene, PROP_ROBOT)
        robot_props.show_collisions = not robot_props.show_collisions
        state_str = "Visible" if robot_props.show_collisions else "Hidden"
        self.report({"INFO"}, f"Collision meshes: {state_str}")
        return {"FINISHED"}


classes = [
    LINKFORGE_OT_auto_fit_gizmo_sizes,
    LINKFORGE_OT_toggle_collisions,
]


def register() -> None:
    """Register display operators."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister display operators."""
    for cls in reversed(classes):
        with contextlib.suppress(RuntimeError):
            bpy.utils.unregister_class(cls)
