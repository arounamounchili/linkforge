"""Main UI Panel for LinkForge.

This module defines the parent panel 'LINKFORGE_PT_forge' that other panels attach to.
It must be registered BEFORE any child panels.
"""

from __future__ import annotations

import contextlib

import bpy
from bpy.types import Context, Panel

from ..constants import (
    PROP_ROBOT,
)
from ..preferences import get_addon_prefs


class LINKFORGE_PT_forge(Panel):
    """Parent panel for building robot structure."""

    bl_label = "Forge"
    bl_description = "Step 1: Create robot structure with links and joints"
    bl_idname = "LINKFORGE_PT_forge"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 0

    def draw(self, context: Context) -> None:
        """Draw the panel.

        Args:
            context: The current Blender context.
        """
        layout = self.layout
        if not layout:
            return

        if not context.scene:
            return
        scene_props = getattr(context.scene, PROP_ROBOT)

        if scene_props.is_importing:
            # Active Import Status
            box = layout.box()
            if box:
                # Active Import Status
                box.alert = True
                row = box.row()
                if row:
                    row.label(text=scene_props.import_status, icon="URL")

                    row = box.row()
                    if row:
                        row.scale_y = 1.2
                        row.prop(
                            scene_props,
                            "abort_import",
                            text="Stop Import",
                            toggle=True,
                            icon="CANCEL",
                        )

            # Prevent clicking import again
            layout.separator()
        else:
            # Regular Import Button
            row = layout.row()
            if row:
                row.scale_y = 1.5
                row.operator(
                    "linkforge.import_robot_model", text="Import Robot Model", icon="IMPORT"
                )

            # Viewport Overlays Quick Toolbar
            prefs = get_addon_prefs(context)
            box = layout.box()
            if box:
                box.label(text="Viewport Overlays", icon="OVERLAY")

                # Row 1: Visibility Toggles
                r1 = box.row(align=True)
                if r1:
                    r1.prop(
                        scene_props,
                        "show_collisions",
                        text="Collisions",
                        toggle=True,
                        icon="MOD_PHYSICS",
                    )
                    if prefs:
                        r1.prop(
                            prefs,
                            "show_joint_axes",
                            text="Joints",
                            toggle=True,
                            icon="EMPTY_ARROWS",
                        )
                        r1.prop(
                            prefs,
                            "show_inertia_gizmos",
                            text="Inertia",
                            toggle=True,
                            icon="PHYSICS",
                        )

                # Row 2: Sizing & Fitting
                r2 = box.row(align=True)
                if r2:
                    r2.operator(
                        "linkforge.auto_fit_gizmo_sizes",
                        text="Fit Gizmos to Robot",
                        icon="ARROW_LEFTRIGHT",
                    )
                    if prefs:
                        r2.prop(prefs, "joint_empty_size", text="Size")

        layout.separator()
        layout.label(text="Create robot structure:", icon="TOOL_SETTINGS")


# Registration
classes = [
    LINKFORGE_PT_forge,
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
