"""Main UI Panel for LinkForge.

This module defines the parent panel 'LINKFORGE_PT_forge' that other panels attach to.
It must be registered BEFORE any child panels.
"""

from __future__ import annotations

import contextlib

import bpy
from bpy.types import Context, Panel


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
        scene_props = getattr(context.scene, "linkforge")

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
                row.operator("linkforge.import_urdf", text="Import URDF/XACRO", icon="IMPORT")

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
