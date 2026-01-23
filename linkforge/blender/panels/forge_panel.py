"""Main UI Panel for LinkForge.

This module defines the parent panel 'LINKFORGE_PT_forge' that other panels attach to.
It must be registered BEFORE any child panels.
"""

from __future__ import annotations

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

    def draw(self, context: Context):
        """Draw the panel."""
        layout = self.layout

        # Import URDF/XACRO
        row = layout.row()
        row.scale_y = 1.5
        row.operator("linkforge.import_urdf", text="Import URDF/XACRO", icon="IMPORT")

        layout.separator()
        layout.label(text="Create robot structure:", icon="TOOL_SETTINGS")


# Registration
classes = [
    LINKFORGE_PT_forge,
]


def register():
    """Register panel."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister():
    """Unregister panel."""
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except RuntimeError:
            pass


if __name__ == "__main__":
    register()
