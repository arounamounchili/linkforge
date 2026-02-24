"""UI Panel for managing robot links."""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.types import Context, Panel

from ..adapters.blender_to_core import detect_primitive_type


class LINKFORGE_PT_links(Panel):
    """Panel for robot link properties in 3D Viewport sidebar."""

    bl_label = "Links"
    bl_idname = "LINKFORGE_PT_links"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_parent_id = "LINKFORGE_PT_forge"
    bl_order = 1

    def draw(self, context: Context) -> None:
        """Draw the panel."""
        layout = self.layout
        if not layout:
            return

        obj = context.active_object

        # Early exit if nothing selected
        if obj is None or not obj.select_get():
            # Nothing selected - offer to create an empty link
            box = layout.box()
            box.label(text="Link Creation", icon="PLUS")
            col = box.column(align=True)
            col.operator("linkforge.add_empty_link", icon="EMPTY_DATA", text="Add Empty Link Frame")

            row = col.row()
            row.enabled = False
            row.operator(
                "linkforge.create_link_from_mesh", icon="ADD", text="Create Link from Mesh"
            )
            return

        props = typing.cast(typing.Any, obj).linkforge

        # Check if selected object is a visual/collision child of a link
        # If so, show parent link properties instead
        if (
            obj
            and obj.parent
            and hasattr(obj.parent, "linkforge")
            and typing.cast(typing.Any, obj.parent).linkforge.is_robot_link
            and props
            and not props.is_robot_link
            and ("_visual" in obj.name.lower() or "_collision" in obj.name.lower())
        ):
            # Switch to parent for property display (visual/collision elements only)
            obj = obj.parent
            props = typing.cast(typing.Any, obj).linkforge

        # Check if selected object is already a link (edit mode vs create mode)
        is_link = props.is_robot_link if props else False

        # Only show Create button when NOT editing a link
        if not is_link:
            box = layout.box()
            box.label(text="Link Creation", icon="PLUS")
            col = box.column(align=True)
            col.operator(
                "linkforge.create_link_from_mesh", icon="ADD", text="Create Link from Mesh"
            )
            col.operator("linkforge.add_empty_link", icon="EMPTY_DATA", text="Add Empty Link Frame")

        # Show link properties only if a link is selected (edit mode)
        if not is_link:
            return

        # IS A LINK - Show link properties
        box = layout.box()
        visual_count = sum(1 for child in obj.children if "_visual" in child.name.lower())
        collision_count = sum(1 for child in obj.children if "_collision" in child.name.lower())
        is_virtual = visual_count == 0 and collision_count == 0

        title = f"Link: {obj.name}"
        icon = "EMPTY_DATA" if is_virtual else "LINKED"
        box.label(text=title, icon=icon)  # type: ignore[arg-type]

        if is_virtual:
            status_box = box.box()
            status_box.label(text="Status: Virtual Frame (No Geometry)", icon="INFO")

        # Link name
        box.prop(props, "link_name")

        # Geometry section
        box.separator()
        box.label(text="Geometry", icon="MESH_CUBE")

        # Visual/Collision status row with counts
        row = box.row(align=True)
        row.label(text=f"Visual: {visual_count}", icon="SHADING_RENDERED")
        row.label(text=f"Collision: {collision_count}", icon="MOD_PHYSICS")

        # Collision Configuration
        row = box.row()
        row.enabled = not is_virtual
        row.prop(props, "collision_type", text="Collision Type")

        # Geometry Detection Info
        detected_type = None
        is_primitive = False
        collision_obj = next((c for c in obj.children if "_collision" in c.name.lower()), None)

        if collision_obj:
            # 1. Check explicit URDF tag (Strongest guarantee)
            if collision_obj.get("urdf_geometry_type"):  # type: ignore[func-returns-value]
                detected_type = collision_obj["urdf_geometry_type"]
                is_primitive = detected_type in ("BOX", "CYLINDER", "SPHERE")

            # 2. Check generator tag (from generate_collision operator)
            stored_type = typing.cast(str, collision_obj.get("collision_geometry_type", "AUTO"))
            if stored_type and stored_type in ("BOX", "CYLINDER", "SPHERE"):
                detected_type = stored_type
                is_primitive = True
            elif stored_type == "MESH":
                detected_type = "MESH"
                is_primitive = False
            # 3. Fallback to heuristic detection
            elif detect_primitive_type:  # type: ignore[truthy-function]
                try:
                    heuristic_type = detect_primitive_type(collision_obj)
                    detected_type = heuristic_type if heuristic_type else "MESH"
                    is_primitive = heuristic_type is not None
                except Exception:
                    detected_type = "MESH"
            else:
                detected_type = "MESH"

        # Display detected type
        # Display detected type - ONLY if collision exists
        if collision_obj:
            row = box.row()
            icon = "INFO"
            icon = "MESH_ICOSPHERE" if is_primitive else "OUTLINER_DATA_MESH"
        if collision_obj:
            row = box.row()
            icon = "INFO"
            icon_name = "MESH_ICOSPHERE" if is_primitive else "OUTLINER_DATA_MESH"
            row.label(text=f"Detected Collision: {detected_type}", icon=icon_name)  # type: ignore[arg-type]

            is_imported = typing.cast(bool, collision_obj.get("imported_from_urdf"))

            # Show slider for meshes (only relevant for non-primitives)
            if detected_type == "MESH":
                box.separator()
                row = box.row()
                # Disable slider if imported from URDF (cannot be simplified via slider)
                row.enabled = not is_imported
                row.prop(props, "collision_quality", text="Collision Quality", slider=True)

            if is_imported:
                box.label(text="Imported collision: Geometry preserved", icon="LOCKED")

        # Collision actions (after quality setting for logical workflow)
        col = box.column(align=True)

        if collision_count == 0:
            # No collision - offer to generate
            col.operator(
                "linkforge.generate_collision", icon="MOD_PHYSICS", text="Generate Collision"
            )
        else:
            # Has collision - offer regenerate AND visibility toggle
            col.operator(
                "linkforge.generate_collision", icon="FILE_REFRESH", text="Regenerate Collision"
            )

            # Visibility toggle
            collision_obj = next((c for c in obj.children if "_collision" in c.name.lower()), None)
            if collision_obj:
                is_hidden = collision_obj.hide_viewport
            if collision_obj:
                is_hidden = collision_obj.hide_viewport
                icon_name = "HIDE_OFF" if is_hidden else "HIDE_ON"
                text = "Show Collision" if is_hidden else "Hide Collision"
                col.operator("linkforge.toggle_collision_visibility", icon=icon_name, text=text)  # type: ignore

        # Physics properties
        box.separator()
        box.label(text="Physics", icon="PHYSICS")
        box.prop(props, "mass")

        # Auto-inertia (always on by default, simplified)
        box.separator()
        row = box.row()
        row.enabled = not is_virtual  # Cannot auto-calculate without geometry
        row.prop(props, "use_auto_inertia", text="Auto-Calculate Inertia")

        if is_virtual:
            row.label(text=" (N/A for frames)", icon="ERROR")
        elif props.use_auto_inertia:
            row.label(text="", icon="CHECKMARK")
        else:
            # Manual inertia input - 2x3 compact table layout
            inertia_box = box.box()
            inertia_box.label(text="Inertia Tensor (kg⋅m²)")

            # Row 1: Diagonal elements [Ixx  Iyy  Izz]
            row = inertia_box.row(align=True)
            row.prop(props, "inertia_ixx", text="Ixx")
            row.prop(props, "inertia_iyy", text="Iyy")
            row.prop(props, "inertia_izz", text="Izz")

            # Row 2: Off-diagonal elements [Ixy  Ixz  Iyz]
            row = inertia_box.row(align=True)
            row.prop(props, "inertia_ixy", text="Ixy")
            row.prop(props, "inertia_ixz", text="Ixz")
            row.prop(props, "inertia_iyz", text="Iyz")

            # Center of Mass (Inertial Origin)
            inertia_box.separator()
            inertia_box.label(text="Center of Mass")

            # Position (XYZ)
            row = inertia_box.row(align=True)
            row.label(text="Position:", icon="EMPTY_AXIS")
            row.prop(props, "inertia_origin_xyz", text="")

            # Rotation (RPY)
            row = inertia_box.row(align=True)
            row.label(text="Rotation:", icon="ORIENTATION_GIMBAL")
            row.prop(props, "inertia_origin_rpy", text="")

        # Material section
        box.separator()
        box.label(text="Material", icon="MATERIAL")

        # Material export checkbox
        row = box.row()
        row.prop(props, "use_material", text="Export Material")

        if props.use_material:
            # Find first visual child
            visual_children = [
                child
                for child in obj.children
                if "_visual" in child.name.lower() and child.type == "MESH"
            ]

            if visual_children:
                visual_obj = visual_children[0]

                # Material selector
                if visual_obj.material_slots:
                    box.template_ID(visual_obj.material_slots[0], "material", new="material.new")

                    # Color preview
                    if visual_obj.material_slots[0].material:
                        blender_mat = visual_obj.material_slots[0].material
                        if blender_mat.use_nodes and blender_mat.node_tree:
                            for node in blender_mat.node_tree.nodes:
                                if getattr(node, "type", "") == "BSDF_PRINCIPLED":
                                    row = box.row()
                                    row.label(text="Color:")
                                    row.prop(node.inputs["Base Color"], "default_value", text="")
                                    break
                else:
                    # UX Improvement: Show Add button when no slot exists
                    # This respects "Separation of Concerns" - separate intent (checkbox) from action (add data)
                    warn_box = box.box()
                    warn_box.alert = True
                    warn_box.label(text="No material slot found", icon="INFO")
                    warn_box.operator(
                        "linkforge.add_material_slot", icon="ADD", text="Add Material Slot"
                    )
            else:
                box.label(text="No visual geometry", icon="INFO")

        # Remove Link button (Danger Zone)
        box.separator()
        box.separator()
        row = box.row()
        row.scale_y = 1.2  # Make it slightly bigger
        row.operator("linkforge.remove_link", icon="TRASH", text="Remove Link")


# Registration
classes = [
    LINKFORGE_PT_links,
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
