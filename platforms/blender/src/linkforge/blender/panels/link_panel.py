"""UI Panel for managing robot links."""

from __future__ import annotations

import contextlib

import bpy
from bpy.types import Context, Panel

from ..constants import (
    SUFFIX_COLLISION,
    SUFFIX_VISUAL,
)
from ..core.constants import (
    GEOM_MESH,
)
from ..properties.geom_props import PROP_GEOM
from ..utils.property_helpers import get_link_props


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

        props = get_link_props(obj)

        # Check if selected object is a visual/collision child of a link
        # If so, show parent link properties instead
        if (
            obj
            and obj.parent
            and (lp_p := get_link_props(obj.parent))
            and lp_p.is_robot_link
            and props
            and not props.is_robot_link
            and (SUFFIX_VISUAL in obj.name.lower() or SUFFIX_COLLISION in obj.name.lower())
        ):
            # Switch to parent for property display (visual/collision elements only)
            obj = obj.parent
            props = get_link_props(obj)

        # If still not a link (e.g. a loose mesh is active), check if there is a link selected
        if (not props or not props.is_robot_link) and context.selected_objects:
            for sel_obj in context.selected_objects:
                sel_props = get_link_props(sel_obj)
                if sel_props and sel_props.is_robot_link:
                    obj = sel_obj
                    props = sel_props
                    break

                # Also check if the selected object is a visual/collision child of a link
                if (
                    sel_obj.parent
                    and (lp_p := get_link_props(sel_obj.parent))
                    and lp_p.is_robot_link
                    and (
                        SUFFIX_VISUAL in sel_obj.name.lower()
                        or SUFFIX_COLLISION in sel_obj.name.lower()
                    )
                ):
                    obj = sel_obj.parent
                    props = lp_p
                    break

        # Only show Create button when NOT editing a link
        if not props or not props.is_robot_link:
            box = layout.box()
            box.label(text="Link Creation", icon="PLUS")
            col = box.column(align=True)
            col.operator(
                "linkforge.create_link_from_mesh", icon="ADD", text="Create Link from Mesh"
            )
            col.operator("linkforge.add_empty_link", icon="EMPTY_DATA", text="Add Empty Link Frame")

        # Show link properties only if a link is selected (edit mode)
        if not props or not props.is_robot_link:
            return

        # IS A LINK - Show link properties
        box = layout.box()
        visual_count = sum(1 for child in obj.children if SUFFIX_VISUAL in child.name.lower())
        collision_count = sum(1 for child in obj.children if SUFFIX_COLLISION in child.name.lower())
        is_virtual = visual_count == 0 and collision_count == 0

        title = f"Link: {props.link_name}"
        icon = "EMPTY_DATA" if is_virtual else "LINKED"
        box.label(text=title, icon=icon)  # type: ignore

        # Display Blender object name if it differs from persistent robot model name
        if obj.name != props.link_name:
            sub = box.row()
            sub.active = False  # Make it subtle
            sub.label(text=f"Blender Obj: {obj.name}", icon="INFO")

        if is_virtual:
            status_box = box.box()
            status_box.label(text="Status: Virtual Frame (No Geometry)", icon="INFO")

        # Link name
        box.prop(props, "link_name")

        # Geometry section — Visuals
        box.separator()
        box.label(text="Visuals", icon="SHADING_RENDERED")

        visual_children = [c for c in obj.children if SUFFIX_VISUAL in c.name.lower()]
        if not visual_children:
            box.label(text="No visual geometry", icon="INFO")
        else:
            list_box = box.box()
            for idx, vis in enumerate(visual_children):
                row = list_box.row(align=True)
                is_active = getattr(props, "active_visual_index", 0) == idx
                icon = "RIGHTARROW_THIN" if is_active else "BLANK1"
                row.label(text="", icon=icon)  # type: ignore
                row.label(text=vis.name, icon="SHADING_RENDERED")

                geom_props = getattr(vis, PROP_GEOM, None)
                if geom_props:
                    row.prop(geom_props, "geometry_type", text="")

        # Visual Actions
        row = box.row(align=True)
        row.operator("linkforge.assign_as_visual", text="Assign Selected", icon="ADD")
        row.operator("linkforge.remove_visual", text="", icon="REMOVE")

        # Geometry section — Collisions
        box.separator()
        box.label(text="Collisions", icon="MOD_PHYSICS")

        collision_children = [c for c in obj.children if SUFFIX_COLLISION in c.name.lower()]
        if not collision_children:
            box.label(text="No collision geometry", icon="INFO")
        else:
            list_box = box.box()
            for idx, col_obj in enumerate(collision_children):
                row = list_box.row(align=True)
                is_active = getattr(props, "active_collision_index", 0) == idx
                icon = "RIGHTARROW_THIN" if is_active else "BLANK1"
                row.label(text="", icon=icon)  # type: ignore
                row.label(text=col_obj.name, icon="MOD_PHYSICS")

                geom_props = getattr(col_obj, PROP_GEOM, None)
                if geom_props:
                    row.prop(geom_props, "geometry_type", text="")
                    if geom_props.geometry_type == GEOM_MESH:
                        row.prop(geom_props, "collision_quality", text="")

        # Collision Actions
        row = box.row(align=True)
        row.operator("linkforge.assign_as_collision", text="Assign Selected", icon="ADD")
        row.operator("linkforge.remove_collision", text="", icon="REMOVE")
        row.operator("linkforge.generate_collision", text="Auto-Generate", icon="FILE_REFRESH")

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

            # Position XYZ
            row = inertia_box.row(align=True)
            row.label(text="Position:", icon="EMPTY_AXIS")
            row.prop(props, "inertia_origin_xyz", text="")

            # Rotation RPY
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
            # Material selector
            if visual_children:
                visual_obj = visual_children[getattr(props, "active_visual_index", 0)]
                if not visual_obj or visual_obj.type != "MESH":
                    visual_obj = visual_children[0]

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
                    warn_box = box.box()
                    warn_box.alert = True
                    warn_box.label(text="No material slot found", icon="INFO")
                    warn_box.operator(
                        "linkforge.add_material_slot", icon="ADD", text="Add Material Slot"
                    )
            else:
                box.label(text="No visual geometry", icon="INFO")

        # Simulation Properties (Advanced)
        box.separator()
        box.label(text="Simulation", icon="WORLD")
        row = box.row()
        row.prop(props, "use_simulation_props", text="Advanced Simulation")

        if props.use_simulation_props:
            sim_box = box.box()
            sim_box.prop(props, "self_collide")
            sim_box.prop(props, "gravity")

            col = sim_box.column(align=True)
            col.label(text="Friction:")
            col.prop(props, "mu", text="mu (Static)")
            col.prop(props, "mu2", text="mu2 (Dynamic)")

            col = sim_box.column(align=True)
            col.label(text="Contact:")
            col.prop(props, "kp_ui", text="kp (Stiffness)")
            col.prop(props, "kd_ui", text="kd (Damping)")

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
