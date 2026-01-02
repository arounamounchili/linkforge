"""UI Panel for managing robot links."""

from __future__ import annotations

import bpy
from bpy.types import Panel

from ..utils.converters import detect_primitive_type


class LINKFORGE_PT_links(Panel):
    """Panel for robot link properties in 3D Viewport sidebar."""

    bl_label = "Links"
    bl_idname = "LINKFORGE_PT_links"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_parent_id = "LINKFORGE_PT_build"
    bl_order = 1

    def draw(self, context):
        """Draw the panel."""
        layout = self.layout
        obj = context.active_object

        # Early exit if nothing selected
        if obj is None or not obj.select_get():
            # Still show Create button when nothing selected
            box = layout.box()
            row = box.row()
            row.enabled = False
            row.operator("linkforge.create_link_from_mesh", icon="ADD", text="Create Link")
            return

        props = obj.linkforge

        # Check if selected object is a visual/collision child of a link
        # If so, show parent link properties instead
        if (
            obj.parent
            and hasattr(obj.parent, "linkforge")
            and obj.parent.linkforge.is_robot_link
            and not obj.linkforge.is_robot_link
            and ("_visual" in obj.name.lower() or "_collision" in obj.name.lower())
        ):
            # Switch to parent for property display (visual/collision elements only)
            obj = obj.parent
            props = obj.linkforge

        # Check if selected object is already a link (edit mode vs create mode)
        is_link = props.is_robot_link

        # Only show Create button when NOT editing a link
        if not is_link:
            box = layout.box()
            row = box.row()
            # Button enabled only if operator poll returns True (mesh selected, not already a link)
            row.operator("linkforge.create_link_from_mesh", icon="ADD", text="Create Link")

        # Show link properties only if a link is selected (edit mode)
        if not is_link:
            return

        # IS A LINK - Show link properties
        box = layout.box()
        box.label(text=f"Link: {obj.name}", icon="LINKED")

        # Link name
        box.prop(props, "link_name")

        # Geometry section
        box.separator()
        box.label(text="Geometry", icon="MESH_CUBE")

        # Show child mesh count
        visual_count = sum(1 for child in obj.children if "_visual" in child.name.lower())
        collision_count = sum(1 for child in obj.children if "_collision" in child.name.lower())

        # Visual/Collision status row with counts
        row = box.row(align=True)
        row.label(text=f"Visual: {visual_count}", icon="SHADING_RENDERED")
        row.label(text=f"Collision: {collision_count}", icon="MOD_PHYSICS")

        # Collision Type (Always visible)
        row = box.row()
        row.prop(props, "collision_type", text="Collision Type")

        # Collision quality slider (show before actions for better workflow)
        # User sets quality FIRST, then generates/manages collision
        if collision_count > 0:
            # Check if collision is a primitive (no quality slider needed)
            is_primitive = False

            collision_obj = next((c for c in obj.children if "_collision" in c.name.lower()), None)

            # Resolve actual geometry type for UX feedback
            detected_type = "UNKNOWN"
            is_primitive = False

            if collision_obj:
                # 1. Check explicit URDF tag (Strongest guarantee)
                if "urdf_geometry_type" in collision_obj:
                    detected_type = collision_obj["urdf_geometry_type"]
                    is_primitive = detected_type in ("BOX", "CYLINDER", "SPHERE")

                # 2. Check generator tag (from generate_collision operator)
                elif collision_obj.get("collision_geometry_type", "AUTO") in (
                    "BOX",
                    "CYLINDER",
                    "SPHERE",
                ):
                    detected_type = collision_obj["collision_geometry_type"]
                    is_primitive = True

                elif collision_obj.get("collision_geometry_type", "AUTO") == "CONVEX_HULL":
                    detected_type = "CONVEX_HULL"
                    is_primitive = False

                # 3. Fallback to heuristic detection
                elif detect_primitive_type is not None:
                    try:
                        heuristic_type = detect_primitive_type(collision_obj)
                        if heuristic_type:
                            detected_type = heuristic_type
                            is_primitive = True
                        else:
                            detected_type = "MESH"
                    except Exception:
                        detected_type = "MESH"
                else:
                    detected_type = "MESH"

                # Display detected type
                row = box.row()
                if is_primitive:
                    row.label(text=f"Detected: {detected_type}", icon="MESH_ICOSPHERE")
                else:
                    row.label(text=f"Detected: {detected_type}", icon="OUTLINER_DATA_MESH")

            if detected_type == "CONVEX_HULL":
                # Show slider for meshes (only relevant for non-primitives)
                box.separator()
                row = box.row()
                row.prop(props, "collision_quality", text="Collision Quality", slider=True)

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
                icon = "HIDE_OFF" if is_hidden else "HIDE_ON"
                text = "Show Collision" if is_hidden else "Hide Collision"
                col.operator("linkforge.toggle_collision_visibility", icon=icon, text=text)

        # Physics properties
        box.separator()
        box.label(text="Physics", icon="PHYSICS")
        box.prop(props, "mass")

        # Auto-inertia (always on by default, simplified)
        box.separator()
        row = box.row()
        row.prop(props, "use_auto_inertia", text="Auto-Calculate Inertia")
        if props.use_auto_inertia:
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
                                if node.type == "BSDF_PRINCIPLED":
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


def register():
    """Register panel."""
    for cls in classes:
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
