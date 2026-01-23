"""UI Panel for robot validation and export."""

from __future__ import annotations

import bpy
from bpy.types import Context, Panel

from .robot_panel import build_tree_structure


class LINKFORGE_PT_export_panel(Panel):
    """Validate & Export panel - robot configuration, validation, and export settings."""

    bl_label = "Validate & Export"
    bl_description = "Step 4: Validate robot structure and export to URDF/XACRO"
    bl_idname = "LINKFORGE_PT_export_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 3
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context: Context):
        """Draw the panel."""
        layout = self.layout
        scene = context.scene
        props = scene.linkforge

        # Count components
        num_links = sum(1 for obj in scene.objects if obj.linkforge.is_robot_link)

        # Only show robot properties if there are links in the scene
        if num_links == 0:
            box = layout.box()
            box.label(text="No robot in scene", icon="INFO")
            box.label(text="Create links in Forge panel to start", icon="FORWARD")
            return

        # Build tree structure to find root
        tree, root_link, joints_dict, links_dict = build_tree_structure(scene)

        # Calculate total mass and DOF
        total_mass = 0.0
        total_dof = 0
        for obj in scene.objects:
            if obj.linkforge.is_robot_link and obj.linkforge.mass > 0:
                total_mass += obj.linkforge.mass

            # Count DOF from actuated joints
            if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint:
                joint_type = obj.linkforge_joint.joint_type
                # Map joint types to DOF contribution
                dof_map = {
                    "FIXED": 0,
                    "REVOLUTE": 1,
                    "CONTINUOUS": 1,
                    "PRISMATIC": 1,
                    "PLANAR": 2,
                    "FLOATING": 6,
                }
                total_dof += dof_map.get(joint_type, 0)

        # === ROBOT PROPERTIES (with essential stats) ===
        layout.separator()
        box = layout.box()
        box.label(text="Properties", icon="ARMATURE_DATA")
        box.prop(props, "robot_name")

        # Show stats in a compact grid layout
        if root_link or total_mass > 0 or total_dof > 0:
            box.separator()

            # Use grid flow for compact 2-column layout
            flow = box.grid_flow(row_major=True, columns=2, even_columns=False, align=True)

            # Root Link
            if root_link:
                flow.label(text="Root Link:")
                flow.label(text=root_link)

            # Total Mass
            if total_mass > 0:
                flow.label(text="Total Mass:")
                flow.label(text=f"{total_mass:.1f} kg")

            # DOF
            if total_dof > 0:
                flow.label(text="DOF:")
                flow.label(text=str(total_dof))

        # === VALIDATION (combined button + results) ===
        layout.separator()
        val_section = layout.box()
        val_section.label(text="Validation", icon="CHECKMARK")

        # Validate button
        row = val_section.row()
        row.scale_y = 1.3
        row.operator("linkforge.validate_robot", text="Validate Robot", icon="CHECKMARK")

        # Show validation results inline (inside same box)
        wm = context.window_manager
        if hasattr(wm, "linkforge_validation"):
            validation = wm.linkforge_validation
            if validation.has_results:
                val_section.separator()

                if validation.is_valid and validation.warning_count == 0:
                    val_section.label(text="Robot is valid!", icon="CHECKMARK")
                elif validation.is_valid:
                    val_section.label(
                        text=f"Valid with {validation.warning_count} warning(s)", icon="ERROR"
                    )
                    # Show warnings inline
                    if validation.warning_count > 0:
                        val_section.prop(
                            validation,
                            "show_warnings",
                            toggle=True,
                            text=f"Show {validation.warning_count} Warning(s)",
                            icon="TRIA_DOWN" if validation.show_warnings else "TRIA_RIGHT",
                        )
                        if validation.show_warnings:
                            for i in range(validation.warning_count):
                                warning = validation.get_warning(i)
                                warn_row = val_section.row()
                                warn_row.label(text=f"  • {warning.title}", icon="ERROR")
                                # Show all message lines (not just first)
                                if warning.message_lines:
                                    for msg_line in warning.message_lines:
                                        if msg_line.strip():  # Skip empty lines
                                            msg_row = val_section.row()
                                            msg_row.label(text=f"    {msg_line}", icon="BLANK1")
                else:
                    val_section.label(
                        text=f"{validation.error_count} error(s), {validation.warning_count} warning(s)",
                        icon="CANCEL",
                    )
                    # Show errors inline
                    if validation.error_count > 0:
                        val_section.prop(
                            validation,
                            "show_errors",
                            toggle=True,
                            text=f"Show {validation.error_count} Error(s)",
                            icon="TRIA_DOWN" if validation.show_errors else "TRIA_RIGHT",
                        )
                        if validation.show_errors:
                            for i in range(validation.error_count):
                                error = validation.get_error(i)
                                err_row = val_section.row()
                                err_row.label(text=f"  • {error.title}", icon="CANCEL")
                                # Show all message lines (not just first)
                                if error.message_lines:
                                    for msg_line in error.message_lines:
                                        if msg_line.strip():  # Skip empty lines
                                            msg_row = val_section.row()
                                            msg_row.label(text=f"    {msg_line}", icon="BLANK1")
                                if error.has_objects:
                                    obj_row = val_section.row()
                                    obj_row.label(
                                        text=f"    Affected: {error.objects_str}",
                                        icon="OBJECT_DATA",
                                    )
                                # Show all suggestion lines (not just first)
                                if error.has_suggestion:
                                    for sug_line in error.suggestion_lines:
                                        if sug_line.strip():  # Skip empty lines
                                            sug_row = val_section.row()
                                            sug_row.label(
                                                text=f"    → Fix: {sug_line}", icon="INFO"
                                            )

        # === EXPORT CONFIGURATION ===
        layout.separator()
        export_box = layout.box()
        export_box.label(text="Export Configuration", icon="EXPORT")
        export_box.prop(props, "export_format", expand=True)

        # XACRO specific settings
        if props.export_format == "XACRO":
            export_box.separator()
            row = export_box.row()
            row.prop(
                props,
                "xacro_advanced_mode",
                icon="TRIA_DOWN" if props.xacro_advanced_mode else "TRIA_RIGHT",
                icon_only=False,
                emboss=False,
            )

            if props.xacro_advanced_mode:
                adv_box = export_box.box()
                adv_box.prop(props, "xacro_extract_materials")
                adv_box.prop(props, "xacro_extract_dimensions")
                adv_box.prop(props, "xacro_generate_macros")
                adv_box.prop(props, "xacro_split_files")

        # Mesh export options
        export_box.separator()
        export_box.prop(props, "export_meshes")
        if props.export_meshes:
            export_box.prop(props, "mesh_format")
            export_box.prop(props, "mesh_directory_name")

        # Validation option
        export_box.separator()
        export_box.prop(props, "validate_before_export")

        # === EXPORT BUTTON ===
        layout.separator()
        export_row = layout.row()
        export_row.scale_y = 1.5
        export_row.operator("linkforge.export_urdf", text="Export URDF/XACRO", icon="EXPORT")

        # === COMPONENT BROWSER (Quick select all components) ===
        layout.separator()

        # Collision Visibility Toggle
        row = layout.row()
        row.prop(props, "show_collisions", toggle=True, icon="SHADING_WIRE")

        layout.separator()
        layout.prop(
            props,
            "show_kinematic_tree",
            toggle=True,
            text="Component Browser",
            icon="VIEWZOOM",
            emboss=True,
        )

        if props.show_kinematic_tree:
            self.draw_component_browser(layout, scene, links_dict, num_links, num_dof=total_dof)

    def draw_component_browser(self, layout, scene, links_dict, num_links, num_dof):
        """Draw the component browser section."""
        select_box = layout.box()

        # Links list
        link_header = select_box.row()
        link_header.label(text=f"Links ({num_links}):", icon="MESH_CUBE")

        for link_name in sorted(links_dict.keys()):
            link_obj = links_dict[link_name]
            row = select_box.row(align=True)
            op = row.operator("linkforge.select_tree_object", text=f"  {link_name}", emboss=False)
            op.object_name = link_obj.name
            op.object_type = "link"

        # Joints list
        joints = [
            obj
            for obj in scene.objects
            if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint
        ]
        if joints:
            select_box.separator()
            joint_header = select_box.row()
            joint_header.label(text=f"Joints ({len(joints)}):", icon="EMPTY_AXIS")

            all_joints = []
            for obj in joints:
                all_joints.append(
                    (obj.linkforge_joint.joint_name, obj.linkforge_joint.joint_type, obj)
                )

            for joint_name, joint_type, joint_obj in sorted(all_joints):
                row = select_box.row(align=True)
                op = row.operator(
                    "linkforge.select_tree_object",
                    text=f"  {joint_name} ({joint_type.lower()})",
                    emboss=False,
                )
                op.object_name = joint_obj.name
                op.object_type = "joint"

        # Sensors and Transmissions can be added here if needed, keeping it simpler for now
        # Reusing the logic from robot_panel but streamlined


# Registration
classes = [
    LINKFORGE_PT_export_panel,
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
