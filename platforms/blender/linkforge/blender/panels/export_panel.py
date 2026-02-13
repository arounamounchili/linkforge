"""UI Panel for robot validation and export."""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.types import Context, Panel, Scene, UILayout

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

    def draw(self, context: Context) -> None:
        """Draw the panel."""
        scene = context.scene
        if not scene:
            return
        layout = self.layout
        if not layout:
            return
        props = typing.cast(typing.Any, scene).linkforge

        # Count components
        num_links = sum(
            1
            for obj in scene.objects
            if hasattr(obj, "linkforge") and typing.cast(typing.Any, obj).linkforge.is_robot_link
        )

        # Only show robot properties if there are links in the scene
        if num_links == 0:
            box = layout.box()
            if box:
                box.label(text="No robot in scene", icon="INFO")
                box.label(text="Create links in Forge panel to start", icon="FORWARD")
            return

        # Build tree structure to find root
        tree, root_link, joints_dict, links_dict = build_tree_structure(scene)

        # Calculate total mass and DOF
        total_mass = 0.0
        total_dof = 0
        for obj in scene.objects:
            if (
                hasattr(obj, "linkforge")
                and typing.cast(typing.Any, obj).linkforge.is_robot_link
                and typing.cast(typing.Any, obj).linkforge.mass > 0
            ):
                total_mass += typing.cast(typing.Any, obj).linkforge.mass

            # Count DOF from actuated joints
            if (
                obj.type == "EMPTY"
                and hasattr(obj, "linkforge_joint")
                and typing.cast(typing.Any, obj).linkforge_joint.is_robot_joint
            ):
                joint_type = typing.cast(typing.Any, obj).linkforge_joint.joint_type
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
        if box:
            box.label(text="Properties", icon="ARMATURE_DATA")
            box.prop(props, "robot_name")

            # Show stats in a compact grid layout
            if scene:
                box.separator()

                # Use grid flow for compact 2-column layout
                flow = box.grid_flow(row_major=True, columns=2, even_columns=False, align=True)
                if flow:
                    # Root Link
                    if root_link:
                        flow.label(text="Root Link:")
                        flow.label(text=root_link)

                    # Total Mass
                    if total_mass > 0:
                        flow.label(text="Total Mass:")
                        flow.label(text=f"{total_mass:.1f} kg")

                    # DOF
                    flow.label(text="DOF:")
                    flow.label(text=str(total_dof))

        # === VALIDATION (Combined status and results) ===
        box = layout.box()
        if box:
            box.label(text="Robot Validation", icon="FILE_TICK")

            row = box.row(align=True)
            row.operator("linkforge.validate_robot", text="Run Validation", icon="PLAY")

            wm = context.window_manager
            validation = None
            if hasattr(wm, "linkforge_validation"):
                validation = typing.cast(typing.Any, wm).linkforge_validation

            if not validation or not validation.has_results:
                box.label(text="Not run yet", icon="INFO")
            else:
                # Status summary row
                if validation.is_valid and validation.error_count == 0:
                    summary_text = "Robot is valid"
                    if validation.warning_count > 0:
                        summary_text += f" (with {validation.warning_count} warnings)"
                    box.label(text=summary_text, icon="CHECKMARK")
                else:
                    box.label(
                        text=f"Issues: {validation.error_count} error(s), {validation.warning_count} warning(s)",
                        icon="ERROR",
                    )

                # Results section
                if validation.error_count > 0:
                    box.separator()
                    box.prop(
                        validation,
                        "show_errors",
                        toggle=True,
                        text=f"Show {validation.error_count} Error(s)",
                        icon="TRIA_DOWN" if validation.show_errors else "TRIA_RIGHT",
                    )
                    if validation.show_errors:
                        error_box = box.box()
                        for i in range(validation.error_count):
                            error = validation.get_error(i)
                            error_box.label(text=error.title, icon="CANCEL")
                            if error.message_lines:
                                for msg_line in error.message_lines:
                                    if msg_line.strip():
                                        error_box.label(text=f"  {msg_line}", icon="BLANK1")
                            if error.has_objects:
                                error_box.label(
                                    text=f"  Affected: {error.objects_str}", icon="OBJECT_DATA"
                                )
                            if error.has_suggestion:
                                for sug_line in error.suggestion_lines:
                                    if sug_line.strip():
                                        error_box.label(text=f"  → {sug_line}", icon="INFO")

                if validation.warning_count > 0:
                    box.separator()
                    box.prop(
                        validation,
                        "show_warnings",
                        toggle=True,
                        text=f"Show {validation.warning_count} Warning(s)",
                        icon="TRIA_DOWN" if validation.show_warnings else "TRIA_RIGHT",
                    )
                    if validation.show_warnings:
                        warn_box = box.box()
                        for i in range(validation.warning_count):
                            warning = validation.get_warning(i)
                            warn_box.label(text=warning.title, icon="ERROR")
                            if warning.message_lines:
                                for msg_line in warning.message_lines:
                                    if msg_line.strip():
                                        warn_box.label(text=f"  {msg_line}", icon="BLANK1")

        # === EXPORT CONFIGURATION ===
        if layout:
            layout.separator()
            export_box = layout.box()
        if export_box:
            export_box.label(text="Export Configuration", icon="EXPORT")
            export_box.prop(props, "export_format", expand=True)

            # XACRO specific settings
            if props.export_format == "XACRO":
                export_box.separator()
                row = export_box.row()
                if row:
                    row.prop(
                        props,
                        "xacro_advanced_mode",
                        icon="TRIA_DOWN" if props.xacro_advanced_mode else "TRIA_RIGHT",
                        icon_only=False,
                        emboss=False,
                    )

                if props.xacro_advanced_mode:
                    adv_box = export_box.box()
                    if adv_box:
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
        if layout:
            layout.separator()
            export_row = layout.row()
            if export_row:
                export_row.scale_y = 1.5
                export_row.operator(
                    "linkforge.export_urdf", text="Export URDF/XACRO", icon="EXPORT"
                )

            # === COMPONENT BROWSER (Quick select all components) ===
            layout.separator()

            # Collision Visibility Toggle
            row = layout.row()
            if row:
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

    def draw_component_browser(
        self,
        layout: UILayout,
        scene: Scene,
        links_dict: dict[str, typing.Any],
        num_links: int,
        num_dof: int,
    ) -> None:
        """Draw the component browser section."""
        select_box = layout.box()
        if not select_box:
            return

        # Links list
        link_header = select_box.row()
        if link_header:
            link_header.label(text=f"Links ({num_links}):", icon="MESH_CUBE")

        for link_name in sorted(links_dict.keys()):
            link_obj = links_dict[link_name]
            row = select_box.row(align=True)
            if row:
                op = row.operator(
                    "linkforge.select_tree_object", text=f"  {link_name}", emboss=False
                )
                op.object_name = link_obj.name
                op.object_type = "link"

        # Joints list
        joints = [
            obj
            for obj in scene.objects
            if obj.type == "EMPTY"
            and hasattr(obj, "linkforge_joint")
            and typing.cast(typing.Any, obj).linkforge_joint.is_robot_joint
        ]

        select_box.separator()
        joint_header = select_box.row()
        if joint_header:
            joint_header.label(text=f"Joints ({len(joints)}):", icon="EMPTY_AXIS")

        for joint_obj in sorted(joints, key=lambda x: x.name):
            row = select_box.row(align=True)
            if row:
                op = row.operator(
                    "linkforge.select_tree_object", text=f"  {joint_obj.name}", emboss=False
                )
                op.object_name = joint_obj.name
                op.object_type = "joint"

        # Sensors list
        sensors = [
            obj
            for obj in scene.objects
            if obj.type == "EMPTY"
            and hasattr(obj, "linkforge_sensor")
            and typing.cast(typing.Any, obj).linkforge_sensor.is_robot_sensor
        ]

        select_box.separator()
        sensor_header = select_box.row()
        if sensor_header:
            sensor_header.label(text=f"Sensors ({len(sensors)}):", icon="TRACKER")

        for sensor_obj in sorted(sensors, key=lambda x: x.name):
            row = select_box.row(align=True)
            if row:
                op = row.operator(
                    "linkforge.select_tree_object", text=f"  {sensor_obj.name}", emboss=False
                )
                op.object_name = sensor_obj.name
                op.object_type = "sensor"


# Registration
classes = [
    LINKFORGE_PT_export_panel,
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
