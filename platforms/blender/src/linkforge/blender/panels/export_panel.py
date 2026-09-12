"""UI Panel for robot validation and export."""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.types import Context, Panel, Scene, UILayout

from ..constants import (
    PROP_ROBOT,
    PROP_VALIDATION,
)
from ..core._utils.dict_utils import filter_items_by_name
from ..utils.scene_utils import (
    build_tree_from_stats,
    get_robot_statistics,
)


class LINKFORGE_PT_export_panel(Panel):
    """Validate & Export panel - robot configuration, validation, and export settings."""

    bl_label = "Validate & Export"
    bl_description = "Step 4: Validate robot structure and export to robot model file"
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
        props = getattr(scene, PROP_ROBOT)

        # Count components
        stats = get_robot_statistics(scene)
        num_links = stats.num_links

        # Only show robot properties if there are links in the scene
        if num_links == 0:
            box = layout.box()
            if box:
                box.label(text="No robot in scene", icon="INFO")
                box.label(text="Create links in Forge panel to start", icon="FORWARD")
            return

        _, root_link, _, links_dict = build_tree_from_stats(stats)

        # Get total mass and DOF from pre-calc stats
        total_mass = stats.total_mass
        total_dof = stats.total_dof

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
            if wm and hasattr(wm, PROP_VALIDATION):
                validation = getattr(wm, PROP_VALIDATION)

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
                        for i in range(validation.error_count):
                            if i > 0:
                                box.separator()
                            error = validation.get_error(i)
                            self._draw_validation_issue(box, error, is_error=True, context=context)

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
                        for i in range(validation.warning_count):
                            if i > 0:
                                box.separator()
                            warning = validation.get_warning(i)
                            self._draw_validation_issue(
                                box, warning, is_error=False, context=context
                            )

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
                    "linkforge.export_robot_model", text="Export Robot Model", icon="EXPORT"
                )

            # === COMPONENT BROWSER (Quick select all components) ===
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
                self.draw_component_browser(
                    layout, scene, links_dict, num_links, _num_dof=total_dof, stats=stats
                )

    def draw_component_browser(
        self,
        layout: UILayout,
        scene: Scene,
        links_dict: dict[str, typing.Any],
        num_links: int,
        _num_dof: int,
        stats: typing.Any,
    ) -> None:
        """Draw the component browser section with search filtering."""
        select_box = layout.box()
        if not select_box:
            return

        props = getattr(scene, PROP_ROBOT)

        # UI
        search_row = select_box.row(align=True)
        if search_row:
            search_row.prop(props, "component_browser_search", text="", icon="VIEWZOOM")

        select_box.separator()

        search_term = props.component_browser_search

        filtered_links_dict = filter_items_by_name(links_dict, search_term)

        # Links list
        link_header = select_box.row()
        if link_header:
            if search_term:
                link_header.label(
                    text=f"Links ({len(filtered_links_dict)}/{len(links_dict)}):",
                    icon="MESH_CUBE",
                )
            else:
                link_header.label(text=f"Links ({num_links}):", icon="MESH_CUBE")

        for link_name in sorted(filtered_links_dict.keys()):
            link_obj = filtered_links_dict[link_name]
            row = select_box.row(align=True)
            if row:
                op = row.operator(
                    "linkforge.select_tree_object", text=f"  {link_name}", emboss=False
                )
                op.object_name = link_obj.name
                op.object_type = "link"

        # Joints list
        joints = stats.joint_objects

        filtered_joints = filter_items_by_name(joints, search_term)

        select_box.separator()
        joint_header = select_box.row()
        if joint_header:
            if search_term:
                joint_header.label(
                    text=f"Joints ({len(filtered_joints)}/{len(joints)}):",
                    icon="EMPTY_AXIS",
                )
            else:
                joint_header.label(text=f"Joints ({len(joints)}):", icon="EMPTY_AXIS")

        for joint_obj in sorted(filtered_joints, key=lambda x: x.name):
            row = select_box.row(align=True)
            if row:
                op = row.operator(
                    "linkforge.select_tree_object", text=f"  {joint_obj.name}", emboss=False
                )
                op.object_name = joint_obj.name
                op.object_type = "joint"

        # Sensors list
        sensors = stats.sensor_objects

        filtered_sensors = filter_items_by_name(sensors, search_term)

        select_box.separator()
        sensor_header = select_box.row()
        if sensor_header:
            if search_term:
                sensor_header.label(
                    text=f"Sensors ({len(filtered_sensors)}/{len(sensors)}):",
                    icon="TRACKER",
                )
            else:
                sensor_header.label(text=f"Sensors ({len(sensors)}):", icon="TRACKER")

        for sensor_obj in sorted(filtered_sensors, key=lambda x: x.name):
            row = select_box.row(align=True)
            if row:
                op = row.operator(
                    "linkforge.select_tree_object", text=f"  {sensor_obj.name}", emboss=False
                )
                op.object_name = sensor_obj.name
                op.object_type = "sensor"

        if search_term and not filtered_links_dict and not filtered_joints and not filtered_sensors:
            select_box.separator()
            empty_row = select_box.row()
            if empty_row:
                empty_row.label(text="No matches", icon="INFO")

    @staticmethod
    def _draw_validation_issue(
        layout: UILayout,
        issue: typing.Any,
        is_error: bool,
        context: Context,
    ) -> None:
        """Draw an individual validation issue as a distinct, actionable card."""
        card = layout.box()

        # Header row: Title and status icon
        header = card.row(align=True)
        if is_error:
            header.label(text=issue.title, icon="CANCEL")
        else:
            header.label(text=issue.title, icon="ERROR")

        # Message details
        if issue.message_lines:
            for msg_line in issue.message_lines:
                if msg_line.strip():
                    card.label(text=f"  {msg_line}", icon="BLANK1")

        # Affected objects and 1-click select operator
        if issue.has_objects:
            scene = context.scene
            scene_objects = scene.objects if scene else None
            affected_list = getattr(issue, "affected_object_list", [])

            # Single affected object with scene match -> render inline on same row
            if scene_objects and len(affected_list) == 1 and affected_list[0] in scene_objects:
                obj_name = affected_list[0]
                row = card.row(align=True)
                row.label(text=f"  Affected: {obj_name}", icon="OBJECT_DATA")
                op = row.operator(
                    "linkforge.select_tree_object",
                    text="Select",
                    icon="RESTRICT_SELECT_OFF",
                )
                op.object_name = obj_name
            else:
                card.label(text=f"  Affected: {issue.objects_str}", icon="OBJECT_DATA")
                if scene_objects:
                    for obj_name in affected_list:
                        if obj_name in scene_objects:
                            btn_row = card.row(align=True)
                            op = btn_row.operator(
                                "linkforge.select_tree_object",
                                text=f"Select '{obj_name}'",
                                icon="RESTRICT_SELECT_OFF",
                            )
                            op.object_name = obj_name

        # Actionable suggestion with single icon (avoid repeating icon on wrapped lines)
        if issue.has_suggestion:
            for idx, sug_line in enumerate(issue.suggestion_lines):
                if sug_line.strip():
                    if idx == 0:
                        card.label(text=f"  → {sug_line}", icon="INFO")
                    else:
                        card.label(text=f"     {sug_line}", icon="BLANK1")


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
