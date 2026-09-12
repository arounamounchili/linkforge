"""UI Panel for robot validation and export."""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.types import Context, Panel, UILayout

from ..constants import (
    PROP_ROBOT,
    PROP_VALIDATION,
)
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

        _, root_link, _, _ = build_tree_from_stats(stats)

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
            layout.separator()
            export_row = layout.row()
            if export_row:
                export_row.scale_y = 1.5
                export_row.operator(
                    "linkforge.export_robot_model", text="Export Robot Model", icon="EXPORT"
                )

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
