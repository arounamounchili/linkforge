"""Blender addon preferences for LinkForge.

User preferences for controlling visualization and behavior.
"""

from __future__ import annotations

import contextlib
from typing import Any

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, StringProperty
from bpy.types import AddonPreferences, Context

from .constants import (
    ADDON_ID_DEFAULT,
    DEFAULT_INERTIA_GIZMO_SIZE,
    DEFAULT_JOINT_GIZMO_SIZE,
    DEFAULT_LINK_GIZMO_SIZE,
    DEFAULT_SENSOR_GIZMO_SIZE,
)
from .utils.property_helpers import get_joint_props, get_link_props, get_sensor_props


def update_joint_axes_visibility(self: LinkForgePreferences | None, context: Context) -> None:
    """Callback when show_joint_axes changes - manage draw handler and empty visibility."""
    from .utils.scene_utils import compute_anchor_size
    from .visualization import joint_gizmos

    joint_gizmos.update_viz_handle(context)

    # Sync viewport visibility and anchor sizes of joint empties
    if self and context and context.scene:
        show = getattr(self, "show_joint_axes", True) is True
        joint_size = getattr(self, "joint_empty_size", DEFAULT_JOINT_GIZMO_SIZE)
        anchor_size = compute_anchor_size(joint_size, show)
        for obj in context.scene.objects:
            if (
                obj.type == "EMPTY"
                and (jp := get_joint_props(obj))
                and getattr(jp, "is_robot_joint", False)
            ):
                obj.hide_viewport = not show
                obj.empty_display_size = anchor_size

    _tag_all_3d_viewports_redraw(context)


def _tag_all_3d_viewports_redraw(context: Context) -> None:
    """Force redraw of all 3D Viewport areas across all windows."""
    if context.window_manager:
        for window in context.window_manager.windows:
            for area in window.screen.areas:
                if area.type == "VIEW_3D":
                    area.tag_redraw()


def _update_empties_display_size(
    context: Context,
    new_size: float,
    prop_getter: Any,
    flag_attr: str,
) -> None:
    """Update empty_display_size for matching empty objects in the scene."""
    if context.scene:
        for obj in context.scene.objects:
            if (
                obj.type == "EMPTY"
                and (props := prop_getter(obj))
                and getattr(props, flag_attr, False)
            ):
                obj.empty_display_size = new_size
    _tag_all_3d_viewports_redraw(context)


def update_joint_empty_size(self: LinkForgePreferences, context: Context) -> None:
    """Callback when joint_empty_size changes - update all joint empties and viewport."""
    from .utils.scene_utils import compute_anchor_size
    from .visualization import joint_gizmos

    joint_gizmos.update_viz_handle(context)
    show_gpu = getattr(self, "show_joint_axes", False) is True
    anchor_size = compute_anchor_size(self.joint_empty_size, show_gpu)
    _update_empties_display_size(context, anchor_size, get_joint_props, "is_robot_joint")


def update_sensor_empty_size(self: LinkForgePreferences, context: Context) -> None:
    """Callback when sensor_empty_size changes - update all sensor empties."""
    _update_empties_display_size(
        context, self.sensor_empty_size, get_sensor_props, "is_robot_sensor"
    )


def update_link_empty_size(self: LinkForgePreferences, context: Context) -> None:
    """Callback when link_empty_size changes - update all link empties."""
    _update_empties_display_size(context, self.link_empty_size, get_link_props, "is_robot_link")


def update_inertia_visibility(_self: LinkForgePreferences, _context: Context) -> None:
    """Callback when show_inertia_gizmos changes."""
    from .visualization import inertia_gizmos

    inertia_gizmos.tag_redraw()
    # If the user just enabled it, make sure the handler is registered
    if _self.show_inertia_gizmos:
        inertia_gizmos.ensure_inertia_handler()


def update_inertia_size(_self: LinkForgePreferences, _context: Context) -> None:
    """Callback when inertia_gizmo_size changes."""
    from .visualization import inertia_gizmos

    inertia_gizmos.tag_redraw()


def get_addon_id() -> str:
    """Determine the addon/extension ID for preference access.

    In Blender 4.2+, extensions use a namespace like 'bl_ext.user_default.linkforge'.
    Traditional addons use just the package name 'linkforge'.
    """
    pkg = __package__
    if pkg and pkg.startswith("bl_ext."):
        # Extension path: bl_ext.<repo>.<id>
        return ".".join(pkg.split(".")[:3])
    return pkg.split(".")[0] if pkg else ADDON_ID_DEFAULT


def get_addon_prefs(context: Context | None = None) -> LinkForgePreferences | None:
    """Retrieve the LinkForge preferences object reliably."""
    if context is None:
        context = bpy.context
    addon_id = get_addon_id()
    if context.preferences:
        addon = context.preferences.addons.get(addon_id)
        if addon:
            import typing

            return typing.cast("LinkForgePreferences", addon.preferences)
    return None


class LinkForgePreferences(AddonPreferences):
    """User preferences for LinkForge extension."""

    bl_idname = get_addon_id()

    # Joint axis visualization (Empty coordinate frames and GPU RViz overlay)
    show_joint_axes: BoolProperty(  # type: ignore
        name="Show Joint Axes",
        description="Show or hide all robot joint axes and coordinate frames in the 3D viewport",
        default=True,
        update=update_joint_axes_visibility,
    )

    joint_axes_display_mode: EnumProperty(  # type: ignore
        name="Joint Axes Mode",
        description="Control which joints display RViz-style RGB axes",
        items=[
            ("ALL", "All Joints", "Show RGB axes for all joints in the scene"),
            (
                "SELECTED_ONLY",
                "Selected Joint Only",
                "Only show RGB axes for the active or selected joint",
            ),
        ],
        default="ALL",
        update=update_joint_axes_visibility,
    )

    joint_axes_depth_mode: EnumProperty(  # type: ignore
        name="Joint Depth Mode",
        description="Depth testing mode for joint axes in 3D viewport",
        items=[
            (
                "ALWAYS",
                "Always On Top (X-Ray)",
                "Draw joint axes over geometry for maximum visibility",
            ),
            (
                "OCCLUDED",
                "Occluded by Geometry",
                "Depth test axes so geometry naturally occludes them",
            ),
        ],
        default="ALWAYS",
        update=update_joint_axes_visibility,
    )

    joint_empty_size: FloatProperty(  # type: ignore
        name="Joint Display Size",
        description="Size of the joint markers and GPU axes in viewport",
        default=DEFAULT_JOINT_GIZMO_SIZE,
        min=0.001,
        max=100.0,
        soft_min=0.01,
        soft_max=5.0,
        step=1,
        precision=2,
        unit="LENGTH",
        update=update_joint_empty_size,  # Update all joint empties and GPU overlay
    )

    sensor_empty_size: FloatProperty(  # type: ignore
        name="Sensor Empty Size",
        description="Size of the sensor markers in viewport (bigger = easier to select, smaller = cleaner view)",
        default=DEFAULT_SENSOR_GIZMO_SIZE,
        min=0.001,
        max=100.0,
        soft_min=0.01,
        soft_max=5.0,
        step=1,
        precision=2,
        unit="LENGTH",
        update=update_sensor_empty_size,  # Update all sensor empties when changed
    )

    link_empty_size: FloatProperty(  # type: ignore
        name="Link Empty Size",
        description="Size of the link markers in viewport (bigger = easier to select, smaller = cleaner view)",
        default=DEFAULT_LINK_GIZMO_SIZE,
        min=0.001,
        max=100.0,
        soft_min=0.01,
        soft_max=5.0,
        step=1,
        precision=2,
        unit="LENGTH",
        update=update_link_empty_size,  # Update all link empties when changed
    )

    # Inertia Visualization
    show_inertia_gizmos: BoolProperty(  # type: ignore
        name="Show Inertia Frames",
        description="Show or hide Center of Mass indicators and principal inertia frames in the 3D viewport",
        default=False,
        update=update_inertia_visibility,
    )

    inertia_display_mode: EnumProperty(  # type: ignore
        name="Inertia Display Mode",
        description="Control which links display Center of Mass and inertia frames",
        items=[
            (
                "SELECTED_ONLY",
                "Selected Link Only",
                "Only show inertia gizmo for the currently selected link",
            ),
            ("ALL", "All Links", "Show inertia gizmos for all links with manual inertia"),
        ],
        default="SELECTED_ONLY",
        update=update_inertia_visibility,
    )

    inertia_gizmo_size: FloatProperty(  # type: ignore
        name="Inertia Frame Size",
        description="Standard display size for CoM spheres and principal axes",
        default=DEFAULT_INERTIA_GIZMO_SIZE,
        min=0.001,
        max=100.0,
        soft_min=0.01,
        soft_max=5.0,
        step=1,
        precision=2,
        unit="LENGTH",
        update=update_inertia_size,
    )

    # File and Environment Paths
    additional_search_paths: StringProperty(  # type: ignore
        name="Additional ROS Package Paths",
        description="Comma (or OS path separator) separated fallback paths for package:// meshes (useful for Snap/Flatpak)",
        default="",
    )

    def draw(self, _context: Context) -> None:
        """Draw the preferences UI."""
        layout = self.layout

        # Joint visualization
        box = layout.box()
        box.label(text="Joint Visualization", icon="EMPTY_ARROWS")

        row = box.row()
        row.prop(self, "show_joint_axes", text="Show Joint Axes")

        if self.show_joint_axes:
            col = box.column(align=True)
            col.scale_y = 0.7
            col.label(
                text="High-visibility RViz-style axes with directional arrow cones (Red=X, Green=Y, Blue=Z)",
                icon="INFO",
            )

            sub_row = box.row()
            sub_row.prop(self, "joint_axes_display_mode", expand=True)

            sub_row = box.row()
            sub_row.prop(self, "joint_axes_depth_mode", text="Depth Occlusion")

            row = box.row()
            row.prop(self, "joint_empty_size", text="Axis Length", slider=True)

        # Sensor visualization
        layout.separator()
        box = layout.box()
        box.label(text="Sensor Visualization", icon="LIGHT_SUN")
        row = box.row()
        row.prop(self, "sensor_empty_size", text="Sensor Empty Size", slider=True)

        # Link visualization
        layout.separator()
        box = layout.box()
        box.label(text="Link Visualization", icon="LINKED")
        row = box.row()
        row.prop(self, "link_empty_size", text="Link Empty Size", slider=True)

        # Inertia visualization
        layout.separator()
        box = layout.box()
        box.label(text="Inertia Visualization", icon="PHYSICS")
        row = box.row()
        row.prop(self, "show_inertia_gizmos", text="Show Inertia Frames")

        if self.show_inertia_gizmos:
            sub_row = box.row()
            sub_row.prop(self, "inertia_display_mode", expand=True)

            row = box.row()
            row.prop(self, "inertia_gizmo_size", text="Frame Size", slider=True)

        # Environment & Paths
        layout.separator()
        box = layout.box()
        box.label(text="Environment & Paths", icon="FILE_FOLDER")
        row = box.row()
        row.prop(self, "additional_search_paths")
        col = box.column(align=True)
        col.scale_y = 0.7
        col.label(
            text="Fallback ROS workspace paths for package:// resolution (comma or OS-path separated)",
            icon="INFO",
        )

        # General help text
        layout.separator()
        col = layout.column(align=True)
        col.scale_y = 0.7
        col.label(text="Tip: Larger empties are easier to click in viewport", icon="HAND")
        col.label(text="Use Outliner to select components if empties are too small")


# Registration
classes = [
    LinkForgePreferences,
]


def register() -> None:
    """Register preferences."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister preferences."""
    for cls in reversed(classes):
        with contextlib.suppress(RuntimeError):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
