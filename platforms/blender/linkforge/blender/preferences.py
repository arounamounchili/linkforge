"""Blender addon preferences for LinkForge.

User preferences for controlling visualization and behavior.
"""

from __future__ import annotations

import contextlib

import bpy
from bpy.props import BoolProperty, FloatProperty
from bpy.types import AddonPreferences, Context


def update_joint_axes_visibility(self: LinkForgePreferences, context: Context) -> None:
    """Callback when show_joint_axes changes - manage draw handler and force viewport redraw."""
    from .visualization import joint_gizmos

    joint_gizmos.update_viz_handle(context)


def update_joint_empty_size(self: LinkForgePreferences, context: Context) -> None:
    """Callback when joint_empty_size changes - update all joint empties and viewport."""
    # From here, we also need to trigger the draw handler update check
    # so the GPU overlay picks up the new size immediately
    from .visualization import joint_gizmos

    joint_gizmos.update_viz_handle(context)

    # Update all existing joint empties in the scene
    for obj in context.scene.objects:
        if (
            obj.type == "EMPTY"
            and hasattr(obj, "linkforge_joint")
            and obj.linkforge_joint.is_robot_joint
        ):
            obj.empty_display_size = self.joint_empty_size

    # Force viewport redraw
    for window in context.window_manager.windows:
        for area in window.screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()


def update_sensor_empty_size(self: LinkForgePreferences, context: Context) -> None:
    """Callback when sensor_empty_size changes - update all sensor empties."""

    # Get new size
    new_size = self.sensor_empty_size

    # Update all existing sensor empties in the scene
    for obj in context.scene.objects:
        if (
            obj.type == "EMPTY"
            and hasattr(obj, "linkforge_sensor")
            and obj.linkforge_sensor.is_robot_sensor
        ):
            obj.empty_display_size = new_size

    # Force viewport redraw
    for window in context.window_manager.windows:
        for area in window.screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()


def update_link_empty_size(self: LinkForgePreferences, context: Context) -> None:
    """Callback when link_empty_size changes - update all link empties."""

    # Get new size
    new_size = self.link_empty_size

    # Update all existing link empties in the scene
    for obj in context.scene.objects:
        if obj.type == "EMPTY" and hasattr(obj, "linkforge") and obj.linkforge.is_robot_link:
            obj.empty_display_size = new_size

    # Force viewport redraw
    for window in context.window_manager.windows:
        for area in window.screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()


def update_inertia_visibility(self: LinkForgePreferences, context: Context) -> None:
    """Callback when show_inertia_gizmos changes."""
    from .visualization import inertia_gizmos

    inertia_gizmos.tag_redraw()
    # If the user just enabled it, make sure the handler is registered
    if self.show_inertia_gizmos:
        inertia_gizmos.ensure_inertia_handler()


def update_inertia_size(self: LinkForgePreferences, context: Context) -> None:
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
    return pkg.split(".")[0] if pkg else "linkforge"


def get_addon_prefs(context: Context | None = None) -> LinkForgePreferences | None:
    """Retrieve the LinkForge preferences object reliably."""
    if context is None:
        context = bpy.context
    addon_id = get_addon_id()
    addon = context.preferences.addons.get(addon_id)
    if addon:
        return addon.preferences
    return None


class LinkForgePreferences(AddonPreferences):
    """User preferences for LinkForge extension."""

    bl_idname = get_addon_id()

    # Joint axis visualization (GPU overlay - optional enhancement)
    show_joint_axes: BoolProperty(  # type: ignore
        name="Show GPU Overlay Axes",
        description="Show thick RGB arrows at each joint (Red=X, Green=Y, Blue=Z) like RViz visualization",
        default=True,
        update=update_joint_axes_visibility,
    )

    joint_empty_size: FloatProperty(  # type: ignore
        name="Joint Display Size",
        description="Size of the joint markers and GPU axes in viewport",
        default=0.2,
        min=0.01,
        max=100.0,
        soft_min=0.05,
        soft_max=5.0,
        step=1,
        precision=2,
        unit="LENGTH",
        update=update_joint_empty_size,  # Update all joint empties and GPU overlay
    )

    sensor_empty_size: FloatProperty(  # type: ignore
        name="Sensor Empty Size",
        description="Size of the sensor markers in viewport (bigger = easier to select, smaller = cleaner view)",
        default=0.1,
        min=0.01,
        max=100.0,
        soft_min=0.05,
        soft_max=5.0,
        step=1,
        precision=2,
        unit="LENGTH",
        update=update_sensor_empty_size,  # Update all sensor empties when changed
    )

    link_empty_size: FloatProperty(  # type: ignore
        name="Link Empty Size",
        description="Size of the link markers in viewport (bigger = easier to select, smaller = cleaner view)",
        default=0.1,
        min=0.01,
        max=100.0,
        soft_min=0.05,
        soft_max=5.0,
        step=1,
        precision=2,
        unit="LENGTH",
        update=update_link_empty_size,  # Update all link empties when changed
    )

    # Inertia Visualization
    show_inertia_gizmos: BoolProperty(  # type: ignore
        name="Show Inertia Frames",
        description="Globally toggle visualization for links with manual inertia (CoM Sphere and Principal Axes)",
        default=True,
        update=update_inertia_visibility,
    )

    inertia_gizmo_size: FloatProperty(  # type: ignore
        name="Inertia Frame Size",
        description="Standard display size for CoM spheres and principal axes",
        default=0.1,
        min=0.01,
        max=100.0,
        soft_min=0.05,
        soft_max=5.0,
        step=1,
        precision=2,
        unit="LENGTH",
        update=update_inertia_size,
    )

    def draw(self, context: Context) -> None:
        """Draw the preferences UI."""
        layout = self.layout

        # Joint visualization
        box = layout.box()
        box.label(text="Joint Visualization", icon="EMPTY_ARROWS")

        # Joint sizing (controls both Empty arrows and GPU overlay)
        row = box.row()
        row.prop(self, "joint_empty_size", text="Joint Size", slider=True)

        col = box.column(align=True)
        col.scale_y = 0.7
        col.label(text="Controls size of both joint markers and enhanced visualization")

        # GPU overlay (optional enhancement)
        box.separator()
        row = box.row()
        row.prop(self, "show_joint_axes", text="Enhanced Visualization (RViz-style)")

        if self.show_joint_axes:
            col = box.column(align=True)
            col.scale_y = 0.7
            col.label(text="High-visibility thick axes with arrow cones", icon="INFO")

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
            row = box.row()
            row.prop(self, "inertia_gizmo_size", text="Frame Size", slider=True)

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
