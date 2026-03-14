"""UI Panel for managing robot sensors."""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.types import Context, Panel, UILayout


class LINKFORGE_PT_perceive(Panel):
    """Panel for adding sensors to perceive the environment."""

    bl_label = "Perceive"
    bl_description = "Step 2: Add sensors for perception (cameras, LiDAR, IMU, GPS)"
    bl_idname = "LINKFORGE_PT_perceive"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 1
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context: Context) -> None:
        """Draw the panel.

        Args:
            context: The current Blender context.
        """
        layout = self.layout
        if not layout:
            return
        obj = context.active_object

        # Check if selected object is a sensor (edit mode vs create mode)
        is_sensor = (
            obj
            and obj.select_get()
            and obj.type == "EMPTY"
            and hasattr(obj, "linkforge_sensor")
            and getattr(obj, "linkforge_sensor").is_robot_sensor
        )

        # Only show Create button when NOT editing a sensor
        if not is_sensor:
            # Detect target link (either selected link or parent of selected visual)
            target_link = None
            if obj and obj.select_get():
                if hasattr(obj, "linkforge") and getattr(obj, "linkforge").is_robot_link:
                    target_link = obj
                elif (
                    obj.parent
                    and hasattr(obj.parent, "linkforge")
                    and getattr(obj.parent, "linkforge").is_robot_link
                ):
                    # Selected object is a visual/collision child of a link
                    target_link = obj.parent

            # Create Sensor button (creation mode)
            box = layout.box()
            if box:
                row = box.row()
                if row:
                    row.enabled = target_link is not None
                    row.operator("linkforge.create_sensor", icon="ADD", text="Create Sensor")

        # Show sensor properties only if a sensor is selected (edit mode)
        if not is_sensor or not obj:
            return

        props = getattr(obj, "linkforge_sensor")

        # === SENSOR IDENTIFICATION ===
        box = layout.box()
        if box:
            box.label(text=f"Sensor: {obj.name}", icon="OUTLINER_OB_CAMERA")
            box.prop(props, "sensor_name")
            box.prop(props, "topic_name")  # Moved here for better UX
            box.prop(props, "sensor_type")

            # === ATTACHMENT ===
            box.separator()
            box.label(text="Attachment", icon="LINKED")
            box.prop(props, "attached_link", text="Link", icon="OUTLINER_OB_EMPTY")

            # === COMMON SETTINGS ===
            box.separator()
            box.label(text="General Settings", icon="SETTINGS")
            box.prop(props, "update_rate")

            # === TYPE-SPECIFIC SETTINGS ===
            sensor_type = props.sensor_type

            if sensor_type == "CAMERA":
                self._draw_camera_settings(box, props, is_depth=False)
            elif sensor_type == "DEPTH_CAMERA":
                self._draw_camera_settings(box, props, is_depth=True)
            elif sensor_type == "LIDAR":
                self._draw_lidar_settings(box, props)
            elif sensor_type == "CONTACT":
                self._draw_contact_settings(box, props)

            # GPS, Contact, and Force/Torque sensors have no type-specific settings
            # They only use the common settings (update rate, topic, noise)

            # === NOISE SETTINGS (Common to all sensors) ===
            box.separator()
            box.label(text="Noise", icon="RNDCURVE")
            box.prop(props, "use_noise")

            if props.use_noise:
                box.prop(props, "noise_type")
                box.prop(props, "noise_mean")
                box.prop(props, "noise_stddev")

            # === GAZEBO PLUGIN ===
            box.separator()
            box.label(text="Custom Plugin (Advanced)", icon="PLUGIN")
            box.prop(props, "use_gazebo_plugin")

            if props.use_gazebo_plugin:
                box.prop(props, "plugin_filename")

            # Remove Sensor button (Danger Zone)
            box.separator()
            box.separator()
            row = box.row()
            if row:
                row.scale_y = 1.2  # Make it slightly bigger
                row.operator("linkforge.delete_sensor", icon="TRASH", text="Remove Sensor")

    def _draw_camera_settings(
        self, box: UILayout, props: typing.Any, is_depth: bool = False
    ) -> None:
        """Draw camera-specific settings.

        Args:
            box: The UILayout box to draw into.
            props: The property group containing sensor settings.
            is_depth: Whether to draw depth-specific settings.
        """
        box.separator()
        if is_depth:
            box.label(text="Depth Camera Settings", icon="CAMERA_DATA")
        else:
            box.label(text="Camera Settings", icon="CAMERA_DATA")

        # Resolution
        row = box.row(align=True)
        row.prop(props, "camera_width")
        row.prop(props, "camera_height")

        # Field of view
        box.prop(props, "camera_horizontal_fov")

        # Image format
        box.prop(props, "camera_format")

        # Clipping planes
        row = box.row(align=True)
        row.prop(props, "camera_near_clip")
        row.prop(props, "camera_far_clip")

    def _draw_lidar_settings(self, box: UILayout, props: typing.Any) -> None:
        """Draw LIDAR-specific settings.

        Args:
            box: The UILayout box to draw into.
            props: The property group containing sensor settings.
        """
        box.separator()
        box.label(text="LIDAR Settings", icon="LIGHT_SPOT")

        # Horizontal scan
        box.label(text="Horizontal Scan:")
        box.prop(props, "lidar_horizontal_samples")
        row = box.row(align=True)
        row.prop(props, "lidar_horizontal_min_angle")
        row.prop(props, "lidar_horizontal_max_angle")

        # Vertical scan (for 3D LIDAR)
        box.separator()
        box.label(text="Vertical Scan:")
        box.prop(props, "lidar_vertical_samples")

        # Range
        box.separator()
        box.label(text="Range:")
        row = box.row(align=True)
        row.prop(props, "lidar_range_min")
        row.prop(props, "lidar_range_max")

    def _draw_contact_settings(self, box: UILayout, props: typing.Any) -> None:
        """Draw Contact-specific settings.

        Args:
            box: The UILayout box to draw into.
            props: The property group containing sensor settings.
        """
        box.separator()
        box.label(text="Contact Settings", icon="PHYSICS")

        # Collision element name
        box.prop(props, "contact_collision")


# Registration
classes = [
    LINKFORGE_PT_perceive,
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
