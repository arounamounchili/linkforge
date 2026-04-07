"""Blender Property Groups for robot sensors.

These properties are stored on Empty objects and define sensor characteristics.
"""

from __future__ import annotations

import typing

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import Context, PropertyGroup

from linkforge.blender.utils.scene_utils import clear_stats_cache

if typing.TYPE_CHECKING:
    from .link_props import LinkPropertyGroup

from linkforge_core.utils.string_utils import sanitize_name as sanitize_urdf_name

from linkforge.blender.utils.property_helpers import find_property_owner


def get_sensor_name(self: SensorPropertyGroup) -> str:
    """Getter for sensor_name - returns the persistent URDF identity.

    Args:
        self: The SensorPropertyGroup instance.

    Returns:
        The sanitized URDF name.
    """
    # Prioritize the stored identity to avoid Blender's .001 suffixing
    if self.urdf_name_stored:
        return str(self.urdf_name_stored)

    if not self.id_data:
        return ""

    return sanitize_urdf_name(str(self.id_data.name))


def set_sensor_name(self: SensorPropertyGroup, value: str) -> None:
    """Setter for sensor_name - updates persistent identity and object name.

    Args:
        self: The SensorPropertyGroup instance.
        value: The new name value to set.
    """
    if not value or not self.id_data:
        return

    # Sanitize sensor name for URDF
    sanitized_name = sanitize_urdf_name(value)

    # Store the persistent identity
    self.urdf_name_stored = sanitized_name

    # Update object name to match sensor name
    if self.id_data.name != sanitized_name:
        # Block handler loop: We only update if they differ already
        self.id_data.name = sanitized_name


def update_sensor_hierarchy(self: SensorPropertyGroup, context: Context) -> None:
    """Update Blender object hierarchy when attached link changes.

    Automatically reparents sensor to new link and moves to link's collection.
    This ensures visual hierarchy matches logical structure.
    """

    # Find the sensor object that owns this property
    sensor_obj = find_property_owner(context, self, "linkforge_sensor")
    if sensor_obj is None or not self.is_robot_sensor:
        return

    from ..utils.transform_utils import clear_parent_keep_transform, set_parent_keep_transform

    link_obj = self.attached_link

    if link_obj:
        # Only reparent if not already parented to the correct link
        if sensor_obj.parent != link_obj:
            # Parent the Sensor to the Link (Keep Transform)
            set_parent_keep_transform(sensor_obj, link_obj)

        # Move to same collection
        from ..utils.scene_utils import sync_object_collections

        sync_object_collections(sensor_obj, link_obj)

    elif sensor_obj.parent:
        # Clear parent while preserving world position
        clear_parent_keep_transform(sensor_obj)


def poll_robot_link(_self: SensorPropertyGroup, obj: bpy.types.Object) -> bool:
    """Filter to only allow robot link objects in pointer selection."""
    return bool(
        obj
        and hasattr(obj, "linkforge")
        and typing.cast("LinkPropertyGroup", obj.linkforge).is_robot_link
    )


class SensorPropertyGroup(PropertyGroup):
    """Properties for a robot sensor stored on an Empty object."""

    # Sensor identification
    is_robot_sensor: BoolProperty(  # type: ignore
        name="Is Robot Sensor",
        description="Mark this object as a robot sensor",
        default=False,
    )

    # Persistent URDF Identity
    # Decouples logical URDF naming from physical Blender object names (resilient to .001 suffixes)
    urdf_name_stored: StringProperty(  # type: ignore
        name="URDF Name",
        description="Persistent URDF name. Prevents mapping breakage if Blender renames the object",
        default="",
    )

    sensor_name: StringProperty(  # type: ignore
        name="Sensor Name",
        description="Name of the sensor in URDF (must be unique)",
        maxlen=64,
        get=get_sensor_name,
        set=set_sensor_name,
        update=clear_stats_cache,
    )

    # Sensor type
    sensor_type: EnumProperty(  # type: ignore[valid-type]
        name="Sensor Type",
        description="Type of sensor",
        items=[
            ("CAMERA", "Camera", "RGB camera sensor"),
            ("DEPTH_CAMERA", "Depth Camera", "Depth/RGBD camera sensor"),
            ("LIDAR", "LIDAR", "2D/3D laser scanner"),
            ("IMU", "IMU", "Inertial measurement unit"),
            ("GPS", "GPS", "Global positioning system"),
            ("CONTACT", "Contact", "Contact sensor"),
            ("FORCE_TORQUE", "Force/Torque", "Force-torque sensor"),
        ],
        default="CAMERA",
    )

    # Attached link
    attached_link: PointerProperty(  # type: ignore[valid-type]
        name="Attached Link",
        description="Select the link this sensor is attached to",
        type=bpy.types.Object,
        poll=poll_robot_link,
        update=update_sensor_hierarchy,
    )

    # Common sensor properties
    update_rate: FloatProperty(  # type: ignore[valid-type]
        name="Update Rate",
        description="Sensor update rate in Hz",
        default=30.0,
        min=0.1,
        soft_max=100.0,
        precision=1,
    )

    always_on: BoolProperty(  # type: ignore[valid-type]
        name="Always On",
        description="Whether the sensor is always active",
        default=False,
    )

    visualize: BoolProperty(  # type: ignore[valid-type]
        name="Visualize",
        description="Enable visualization in the simulator",
        default=False,
    )

    topic_name: StringProperty(  # type: ignore[valid-type]
        name="Topic Name",
        description="ROS topic name for sensor data",
        default="",
        maxlen=128,
    )

    # Camera-specific properties
    camera_horizontal_fov: FloatProperty(  # type: ignore[valid-type]
        name="Horizontal FOV",
        description="Camera horizontal field of view (displayed in degrees, stored as radians). Standard cameras support up to 180°",
        default=1.047,  # ~60 degrees in radians
        min=0.1,
        max=3.14159265359,  # π radians = 180° (maximum for pinhole camera model)
        precision=3,
        subtype="ANGLE",  # Blender displays this in degrees
    )

    camera_width: IntProperty(  # type: ignore[valid-type]
        name="Image Width",
        description="Camera image width in pixels",
        default=640,
        min=1,
        soft_max=1920,
    )

    camera_height: IntProperty(  # type: ignore[valid-type]
        name="Image Height",
        description="Camera image height in pixels",
        default=480,
        min=1,
        soft_max=1080,
    )

    camera_near_clip: FloatProperty(  # type: ignore[valid-type]
        name="Near Clip",
        description="Camera near clipping plane distance (meters)",
        default=0.1,
        min=0.001,
        soft_max=10.0,
        precision=3,
    )

    camera_far_clip: FloatProperty(  # type: ignore[valid-type]
        name="Far Clip",
        description="Camera far clipping plane distance (meters)",
        default=100.0,
        min=0.1,
        soft_max=1000.0,
        precision=1,
    )

    camera_format: EnumProperty(  # type: ignore[valid-type]
        name="Image Format",
        description="Camera image pixel format",
        items=[
            ("R8G8B8", "RGB8", "8-bit RGB color"),
            ("R16G16B16", "RGB16", "16-bit RGB color"),
            ("L8", "L8 (Grayscale)", "8-bit grayscale"),
            ("L16", "L16 (Grayscale)", "16-bit grayscale"),
            ("BAYER_RGGB8", "Bayer RGGB8", "8-bit Bayer pattern"),
            ("BAYER_BGGR8", "Bayer BGGR8", "8-bit Bayer pattern"),
        ],
        default="R8G8B8",
    )

    # LIDAR-specific properties
    lidar_horizontal_samples: IntProperty(  # type: ignore[valid-type]
        name="Horizontal Samples",
        description="Number of horizontal scan samples",
        default=640,
        min=1,
        soft_max=2048,
    )

    lidar_horizontal_min_angle: FloatProperty(  # type: ignore[valid-type]
        name="Horizontal Min Angle",
        description="Minimum horizontal scan angle (displayed in degrees, stored as radians)",
        default=-1.5707963267948966,  # -90 degrees
        min=-3.14159265359,  # -180°
        max=3.14159265359,  # 180°
        precision=3,
        subtype="ANGLE",  # Blender displays this in degrees
    )

    lidar_horizontal_max_angle: FloatProperty(  # type: ignore[valid-type]
        name="Horizontal Max Angle",
        description="Maximum horizontal scan angle (displayed in degrees, stored as radians)",
        default=1.5707963267948966,  # 90 degrees
        min=-3.14159265359,  # -180°
        max=3.14159265359,  # 180°
        precision=3,
        subtype="ANGLE",  # Blender displays this in degrees
    )

    lidar_vertical_samples: IntProperty(  # type: ignore[valid-type]
        name="Vertical Samples",
        description="Number of vertical scan samples (1 for 2D LIDAR)",
        default=1,
        min=1,
        soft_max=128,
    )

    lidar_range_min: FloatProperty(  # type: ignore[valid-type]
        name="Range Min",
        description="Minimum detection range in meters",
        default=0.1,
        min=0.001,
        soft_max=10.0,
        precision=3,
    )

    lidar_range_max: FloatProperty(  # type: ignore[valid-type]
        name="Range Max",
        description="Maximum detection range in meters",
        default=10.0,
        min=0.1,
        soft_max=100.0,
        precision=1,
    )

    # Contact-specific properties
    contact_collision: StringProperty(  # type: ignore[valid-type]
        name="Collision Name",
        description="Name of the collision element to monitor (defaults to linkname_collision if empty)",
        default="",
        maxlen=64,
    )

    # IMU-specific properties
    # Gravity is handled by World settings in Gazebo

    # Noise properties
    use_noise: BoolProperty(  # type: ignore[valid-type]
        name="Use Noise",
        description="Add realistic noise to sensor measurements",
        default=False,
    )

    noise_type: EnumProperty(  # type: ignore[valid-type]
        name="Noise Type",
        description="Type of noise model",
        items=[
            ("gaussian", "Gaussian", "Gaussian noise"),
            ("gaussian_quantized", "Gaussian Quantized", "Quantized Gaussian noise"),
        ],
        default="gaussian",
    )

    noise_mean: FloatProperty(  # type: ignore[valid-type]
        name="Noise Mean",
        description="Mean of the noise distribution",
        default=0.0,
        precision=5,
    )

    noise_stddev: FloatProperty(  # type: ignore[valid-type]
        name="Noise Std Dev",
        description="Standard deviation of the noise",
        default=0.0,
        min=0.0,
        precision=5,
    )

    # Gazebo plugin settings
    use_gazebo_plugin: BoolProperty(  # type: ignore[valid-type]
        name="Gazebo Plugin",
        description="Enable Gazebo plugin for this sensor",
        default=False,
    )

    plugin_filename: StringProperty(  # type: ignore[valid-type]
        name="Plugin Filename",
        description="Gazebo plugin library filename (e.g., libgazebo_ros_camera.so)",
        default="",
        maxlen=128,
    )

    plugin_raw_xml: StringProperty(  # type: ignore[valid-type]
        name="Plugin Raw XML",
        description="Raw XML content of plugin (for round-trip fidelity)",
        default="",
    )


# Registration
def register() -> None:
    """Register property group."""
    try:
        bpy.utils.register_class(SensorPropertyGroup)
    except ValueError:
        # If already registered (e.g. from reload), unregister first to ensure clean state
        bpy.utils.unregister_class(SensorPropertyGroup)
        bpy.utils.register_class(SensorPropertyGroup)

    prop_name = "linkforge_sensor"
    setattr(
        bpy.types.Object,
        prop_name,
        typing.cast(typing.Any, PointerProperty(type=SensorPropertyGroup)),
    )


def unregister() -> None:
    """Unregister property group."""
    import contextlib

    with contextlib.suppress(AttributeError):
        delattr(bpy.types.Object, "linkforge_sensor")

    with contextlib.suppress(RuntimeError):
        bpy.utils.unregister_class(SensorPropertyGroup)


if __name__ == "__main__":
    register()
