"""Sensor models for robotic simulation (Gazebo, SDF, URDF).

This module defines virtual instruments attached to robot links, enabling
feedback for perception, navigation, and control. It serves as a bridge
between physical robot hardware and simulation-specific sensor descriptions.

Sensor Categories:
- **Visual**: Camera and depth camera models.
- **Ranging**: LIDAR (Laser Scanners) with 2D/3D support.
- **Kinematic**: IMU (Inertial Measurement Unit) and GPS (NavSat).
- **Physical**: Force/Torque and contact/collision sensors.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import StrEnum

from ..constants import (
    DEFAULT_CAMERA_FAR,
    DEFAULT_CAMERA_FORMAT,
    DEFAULT_CAMERA_FOV,
    DEFAULT_CAMERA_HEIGHT,
    DEFAULT_CAMERA_NEAR,
    DEFAULT_CAMERA_WIDTH,
    DEFAULT_LIDAR_MAX_ANGLE,
    DEFAULT_LIDAR_MIN_ANGLE,
    DEFAULT_LIDAR_RANGE_MAX,
    DEFAULT_LIDAR_RANGE_MIN,
    DEFAULT_LIDAR_RANGE_RESOLUTION,
    DEFAULT_LIDAR_SAMPLES,
    DEFAULT_LIDAR_VERTICAL_MAX_ANGLE,
    DEFAULT_LIDAR_VERTICAL_MIN_ANGLE,
    DEFAULT_LIDAR_VERTICAL_RESOLUTION,
    DEFAULT_LIDAR_VERTICAL_SAMPLES,
    DEFAULT_UPDATE_RATE,
    EPSILON,
    FT_DIR_CHILD_TO_PARENT,
    FT_DIR_PARENT_TO_CHILD,
    FT_FRAME_CHILD,
    FT_FRAME_PARENT,
    FT_FRAME_SENSOR,
    NOISE_GAUSSIAN,
    SENSOR_CAMERA,
    SENSOR_CONTACT,
    SENSOR_DEPTH_CAMERA,
    SENSOR_FORCE_TORQUE,
    SENSOR_GPS,
    SENSOR_GPU_LIDAR,
    SENSOR_IMU,
    SENSOR_LIDAR,
)
from ..exceptions import RobotValidationError, ValidationErrorCode
from .gazebo import GazeboPlugin
from .geometry import Transform


class SensorType(StrEnum):
    """Enumeration of supported sensor types in the LinkForge IR."""

    CAMERA = SENSOR_CAMERA
    DEPTH_CAMERA = SENSOR_DEPTH_CAMERA
    LIDAR = SENSOR_LIDAR
    GPU_LIDAR = SENSOR_GPU_LIDAR
    IMU = SENSOR_IMU
    GPS = SENSOR_GPS
    FORCE_TORQUE = SENSOR_FORCE_TORQUE
    CONTACT = SENSOR_CONTACT


@dataclass(frozen=True)
class SensorNoise:
    """Noise model for sensor measurements."""

    type: str = NOISE_GAUSSIAN  # gaussian, gaussian_quantized
    mean: float = 0.0
    stddev: float = 0.0
    bias_mean: float = 0.0
    bias_stddev: float = 0.0


@dataclass(frozen=True)
class CameraInfo:
    """Camera-specific sensor information."""

    horizontal_fov: float = DEFAULT_CAMERA_FOV
    width: int = DEFAULT_CAMERA_WIDTH
    height: int = DEFAULT_CAMERA_HEIGHT
    format: str = DEFAULT_CAMERA_FORMAT
    near_clip: float = DEFAULT_CAMERA_NEAR
    far_clip: float = DEFAULT_CAMERA_FAR
    noise: SensorNoise | None = None

    def __post_init__(self) -> None:
        """Validate camera parameters."""

        # Standard pinhole cameras support FOV up to 180° (π radians)
        # For FOV > 180°, use wideanglecamera sensor type instead
        if self.horizontal_fov <= 0 or self.horizontal_fov > (math.pi + EPSILON):
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Camera FOV must be between 0 and 180 degrees (π radians)",
                target="CameraFOV",
                value=self.horizontal_fov,
            )
        if self.width <= 0 or self.height <= 0:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Image dimensions must be positive",
                target="ImageDimensions",
                value=(self.width, self.height),
            )
        if self.near_clip <= 0:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Near clip must be positive",
                target="NearClip",
                value=self.near_clip,
            )
        if self.far_clip <= self.near_clip:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Far clip must be greater than near clip",
                target="FarClip",
                value=self.far_clip,
            )


@dataclass(frozen=True)
class LidarInfo:
    """LIDAR/laser scanner sensor information."""

    # Horizontal scan parameters
    horizontal_samples: int = DEFAULT_LIDAR_SAMPLES
    horizontal_resolution: float = 1.0
    horizontal_min_angle: float = DEFAULT_LIDAR_MIN_ANGLE
    horizontal_max_angle: float = DEFAULT_LIDAR_MAX_ANGLE

    # Vertical scan parameters (for 3D LIDAR)
    vertical_samples: int = DEFAULT_LIDAR_VERTICAL_SAMPLES
    vertical_resolution: float = DEFAULT_LIDAR_VERTICAL_RESOLUTION
    vertical_min_angle: float = DEFAULT_LIDAR_VERTICAL_MIN_ANGLE
    vertical_max_angle: float = DEFAULT_LIDAR_VERTICAL_MAX_ANGLE

    # Range parameters
    range_min: float = DEFAULT_LIDAR_RANGE_MIN
    range_max: float = DEFAULT_LIDAR_RANGE_MAX
    range_resolution: float = DEFAULT_LIDAR_RANGE_RESOLUTION  # m

    # Noise
    noise: SensorNoise | None = None

    def __post_init__(self) -> None:
        """Validate LIDAR parameters."""
        if self.horizontal_samples <= 0:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Lidar samples must be positive",
                target="LidarSamples",
                value=self.horizontal_samples,
            )
        if self.range_min <= 0:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Lidar range_min must be positive",
                target="LidarRangeMin",
                value=self.range_min,
            )
        if self.range_max <= self.range_min:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Lidar range_max must be greater than range_min",
                target="LidarRangeMax",
                value=self.range_max,
            )
        if self.range_resolution <= 0:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Lidar range_resolution must be positive",
                target="LidarRangeResolution",
                value=self.range_resolution,
            )
        if self.horizontal_min_angle >= self.horizontal_max_angle:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Lidar horizontal min_angle must be less than max_angle",
                target="LidarAngleRange",
                value=(self.horizontal_min_angle, self.horizontal_max_angle),
            )
        if self.vertical_samples > 1 and self.vertical_min_angle >= self.vertical_max_angle:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Lidar vertical min_angle must be less than max_angle for 3D scans",
                target="LidarVerticalAngleRange",
                value=(self.vertical_min_angle, self.vertical_max_angle),
            )


@dataclass(frozen=True)
class IMUInfo:
    """IMU sensor information."""

    angular_velocity_noise: SensorNoise | None = None
    linear_acceleration_noise: SensorNoise | None = None


@dataclass(frozen=True)
class GPSInfo:
    """GPS sensor information."""

    # Position noise
    position_sensing_horizontal_noise: SensorNoise | None = None
    position_sensing_vertical_noise: SensorNoise | None = None

    # Velocity noise
    velocity_sensing_horizontal_noise: SensorNoise | None = None
    velocity_sensing_vertical_noise: SensorNoise | None = None


@dataclass(frozen=True)
class ContactInfo:
    """Contact sensor information.

    Contact sensors detect collisions and contact forces.
    """

    # Name of the collision element to monitor
    collision: str

    # Noise model for contact detection
    noise: SensorNoise | None = None

    def with_prefix(self, prefix: str) -> ContactInfo:
        """Create a new contact info with a prefixed collision reference."""
        return replace(self, collision=f"{prefix}{self.collision}")


@dataclass(frozen=True)
class ForceTorqueInfo:
    """Force/Torque sensor information.

    F/T sensors measure forces and torques applied to joints or links.
    """

    # Measurement frame (child, parent, or sensor)
    frame: str = FT_FRAME_CHILD
    # Defines the direction (parent_to_child or child_to_parent)
    measure_direction: str = FT_DIR_CHILD_TO_PARENT

    # Noise model for force/torque measurements
    noise: SensorNoise | None = None

    def __post_init__(self) -> None:
        """Validate F/T sensor parameters."""
        if self.frame not in (FT_FRAME_CHILD, FT_FRAME_PARENT, FT_FRAME_SENSOR):
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Invalid F/T frame '{self.frame}' (must be child, parent, or sensor)",
                target="ForceTorqueFrame",
                value=self.frame,
            )
        if self.measure_direction not in (FT_DIR_CHILD_TO_PARENT, FT_DIR_PARENT_TO_CHILD):
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Invalid F/T direction '{self.measure_direction}'",
                target="ForceTorqueDirection",
                value=self.measure_direction,
            )


# GazeboPlugin is imported from gazebo module to avoid duplication


@dataclass(frozen=True)
class Sensor:
    """Unified sensor model for simulation and hardware abstraction.

    Sensors are logical entities attached to robot links. They define
    data acquisition parameters (update rate, FOV, range) and are
    exported to simulation-specific formats (e.g., Gazebo <sensor> tags).
    """

    name: str
    type: SensorType
    link_name: str  # Link this sensor is attached to
    update_rate: float = DEFAULT_UPDATE_RATE  # Hz
    always_on: bool = True
    visualize: bool = False

    # Sensor-specific information (only one should be set based on type)
    camera_info: CameraInfo | None = None
    lidar_info: LidarInfo | None = None
    imu_info: IMUInfo | None = None
    gps_info: GPSInfo | None = None
    contact_info: ContactInfo | None = None
    force_torque_info: ForceTorqueInfo | None = None

    # Transform relative to parent link
    origin: Transform = field(default_factory=Transform.identity)

    # Optional topic name for ROS
    topic: str | None = None

    # Plugin configuration
    plugin: GazeboPlugin | None = None

    def __post_init__(self) -> None:
        """Validate sensor configuration."""
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "Sensor name cannot be empty",
                target="SensorName",
                value=self.name,
            )
        if not self.link_name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "Sensor link_name cannot be empty",
                target="LinkName",
                value=self.link_name,
            )
        if self.update_rate <= 0:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Sensor update rate must be positive",
                target="UpdateRate",
                value=self.update_rate,
            )

        # Validate that appropriate info is set for sensor type
        if self.type in (SensorType.CAMERA, SensorType.DEPTH_CAMERA):
            if self.camera_info is None:
                raise RobotValidationError(
                    ValidationErrorCode.INVALID_VALUE,
                    f"Sensor '{self.name}' [type: {self.type.value}] requires camera_info",
                    target="SensorInfo",
                    value=self.name,
                )
        elif self.type == SensorType.LIDAR:
            if self.lidar_info is None:
                raise RobotValidationError(
                    ValidationErrorCode.INVALID_VALUE,
                    f"Sensor '{self.name}' [type: {self.type.value}] requires lidar_info",
                    target="SensorInfo",
                    value=self.name,
                )
        elif self.type == SensorType.IMU:
            if self.imu_info is None:
                raise RobotValidationError(
                    ValidationErrorCode.INVALID_VALUE,
                    f"Sensor '{self.name}' [type: {self.type.value}] requires imu_info",
                    target="SensorInfo",
                    value=self.name,
                )
        elif self.type == SensorType.GPS and self.gps_info is None:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Sensor '{self.name}' [type: {self.type.value}] requires gps_info",
                target="SensorInfo",
                value=self.name,
            )
        elif self.type == SensorType.CONTACT and self.contact_info is None:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Sensor '{self.name}' [type: {self.type.value}] requires contact_info",
                target="SensorInfo",
                value=self.name,
            )
        elif self.type == SensorType.FORCE_TORQUE and self.force_torque_info is None:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Sensor '{self.name}' [type: {self.type.value}] requires force_torque_info",
                target="SensorInfo",
                value=self.name,
            )

    def with_prefix(self, prefix: str) -> Sensor:
        """Create a new sensor with prefixed name, link, topic, contact_info, and plugin."""
        return replace(
            self,
            name=f"{prefix}{self.name}",
            link_name=f"{prefix}{self.link_name}",
            topic=f"{prefix}{self.topic}" if self.topic else None,
            contact_info=self.contact_info.with_prefix(prefix) if self.contact_info else None,
            plugin=self.plugin.with_prefix(prefix) if self.plugin else None,
        )
