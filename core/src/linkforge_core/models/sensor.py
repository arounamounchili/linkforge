"""Sensor models for URDF/Gazebo integration."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from ..exceptions import RobotValidationError, ValidationErrorCode
from .gazebo import GazeboPlugin
from .geometry import Transform


class SensorType(str, Enum):
    """Supported sensor types in Gazebo/SDF."""

    CAMERA = "camera"
    DEPTH_CAMERA = "depth_camera"
    LIDAR = "lidar"
    IMU = "imu"
    GPS = "gps"
    FORCE_TORQUE = "force_torque"
    CONTACT = "contact"


@dataclass(frozen=True)
class SensorNoise:
    """Noise model for sensor measurements."""

    type: str = "gaussian"  # gaussian, gaussian_quantized
    mean: float = 0.0
    stddev: float = 0.0
    bias_mean: float = 0.0
    bias_stddev: float = 0.0


@dataclass(frozen=True)
class CameraInfo:
    """Camera-specific sensor information."""

    horizontal_fov: float = 1.047  # ~60 degrees in radians
    width: int = 640
    height: int = 480
    format: str = "R8G8B8"  # Pixel format
    near_clip: float = 0.1
    far_clip: float = 100.0
    noise: SensorNoise | None = None

    def __post_init__(self) -> None:
        """Validate camera parameters."""
        import math

        # Standard pinhole cameras support FOV up to 180° (π radians)
        # For FOV > 180°, use wideanglecamera sensor type instead
        # Use small tolerance (1e-6) to handle floating-point precision from UI conversions
        if self.horizontal_fov <= 0 or self.horizontal_fov > (math.pi + 1e-6):
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
    horizontal_samples: int = 640
    horizontal_resolution: float = 1.0
    horizontal_min_angle: float = -1.570796  # -π/2 radians (-90°)
    horizontal_max_angle: float = 1.570796  # π/2 radians (90°)

    # Vertical scan parameters (for 3D LIDAR)
    vertical_samples: int = 1
    vertical_resolution: float = 1.0
    vertical_min_angle: float = 0.0
    vertical_max_angle: float = 0.0

    # Range parameters
    range_min: float = 0.1
    range_max: float = 10.0
    range_resolution: float = 0.01

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
        if self.horizontal_min_angle >= self.horizontal_max_angle:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Lidar horizontal min_angle must be less than max_angle",
                target="LidarAngleRange",
                value=(self.horizontal_min_angle, self.horizontal_max_angle),
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


@dataclass(frozen=True)
class ForceTorqueInfo:
    """Force/Torque sensor information.

    F/T sensors measure forces and torques applied to joints or links.
    """

    # Measurement frame (child or parent or sensor)
    frame: str = "child"

    # Direction of measurement (parent_to_child or child_to_parent)
    measure_direction: str = "child_to_parent"

    # Noise model for force/torque measurements
    noise: SensorNoise | None = None


# GazeboPlugin is imported from gazebo module to avoid duplication


@dataclass(frozen=True)
class Sensor:
    """Generic sensor definition for Gazebo simulation.

    Sensors are attached to links and provide measurements in simulation.
    """

    name: str
    type: SensorType
    link_name: str  # Link this sensor is attached to
    update_rate: float = 30.0  # Hz
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
