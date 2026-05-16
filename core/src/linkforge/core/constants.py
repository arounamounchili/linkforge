"""Central constants and configuration defaults for the LinkForge ecosystem.

This module provides industry-standard baselines and architectural tokens used
throughout the Intermediate Representation (IR), ensuring cross-platform
consistency and physical plausibility.

Core Components:
    - Infrastructure: Official XML namespaces, versioning, and structural prefixes.
    - Numerical Stability: Precision thresholds, epsilon values, and solver guards.
    - Collision Filtering: Constants for collision mask generation.
    - Physical Defaults: Standard update rates, gravity settings, and material properties.
    - Semantic Tokens: Standardized identifiers for joints, sensors, and hardware interfaces.
"""

from __future__ import annotations

from typing import Final

# 1. XML and XACRO Infrastructure
# ----------------------------

# Official XACRO namespace URIs
XACRO_URIS: Final[set[str]] = {
    "http://www.ros.org/wiki/xacro",
    "http://wiki.ros.org/xacro",
    "http://ros.org/xacro",
}

# Standard prefix for internal structural processing
XACRO_PREFIX: Final[str] = "xacro:"


# 2. Numerical Stability (Foundation)
# ----------------------------

# General small value for floating point comparisons
EPSILON: Final[float] = 1e-9

# Mathematical Constants (Architectural precision)
PI: Final[float] = 3.14159265358979323846
HALF_PI: Final[float] = 1.57079632679489661923

# Stability epsilon for Sylvester's criterion and inertia checks
SYLVESTER_TOLERANCE_EPSILON: Final[float] = 1e-9

# General geometric epsilon for dimension and distance checks
GEOM_EPSILON: Final[float] = 1e-6

# Minimum mass to prevent singular matrices in dynamics solvers
MIN_REASONABLE_MASS: Final[float] = 1e-6  # kg

# Minimum inertia diagonal value to prevent zero-inertia crashes
MIN_REASONABLE_INERTIA: Final[float] = 1e-9  # kg·m²

# Thresholds for inertia calculation fallback and stability
MIN_MASS_STABILITY_THRESHOLD: Final[float] = 0.01  # kg
MIN_INERTIA_STABILITY_VALUE: Final[float] = 1e-6  # kg·m²


# 3. Validation Limits (Guardrails)
# ----------------------------

# Maximum absolute value allowed for floats in robot models
# 1e18 is safe for stiffness (kp) while preventing simulation-breaking overflows
MAX_REASONABLE_FLOAT: Final[float] = 1e18

# Maximum absolute value allowed for integers (IDs, sample counts, etc.)
MAX_REASONABLE_INT: Final[int] = 1000000

# Maximum file size for parsers (100 MB)
MAX_FILE_SIZE: Final[int] = 100 * 1024 * 1024  # bytes

# Maximum depth for XML tree parsing to prevent Billion Laughs / recursion issues
MAX_XML_DEPTH: Final[int] = 2000

# Geometric and Mesh thresholds
DEGENERATE_VOL_THRESHOLD: Final[float] = 1e-12  # m³
NEGATIVE_INERTIA_THRESHOLD: Final[float] = -1e-06
MESH_PROXIMITY_THRESHOLD: Final[int] = 6
MESH_SLIVER_THRESHOLD: Final[float] = 1000.0
MIN_MESH_AREA: Final[float] = 1e-15  # m²


# 4. Global Physics Defaults
# ----------------------------

# Default static/dynamic friction coefficient (Coulomb)
DEFAULT_FRICTION_MU: Final[float] = 1.0
DEFAULT_FRICTION_MU2: Final[float] = 1.0

# Default contact stiffness and damping
# 1e12 is the industry standard for 'hard' contact in Gazebo/GZ
DEFAULT_CONTACT_KP: Final[float] = 1e12  # N/m
DEFAULT_CONTACT_KD: Final[float] = 1.0  # N·s/m

# Simulation toggles
DEFAULT_GRAVITY: Final[bool] = True
DEFAULT_SELF_COLLIDE: Final[bool] = False


# 5. Component Defaults
# ----------------------------

# --- Link Defaults ---
DEFAULT_LINK_MASS: Final[float] = 1.0  # kg
DEFAULT_MATERIAL_RGBA: Final[tuple[float, float, float, float]] = (0.7, 0.7, 0.7, 1.0)
DEFAULT_MATERIAL_RGBA_STR: Final[str] = "0.7 0.7 0.7 1.0"
DEFAULT_MESH_SCALE_STR: Final[str] = "1 1 1"
DEFAULT_GEOMETRY_RADIUS: Final[float] = 0.1  # m
DEFAULT_GEOMETRY_LENGTH: Final[float] = 0.5  # m

# Joint Defaults
DEFAULT_AXIS_XYZ: Final[tuple[float, float, float]] = (0.0, 0.0, 1.0)
DEFAULT_AXIS_XYZ_STR: Final[str] = "0 0 1"
DEFAULT_URDF_AXIS_XYZ: Final[tuple[float, float, float]] = (1.0, 0.0, 0.0)  # URDF spec default
DEFAULT_URDF_AXIS_XYZ_STR: Final[str] = "1 0 0"

# Joint Dynamics
DEFAULT_JOINT_DAMPING: Final[float] = 0.0  # N·s/m
DEFAULT_JOINT_FRICTION: Final[float] = 0.0  # N·m
DEFAULT_JOINT_EFFORT: Final[float] = 10.0  # N or N·m
DEFAULT_JOINT_VELOCITY: Final[float] = 1.0  # m/s or rad/s

# --- Sensor Defaults ---
# Common
DEFAULT_UPDATE_RATE: Final[float] = 30.0  # Hz
DEFAULT_SENSOR_TYPE: Final[str] = "camera"
DEFAULT_SENSOR_ALWAYS_ON: Final[bool] = True
DEFAULT_SENSOR_VISUALIZE: Final[bool] = False

# Camera
DEFAULT_CAMERA_FOV: Final[float] = 1.047  # rad (~60 deg)
DEFAULT_CAMERA_WIDTH: Final[int] = 640  # px
DEFAULT_CAMERA_HEIGHT: Final[int] = 480  # px
DEFAULT_CAMERA_FORMAT: Final[str] = "R8G8B8"
DEFAULT_CAMERA_NEAR: Final[float] = 0.1  # m
DEFAULT_CAMERA_FAR: Final[float] = 100.0  # m

# Standard Camera Formats
CAM_FORMAT_RGB8: Final[str] = "R8G8B8"
CAM_FORMAT_RGB16: Final[str] = "R16G16B16"
CAM_FORMAT_GRAY8: Final[str] = "L8"
CAM_FORMAT_GRAY16: Final[str] = "L16"
CAM_FORMAT_BAYER_RGGB8: Final[str] = "BAYER_RGGB8"
CAM_FORMAT_BAYER_BGGR8: Final[str] = "BAYER_BGGR8"

# LIDAR Horizontal Parameters
DEFAULT_LIDAR_SAMPLES: Final[int] = 640
DEFAULT_LIDAR_HORIZONTAL_RESOLUTION: Final[float] = 1.0
DEFAULT_LIDAR_RANGE_MIN: Final[float] = 0.1  # m
DEFAULT_LIDAR_RANGE_MAX: Final[float] = 10.0  # m
DEFAULT_LIDAR_RANGE_RESOLUTION: Final[float] = 0.01  # m
DEFAULT_LIDAR_MIN_ANGLE: Final[float] = -HALF_PI  # rad (-90 deg)
DEFAULT_LIDAR_MAX_ANGLE: Final[float] = HALF_PI  # rad (+90 deg)

# LIDAR Vertical Parameters
DEFAULT_LIDAR_VERTICAL_SAMPLES: Final[int] = 1
DEFAULT_LIDAR_VERTICAL_RESOLUTION: Final[float] = 1.0
DEFAULT_LIDAR_VERTICAL_MIN_ANGLE: Final[float] = 0.0  # rad
DEFAULT_LIDAR_VERTICAL_MAX_ANGLE: Final[float] = 0.0  # rad


# 6. Categorical Standards (URDF & LinkForge Types)
# ----------------------------

# LinkForge IR Version
IR_VERSION: Final[str] = "1.1"

# Geometry Types
GEOM_BOX: Final[str] = "box"
GEOM_CYLINDER: Final[str] = "cylinder"
GEOM_SPHERE: Final[str] = "sphere"
GEOM_MESH: Final[str] = "mesh"

# Joint Types
JOINT_REVOLUTE: Final[str] = "revolute"
JOINT_CONTINUOUS: Final[str] = "continuous"
JOINT_PRISMATIC: Final[str] = "prismatic"
JOINT_FIXED: Final[str] = "fixed"
JOINT_FLOATING: Final[str] = "floating"
JOINT_PLANAR: Final[str] = "planar"
DEFAULT_JOINT_TYPE: Final[str] = JOINT_REVOLUTE

# Sensor Types
SENSOR_CAMERA: Final[str] = "camera"
SENSOR_DEPTH_CAMERA: Final[str] = "depth_camera"
SENSOR_LIDAR: Final[str] = "lidar"
SENSOR_GPU_LIDAR: Final[str] = "gpu_lidar"
SENSOR_IMU: Final[str] = "imu"
SENSOR_GPS: Final[str] = "gps"
SENSOR_CONTACT: Final[str] = "contact"
SENSOR_FORCE_TORQUE: Final[str] = "force_torque"

# Transmission Types
TRANS_SIMPLE: Final[str] = "simple"
TRANS_DIFFERENTIAL: Final[str] = "differential"
TRANS_FOUR_BAR: Final[str] = "four_bar_linkage"
TRANS_CUSTOM: Final[str] = "custom"

# Hardware Interfaces
HW_IF_POSITION: Final[str] = "position"
HW_IF_VELOCITY: Final[str] = "velocity"
HW_IF_EFFORT: Final[str] = "effort"

# ROS2 Control Hardware Types
CONTROL_TYPE_SYSTEM: Final[str] = "system"
CONTROL_TYPE_ACTUATOR: Final[str] = "actuator"
CONTROL_TYPE_SENSOR: Final[str] = "sensor"

# SRDF Semantic Types
SRDF_VJOIN_FIXED: Final[str] = "fixed"
SRDF_VJOIN_PLANAR: Final[str] = "planar"
SRDF_VJOIN_FLOATING: Final[str] = "floating"

SRDF_REASON_ADJACENT: Final[str] = "Adjacent"
SRDF_REASON_NEVER: Final[str] = "Never"
SRDF_REASON_USER: Final[str] = "User"
SRDF_REASON_DEFAULT: Final[str] = "Default"

# Standard fallback names
UNNAMED_LINK: Final[str] = "unnamed_link"
UNNAMED_JOINT: Final[str] = "unnamed_joint"
UNNAMED_SENSOR: Final[str] = "unnamed_sensor"

# Sensor Update Rates (Industry standard defaults)
DEFAULT_UPDATE_RATE_IMU: Final[float] = 100.0  # Hz
DEFAULT_UPDATE_RATE_GPS: Final[float] = 5.0  # Hz
DEFAULT_UPDATE_RATE_LIDAR: Final[float] = 30.0  # Hz
DEFAULT_UPDATE_RATE_CAMERA: Final[float] = 30.0  # Hz
DEFAULT_UPDATE_RATE_CONTACT: Final[float] = 50.0  # Hz
DEFAULT_UPDATE_RATE_FORCE_TORQUE: Final[float] = 100.0  # Hz


# 7. Internal Engine Configuration
# ----------------------------
# --- Sensor Config ---
NOISE_GAUSSIAN: Final[str] = "gaussian"
NOISE_GAUSSIAN_QUANTIZED: Final[str] = "gaussian_quantized"

FT_FRAME_CHILD: Final[str] = "child"
FT_FRAME_PARENT: Final[str] = "parent"
FT_FRAME_SENSOR: Final[str] = "sensor"

FT_DIR_CHILD_TO_PARENT: Final[str] = "child_to_parent"
FT_DIR_PARENT_TO_CHILD: Final[str] = "parent_to_child"

# Cache size for inertia calculations
DEFAULT_INERTIA_CACHE_SIZE: Final[int] = 512


# 8. Platform-Specific Tokens (Gazebo, ROS2, XACRO)
# ----------------------------

# Gazebo Sim / Ignition Sensor Type Names
GZ_SENSOR_LIDAR: Final[str] = "lidar"
GZ_SENSOR_GPU_LIDAR: Final[str] = "gpu_lidar"
GZ_SENSOR_NAVSAT: Final[str] = "navsat"
GZ_SENSOR_CAMERA: Final[str] = "camera"
GZ_SENSOR_DEPTH_CAMERA: Final[str] = "depth_camera"
GZ_SENSOR_IMU: Final[str] = "imu"
GZ_SENSOR_CONTACT: Final[str] = "contact"
GZ_SENSOR_FORCE_TORQUE: Final[str] = "force_torque"

# Gazebo / GZ XML Elements and Attributes
GZ_ELEM_GAZEBO: Final[str] = "gazebo"
GZ_ELEM_SENSOR: Final[str] = "sensor"
GZ_ELEM_PLUGIN: Final[str] = "plugin"
GZ_ATTR_REFERENCE: Final[str] = "reference"
COLLISION_ADJACENT: Final[str] = "Adjacent"

# XACRO Parameters and Attributes
XACRO_PARAM_NAME: Final[str] = "name"
XACRO_PARAM_PARENT: Final[str] = "parent"
XACRO_PARAM_XYZ: Final[str] = "xyz"
XACRO_PARAM_RPY: Final[str] = "rpy"
XACRO_DEFAULT_PARAMS: Final[str] = "name parent xyz rpy"

# ROS2 Control Defaults
ROS2_CONTROL_DEFAULT_PLUGIN: Final[str] = "gz_ros2_control/GazeboSimSystem"
ROS2_CONTROL_DEFAULT_GAZEBO_PLUGIN: Final[str] = "gz_ros2_control::GazeboSimROS2ControlPlugin"


# 9. XML Formatting Constants
# ----------------------------
COMMENT_MATERIALS: Final[str] = " Materials "
COMMENT_LINKS: Final[str] = " Links "
COMMENT_JOINTS: Final[str] = " Joints "
COMMENT_TRANSMISSIONS: Final[str] = " Transmissions "
COMMENT_SENSORS: Final[str] = " Sensors "
COMMENT_GAZEBO: Final[str] = " Gazebo Extensions "
COMMENT_ROS2_CONTROL: Final[str] = " ROS 2 Control "
COMMENT_PROPERTIES: Final[str] = " Properties "
COMMENT_MACROS: Final[str] = " Macros "
