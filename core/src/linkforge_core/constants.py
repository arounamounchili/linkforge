"""Central constants and configuration defaults for the LinkForge ecosystem.

This module provides industry-standard baselines used during robot model
definition and validation. Constants are categorized into:

1.  **Infrastructure**: XML/XACRO namespaces and structural prefixes.
2.  **Numerical Stability**: Fundamental precision and solver guardrails.
3.  **Validation**: Physical and geometric sanity thresholds.
4.  **Physics & Dynamics**: Global simulation and component behaviors.
5.  **Component Defaults**: Standard baselines for Links, Joints, and Sensors.
"""

from __future__ import annotations

# 1. XML and XACRO Infrastructure
# ----------------------------

# Official XACRO namespace URIs
XACRO_URIS = {
    "http://www.ros.org/wiki/xacro",
    "http://wiki.ros.org/xacro",
    "http://ros.org/xacro",
}

# Standard prefix for internal structural processing
XACRO_PREFIX = "xacro:"


# 2. Numerical Stability (Foundation)
# ----------------------------

# General small value for floating point comparisons
EPSILON = 1e-9

# Stability epsilon for Sylvester's criterion and inertia checks
SYLVESTER_TOLERANCE_EPSILON = 1e-9

# Minimum mass in kg to prevent singular matrices in dynamics solvers
MIN_REASONABLE_MASS = 1e-6

# Minimum inertia diagonal value to prevent zero-inertia crashes
MIN_REASONABLE_INERTIA = 1e-9

# Thresholds for inertia calculation fallback and stability
MIN_MASS_STABILITY_THRESHOLD = 0.01  # kg
MIN_INERTIA_STABILITY_VALUE = 1e-6  # kg·m²


# 3. Validation Limits (Guardrails)
# ----------------------------

# Maximum absolute value allowed for floats in robot models
# 1e18 is safe for stiffness (kp) while preventing simulation-breaking overflows
MAX_REASONABLE_FLOAT = 1e18

# Maximum absolute value allowed for integers (IDs, sample counts, etc.)
MAX_REASONABLE_INT = 1000000

# Maximum file size for parsers (100 MB)
MAX_FILE_SIZE = 100 * 1024 * 1024

# Maximum depth for XML tree parsing to prevent Billion Laughs / recursion issues
MAX_XML_DEPTH = 2000

# Geometric and Mesh thresholds
DEGENERATE_VOL_THRESHOLD = 1e-12  # m³
NEGATIVE_INERTIA_THRESHOLD = -1e-06
MESH_PROXIMITY_THRESHOLD = 6
MESH_SLIVER_THRESHOLD = 1000.0
MIN_MESH_AREA = 1e-15


# 4. Global Physics Defaults
# ----------------------------

# Default static/dynamic friction coefficient (Coulomb)
DEFAULT_FRICTION_MU = 1.0
DEFAULT_FRICTION_MU2 = 1.0

# Default contact stiffness (N/m) and damping (N s/m)
# 1e12 is the industry standard for 'hard' contact in Gazebo/GZ
DEFAULT_CONTACT_KP = 1e12
DEFAULT_CONTACT_KD = 1.0

# Simulation toggles
DEFAULT_GRAVITY = True
DEFAULT_SELF_COLLIDE = False


# 5. Component Defaults
# ----------------------------

# --- Link Defaults ---
DEFAULT_LINK_MASS = 1.0
DEFAULT_MATERIAL_RGBA = (0.7, 0.7, 0.7, 1.0)
DEFAULT_MATERIAL_RGBA_STR = "0.7 0.7 0.7 1.0"
DEFAULT_MESH_SCALE_STR = "1 1 1"
DEFAULT_GEOMETRY_RADIUS = 0.1
DEFAULT_GEOMETRY_LENGTH = 0.5

# --- Joint Defaults ---
# Default axis (Z-axis is standard for many robotics defaults)
DEFAULT_AXIS_XYZ = (0.0, 0.0, 1.0)
DEFAULT_AXIS_XYZ_STR = "0 0 1"

# Joint Dynamics
DEFAULT_JOINT_DAMPING = 0.0
DEFAULT_JOINT_FRICTION = 0.0
DEFAULT_JOINT_EFFORT = 10.0
DEFAULT_JOINT_VELOCITY = 1.0

# --- Sensor Defaults ---
# Common
DEFAULT_UPDATE_RATE = 30.0
DEFAULT_SENSOR_TYPE = "CAMERA"
DEFAULT_SENSOR_ALWAYS_ON = True
DEFAULT_SENSOR_VISUALIZE = False

# Camera
DEFAULT_CAMERA_FOV = 1.047  # Radians (~60 degrees)
DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 480
DEFAULT_CAMERA_FORMAT = "R8G8B8"
DEFAULT_CAMERA_NEAR = 0.1
DEFAULT_CAMERA_FAR = 100.0

# LIDAR Horizontal Parameters
DEFAULT_LIDAR_SAMPLES = 640
DEFAULT_LIDAR_RANGE_MIN = 0.1
DEFAULT_LIDAR_RANGE_MAX = 10.0
DEFAULT_LIDAR_RANGE_RESOLUTION = 0.01
DEFAULT_LIDAR_MIN_ANGLE = -1.570796  # -90 degrees
DEFAULT_LIDAR_MAX_ANGLE = 1.570796  # +90 degrees

# LIDAR Vertical Parameters
DEFAULT_LIDAR_VERTICAL_SAMPLES = 1
DEFAULT_LIDAR_VERTICAL_MIN_ANGLE = 0.0
DEFAULT_LIDAR_VERTICAL_MAX_ANGLE = 0.0
