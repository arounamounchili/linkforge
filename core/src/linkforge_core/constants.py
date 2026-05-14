"""Central constants and configuration defaults for the LinkForge ecosystem.

This module provides industry-standard baselines used during robot model
definition and validation. Constants are categorized into:

- **Physics Defaults**: Standard friction, stiffness, and damping coefficients.
- **Namespaces**: Official URIs and prefixes for XML/XACRO processing.
- **Numerical Stability**: Guardrails (mass, inertia, epsilon) for simulation.
- **Validation**: Thresholds for mesh topology and file size.
"""

from __future__ import annotations

# Physics Defaults (Simulation)
# ----------------------------

# Default static friction coefficient (Coulomb)
DEFAULT_FRICTION_MU = 1.0

# Default dynamic friction coefficient (Coulomb)
DEFAULT_FRICTION_MU2 = 1.0

# Default contact stiffness (N/m)
# 1e12 is the industry standard for 'hard' contact in Gazebo/GZ
DEFAULT_CONTACT_KP = 1e12

# Default contact damping (N s/m)
DEFAULT_CONTACT_KD = 1.0

# Default gravity inclusion
DEFAULT_GRAVITY = True

# Default self-collision inclusion
DEFAULT_SELF_COLLIDE = False

# Default Joint Axis (Z-axis is standard for many robotics defaults)
DEFAULT_AXIS_XYZ = (0.0, 0.0, 1.0)
DEFAULT_AXIS_XYZ_STR = "0 0 1"


# XML and XACRO Namespaces
# ----------------------------

# Official XACRO namespace URIs
XACRO_URIS = {
    "http://www.ros.org/wiki/xacro",
    "http://wiki.ros.org/xacro",
    "http://ros.org/xacro",
}

# Standard prefix for internal structural processing
XACRO_PREFIX = "xacro:"


# Validation Limits (Sanity Checks)
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


# Numerical Stability (Guardrails)
# ----------------------------

# Small value for floating point comparisons
EPSILON = 1e-9

# Minimum mass in kg to prevent singular matrices in dynamics solvers
MIN_REASONABLE_MASS = 1e-6

# Minimum inertia diagonal value to prevent zero-inertia crashes
MIN_REASONABLE_INERTIA = 1e-9

# Thresholds for inertia calculation fallback and stability
MIN_MASS_STABILITY_THRESHOLD = 0.01  # kg
MIN_INERTIA_STABILITY_VALUE = 1e-6  # kg·m²

# Geometric thresholds
DEGENERATE_VOL_THRESHOLD = 1e-12  # m³
NEGATIVE_INERTIA_THRESHOLD = -1e-06
SYLVESTER_TOLERANCE_EPSILON = 1e-9

# Mesh Validation Thresholds
MESH_PROXIMITY_THRESHOLD = 6
MESH_SLIVER_THRESHOLD = 1000.0
MIN_MESH_AREA = 1e-15


# Joint Dynamics Defaults
# ----------------------------

# Default joint damping (N s / m or N m s / rad)
DEFAULT_JOINT_DAMPING = 0.0

# Default joint friction (N or N m)
DEFAULT_JOINT_FRICTION = 0.0

# Visualization Defaults (Core)
# ----------------------------
DEFAULT_COLOR_RGBA_STR = "0.7 0.7 0.7 1.0"
DEFAULT_MESH_SCALE_STR = "1 1 1"
DEFAULT_GEOMETRY_RADIUS = 0.1
DEFAULT_GEOMETRY_LENGTH = 0.5
