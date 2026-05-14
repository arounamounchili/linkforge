"""Platform-specific constants for LinkForge in Blender.

This module contains UI defaults, naming conventions, and heuristics
specific to the Blender adapter.
"""

# Object Suffixes for Robot Components
# ----------------------------
SUFFIX_VISUAL = "_visual"
SUFFIX_COLLISION = "_collision"
SUFFIX_SENSOR = "_sensor"

# Metadata Tags (Blender ID Properties)
# ----------------------------
TAG_SOURCE_NAME = "source_name"
TAG_SOURCE_GEOM = "source_geometry_type"
TAG_IMPORTED_SOURCE = "imported_from_source"
TAG_COLLISION_GEOM = "collision_geometry_type"
TAG_SENSOR_TYPE = "sensor_type"

# UI and Visualization Defaults
# ----------------------------
DEFAULT_LINK_GIZMO_SIZE = 0.2
DEFAULT_JOINT_GIZMO_SIZE = 0.2
DEFAULT_SENSOR_GIZMO_SIZE = 0.1
DEFAULT_INERTIA_GIZMO_SIZE = 0.1

# Heuristic Thresholds (Primitive Detection)
# ----------------------------
# Maximum allowed face count for a mesh to be considered for primitive detection
PRIMITIVE_MAX_FACES = 1000

# Tolerance for geometric comparisons (e.g. vertex alignment)
GEOM_TOLERANCE = 1e-4

# LIDAR visualization defaults
DEFAULT_LIDAR_SCAN_COUNT = 640
DEFAULT_LIDAR_HORIZONTAL_FOV = 6.283185  # 2 * PI
