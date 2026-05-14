"""Platform-specific constants for LinkForge in Blender.

This module contains UI defaults, naming conventions, and heuristics
specific to the Blender adapter.
"""

# Object Suffixes for Robot Components
# ----------------------------
# Used for naming child objects representing visual and collision geometry
SUFFIX_VISUAL = "_visual"
SUFFIX_COLLISION = "_collision"
SUFFIX_SENSOR = "_sensor"

# Metadata Tags (Blender ID Properties)
# ----------------------------
# These keys are used in object['key'] storage for persistence
TAG_SOURCE_NAME = "source_name"
TAG_SOURCE_GEOM = "source_geometry_type"
TAG_IMPORTED_SOURCE = "imported_from_source"
TAG_COLLISION_GEOM = "collision_geometry_type"
TAG_SENSOR_TYPE = "sensor_type"

# UI and Visualization Defaults
# ----------------------------
# Gizmo sizes for viewport display
DEFAULT_LINK_GIZMO_SIZE = 0.1
DEFAULT_JOINT_GIZMO_SIZE = 0.1
DEFAULT_SENSOR_GIZMO_SIZE = 0.1
DEFAULT_INERTIA_GIZMO_SIZE = 0.1

# Default simplification ratio for generated collision meshes
DEFAULT_COLLISION_QUALITY = 50.0

# Heuristic Thresholds (Primitive Detection)
# ----------------------------
# Maximum allowed face count for a mesh to be considered for primitive detection
PRIMITIVE_MAX_FACES = 1000

# Tolerance for geometric comparisons (e.g. vertex alignment)
GEOM_TOLERANCE = 1e-4
