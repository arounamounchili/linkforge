"""Platform-specific constants and naming conventions for the LinkForge Blender adapter.

This module defines the structural glue between Blender's internal data
structures and the LinkForge IR, including UI defaults, object suffixes,
and property registration keys.

Core Components:
    - Naming Conventions: Standardized suffixes for visual, collision, and sensor objects.
    - Property Keys: Registration identifiers for custom Blender property groups.
    - UI Defaults: Viewport display sizes and asynchronous builder settings.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

# Addon metadata
ADDON_ID_DEFAULT: Final[str] = "linkforge"

# Object Suffixes for Robot Components
# ----------------------------
# Used for naming child objects representing visual and collision geometry
SUFFIX_VISUAL: Final[str] = "_visual"
SUFFIX_COLLISION: Final[str] = "_collision"
SUFFIX_SENSOR: Final[str] = "_sensor"

# Geometry Purposes
PURPOSE_VISUAL: Final[str] = "visual"
PURPOSE_COLLISION: Final[str] = "collision"

# Metadata Tags (Blender ID Properties)
# ----------------------------
# These keys are used in object['key'] storage for persistence
TAG_SOURCE_NAME: Final[str] = "source_name"
TAG_SOURCE_GEOM: Final[str] = "source_geometry_type"
TAG_IMPORTED_SOURCE: Final[str] = "imported_from_source"
TAG_COLLISION_GEOM: Final[str] = "collision_geometry_type"
TAG_SENSOR_TYPE: Final[str] = "sensor_type"

# Blender Property Group Identifiers
# ----------------------------
# Used for registration and access via bpy.types.Object.linkforge_...
PROP_LINK: Final[str] = "linkforge"
PROP_JOINT: Final[str] = "linkforge_joint"
PROP_SENSOR: Final[str] = "linkforge_sensor"
PROP_TRANSMISSION: Final[str] = "linkforge_transmission"
PROP_CONTROL: Final[str] = "linkforge_control"
PROP_ROBOT: Final[str] = "linkforge_robot"
PROP_VALIDATION: Final[str] = "linkforge_validation"

# File Formats and Extensions
# ----------------------------
FORMAT_STL: Final[str] = "STL"
FORMAT_OBJ: Final[str] = "OBJ"
FORMAT_GLB: Final[str] = "GLB"

# Default Names
# ----------------------------
DEFAULT_LINK_NAME: Final[str] = "base_link"
DEFAULT_JOINT_NAME: Final[str] = "joint"
DEFAULT_SENSOR_NAME: Final[str] = "sensor"
DEFAULT_ROBOT_NAME: Final[str] = "robot"

# UI and Visualization Defaults
# ----------------------------
# Gizmo sizes for viewport display
DEFAULT_LINK_GIZMO_SIZE: Final[float] = 0.1
DEFAULT_JOINT_GIZMO_SIZE: Final[float] = 0.1
DEFAULT_SENSOR_GIZMO_SIZE: Final[float] = 0.1
DEFAULT_INERTIA_GIZMO_SIZE: Final[float] = 0.1

# Default simplification ratio for generated collision meshes
DEFAULT_COLLISION_QUALITY: Final[float] = 50.0

# Heuristic Thresholds (Primitive Detection)
# ----------------------------
# Maximum allowed face count for a mesh to be considered for primitive detection
PRIMITIVE_MAX_FACES: Final[int] = 1000

# Tolerance for geometric comparisons (e.g. vertex alignment)
GEOM_TOLERANCE: Final[float] = 1e-4

# Automation and Logic Defaults
# ----------------------------
GEOM_AUTO: Final[str] = "auto"


@dataclass(frozen=True)
class PrimitiveDetectionConfig:
    """Configuration for primitive shape detection from Blender meshes.

    Start with specific vertex counts and use bounding box ratios to
    fuzzy match geometry.
    """

    # Cube detection - exact match required
    cube_vert_count: int = 8  # Cubes always have 8 vertices
    cube_face_count: int = 6  # Cubes always have 6 faces
    cube_verts_per_face: int = 4  # Each face has 4 vertices

    # Sphere detection (UV Sphere with various subdivision levels)
    # Based on Blender UV Sphere: 16 segments x 8 rings = 240 verts (minimum acceptable)
    sphere_min_verts: int = 240  # Minimum for low-poly spheres
    sphere_max_verts: int = 1000  # Maximum for high-poly spheres
    sphere_min_faces: int = 240  # Minimum face count
    sphere_max_faces: int = 1000  # Maximum face count
    # Empirically determined: 0.9 allows for minor mesh imperfections
    sphere_uniformity_tolerance: float = 0.9

    # Cylinder detection (default 32 vertices, supports 16-64 range)
    cylinder_min_verts: int = 32  # Minimum vertices (16-sided cylinder minimum)
    cylinder_max_verts: int = 128  # Maximum vertices (64-sided cylinder maximum)
    cylinder_min_faces: int = 18  # 16 vertices = 16 side faces + 2 caps
    cylinder_max_faces: int = 66  # 64 vertices = 64 side faces + 2 caps
    cylinder_base_tolerance: float = 0.9  # XY ratio must be > 0.9 for circular base
    cylinder_height_min_ratio: float = 0.9  # Z/XY ratio boundaries
    cylinder_height_max_ratio: float = 1.1


# Default primitive detection configuration
DEFAULT_PRIMITIVE_CONFIG: Final[PrimitiveDetectionConfig] = PrimitiveDetectionConfig()
