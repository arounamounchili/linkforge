"""LinkForge Core Library.

The platform-independent heart of the LinkForge project, providing a
unified Intermediate Representation (IR) for robotics. This core library
handles the "Robotics Intelligence" isolated from design tools.

Modules:
    composer: High-level API for assembling robots via RobotBuilder.
    generators: URDF, XACRO, and SRDF file generation.
    models: Core data structures for robots, links, joints, and geometry.
    parsers: Lossless URDF, XACRO, and SRDF file parsing.
    physics: Scientifically grounded inertia and mass calculations.
    validation: Multi-phase hardened validation of robot descriptions.
"""

from __future__ import annotations

__version__ = "1.3.0"  # x-release-please-version

from . import composer, generators, models, parsers, physics, validation
from .base import FileSystemResolver, RobotGenerator, RobotParser
from .composer import LinkBuilder, RobotBuilder
from .exceptions import (
    LinkForgeError,
    RobotGeneratorError,
    RobotMathError,
    RobotModelError,
    RobotParserError,
    RobotPhysicsError,
    RobotValidationError,
    ValidationErrorCode,
    XacroDetectedError,
)
from .generators import SRDFGenerator, URDFGenerator, XACROGenerator
from .logging_config import get_logger
from .models import (
    Box,
    CameraInfo,
    Color,
    ContactInfo,
    Cylinder,
    ForceTorqueInfo,
    GazeboElement,
    GazeboPlugin,
    Geometry,
    GPSInfo,
    IMUInfo,
    Inertial,
    InertiaTensor,
    Joint,
    JointType,
    LidarInfo,
    Link,
    LinkPhysics,
    Material,
    Mesh,
    Robot,
    Ros2Control,
    Ros2ControlJoint,
    Sensor,
    SensorNoise,
    SensorType,
    Sphere,
    Transform,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    TransmissionType,
    Vector3,
)
from .parsers import (
    SRDFParser,
    URDFParser,
    XACROParser,
    XacroResolver,
    clear_xacro_cache,
)
from .physics import (
    calculate_box_inertia,
    calculate_cylinder_inertia,
    calculate_inertia,
    calculate_mesh_inertia_approximation,
    calculate_mesh_inertia_from_triangles,
    calculate_sphere_inertia,
    validate_mesh_topology,
)
from .utils.dict_utils import filter_items_by_name
from .utils.math_utils import clean_float, format_float, format_vector
from .utils.string_utils import (
    format_scientific,
    parse_scientific,
    sanitize_name,
)
from .validation import RobotValidator, Severity, ValidationResult, find_sandbox_root

__all__ = [
    "models",
    "physics",
    "parsers",
    "composer",
    "validation",
    "Robot",
    "Link",
    "Joint",
    "Transform",
    "InertiaTensor",
    "Inertial",
    "RobotValidator",
    "RobotGenerator",
    "RobotParser",
    "FileSystemResolver",
    "XacroResolver",
    "clear_xacro_cache",
    "find_sandbox_root",
    "Sensor",
    "Transmission",
    "Ros2Control",
    "LinkBuilder",
    "RobotBuilder",
    "get_logger",
    "Box",
    "Color",
    "Cylinder",
    "Geometry",
    "Material",
    "Mesh",
    "Sphere",
    "Vector3",
    "JointType",
    "SensorType",
    "TransmissionType",
    "Ros2ControlJoint",
    "GazeboPlugin",
    "GazeboElement",
    "CameraInfo",
    "ContactInfo",
    "ForceTorqueInfo",
    "GPSInfo",
    "IMUInfo",
    "LidarInfo",
    "SensorNoise",
    "TransmissionJoint",
    "TransmissionActuator",
    "sanitize_name",
    "clean_float",
    "format_scientific",
    "parse_scientific",
    "validate_mesh_topology",
    "calculate_inertia",
    "calculate_box_inertia",
    "calculate_cylinder_inertia",
    "calculate_sphere_inertia",
    "calculate_mesh_inertia_approximation",
    "calculate_mesh_inertia_from_triangles",
    "filter_items_by_name",
    "URDFParser",
    "XACROParser",
    "SRDFParser",
    "URDFGenerator",
    "XACROGenerator",
    "SRDFGenerator",
    "format_float",
    "format_vector",
    "LinkForgeError",
    "RobotGeneratorError",
    "RobotModelError",
    "RobotParserError",
    "RobotPhysicsError",
    "RobotValidationError",
    "RobotMathError",
    "ValidationErrorCode",
    "ValidationResult",
    "Severity",
    "XacroDetectedError",
    "LinkPhysics",
]
