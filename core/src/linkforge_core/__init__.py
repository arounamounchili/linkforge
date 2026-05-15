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
from .base import (
    FileSystemResolver as FileSystemResolver,
)
from .base import (
    IResourceResolver as IResourceResolver,
)
from .base import (
    RobotGenerator as RobotGenerator,
)
from .base import (
    RobotParser as RobotParser,
)
from .composer import LinkBuilder as LinkBuilder
from .composer import RobotBuilder as RobotBuilder
from .exceptions import (
    LinkForgeError as LinkForgeError,
)
from .exceptions import (
    RobotGeneratorError as RobotGeneratorError,
)
from .exceptions import (
    RobotMathError as RobotMathError,
)
from .exceptions import (
    RobotModelError as RobotModelError,
)
from .exceptions import (
    RobotParserError as RobotParserError,
)
from .exceptions import (
    RobotPhysicsError as RobotPhysicsError,
)
from .exceptions import (
    RobotSecurityError as RobotSecurityError,
)
from .exceptions import (
    RobotValidationError as RobotValidationError,
)
from .exceptions import (
    RobotXacroError as RobotXacroError,
)
from .exceptions import (
    RobotXacroExpressionError as RobotXacroExpressionError,
)
from .exceptions import (
    RobotXacroRecursionError as RobotXacroRecursionError,
)
from .exceptions import (
    ValidationErrorCode as ValidationErrorCode,
)
from .exceptions import (
    XacroDetectedError as XacroDetectedError,
)
from .generators import (
    SRDFGenerator as SRDFGenerator,
)
from .generators import (
    URDFGenerator as URDFGenerator,
)
from .generators import (
    XACROGenerator as XACROGenerator,
)
from .logging_config import get_logger as get_logger
from .models import (
    Box as Box,
)
from .models import (
    CameraInfo as CameraInfo,
)
from .models import (
    Chain as Chain,
)
from .models import (
    CollisionPair as CollisionPair,
)
from .models import (
    Color as Color,
)
from .models import (
    ContactInfo as ContactInfo,
)
from .models import (
    Cylinder as Cylinder,
)
from .models import (
    EndEffector as EndEffector,
)
from .models import (
    ForceTorqueInfo as ForceTorqueInfo,
)
from .models import (
    GazeboElement as GazeboElement,
)
from .models import (
    GazeboPlugin as GazeboPlugin,
)
from .models import (
    Geometry as Geometry,
)
from .models import (
    GPSInfo as GPSInfo,
)
from .models import (
    GroupState as GroupState,
)
from .models import (
    IMUInfo as IMUInfo,
)
from .models import (
    Inertial as Inertial,
)
from .models import (
    InertiaTensor as InertiaTensor,
)
from .models import (
    Joint as Joint,
)
from .models import (
    JointProperty as JointProperty,
)
from .models import (
    JointType as JointType,
)
from .models import (
    KinematicGraph as KinematicGraph,
)
from .models import (
    LidarInfo as LidarInfo,
)
from .models import (
    Link as Link,
)
from .models import (
    LinkPhysics as LinkPhysics,
)
from .models import (
    LinkSphereApproximation as LinkSphereApproximation,
)
from .models import (
    Material as Material,
)
from .models import (
    Mesh as Mesh,
)
from .models import (
    PassiveJoint as PassiveJoint,
)
from .models import (
    PlanningGroup as PlanningGroup,
)
from .models import (
    Robot as Robot,
)
from .models import (
    Ros2Control as Ros2Control,
)
from .models import (
    Ros2ControlJoint as Ros2ControlJoint,
)
from .models import (
    SemanticRobotDescription as SemanticRobotDescription,
)
from .models import (
    Sensor as Sensor,
)
from .models import (
    SensorNoise as SensorNoise,
)
from .models import (
    SensorType as SensorType,
)
from .models import (
    Sphere as Sphere,
)
from .models import (
    SrdfSphere as SrdfSphere,
)
from .models import (
    Transform as Transform,
)
from .models import (
    Transmission as Transmission,
)
from .models import (
    TransmissionActuator as TransmissionActuator,
)
from .models import (
    TransmissionJoint as TransmissionJoint,
)
from .models import (
    TransmissionType as TransmissionType,
)
from .models import (
    Vector3 as Vector3,
)
from .models import (
    VirtualJoint as VirtualJoint,
)
from .parsers import (
    SRDFParser as SRDFParser,
)
from .parsers import (
    URDFParser as URDFParser,
)
from .parsers import (
    XACROParser as XACROParser,
)
from .parsers import (
    XacroResolver as XacroResolver,
)
from .parsers import (
    clear_xacro_cache as clear_xacro_cache,
)
from .physics import (
    calculate_box_inertia as calculate_box_inertia,
)
from .physics import (
    calculate_cylinder_inertia as calculate_cylinder_inertia,
)
from .physics import (
    calculate_inertia as calculate_inertia,
)
from .physics import (
    calculate_mesh_inertia_approximation as calculate_mesh_inertia_approximation,
)
from .physics import (
    calculate_mesh_inertia_from_triangles as calculate_mesh_inertia_from_triangles,
)
from .physics import (
    calculate_sphere_inertia as calculate_sphere_inertia,
)
from .physics import (
    validate_mesh_topology as validate_mesh_topology,
)
from .utils.dict_utils import filter_items_by_name as filter_items_by_name
from .utils.math_utils import (
    clean_float as clean_float,
)
from .utils.math_utils import (
    format_float as format_float,
)
from .utils.math_utils import (
    format_vector as format_vector,
)
from .utils.path_utils import (
    get_export_path as get_export_path,
)
from .utils.path_utils import (
    normalize_uri_to_path as normalize_uri_to_path,
)
from .utils.path_utils import (
    resolve_package_path as resolve_package_path,
)
from .utils.string_utils import (
    format_scientific as format_scientific,
)
from .utils.string_utils import (
    parse_scientific as parse_scientific,
)
from .utils.string_utils import (
    sanitize_name as sanitize_name,
)
from .utils.xml_utils import (
    parse_float as parse_float,
)
from .utils.xml_utils import (
    parse_int as parse_int,
)
from .utils.xml_utils import (
    parse_vector3 as parse_vector3,
)
from .utils.xml_utils import (
    serialize_xml as serialize_xml,
)
from .validation import (
    RobotValidator as RobotValidator,
)
from .validation import (
    Severity as Severity,
)
from .validation import (
    ValidationResult as ValidationResult,
)
from .validation import (
    find_sandbox_root as find_sandbox_root,
)

__all__ = [
    # Sub-Packages
    "composer",
    "generators",
    "models",
    "parsers",
    "physics",
    "validation",
    # Core Entities
    "Robot",
    "Link",
    "Joint",
    "Transform",
    "InertiaTensor",
    "Inertial",
    "KinematicGraph",
    "LinkPhysics",
    # Geometry and Materials
    "Box",
    "Cylinder",
    "Sphere",
    "Mesh",
    "Geometry",
    "Material",
    "Color",
    "Vector3",
    # Sensors and Hardware
    "Sensor",
    "SensorType",
    "SensorNoise",
    "LidarInfo",
    "CameraInfo",
    "IMUInfo",
    "GPSInfo",
    "ContactInfo",
    "ForceTorqueInfo",
    # Transmission and Control
    "Transmission",
    "TransmissionType",
    "TransmissionJoint",
    "TransmissionActuator",
    "Ros2Control",
    "Ros2ControlJoint",
    # Gazebo Integration
    "GazeboPlugin",
    "GazeboElement",
    # Semantic API (SRDF)
    "SemanticRobotDescription",
    "PlanningGroup",
    "Chain",
    "GroupState",
    "EndEffector",
    "PassiveJoint",
    "VirtualJoint",
    "CollisionPair",
    "LinkSphereApproximation",
    "SrdfSphere",
    "JointProperty",
    # Robotics Logic (IO & Validation)
    "RobotValidator",
    "RobotGenerator",
    "RobotParser",
    "URDFParser",
    "XACROParser",
    "SRDFParser",
    "URDFGenerator",
    "XACROGenerator",
    "SRDFGenerator",
    "ValidationResult",
    "Severity",
    "XacroResolver",
    "clear_xacro_cache",
    "find_sandbox_root",
    "FileSystemResolver",
    "IResourceResolver",
    # Composer API
    "RobotBuilder",
    "LinkBuilder",
    # Physics and Math
    "calculate_inertia",
    "calculate_box_inertia",
    "calculate_cylinder_inertia",
    "calculate_sphere_inertia",
    "calculate_mesh_inertia_approximation",
    "calculate_mesh_inertia_from_triangles",
    "validate_mesh_topology",
    "clean_float",
    "format_float",
    "format_vector",
    "format_scientific",
    "parse_scientific",
    # Utilities
    "get_logger",
    "sanitize_name",
    "filter_items_by_name",
    "resolve_package_path",
    "normalize_uri_to_path",
    "get_export_path",
    "serialize_xml",
    "parse_float",
    "parse_int",
    "parse_vector3",
    # Exceptions
    "LinkForgeError",
    "RobotGeneratorError",
    "RobotModelError",
    "RobotParserError",
    "RobotPhysicsError",
    "RobotValidationError",
    "RobotMathError",
    "RobotSecurityError",
    "RobotXacroError",
    "RobotXacroRecursionError",
    "RobotXacroExpressionError",
    "ValidationErrorCode",
    "XacroDetectedError",
]
