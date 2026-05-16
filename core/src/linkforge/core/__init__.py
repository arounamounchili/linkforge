"""LinkForge Core Library.

The platform-independent heart of the LinkForge project, providing a
unified Intermediate Representation (IR) for robotics. This core library
handles the "Robotics Intelligence" isolated from design tools.

As the "LLVM for Robotics," LinkForge Core provides the essential IR and tools
for parsing, generating, and validating robot descriptions across formats.
"""

from __future__ import annotations

# Versioning
__version__ = "1.3.0"  # x-release-please-version

# Sub-Package Exposure
# (Ensures lf.models, lf.parsers, etc. are accessible via dot-notation)
from . import (
    composer,
    constants,
    exceptions,
    generators,
    models,
    parsers,
    physics,
    validation,
)

# Base Architecture (Interfaces and Resolvers)  # noqa: ERA001
from .base import (
    FileSystemResolver,
    IResourceResolver,
    NetworkResolver,
    RobotGenerator,
    RobotParser,
)

# Composer API (The "LinkForge way" to build robots)  # noqa: ERA001
from .composer import (
    LinkBuilder,
    RobotBuilder,
    box,
    cylinder,
    mesh,
    sphere,
)

# Core Infrastructure (Constants and Exceptions)  # noqa: ERA001
from .constants import (
    DEFAULT_GRAVITY,
    DEFAULT_LINK_MASS,
    EPSILON,
    PI,
)
from .exceptions import (
    LinkForgeError,
    RobotGeneratorError,
    RobotMathError,
    RobotModelError,
    RobotParserError,
    RobotParserIOError,
    RobotPhysicsError,
    RobotSecurityError,
    RobotValidationError,
    RobotXacroError,
    RobotXacroExpressionError,
    RobotXacroRecursionError,
    ValidationErrorCode,
    XacroDetectedError,
)

# Generators and Functional I/O  # noqa: ERA001
from .generators import (
    RobotXMLGenerator,
    SRDFGenerator,
    URDFGenerator,
    XACROGenerator,
)
from .io import (
    read_srdf,
    read_urdf,
    read_xacro,
    validate_robot,
    write_srdf,
    write_urdf,
    write_xacro,
)
from .logging_config import get_logger, setup_logging

# IR Models (Entities, Sensors and Hardware)  # noqa: ERA001
from .models import (
    Box,
    CameraInfo,
    Chain,
    Collision,
    CollisionPair,
    Color,
    ContactInfo,
    Cylinder,
    EndEffector,
    ForceTorqueInfo,
    GazeboElement,
    GazeboPlugin,
    Geometry,
    GeometryType,
    GPSInfo,
    GroupState,
    IMUInfo,
    Inertial,
    InertiaTensor,
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointProperty,
    JointSafetyController,
    JointType,
    KinematicGraph,
    LidarInfo,
    Link,
    LinkPhysics,
    LinkSphereApproximation,
    Material,
    Mesh,
    PassiveJoint,
    PlanningGroup,
    Robot,
    Ros2Control,
    Ros2ControlJoint,
    Ros2ControlSensor,
    SemanticRobotDescription,
    Sensor,
    SensorNoise,
    SensorType,
    Sphere,
    SrdfSphere,
    Transform,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    TransmissionType,
    Vector3,
    VirtualJoint,
    Visual,
)

# Parsers and Resolvers  # noqa: ERA001
from .parsers import (
    RobotXMLParser,
    SRDFParser,
    URDFParser,
    XACROParser,
    XacroResolver,
    clear_xacro_cache,
)

# Processing and Validation (Physics and Checks)  # noqa: ERA001
from .physics import (
    calculate_inertia,
    validate_mesh_topology,
)
from .validation import (
    RobotValidator,
    Severity,
    ValidationCheck,
    ValidationIssue,
    ValidationResult,
)

__all__ = [
    # Sub-Packages
    "composer",
    "constants",
    "exceptions",
    "generators",
    "models",
    "parsers",
    "physics",
    "validation",
    # Functional API
    "read_urdf",
    "write_urdf",
    "read_xacro",
    "write_xacro",
    "read_srdf",
    "write_srdf",
    "validate_robot",
    # Core Models
    "Robot",
    "Link",
    "Joint",
    "Visual",
    "Collision",
    "Inertial",
    "InertiaTensor",
    "KinematicGraph",
    "LinkPhysics",
    "Transform",
    "Vector3",
    "Geometry",
    "GeometryType",
    "Material",
    "Color",
    "Box",
    "Cylinder",
    "Sphere",
    "Mesh",
    # Sensors & Hardware
    "Sensor",
    "SensorType",
    "SensorNoise",
    "LidarInfo",
    "CameraInfo",
    "IMUInfo",
    "GPSInfo",
    "ContactInfo",
    "ForceTorqueInfo",
    "Transmission",
    "TransmissionType",
    "TransmissionJoint",
    "TransmissionActuator",
    "Ros2Control",
    "Ros2ControlJoint",
    "Ros2ControlSensor",
    "GazeboPlugin",
    "GazeboElement",
    # Semantic API
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
    # Properties & Dynamics
    "JointLimits",
    "JointDynamics",
    "JointMimic",
    "JointSafetyController",
    "JointCalibration",
    "JointType",
    # IO (Parsers & Generators)  # noqa: ERA001
    "URDFParser",
    "XACROParser",
    "SRDFParser",
    "RobotXMLParser",
    "XacroResolver",
    "clear_xacro_cache",
    "URDFGenerator",
    "XACROGenerator",
    "SRDFGenerator",
    "RobotXMLGenerator",
    "RobotParser",
    "RobotGenerator",
    "IResourceResolver",
    "FileSystemResolver",
    "NetworkResolver",
    # Composer API
    "RobotBuilder",
    "LinkBuilder",
    "box",
    "cylinder",
    "sphere",
    "mesh",
    # Validation & Physics
    "RobotValidator",
    "ValidationResult",
    "ValidationIssue",
    "Severity",
    "ValidationErrorCode",
    "ValidationCheck",
    "calculate_inertia",
    "validate_mesh_topology",
    # Exceptions
    "LinkForgeError",
    "RobotModelError",
    "RobotParserError",
    "RobotParserIOError",
    "RobotGeneratorError",
    "RobotValidationError",
    "RobotPhysicsError",
    "RobotMathError",
    "RobotSecurityError",
    "RobotXacroError",
    "RobotXacroRecursionError",
    "RobotXacroExpressionError",
    "XacroDetectedError",
    # Constants
    "PI",
    "EPSILON",
    "DEFAULT_GRAVITY",
    "DEFAULT_LINK_MASS",
    # Logging
    "get_logger",
    "setup_logging",
]
