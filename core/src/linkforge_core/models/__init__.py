"""Core data models for robot descriptions.

This sub-package defines the foundational data structures that represent
a robot's physical, kinematic, sensor, and semantic (SRDF) properties.
These models are designed to be simulator-agnostic and form the central API
for all LinkForge operations.
"""

from .gazebo import GazeboElement, GazeboPlugin
from .geometry import (
    Box,
    Cylinder,
    Geometry,
    GeometryType,
    Mesh,
    Sphere,
    Transform,
    Vector3,
)
from .graph import KinematicGraph
from .joint import (
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafetyController,
    JointType,
)
from .link import Collision, Inertial, InertiaTensor, Link, LinkPhysics, Visual
from .material import Color, Material
from .robot import Robot
from .ros2_control import Ros2Control, Ros2ControlJoint
from .sensor import (
    CameraInfo,
    ContactInfo,
    ForceTorqueInfo,
    GPSInfo,
    IMUInfo,
    LidarInfo,
    Sensor,
    SensorNoise,
    SensorType,
)
from .srdf import (
    Chain,
    CollisionPair,
    EndEffector,
    GroupState,
    JointProperty,
    LinkSphereApproximation,
    PassiveJoint,
    PlanningGroup,
    SemanticRobotDescription,
    SrdfSphere,
    VirtualJoint,
)
from .transmission import (
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    TransmissionType,
)

__all__ = [
    # Geometry
    "Vector3",
    "Transform",
    "GeometryType",
    "Box",
    "Cylinder",
    "Sphere",
    "Mesh",
    "Geometry",
    # Material
    "Color",
    "Material",
    # Link
    "InertiaTensor",
    "Inertial",
    "Visual",
    "Collision",
    "Link",
    "LinkPhysics",
    # Joint
    "JointType",
    "JointLimits",
    "JointDynamics",
    "JointMimic",
    "JointSafetyController",
    "JointCalibration",
    "Joint",
    # Robot
    "Robot",
    # ros2_control
    "Ros2Control",
    "Ros2ControlJoint",
    # Sensor
    "SensorType",
    "SensorNoise",
    "CameraInfo",
    "LidarInfo",
    "IMUInfo",
    "GPSInfo",
    "ContactInfo",
    "ForceTorqueInfo",
    "Sensor",
    # Transmission
    "TransmissionType",
    "TransmissionJoint",
    "TransmissionActuator",
    "Transmission",
    # Gazebo
    "GazeboPlugin",
    "GazeboElement",
    # SRDF
    "Chain",
    "CollisionPair",
    "EndEffector",
    "GroupState",
    "JointProperty",
    "LinkSphereApproximation",
    "PassiveJoint",
    "PlanningGroup",
    "SemanticRobotDescription",
    "SrdfSphere",
    "VirtualJoint",
    "KinematicGraph",
]
