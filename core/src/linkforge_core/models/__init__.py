"""Core data models for robot descriptions.

This sub-package defines the foundational data structures that represent
a robot's physical, kinematic, and sensor properties. These models are
designed to be simulator-agnostic and form the central API for all
LinkForge operations.
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
from .joint import (
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafetyController,
    JointType,
)
from .link import Collision, Inertial, InertiaTensor, Link, Visual
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
    DisabledCollision,
    EndEffector,
    GroupState,
    PassiveJoint,
    PlanningGroup,
    SemanticRobotDescription,
    VirtualJoint,
)
from .transmission import (
    HardwareInterface,
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
    "HardwareInterface",
    "TransmissionJoint",
    "TransmissionActuator",
    "Transmission",
    # Gazebo
    "GazeboPlugin",
    "GazeboElement",
    # SRDF
    "VirtualJoint",
    "PlanningGroup",
    "GroupState",
    "EndEffector",
    "PassiveJoint",
    "DisabledCollision",
    "SemanticRobotDescription",
]
