"""Robot assembly and composition tools.

This package provides high-level APIs for building modular robots
by attaching components and programmatically constructing links/joints.
"""

from ..models.geometry import Transform, Vector3
from ..models.joint import JointType
from .robot_assembly import RobotAssembly


def fixed_joint() -> JointType:
    """Shortcut for JointType.FIXED."""
    return JointType.FIXED


def revolute_joint() -> JointType:
    """Shortcut for JointType.REVOLUTE."""
    return JointType.REVOLUTE


def origin(
    xyz: tuple[float, float, float] = (0, 0, 0), rpy: tuple[float, float, float] = (0, 0, 0)
) -> Transform:
    """Shortcut to create a Transform origin."""
    return Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy))


__all__ = ["RobotAssembly", "FixedJoint", "RevoluteJoint", "Origin"]
