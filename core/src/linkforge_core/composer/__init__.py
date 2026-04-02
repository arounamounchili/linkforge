"""Robot assembly and composition tools.

This package provides high-level APIs for building modular robots
by attaching components and programmatically constructing links/joints.
"""

from .factories import fixed_joint, origin, revolute_joint
from .robot_assembly import RobotAssembly

__all__ = ["RobotAssembly", "fixed_joint", "revolute_joint", "origin"]
