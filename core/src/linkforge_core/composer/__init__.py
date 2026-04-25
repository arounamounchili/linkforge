"""Robot assembly and composition tools.

This package provides high-level APIs for building modular robots
by attaching components and programmatically constructing links/joints.
"""

from .factories import fixed_joint, origin, revolute_joint
from .robot_builder import RobotBuilder

__all__ = ["RobotBuilder", "fixed_joint", "revolute_joint", "origin"]
