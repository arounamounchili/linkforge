"""Robot assembly and composition tools.

This package provides high-level APIs for building modular robots
by programmatically constructing links, joints, and semantic data.
"""

from .robot_builder import RobotBuilder, box, cylinder, mesh, sphere

__all__ = ["RobotBuilder", "box", "cylinder", "sphere", "mesh"]
