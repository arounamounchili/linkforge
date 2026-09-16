"""Robot assembly and composition tools.

This package provides high-level APIs for building modular robots
by programmatically constructing links, joints, and semantic data.
"""

from .helpers import box, cylinder, mesh, sphere
from .interfaces import IComposer
from .link_builder import LinkBuilder
from .robot_builder import RobotBuilder

__all__ = ["IComposer", "RobotBuilder", "LinkBuilder", "box", "cylinder", "sphere", "mesh"]
