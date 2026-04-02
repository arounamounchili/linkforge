"""Factory functions for the LinkForge Composer DSL.

These helpers provide an ergonomic, high-level interface for creating
robot models during assembly, avoiding the need for deep model imports.
"""

from __future__ import annotations

from ..models.geometry import Transform, Vector3
from ..models.joint import JointType


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
