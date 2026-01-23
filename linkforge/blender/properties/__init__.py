"""Blender Property Groups for LinkForge.

Property groups store data on Blender objects and scenes:
- LinkPropertyGroup: Stored on Objects (link properties)
- JointPropertyGroup: Stored on Empty objects (joint properties)
- RobotPropertyGroup: Stored on Scene (global robot settings)
"""

from __future__ import annotations

from . import (
    control_props,
    joint_props,
    link_props,
    robot_props,
    sensor_props,
    transmission_props,
    validation_props,
)

# Module list for registration
modules = [
    link_props,
    joint_props,
    sensor_props,
    transmission_props,
    control_props,
    robot_props,
    validation_props,
]


def register():
    """Register all property groups."""
    for module in modules:
        module.register()


def unregister():
    """Unregister all property groups."""
    for module in reversed(modules):
        module.unregister()


__all__ = [
    "link_props",
    "joint_props",
    "sensor_props",
    "transmission_props",
    "robot_props",
    "validation_props",
    "register",
    "unregister",
]
