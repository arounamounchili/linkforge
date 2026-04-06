"""Blender UI Panels for LinkForge.

This package provides the 3D Viewport sidebar panels for:
- Forge: Project management and URDF/XACRO import.
- Link & Joint: Physical and kinematic property configuration.
- Sensor, Control, & Transmission: Component and hardware settings.
- Robot & Export: Global metadata and unified export commands.
"""

from __future__ import annotations

from . import (
    control_panel,
    export_panel,
    forge_panel,
    joint_panel,
    link_panel,
    robot_panel,
    sensor_panel,
)

# Module list for registration
modules = [
    forge_panel,
    link_panel,
    joint_panel,
    sensor_panel,
    control_panel,
    robot_panel,
    export_panel,
]


def register() -> None:
    """Register all panels."""
    for module in modules:
        module.register()


def unregister() -> None:
    """Unregister all panels."""
    for module in reversed(modules):
        module.unregister()


__all__ = [
    "link_panel",
    "joint_panel",
    "sensor_panel",
    "control_panel",
    "robot_panel",
    "export_panel",
    "register",
    "unregister",
]
