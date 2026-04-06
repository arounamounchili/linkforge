"""Blender Operators for LinkForge actions.

This package provide commands for:
- Link & Joint Management: Marking objects and resetting properties.
- Component Setup: Adding sensors, transmissions, and hardware interfaces.
- Data Processing: Importing and exporting URDF, XACRO, and meshes.
"""

from __future__ import annotations

from . import control_ops, export_ops, import_ops, joint_ops, link_ops, sensor_ops, transmission_ops

# Module list for registration
modules = [
    link_ops,
    joint_ops,
    sensor_ops,
    transmission_ops,
    control_ops,
    import_ops,
    export_ops,
]


def register() -> None:
    """Register all operators."""
    for module in modules:
        module.register()


def unregister() -> None:
    """Unregister all operators."""
    for module in reversed(modules):
        module.unregister()


__all__ = [
    "link_ops",
    "joint_ops",
    "sensor_ops",
    "transmission_ops",
    "import_ops",
    "export_ops",
    "register",
    "unregister",
]
