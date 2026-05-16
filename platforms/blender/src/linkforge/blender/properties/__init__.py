"""Blender Property Groups for LinkForge.

Property groups store data on Blender objects and scenes:
- Robot & Validation: Global settings and diagnostic results.
- Link & Joint: Core kinematic and physical properties.
- Sensor, Transmission, & Control: Component-specific hardware settings.
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


def register() -> None:
    """Register all property groups and patch global types."""
    for module in modules:
        module.register()


def unregister() -> None:
    """Unregister all property groups and unpatch global types."""
    import contextlib

    import bpy

    from ..constants import (
        PROP_JOINT,
        PROP_LINK,
        PROP_SENSOR,
        PROP_TRANSMISSION,
    )

    # 1. Unpatch global types first to break references
    obj_props = [PROP_LINK, PROP_JOINT, PROP_SENSOR, PROP_TRANSMISSION]
    scene_props = [PROP_LINK]

    for p in obj_props:
        with contextlib.suppress(AttributeError):
            delattr(bpy.types.Object, p)
    for p in scene_props:
        with contextlib.suppress(AttributeError):
            delattr(bpy.types.Scene, p)

    # 2. Unregister classes in reverse order
    for module in reversed(modules):
        with contextlib.suppress(Exception):
            module.unregister()
