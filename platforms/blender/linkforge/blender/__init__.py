"""Blender integration layer for LinkForge.

This module contains all Blender-specific logic and UI integration:
- Property Groups: Stored data for robot, link, joint, and sensor settings.
- Operators & Panels: User actions and 3D Viewport sidebar interface.
- Preferences & Handlers: Global configuration and scene-level update logic.
- Visualization: 3D gizmos for physical and kinematic property inspection.
"""

from __future__ import annotations

from . import handlers, operators, panels, preferences, properties
from .visualization import inertia_gizmos, joint_gizmos

# Registration order matters: properties first, then operators, then panels, then gizmos
modules = [
    properties,
    preferences,
    operators,
    panels,
    joint_gizmos,
    inertia_gizmos,
    handlers,
]


def register() -> None:
    """Register all Blender components."""
    for module in modules:
        module.register()


def unregister() -> None:
    """Unregister all Blender components."""
    import contextlib

    for module in reversed(modules):
        with contextlib.suppress(Exception):
            module.unregister()


__all__ = [
    "properties",
    "preferences",
    "operators",
    "panels",
    "joint_gizmos",
    "register",
    "unregister",
]
