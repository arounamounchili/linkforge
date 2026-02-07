"""Blender integration layer for LinkForge.

This module contains all Blender-specific code:
- Property Groups: Store data on Blender objects
- Operators: User actions/commands
- Panels: User interface
"""

from __future__ import annotations

from . import operators, panels, preferences, properties
from .visualization import inertia_gizmos, joint_gizmos

# Registration order matters: properties first, then operators, then panels, then gizmos
modules = [
    properties,
    preferences,
    operators,
    panels,
    joint_gizmos,
    inertia_gizmos,
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
