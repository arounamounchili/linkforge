"""Visualization adapters package for LinkForge Blender integration.

This package contains specialized viewport rendering adapters (gizmos) that
visualize robot properties directly in the Blender 3D viewport.
"""

from __future__ import annotations

from . import inertia_gizmos, joint_gizmos

modules = [
    joint_gizmos,
    inertia_gizmos,
]


def register() -> None:
    """Register all visualization gizmos."""
    for module in modules:
        module.register()


def unregister() -> None:
    """Unregister all visualization gizmos."""
    for module in reversed(modules):
        module.unregister()
