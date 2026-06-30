"""Blender integration layer for LinkForge.

This module contains all Blender-specific logic and UI integration:
- Property Groups: Stored data for robot, link, joint, and sensor settings.
- Operators & Panels: User actions and 3D Viewport sidebar interface.
- Preferences & Handlers: Global configuration and scene-level update logic.
- Visualization: 3D gizmos for physical and kinematic property inspection.
"""

from __future__ import annotations

import sys
from pathlib import Path

# When loaded as a Blender 4.2+ extension, the module name is often nested
# (e.g. bl_ext.user_default.linkforge). To support absolute imports in the source
# tree (e.g., `from linkforge.core import X`) during local development, we alias it here.
if __name__ != "linkforge" and "linkforge" not in sys.modules:
    sys.modules["linkforge"] = sys.modules[__name__]
    sys.modules["linkforge.blender"] = sys.modules[__name__]

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
    # Populate scene properties from modules
    pass
    for module in modules:
        module.register()


def unregister() -> None:
    """Unregister all Blender components."""
    import contextlib

    for module in reversed(modules):
        with contextlib.suppress(Exception):
            module.unregister()


# Entry point for Blender Extension system
if __name__ == "__main__":
    register()
