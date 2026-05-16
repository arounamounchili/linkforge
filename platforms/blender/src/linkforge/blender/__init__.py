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


# --- Health Checks & Dev Mode ---
def _check_health() -> bool:
    """Verify the extension environment and dependencies."""
    # In production, core is bundled. In dev, we use mypy/PYTHONPATH.
    try:
        from . import core  # type: ignore[attr-defined] # noqa: F401

        return True
    except ImportError:
        # Fallback for dev mode
        try:
            import linkforge.core  # noqa: F401

            return True
        except ImportError:
            return False


_HEALTHY = _check_health()

from . import handlers, operators, panels, preferences, properties  # noqa: E402
from .visualization import inertia_gizmos, joint_gizmos  # noqa: E402

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
