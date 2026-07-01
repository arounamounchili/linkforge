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

_pkg = __package__ or __name__

# --- Module Identity Resolution ---
# Blender 4.5+ strictly forbids top-level module names in `sys.modules` that are
# not prefixed with the extension's `bl_ext.*` namespace.
# To comply, all source files use relative imports (e.g. `from ..core import X`).
#
# However, in local testing (`pytest`), this causes the `core` symlink to be loaded
# as `linkforge.blender.core`, while tests import it as `linkforge.core`. This creates
# duplicate class objects in memory and breaks `isinstance()` checks.
#
# To fix this, we map `linkforge.blender.core` -> `linkforge.core` ONLY during local tests.
if not _pkg.startswith("bl_ext."):
    try:
        import linkforge.core

        _blender_core_prefix = f"{_pkg}.core"
        sys.modules[_blender_core_prefix] = linkforge.core

        # Map any currently loaded submodules
        for _key, _mod in list(sys.modules.items()):
            if _key.startswith("linkforge.core."):
                _mapped_key = _key.replace("linkforge.core", _blender_core_prefix, 1)
                sys.modules[_mapped_key] = _mod
    except ImportError:
        pass


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
