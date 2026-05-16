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
    # 1. Standard Production Path (Bundled core)
    # If running as a Blender extension (bl_ext namespace), we map the bundled
    # core into the global 'linkforge.core' namespace so absolute imports work.
    try:
        from . import core as bundled_core  # type: ignore[attr-defined]

        # If we are in a namespace (extensions), alias it to the expected package name
        if __name__.startswith("bl_ext."):
            sys.modules["linkforge.core"] = bundled_core
            # Also alias submodules if needed (recursive mapping might be better but this is usually enough)
            # Actually, Python handles submodules if 'linkforge.core' is set.

        return True
    except ImportError:
        pass

    # 2. Development Fallback: Try to find linkforge.core in the environment
    # This is critical for running tests and mypy in the source layout
    try:
        import linkforge.core  # noqa: F401

        return True
    except ImportError:
        pass

    # 3. Deep Search: Search for core in workspace (fallback for complex envs)
    try:
        current = Path(__file__).resolve()
        for _ in range(10):
            if current.parent == current:
                break
            core_src = current / "core" / "src"
            if (core_src / "linkforge" / "core").is_dir():
                if str(core_src) not in sys.path:
                    sys.path.insert(0, str(core_src))
                try:
                    import linkforge.core  # noqa: F401

                    return True
                except ImportError:
                    break
            current = current.parent
    except Exception:
        pass
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
