"""LinkForge - Professional URDF/XACRO Exporter for Blender.

Convert 3D robot models to standard URDF/XACRO files for robotics simulation and control.

This is a Blender Extension compatible with Blender 4.2+.
Metadata is defined in blender_manifest.toml at the root of the extension.
"""

from __future__ import annotations

# Blender Extension Entry Point
import sys
from pathlib import Path


# --- Health Checks ---
def _check_health() -> bool:
    """Verify the extension environment and dependencies.

    Supports both production (bundled .zip) and development (workspace clone) setups.
    """
    # 1. Standard Production Path (installed as a package)
    try:
        import linkforge_core  # noqa: F401

        return True
    except ImportError:
        pass

    # 2. Development Fallback: Search upwards for the core library in the workspace
    try:
        current = Path(__file__).resolve()
        # Search up to 8 levels (sufficient for standard repo structures)
        for _ in range(8):
            if current.parent == current:  # Reached filesystem root
                break

            core_src = current / "core" / "src"
            if (core_src / "linkforge_core").is_dir():
                if str(core_src) not in sys.path:
                    sys.path.insert(0, str(core_src))

                try:
                    import linkforge_core  # noqa: F401

                    print(f"LinkForge: Dev mode enabled (Core loaded from {core_src})")
                    return True
                except ImportError:
                    break
            current = current.parent
    except Exception:
        pass

    # 3. Dependency Diagnostics (for developers running from source)
    try:
        import yaml  # noqa: F401
    except ImportError:
        print("LinkForge Warning: 'PyYAML' dependency is missing. Please install it.")

    # 4. Final failure reporting
    print(f"LinkForge Error: 'linkforge_core' not found (Loaded from: {Path(__file__).resolve()})")
    print("Action Required: Please ensure the extension is correctly installed.")
    return False


if _check_health():
    # Import blender module if bpy is available and seems like real Blender
    try:
        import bpy

        # Harder check: fake-bpy-module doesn't always have app or version
        IS_BLENDER = hasattr(bpy, "app") and hasattr(bpy.app, "version")
    except ImportError:
        IS_BLENDER = False

    if IS_BLENDER:
        from . import blender

        def register() -> None:
            """Register the extension with Blender."""
            blender.register()

        def unregister() -> None:
            """Unregister the extension from Blender."""
            blender.unregister()

        # Entry point for Blender Extension system
        if __name__ == "__main__":
            register()
    else:
        # Handle non-Blender environment
        import typing

        if not typing.TYPE_CHECKING:
            bpy = None
            blender = None
