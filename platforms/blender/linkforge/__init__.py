"""LinkForge - Professional URDF/XACRO Exporter for Blender.

Convert 3D robot models to standard URDF/XACRO files for robotics simulation and control.

This is a Blender Extension compatible with Blender 4.2+.
Metadata is defined in blender_manifest.toml at the root of the extension.
"""

from __future__ import annotations

# Blender Extension Entry Point
# Metadata is defined in blender_manifest.toml

# Package version and metadata
__version__ = "1.2.0"
__all__ = ["register", "unregister"]

# Import blender module if bpy is available (inside Blender)
try:
    import bpy

    from . import blender
except ImportError:
    # Running outside Blender (e.g. for testing core logic or generating docs)
    bpy = None
    blender = None


def register() -> None:
    """Register the extension with Blender.

    This function is called when the extension is enabled.
    It registers all operators, panels, property groups, and other Blender types.
    """
    # Register Blender components
    blender.register()


def unregister() -> None:
    """Unregister the extension from Blender.

    This function is called when the extension is disabled.
    It unregisters all operators, panels, property groups, and other Blender types.
    """
    # Unregister Blender components
    blender.unregister()


# Entry point for Blender Extension system
if __name__ == "__main__":
    register()
