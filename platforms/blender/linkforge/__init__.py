"""LinkForge - Professional URDF/XACRO Exporter for Blender.

Convert 3D robot models to standard URDF/XACRO files for robotics simulation and control.

This is a Blender Extension compatible with Blender 4.2+.
Metadata is defined in blender_manifest.toml at the root of the extension.
"""

from __future__ import annotations

# Blender Extension Entry Point
import sys
from pathlib import Path

# Add the extension root and wheels to sys.path
# This ensures bundled linkforge_core and dependencies are found both
# in packaged extensions and when running from source.
EXT_ROOT = Path(__file__).parent.resolve()
if str(EXT_ROOT) not in sys.path:
    # Insert at index 1 to respect priority but stay below the primary script path
    sys.path.insert(1, str(EXT_ROOT))

WHEELS_DIR = EXT_ROOT / "wheels"
if WHEELS_DIR.exists() and str(WHEELS_DIR) not in sys.path:
    sys.path.append(str(WHEELS_DIR))

# --- Namespace Bridge ---
# Blender Extensions use the 'bl_ext' namespace. To support project-wide
# interoperability and legacy scripts, we bridge the 'linkforge' alias.
if "linkforge" not in sys.modules:
    sys.modules["linkforge"] = sys.modules[__name__]


# --- Health Checks ---
def _check_health() -> bool:
    """Verify the extension environment and dependencies."""
    try:
        import linkforge_core  # noqa: F401

        return True
    except ImportError as e:
        print(f"LinkForge Initialization Warning: Core dependencies not found ({e}).")
        print("If this is a fresh install, please ensure the extension is correctly enabled.")
        return False


_check_health()

# Import blender module if bpy is available (inside Blender)
try:
    import bpy

    IS_BLENDER = True
except ImportError:
    IS_BLENDER = False

if IS_BLENDER:
    from . import blender
else:
    # Handle environment where bpy is not available (e.g., CI, non-Blender python)
    # But for Mypy (with fake-bpy-module), we don't want to see this redefinition
    import typing

    if not typing.TYPE_CHECKING:
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
