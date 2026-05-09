import os
from unittest.mock import MagicMock

import pytest

# Relative import from the mock environment package
from .mock_bpy_env import setup_mock_bpy  # noqa: E402

# -----------------------------------------------------------------------------
# BPY Mocking (Global)
# -----------------------------------------------------------------------------
# We initialize the mock once at the module level because Blender's RNA
# and property registration system is effectively global.

# Check if we are running inside real Blender
try:
    import bpy

    is_real_blender = hasattr(bpy, "app") and not isinstance(bpy.app, MagicMock)
except (ImportError, AttributeError):
    is_real_blender = False

if not is_real_blender:
    bpy = setup_mock_bpy()
    # Force registration of linkforge properties in the mock environment
    import linkforge.blender

    linkforge.blender.register()
else:
    HAS_BPY = True


@pytest.fixture
def blender_context():
    """Returns the Blender context adapter."""
    import bpy
    from linkforge.blender.adapters.context import BlenderContext

    return BlenderContext(bpy)


@pytest.fixture
def scene(blender_context):
    """Returns the active scene."""
    return blender_context.scene


@pytest.fixture(autouse=True)
def clean_scene(blender_context):
    """Automatically cleans the scene before each test."""

    # Clear objects list (real bpy collections don't have .clear())
    if hasattr(blender_context.scene, "objects"):
        objs = blender_context.scene.objects
        if hasattr(objs, "clear") and not is_real_blender:
            objs.clear()
        else:
            # Real Blender removal
            import bpy

            for obj in list(objs):
                bpy.data.objects.remove(obj, do_unlink=True)

    # Clear architectural statistics cache for test isolation
    os.environ["LINKFORGE_DISABLE_CACHE"] = "1"
    from linkforge.blender.utils.scene_utils import clear_stats_cache

    clear_stats_cache()
    yield
