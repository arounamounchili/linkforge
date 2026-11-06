import contextlib
import os
from unittest.mock import MagicMock

import pytest

# Relative import from the mock environment package
from tests.mock_bpy_env import setup_mock_bpy  # noqa: E402

# BPY Mocking (Global)
# We initialize the mock once at the module level because Blender's RNA
# and property registration system is effectively global.

# Check if we are running inside real Blender
try:
    import bpy

    # Real Blender has a valid binary_path. fake-bpy-module usually has it empty or missing.
    # We also check for 'version' to be sure it's a fully-formed app object.
    is_real_blender = (
        hasattr(bpy, "app")
        and bool(getattr(bpy.app, "binary_path", ""))
        and not isinstance(bpy.app, MagicMock)
    )
except (ImportError, AttributeError):
    is_real_blender = False
    bpy = None

if is_real_blender:
    # Always force registration of linkforge properties to ensure test stability
    import linkforge.blender

    with contextlib.suppress(Exception):
        linkforge.blender.register()
else:
    bpy = setup_mock_bpy()
    # Force registration of linkforge properties in the mock environment
    import linkforge.blender

    linkforge.blender.register()


@pytest.fixture
def blender_context():
    """Returns the Blender context adapter."""
    import sys

    from linkforge.blender.adapters.context import BlenderContext

    return BlenderContext(sys.modules.get("bpy", bpy))


@pytest.fixture
def scene(blender_context):
    """Returns the active scene."""
    return blender_context.scene


@pytest.fixture(autouse=True)
def clean_scene():
    """Automatically cleans the scene before each test."""
    global bpy
    if not is_real_blender:
        bpy = setup_mock_bpy()
        import linkforge.blender

        linkforge.blender.register()

    # Real Blender removal of all objects and underlying data

    # Clear scene-level LinkForge property collections (persisted on bpy.data.scenes)
    scene = bpy.context.scene
    props = getattr(scene, "linkforge_robot", None)
    if props:
        if hasattr(props, "ros2_control_joints"):
            props.ros2_control_joints.clear()
        if hasattr(props, "ros2_control_parameters"):
            props.ros2_control_parameters.clear()
        if hasattr(props, "planning_groups"):
            props.planning_groups.clear()
        if hasattr(props, "end_effectors"):
            props.end_effectors.clear()
        if hasattr(props, "group_states"):
            props.group_states.clear()
        if hasattr(props, "disabled_collisions"):
            props.disabled_collisions.clear()

    for data_type in ["objects", "meshes", "materials", "armatures", "actions", "collections"]:
        data_block = getattr(bpy.data, data_type)
        for item in list(data_block):
            try:
                # Don't remove the Scene Collection or the Scene itself
                if data_type == "collections" and item.name == "Scene Collection":
                    continue
                data_block.remove(item, do_unlink=True)
            except (ReferenceError, RuntimeError, AttributeError):
                pass

    # Nuclear purge of any orphan data
    bpy.data.orphans_purge()

    # Clear architectural statistics cache for test isolation
    os.environ["LINKFORGE_DISABLE_CACHE"] = "1"
    from linkforge.blender.utils.scene_utils import clear_stats_cache

    clear_stats_cache()
    yield
