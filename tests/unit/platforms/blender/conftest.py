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
    from linkforge.blender.adapters.context import BlenderContext

    return BlenderContext(bpy)


@pytest.fixture
def scene(blender_context):
    """Returns the active scene."""
    return blender_context.scene


@pytest.fixture(autouse=True)
def clean_scene(blender_context):
    """Automatically cleans the scene before each test."""
    # Real Blender removal of all objects and underlying data

    # Clear scene-level LinkForge property collections (persisted on bpy.data.scenes)
    scene = bpy.context.scene
    props = getattr(scene, "linkforge_robot", None)
    from unittest.mock import MagicMock, Mock

    from tests.mock_bpy_env import MockPropertyGroup, MockScene

    if not is_real_blender and (
        props is None
        or isinstance(props, (MagicMock, Mock))
        or not isinstance(props, MockPropertyGroup)
    ):
        if (
            not hasattr(scene, "linkforge")
            or isinstance(scene.linkforge, (MagicMock, Mock))
            or not isinstance(scene.linkforge, MockPropertyGroup)
        ):
            scene.linkforge = MockScene("Scene").linkforge
        scene.linkforge_robot = scene.linkforge
        props = scene.linkforge_robot

    if not is_real_blender:
        wm = getattr(bpy.context, "window_manager", None)
        if wm is not None:
            from linkforge.blender.properties.validation_props import ValidationResultProperty

            val = getattr(wm, "linkforge_validation", None)
            if (
                val is None
                or isinstance(val, MagicMock)
                or not isinstance(val, ValidationResultProperty)
            ):
                wm.linkforge_validation = ValidationResultProperty()
            else:
                val.clear()

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
        if hasattr(props, "virtual_joint_child_link"):
            props.virtual_joint_child_link = None
        if hasattr(props, "virtual_joint_child_link_name"):
            props.virtual_joint_child_link_name = ""
        if hasattr(props, "use_virtual_joint"):
            props.use_virtual_joint = False
        if hasattr(props, "use_srdf"):
            props.use_srdf = False
        if hasattr(props, "preview_active_state_name"):
            props.preview_active_state_name = ""

    for data_type in ["objects", "meshes", "materials", "armatures", "actions", "collections"]:
        data_block = getattr(bpy.data, data_type, None)
        if data_block is None:
            continue
        for item in list(data_block):
            try:
                # Don't remove the Scene Collection or the Scene itself
                if data_type == "collections" and item.name == "Scene Collection":
                    continue
                data_block.remove(item, do_unlink=True)
            except (ReferenceError, RuntimeError, AttributeError):
                pass
        if not is_real_blender and hasattr(data_block, "clear") and data_type != "collections":
            data_block.clear()

    if not is_real_blender:
        if (
            hasattr(scene, "collection")
            and hasattr(scene.collection, "objects")
            and hasattr(scene.collection.objects, "clear")
        ):
            scene.collection.objects.clear()
        if hasattr(bpy.context, "active_object"):
            bpy.context.active_object = None
        if hasattr(bpy.context, "view_layer") and hasattr(bpy.context.view_layer, "objects"):
            bpy.context.view_layer.objects.active = None

    # Nuclear purge of any orphan data
    bpy.data.orphans_purge()

    # Clear architectural statistics cache for test isolation
    os.environ["LINKFORGE_DISABLE_CACHE"] = "1"
    from linkforge.blender.utils.scene_utils import clear_stats_cache

    clear_stats_cache()
    yield
