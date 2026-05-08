import typing

import pytest

try:
    import bpy

    HAS_BPY = True
except (ImportError, AttributeError):
    HAS_BPY = False

from tests.blender_test_utils import create_test_object

if HAS_BPY:
    import linkforge.blender

    @pytest.fixture(scope="session", autouse=True)
    def register_addon() -> None:
        """Register the LinkForge addon once for the entire test session.

        This ensures that all Blender operators and property groups are
        globally available before any tests are executed.
        """
        linkforge.blender.register()

    @pytest.fixture(scope="module", autouse=True)
    def ensure_registered():
        """Ensure LinkForge properties are registered and fully active for the module."""
        from tests.blender_test_utils import ensure_linkforge_registered

        ensure_linkforge_registered()
        yield

    @pytest.fixture
    def scene(ensure_registered) -> bpy.types.Scene:
        """Provide a robust Blender scene for testing."""
        target_scene = bpy.context.scene or (bpy.data.scenes[0] if bpy.data.scenes else None)
        assert target_scene is not None, "No Blender scene available for testing"

        # Final sanity check on an actual instance
        temp_obj = create_test_object("RegistrationCheck", None)
        has_prop = hasattr(temp_obj, "linkforge")
        bpy.data.objects.remove(temp_obj, do_unlink=True)

        assert has_prop, "LinkForge properties not active on Blender Objects"
        return target_scene

    @pytest.fixture(autouse=True)
    def clean_scene(scene: bpy.types.Scene) -> typing.Generator[None, None, None]:
        """Prepare a clean Blender environment for each test.

        Actions performed:
        - Removes all objects and their linked data (meshes, materials).
        - Clears all non-default collections.
        - Resets LinkForge-specific global scene properties to default states.
        """
        # Delete all objects in all collections
        for obj in bpy.data.objects:
            bpy.data.objects.remove(obj, do_unlink=True)

        # Delete all mesh data
        for mesh in bpy.data.meshes:
            bpy.data.meshes.remove(mesh, do_unlink=True)

        # Delete all materials
        for mat in bpy.data.materials:
            bpy.data.materials.remove(mat, do_unlink=True)

        # Delete all collections (except master)
        for col in bpy.data.collections:
            if col.name != "Collection":
                bpy.data.collections.remove(col, do_unlink=True)

        # Reset Scene properties
        if hasattr(scene, "linkforge"):
            from linkforge.blender.properties.robot_props import RobotPropertyGroup

            props = typing.cast(RobotPropertyGroup, scene.linkforge)
            props.use_ros2_control = False
            props.ros2_control_joints.clear()

        # Clear architectural statistics cache for test isolation
        from linkforge.blender.utils.scene_utils import clear_stats_cache

        clear_stats_cache()

        yield
