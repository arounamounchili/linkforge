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
        for obj in list(bpy.data.objects):
            bpy.data.objects.remove(obj, do_unlink=True)

        # Delete all mesh data
        for mesh in list(bpy.data.meshes):
            bpy.data.meshes.remove(mesh, do_unlink=True)

        # Delete all materials
        for mat in list(bpy.data.materials):
            bpy.data.materials.remove(mat, do_unlink=True)

        # Delete all actions (animations)
        for action in list(bpy.data.actions):
            bpy.data.actions.remove(action, do_unlink=True)

        # Delete all collections (except master)
        for col in list(bpy.data.collections):
            if col.name not in ["Collection", "Scene Collection"]:
                bpy.data.collections.remove(col, do_unlink=True)

        # Reset Scene properties
        if hasattr(scene, "linkforge"):
            from linkforge.blender.properties.robot_props import RobotPropertyGroup

            props = typing.cast(RobotPropertyGroup, scene.linkforge)
            props.robot_name = "robot"
            props.strict_mode = False
            props.use_ros2_control = False
            props.ros2_control_joints.clear()
            props.gazebo_plugin_name = "libgazebo_ros2_control.so"
            props.controllers_yaml_path = ""

        # Clear architectural statistics cache for test isolation
        from linkforge.blender.utils.scene_utils import clear_stats_cache

        clear_stats_cache()

        yield

    @pytest.fixture
    def mock_context(mocker, scene) -> typing.Any:
        """Provide a mocked Blender context with the current scene and view_layer.

        This eliminates the need to manually create context mocks in every operator test.
        """
        context = mocker.MagicMock()
        context.scene = scene
        context.view_layer = scene.view_layers[0] if hasattr(scene, "view_layers") else None
        context.window_manager = bpy.context.window_manager
        context.workspace = bpy.context.workspace
        context.area = bpy.context.area
        context.region = bpy.context.region
        return context
