import typing

import pytest

try:
    import bpy

    HAS_BPY = True
except ImportError:
    HAS_BPY = False

if HAS_BPY:
    import linkforge.blender

    @pytest.fixture(scope="session", autouse=True)
    def register_addon() -> None:
        """Register the LinkForge addon once for the entire test session.

        This ensures that all Blender operators and property groups are
        globally available before any tests are executed.
        """
        linkforge.blender.register()

    @pytest.fixture(autouse=True)
    def ensure_registered() -> None:
        """Verify addon registration before each test.

        Blender's internal state can occasionally reset (e.g., during factory
        resets). This fixture checks for the presence of LinkForge properties
        on core types and re-registers the addon if they are missing.
        """
        needs_re_reg = not hasattr(bpy.types.Object, "linkforge") or not hasattr(
            bpy.types.Scene, "linkforge"
        )

        if needs_re_reg:
            linkforge.blender.register()

    @pytest.fixture(autouse=True)
    def clean_scene() -> typing.Generator[None, None, None]:
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
        scene = bpy.context.scene
        if scene and hasattr(scene, "linkforge"):
            from linkforge.blender.properties.robot_props import RobotPropertyGroup

            props = typing.cast(RobotPropertyGroup, scene.linkforge)
            props.use_ros2_control = False
            props.ros2_control_joints.clear()

        # Clear architectural statistics cache for test isolation
        from linkforge.blender.utils.scene_utils import clear_stats_cache

        clear_stats_cache()

        yield
