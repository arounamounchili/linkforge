try:
    import bpy

    HAS_BPY = True
except ImportError:
    HAS_BPY = False

import typing

import pytest

if HAS_BPY:
    import linkforge.blender

    @pytest.fixture(scope="session", autouse=True)
    def register_addon() -> None:
        """Ensure the addon is registered at the start of the session."""
        linkforge.blender.register()

    @pytest.fixture(autouse=True)
    def ensure_registered() -> None:
        """Check and re-register properties if they were lost (e.g. by factory reset)."""
        # Check for both standard Object properties and Scene properties
        # Sometimes read_factory_settings wipes Scene but not Object types OR vice versa
        needs_re_reg = not hasattr(bpy.types.Object, "linkforge") or not hasattr(
            bpy.types.Scene, "linkforge"
        )

        if needs_re_reg:
            linkforge.blender.register()

    @pytest.fixture(autouse=True)
    def clean_scene() -> typing.Generator[None, None, None]:
        """Clear all objects and data from the scene before each test."""
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
            if col.name != "Collection":  # Preserve default if needed, or just delete all
                bpy.data.collections.remove(col, do_unlink=True)

        # Reset Scene properties
        if hasattr(bpy.context.scene, "linkforge"):
            bpy.context.scene.linkforge.use_ros2_control = False
            bpy.context.scene.linkforge.ros2_control_joints.clear()

        yield
