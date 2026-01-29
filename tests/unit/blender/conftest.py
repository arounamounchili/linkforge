import bpy
import pytest


@pytest.fixture(scope="session", autouse=True)
def register_addon():
    """Register the LinkForge addon for the test session."""
    import linkforge.blender

    linkforge.blender.register()
    yield
    linkforge.blender.unregister()


@pytest.fixture(autouse=True)
def clean_scene():
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

    yield
