import contextlib
from typing import Any

import bpy


def ensure_linkforge_registered():
    """Ensure LinkForge properties are registered and fully active.

    This performs a clean unregister/register cycle and verifies that
    all property groups are correctly bound to Blender's global types.
    """
    import linkforge.blender

    # Properties that MUST be present on global types
    object_props = [
        "linkforge",
        "linkforge_joint",
        "linkforge_sensor",
        "linkforge_transmission",
    ]
    scene_props = ["linkforge"]

    # Exhaustive cleanup
    with contextlib.suppress(Exception):
        linkforge.blender.unregister()

    for p in object_props:
        with contextlib.suppress(AttributeError):
            delattr(bpy.types.Object, p)
    for p in scene_props:
        with contextlib.suppress(AttributeError):
            delattr(bpy.types.Scene, p)

    # Fresh registration
    linkforge.blender.register()

    # Critical verification
    for p in object_props:
        if not hasattr(bpy.types.Object, p):
            raise RuntimeError(f"Registration Failed: bpy.types.Object missing '{p}'")
    for p in scene_props:
        if not hasattr(bpy.types.Scene, p):
            raise RuntimeError(f"Registration Failed: bpy.types.Scene missing '{p}'")


def create_test_object(name: str, object_data: Any = None, scene: Any = None) -> bpy.types.Object:
    """
    Robust factory for creating Blender objects in a test environment.
    Ensures LinkForge property groups are fully instantiated and RNA-dispatchable.
    """
    # Create the object instance
    obj = bpy.data.objects.new(name, object_data)

    # Link to scene collection (essential for RNA dispatch table initialization)
    # ONLY if scene is explicitly provided, to avoid breaking tests that do manual linking
    if scene and hasattr(scene, "collection"):
        if not obj.users_collection:
            scene.collection.objects.link(obj)

        # Ensure view layer is updated so properties are accessible
        if hasattr(scene, "view_layers"):
            for vl in scene.view_layers:
                vl.update()
        elif hasattr(bpy.context, "view_layer") and bpy.context.view_layer:
            bpy.context.view_layer.update()

    # Final Binding Health Check
    if not hasattr(obj, "linkforge"):
        raise AttributeError(
            f"Registration Failure: Object '{obj.name}' missing 'linkforge' property. "
            "Ensure the addon is registered in the test session."
        )

    return obj


def create_simple_robot_scene(
    scene_name: str = "RobotScene",
) -> tuple[bpy.types.Collection, bpy.types.Object, bpy.types.Object]:
    """Helper to create a standard robot link-child hierarchy for testing.

    Returns:
        A tuple of (collection, parent_link, child_mesh)
    """
    # Create collection
    collection = bpy.data.collections.new(scene_name)

    # Robustly get the target scene (context might be None in background)
    target_scene = bpy.context.scene or (bpy.data.scenes[0] if bpy.data.scenes else None)
    if not target_scene:
        raise RuntimeError("No Blender scene available to link test collection")

    target_scene.collection.children.link(collection)

    # Create parent link (Empty)
    parent = create_test_object("parent_link", None, scene=target_scene)
    collection.objects.link(parent)
    parent.linkforge.is_robot_link = True

    # Create child mesh
    bpy.ops.mesh.primitive_cube_add(size=0.5)
    child = bpy.context.active_object
    child.name = "child_visual"
    child.parent = parent

    # Ensure everything is linked to the right collection
    if child.name not in collection.objects:
        collection.objects.link(child)

    bpy.context.view_layer.update()

    return collection, parent, child
