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

    # Final Binding Health Check with Nuclear Recovery
    # We actively probe the property group on the instance.
    try:
        # Accessing bl_rna forces Blender to finalize the instance-level binding
        if not (hasattr(obj, "linkforge") and obj.linkforge and obj.linkforge.bl_rna):
            raise AttributeError("RNA Binding Incomplete")
    except (AttributeError, RuntimeError):
        # Nuclear Recovery: Re-register the addon and refresh properties
        import linkforge.blender

        with contextlib.suppress(Exception):
            linkforge.blender.unregister()
        linkforge.blender.register()

        # Verify class-level registration
        if not hasattr(bpy.types.Object, "linkforge"):
            raise RuntimeError(
                "Fatal: linkforge property missing from bpy.types.Object after registration"
            ) from None

        # Force depsgraph update to push properties to instances
        if scene and hasattr(scene, "view_layers"):
            for vl in scene.view_layers:
                vl.update()
        elif hasattr(bpy.context, "view_layer") and bpy.context.view_layer:
            bpy.context.view_layer.update()

        # Final Verification on the fresh wrapper
        obj = bpy.data.objects[obj.name]
        try:
            if not (hasattr(obj, "linkforge") and obj.linkforge and obj.linkforge.bl_rna):
                raise AttributeError("Final Binding Failure")
        except (AttributeError, RuntimeError):
            # This is the "Nuclear" error - it means the environment is broken
            raise AttributeError(
                f"Ironclad Registration Failure: Object '{obj.name}' lost LinkForge property binding "
                "and could not be recovered. This usually indicates a conflict in the Blender environment."
            ) from None

    return obj


def create_mesh_object(name: str, scene: Any = None) -> bpy.types.Object:
    """Creates a simple mesh object (cube) for testing."""
    mesh_data = bpy.data.meshes.new(f"{name}_mesh")
    # Simple cube-like data
    mesh_data.from_pydata([(1, 1, 1), (1, -1, 1), (-1, -1, 1), (-1, 1, 1)], [], [(0, 1, 2, 3)])
    return create_test_object(name, mesh_data, scene)


def safe_get_linkforge(obj: bpy.types.Object, scene: Any = None) -> Any:
    """Safe accessor for the 'linkforge' property group with auto-recovery."""
    try:
        if hasattr(obj, "linkforge") and obj.linkforge and obj.linkforge.bl_rna:
            return obj.linkforge
    except (AttributeError, RuntimeError):
        pass

    # Environment Refresh & Re-fetch
    _refresh_blender_environment(scene)

    # Re-fetch the object wrapper by name to ensure we have a fresh RNA pointer
    try:
        fresh_obj = bpy.data.objects[obj.name]
        prop = getattr(fresh_obj, "linkforge", None)
        if prop and prop.bl_rna:
            return prop
    except (KeyError, AttributeError, RuntimeError):
        pass

    raise AttributeError(
        f"Object '{obj.name}' missing 'linkforge' property after environment refresh."
    )


def safe_get_linkforge_scene(scene: bpy.types.Scene) -> Any:
    """Safe accessor for the scene-level 'linkforge' property group."""
    try:
        if hasattr(scene, "linkforge") and scene.linkforge and scene.linkforge.bl_rna:
            return scene.linkforge
    except (AttributeError, RuntimeError):
        pass

    _refresh_blender_environment(scene)

    try:
        fresh_scene = bpy.data.scenes[scene.name]
        prop = getattr(fresh_scene, "linkforge", None)
        if prop and prop.bl_rna:
            return prop
    except (KeyError, AttributeError, RuntimeError):
        pass

    raise AttributeError(
        f"Scene '{scene.name}' missing 'linkforge' property after environment refresh."
    )


def safe_get_joint(obj: bpy.types.Object, scene: Any = None) -> Any:
    """Safe accessor for the 'linkforge_joint' property group."""
    try:
        if hasattr(obj, "linkforge_joint") and obj.linkforge_joint and obj.linkforge_joint.bl_rna:
            return obj.linkforge_joint
    except (AttributeError, RuntimeError):
        pass

    _refresh_blender_environment(scene)

    try:
        fresh_obj = bpy.data.objects[obj.name]
        prop = getattr(fresh_obj, "linkforge_joint", None)
        if prop and prop.bl_rna:
            return prop
    except (KeyError, AttributeError, RuntimeError):
        pass

    raise AttributeError(
        f"Object '{obj.name}' missing 'linkforge_joint' property after environment refresh."
    )


def safe_get_sensor(obj: bpy.types.Object, scene: Any = None) -> Any:
    """Safe accessor for the 'linkforge_sensor' property group."""
    try:
        if (
            hasattr(obj, "linkforge_sensor")
            and obj.linkforge_sensor
            and obj.linkforge_sensor.bl_rna
        ):
            return obj.linkforge_sensor
    except (AttributeError, RuntimeError):
        pass

    _refresh_blender_environment(scene)

    try:
        fresh_obj = bpy.data.objects[obj.name]
        prop = getattr(fresh_obj, "linkforge_sensor", None)
        if prop and prop.bl_rna:
            return prop
    except (KeyError, AttributeError, RuntimeError):
        pass

    raise AttributeError(
        f"Object '{obj.name}' missing 'linkforge_sensor' property after environment refresh."
    )


def safe_get_transmission(obj: bpy.types.Object, scene: Any = None) -> Any:
    """Safe accessor for the 'linkforge_transmission' property group."""
    try:
        if (
            hasattr(obj, "linkforge_transmission")
            and obj.linkforge_transmission
            and obj.linkforge_transmission.bl_rna
        ):
            return obj.linkforge_transmission
    except (AttributeError, RuntimeError):
        pass

    _refresh_blender_environment(scene)

    try:
        fresh_obj = bpy.data.objects[obj.name]
        prop = getattr(fresh_obj, "linkforge_transmission", None)
        if prop and prop.bl_rna:
            return prop
    except (KeyError, AttributeError, RuntimeError):
        pass

    raise AttributeError(
        f"Object '{obj.name}' missing 'linkforge_transmission' property after environment refresh."
    )


def _refresh_blender_environment(scene: Any = None) -> None:
    """Internal helper to refresh the Blender RNA state (Nuclear Recovery)."""
    import linkforge.blender

    # Fallback to the active scene if none provided
    if not scene:
        scene = bpy.context.scene or (bpy.data.scenes[0] if bpy.data.scenes else None)

    with contextlib.suppress(Exception):
        linkforge.blender.unregister()
    linkforge.blender.register()

    # Force depsgraph update to push properties to instances
    if scene and hasattr(scene, "view_layers"):
        for vl in scene.view_layers:
            vl.update()
    elif hasattr(bpy.context, "view_layer") and bpy.context.view_layer:
        bpy.context.view_layer.update()


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
    if parent.name not in collection.objects:
        collection.objects.link(parent)

    # Set properties using the safe accessor
    safe_get_linkforge(parent, scene=target_scene).is_robot_link = True

    # Create child mesh
    mesh_data = bpy.data.meshes.new("child_visual")
    mesh_data.from_pydata(
        [(0.25, 0.25, 0.25), (0.25, -0.25, 0.25), (-0.25, -0.25, 0.25), (-0.25, 0.25, 0.25)],
        [],
        [(0, 1, 2, 3)],
    )
    child = create_test_object("child_visual", mesh_data, scene=target_scene)
    child.parent = parent

    if child.name not in collection.objects:
        collection.objects.link(child)

    # Final update
    if target_scene.view_layers:
        target_scene.view_layers[0].update()

    return collection, parent, child
