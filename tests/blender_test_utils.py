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

    # 1. Exhaustive cleanup
    with contextlib.suppress(Exception):
        linkforge.blender.unregister()

    for p in object_props:
        with contextlib.suppress(AttributeError):
            delattr(bpy.types.Object, p)
    for p in scene_props:
        with contextlib.suppress(AttributeError):
            delattr(bpy.types.Scene, p)

    # 2. Fresh registration
    linkforge.blender.register()

    # 3. Critical verification
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

    This utility manages the 'Zombie Property' issue by forcing Blender to load
    the C-level RNA metadata before returning the object.
    """
    import linkforge.blender

    # Critical property groups that must be fully bound
    props = ["linkforge", "linkforge_joint", "linkforge_sensor", "linkforge_transmission"]

    def verify_rna_health(o: bpy.types.Object) -> bool:
        """Verify that property groups have allocated RNA data."""
        try:
            return all(getattr(o, p).bl_rna for p in props)
        except (AttributeError, RuntimeError):
            return False

    # 1. Create the object instance
    obj = bpy.data.objects.new(name, object_data)
    if obj.name not in bpy.data.objects:
        # Emergency retry if Blender database state is inconsistent
        obj = bpy.data.objects.new(name, object_data)

    # 2. Link to scene collection (essential for RNA dispatch table initialization)
    if scene and hasattr(scene, "collection"):
        with contextlib.suppress(RuntimeError):
            scene.collection.objects.link(obj)
            if hasattr(bpy.context, "view_layer") and bpy.context.view_layer:
                bpy.context.view_layer.update()
            # Force update on the provided scene's view layers too
            if hasattr(scene, "view_layers"):
                for vl in scene.view_layers:
                    vl.update()

    # 3. Final Binding Health Check
    if not verify_rna_health(obj):
        # Nuclear re-registration
        with contextlib.suppress(Exception):
            linkforge.blender.unregister()
        linkforge.blender.register()

        # Force update on both scene and context view layer if available
        if hasattr(bpy.context, "view_layer") and bpy.context.view_layer:
            bpy.context.view_layer.update()
        if scene and hasattr(scene, "view_layers"):
            for vl in scene.view_layers:
                vl.update()

        # Last resort check
        if not verify_rna_health(obj):
            # If still unhealthy, the environment is likely unrecoverable
            # We report detailed diagnostics to help pinpoint the failure
            has_linkforge = hasattr(bpy.types.Object, "linkforge")
            raise AttributeError(
                f"Ironclad Factory Failure: Object '{obj.name}' failed RNA health check even after nuclear recovery.\n"
                f"bpy.types.Object.linkforge registration status: {has_linkforge}"
            )

    return obj
