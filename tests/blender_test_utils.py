import contextlib
import typing

import bpy
import bpy.types


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

    # Quick check: are they all there?
    if hasattr(bpy, "types") and hasattr(bpy.types, "Object"):
        all_present = all(hasattr(bpy.types.Object, p) for p in object_props) and hasattr(
            bpy.types.WindowManager, "linkforge_validation"
        )
    else:
        all_present = False

    if not all_present:
        # Force a clean re-registration cycle
        with contextlib.suppress(Exception):
            linkforge.blender.unregister()
        linkforge.blender.register()
    else:
        # Even if properties are present, handlers might be missing (e.g. after mock reset)
        # Calling register() again is safe as it checks for duplicate handlers.
        linkforge.blender.register()


def safe_get_linkforge(obj: typing.Any, scene: typing.Any = None) -> typing.Any:
    """Safe accessor for the 'linkforge' property group on a Blender object."""
    prop = getattr(obj, "linkforge", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment(scene)
    prop = getattr(obj, "linkforge", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    raise AttributeError(f"Object '{obj.name}' missing 'linkforge' property group.")


def safe_get_joint(obj: typing.Any, scene: typing.Any = None) -> typing.Any:
    """Safe accessor for the 'linkforge_joint' property group on a Blender object."""
    prop = getattr(obj, "linkforge_joint", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment(scene)
    prop = getattr(obj, "linkforge_joint", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    raise AttributeError(f"Object '{obj.name}' missing 'linkforge_joint' property group.")


def safe_get_linkforge_scene(scene: typing.Any) -> typing.Any:
    """Safe accessor for the 'linkforge' property group on a Blender scene."""
    prop = getattr(scene, "linkforge", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment(scene)
    prop = getattr(scene, "linkforge", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    raise AttributeError(f"Scene '{scene.name}' missing 'linkforge' property group.")


def safe_get_transmission(obj: typing.Any, scene: typing.Any = None) -> typing.Any:
    """Safe accessor for the 'linkforge_transmission' property group on a Blender object."""
    prop = getattr(obj, "linkforge_transmission", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment(scene)
    prop = getattr(obj, "linkforge_transmission", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    raise AttributeError(f"Object '{obj.name}' missing 'linkforge_transmission' property group.")


def safe_get_validation(wm: typing.Any) -> typing.Any:
    """Safe accessor for the window manager 'linkforge_validation' property group."""
    prop = getattr(wm, "linkforge_validation", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment()
    prop = getattr(wm, "linkforge_validation", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    raise AttributeError("WindowManager missing 'linkforge_validation' property group.")


def safe_get_sensor(obj: typing.Any, scene: typing.Any = None) -> typing.Any:
    """Safely retrieve or initialize sensor properties on an object."""
    prop = getattr(obj, "linkforge_sensor", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment(scene)
    prop = getattr(obj, "linkforge_sensor", None)
    if prop is not None and hasattr(prop, "bl_rna"):
        return prop

    raise AttributeError(f"Object '{obj.name}' missing 'linkforge_sensor' property group.")


def _refresh_blender_environment(scene: typing.Any = None) -> None:
    """Trigger a clean re-registration of the LinkForge addon.

    Used as a 'nuclear option' when Blender's internal RNA mapping gets lost
    during intensive headless test runs.
    """
    import linkforge.blender

    with contextlib.suppress(Exception):
        linkforge.blender.unregister()
    linkforge.blender.register()


def create_test_object(
    name: str, data: typing.Any = None, scene: typing.Any | None = None
) -> typing.Any:
    """Create a new Blender object.

    Linking behavior:
    - If 'scene' is provided: Links to the scene's collection (standard behavior).
    - If 'scene' is None: Only creates in data (legacy/manual behavior).
    """
    # Clean up existing data-only object with same name if it exists (prevents .001)
    if name in bpy.data.objects:
        old_obj = bpy.data.objects[name]
        if not old_obj.users_collection:
            bpy.data.objects.remove(old_obj, do_unlink=True)

    obj = bpy.data.objects.new(name, data)

    if scene:
        with contextlib.suppress(RuntimeError):
            scene.collection.objects.link(obj)
        if hasattr(scene, "objects") and isinstance(scene.objects, list):
            scene.objects.append(obj)

    # Set as active
    if hasattr(bpy.context, "view_layer") and bpy.context.view_layer is not None:
        with contextlib.suppress(AttributeError, RuntimeError):
            bpy.context.view_layer.objects.active = obj

    # Fallback/Direct set for mocks
    with contextlib.suppress(AttributeError, RuntimeError):
        bpy.context.active_object = obj

    return obj


def create_mesh_object(
    name: str, scene: typing.Any | None = None, with_cube: bool = False
) -> typing.Any:
    """Create a new mesh object, optionally with a unit cube."""
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    if with_cube:
        import bmesh

        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=2.0)
        bm.to_mesh(mesh)
        bm.free()
    obj = create_test_object(name, mesh, scene=scene)
    return obj


def create_simple_robot_scene(
    scene: typing.Any,
) -> tuple[typing.Any, typing.Any, typing.Any]:
    """Create a minimal 2-link robot scene for integration tests.

    Hierarchy: root_collection -> [parent_link, child_link, joint]
    """
    collection = bpy.data.collections.new("TestRobot")
    scene.collection.children.link(collection)

    parent = create_mesh_object("parent_link", scene=scene)
    child = create_mesh_object("child_link", scene=scene)

    # Parent child link far away to avoid origin overlaps
    child.location = (0, 0, 1)

    # Setup joint
    joint = create_test_object("joint", None, scene=scene)

    joint_props = safe_get_joint(joint, scene)
    joint_props.is_robot_joint = True
    joint_props.parent_link = parent
    joint_props.child_link = child

    # Final update
    if scene.view_layers:
        scene.view_layers[0].update()

    return collection, parent, child


def create_robot_link(
    name: str,
    scene: typing.Any,
    parent: typing.Any | None = None,
    with_visual: bool = True,
    with_collision: bool = True,
    with_cube: bool = True,
) -> typing.Any:
    """High-level factory to create a LinkForge robot link.

    Creates an Empty object as the link frame and optionally child meshes.
    """
    link_obj = create_test_object(name, None, scene=scene)

    safe_get_linkforge(link_obj, scene).is_robot_link = True

    if parent:
        link_obj.parent = parent

    if with_visual:
        mesh_obj = create_mesh_object(f"{name}_visual", scene=scene, with_cube=with_cube)
        mesh_obj.parent = link_obj
        safe_get_linkforge(mesh_obj, scene).is_robot_visual = True

    if with_collision:
        mesh_obj = create_mesh_object(f"{name}_collision", scene=scene, with_cube=with_cube)
        mesh_obj.parent = link_obj
        safe_get_linkforge(mesh_obj, scene).is_robot_collision = True

    if scene.view_layers:
        scene.view_layers[0].update()

    return link_obj


def create_robot_joint(
    name: str,
    parent_link: typing.Any,
    child_link: typing.Any,
    scene: typing.Any,
    joint_type: str = "REVOLUTE",
) -> typing.Any:
    """High-level factory to create a LinkForge robot joint.

    Handles object creation, parenting, and RNA property assignment.
    """
    joint_obj = create_test_object(name, None, scene=scene)

    joint_props = safe_get_joint(joint_obj, scene)
    joint_props.is_robot_joint = True
    joint_props.joint_type = joint_type
    joint_props.parent_link = parent_link
    joint_props.child_link = child_link

    if scene.view_layers:
        scene.view_layers[0].update()

    return joint_obj


def setup_2_link_arm(
    scene: typing.Any, prefix: str = "test_arm"
) -> tuple[typing.Any, typing.Any, typing.Any]:
    """Sets up a minimal 2-link robotic arm hierarchy.

    Structure: base_link -> joint -> child_link

    Returns:
        tuple: (base_link, joint, child_link)
    """
    base = create_robot_link(f"{prefix}_base", scene)
    child = create_robot_link(f"{prefix}_child", scene)
    child.location = (0, 0, 1)

    joint = create_robot_joint(f"{prefix}_joint", base, child, scene)

    if scene.view_layers:
        scene.view_layers[0].update()

    return base, joint, child


def cleanup_blender_scene(scene: typing.Any | None = None) -> None:
    """Surgically clean the Blender environment for test isolation.

    Resets only relevant bpy.data collections and linkforge property groups
    to avoid expensive addon re-registration cycles.
    """
    import bpy

    # Delete all objects in all collections
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)

    # Reset context state to prevent leakage between tests
    if hasattr(bpy.context, "active_object"):
        # In mock environment, we can set it directly
        with contextlib.suppress(AttributeError, RuntimeError):
            bpy.context.active_object = None

    if hasattr(bpy.context, "view_layer") and bpy.context.view_layer:
        with contextlib.suppress(AttributeError, RuntimeError):
            bpy.context.view_layer.objects.active = None

    if hasattr(bpy.context, "selected_objects"):
        with contextlib.suppress(AttributeError, RuntimeError):
            bpy.context.selected_objects.clear()

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
    target_scene = scene or bpy.context.scene
    props = getattr(target_scene, "linkforge", None)
    if props:
        props.robot_name = "robot"
        props.strict_mode = False
        props.use_ros2_control = False
        props.ros2_control_joints.clear()
        props.gazebo_plugin_name = "libgazebo_ros2_control.so"
        props.controllers_yaml_path = ""

    # Clear architectural statistics cache for test isolation
    from linkforge.blender.utils.scene_utils import clear_stats_cache

    clear_stats_cache()


def safe_update(scene: typing.Any | None = None) -> None:
    """Bulletproof scene update that handles NoneType contexts in headless mode.

    Attempts to resolve the scene from context, then falls back to global data.
    """
    import bpy

    # Fallback order: provided scene -> context scene -> first scene in data
    target_scene = (
        scene
        or getattr(bpy.context, "scene", None)
        or (bpy.data.scenes[0] if bpy.data.scenes else None)
    )

    # Flush before to clear any previous pending renames
    try:
        from linkforge.blender.handlers import name_sync_handler

        name_sync_handler.flush_deferred_renames()
    except ImportError:
        pass

    if target_scene and hasattr(target_scene, "view_layers") and len(target_scene.view_layers) > 0:
        target_scene.view_layers[0].update()

    # Flush after to handle renames triggered by THIS update
    try:
        from linkforge.blender.handlers import name_sync_handler

        name_sync_handler.flush_deferred_renames()
    except ImportError:
        pass
