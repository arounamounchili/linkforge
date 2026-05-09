import contextlib
import typing

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

    # Quick check: are they all there?
    all_present = all(hasattr(bpy.types.Object, p) for p in object_props) and hasattr(
        bpy.types.WindowManager, "linkforge_validation"
    )

    if not all_present:
        # Force a clean re-registration cycle
        with contextlib.suppress(Exception):
            linkforge.blender.unregister()
        linkforge.blender.register()


def safe_get_linkforge(obj: bpy.types.Object, scene: typing.Any = None) -> typing.Any:
    """Safe accessor for the 'linkforge' property group on a Blender object."""
    prop = getattr(obj, "linkforge", None)
    if prop and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment(scene)
    prop = getattr(obj, "linkforge", None)
    if prop and hasattr(prop, "bl_rna"):
        return prop

    raise AttributeError(f"Object '{obj.name}' missing 'linkforge' property group.")


def safe_get_joint(obj: bpy.types.Object, scene: typing.Any = None) -> typing.Any:
    """Safe accessor for the 'linkforge_joint' property group on a Blender object."""
    prop = getattr(obj, "linkforge_joint", None)
    if prop and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment(scene)
    prop = getattr(obj, "linkforge_joint", None)
    if prop and hasattr(prop, "bl_rna"):
        return prop

    raise AttributeError(f"Object '{obj.name}' missing 'linkforge_joint' property group.")


def safe_get_linkforge_scene(scene: bpy.types.Scene) -> typing.Any:
    """Safe accessor for the 'linkforge' property group on a Blender scene."""
    prop = getattr(scene, "linkforge", None)
    if prop and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment(scene)
    prop = getattr(scene, "linkforge", None)
    if prop and hasattr(prop, "bl_rna"):
        return prop

    raise AttributeError(f"Scene '{scene.name}' missing 'linkforge' property group.")


def safe_get_transmission(obj: bpy.types.Object, scene: typing.Any = None) -> typing.Any:
    """Safe accessor for the 'linkforge_transmission' property group on a Blender object."""
    prop = getattr(obj, "linkforge_transmission", None)
    if prop and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment(scene)
    prop = getattr(obj, "linkforge_transmission", None)
    if prop and hasattr(prop, "bl_rna"):
        return prop

    raise AttributeError(f"Object '{obj.name}' missing 'linkforge_transmission' property group.")


def safe_get_validation(wm: bpy.types.WindowManager) -> typing.Any:
    """Safe accessor for the window manager 'linkforge_validation' property group."""
    prop = getattr(wm, "linkforge_validation", None)
    if prop and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment()
    prop = getattr(wm, "linkforge_validation", None)
    if prop and hasattr(prop, "bl_rna"):
        return prop

    raise AttributeError("WindowManager missing 'linkforge_validation' property group.")


def safe_get_sensor(obj: bpy.types.Object, scene: typing.Any = None) -> typing.Any:
    """Safely retrieve or initialize sensor properties on an object."""
    prop = getattr(obj, "linkforge_sensor", None)
    if prop and hasattr(prop, "bl_rna"):
        return prop

    # If missing, try a quick refresh
    _refresh_blender_environment(scene)
    prop = getattr(obj, "linkforge_sensor", None)
    if prop and hasattr(prop, "bl_rna"):
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
    name: str, data: typing.Any = None, scene: bpy.types.Scene | None = None
) -> bpy.types.Object:
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

    return obj


def create_mesh_object(name: str, scene: bpy.types.Scene | None = None) -> bpy.types.Object:
    """Create a new mesh object."""
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    return create_test_object(name, mesh, scene=scene)


def create_simple_robot_scene(
    scene: bpy.types.Scene,
) -> tuple[bpy.types.Collection, bpy.types.Object, bpy.types.Object]:
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
    scene: bpy.types.Scene,
    parent: bpy.types.Object | None = None,
    with_visual: bool = True,
) -> bpy.types.Object:
    """High-level factory to create a LinkForge robot link.

    Creates an Empty object as the link frame and optionally a child visual mesh.
    """
    link_obj = create_test_object(name, None, scene=scene)

    safe_get_linkforge(link_obj, scene).is_robot_link = True

    if parent:
        link_obj.parent = parent

    if with_visual:
        mesh_obj = create_mesh_object(f"{name}_visual", scene=scene)
        mesh_obj.parent = link_obj

    if scene.view_layers:
        scene.view_layers[0].update()

    return link_obj


def create_robot_joint(
    name: str,
    parent_link: bpy.types.Object,
    child_link: bpy.types.Object,
    scene: bpy.types.Scene,
    joint_type: str = "REVOLUTE",
) -> bpy.types.Object:
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
    scene: bpy.types.Scene, prefix: str = "test_arm"
) -> tuple[bpy.types.Object, bpy.types.Object, bpy.types.Object]:
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
