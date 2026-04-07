"""Scene Builder utilities for creating Blender objects from generic Robot models."""

from __future__ import annotations

import contextlib
import typing
from pathlib import Path

import bpy
from linkforge_core.logging_config import get_logger
from linkforge_core.models import (
    Box,
    Color,
    Cylinder,
    Joint,
    Link,
    Mesh,
    Robot,
    Sphere,
)
from linkforge_core.utils.kinematics import sort_joints_topological
from mathutils import Matrix

from linkforge.blender.preferences import get_addon_prefs
from linkforge.blender.utils.joint_utils import resolve_mimic_joints
from linkforge.blender.utils.scene_utils import move_to_collection, sync_object_collections

logger = get_logger(__name__)


def create_material_from_color(color: Color, name: str) -> bpy.types.Material | None:
    """Create Blender material from Color model.

    Args:
        color: Color model
        name: Material name

    Returns:
        Blender Material or None

    """
    # Check if material already exists
    if name in bpy.data.materials:
        return bpy.data.materials[name]

    # Create new material
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True

    # Get Principled BSDF node
    if mat.node_tree:
        nodes = mat.node_tree.nodes
        links = mat.node_tree.links

        # Clear existing nodes
        nodes.clear()

        # Create basic PBR nodes
        node_principled = nodes.new(type="ShaderNodeBsdfPrincipled")
        node_output = nodes.new(type="ShaderNodeOutputMaterial")

        # Set color
        if hasattr(node_principled.inputs[0], "default_value"):
            node_principled.inputs[0].default_value = (color.r, color.g, color.b, color.a)

        # Link nodes
        links.new(node_principled.outputs[0], node_output.inputs[0])

    return mat


def create_primitive_mesh(geometry: Box | Cylinder | Sphere, name: str) -> bpy.types.Object | None:
    """Create a Blender mesh object from primitive geometry.

    This function generates native Blender mesh primitives (Cube, Cylinder,
    Sphere) based on the Core geometry model and applies the correct
    dimensions and URDF-specific metadata tags.

    Args:
        geometry: One of Box, Cylinder, or Sphere models.
        name: Name to assign to the created Blender object.

    Returns:
        The created Blender Object or None if creation failed.
    """
    # Deselect all first
    bpy.ops.object.select_all(action="DESELECT")

    obj = None

    try:
        if isinstance(geometry, Box):
            bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
            obj = bpy.context.active_object
            if obj:
                obj.name = name
                obj.dimensions = (geometry.size.x, geometry.size.y, geometry.size.z)
                # Force update to ensure dimensions are applied correctly before return
                if bpy.context.view_layer:
                    bpy.context.view_layer.update()
                obj["urdf_geometry_type"] = "BOX"

        elif isinstance(geometry, Cylinder):
            bpy.ops.mesh.primitive_cylinder_add(location=(0, 0, 0))
            obj = bpy.context.active_object
            if obj:
                obj.name = name
                obj.dimensions = (geometry.radius * 2, geometry.radius * 2, geometry.length)
                if bpy.context.view_layer:
                    bpy.context.view_layer.update()
                obj["urdf_geometry_type"] = "CYLINDER"

        elif isinstance(geometry, Sphere):
            bpy.ops.mesh.primitive_uv_sphere_add(location=(0, 0, 0))
            obj = bpy.context.active_object
            if obj:
                obj.name = name
                obj.dimensions = (geometry.radius * 2, geometry.radius * 2, geometry.radius * 2)
                if bpy.context.view_layer:
                    bpy.context.view_layer.update()
                obj["urdf_geometry_type"] = "SPHERE"

        else:
            return None

    except (AttributeError, TypeError, ValueError) as e:
        logger.error(f"Failed to create primitive geometry: {e}")
        return None
    except Exception as e:
        logger.critical(f"Unexpected error creating geometry: {e}", exc_info=True)
        raise

    return obj


def import_mesh_file(mesh_path: Path, name: str) -> bpy.types.Object | None:
    """Import an external mesh file into the Blender scene.

    Supported formats include STL, OBJ, and GLB. This function utilizes
    modern Blender WM operators for improved performance and stability.

    Args:
        mesh_path: Absolute path to the mesh file.
        name: Name to assign to the imported object.

    Returns:
        The imported Blender Object or None if import failed.
    """
    if not mesh_path.exists():
        logger.error(f"Mesh file not found: {mesh_path}")
        return None

    logger.info(f"Importing mesh: {mesh_path} as '{name}'")

    # Deselect all
    bpy.ops.object.select_all(action="DESELECT")

    # Import based on file extension
    ext = mesh_path.suffix.lower()

    # Define operator candidates for each file extension
    # We use modern WM operators (C++ based) for performance and stability
    operators = {
        ".obj": ["wm.obj_import"],
        ".stl": ["wm.stl_import"],
        ".glb": ["wm.gltf_import", "import_scene.gltf"],
        ".gltf": ["wm.gltf_import", "import_scene.gltf"],
        ".dae": ["wm.collada_import"],
    }

    # Bug #3: Conditional DAE support for Blender < 5.0.0
    if ext == ".dae":
        if bpy.app.version >= (5, 0, 0):
            logger.error(
                f"Collada (.dae) support was removed in Blender 5.0. "
                f"Please convert '{mesh_path.name}' to .glb or .obj."
            )
            return None
        logger.warning(
            f"Collada (.dae) support is deprecated and will be removed in Blender 5.0. "
            f"Consider converting '{mesh_path.name}' to a modern format like .glb."
        )

    if ext not in operators:
        logger.warning(f"Unsupported mesh file extension: {ext} for '{mesh_path.name}'")
        return None

    # Snapshot objects and collections to identify stragglers afterward.
    pre_import_objects = set(bpy.data.objects)
    pre_import_collections = set(bpy.data.collections)

    # Dispatcher: Try each operator until one succeeds
    success = False
    for op_name in operators[ext]:
        try:
            # Dynamically look up operator
            op_parts = op_name.split(".")
            op = bpy.ops
            for part in op_parts:
                op = getattr(op, part)

            # Call the operator
            callable_op = typing.cast(typing.Callable[..., typing.Any], op)
            callable_op(filepath=str(mesh_path))
            success = True
            logger.debug(f"Successfully used importer: {op_name}")
            break
        except (AttributeError, RuntimeError) as e:
            logger.debug(f"Importer '{op_name}' failed or not found: {e}")
            continue
        except Exception as e:
            logger.warning(f"Importer '{op_name}' raised unexpected error: {e}")
            continue

    if not success:
        logger.error(f"No functional {ext.upper()} importer found for '{mesh_path.name}'.")
        return None

    try:
        # Get all imported objects (should be selected)
        imported_objects = list(bpy.context.selected_objects)
        if not imported_objects:
            logger.warning(f"Importer ran but no objects were found for '{mesh_path.name}'")
            return None

        # Consolidate ALL new objects created by the importer.
        # This catches stray nodes that CAD importers link but don't select.
        new_objects = set(bpy.data.objects) - pre_import_objects
        res_obj = normalize_and_consolidate_imported_objects(new_objects, name)

        if res_obj:
            # CLEANUP: Handle unwanted collections created by certain importers (e.g. GLTF).
            post_import_collections = set(bpy.data.collections)
            new_collections = post_import_collections - pre_import_collections

            if new_collections:
                # Ensure result object is in current active collection (not the importer's new one)
                current_col = bpy.context.collection
                if current_col and res_obj.name not in current_col.objects:
                    current_col.objects.link(res_obj)

                # Unlink from new collections and delete them
                for new_col in new_collections:
                    # Unlink object if it's there
                    if res_obj.name in new_col.objects:
                        new_col.objects.unlink(res_obj)

                    # Remove the collection itself
                    bpy.data.collections.remove(new_col, do_unlink=True)

                logger.debug(f"Cleaned up {len(new_collections)} import collections.")

            return res_obj

        return None

    except (RuntimeError, OSError) as e:
        logger.error(f"Failed to process imported mesh '{mesh_path.name}': {e}")
        return None
    except Exception as e:
        logger.critical(f"Unexpected error processing mesh '{mesh_path.name}': {e}", exc_info=True)
        raise


def normalize_and_consolidate_imported_objects(
    objects: typing.Iterable[bpy.types.Object], name: str
) -> bpy.types.Object | None:
    """Consolidation logic that processes all supplied objects (meshes → join, others → delete)."""
    if not objects:
        return None

    mesh_objs = set()
    to_delete = set()
    seen = set()

    def process_recursive(o: bpy.types.Object) -> None:
        if o in seen:
            return
        seen.add(o)

        # Unparent while keeping world transform
        world_mat = o.matrix_world.copy()
        o.parent = None
        o.matrix_world = world_mat

        if o.type == "MESH":
            mesh_objs.add(o)
        else:
            to_delete.add(o)

        for child in list(o.children):
            process_recursive(child)

    for obj in objects:
        with contextlib.suppress(ReferenceError):
            process_recursive(obj)

    if not mesh_objs:
        for obj in to_delete:
            with contextlib.suppress(RuntimeError, ReferenceError):
                bpy.data.objects.remove(obj, do_unlink=True)
        return None

    # Prepare for join
    mesh_list = list(mesh_objs)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in mesh_list:
        obj.matrix_world = Matrix.Identity(4)
        obj.select_set(True)

    if bpy.context.view_layer:
        bpy.context.view_layer.objects.active = mesh_list[0]

    # Bake axis/scale
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)

    # Join
    final_obj = mesh_list[0]
    if len(mesh_list) > 1:
        bpy.ops.object.join()

    # Final cleanup
    final_obj.name = name
    final_obj.rotation_mode = "XYZ"
    final_obj.location = (0, 0, 0)
    final_obj.rotation_euler = (0, 0, 0)
    final_obj.scale = (1, 1, 1)

    # Remove all containers/stragglers identified in Step 1
    for obj in to_delete:
        with contextlib.suppress(RuntimeError, ReferenceError):
            bpy.data.objects.remove(obj, do_unlink=True)

    if bpy.context.view_layer:
        bpy.context.view_layer.objects.active = final_obj
    return final_obj


def _get_geometry_type_str(geometry: Box | Cylinder | Sphere | Mesh) -> str:
    """Get geometry type string from geometry instance."""
    geometry_type_map = {
        Box: "BOX",
        Cylinder: "CYLINDER",
        Sphere: "SPHERE",
        Mesh: "MESH",
    }
    for geom_class, type_str in geometry_type_map.items():
        if isinstance(geometry, geom_class):
            return type_str
    return "MESH"  # Default fallback


def create_link_object(
    link: Link,
    robot: Robot,
    urdf_dir: Path,
    collection: bpy.types.Collection | None = None,
) -> bpy.types.Object | None:
    """Create Blender object from Link model with support for multiple visual/collision elements.

    Args:
        link: Link model
        robot: Robot model (for resolving resources)
        urdf_dir: Directory containing URDF file (for resolving relative paths)
        collection: Blender Collection to add object to

    Returns:
        Blender Object or None (returns the link Empty object with properties)

    """
    # Create an Empty object to represent the link (always)
    # Using bpy.data.objects.new is safer than bpy.ops in asynchronous/timer environments
    link_obj = bpy.data.objects.new(link.name, None)
    link_obj.empty_display_type = "PLAIN_AXES"
    link_obj.rotation_mode = "XYZ"
    link_obj.location = (0, 0, 0)

    # Set display size from preferences
    prefs = get_addon_prefs()
    link_obj.empty_display_size = prefs.link_empty_size if prefs else 0.1

    # Add to collection
    if collection:
        move_to_collection(link_obj, collection)

    # Set link properties on the main link object
    if hasattr(link_obj, "linkforge"):
        props = link_obj.linkforge
        props.is_robot_link = True
        props.urdf_name_stored = link.name
        props.link_name = link.name

    # Create all visual geometries (URDF allows multiple <visual> per link)
    for idx, visual in enumerate(link.visuals):
        visual_obj = None

        # Determine visual object name (single vs multiple)
        if len(link.visuals) == 1:
            visual_name = f"{link.name}_visual"
        else:
            # Use URDF name attribute if available, otherwise use index
            suffix = visual.name if visual.name else str(idx)
            visual_name = f"{link.name}_visual_{suffix}"

        # Create geometry
        if isinstance(visual.geometry, Mesh):
            # Resolve mesh path using unified Robot resolver
            try:
                mesh_path = robot.resolve_resource(visual.geometry.resource, relative_to=urdf_dir)
                visual_obj = import_mesh_file(mesh_path, visual_name)
            except FileNotFoundError as e:
                logger.warning(f"Mesh not found for visual '{visual_name}': {e}")
                visual_obj = None

            # Apply scale from URDF
            if visual_obj and visual.geometry.scale:
                scale = visual.geometry.scale
                visual_obj.scale = (scale.x, scale.y, scale.z)
        else:
            # Create primitive geometry
            visual_obj = create_primitive_mesh(visual.geometry, visual_name)

        if visual_obj:
            # Parent to link object
            visual_obj.parent = link_obj

            # Reset matrix_parent_inverse to ensure clean local transform
            # Critical: Without this, visual geometry won't be centered at link frame
            visual_obj.matrix_parent_inverse.identity()

            # Apply visual origin transform (URDF visual offset)
            if visual.origin:
                origin = visual.origin
                visual_obj.rotation_mode = "XYZ"
                visual_obj.location = (origin.xyz.x, origin.xyz.y, origin.xyz.z)
                visual_obj.rotation_euler = (origin.rpy.x, origin.rpy.y, origin.rpy.z)
            else:
                visual_obj.rotation_mode = "XYZ"
                visual_obj.location = (0, 0, 0)
                visual_obj.rotation_euler = (0, 0, 0)

            # Store URDF name attribute for round-trip
            if visual.name:
                visual_obj["urdf_name"] = visual.name

            # Add visual mesh to collection
            if collection:
                move_to_collection(visual_obj, collection)

            # Apply material to visual mesh
            if visual.material and visual.material.color:
                mat = create_material_from_color(visual.material.color, visual.material.name)
                if mat and visual_obj.data and hasattr(visual_obj.data, "materials"):
                    # Use a localized ignore or cast if hasattr is not enough for MyPy's Mesh union
                    mesh_data = typing.cast(bpy.types.Mesh, visual_obj.data)
                    mesh_data.materials.clear()
                    mesh_data.materials.append(mat)

    # Create all collision geometries (URDF allows multiple <collision> per link)
    for idx, collision in enumerate(link.collisions):
        collision_obj = None

        # Determine collision object name (single vs multiple)
        if len(link.collisions) == 1:
            collision_name = f"{link.name}_collision"
        else:
            # Use URDF name attribute if available, otherwise use index
            suffix = collision.name if collision.name else str(idx)
            collision_name = f"{link.name}_collision_{suffix}"

        # Create geometry
        if isinstance(collision.geometry, Mesh):
            # Resolve mesh path using unified Robot resolver
            try:
                mesh_path = robot.resolve_resource(
                    collision.geometry.resource, relative_to=urdf_dir
                )
                collision_obj = import_mesh_file(mesh_path, collision_name)
            except FileNotFoundError as e:
                logger.warning(f"Mesh not found for collision '{collision_name}': {e}")
                collision_obj = None

            # Apply scale from URDF
            if collision_obj and collision.geometry.scale:
                scale = collision.geometry.scale
                collision_obj.scale = (scale.x, scale.y, scale.z)
        else:
            # Create primitive geometry
            collision_obj = create_primitive_mesh(collision.geometry, collision_name)

        if collision_obj:
            # Parent to link object
            collision_obj.parent = link_obj

            # Reset matrix_parent_inverse to ensure clean local transform
            # Critical: Without this, collision geometry won't be centered at link frame
            collision_obj.matrix_parent_inverse.identity()

            # Apply collision origin transform
            if collision.origin:
                origin = collision.origin
                collision_obj.rotation_mode = "XYZ"
                collision_obj.location = (origin.xyz.x, origin.xyz.y, origin.xyz.z)
                collision_obj.rotation_euler = (origin.rpy.x, origin.rpy.y, origin.rpy.z)
            else:
                collision_obj.rotation_mode = "XYZ"
                collision_obj.location = (0, 0, 0)
                collision_obj.rotation_euler = (0, 0, 0)

            # Store URDF name attribute for round-trip
            if collision.name:
                collision_obj["urdf_name"] = collision.name

            # Mark as imported to prevent re-simplification on export.
            # Without this, collision meshes degrade with each import-export cycle.
            collision_obj["imported_from_urdf"] = True

            # Add collision mesh to link's collections
            sync_object_collections(collision_obj, link_obj)

            # Clear materials from collision mesh (collision doesn't need materials)
            # Materials may come from imported mesh files (OBJ, DAE, etc.)
            if collision_obj.data and hasattr(collision_obj.data, "materials"):
                mesh_data = typing.cast(bpy.types.Mesh, collision_obj.data)
                mesh_data.materials.clear()

            # Set display properties for collision (wireframe, non-rendering)
            collision_obj.display_type = "WIRE"
            collision_obj.show_in_front = (
                True  # X-ray mode for consistency with generated collisions
            )
            collision_obj.hide_render = True

            # Set collision geometry type for UI consistency
            if isinstance(collision.geometry, Mesh):
                collision_obj["collision_geometry_type"] = "MESH"
            elif isinstance(collision.geometry, Box):
                collision_obj["collision_geometry_type"] = "BOX"
            elif isinstance(collision.geometry, Cylinder):
                collision_obj["collision_geometry_type"] = "CYLINDER"
            elif isinstance(collision.geometry, Sphere):
                collision_obj["collision_geometry_type"] = "SPHERE"

    # Set mass and inertia properties on link object
    if link.inertial and hasattr(link_obj, "linkforge"):
        props = link_obj.linkforge
        props.mass = link.inertial.mass

        if link.inertial.inertia:
            inertia = link.inertial.inertia
            props.inertia_ixx = inertia.ixx
            props.inertia_ixy = inertia.ixy
            props.inertia_ixz = inertia.ixz
            props.inertia_iyy = inertia.iyy
            props.inertia_iyz = inertia.iyz
            props.inertia_izz = inertia.izz
            props.use_auto_inertia = False  # Don't recalculate

        # Store inertial origin
        if link.inertial.origin:
            origin = link.inertial.origin
            props.inertia_origin_xyz = (origin.xyz.x, origin.xyz.y, origin.xyz.z)
            props.inertia_origin_rpy = (origin.rpy.x, origin.rpy.y, origin.rpy.z)

    elif hasattr(link_obj, "linkforge"):
        # If link has no inertial data (e.g. a dummy root link), set mass to 0
        # and disable auto-inertia to avoid falling back to the default 1.0 kg.
        props = link_obj.linkforge
        props.mass = 0.0
        props.use_auto_inertia = False

    # Set geometry types on link properties (use first element if multiple)
    if hasattr(link_obj, "linkforge"):
        props = link_obj.linkforge

        if link.collisions:
            # Set collision quality to 100% for imported meshes (preserve original detail)
            # This prevents degradation on re-export and gives user full control
            props.collision_quality = 100.0  # 100% preservation for imported collision

            # Set collision_type based on imported geometry (for UI display)
            # This ensures the UI shows the correct type instead of defaulting to "Auto"
            collision_geom_type = _get_geometry_type_str(link.collisions[0].geometry)
            if collision_geom_type in ("BOX", "CYLINDER", "SPHERE"):
                props.collision_type = collision_geom_type
            elif collision_geom_type == "MESH":
                # For mesh collisions, default to MESH (Simplified)
                # (most imported mesh collisions are simplified meshes)
                props.collision_type = "MESH"

        # Enable material export if imported URDF has material
        if link.visuals and link.visuals[0].material:
            props.use_material = True
            # Material name will come from Blender material assigned to visual child

    return link_obj


def create_joint_object(
    joint: Joint,
    link_objects: dict[str, bpy.types.Object],
    collection: bpy.types.Collection | None = None,
) -> bpy.types.Object | None:
    """Create Empty object from Joint model.

    Args:
        joint: Joint model
        link_objects: Dictionary mapping link names to Blender objects
        collection: Blender Collection to add object to

    Returns:
        Blender Empty object or None

    """
    empty_size = 0.2  # Default fallback
    prefs = get_addon_prefs()
    if prefs:
        empty_size = getattr(prefs, "joint_empty_size", empty_size)

    # Create Empty object (ARROWS shows RGB colored axes)
    # Using bpy.data.objects.new for context safety
    empty = bpy.data.objects.new(joint.name, None)
    empty.empty_display_type = "ARROWS"
    empty.empty_display_size = empty_size
    empty.rotation_mode = "XYZ"
    empty.location = (0, 0, 0)

    # Add to collection (hierarchy-aware)
    if collection:
        if isinstance(collection, bpy.types.Collection):
            collection.objects.link(empty)
        else:
            # It's an object, sync to its collections
            sync_object_collections(empty, collection)
    elif bpy.context.scene and bpy.context.scene.collection:
        bpy.context.scene.collection.objects.link(empty)

    # Set joint properties
    if hasattr(empty, "linkforge_joint"):
        props = empty.linkforge_joint
        props.is_robot_joint = True
        props.urdf_name_stored = joint.name
        props.joint_name = joint.name

        # Set joint type
        type_map = {
            "REVOLUTE": "REVOLUTE",
            "CONTINUOUS": "CONTINUOUS",
            "PRISMATIC": "PRISMATIC",
            "FIXED": "FIXED",
            "FLOATING": "FLOATING",
            "PLANAR": "PLANAR",
        }
        props.joint_type = type_map.get(joint.type.name, "FIXED")

        # Set parent and child links (PointerProperty expects Blender objects)
        props.parent_link = link_objects.get(joint.parent)
        props.child_link = link_objects.get(joint.child)

        # Set axis
        if joint.axis:
            # Check if it's a standard axis (X, Y, or Z)
            if joint.axis.x == 1.0 and joint.axis.y == 0.0 and joint.axis.z == 0.0:
                props.axis = "X"
            elif joint.axis.x == 0.0 and joint.axis.y == 1.0 and joint.axis.z == 0.0:
                props.axis = "Y"
            elif joint.axis.x == 0.0 and joint.axis.y == 0.0 and joint.axis.z == 1.0:
                props.axis = "Z"
            else:
                # Custom axis
                props.axis = "CUSTOM"
                props.custom_axis_x = joint.axis.x
                props.custom_axis_y = joint.axis.y
                props.custom_axis_z = joint.axis.z

        # Set limits
        if joint.limits:
            props.use_limits = True
            # Continuous joints may have None for lower/upper (no position limits)
            # Blender properties require float values, so use defaults for None
            props.limit_lower = joint.limits.lower if joint.limits.lower is not None else 0.0
            props.limit_upper = joint.limits.upper if joint.limits.upper is not None else 0.0
            props.limit_effort = joint.limits.effort
            props.limit_velocity = joint.limits.velocity

        # Set dynamics
        if joint.dynamics:
            props.use_dynamics = True
            props.dynamics_damping = joint.dynamics.damping
            props.dynamics_friction = joint.dynamics.friction

        # Set mimic
        if joint.mimic:
            props.use_mimic = True
            # Mimic joint pointer will be resolved in a second pass in import_robot_to_scene
            # because the mimicked joint might not be created yet.
            props.mimic_multiplier = joint.mimic.multiplier
            props.mimic_offset = joint.mimic.offset

        # Set safety controller
        if joint.safety_controller:
            props.use_safety_controller = True
            props.safety_soft_lower_limit = joint.safety_controller.soft_lower_limit
            props.safety_soft_upper_limit = joint.safety_controller.soft_upper_limit
            props.safety_k_position = joint.safety_controller.k_position
            props.safety_k_velocity = joint.safety_controller.k_velocity

        # Set calibration
        if joint.calibration:
            props.use_calibration = True
            if joint.calibration.rising is not None:
                props.use_calibration_rising = True
                props.calibration_rising = joint.calibration.rising
            if joint.calibration.falling is not None:
                props.use_calibration_falling = True
                props.calibration_falling = joint.calibration.falling

    # Set up parent-child relationship in Blender
    parent_obj = link_objects.get(joint.parent)
    child_obj = link_objects.get(joint.child)

    if parent_obj and child_obj:
        # Parent the joint Empty to the parent link
        empty.parent = parent_obj

        # Reset matrix_parent_inverse to ensure clean local transform
        empty.matrix_parent_inverse.identity()

        # Apply joint origin transform (offset from parent link frame)
        if joint.origin:
            origin = joint.origin
            empty.rotation_mode = "XYZ"
            empty.location = (origin.xyz.x, origin.xyz.y, origin.xyz.z)
            empty.rotation_euler = (origin.rpy.x, origin.rpy.y, origin.rpy.z)

        # Parent the child link to the joint Empty
        child_obj.parent = empty
        child_obj.matrix_parent_inverse.identity()

        # Force update of parent's world matrix
        _ = empty.matrix_world

        # Set child link's local position to origin (0,0,0)
        child_obj.rotation_mode = "XYZ"
        child_obj.location = (0, 0, 0)
        child_obj.rotation_euler = (0, 0, 0)
    else:
        logger.error(
            f"Failed to parent joint '{joint.name}': "
            f"Parent link '{joint.parent}' or child link '{joint.child}' not found in internal map. "
            f"Existing links: {list(link_objects.keys())}"
        )

    # Add to collection
    if collection:
        move_to_collection(empty, collection)

    # Joint visibility is controlled by RGB axes (GPU overlay) and empty display size
    # Empties are always visible in viewport, hide from render only
    empty.hide_render = True

    return empty


def create_sensor_object(
    sensor: typing.Any,
    link_objects: dict[str, bpy.types.Object],
    collection: bpy.types.Collection | None = None,
) -> bpy.types.Object | None:
    """Create Empty object from Sensor model.

    Args:
        sensor: Sensor model from core
        link_objects: Dictionary mapping link names to Blender objects
        collection: Blender Collection to add object to

    Returns:
        Blender Empty object or None

    """
    # Verify attached link exists
    if sensor.link_name not in link_objects:
        logger.warning(f"Sensor '{sensor.name}' attached to unknown link '{sensor.link_name}'")
        return None

    # Create Empty object for sensor (SPHERE for sensors)
    # Using bpy.data.objects.new for context safety
    empty = bpy.data.objects.new(sensor.name, None)
    empty.empty_display_type = "SPHERE"

    # Set display size from preferences
    prefs = get_addon_prefs()
    empty.empty_display_size = prefs.sensor_empty_size if prefs else 0.1

    # Set sensor properties
    if hasattr(empty, "linkforge_sensor"):
        props = empty.linkforge_sensor
        props.is_robot_sensor = True
        props.sensor_name = sensor.name

        # Map sensor type
        type_map = {
            "camera": "CAMERA",
            "depth_camera": "DEPTH_CAMERA",
            "lidar": "LIDAR",
            "gpu_lidar": "LIDAR",
            "imu": "IMU",
            "gps": "GPS",
            "contact": "CONTACT",
            "force_torque": "FORCE_TORQUE",
        }
        props.sensor_type = type_map.get(sensor.type.value, "CAMERA")

        # Set attached link (PointerProperty expects Blender object)
        props.attached_link = link_objects.get(sensor.link_name)

        # Common properties
        if sensor.update_rate:
            props.update_rate = sensor.update_rate
        if sensor.topic:
            props.topic_name = sensor.topic

        # Camera properties
        if sensor.camera_info:
            cam = sensor.camera_info
            props.camera_horizontal_fov = cam.horizontal_fov
            props.camera_width = cam.width
            props.camera_height = cam.height
            if cam.format:
                props.camera_format = cam.format
            if cam.near_clip:
                props.camera_near_clip = cam.near_clip
            if cam.far_clip:
                props.camera_far_clip = cam.far_clip

        # LIDAR properties
        if sensor.lidar_info:
            lidar = sensor.lidar_info
            props.lidar_horizontal_samples = lidar.horizontal_samples
            props.lidar_horizontal_min_angle = lidar.horizontal_min_angle
            props.lidar_horizontal_max_angle = lidar.horizontal_max_angle
            if lidar.vertical_samples:
                props.lidar_vertical_samples = lidar.vertical_samples
            props.lidar_range_min = lidar.range_min
            props.lidar_range_max = lidar.range_max

            # LIDAR noise
            if lidar.noise:
                props.use_noise = True
                props.noise_type = lidar.noise.type
                props.noise_mean = lidar.noise.mean
                props.noise_stddev = lidar.noise.stddev

        # IMU properties
        if sensor.imu_info:
            imu = sensor.imu_info
            # Gravity magnitude is handled by World settings

            # IMU noise (from angular velocity or linear acceleration)
            if imu.angular_velocity_noise:
                props.use_noise = True
                props.noise_type = imu.angular_velocity_noise.type
                props.noise_mean = imu.angular_velocity_noise.mean
                props.noise_stddev = imu.angular_velocity_noise.stddev

        # GPS properties (GPS info mainly contains noise)
        if sensor.gps_info:
            gps = sensor.gps_info
            if gps.position_sensing_horizontal_noise:
                props.use_noise = True
                props.noise_type = gps.position_sensing_horizontal_noise.type
                props.noise_mean = gps.position_sensing_horizontal_noise.mean
                props.noise_stddev = gps.position_sensing_horizontal_noise.stddev

        # Gazebo plugin
        if sensor.plugin:
            props.use_gazebo_plugin = True
            props.plugin_filename = sensor.plugin.filename
            # Store raw XML for round-trip fidelity
            if sensor.plugin.raw_xml:
                props.plugin_raw_xml = sensor.plugin.raw_xml

    # Set up parenting
    link_obj = link_objects[sensor.link_name]
    empty.parent = link_obj

    # Reset matrix_parent_inverse to ensure clean local transform
    empty.matrix_parent_inverse.identity()

    # Display basic properties
    if sensor.origin:
        origin = sensor.origin
        empty.rotation_mode = "XYZ"
        empty.location = (origin.xyz.x, origin.xyz.y, origin.xyz.z)
        empty.rotation_euler = (origin.rpy.x, origin.rpy.y, origin.rpy.z)

    # Add to collection
    if collection:
        move_to_collection(empty, collection)

    # Hide from render
    empty.hide_render = True

    return empty


def setup_scene_for_robot(scene: bpy.types.Scene, robot: Robot) -> None:
    """Initialize scene properties for a robot model.

    This populates the Centralized Control Dashboard, Gazebo settings,
    and metadata based on the robot model.

    Args:
        scene: Blender Scene object
        robot: Robot model to extract settings from
    """
    # Set robot name in scene properties
    if hasattr(scene, "linkforge"):
        scene.linkforge.robot_name = robot.name

    # Reset Global LinkForge State
    # Since LinkForge manages a single centralized configuration per scene,
    # Populate centralized ROS2 Control
    if robot.ros2_controls:
        lp = getattr(scene, "linkforge")
        lp.use_ros2_control = True
        control = robot.ros2_controls[0]
        lp.ros2_control_name = control.name
        lp.ros2_control_type = control.type
        lp.hardware_plugin = control.hardware_plugin

        # Map global parameters
        lp.ros2_control_parameters.clear()
        for key, value in control.parameters.items():
            param_item = lp.ros2_control_parameters.add()
            param_item.name = key
            param_item.value = value

        # Map joints
        lp.ros2_control_joints.clear()
        for rc_joint in control.joints:
            item = lp.ros2_control_joints.add()
            item.name = rc_joint.name
            item.cmd_position = "position" in rc_joint.command_interfaces
            item.cmd_velocity = "velocity" in rc_joint.command_interfaces
            item.cmd_effort = "effort" in rc_joint.command_interfaces
            item.state_position = "position" in rc_joint.state_interfaces
            item.state_velocity = "velocity" in rc_joint.state_interfaces
            item.state_effort = "effort" in rc_joint.state_interfaces

            # Map joint-level parameters
            item.parameters.clear()
            for key, value in rc_joint.parameters.items():
                param_item = item.parameters.add()
                param_item.name = key
                param_item.value = value
    # Master reset of ROS2 Control / Gazebo state if not present in robot
    if not robot.ros2_controls and hasattr(scene, "linkforge"):
        lp = scene.linkforge
        lp.use_ros2_control = False
        lp.ros2_control_joints.clear()
        lp.ros2_control_parameters.clear()
        lp.gazebo_plugin_name = "gz_ros2_control::GazeboSimROS2ControlPlugin"  # Default
        lp.controllers_yaml_path = ""

    # Map Gazebo simulation settings if present
    if robot.gazebo_elements and hasattr(scene, "linkforge"):
        plugin_found = False
        for elem in robot.gazebo_elements:
            for plugin in elem.plugins:
                if "ros2_control" in plugin.name.lower():
                    scene.linkforge.gazebo_plugin_name = plugin.name
                    if "parameters" in plugin.parameters:
                        scene.linkforge.controllers_yaml_path = plugin.parameters["parameters"]
                    plugin_found = True
                    break
            if plugin_found:
                break


def import_robot_to_scene(robot: Robot, urdf_path: Path, context: bpy.types.Context) -> bool:
    """Import Robot model to Blender scene.

    Args:
        robot: Robot model
        urdf_path: Path to URDF file
        context: Blender context

    Returns:
        True if import succeeded, False otherwise

    """
    # Setup global scene properties (ros2_control, metadata, etc.)
    if context.scene:
        setup_scene_for_robot(context.scene, robot)

    # Create collection for this robot
    collection = bpy.data.collections.new(robot.name)
    if context.scene:
        context.scene.collection.children.link(collection)

    # Get URDF directory for resolving mesh paths
    urdf_dir = urdf_path.parent

    # Count additional elements for better logging
    sensor_count = len(robot.sensors) if hasattr(robot, "sensors") else 0

    # Create link objects
    link_objects = {}
    parts = [f"{len(robot.links)} links", f"{len(robot.joints)} joints"]
    if sensor_count > 0:
        parts.append(f"{sensor_count} sensors")

    logger.info(f"Importing robot '{robot.name}' ({', '.join(parts)})")

    for link in robot.links:
        obj = create_link_object(link, robot, urdf_dir, collection)
        if obj:
            link_objects[link.name] = obj

    # Convert tuples to lists for MyPy compatibility with topological sort
    joints_list = list(robot.joints)
    links_list = list(robot.links)
    sorted_joints = sort_joints_topological(joints_list, links_list)

    # Create joint objects in topological order
    joint_objects = {}
    for joint in sorted_joints:
        joint_obj = create_joint_object(joint, link_objects, collection)
        if joint_obj:
            joint_objects[joint.name] = joint_obj

    # Second pass: Resolve mimic joint pointers
    # Convert tuple to list for MyPy compatibility
    all_joints_list = list(robot.joints)
    resolve_mimic_joints(all_joints_list, joint_objects)

    # Create sensor objects
    sensors_created = 0
    if hasattr(robot, "sensors") and robot.sensors:
        for sensor in robot.sensors:
            if create_sensor_object(sensor, link_objects, collection):
                sensors_created += 1

    # Update scene to ensure all transforms are calculated correctly
    # This is critical for round-trip: ensures child link locations are properly evaluated
    if context.view_layer:
        context.view_layer.update()

    # Build completion message
    completion_parts = [
        f"{len(link_objects)}/{len(robot.links)} links",
        f"{len(joint_objects)}/{len(robot.joints)} joints",
    ]
    if sensor_count > 0:
        completion_parts.append(f"{sensors_created}/{sensor_count} sensors")

    logger.info(f"Import complete - {', '.join(completion_parts)} created")
    # Sync collision visibility with scene property
    # This ensures that if 'Show Collisions' is off (default), the newly imported collision meshes are hidden
    if (
        context.scene
        and hasattr(context.scene, "linkforge")
        and hasattr(context.scene.linkforge, "update_collision_visibility")
    ):
        # We need to pass the context or property group self. Since update method expects self,
        # we can call the function directly or trigger property update.
        # Calling the update function bound to the property group instance is safest.
        context.scene.linkforge.update_collision_visibility(context)

    return True
