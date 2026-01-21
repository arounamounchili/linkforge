"""URDF importer utilities for creating Blender objects from URDF models."""

from __future__ import annotations

from pathlib import Path

import bpy

from ..core.logging_config import get_logger
from ..core.models import (
    Box,
    Color,
    Cylinder,
    Joint,
    Link,
    Mesh,
    Robot,
    Sphere,
)
from .preferences import get_addon_prefs

logger = get_logger(__name__)


def create_material_from_color(color: Color, name: str):
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
    nodes = mat.node_tree.nodes
    bsdf = None
    for node in nodes:
        if node.type == "BSDF_PRINCIPLED":
            bsdf = node
            break

    if bsdf:
        bsdf.inputs["Base Color"].default_value = (color.r, color.g, color.b, color.a)

    return mat


def create_primitive_mesh(geometry, name: str):
    """Create Blender mesh object from primitive geometry.

    Args:
        geometry: Box, Cylinder, or Sphere
        name: Object name

    Returns:
        Blender Object or None

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
                obj["urdf_geometry_type"] = "BOX"

        elif isinstance(geometry, Cylinder):
            bpy.ops.mesh.primitive_cylinder_add(location=(0, 0, 0))
            obj = bpy.context.active_object
            if obj:
                obj.name = name
                obj.dimensions = (geometry.radius * 2, geometry.radius * 2, geometry.length)
                obj["urdf_geometry_type"] = "CYLINDER"

        elif isinstance(geometry, Sphere):
            bpy.ops.mesh.primitive_uv_sphere_add(location=(0, 0, 0))
            obj = bpy.context.active_object
            if obj:
                obj.name = name
                obj.dimensions = (geometry.radius * 2, geometry.radius * 2, geometry.radius * 2)
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


def import_mesh_file(mesh_path: Path, name: str):
    """Import mesh file into Blender.

    Args:
        mesh_path: Path to mesh file (OBJ, STL, etc.)
        name: Object name

    Returns:
        Blender Object or None

    """
    if not mesh_path.exists():
        return None

    # Deselect all
    bpy.ops.object.select_all(action="DESELECT")

    # Import based on file extension
    ext = mesh_path.suffix.lower()

    try:
        if ext == ".obj":
            bpy.ops.wm.obj_import(filepath=str(mesh_path))
        elif ext == ".stl":
            bpy.ops.wm.stl_import(filepath=str(mesh_path))
        elif ext == ".dae":
            # DAE might not be available in all Blender versions (Removed in 5.0)
            if hasattr(bpy.ops.wm, "collada_import"):
                bpy.ops.wm.collada_import(filepath=str(mesh_path))
            else:
                msg = (
                    f"COLLADA (.dae) support was removed in Blender 5.0.\n"
                    f"Failed to import mesh '{mesh_path.name}'.\n"
                    "Please convert the mesh to .glb or .obj format."
                )
                logger.error(msg)
                return None
        elif ext in (".glb", ".gltf"):
            # Import GLB/GLTF (available in Blender 4.1+)
            bpy.ops.wm.gltf_import(filepath=str(mesh_path))
        else:
            return None

        # Get imported object (should be selected)
        imported_objects = [obj for obj in bpy.context.selected_objects]
        if imported_objects:
            obj = imported_objects[0]
            obj.name = name
            return obj

    except (RuntimeError, OSError) as e:
        logger.error(f"Failed to import mesh '{mesh_path.name}': {e}")
        return None
    except Exception as e:
        logger.critical(f"Unexpected error importing mesh '{mesh_path.name}': {e}", exc_info=True)
        raise

    return None


def create_link_object(link: Link, urdf_dir: Path, collection=None) -> object | None:
    """Create Blender object from Link model with support for multiple visual/collision elements.

    Args:
        link: Link model
        urdf_dir: Directory containing URDF file (for resolving mesh paths)
        collection: Blender Collection to add object to

    Returns:
        Blender Object or None (returns the link Empty object with properties)

    """
    # Create an Empty object to represent the link (always)
    # Use PLAIN_AXES type with sizing from preferences
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0, 0, 0))
    link_obj = bpy.context.active_object
    link_obj.name = link.name

    # Set display size from preferences
    prefs = get_addon_prefs()
    link_obj.empty_display_size = prefs.link_empty_size if prefs else 0.1

    # Add to collection
    if collection:
        for coll in list(link_obj.users_collection):
            if coll != collection:
                coll.objects.unlink(link_obj)
        if link_obj not in collection.objects[:]:
            collection.objects.link(link_obj)

    # Set link properties on the main link object
    if hasattr(link_obj, "linkforge"):
        props = link_obj.linkforge
        props.is_robot_link = True
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
            # Resolve mesh path relative to URDF directory
            mesh_path = urdf_dir / visual.geometry.filepath
            if not mesh_path.exists():
                # Try as absolute path
                mesh_path = visual.geometry.filepath

            visual_obj = import_mesh_file(mesh_path, visual_name)

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
                visual_obj.location = (origin.xyz.x, origin.xyz.y, origin.xyz.z)
                visual_obj.rotation_euler = (origin.rpy.x, origin.rpy.y, origin.rpy.z)

            else:
                visual_obj.location = (0, 0, 0)
                visual_obj.rotation_euler = (0, 0, 0)

            # Store URDF name attribute for round-trip
            if visual.name:
                visual_obj["urdf_name"] = visual.name

            # Add visual mesh to collection
            if collection:
                for coll in list(visual_obj.users_collection):
                    if coll != collection:
                        coll.objects.unlink(visual_obj)
                if visual_obj not in collection.objects[:]:
                    collection.objects.link(visual_obj)

            # Apply material to visual mesh
            if visual.material and visual.material.color:
                mat = create_material_from_color(visual.material.color, visual.material.name)
                if mat and visual_obj.data:
                    visual_obj.data.materials.clear()
                    visual_obj.data.materials.append(mat)

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
            # Resolve mesh path relative to URDF directory
            mesh_path = urdf_dir / collision.geometry.filepath
            if not mesh_path.exists():
                # Try as absolute path
                mesh_path = collision.geometry.filepath

            collision_obj = import_mesh_file(mesh_path, collision_name)

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
                collision_obj.location = (origin.xyz.x, origin.xyz.y, origin.xyz.z)
                collision_obj.rotation_euler = (origin.rpy.x, origin.rpy.y, origin.rpy.z)

            else:
                collision_obj.location = (0, 0, 0)
                collision_obj.rotation_euler = (0, 0, 0)

            # Store URDF name attribute for round-trip
            if collision.name:
                collision_obj["urdf_name"] = collision.name

            # CRITICAL FIX: Mark as imported to prevent re-simplification on export
            # Without this, collision meshes degrade with each import-export cycle
            # (100% → 50% → 25% → 12.5% → ... exponential degradation)
            collision_obj["imported_from_urdf"] = True

            # Add collision mesh to collection
            if collection:
                for coll in list(collision_obj.users_collection):
                    if coll != collection:
                        coll.objects.unlink(collision_obj)
                if collision_obj not in collection.objects[:]:
                    collection.objects.link(collision_obj)

            # Clear materials from collision mesh (collision doesn't need materials)
            # Materials may come from imported mesh files (OBJ, DAE, etc.)
            if collision_obj.data and hasattr(collision_obj.data, "materials"):
                collision_obj.data.materials.clear()

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

    # Helper to get geometry type string from instance
    def _get_geometry_type_str(geometry):
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
                # For mesh collisions, default to CONVEX_HULL
                # (most imported mesh collisions are convex hulls or simplified meshes)
                props.collision_type = "CONVEX_HULL"

        # Enable material export if imported URDF has material
        if link.visuals and link.visuals[0].material:
            props.use_material = True
            # Material name will come from Blender material assigned to visual child

    return link_obj


def create_joint_object(joint: Joint, link_objects: dict, collection=None) -> object | None:
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
    bpy.ops.object.empty_add(type="ARROWS", location=(0, 0, 0))
    empty = bpy.context.active_object
    empty.name = joint.name

    # Set display size from preferences
    empty.empty_display_size = empty_size

    # Set joint properties
    if hasattr(empty, "linkforge_joint"):
        props = empty.linkforge_joint
        props.is_robot_joint = True
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

    # Set up parent-child relationship in Blender
    if joint.parent in link_objects and joint.child in link_objects:
        parent_obj = link_objects[joint.parent]
        child_obj = link_objects[joint.child]

        # Parent the joint Empty to the parent link
        empty.parent = parent_obj

        # Reset matrix_parent_inverse to ensure clean local transform
        # This is critical for nested hierarchies - without it, the joint Empty's
        # visual position won't match its local coordinates
        empty.matrix_parent_inverse.identity()

        # Apply joint origin transform (offset from parent link frame)
        if joint.origin:
            origin = joint.origin
            empty.location = (origin.xyz.x, origin.xyz.y, origin.xyz.z)
            empty.rotation_euler = (origin.rpy.x, origin.rpy.y, origin.rpy.z)

        # Parent the child link to the joint Empty
        # This creates a cleaner hierarchy: Parent Link -> Joint -> Child Link
        child_obj.parent = empty

        # Reset matrix_parent_inverse to ensure clean local transform
        child_obj.matrix_parent_inverse.identity()

        # Force update of parent's world matrix before setting child position
        _ = empty.matrix_world  # Access to force recalculation

        # Set child link's local position to origin (0,0,0)
        # Since child is parented to joint, and joint is at the child frame origin,
        # the child link should be at local (0,0,0) relative to the joint.
        child_obj.location = (0, 0, 0)
        child_obj.rotation_euler = (0, 0, 0)

    # Add to collection
    if collection:
        # Link to target collection if not already there
        if empty not in collection.objects[:]:
            collection.objects.link(empty)
        # Remove from all other collections
        for coll in empty.users_collection[:]:
            if coll != collection:
                coll.objects.unlink(empty)

    # Joint visibility is controlled by RGB axes (GPU overlay) and empty display size
    # Empties are always visible in viewport, hide from render only
    empty.hide_render = True

    return empty


def create_sensor_object(sensor, link_objects: dict, collection=None) -> object | None:
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
    bpy.ops.object.empty_add(type="SPHERE", location=(0, 0, 0))
    empty = bpy.context.active_object
    empty.name = sensor.name

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
            # Smart Filter: Check for legacy Gazebo Classic plugins
            # If we find a known legacy plugin (libgazebo_ros_*), we SKIP importing it.
            # This ensures that when we re-export, we get a clean "Modern Gazebo Sim" definition
            # (which uses built-in sensor behavior instead of plugins).
            is_legacy_plugin = sensor.plugin.filename and "libgazebo_ros_" in sensor.plugin.filename

            if is_legacy_plugin:
                logger.info(
                    f"Modernizing: Dropping legacy plugin '{sensor.plugin.filename}' from sensor '{sensor.name}'"
                )
            else:
                # Only import if it's NOT a legacy plugin (e.g. custom user plugin)
                props.use_gazebo_plugin = True
                props.plugin_filename = sensor.plugin.filename
                # Store raw XML for round-trip fidelity
                if sensor.plugin.raw_xml:
                    props.plugin_raw_xml = sensor.plugin.raw_xml

    link_obj = link_objects[sensor.link_name]
    empty.parent = link_obj

    from mathutils import Matrix

    link_scale = link_obj.matrix_world.to_scale()
    scale_inv = Matrix.Diagonal((1.0 / link_scale.x, 1.0 / link_scale.y, 1.0 / link_scale.z, 1.0))
    empty.matrix_parent_inverse = scale_inv

    # Display basic properties
    if sensor.origin:
        origin = sensor.origin
        empty.location = (origin.xyz.x, origin.xyz.y, origin.xyz.z)
        empty.rotation_euler = (origin.rpy.x, origin.rpy.y, origin.rpy.z)

    # Add to collection
    if collection:
        for coll in list(empty.users_collection):
            if coll != collection:
                coll.objects.unlink(empty)
        if empty not in collection.objects[:]:
            collection.objects.link(empty)

    # Hide from render
    empty.hide_render = True

    return empty


def create_transmission_object(
    transmission,
    joint_objects: dict | None = None,
    collection=None,
    explicit_axis: tuple[float, float, float] | None = None,
) -> object | None:
    """Create Empty object from Transmission model.

    Args:
        transmission: Transmission model from core
        joint_objects: Dictionary mapping joint names to Blender Empty objects (joints)
        collection: Blender Collection to add object to
        explicit_axis: Optional explicit axis vector (x,y,z) to skip property lookup

    Returns:
        Blender Empty object or None

    """
    # Create Empty object for transmission (SINGLE_ARROW for actuation vector)
    bpy.ops.object.empty_add(type="SINGLE_ARROW", location=(0, 0, 0))
    empty = bpy.context.active_object
    empty.name = transmission.name

    # Set display size from preferences
    prefs = get_addon_prefs()
    empty.empty_display_size = prefs.transmission_empty_size if prefs else 0.05

    # ALIGNMENT:
    # By default, SINGLE_ARROW points +Z. We want it to point along the Joint Axis.
    target_axis = None

    # Priority 1: Explicit axis passed from importer (most reliable)
    if explicit_axis:
        target_axis = explicit_axis

    # Priority 2: Look up from joint object properties (fallback)
    elif joint_objects and transmission.joints:
        joint_name = transmission.joints[0].name
        if joint_name in joint_objects:
            joint_obj = joint_objects[joint_name]
            if hasattr(joint_obj, "linkforge_joint"):
                jp = joint_obj.linkforge_joint
                if jp.axis == "X":
                    target_axis = (1, 0, 0)
                elif jp.axis == "Y":
                    target_axis = (0, 1, 0)
                elif jp.axis == "Z":
                    target_axis = (0, 0, 1)
                elif jp.axis == "NEG_X":
                    target_axis = (-1, 0, 0)
                elif jp.axis == "NEG_Y":
                    target_axis = (0, -1, 0)
                elif jp.axis == "NEG_Z":
                    target_axis = (0, 0, -1)
                elif jp.axis == "CUSTOM":
                    target_axis = (jp.custom_axis_x, jp.custom_axis_y, jp.custom_axis_z)

    # Apply alignment if we have a target
    if target_axis:
        from mathutils import Vector

        vec = Vector(target_axis)
        if vec.length > 0:
            # 'TRACK' aligns Z axis to vector
            rot_quat = Vector((0, 0, 1)).rotation_difference(vec)
            empty.rotation_euler = rot_quat.to_euler()

    # Set transmission properties
    # (Original logic continues here)
    if hasattr(empty, "linkforge_transmission"):
        props = empty.linkforge_transmission
        props.is_robot_transmission = True
        props.transmission_name = transmission.name

        # Map transmission type
        type_map = {
            "transmission_interface/SimpleTransmission": "SIMPLE",
            "transmission_interface/DifferentialTransmission": "DIFFERENTIAL",
            "transmission_interface/FourBarLinkageTransmission": "FOUR_BAR_LINKAGE",
        }

        # Check if it's a known type or custom
        if transmission.type in type_map:
            props.transmission_type = type_map[transmission.type]
        else:
            props.transmission_type = "CUSTOM"
            props.custom_type = transmission.type

        # Set joints based on transmission type
        if len(transmission.joints) == 1:
            # Simple transmission
            joint = transmission.joints[0]
            if joint_objects:
                props.joint_name = joint_objects.get(joint.name)

            # Hardware interface
            if joint.hardware_interfaces:
                hw_iface = joint.hardware_interfaces[0]
                if "position" in hw_iface.lower():
                    props.hardware_interface = "POSITION"
                elif "velocity" in hw_iface.lower():
                    props.hardware_interface = "VELOCITY"
                elif "effort" in hw_iface.lower():
                    props.hardware_interface = "EFFORT"

            # Actuator properties (mechanical reduction typically on actuator in URDF)
            if transmission.actuators:
                actuator = transmission.actuators[0]
                props.use_custom_actuator_name = True
                props.actuator_name = actuator.name

                # Mechanical properties from actuator (standard URDF location)
                if actuator.mechanical_reduction:
                    props.mechanical_reduction = actuator.mechanical_reduction
                if actuator.offset:
                    props.offset = actuator.offset
            # Fallback to joint mechanical properties if actuator doesn't have them
            elif joint.mechanical_reduction:
                props.mechanical_reduction = joint.mechanical_reduction
            if joint.offset and not transmission.actuators:
                props.offset = joint.offset

        elif len(transmission.joints) == 2:
            # Differential transmission
            joint1 = transmission.joints[0]
            joint2 = transmission.joints[1]
            if joint_objects:
                props.joint1_name = joint_objects.get(joint1.name)
                props.joint2_name = joint_objects.get(joint2.name)

            # Hardware interface
            if joint1.hardware_interfaces:
                hw_iface = joint1.hardware_interfaces[0]
                if "position" in hw_iface.lower():
                    props.hardware_interface = "POSITION"
                elif "velocity" in hw_iface.lower():
                    props.hardware_interface = "VELOCITY"
                elif "effort" in hw_iface.lower():
                    props.hardware_interface = "EFFORT"

            # Mechanical properties from actuator (standard URDF location)
            if transmission.actuators and len(transmission.actuators) >= 1:
                actuator1 = transmission.actuators[0]
                if actuator1.mechanical_reduction:
                    props.mechanical_reduction = actuator1.mechanical_reduction
                if actuator1.offset:
                    props.offset = actuator1.offset
            # Fallback to joint mechanical properties if actuator doesn't have them
            elif joint1.mechanical_reduction:
                props.mechanical_reduction = joint1.mechanical_reduction
            if joint1.offset and not transmission.actuators:
                props.offset = joint1.offset

            # Actuator names
            if len(transmission.actuators) >= 2:
                props.use_custom_actuator_name = True
                props.actuator1_name = transmission.actuators[0].name
                props.actuator2_name = transmission.actuators[1].name

    # Parent transmission to the joint's location (if joint_objects provided)
    # Transmissions control joints, so we parent them to the joint Empty objects
    # This ensures they move with the robot when links are repositioned
    if joint_objects and transmission.joints:
        # Get the first joint (for simple transmissions or primary joint of differential)
        first_joint_name = transmission.joints[0].name
        if first_joint_name in joint_objects:
            joint_empty = joint_objects[first_joint_name]
            empty.parent = joint_empty

            # Snap to Joint Origin (Local 0,0,0) with aligned rotation
            empty.matrix_parent_inverse.identity()
            empty.location = (0, 0, 0)

            # Note: empty.rotation_euler was already set by ALIGNMENT logic above.
            # Since matrix_parent_inverse is identity, rotation_euler is relative to Parent Frame.
            # This is exactly what we want if we calculated rotation relative to local axes.

            # Move to correct collection
            if collection:
                for coll in list(empty.users_collection):
                    if coll != collection:
                        coll.objects.unlink(empty)
                if empty not in collection.objects[:]:
                    collection.objects.link(empty)
            # Reset local position to be at joint origin
            empty.location = (0, 0, 0)
            # Alignment rotation is already set above, do not reset it!

    # Add to collection
    if collection:
        for coll in list(empty.users_collection):
            if coll != collection:
                coll.objects.unlink(empty)
        if empty not in collection.objects[:]:
            collection.objects.link(empty)

    # Hide from render
    empty.hide_render = True

    return empty


def import_robot_to_scene(robot: Robot, urdf_path: Path, context) -> bool:
    """Import Robot model to Blender scene.

    Args:
        robot: Robot model
        urdf_path: Path to URDF file
        context: Blender context

    Returns:
        True if import succeeded, False otherwise

    """
    # Set robot name in scene properties
    scene = context.scene
    scene.linkforge.robot_name = robot.name

    # Check for ROS2 Control
    if robot.ros2_controls:
        scene.linkforge.use_ros2_control = True
    else:
        # Default to False if not present in imported URDF
        scene.linkforge.use_ros2_control = False

    # Create collection for this robot
    collection = bpy.data.collections.new(robot.name)
    context.scene.collection.children.link(collection)

    # Get URDF directory for resolving mesh paths
    urdf_dir = urdf_path.parent

    # Count additional elements for better logging
    sensor_count = len(robot.sensors) if hasattr(robot, "sensors") else 0
    transmission_count = len(robot.transmissions) if hasattr(robot, "transmissions") else 0

    # Create link objects
    link_objects = {}
    parts = [f"{len(robot.links)} links", f"{len(robot.joints)} joints"]
    if sensor_count > 0:
        parts.append(f"{sensor_count} sensors")
    if transmission_count > 0:
        parts.append(f"{transmission_count} transmissions")

    logger.info(f"Importing robot '{robot.name}' ({', '.join(parts)})")

    for link in robot.links:
        obj = create_link_object(link, urdf_dir, collection)
        if obj:
            link_objects[link.name] = obj

    # Sort joints in topological order (parents before children)
    # This ensures nested hierarchies are built correctly
    def sort_joints_topological(joints, links):
        """Sort joints so parents are processed before children."""
        # Build a map of which links are children
        child_links = {j.child for j in joints}
        # Find root links (not children of any joint)
        root_links = {link.name for link in links if link.name not in child_links}

        # Build adjacency list: parent_link -> [(joint, child_link), ...]
        children_of = {}
        for joint in joints:
            if joint.parent not in children_of:
                children_of[joint.parent] = []
            children_of[joint.parent].append(joint)

        # Traverse tree from roots, collecting joints in order
        sorted_joints = []
        visited = set()

        def visit(link_name):
            if link_name in visited:
                return
            visited.add(link_name)
            if link_name in children_of:
                for joint in children_of[link_name]:
                    sorted_joints.append(joint)
                    visit(joint.child)

        for root in root_links:
            visit(root)

        return sorted_joints

    sorted_joints = sort_joints_topological(robot.joints, robot.links)

    # Create joint objects in topological order
    joint_objects = {}
    for joint in sorted_joints:
        joint_obj = create_joint_object(joint, link_objects, collection)
        if joint_obj:
            joint_objects[joint.name] = joint_obj

    # Second pass: Resolve mimic joint pointers
    # This is necessary because URDF doesn't mandate mimic order relative to kinematic tree
    for joint in robot.joints:
        if joint.mimic and joint.name in joint_objects:
            joint_obj = joint_objects[joint.name]
            mimic_joint_obj = joint_objects.get(joint.mimic.joint)
            if mimic_joint_obj:
                joint_obj.linkforge_joint.mimic_joint = mimic_joint_obj

    # Create sensor objects
    sensors_created = 0
    if hasattr(robot, "sensors") and robot.sensors:
        for sensor in robot.sensors:
            if create_sensor_object(sensor, link_objects, collection):
                sensors_created += 1

    # Create transmission objects (pass joint_objects for parenting)
    transmissions_created = 0
    if hasattr(robot, "transmissions") and robot.transmissions:
        # Create joint map for looking up joint models (to get axis info)
        joint_model_map = {j.name: j for j in robot.joints}

        for transmission in robot.transmissions:
            # Look up explicit axis from joint model
            explicit_axis = None
            if transmission.joints:
                j_name = transmission.joints[0].name
                if j_name in joint_model_map:
                    ax = joint_model_map[j_name].axis
                    if ax:
                        explicit_axis = (ax.x, ax.y, ax.z)

            if create_transmission_object(
                transmission, joint_objects, collection, explicit_axis=explicit_axis
            ):
                transmissions_created += 1

    # Forward Compatibility: If no standard transmissions found, try to reconstruct from ros2_control
    # This allows importing "modern-only" URDFs and getting editable transmission objects in UI
    elif hasattr(robot, "ros2_controls") and robot.ros2_controls:
        logger.info(
            "No standard transmissions found. Attempting to reconstruct from ros2_control..."
        )

        # Create joint map if not created above (optimization: do standard check first)
        joint_model_map = {j.name: j for j in robot.joints}

        for rc in robot.ros2_controls:
            for joint in rc.joints:
                # Determine interface type (simplified heuristic)
                hw_iface = "POSITION"
                if "effort" in joint.command_interfaces:
                    hw_iface = "EFFORT"
                elif "velocity" in joint.command_interfaces:
                    hw_iface = "VELOCITY"

                # Check if we already have a transmission for this joint (avoid duplicates)
                # (Though standard transmissions loop above handles this, checking here is safe)

                # Create a pseudo-transmission object
                # We do this by calling create_transmission_object with a synthesized Transmission model
                # or by manually creating the object. Manually is safer here to avoid model dependency issues.

                if joint.name in joint_objects:
                    # Create Empty object for transmission
                    # Use SINGLE_ARROW to represent actuation vector (less cluttered than ARROWS)
                    bpy.ops.object.empty_add(type="SINGLE_ARROW", location=(0, 0, 0))
                    empty = bpy.context.active_object
                    empty.name = f"{joint.name}_trans"

                    # Set display size from preferences
                    prefs = get_addon_prefs()
                    empty.empty_display_size = prefs.transmission_empty_size if prefs else 0.05

                    if hasattr(empty, "linkforge_transmission"):
                        props = empty.linkforge_transmission
                        props.is_robot_transmission = True
                        props.transmission_name = f"{joint.name}_trans"
                        props.transmission_type = "SIMPLE"  # Default to simple
                        props.joint_name = joint_obj  # Assign the object pointer
                        props.hardware_interface = hw_iface

                    joint_obj = joint_objects[joint.name]
                    empty.parent = joint_obj
                    empty.matrix_parent_inverse.identity()
                    empty.location = (0, 0, 0)

                    # ALIGNMENT: Point arrow along Joint Axis
                    # Reconstruct axis vector from joint props (or model if available)
                    axis_vec = None
                    if joint.name in joint_model_map:
                        ax = joint_model_map[joint.name].axis
                        if ax:
                            axis_vec = (ax.x, ax.y, ax.z)

                    if axis_vec:
                        from mathutils import Vector

                        vec = Vector(axis_vec)
                        if vec.length > 0:
                            rot_quat = Vector((0, 0, 1)).rotation_difference(vec)
                            empty.rotation_euler = rot_quat.to_euler()
                    else:
                        empty.rotation_euler = (0, 0, 0)

                    # Add to collection
                    if collection:
                        for coll in list(empty.users_collection):
                            if coll != collection:
                                coll.objects.unlink(empty)
                        if empty not in collection.objects[:]:
                            collection.objects.link(empty)

                    transmissions_created += 1

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
    if transmission_count > 0:
        completion_parts.append(f"{transmissions_created}/{transmission_count} transmissions")

    logger.info(f"Import complete - {', '.join(completion_parts)} created")
    return True
