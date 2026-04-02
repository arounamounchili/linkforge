"""Converters between Blender properties and Core models.

These functions bridge the gap between Blender's property system
and LinkForge's core data models.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

try:
    import numpy as np  # type: ignore[import-not-found]
except ImportError:
    np = None

if TYPE_CHECKING:
    # Type stubs for Blender types when type checking
    bpy: Any
    Matrix: Any
    Vector: Any
else:
    import bpy
    from mathutils import Matrix

from dataclasses import dataclass

from ...linkforge_core.exceptions import RobotValidationError
from ...linkforge_core.logging_config import get_logger
from ...linkforge_core.models import (
    Box,
    CameraInfo,
    Collision,
    Color,
    ContactInfo,
    Cylinder,
    ForceTorqueInfo,
    GazeboPlugin,
    Geometry,
    GPSInfo,
    IMUInfo,
    Inertial,
    InertiaTensor,
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafetyController,
    JointType,
    LidarInfo,
    Link,
    Material,
    Mesh,
    Robot,
    Ros2Control,
    Ros2ControlJoint,
    Sensor,
    SensorNoise,
    SensorType,
    Sphere,
    Transform,
    Vector3,
    Visual,
)
from ...linkforge_core.models.transmission import (
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    TransmissionType,
)
from ...linkforge_core.physics import calculate_inertia, calculate_mesh_inertia_from_triangles
from ...linkforge_core.utils.math_utils import clean_float, normalize_vector
from ...linkforge_core.utils.string_utils import sanitize_name

# Constants
logger = get_logger(__name__)


def matrix_to_transform(matrix: Any) -> Transform:
    """Convert Blender 4x4 matrix to Transform.

    Args:
        matrix: Blender mathutils.Matrix (4x4)

    Returns:
        Core Transform with XYZ position and RPY rotation.

    """
    if matrix is None or Matrix is None:
        return Transform.identity()

    # Extract translation and rotation (Euler angles in radians)
    translation = matrix.to_translation()
    rotation = matrix.to_euler("XYZ")

    xyz = Vector3(
        clean_float(translation.x),
        clean_float(translation.y),
        clean_float(translation.z),
    )
    rpy = Vector3(
        clean_float(rotation.x),
        clean_float(rotation.y),
        clean_float(rotation.z),
    )

    return Transform(xyz=xyz, rpy=rpy)


@dataclass(frozen=True)
class PrimitiveDetectionConfig:
    """Configuration for primitive shape detection from Blender meshes.

    Start with specific vertex counts and use bounding box ratios to
    fuzzy match geometry.
    """

    # Cube detection - exact match required
    cube_vert_count: int = 8  # Cubes always have 8 vertices
    cube_face_count: int = 6  # Cubes always have 6 faces
    cube_verts_per_face: int = 4  # Each face has 4 vertices

    # Sphere detection (UV Sphere with various subdivision levels)
    # Based on Blender UV Sphere: 16 segments × 8 rings = 240 verts (minimum acceptable)
    sphere_min_verts: int = 240  # Minimum for low-poly spheres (less may be too coarse)
    sphere_max_verts: int = (
        1000  # Maximum for high-poly spheres (prevents complex mesh false positives)
    )
    sphere_min_faces: int = 240  # Minimum face count
    sphere_max_faces: int = 1000  # Maximum face count
    # Empirically determined: 0.9 allows for minor mesh imperfections while rejecting non-spherical shapes
    sphere_uniformity_tolerance: float = (
        0.9  # Dimensions within 10% to be spherical (1.0 = perfect sphere)
    )

    # Cylinder detection (default 32 vertices, supports 16-64 range)
    cylinder_min_verts: int = 32  # Minimum vertices (16-sided cylinder minimum)
    cylinder_max_verts: int = 128  # Maximum vertices (64-sided cylinder maximum)
    cylinder_min_faces: int = 18  # 16 vertices = 16 side faces + 2 caps
    cylinder_max_faces: int = 66  # 64 vertices = 64 side faces + 2 caps
    cylinder_base_tolerance: float = 0.9  # XY ratio must be > 0.9 for circular base
    cylinder_height_min_ratio: float = 0.9  # Z/XY ratio boundaries to distinguish from sphere
    cylinder_height_max_ratio: float = 1.1  # If height/radius ratio is 0.9-1.1, might be sphere


# Default primitive detection configuration
# Users can override by creating a custom config and passing it to detection functions
DEFAULT_PRIMITIVE_CONFIG = PrimitiveDetectionConfig()


def detect_primitive_type(obj: bpy.types.Object | None) -> str | None:
    """Detect if a Blender mesh object matches a standard primitive shape.

    Analyzes topology and dimensions to determine if the object can be
    exported as a URDF primitive (BOX, CYLINDER, or SPHERE). This function
    is critical for optimizing exports and ensuring compatibility with
    physics simulators.

    Args:
        obj: The Blender mesh object to analyze.

    Returns:
        "BOX", "CYLINDER", or "SPHERE" if a match is detected, else None.
    """
    if obj is None or obj.type != "MESH":
        return None

    mesh = obj.data
    if mesh is None:
        return None

    # Check for explicit geometry type tags
    # This guarantees round-trip stability and prevents auto-detection failures
    tags = ["urdf_geometry_type", "collision_geometry_type"]
    for tag in tags:
        if tag in obj:
            geom_type = str(obj[tag])
            if geom_type in ("BOX", "CYLINDER", "SPHERE"):
                return geom_type
            if geom_type == "MESH":
                return None

    # Count vertices and faces
    vert_count = len(mesh.vertices)
    face_count = len(mesh.polygons)

    # Get config for primitive detection thresholds
    config = DEFAULT_PRIMITIVE_CONFIG

    # Match Box: 8 vertices, 6 quad faces
    if vert_count == config.cube_vert_count and face_count == config.cube_face_count:
        # Verify it's roughly box-shaped by checking if all faces are quads
        all_quads = all(len(poly.vertices) == config.cube_verts_per_face for poly in mesh.polygons)
        if all_quads:
            return "BOX"

    # UV Sphere: Variable subdivision levels
    # Default (32 segs, 16 rings) = 482 verts, 480 faces
    if (
        config.sphere_min_verts <= vert_count <= config.sphere_max_verts
        and config.sphere_min_faces <= face_count <= config.sphere_max_faces
    ):
        # Check if roughly spherical (all dimensions similar)
        dims = obj.dimensions
        if dims.x > 0 and dims.y > 0 and dims.z > 0:
            max_dim = max(dims.x, dims.y, dims.z)
            min_dim = min(dims.x, dims.y, dims.z)
            # Within tolerance (sphere should be uniform)
            if min_dim / max_dim > config.sphere_uniformity_tolerance:
                return "SPHERE"

    # Cylinder: Variable vertex counts (16, 32, 64 typical)
    # Formula: verts = segments * 2, faces = segments + 2 (caps)
    if (
        config.cylinder_min_verts <= vert_count <= config.cylinder_max_verts
        and config.cylinder_min_faces <= face_count <= config.cylinder_max_faces
    ):
        # Check if roughly cylindrical (two dimensions similar, one different)
        dims = obj.dimensions
        if dims.x > 0 and dims.y > 0 and dims.z > 0:
            # XY should be similar (cylinder base), Z different (height)
            xy_ratio = min(dims.x, dims.y) / max(dims.x, dims.y)
            # XY dimensions must form circular base
            if xy_ratio > config.cylinder_base_tolerance:
                # Z should be different from XY (not a sphere)
                z_vs_xy = dims.z / max(dims.x, dims.y)
                if (
                    z_vs_xy < config.cylinder_height_min_ratio
                    or z_vs_xy > config.cylinder_height_max_ratio
                ):
                    return "CYLINDER"

    # If none match, it's a complex mesh
    return None


def get_object_geometry(
    obj: Any,
    geometry_type: str = "AUTO",
    link_name: str | None = None,
    geom_purpose: str = "visual",
    meshes_dir: Path | None = None,
    mesh_format: str = "STL",
    simplify: bool = False,
    decimation_ratio: float = 0.5,
    dry_run: bool = False,
    suffix: str = "",
    depsgraph: Any | None = None,
) -> tuple[Geometry | None, Matrix]:
    """Extract geometry from Blender object.

    Args:
        obj: Blender Object
        geometry_type: Type of geometry to extract
            - "AUTO": Auto-detect (primitives for simple shapes, mesh for complex)
            - "MESH": Force mesh export
            - "BOX", "CYLINDER", "SPHERE": Force specific primitive
        link_name: Name of the link (for mesh filename)
        geom_purpose: "visual" or "collision" (for mesh filename)
        meshes_dir: Directory to export mesh files to
        mesh_format: "STL", "OBJ", or "GLB"
        simplify: Whether to simplify mesh (for collision)
        decimation_ratio: Simplification ratio if simplify=True
        dry_run: If True, generate mesh paths but don't write files
        suffix: Optional unique suffix (e.g., index or name)

    Returns:
        tuple of (Core Geometry or None, geometry_world_matrix)

    """
    if obj is None:
        return None, Matrix.Identity(4)

    # Determine actual geometry type to use (AUTO requires detection)
    actual_geometry_type = geometry_type
    if actual_geometry_type == "AUTO":
        detected_type = detect_primitive_type(obj)
        # Use detected primitive (cleaner URDF) or fallback to mesh for complex shapes
        actual_geometry_type = detected_type or "MESH"

    if actual_geometry_type == "MESH":
        # Export actual mesh file if meshes_dir is provided
        if meshes_dir and link_name and obj.type == "MESH":
            from .mesh_io import export_link_mesh

            mesh_path, geom_world_matrix = export_link_mesh(
                obj=obj,
                link_name=link_name,
                geometry_type=geom_purpose,
                mesh_format=mesh_format,
                meshes_dir=meshes_dir,
                simplify=simplify,
                decimation_ratio=decimation_ratio,
                dry_run=dry_run,
                suffix=suffix,
                depsgraph=depsgraph,
            )

            if mesh_path:
                # Return Mesh geometry with file path
                return Mesh(
                    resource=str(mesh_path), scale=Vector3(1.0, 1.0, 1.0)
                ), geom_world_matrix

        # Fallback: approximate with bounding box if export failed or not requested
        actual_geometry_type = "BOX"

    # For primitives, the pose is just the current object matrix
    geom_world_matrix = obj.matrix_world

    if actual_geometry_type == "BOX":
        # Use bounding box dimensions
        dimensions = obj.dimensions
        # Robustness Check: Skip zero-size objects (e.g. empties from failed imports)
        if dimensions.length < 1e-6:
            logger.warning(f"Skipping geometry for '{obj.name}': Dimensions are zero.")
            return None, Matrix.Identity(4)

        return Box(size=Vector3(dimensions.x, dimensions.y, dimensions.z)), geom_world_matrix

    elif actual_geometry_type == "CYLINDER":
        # Approximate with bounding cylinder
        dimensions = obj.dimensions
        radius = max(dimensions.x, dimensions.y) / 2.0
        length = dimensions.z
        return Cylinder(radius=radius, length=length), geom_world_matrix

    elif actual_geometry_type == "SPHERE":
        # Approximate with bounding sphere
        radius = max(obj.dimensions) / 2.0
        return Sphere(radius=radius), geom_world_matrix

    return None, Matrix.Identity(4)


def extract_mesh_triangles(
    obj: Any,
    depsgraph: Any | None = None,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]] | None:
    """Extract triangle mesh data from Blender object.

    Args:
        obj: Blender mesh object

    Returns:
        Tuple of (vertices, triangles) or None if not a mesh
        vertices: List of (x, y, z) coordinates
        triangles: List of (v0, v1, v2) triangle vertex indices

    """
    if obj is None or obj.type != "MESH":
        return None

    # Get evaluated mesh (with modifiers applied)
    if depsgraph is None:
        depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    mesh = eval_obj.to_mesh()

    if mesh is None:
        return None

    # Ensure mesh has triangulated faces
    mesh.calc_loop_triangles()

    # We use the scale matrix (not full world matrix) to get correct dimensions
    # but keep the object centered at its local origin for proper inertia calculation
    # The inertia tensor is always computed relative to the object's center of mass
    scale_matrix = obj.matrix_world.to_scale()

    # NumPy-accelerated extraction for O(N) mesh processing (avoids Python loop overhead)
    if np is not None:
        # Fast vertex extraction via foreach_get
        num_verts = len(mesh.vertices)
        verts = np.empty(num_verts * 3, dtype=np.float32)
        mesh.vertices.foreach_get("co", verts)
        vertices_array = verts.reshape((-1, 3))
        # Apply scale
        vertices_array[:, 0] *= scale_matrix.x
        vertices_array[:, 1] *= scale_matrix.y
        vertices_array[:, 2] *= scale_matrix.z
        vertices_list = vertices_array.tolist()

        # Fast triangle extraction via loop_triangles
        num_tris = len(mesh.loop_triangles)
        tris = np.empty(num_tris * 3, dtype=np.int32)
        mesh.loop_triangles.foreach_get("vertices", tris)
        triangles_list = tris.reshape((-1, 3)).tolist()

        # Cleanup memory
        eval_obj.to_mesh_clear()
        return vertices_list, triangles_list

    # Pure Python Fallback (if NumPy is missing)
    vertices = [
        (v.co.x * scale_matrix.x, v.co.y * scale_matrix.y, v.co.z * scale_matrix.z)
        for v in mesh.vertices
    ]
    triangles = [tuple(t.vertices) for t in mesh.loop_triangles]

    # Cleanup memory
    eval_obj.to_mesh_clear()
    return vertices, triangles


def get_object_material(obj: Any, props: Any) -> Material | None:
    """Extract material from Blender object.

    Args:
        obj: Blender Object
        props: LinkPropertyGroup with material settings

    Returns:
        Core Material or None

    """
    if not props.use_material:
        return None

    # Use Blender material name, sanitized for XACRO compatibility
    mat_name = f"{sanitize_name(obj.name)}_material"  # Default fallback
    if obj.material_slots and obj.material_slots[0].material:
        # Sanitize material name to be valid Python identifier (required for XACRO)
        mat_name = sanitize_name(obj.material_slots[0].material.name)

    # Extract color from Blender material (if assigned)
    color = None
    if obj.material_slots and obj.material_slots[0].material:
        blender_mat = obj.material_slots[0].material

        # Try to get color from Principled BSDF node (modern Blender)
        if blender_mat.use_nodes and blender_mat.node_tree:
            # Find Principled BSDF node
            for node in blender_mat.node_tree.nodes:
                if node.type == "BSDF_PRINCIPLED":
                    # Get Base Color input
                    base_color_input = node.inputs["Base Color"]
                    base_color = base_color_input.default_value
                    color = Color(
                        r=base_color[0],
                        g=base_color[1],
                        b=base_color[2],
                        a=base_color[3] if len(base_color) > 3 else 1.0,
                    )
                    break

        # Fallback to viewport display color if no node shader
        if color is None:
            diffuse = blender_mat.diffuse_color
            color = Color(r=diffuse[0], g=diffuse[1], b=diffuse[2], a=diffuse[3])

    # If no Blender material assigned, use default gray
    if color is None:
        color = Color(0.8, 0.8, 0.8, 1.0)

    return Material(name=mat_name, color=color)


def blender_link_to_core_with_origin(
    obj: Any,
    meshes_dir: Path | None = None,
    robot_props: Any = None,
    dry_run: bool = False,
    depsgraph: Any | None = None,
) -> Link | None:
    """Convert Blender link (Empty with children) to Core Link with multiple visual/collision support.

    Args:
        obj: Blender Empty object with linkforge property group and visual/collision children
        meshes_dir: Optional directory for exporting mesh files
        robot_props: Robot property group with export settings
        dry_run: If True, generate mesh paths but don't write files

    Returns:
        Core Link model or None

    """
    if obj is None:
        return None

    props = obj.linkforge
    if not props.is_robot_link:
        return None

    link_name = props.link_name if props.link_name else obj.name

    # Get mesh format from robot props
    mesh_format = robot_props.mesh_format if robot_props else "STL"

    # Find all visual and collision geometry objects (children with _visual or _collision in name)
    visuals: list[Visual] = []
    collisions: list[Collision] = []

    visual_count = 0
    collision_count = 0

    # Pre-calculate totals for clean naming logic
    total_visuals = sum(1 for c in obj.children if "_visual" in c.name)
    total_collisions = sum(1 for c in obj.children if "_collision" in c.name)

    for child in obj.children:
        child_name = child.name

        if "_visual" in child_name:
            # Extract material
            material = get_object_material(child, props)

            # Determine unique suffix (use urdf_name if present, otherwise counter)
            urdf_name = child.get("urdf_name", None)
            if urdf_name:
                suffix = f"_{sanitize_name(urdf_name)}"
            elif total_visuals > 1:
                suffix = f"_{visual_count}"
            else:
                suffix = ""

            visual_count += 1

            # Get geometry (auto-detect: primitives for simple shapes, mesh for complex)
            visual_geom, geom_world_matrix = get_object_geometry(
                obj=child,
                geometry_type="AUTO",  # Auto-detect primitive vs mesh
                link_name=link_name,
                geom_purpose="visual",
                meshes_dir=meshes_dir,
                mesh_format=mesh_format,
                simplify=False,
                decimation_ratio=1.0,
                dry_run=dry_run,
                suffix=suffix,
                depsgraph=depsgraph,
            )

            # Extract relative origin (Relative_Pose = Link_Inv @ Geometry_World_Pose)
            relative_matrix = obj.matrix_world.inverted() @ geom_world_matrix
            origin = matrix_to_transform(relative_matrix)

            if visual_geom:
                visuals.append(
                    Visual(geometry=visual_geom, origin=origin, material=material, name=urdf_name)
                )

        elif "_collision" in child_name:
            # Check if this collision was imported from URDF (skip simplification)
            is_imported = child.get("imported_from_urdf", False)

            # Determine simplification ratio based on collision quality setting
            quality_percent = props.collision_quality
            quality_ratio = quality_percent / 100.0  # Convert 0-100 to 0.0-1.0

            # Only simplify if quality < 100% AND not imported
            should_simplify = (quality_ratio < 1.0) and not is_imported

            # Check if collision has stored geometry type
            stored_geom_type = child.get("collision_geometry_type", "AUTO")

            # Determine unique suffix
            urdf_name = child.get("urdf_name", None)
            if urdf_name:
                suffix = f"_{sanitize_name(urdf_name)}"
            elif total_collisions > 1:
                suffix = f"_{collision_count}"
            else:
                suffix = ""

            collision_count += 1

            # Get geometry
            collision_geom, geom_world_matrix = get_object_geometry(
                obj=child,
                geometry_type=stored_geom_type,
                link_name=link_name,
                geom_purpose="collision",
                meshes_dir=meshes_dir,
                mesh_format="STL",  # Collision always STL
                simplify=should_simplify,
                decimation_ratio=quality_ratio,
                dry_run=dry_run,
                suffix=suffix,
                depsgraph=depsgraph,
            )

            # EXTRACT RELATIVE ORIGIN
            relative_matrix = obj.matrix_world.inverted() @ geom_world_matrix
            origin = matrix_to_transform(relative_matrix)

            if collision_geom:
                collisions.append(Collision(geometry=collision_geom, origin=origin, name=urdf_name))

    # Inertial properties
    inertial = None
    if props.mass > 0:
        if props.use_auto_inertia:
            # Auto-calculate inertia from geometry
            # Prefer collision geometry, fallback to visual if no collision exists
            geom_obj = None
            geom = None

            # First try collision
            for child in obj.children:
                if "_collision" in child.name and collisions:
                    geom_obj = child
                    geom = collisions[0].geometry
                    break

            # Fallback to visual if no collision
            if geom_obj is None:
                for child in obj.children:
                    if "_visual" in child.name and visuals:
                        geom_obj = child
                        geom = visuals[0].geometry
                        break

            if geom and geom_obj:
                # Calculate inertia based on geometry type
                if isinstance(geom, Mesh) and geom_obj.type == "MESH":
                    mesh_data = extract_mesh_triangles(geom_obj, depsgraph=depsgraph)
                    if mesh_data:
                        vertices, triangles = mesh_data
                        inertia_tensor = calculate_mesh_inertia_from_triangles(
                            vertices, triangles, props.mass
                        )
                    else:
                        # Fallback to bounding box if mesh extraction fails
                        dimensions = geom_obj.dimensions
                        bbox_geom = Box(size=Vector3(dimensions.x, dimensions.y, dimensions.z))
                        inertia_tensor = calculate_inertia(bbox_geom, props.mass)
                else:
                    # Use primitive geometry (Box, Sphere, Cylinder)
                    inertia_tensor = calculate_inertia(geom, props.mass)
            else:
                # No geometry available - use default manual values
                inertia_tensor = InertiaTensor(
                    ixx=props.inertia_ixx,
                    ixy=props.inertia_ixy,
                    ixz=props.inertia_ixz,
                    iyy=props.inertia_iyy,
                    iyz=props.inertia_iyz,
                    izz=props.inertia_izz,
                )
        else:
            # Use manual inertia
            inertia_tensor = InertiaTensor(
                ixx=props.inertia_ixx,
                ixy=props.inertia_ixy,
                ixz=props.inertia_ixz,
                iyy=props.inertia_iyy,
                iyz=props.inertia_iyz,
                izz=props.inertia_izz,
            )

        # Inertial properties (mass and inertia tensor)
        # Use stored inertia origin from link properties
        inertial_origin = Transform(
            xyz=Vector3(
                clean_float(props.inertia_origin_xyz[0]),
                clean_float(props.inertia_origin_xyz[1]),
                clean_float(props.inertia_origin_xyz[2]),
            ),
            rpy=Vector3(
                clean_float(props.inertia_origin_rpy[0]),
                clean_float(props.inertia_origin_rpy[1]),
                clean_float(props.inertia_origin_rpy[2]),
            ),
        )
        inertial = Inertial(mass=props.mass, origin=inertial_origin, inertia=inertia_tensor)

    return Link(
        name=link_name, initial_visuals=visuals, initial_collisions=collisions, inertial=inertial
    )


def blender_joint_to_core(obj: Any) -> Joint | None:
    """Convert Blender Empty with JointPropertyGroup to Core Joint.

    Args:
        obj: Blender Empty object with linkforge_joint property group

    Returns:
        Core Joint model or None

    """
    if obj is None:
        return None

    props = obj.linkforge_joint
    if not props.is_robot_joint:
        return None

    joint_name = props.joint_name if props.joint_name else obj.name

    # Joint type
    joint_type = JointType(props.joint_type.lower())

    # Joint axis
    # Valid axis for joints that support it
    if joint_type in (JointType.FIXED, JointType.FLOATING):
        # FIXED and FLOATING joints MUST NOT have an axis per URDF strict mode
        axis = None
    elif props.axis == "X":
        axis = Vector3(1.0, 0.0, 0.0)
    elif props.axis == "Y":
        axis = Vector3(0.0, 1.0, 0.0)
    elif props.axis == "Z":
        axis = Vector3(0.0, 0.0, 1.0)
    else:  # CUSTOM
        # URDF spec requires unit vectors for joint axes - normalize custom axes
        nx, ny, nz = normalize_vector(props.custom_axis_x, props.custom_axis_y, props.custom_axis_z)
        if nx == 0 and ny == 0 and nz == 0:
            # Zero vector - fallback to default Z-axis
            logger.warning(
                f"Joint '{joint_name}' has zero-length custom axis, using default Z-axis"
            )
            axis = Vector3(0.0, 0.0, 1.0)
        else:
            axis = Vector3(nx, ny, nz)

    # Joint origin is already calculated relative to parent in blender_to_core.scene_to_robot
    # Just use the joint's world transform here, will be made relative in scene_to_robot
    origin = matrix_to_transform(obj.matrix_world)

    # Joint limits
    # Per URDF spec:
    # - REVOLUTE/PRISMATIC: limits are REQUIRED
    # - CONTINUOUS: limits are OPTIONAL (only effort/velocity, no position limits)
    # - FIXED/FLOATING/PLANAR: limits are NOT ALLOWED
    limits = None
    if joint_type in (JointType.REVOLUTE, JointType.PRISMATIC):
        # Always export limits for REVOLUTE and PRISMATIC (required by URDF spec)
        limits = JointLimits(
            lower=props.limit_lower,
            upper=props.limit_upper,
            effort=props.limit_effort,
            velocity=props.limit_velocity,
        )
    elif joint_type == JointType.CONTINUOUS and props.use_limits:
        # Optional limits for CONTINUOUS (only effort/velocity, no position limits)
        limits = JointLimits(
            effort=props.limit_effort,
            velocity=props.limit_velocity,
        )

    # Dynamics
    dynamics = None
    if props.use_dynamics:
        dynamics = JointDynamics(
            damping=props.dynamics_damping,
            friction=props.dynamics_friction,
        )

    # Mimic
    mimic = None
    if props.use_mimic and props.mimic_joint:
        mimic_obj = props.mimic_joint
        mimic_joint_name = ""
        if mimic_obj:
            if hasattr(mimic_obj, "linkforge_joint") and mimic_obj.linkforge_joint.joint_name:
                mimic_joint_name = mimic_obj.linkforge_joint.joint_name
            else:
                mimic_joint_name = sanitize_name(mimic_obj.name)

        if mimic_joint_name:
            mimic = JointMimic(
                joint=mimic_joint_name,
                multiplier=props.mimic_multiplier,
                offset=props.mimic_offset,
            )

    # Handle PointerProperty for parent/child links
    parent_obj = props.parent_link
    child_obj = props.child_link

    parent = parent_obj.linkforge.link_name if parent_obj else ""
    child = child_obj.linkforge.link_name if child_obj else ""

    if not parent:
        raise RobotValidationError(
            "ParentLink", joint_name, "Joint has no parent link. Please select a Parent Link."
        )
    if not child:
        raise RobotValidationError(
            "ChildLink", joint_name, "Joint has no child link. Please select a Child Link."
        )

    return Joint(
        name=joint_name,
        type=joint_type,
        parent=parent,
        child=child,
        origin=origin,
        axis=axis,
        limits=limits,
        dynamics=dynamics,
        mimic=mimic,
        safety_controller=JointSafetyController(
            soft_lower_limit=props.safety_soft_lower_limit,
            soft_upper_limit=props.safety_soft_upper_limit,
            k_position=props.safety_k_position,
            k_velocity=props.safety_k_velocity,
        )
        if props.use_safety_controller
        else None,
        calibration=JointCalibration(
            rising=props.calibration_rising if props.use_calibration_rising else None,
            falling=props.calibration_falling if props.use_calibration_falling else None,
        )
        if props.use_calibration
        else None,
    )


def blender_transmission_to_core(obj: Any) -> Transmission | None:
    """Convert Blender Empty with TransmissionPropertyGroup to Core Transmission.

    Args:
        obj: Blender Empty object with linkforge_transmission property group

    Returns:
        Core Transmission model or None

    """
    if obj is None:
        return None

    props = obj.linkforge_transmission
    if not props.is_robot_transmission:
        return None

    trans_name = props.transmission_name if props.transmission_name else obj.name

    # Transmission type mapping
    trans_type_map = {
        "SIMPLE": TransmissionType.SIMPLE.value,
        "DIFFERENTIAL": TransmissionType.DIFFERENTIAL.value,
        "FOUR_BAR_LINKAGE": TransmissionType.FOUR_BAR_LINKAGE.value,
        "CUSTOM": props.custom_type if props.custom_type else TransmissionType.CUSTOM.value,
    }
    trans_type = trans_type_map.get(props.transmission_type, TransmissionType.SIMPLE.value)

    # Hardware interface mapping
    hw_if_map = {
        "POSITION": "position",
        "VELOCITY": "velocity",
        "EFFORT": "effort",
    }
    hw_if = hw_if_map.get(props.hardware_interface, "position")

    joints = []
    actuators = []

    if props.transmission_type in ("SIMPLE", "CUSTOM", "FOUR_BAR_LINKAGE"):
        joint_obj = props.joint_name
        if joint_obj:
            joint_name = (
                joint_obj.linkforge_joint.joint_name
                if hasattr(joint_obj, "linkforge_joint") and joint_obj.linkforge_joint.joint_name
                else ""
            ) or joint_obj.name

            joints.append(
                TransmissionJoint(
                    name=joint_name,
                    hardware_interfaces=[hw_if],
                    mechanical_reduction=props.mechanical_reduction,
                    offset=props.offset,
                )
            )

            act_name = (
                props.actuator_name
                if props.use_custom_actuator_name and props.actuator_name
                else f"{joint_name}_motor"
            )
            actuators.append(TransmissionActuator(name=act_name, hardware_interfaces=[hw_if]))
    elif props.transmission_type == "DIFFERENTIAL":
        j1_obj = props.joint1_name
        j2_obj = props.joint2_name
        if j1_obj and j2_obj:
            j1_name = (
                j1_obj.linkforge_joint.joint_name if hasattr(j1_obj, "linkforge_joint") else ""
            ) or j1_obj.name
            j2_name = (
                j2_obj.linkforge_joint.joint_name if hasattr(j2_obj, "linkforge_joint") else ""
            ) or j2_obj.name

            joints.append(
                TransmissionJoint(
                    name=j1_name,
                    hardware_interfaces=[hw_if],
                    mechanical_reduction=props.mechanical_reduction,
                )
            )
            joints.append(
                TransmissionJoint(
                    name=j2_name,
                    hardware_interfaces=[hw_if],
                    mechanical_reduction=props.mechanical_reduction,
                )
            )

            a1_name = props.actuator1_name if props.actuator1_name else f"{j1_name}_motor"
            a2_name = props.actuator2_name if props.actuator2_name else f"{j2_name}_motor"

            actuators.append(TransmissionActuator(name=a1_name, hardware_interfaces=[hw_if]))
            actuators.append(TransmissionActuator(name=a2_name, hardware_interfaces=[hw_if]))

    if not joints:
        return None

    return Transmission(name=trans_name, type=trans_type, joints=joints, actuators=actuators)


def _categorize_scene_objects(
    scene: Any,
) -> tuple[
    dict[str, Any],
    list[Any],
    list[Any],
    list[Any],
    dict[str, tuple[str, Any]],
    tuple[str, Any] | None,
]:
    """Extract and categorize objects from Blender scene.

    Args:
        scene: Blender scene object

    Returns:
        Tuple of (link_objects, joint_objects, sensor_objects,
                 joints_map, root_link)
    """
    link_objects = {}  # link_name -> link Empty object
    joint_objects = []
    sensor_objects = []
    transmission_objects = []
    joints_map = {}  # child_link_name -> (parent_link_name, joint_empty_obj)
    root_link = None

    for obj in scene.objects:
        # Check for Link
        if hasattr(obj, "linkforge") and obj.linkforge.is_robot_link:
            link_name = obj.linkforge.link_name if obj.linkforge.link_name else obj.name
            link_objects[link_name] = obj

        # Check for Joint
        elif hasattr(obj, "linkforge_joint") and obj.linkforge_joint.is_robot_joint:
            joint_objects.append(obj)
            props = obj.linkforge_joint
            parent_obj = props.parent_link
            child_obj = props.child_link

            parent_name = (
                parent_obj.linkforge.link_name
                if parent_obj and hasattr(parent_obj, "linkforge")
                else (parent_obj.name if parent_obj else "")
            )
            child_name = (
                child_obj.linkforge.link_name
                if child_obj and hasattr(child_obj, "linkforge")
                else (child_obj.name if child_obj else "")
            )

            if parent_name and child_name:
                joints_map[child_name] = (parent_name, obj)

        # Check for Sensor
        elif hasattr(obj, "linkforge_sensor") and obj.linkforge_sensor.is_robot_sensor:
            sensor_objects.append(obj)

        # Check for Transmission
        elif (
            hasattr(obj, "linkforge_transmission")
            and obj.linkforge_transmission.is_robot_transmission
        ):
            transmission_objects.append(obj)

    # Find root link (link with no parent joint)
    for link_name, obj in link_objects.items():
        if link_name not in joints_map:
            root_link = (link_name, obj)
            break

    return link_objects, joint_objects, sensor_objects, transmission_objects, joints_map, root_link


def _calculate_link_frames(
    link_objects: dict[str, Any],
    joints_map: dict[str, tuple[str, Any]],
    root_link: tuple[str, Any] | None,
) -> dict[str, Any]:
    """Calculate coordinate frames for all links in the kinematic tree.

    Args:
        link_objects: Dictionary of link names to Blender objects
        joints_map: Mapping of child links to (parent, joint_object) tuples
        root_link: Tuple of (root_link_name, root_link_object)

    Returns:
        Dictionary mapping link names to their world transformation matrices
    """
    link_frames = {}  # link_name -> world matrix where link frame is

    if root_link and Matrix:
        root_name, root_obj = root_link
        link_frames[root_name] = Matrix.Identity(4)

        root_world = root_obj.matrix_world.copy()
        root_translation = root_world.to_translation()
        root_rotation = root_world.to_quaternion()
        root_transform = Matrix.Translation(root_translation) @ root_rotation.to_matrix().to_4x4()
        root_world_transform_inv = root_transform.inverted()

        def calc_child_frames(parent_name: str) -> None:
            """Recursively calculate child link coordinate frames."""
            for child_name, (parent, _joint_obj) in joints_map.items():
                if parent == parent_name and child_name not in link_frames:
                    child_obj = link_objects.get(child_name)
                    if child_obj:
                        child_world = child_obj.matrix_world.copy()
                        child_translation = child_world.to_translation()
                        child_rotation = child_world.to_quaternion()
                        child_transform = (
                            Matrix.Translation(child_translation)
                            @ child_rotation.to_matrix().to_4x4()
                        )
                        child_frame = root_world_transform_inv @ child_transform
                        link_frames[child_name] = child_frame
                        calc_child_frames(child_name)

        calc_child_frames(root_name)

    return link_frames


def scene_to_robot(
    context: Any,
    meshes_dir: Path | None = None,
    dry_run: bool = False,
) -> tuple[Robot, list[str]]:
    """Convert entire Blender scene to Core Robot.

    This function orchestrates the conversion process by:
    1. Categorizing scene objects (links, joints, sensors, transmissions)
    2. Calculating link coordinate frames
    3. Converting each object type to core models
    4. Assembling the complete Robot model

    Args:
        context: Blender context
        meshes_dir: Optional directory for exporting mesh files
        dry_run: If True, don't write mesh files

    Returns:
        Tuple of (Core Robot model, list of error messages)

    Note:
        Error handling behavior is controlled by the robot's strict_mode property:
        - strict_mode=False (default): Collects all errors and shows them together
        - strict_mode=True: Fails immediately on first error (useful for debugging)
    """
    if context is None:
        return Robot(name="empty_robot"), []

    scene = context.scene
    robot_props = scene.linkforge
    robot_name = robot_props.robot_name if robot_props.robot_name else "robot"
    strict_mode = robot_props.strict_mode  # Get strict mode from properties
    robot = Robot(name=robot_name)
    # Get evaluated depsgraph once for the entire conversion
    # This ensures all objects are evaluated at the same point in time
    # and significantly improves performance for complex robots.
    depsgraph = context.evaluated_depsgraph_get()
    conversion_errors: list[str] = []

    # Categorize scene objects
    link_objects, joint_objects, sensor_objects, transmission_objects, joints_map, root_link = (
        _categorize_scene_objects(scene)
    )

    # Calculate link coordinate frames
    link_frames = _calculate_link_frames(link_objects, joints_map, root_link)

    # Process Links
    for _link_name, obj in link_objects.items():
        try:
            # Create link
            link = blender_link_to_core_with_origin(
                obj, meshes_dir, robot_props, dry_run=dry_run, depsgraph=depsgraph
            )
            if link:
                robot.add_link(link)
        except Exception as e:
            if strict_mode:
                raise  # Fail immediately in strict mode
            conversion_errors.append(f"Link '{obj.name}': {e}")

    # Process Joints
    for obj in joint_objects:
        try:
            joint = blender_joint_to_core(obj)
            if joint:
                # Calculate joint origin relative to parent link frame
                # IMPORTANT: Joint origin should represent where the CHILD LINK's frame is,
                # not where the joint Empty object is positioned in Blender
                # This ensures export always reflects actual Blender scene state
                if (
                    (parent_name := joint.parent)
                    and parent_name in link_frames
                    and joint.child in link_frames
                    and Matrix
                ):
                    parent_frame = link_frames[parent_name]
                    child_frame = link_frames[joint.child]
                    parent_frame_inv = parent_frame.inverted()
                    # Use child link's frame position as joint origin
                    joint_relative = parent_frame_inv @ child_frame
                    corrected_origin = matrix_to_transform(joint_relative)
                    # Create new joint with corrected origin (Joint is frozen dataclass)
                    joint = replace(joint, origin=corrected_origin)

                robot.add_joint(joint)
        except Exception as e:
            if strict_mode:
                raise  # Fail immediately in strict mode
            conversion_errors.append(f"Joint '{obj.name}': {e}")

    # Process Sensors
    for obj in sensor_objects:
        try:
            sensor = blender_sensor_to_core(obj)
            if sensor and (link_name := sensor.link_name) and link_name in link_frames and Matrix:
                link_obj = link_objects.get(link_name)
                if link_obj and obj.parent == link_obj:
                    # Extract relative origin using matrix math (robust against 'Keep Transform')
                    sensor_relative = link_obj.matrix_world.inverted() @ obj.matrix_world
                    corrected_origin = matrix_to_transform(sensor_relative)
                    sensor = replace(sensor, origin=corrected_origin)
                else:
                    # Not direct child, but link_name is specified (custom mount)
                    link_frame_inv = link_frames[link_name].inverted()
                    sensor_relative = link_frame_inv @ obj.matrix_world
                    corrected_origin = matrix_to_transform(sensor_relative)
                    sensor = replace(sensor, origin=corrected_origin)

                robot.add_sensor(sensor)
        except Exception as e:
            if strict_mode:
                raise  # Fail immediately in strict mode
            conversion_errors.append(f"Sensor '{obj.name}': {e}")

    # Process Transmissions
    for obj in transmission_objects:
        try:
            transmission = blender_transmission_to_core(obj)
            if transmission:
                robot.add_transmission(transmission)
        except Exception as e:
            if strict_mode:
                raise
            conversion_errors.append(f"Transmission '{obj.name}': {e}")

    # Process centralized ROS2 Control
    if robot_props.use_ros2_control:
        try:
            ros2_control = blender_ros2_control_to_core(robot_props)
            if ros2_control:
                robot.add_ros2_control(ros2_control)

                # Add Gazebo ros2_control plugin if configured (ONLY if we have valid control config)
                if robot_props.gazebo_plugin_name:
                    params = {}
                    if robot_props.controllers_yaml_path:
                        params["parameters"] = robot_props.controllers_yaml_path

                    # Map UI string directly. Conventionally users input the exact plugin tag content,
                    # e.g., gz_ros2_control::GazeboSimROS2ControlPlugin, or libgazebo_ros2_control.so.
                    gazebo_plugin = GazeboPlugin(
                        name="gazebo_ros2_control",
                        filename=robot_props.gazebo_plugin_name,
                        parameters=params,
                    )
                    # Note: We wrap the plugin in a GazeboElement without a reference (global)
                    from ...linkforge_core.models.gazebo import GazeboElement

                    robot.add_gazebo_element(GazeboElement(plugins=[gazebo_plugin]))
        except Exception as e:
            if strict_mode:
                raise
            conversion_errors.append(f"ROS2 Control System: {e}")

    # If there were any conversion errors (only reached if strict_mode=False)
    if conversion_errors:
        error_summary = "\n".join(f"  - {err}" for err in conversion_errors)
        # In non-strict mode, always raise with all collected errors
        raise RobotValidationError(
            "RobotConversion", robot_name, f"Multiple configuration errors found:\n{error_summary}"
        )

    return robot, conversion_errors


def blender_sensor_to_core(obj: Any) -> Sensor | None:
    """Convert a Blender sensor Empty and its properties to a Core Sensor model.

    This function extracts sensor-specific configuration (Lidar, Camera, IMU)
    from Blender custom properties and maps them to the structured LinkForge
    core models for export.

    Args:
        obj: The Blender Empty object representing the sensor.

    Returns:
        A Core Sensor model if successful, or None if the object is invalid.

    """
    if obj is None:
        return None

    props = obj.linkforge_sensor
    if not props.is_robot_sensor:
        return None

    sensor_name = props.sensor_name if props.sensor_name else obj.name
    sensor_type = SensorType(props.sensor_type.lower())
    link_obj = props.attached_link
    link_name = link_obj.linkforge.link_name if link_obj else ""

    if not link_name:
        raise RobotValidationError(
            "SensorAttachment",
            sensor_name,
            "Sensor is not attached to any link. Please select a parent link.",
        )

    # Build sensor origin from object transform
    origin = matrix_to_transform(obj.matrix_world)

    # Type-specific info
    camera_info = None
    lidar_info = None
    imu_info = None
    gps_info = None
    contact_info = None
    force_torque_info = None

    # Noise model
    noise = None
    if props.use_noise:
        noise = SensorNoise(
            type=props.noise_type,
            mean=props.noise_mean,
            stddev=props.noise_stddev,
        )

    # Camera info
    if sensor_type in (SensorType.CAMERA, SensorType.DEPTH_CAMERA):
        camera_info = CameraInfo(
            horizontal_fov=props.camera_horizontal_fov,
            width=props.camera_width,
            height=props.camera_height,
            format=props.camera_format,
            near_clip=props.camera_near_clip,
            far_clip=props.camera_far_clip,
            noise=noise,
        )

    # LIDAR info
    elif sensor_type == SensorType.LIDAR:
        lidar_info = LidarInfo(
            horizontal_samples=props.lidar_horizontal_samples,
            horizontal_min_angle=props.lidar_horizontal_min_angle,
            horizontal_max_angle=props.lidar_horizontal_max_angle,
            vertical_samples=props.lidar_vertical_samples,
            range_min=props.lidar_range_min,
            range_max=props.lidar_range_max,
            noise=noise,
        )

    # IMU info
    elif sensor_type == SensorType.IMU:
        imu_info = IMUInfo(
            angular_velocity_noise=noise,
            linear_acceleration_noise=noise,
        )

    # GPS info
    elif sensor_type == SensorType.GPS:
        gps_info = GPSInfo(
            position_sensing_horizontal_noise=noise,
            velocity_sensing_horizontal_noise=noise,
        )

    # Contact info
    elif sensor_type == SensorType.CONTACT:
        collision_name = props.contact_collision
        if not collision_name:
            # Fallback: try to guess standard name
            collision_name = f"{link_name}_collision"
        contact_info = ContactInfo(collision=collision_name, noise=noise)

    # Force/Torque info
    elif sensor_type == SensorType.FORCE_TORQUE:
        force_torque_info = ForceTorqueInfo(noise=noise)

    # Gazebo plugin
    plugin = None
    if props.use_gazebo_plugin and props.plugin_filename:
        plugin = GazeboPlugin(
            name=f"{sensor_name}_plugin",
            filename=props.plugin_filename,
        )

    # Topic name
    topic = props.topic_name if props.topic_name else None

    return Sensor(
        name=sensor_name,
        type=sensor_type,
        link_name=link_name,
        origin=origin,
        update_rate=props.update_rate,
        always_on=props.always_on,
        visualize=props.visualize,
        camera_info=camera_info,
        lidar_info=lidar_info,
        imu_info=imu_info,
        gps_info=gps_info,
        contact_info=contact_info,
        force_torque_info=force_torque_info,
        plugin=plugin,
        topic=topic,
    )


def blender_ros2_control_to_core(props: Any) -> Ros2Control | None:
    """Convert centralized Blender ros2_control properties to Core model.

    Args:
        props: RobotPropertyGroup containing ros2_control settings

    Returns:
        Core Ros2Control model or None
    """
    if not props or not props.ros2_control_name:
        return None

    joints: list[Ros2ControlJoint] = []
    for item in props.ros2_control_joints:
        cmd_ifs = []
        if item.cmd_position:
            cmd_ifs.append("position")
        if item.cmd_velocity:
            cmd_ifs.append("velocity")
        if item.cmd_effort:
            cmd_ifs.append("effort")

        state_ifs = []
        if item.state_position:
            state_ifs.append("position")
        if item.state_velocity:
            state_ifs.append("velocity")
        if item.state_effort:
            state_ifs.append("effort")

        # Intelligent defaults: if one side is empty but the other isn't,
        # apply 'position' as a sensible default to ensure validity.
        # NOTE: sensor hardware types cannot have command interfaces.
        if props.ros2_control_type == "sensor":
            if cmd_ifs:
                logger.warning(
                    f"ROS2 Control: Hardware type 'sensor' cannot have command interfaces. "
                    f"Stripping {cmd_ifs} from joint '{item.name}'."
                )
                cmd_ifs = []
            if not state_ifs:
                state_ifs.append("position")
        else:
            if state_ifs and not cmd_ifs:
                cmd_ifs.append("position")
            elif cmd_ifs and not state_ifs:
                state_ifs.append("position")

        # Extract joint-level parameters
        parameters = {p.name: p.value for p in item.parameters if p.name}

        # Determine the correct joint name
        joint_name = item.joint_obj.linkforge_joint.joint_name if item.joint_obj else item.name

        if cmd_ifs or state_ifs:
            joints.append(
                Ros2ControlJoint(
                    name=joint_name,
                    command_interfaces=cmd_ifs,
                    state_interfaces=state_ifs,
                    parameters=parameters,
                )
            )

    # ROS 2 Specification: 'actuator' types must have exactly one joint.
    # Handle gracefully by taking only the first if multiple are configured.
    if props.ros2_control_type == "actuator" and len(joints) > 1:
        logger.warning(
            f"ROS2 Control: Hardware type 'actuator' is limited to exactly one joint by ROS 2 "
            f"specification. Truncating {len(joints)} joints to only include '{joints[0].name}'."
        )
        joints = joints[:1]

    if not joints:
        return None

    return Ros2Control(
        name=props.ros2_control_name,
        type=props.ros2_control_type,
        hardware_plugin=props.hardware_plugin,
        joints=joints,
        parameters={p.name: p.value for p in props.ros2_control_parameters if p.name},
    )
