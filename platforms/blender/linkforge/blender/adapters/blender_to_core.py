"""Converters between Blender properties and Core models.

These functions bridge the gap between Blender's property system
and LinkForge's core data models.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from pathlib import Path

    from linkforge_core.composer import RobotBuilder
    from linkforge_core.models import Joint, Link, Robot
    from linkforge_core.validation.result import ValidationResult

try:
    import numpy as np  # type: ignore[import-not-found]
except ImportError:
    np = None

from dataclasses import dataclass

import bpy
from linkforge_core.composer import RobotBuilder
from linkforge_core.exceptions import RobotValidationError, ValidationErrorCode
from linkforge_core.logging_config import get_logger
from linkforge_core.models import (
    Box,
    CameraInfo,
    Color,
    ContactInfo,
    Cylinder,
    ForceTorqueInfo,
    GazeboPlugin,
    Geometry,
    GPSInfo,
    IMUInfo,
    LidarInfo,
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
)
from linkforge_core.models.transmission import (
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    TransmissionType,
)
from linkforge_core.utils.math_utils import clean_float
from linkforge_core.utils.string_utils import sanitize_name
from linkforge_core.validation.result import ValidationResult
from mathutils import Matrix

from .context import IBlenderContext
from .translator import SensorTranslator

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
    # Type-narrowing for Mypy, with resilience for mocked test environments
    is_mesh = isinstance(mesh, bpy.types.Mesh)
    if not is_mesh and obj.type == "MESH" and mesh is not None:
        # Fallback for mocked environments where isinstance might fail
        is_mesh = hasattr(mesh, "vertices") and hasattr(mesh, "polygons")

    if not is_mesh or mesh is None:
        return None

    # Narrow type for Mypy
    from typing import cast

    mesh_obj = cast(bpy.types.Mesh, mesh)

    tags = ["source_geometry_type", "collision_geometry_type"]
    for tag in tags:
        tag_val = obj.get(tag)  # type: ignore[func-returns-value]
        if isinstance(tag_val, str):
            if tag_val in ("BOX", "CYLINDER", "SPHERE"):
                return tag_val
            if tag_val == "MESH":
                return None

    # Count vertices and faces
    vert_count = len(mesh_obj.vertices)
    face_count = len(mesh_obj.polygons)

    # Get config for primitive detection thresholds
    config = DEFAULT_PRIMITIVE_CONFIG

    # Match Box: 8 vertices, 6 quad faces
    if vert_count == config.cube_vert_count and face_count == config.cube_face_count:
        # Verify it's roughly box-shaped by checking if all faces are quads
        all_quads = all(
            len(poly.vertices) == config.cube_verts_per_face for poly in mesh_obj.polygons
        )
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
    obj: bpy.types.Object | None,
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
        dimensions = getattr(obj, "dimensions", None)
        if dimensions is None:
            return None, Matrix.Identity(4)

        # Robustness Check: Skip zero-size objects (e.g. empties from failed imports)
        if dimensions.length < 1e-6:
            logger.warning(f"Skipping geometry for '{obj.name}': Dimensions are zero.")
            return None, Matrix.Identity(4)

        return Box(size=Vector3(dimensions.x, dimensions.y, dimensions.z)), geom_world_matrix

    elif actual_geometry_type == "CYLINDER":
        # Approximate with bounding cylinder
        dimensions = getattr(obj, "dimensions", None)
        if dimensions is None:
            return None, Matrix.Identity(4)

        radius = max(dimensions.x, dimensions.y) / 2.0
        length = dimensions.z
        return Cylinder(radius=radius, length=length), geom_world_matrix

    elif actual_geometry_type == "SPHERE":
        # Approximate with bounding sphere
        dimensions = getattr(obj, "dimensions", None)
        if dimensions is None:
            return None, Matrix.Identity(4)

        radius = max(dimensions) / 2.0
        return Sphere(radius=radius), geom_world_matrix

    return None, Matrix.Identity(4)


def extract_mesh_triangles(
    obj: bpy.types.Object | None,
    depsgraph: Any | None = None,
    as_numpy: bool = False,
) -> tuple[Any, Any] | None:
    """Extract triangle mesh data from Blender object.

    Args:
        obj: Blender mesh object
        depsgraph: Optional evaluated dependency graph
        as_numpy: If True, return NumPy arrays instead of Python lists

    Returns:
        Tuple of (vertices, triangles) or None if not a mesh:
            - vertices: List of (x, y, z) coordinates or (N, 3) NumPy array
            - triangles: List of (v0, v1, v2) vertex indices or (M, 3) NumPy array
    """
    if obj is None or obj.type != "MESH":
        return None

    # Get evaluated mesh (with modifiers applied)
    if depsgraph is None:
        depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    mesh_data = eval_obj.to_mesh()

    if mesh_data is None:
        return None

    # Ensure mesh has triangulated faces
    mesh_data.calc_loop_triangles()

    if mesh_data.loop_triangles is None:
        eval_obj.to_mesh_clear()
        return None

    # We use the scale matrix (not full world matrix) to get correct dimensions
    # but keep the object centered at its local origin for proper inertia calculation
    # The inertia tensor is always computed relative to the object's center of mass
    scale_matrix = obj.matrix_world.to_scale()

    # Fast O(N) extraction via NumPy
    if np is not None:
        # Fast vertex extraction via foreach_get
        num_verts = len(mesh_data.vertices)
        verts = np.empty(num_verts * 3, dtype=np.float32)
        mesh_data.vertices.foreach_get("co", verts)
        vertices_array = verts.reshape((-1, 3))

        # Fast face index extraction (triangles)
        num_tris = len(mesh_data.loop_triangles)
        tris = np.empty(num_tris * 3, dtype=np.int32)
        mesh_data.loop_triangles.foreach_get("vertices", tris)
        triangles_array = tris.reshape((-1, 3))

        # Apply scale
        vertices_array[:, 0] *= scale_matrix.x
        vertices_array[:, 1] *= scale_matrix.y
        vertices_array[:, 2] *= scale_matrix.z

        # Optional: Return arrays directly
        if as_numpy:
            eval_obj.to_mesh_clear()
            return vertices_array, triangles_array

        vertices_list = vertices_array.tolist()
        triangles_list = triangles_array.tolist()

        eval_obj.to_mesh_clear()
        return vertices_list, triangles_list

    # Python fallback
    vertices = [
        (v.co.x * scale_matrix.x, v.co.y * scale_matrix.y, v.co.z * scale_matrix.z)
        for v in mesh_data.vertices
    ]
    triangles = [tuple(t.vertices) for t in mesh_data.loop_triangles]

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
                    base_color_input = node.inputs.get("Base Color")
                    if base_color_input and hasattr(base_color_input, "default_value"):
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


def blender_transmission_to_core(obj: Any) -> Transmission | None:
    """Convert Blender Empty with TransmissionPropertyGroup to Core Transmission.

    Args:
        obj: Blender Empty object with linkforge_transmission property group

    Returns:
        Core Transmission model or None

    """
    if obj is None:
        return None

    props = getattr(obj, "linkforge_transmission", None)
    if not props or not getattr(props, "is_robot_transmission", False):
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
            joint_props = getattr(joint_obj, "linkforge_joint", None)
            joint_name = (
                joint_props.joint_name
                if joint_props and getattr(joint_props, "joint_name", "")
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
            j1_props = getattr(j1_obj, "linkforge_joint", None)
            j1_name = (
                j1_props.joint_name if j1_props and getattr(j1_props, "joint_name", "") else ""
            ) or j1_obj.name
            j2_props = getattr(j2_obj, "linkforge_joint", None)
            j2_name = (
                j2_props.joint_name if j2_props and getattr(j2_props, "joint_name", "") else ""
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
        lf = getattr(obj, "linkforge", None)
        if lf and getattr(lf, "is_robot_link", False):
            link_name = lf.link_name if lf.link_name else obj.name
            link_objects[link_name] = obj

        # Check for Joint
        j_lf = getattr(obj, "linkforge_joint", None)
        if j_lf and getattr(j_lf, "is_robot_joint", False):
            joint_objects.append(obj)
            props = j_lf
            parent_obj = props.parent_link
            child_obj = props.child_link

            parent_props = getattr(parent_obj, "linkforge", None)
            parent_name = (
                parent_props.link_name
                if parent_props and getattr(parent_props, "link_name", "")
                else (parent_obj.name if parent_obj else "")
            )
            child_props = getattr(child_obj, "linkforge", None)
            child_name = (
                child_props.link_name
                if child_props and getattr(child_props, "link_name", "")
                else (child_obj.name if child_obj else "")
            )

            if parent_name and child_name:
                joints_map[child_name] = (parent_name, obj)

        # Check for Sensor
        s_lf = getattr(obj, "linkforge_sensor", None)
        if s_lf and getattr(s_lf, "is_robot_sensor", False):
            sensor_objects.append(obj)

        # Check for Transmission
        t_lf = getattr(obj, "linkforge_transmission", None)
        if t_lf and getattr(t_lf, "is_robot_transmission", False):
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

    if root_link is not None and Matrix is not None:
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


class SceneToRobotTranslator:
    """Orchestrates the conversion of a Blender scene to a Core Robot model.

    This class follows the SOLID principles by encapsulating the translation logic
    and leveraging the RobotBuilder (Composer) API for structural integrity.
    """

    def __init__(
        self,
        context: IBlenderContext,
        meshes_dir: Path | None = None,
        dry_run: bool = False,
        depsgraph: Any | None = None,
    ):
        self.context = context
        self.meshes_dir = meshes_dir
        self.dry_run = dry_run
        self.depsgraph = depsgraph

        # Get robot properties from scene
        self.robot_props = getattr(context.scene, "linkforge", None)
        if not self.robot_props:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND, "Scene has no LinkForge properties"
            )

        self.robot_name = self.robot_props.robot_name if self.robot_props.robot_name else "robot"
        self.builder = RobotBuilder(self.robot_name)
        self.validation_result = ValidationResult(robot_name=self.robot_name)

    def translate(self) -> tuple[Robot, ValidationResult]:
        """Perform the translation and return the built Robot model."""
        # 1. Categorize scene objects
        link_objects, joint_objects, sensor_objects, transmission_objects, joints_map, root = (
            _categorize_scene_objects(self.context.scene)
        )

        # 2. Calculate coordinate frames (needed for joint relative origins)
        link_frames = _calculate_link_frames(link_objects, joints_map, root)

        # 3. Translate Materials globally (Centralized management)
        self._translate_global_materials(link_objects)

        # 4. Build Kinematic Tree recursively (The "Composer" way)
        if root:
            root_name, _ = root
            self._build_link_recursive(root_name, None, link_objects, joints_map, link_frames)
        else:
            self.validation_result.add_error(
                title="No root link",
                message="No root link found in scene. Ensure at least one link has no parent joint.",
                code=ValidationErrorCode.NO_ROOT,
            )

        # 5. Translate orphaned components (Sensors, Transmissions)
        self._translate_sensors(sensor_objects, link_frames, link_objects)
        self._translate_transmissions(transmission_objects)
        self._translate_ros2_control()

        # 6. Finalize and return
        try:
            robot = self.builder.build()
        except Exception as e:
            self.validation_result.add_error(
                title="Build failed", message=str(e), code=ValidationErrorCode.INVALID_VALUE
            )
            robot = Robot(name=self.robot_name)

        if self.validation_result.errors:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Multiple configuration errors found ({len(self.validation_result.errors)})",
            )

        return robot, self.validation_result

    def _translate_global_materials(self, link_objects: dict[str, Any]) -> None:
        """Collect and register all unique materials used in the robot."""
        processed_mats = set()
        for link_obj in link_objects.values():
            props = getattr(link_obj, "linkforge", None)
            if props and props.use_material:
                for child in link_obj.children:
                    if "_visual" in child.name and child.type == "MESH":
                        mat = get_object_material(child, props)
                        if mat and mat.name not in processed_mats:
                            # Register material in the robot model to satisfy LinkBuilder validation
                            if mat.name not in self.builder.robot.materials:
                                self.builder.robot.materials[mat.name] = mat
                            # Register with builder
                            color_tuple = (
                                (mat.color.r, mat.color.g, mat.color.b, mat.color.a)
                                if mat.color
                                else (0.8, 0.8, 0.8, 1.0)
                            )
                            self.builder.material(mat.name, color=color_tuple)
                            processed_mats.add(mat.name)

    def _build_link_recursive(
        self,
        link_name: str,
        parent_lb: Any,
        link_objects: dict[str, Any],
        joints_map: dict[str, tuple[str, Any]],
        link_frames: dict[str, Any],
    ) -> None:
        """Recursively build links and joints using specialized translators."""
        if link_name not in link_objects:
            return

        obj = link_objects[link_name]

        try:
            # 1. Start link in composer
            from .translator import JointTranslator, LinkTranslator

            if parent_lb is None:
                lb = self.builder.link(link_name)
            else:
                joint_info = joints_map.get(link_name)
                if not joint_info:
                    return
                _parent_name, joint_obj = joint_info
                joint_props = getattr(joint_obj, "linkforge_joint", None)
                joint_name = joint_props.joint_name if joint_props else joint_obj.name
                lb = parent_lb.child(link_name, joint_name=joint_name)

                # Configure Joint
                joint_translator = JointTranslator()
                joint_translator.translate(
                    obj=joint_obj,
                    builder=self.builder,
                    context=self.context,
                    validation_result=self.validation_result,
                    lb=lb,
                    link_frames=link_frames,
                )

            # 2. Configure Link
            link_translator = LinkTranslator()
            link_translator.translate(
                obj=obj,
                builder=self.builder,
                context=self.context,
                meshes_dir=self.meshes_dir,
                dry_run=self.dry_run,
                depsgraph=self.depsgraph,
                validation_result=self.validation_result,
                lb=lb,
            )

            # 3. Recurse to children
            for child_name, (p_name, _j_obj) in joints_map.items():
                if p_name == link_name:
                    self._build_link_recursive(
                        child_name, lb, link_objects, joints_map, link_frames
                    )

            # 4. Commit link
            lb.commit()

        except Exception as e:
            if self.robot_props and getattr(self.robot_props, "strict_mode", False):
                raise
            self.validation_result.add_error(
                title=f"Link translation failed: {link_name}",
                message=str(e),
                code=ValidationErrorCode.INVALID_VALUE,
                affected_objects=[link_name],
            )

    def _translate_sensors(
        self, sensor_objects: list[Any], link_frames: dict[str, Any], _link_objects: dict[str, Any]
    ) -> None:
        """Translate sensors using specialized SensorTranslator."""

        sensor_translator = SensorTranslator()
        for obj in sensor_objects:
            sensor_translator.translate(
                obj=obj,
                builder=self.builder,
                context=self.context,
                validation_result=self.validation_result,
                link_frames=link_frames,
            )

    def _translate_transmissions(self, transmission_objects: list[Any]) -> None:
        for obj in transmission_objects:
            try:
                transmission = blender_transmission_to_core(obj)
                if transmission:
                    self.builder.robot.add_transmission(transmission)
            except Exception as e:
                self.validation_result.add_error(
                    title=f"Transmission translation failed: {obj.name}",
                    message=str(e),
                    code=ValidationErrorCode.INVALID_VALUE,
                    affected_objects=[obj.name],
                )

    def _translate_ros2_control(self) -> None:
        if self.robot_props and getattr(self.robot_props, "use_ros2_control", False):
            try:
                ros2_control = blender_ros2_control_to_core(self.robot_props)
                if ros2_control:
                    self.builder.robot.add_ros2_control(ros2_control)

                    # Gazebo plugin
                    if getattr(self.robot_props, "gazebo_plugin_name", ""):
                        params = {}
                        if getattr(self.robot_props, "controllers_yaml_path", ""):
                            params["parameters"] = self.robot_props.controllers_yaml_path

                        gazebo_plugin = GazeboPlugin(
                            name="gazebo_ros2_control",
                            filename=self.robot_props.gazebo_plugin_name,
                            parameters=params,
                        )
                        from linkforge_core.models.gazebo import GazeboElement

                        self.builder.robot.add_gazebo_element(
                            GazeboElement(plugins=[gazebo_plugin])
                        )
            except Exception as e:
                self.validation_result.add_error(
                    title="ROS2 Control translation failed",
                    message=str(e),
                    code=ValidationErrorCode.INVALID_VALUE,
                )

    def _get_geom_suffix(self, child: Any, parent_obj: Any, type_tag: str) -> str:
        visual_count = sum(1 for c in parent_obj.children if type_tag in c.name)
        source_name = child.get("source_name", None)
        if source_name:
            return f"_{sanitize_name(source_name)}"
        elif visual_count > 1:
            # Find index
            idx = [c for c in parent_obj.children if type_tag in c.name].index(child)
            return f"_{idx}"
        return ""


def scene_to_robot(
    context: IBlenderContext | bpy.types.Context,
    meshes_dir: Path | None = None,
    dry_run: bool = False,
) -> tuple[Robot, ValidationResult]:
    """Convert entire Blender scene to Core Robot using the Translator orchestrator."""
    from .context import BlenderContext

    # Auto-wrap for legacy compatibility
    if not isinstance(context, IBlenderContext):
        import bpy

        context = BlenderContext(bpy)

    translator = SceneToRobotTranslator(context, meshes_dir, dry_run)
    return translator.translate()


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
    props = getattr(obj, "linkforge_sensor", None)
    if not props or not props.is_robot_sensor:
        return None

    sensor_name = props.sensor_name if props.sensor_name else obj.name
    sensor_type = SensorType(props.sensor_type.lower())
    link_obj = props.attached_link
    link_props = getattr(link_obj, "linkforge", None)
    link_name = (
        (link_props.link_name if link_props and link_props.link_name else link_obj.name)
        if link_obj
        else ""
    )

    if not link_name:
        raise RobotValidationError(
            ValidationErrorCode.NOT_FOUND,
            "Sensor is not attached to any link. Please select a parent link.",
            target="SensorAttachment",
            value=sensor_name,
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
        joint_obj = getattr(item, "joint_obj", None)
        joint_props = getattr(joint_obj, "linkforge_joint", None)
        joint_name = joint_props.joint_name if joint_props else item.name

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


# --- Legacy Wrappers for Test Compatibility ---
# TODO: Refactor tests to use Translator architecture and remove these.


def blender_link_to_core_with_origin(obj: Any, **kwargs: Any) -> Link | None:
    """Legacy wrapper for Link conversion. Use LinkTranslator instead."""
    import bpy
    from linkforge_core.composer import RobotBuilder

    from .context import BlenderContext
    from .translator import LinkTranslator

    if obj is None:
        return None
    context = kwargs.get("context") or BlenderContext(bpy)
    builder = RobotBuilder("temp")
    lb = LinkTranslator().translate(obj, builder, context, **kwargs)
    if lb:
        lb.commit()
    props = getattr(obj, "linkforge", None)
    link_name = props.link_name if props and props.link_name else obj.name
    return builder.robot.get_link(link_name)


def blender_joint_to_core(obj: Any) -> Joint | None:
    """Legacy wrapper for Joint conversion. Use JointTranslator instead."""
    import bpy
    from linkforge_core.composer import RobotBuilder

    from .context import BlenderContext
    from .translator import JointTranslator

    context = BlenderContext(bpy)
    builder = RobotBuilder("temp")

    if obj is None:
        return None
    props = getattr(obj, "linkforge_joint", None)
    if not props or not getattr(props, "is_robot_joint", False):
        return None
    if not props.child_link:
        raise RobotValidationError(
            ValidationErrorCode.NOT_FOUND,
            "Joint has no child link. Please select a child link in the Joint properties.",
            target="JointBuilder",
        )

    # We need the child link to be in the builder to create the joint
    child_obj = props.child_link
    child_props = getattr(child_obj, "linkforge", None)
    child_name = child_props.link_name if child_props and child_props.link_name else child_obj.name

    # Also need the parent link
    parent_obj = props.parent_link
    if not parent_obj:
        raise RobotValidationError(
            ValidationErrorCode.NOT_FOUND,
            "Joint has no parent link. Please select a parent link in the Joint properties.",
            target="JointBuilder",
        )

    parent_props = getattr(parent_obj, "linkforge", None)
    parent_name = (
        parent_props.link_name if parent_props and parent_props.link_name else parent_obj.name
    )
    builder.link(parent_name).root()  # Add as root for now

    lb = builder.link(child_name, parent=parent_name)
    JointTranslator().translate(obj, builder, context, lb=lb)
    lb.commit()

    joint_name = props.joint_name if props and props.joint_name else obj.name
    return builder.robot.get_joint(joint_name)
