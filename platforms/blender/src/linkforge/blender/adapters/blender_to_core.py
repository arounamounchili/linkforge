"""Converters between Blender properties and Core models.

These functions bridge the gap between Blender's property system
and LinkForge's core data models.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from linkforge.core.constants import (
    DEFAULT_MATERIAL_RGBA,
    GEOM_BOX,
    GEOM_CYLINDER,
    GEOM_EPSILON,
    GEOM_MESH,
    GEOM_SPHERE,
)

from ..constants import (
    DEFAULT_PRIMITIVE_CONFIG,
    FORMAT_STL,
    GEOM_AUTO,
    PRIMITIVE_MAX_FACES,
    PURPOSE_VISUAL,
    SUFFIX_VISUAL,
    TAG_COLLISION_GEOM,
    TAG_SOURCE_GEOM,
)

if TYPE_CHECKING:
    from pathlib import Path

try:
    import numpy as np  # type: ignore[import-not-found]
except ImportError:
    np = None

import bpy
from linkforge.core import (
    Box,
    Color,
    Cylinder,
    GazeboElement,
    GazeboPlugin,
    Geometry,
    Material,
    Mesh,
    Robot,
    RobotBuilder,
    RobotValidationError,
    Sphere,
    Transform,
    ValidationErrorCode,
    ValidationResult,
    Vector3,
    get_logger,
)
from linkforge.core._utils.math_utils import clean_float
from linkforge.core._utils.string_utils import sanitize_name
from mathutils import Matrix

from ..utils.property_helpers import (
    get_joint_props,
    get_link_props,
    get_robot_props,
    get_sensor_props,
    get_transmission_props,
)
from .context import IBlenderContext
from .translator import (
    Ros2ControlTranslator,
    SensorTranslator,
    TransmissionTranslator,
)

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


def detect_primitive_type(obj: bpy.types.Object | None) -> str | None:
    """Detect if a Blender mesh object matches a standard primitive shape.

    Analyzes topology and dimensions to determine if the object can be
    exported as a URDF primitive (BOX, CYLINDER, or SPHERE). This function
    is critical for optimizing exports and ensuring compatibility with
    physics simulators.

    Args:
        obj: The Blender mesh object to analyze.

    Returns:
        "box", "cylinder", or "sphere" if a match is detected, else None.
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

    tags = [TAG_SOURCE_GEOM, TAG_COLLISION_GEOM]
    for tag in tags:
        tag_val = obj.get(tag)  # type: ignore[func-returns-value]
        if isinstance(tag_val, str):
            tag_val_lower = tag_val.lower()
            if tag_val_lower in (GEOM_SPHERE):
                return tag_val_lower
            if tag_val_lower == GEOM_MESH:
                return None

    # Count vertices and faces
    vert_count = len(mesh_obj.vertices)
    face_count = len(mesh_obj.polygons)

    if face_count > PRIMITIVE_MAX_FACES:
        return None

    # Get config for primitive detection thresholds
    config = DEFAULT_PRIMITIVE_CONFIG

    # Match Box: 8 vertices, 6 quad faces
    if vert_count == config.cube_vert_count and face_count == config.cube_face_count:
        # Verify it's roughly box-shaped by checking if all faces are quads
        all_quads = all(
            len(poly.vertices) == config.cube_verts_per_face for poly in mesh_obj.polygons
        )
        if all_quads:
            return GEOM_BOX

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
                return GEOM_SPHERE

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
                    return GEOM_CYLINDER

    # If none match, it's a complex mesh
    return None


def get_object_geometry(
    obj: bpy.types.Object | None,
    geometry_type: str = GEOM_AUTO,
    link_name: str | None = None,
    geom_purpose: str = PURPOSE_VISUAL,
    meshes_dir: Path | None = None,
    mesh_format: str = FORMAT_STL,
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
            - "auto": Auto-detect (primitives for simple shapes, mesh for complex)
            - "mesh": Force mesh export
            - "box", "cylinder", "sphere": Force specific primitive
        link_name: Name of the link (for mesh filename)
        geom_purpose: "visual" or "collision" (use PURPOSE_VISUAL, PURPOSE_COLLISION)
        meshes_dir: Directory to export mesh files to
        mesh_format: "STL", "OBJ", or "GLB" (use FORMAT_STL, etc.)
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
    if actual_geometry_type == GEOM_AUTO:
        detected_type = detect_primitive_type(obj)
        # Use detected primitive (cleaner URDF) or fallback to mesh for complex shapes
        actual_geometry_type = detected_type or GEOM_MESH

    if actual_geometry_type == GEOM_MESH:
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
        actual_geometry_type = GEOM_BOX

    # For primitives, the pose is just the current object matrix
    geom_world_matrix = obj.matrix_world

    if actual_geometry_type == GEOM_BOX:
        # Use bounding box dimensions
        dimensions = getattr(obj, "dimensions", None)
        if dimensions is None:
            return None, Matrix.Identity(4)

        # Robustness Check: Skip zero-size objects (e.g. empties from failed imports)
        if dimensions.length < GEOM_EPSILON:
            logger.warning(f"Skipping geometry for '{obj.name}': Dimensions are zero.")
            return None, Matrix.Identity(4)

        return Box(size=Vector3(dimensions.x, dimensions.y, dimensions.z)), geom_world_matrix

    elif actual_geometry_type == GEOM_CYLINDER:
        # Approximate with bounding cylinder
        dimensions = getattr(obj, "dimensions", None)
        if dimensions is None:
            return None, Matrix.Identity(4)

        radius = max(dimensions.x, dimensions.y) / 2.0
        length = dimensions.z
        return Cylinder(radius=radius, length=length), geom_world_matrix

    elif actual_geometry_type == GEOM_SPHERE:
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
        verts = np.zeros(num_verts * 3, dtype=np.float32)
        mesh_data.vertices.foreach_get("co", verts)
        vertices_array = verts.reshape((-1, 3))

        # Fast face index extraction (triangles)
        num_tris = len(mesh_data.loop_triangles)
        tris = np.zeros(num_tris * 3, dtype=np.int32)
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
        color = Color(*DEFAULT_MATERIAL_RGBA)

    return Material(name=mat_name, color=color)


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

    import bpy

    logger.debug(
        f"_categorize_scene_objects: scene.objects count={len(scene.objects)}, "
        f"data.objects count={len(bpy.data.objects)}"
    )
    for obj in scene.objects:
        # Check for Link
        lf = get_link_props(obj)
        if lf and getattr(lf, "is_robot_link", False):
            link_name = lf.link_name if lf.link_name else obj.name
            link_objects[link_name] = obj

        # Check for Joint
        j_lf = get_joint_props(obj)
        if j_lf and getattr(j_lf, "is_robot_joint", False):
            joint_objects.append(obj)
            props = j_lf
            parent_obj = props.parent_link
            child_obj = props.child_link

            parent_props = get_link_props(parent_obj)
            parent_name = (
                parent_props.link_name
                if parent_props and parent_props.link_name
                else (parent_obj.name if parent_obj else "")
            )
            child_props = get_link_props(child_obj)
            child_name = (
                child_props.link_name
                if child_props and child_props.link_name
                else (child_obj.name if child_obj else "")
            )

            if parent_name and child_name:
                joints_map[child_name] = (parent_name, obj)

        # Check for Sensor
        s_lf = get_sensor_props(obj)
        if s_lf and getattr(s_lf, "is_robot_sensor", False):
            sensor_objects.append(obj)

        # Check for Transmission
        t_lf = get_transmission_props(obj)
        if t_lf and getattr(t_lf, "is_robot_transmission", False):
            transmission_objects.append(obj)

    # Find root link (link with no parent joint)
    for link_name, obj in link_objects.items():
        if link_name not in joints_map:
            root_link = (link_name, obj)
            break

    logger.debug(
        f"_categorize_scene_objects: links={list(link_objects.keys())}, "
        f"joints={len(joint_objects)}, sensors={len(sensor_objects)}, "
        f"root={root_link[0] if root_link else 'None'}"
    )

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
        self.robot_props = get_robot_props(context.scene)
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
        self._translate_scene_gazebo_plugins()

        # 6. Finalize and return
        try:
            robot = self.builder.build()
        except Exception as e:
            self.validation_result.add_error(
                title="Build failed", message=str(e), code=ValidationErrorCode.INVALID_VALUE
            )
            robot = Robot(name=self.robot_name)

        if self.validation_result.errors:
            first_err = self.validation_result.errors[0]
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Multiple configuration errors found ({len(self.validation_result.errors)}). First: {first_err.title} - {first_err.message}",
            )

        return robot, self.validation_result

    def _translate_global_materials(self, link_objects: dict[str, Any]) -> None:
        """Collect and register all unique materials used in the robot."""
        processed_mats = set()
        for link_obj in link_objects.values():
            props = get_link_props(link_obj)
            if props and props.use_material:
                for child in link_obj.children:
                    if SUFFIX_VISUAL in child.name and child.type == "MESH":
                        mat = get_object_material(child, props)
                        if mat and mat.name not in processed_mats:
                            # Register material in the robot model to satisfy LinkBuilder validation
                            if mat.name not in self.builder.robot.materials:
                                self.builder.robot.materials[mat.name] = mat
                            # Register with builder
                            color_tuple = (
                                (mat.color.r, mat.color.g, mat.color.b, mat.color.a)
                                if mat.color
                                else DEFAULT_MATERIAL_RGBA
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
                joint_props = get_joint_props(joint_obj)
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
        """Translate transmissions using specialized TransmissionTranslator."""

        transmission_translator = TransmissionTranslator()
        for obj in transmission_objects:
            transmission_translator.translate(
                obj=obj,
                builder=self.builder,
                context=self.context,
                validation_result=self.validation_result,
            )

    def _translate_ros2_control(self) -> None:
        """Translate ROS2 Control settings from robot properties."""
        if self.robot_props and getattr(self.robot_props, "use_ros2_control", False):
            translator = Ros2ControlTranslator()
            translator.translate(
                obj=self.robot_props,
                builder=self.builder,
                context=self.context,
                validation_result=self.validation_result,
            )

    def _translate_scene_gazebo_plugins(self) -> None:
        """Translate scene-level Gazebo plugins (e.g. ros2_control or custom)."""
        if not self.robot_props:
            return

        plugin_filename = getattr(self.robot_props, "gazebo_plugin_name", "")
        if not plugin_filename:
            return

        params = {}
        is_standard_control = (
            "gz_ros2_control" in plugin_filename or "gazebo_ros2_control" in plugin_filename
        )

        # Add controllers YAML if ros2_control is active
        if getattr(self.robot_props, "use_ros2_control", False):
            # Special case for standard gz_ros2_control: only add if we actually have joints to control
            if is_standard_control and not self.builder.robot.ros2_controls:
                return

            yaml_path = getattr(self.robot_props, "controllers_yaml_path", "")
            if yaml_path:
                params["parameters"] = yaml_path
        elif is_standard_control:
            # If standard control is NOT used, don't add the plugin at all
            return

        # Determine plugin name
        # For standard gz_ros2_control, we use 'gazebo_ros2_control' for compatibility
        if "gz_ros2_control" in plugin_filename or "gazebo_ros2_control" in plugin_filename:
            name = "gazebo_ros2_control"
        else:
            # Custom plugin: use filename as name (matches test expectation)
            name = plugin_filename

        gazebo_plugin = GazeboPlugin(
            name=name,
            filename=plugin_filename,
            parameters=params,
        )

        self.builder.robot.add_gazebo_element(GazeboElement(plugins=[gazebo_plugin]))


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
