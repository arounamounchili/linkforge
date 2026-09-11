"""Geometry and material extraction from Blender objects to LinkForge core models.

This module provides focused utilities to extract primitive shapes, triangle mesh
data, and materials from Blender objects without coupling to scene translation.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, cast

try:
    import numpy as np  # type: ignore[import-not-found]
except ImportError:
    np = None

import bpy
from mathutils import Matrix

from ..constants import (
    DEFAULT_PRIMITIVE_CONFIG,
    FORMAT_STL,
    PRIMITIVE_MAX_FACES,
    PURPOSE_VISUAL,
)
from ..core import (
    Box,
    Color,
    Cylinder,
    Geometry,
    Material,
    Mesh,
    Sphere,
    Vector3,
    get_logger,
)
from ..core._utils.string_utils import sanitize_name
from ..core.constants import (
    DEFAULT_MATERIAL_RGBA,
    GEOM_BOX,
    GEOM_CYLINDER,
    GEOM_EPSILON,
    GEOM_MESH,
    GEOM_SPHERE,
)
from ..properties.geom_props import PROP_GEOM
from ..utils.transform_utils import get_local_bounding_box_center

logger = get_logger(__name__)


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

    geom_props = getattr(obj, PROP_GEOM, None)
    if geom_props:
        gt = getattr(geom_props, "geometry_type", None)
        if gt in (GEOM_BOX, GEOM_CYLINDER, GEOM_SPHERE):
            return cast(str, gt)

    mesh = obj.data
    is_mesh = isinstance(mesh, bpy.types.Mesh)
    if not is_mesh and obj.type == "MESH" and mesh is not None:
        is_mesh = hasattr(mesh, "vertices") and hasattr(mesh, "polygons")

    if not is_mesh or mesh is None:
        return None

    mesh_obj = cast(bpy.types.Mesh, mesh)

    vert_count = len(mesh_obj.vertices)
    face_count = len(mesh_obj.polygons)

    if face_count > PRIMITIVE_MAX_FACES:
        return None

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
        link_name: Name of the link (for mesh filename)
        geom_purpose: "visual" or "collision" (use PURPOSE_VISUAL, PURPOSE_COLLISION)
        meshes_dir: Directory to export mesh files to
        mesh_format: "STL", "OBJ", or "GLB" (use FORMAT_STL, etc.)
        simplify: Whether to simplify mesh (for collision)
        decimation_ratio: Simplification ratio if simplify=True
        dry_run: If True, generate mesh paths but don't write files
        suffix: Optional unique suffix (e.g., index or name)
        depsgraph: Optional dependency graph for evaluation

    Returns:
        tuple of (Core Geometry or None, geometry_world_matrix)
    """
    if obj is None:
        return None, Matrix.Identity(4)

    geom_props = getattr(obj, PROP_GEOM, None)
    actual_geometry_type = geom_props.geometry_type if geom_props else GEOM_MESH

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
                return Mesh(
                    resource=str(mesh_path), scale=Vector3(1.0, 1.0, 1.0)
                ), geom_world_matrix

        actual_geometry_type = detect_primitive_type(obj) or GEOM_BOX

    if actual_geometry_type in (GEOM_BOX, GEOM_CYLINDER, GEOM_SPHERE):
        # Calculate local geometric center from bounding box
        local_center = get_local_bounding_box_center(obj)
        # Apply offset to get the true center of the geometry
        geom_world_matrix = obj.matrix_world @ Matrix.Translation(local_center)

        dimensions = getattr(obj, "dimensions", None)
        if dimensions is None:
            return None, Matrix.Identity(4)

        if dimensions.length < GEOM_EPSILON:
            logger.warning(f"Skipping geometry for '{obj.name}': Dimensions are zero.")
            return None, Matrix.Identity(4)

        if actual_geometry_type == GEOM_BOX:
            return Box(size=Vector3(dimensions.x, dimensions.y, dimensions.z)), geom_world_matrix

        elif actual_geometry_type == GEOM_CYLINDER:
            radius = max(dimensions.x, dimensions.y) / 2.0
            length = dimensions.z
            return Cylinder(radius=radius, length=length), geom_world_matrix

        elif actual_geometry_type == GEOM_SPHERE:
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

    if np is not None:
        num_verts = len(mesh_data.vertices)
        verts = np.zeros(num_verts * 3, dtype=np.float32)
        mesh_data.vertices.foreach_get("co", verts)
        vertices_array = verts.reshape((-1, 3))

        num_tris = len(mesh_data.loop_triangles)
        tris = np.zeros(num_tris * 3, dtype=np.int32)
        mesh_data.loop_triangles.foreach_get("vertices", tris)
        triangles_array = tris.reshape((-1, 3))

        # Apply scale
        vertices_array[:, 0] *= scale_matrix.x
        vertices_array[:, 1] *= scale_matrix.y
        vertices_array[:, 2] *= scale_matrix.z

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

    mat_name = f"{sanitize_name(obj.name)}_material"
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
