"""Mesh export utilities for LinkForge.

Export Blender mesh objects to STL, OBJ, and GLB files for URDF.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import bpy
from linkforge_core.logging_config import get_logger
from linkforge_core.utils.string_utils import sanitize_name
from mathutils import Matrix, Vector

logger = get_logger(__name__)


def export_mesh_stl(obj: Any, filepath: Path) -> bool:
    """Export a Blender object to an STL file.

    This function utilizes the modern Blender WM STL exporter, ensuring
    correct axis orientations (Y-forward, Z-up) for ROS 2 compatibility.

    Args:
        obj: The Blender mesh object to export.
        filepath: Target filesystem path for the STL file.

    Returns:
        True if the export completed successfully, False otherwise.
    """
    if obj is None:
        return False

    # Ensure object is visible before selection for reliable Blender context.
    was_hidden = obj.hide_viewport

    # Ensure parent directory exists
    try:
        filepath.parent.mkdir(parents=True, exist_ok=True)

        # Deselect all and select only target object
        obj.hide_viewport = False

        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = obj

        # Export to STL
        bpy.ops.wm.stl_export(
            filepath=str(filepath),
            export_selected_objects=True,
            apply_modifiers=True,
            forward_axis="Y",
            up_axis="Z",
        )
    except (RuntimeError, OSError) as e:
        logger.warning(f"STL export failed: {e}")
        # Restore visibility if failed
        if "was_hidden" in locals():
            obj.hide_viewport = was_hidden
        return False
    except (TypeError, AttributeError, KeyError) as e:
        logger.error(f"Unexpected error during STL export: {e}", exc_info=True)
        raise
    except Exception as e:
        logger.critical(f"Critical unexpected error during STL export: {e}", exc_info=True)
        raise
    finally:
        # Restore visibility state
        obj.hide_viewport = was_hidden

    return True


def export_mesh_obj(obj: Any, filepath: Path) -> bool:
    """Export a Blender object to an OBJ file with associated MTL materials.

    This function ensures that materials are correctly exported alongside
    the geometry, maintaining visual fidelity in the target URDF.

    Args:
        obj: The Blender mesh object to export.
        filepath: Target filesystem path for the OBJ file.

    Returns:
        True if the export completed successfully, False otherwise.
    """
    if obj is None:
        return False

    # Store visibility state before modifying
    was_hidden = obj.hide_viewport
    # Ensure parent directory exists
    try:
        filepath.parent.mkdir(parents=True, exist_ok=True)

        # Deselect all and select only target object
        # Selection requires object visibility
        obj.hide_viewport = False

        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = obj

        # Export to OBJ
        bpy.ops.wm.obj_export(
            filepath=str(filepath),
            export_selected_objects=True,
            apply_modifiers=True,
            export_materials=True,
            forward_axis="Y",
            up_axis="Z",
        )
    except (RuntimeError, OSError) as e:
        logger.warning(f"OBJ export failed: {e}")
        if "was_hidden" in locals():
            obj.hide_viewport = was_hidden
        return False
    except (TypeError, AttributeError, KeyError) as e:
        logger.error(f"Unexpected error during OBJ export: {e}", exc_info=True)
        raise
    except Exception as e:
        logger.critical(f"Critical unexpected error during OBJ export: {e}", exc_info=True)
        raise
    finally:
        # Restore visibility state
        obj.hide_viewport = was_hidden

    return True


def create_simplified_mesh(obj: Any, decimation_ratio: float) -> Any | None:
    """Create a simplified mesh copy using Blender's Decimate modifier.

    This function is primarily used to generate lightweight collision geometry
    from high-fidelity visual meshes, reducing physics computation overhead.

    Args:
        obj: The source Blender mesh object.
        decimation_ratio: The target triangle count ratio (0.0 to 1.0).

    Returns:
        A new Blender object with the simplified mesh, or None if failed.
    """
    if obj is None or obj.type != "MESH":
        return None

    # Use data-level copy instead of high-level duplicate operator to ensure
    # operation succeeds regardless of viewport visibility (hide_viewport state).
    simplified_obj = obj.copy()
    simplified_obj.data = obj.data.copy()

    # Ensure temporary object is visible for modifier application
    simplified_obj.hide_viewport = False

    # Link to the same collections as the original
    for col in obj.users_collection:
        col.objects.link(simplified_obj)

    # Add Decimate modifier
    decimate_mod = simplified_obj.modifiers.new(name="Decimate", type="DECIMATE")
    decimate_mod.ratio = decimation_ratio
    decimate_mod.decimate_type = "COLLAPSE"

    # Apply the modifier
    bpy.ops.object.select_all(action="DESELECT")
    simplified_obj.select_set(True)
    if bpy.context.view_layer:
        bpy.context.view_layer.objects.active = simplified_obj
    bpy.ops.object.modifier_apply(modifier=decimate_mod.name)

    return simplified_obj


def get_mesh_filename(
    link_name: str, geometry_type: str, mesh_format: str, suffix: str = ""
) -> str:
    """Generate mesh filename based on link and geometry type.

    Args:
        link_name: Name of the robot link
        geometry_type: "visual" or "collision"
        mesh_format: "STL", "OBJ", or "GLB"
        suffix: Optional unique suffix (e.g., index or name)

    Returns:
        Filename string (e.g., "base_link_visual_0.stl").

    """
    ext = mesh_format.lower()
    # Sanitize both link_name and suffix for URDF/filesystem compatibility
    clean_link = sanitize_name(link_name)
    clean_suffix = sanitize_name(suffix) if suffix else ""
    return f"{clean_link}_{geometry_type}{clean_suffix}.{ext}"


def export_mesh_glb(obj: Any, filepath: Path) -> bool:
    """Export Blender object to GLB (glTF Binary) file.

    Args:
        obj: Blender Object to export
        filepath: Path where GLB file should be saved

    Returns:
        True if export succeeded, False otherwise

    """
    if obj is None:
        return False

    # Store visibility state before modifying
    was_hidden = obj.hide_viewport
    # Ensure parent directory exists
    try:
        filepath.parent.mkdir(parents=True, exist_ok=True)

        # Deselect all and select only target object
        # Selection requires object visibility
        obj.hide_viewport = False

        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = obj

        # Export to GLB
        bpy.ops.export_scene.gltf(
            filepath=str(filepath),
            export_format="GLB",
            use_selection=True,
            export_apply=True,
            # We want Y-up for standard conventions, usually handled by glTF exporter automatically
            # but Blender Z-up to glTF Y-up conversion is standard.
        )
    except (RuntimeError, OSError) as e:
        logger.warning(f"GLB export failed: {e}")
        if "was_hidden" in locals():
            obj.hide_viewport = was_hidden
        return False
    except (TypeError, AttributeError, KeyError) as e:
        logger.error(f"Unexpected error during GLB export: {e}", exc_info=True)
        raise
    except Exception as e:
        logger.critical(f"Critical unexpected error during GLB export: {e}", exc_info=True)
        raise
    finally:
        # Restore visibility state
        obj.hide_viewport = was_hidden

    return True


def export_link_mesh(
    obj: Any,
    link_name: str,
    geometry_type: str,
    mesh_format: str,
    meshes_dir: Path,
    simplify: bool = False,
    decimation_ratio: float = 0.5,
    dry_run: bool = False,
    suffix: str = "",
    depsgraph: Any | None = None,
) -> tuple[Path | None, Matrix]:
    """Export mesh for a robot link.

    CRITICAL: Exports mesh geometry centered at origin (0,0,0) with no transforms.
    The visual origin in URDF will handle all positioning. This prevents double-offset
    issues when the mesh is re-imported.

    Args:
        obj: Blender Object to export
        link_name: Name of the robot link
        geometry_type: "visual" or "collision"
        mesh_format: "STL", "OBJ", or "GLB"
        meshes_dir: Directory where mesh files should be saved
        simplify: Whether to simplify mesh (for collision)
        decimation_ratio: Simplification ratio if simplify=True
        dry_run: If True, return expected path without exporting

    Returns:
        tuple of (Path to exported mesh file or None, geometric_offset)

    """
    if obj is None or obj.type != "MESH":
        return None, Matrix.Identity(4)

    # Initialize variables for cleanup in finally block
    temp_export_obj = None
    simplified_obj = None
    final_mesh_data = None

    # Generate filename with suffix
    filename = get_mesh_filename(link_name, geometry_type, mesh_format, suffix=suffix)
    filepath = meshes_dir / filename

    if dry_run:
        return filepath, obj.matrix_world.copy()

    # Create a temporary clone for mesh export.
    temp_export_obj = obj.copy()
    temp_export_obj.data = obj.data.copy()

    # Link to the same collections as the original
    for col in obj.users_collection:
        col.objects.link(temp_export_obj)

    # CRITICAL FIX: Local Fidelity Centering
    # 1. Bake SCALE into the mesh data (ensures 1.0 scale in URDF)
    # 2. DO NOT bake rotation (keeps mesh orientations relative to links)
    scale_matrix = Matrix.Diagonal((*obj.scale, 1.0))
    temp_export_obj.data.transform(scale_matrix)
    temp_export_obj.scale = (1, 1, 1)

    # Calculate local geometric center of the EVALUATED mesh
    try:
        if depsgraph is None:
            depsgraph = bpy.context.evaluated_depsgraph_get()
        obj_eval = obj.evaluated_get(depsgraph)

        # Corners are in local space
        local_corners = [Vector(corner) for corner in obj_eval.bound_box]

        min_v = Vector(tuple(min(v[i] for v in local_corners) for i in range(3)))
        max_v = Vector(tuple(max(v[i] for v in local_corners) for i in range(3)))
        local_center = (min_v + max_v) / 2

        # Create the final mesh data (evaluated with modifiers applied)
        final_mesh_data = bpy.data.meshes.new_from_object(
            obj_eval, preserve_all_data_layers=True, depsgraph=depsgraph
        )

        # Apply Scale to final data
        final_mesh_data.transform(scale_matrix)

        # Local Centering: Shift vertices to origin
        if local_center.length > 1e-6:
            logger.info(f"Localizing mesh data for '{obj.name}' (center: {local_center})")
            final_mesh_data.transform(Matrix.Translation(-local_center))

        # Update temporary object to use this centered data
        temp_export_obj.data = final_mesh_data

        # CRITICAL: Reset object transform to Identity
        # This prevents the file exporter (STL/OBJ) from applying the world transform
        # a second time. The mesh data is already scaled and centered locally.
        # The world pose will be handled entirely by the URDF <origin>.
        temp_export_obj.matrix_world.identity()

        # The World Pose of this centered geometry is: Obj_World @ Translation(local_center)
        # This correctly accounts for the object's rotation while incorporating the centering shift.
        geom_world_matrix = obj.matrix_world @ Matrix.Translation(local_center)

        # Simplify if requested for collision
        export_obj = temp_export_obj

        if simplify and geometry_type == "collision":
            # Simplified object will inherit centered data
            simplified_obj = create_simplified_mesh(temp_export_obj, decimation_ratio)
            if simplified_obj:
                export_obj = simplified_obj

        # Export based on format
        success = False
        if mesh_format.upper() == "STL":
            success = export_mesh_stl(export_obj, filepath)
        elif mesh_format.upper() == "OBJ":
            success = export_mesh_obj(export_obj, filepath)
        elif mesh_format.upper() == "GLB":
            success = export_mesh_glb(export_obj, filepath)
        else:
            # Unknown format, default to OBJ
            logger.warning(f"Unknown mesh format '{mesh_format}', defaulting to OBJ")
            filepath = filepath.with_suffix(".obj")
            success = export_mesh_obj(export_obj, filepath)

        if success:
            logger.info(f"Successfully exported mesh: {filepath}")
            # Return the world matrix of the centered geometry
            return filepath, geom_world_matrix
        else:
            logger.error(f"Failed to export mesh: {filepath}")
            return None, Matrix.Identity(4)

    except Exception as e:
        logger.error(f"Error during mesh export: {e}")
        return None, Matrix.Identity(4)
    finally:
        # Cleanup temporary objects
        if simplified_obj:
            data = simplified_obj.data
            bpy.data.objects.remove(simplified_obj, do_unlink=True)
            if data:
                bpy.data.meshes.remove(data, do_unlink=True)
        if temp_export_obj:
            data = temp_export_obj.data
            bpy.data.objects.remove(temp_export_obj, do_unlink=True)
            # Remove both the cloned data and the final mesh data if they exist
            if data:
                bpy.data.meshes.remove(data, do_unlink=True)
            if final_mesh_data and final_mesh_data != data:
                bpy.data.meshes.remove(final_mesh_data, do_unlink=True)
