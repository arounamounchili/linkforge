"""Mesh export utilities for LinkForge.

Export Blender mesh objects to STL, OBJ, and DAE files for URDF.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import bpy

from ..core.logging_config import get_logger

logger = get_logger(__name__)


def export_mesh_stl(obj: Any, filepath: Path) -> bool:
    """Export Blender object to STL file.

    Args:
        obj: Blender Object to export
        filepath: Path where STL file should be saved

    Returns:
        True if export succeeded, False otherwise

    """
    if obj is None:
        return False

    # Ensure parent directory exists
    filepath.parent.mkdir(parents=True, exist_ok=True)

    # Deselect all and select only target object
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    # Export to STL
    try:
        bpy.ops.wm.stl_export(
            filepath=str(filepath),
            export_selected_objects=True,
            apply_modifiers=True,
            forward_axis="Y",
            up_axis="Z",
        )
        return True
    except (RuntimeError, OSError) as e:
        logger.warning(f"STL export failed: {e}")
        return False
    except (TypeError, AttributeError, KeyError) as e:
        logger.error(f"Unexpected error during STL export: {e}", exc_info=True)
        raise
    except Exception as e:
        logger.critical(f"Critical unexpected error during STL export: {e}", exc_info=True)
        raise


def export_mesh_obj(obj: Any, filepath: Path) -> bool:
    """Export Blender object to OBJ file (with MTL material).

    Args:
        obj: Blender Object to export
        filepath: Path where OBJ file should be saved

    Returns:
        True if export succeeded, False otherwise

    """
    if obj is None:
        return False

    # Ensure parent directory exists
    filepath.parent.mkdir(parents=True, exist_ok=True)

    # Deselect all and select only target object
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    # Export to OBJ
    try:
        bpy.ops.wm.obj_export(
            filepath=str(filepath),
            export_selected_objects=True,
            apply_modifiers=True,
            export_materials=True,
            forward_axis="Y",
            up_axis="Z",
        )
        return True
    except (RuntimeError, OSError) as e:
        logger.warning(f"OBJ export failed: {e}")
        return False
    except (TypeError, AttributeError, KeyError) as e:
        logger.error(f"Unexpected error during OBJ export: {e}", exc_info=True)
        raise
    except Exception as e:
        logger.critical(f"Critical unexpected error during OBJ export: {e}", exc_info=True)
        raise


def is_dae_supported() -> bool:
    """Check if COLLADA (DAE) export is supported in the current environment.

    Blender 5.0+ has removed built-in COLLADA support. For older versions,
    support depends on whether the Collada operator is available.

    Returns:
        True if the exporter is available, False otherwise.

    """
    if bpy.app.version >= (5, 0, 0) or not hasattr(bpy.ops.wm, "collada_export"):
        msg = (
            "COLLADA (DAE) support was removed in Blender 5.0. "
            "Please use glTF (.glb) for textured visual meshes."
        )
        logger.error(msg)
        return False

    return True


def export_mesh_dae(obj: Any, filepath: Path) -> bool:
    """Export Blender object to DAE (COLLADA) file.

    Args:
        obj: Blender Object to export
        filepath: Path where DAE file should be saved

    Returns:
        True if export succeeded, False otherwise

    """
    if obj is None:
        return False

    # Check for DAE support (Version-aware and hack-free)
    if not is_dae_supported():
        return False

    # Ensure parent directory exists
    filepath.parent.mkdir(parents=True, exist_ok=True)

    # Deselect all and select only target object
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    # Export to COLLADA (DAE)
    try:
        bpy.ops.wm.collada_export(
            filepath=str(filepath),
            selected=True,
            apply_modifiers=True,
            export_mesh_type_selection="view",
            triangulate=True,
        )
        return True
    except (RuntimeError, OSError) as e:
        logger.warning(f"DAE export failed: {e}")
        return False
    except (TypeError, AttributeError, KeyError) as e:
        logger.error(f"Unexpected error during DAE export: {e}", exc_info=True)
        return False
    except Exception as e:
        logger.critical(f"Critical unexpected error during DAE export: {e}", exc_info=True)
        raise


def create_simplified_mesh(obj: Any, decimation_ratio: float) -> Any | None:
    """Create a simplified copy of mesh using decimation.

    Args:
        obj: Blender Object to simplify
        decimation_ratio: Target ratio of faces to keep (0.0-1.0)

    Returns:
        Simplified Blender Object or None

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
    bpy.context.view_layer.objects.active = simplified_obj
    bpy.ops.object.modifier_apply(modifier=decimate_mod.name)

    return simplified_obj


def get_mesh_filename(link_name: str, geometry_type: str, mesh_format: str) -> str:
    """Generate mesh filename based on link and geometry type.

    Args:
        link_name: Name of the robot link
        geometry_type: "visual" or "collision"
        mesh_format: "STL" or "DAE"

    Returns:
        Filename string (e.g., "base_link_visual.stl")

    """
    ext = mesh_format.lower()
    return f"{link_name}_{geometry_type}.{ext}"


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

    # Ensure parent directory exists
    filepath.parent.mkdir(parents=True, exist_ok=True)

    # Deselect all and select only target object
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    # Export to GLB
    try:
        bpy.ops.export_scene.gltf(
            filepath=str(filepath),
            export_format="GLB",
            use_selection=True,
            export_apply=True,
            # We want Y-up for standard conventions, usually handled by glTF exporter automatically
            # but Blender Z-up to glTF Y-up conversion is standard.
        )
        return True
    except (RuntimeError, OSError) as e:
        logger.warning(f"GLB export failed: {e}")
        return False
    except (TypeError, AttributeError, KeyError) as e:
        logger.error(f"Unexpected error during GLB export: {e}", exc_info=True)
        raise
    except Exception as e:
        logger.critical(f"Critical unexpected error during GLB export: {e}", exc_info=True)
        raise


def export_link_mesh(
    obj: Any,
    link_name: str,
    geometry_type: str,
    mesh_format: str,
    meshes_dir: Path,
    simplify: bool = False,
    decimation_ratio: float = 0.5,
    dry_run: bool = False,
) -> Path | None:
    """Export mesh for a robot link.

    CRITICAL: Exports mesh geometry centered at origin (0,0,0) with no transforms.
    The visual origin in URDF will handle all positioning. This prevents double-offset
    issues when the mesh is re-imported.

    Args:
        obj: Blender Object to export
        link_name: Name of the robot link
        geometry_type: "visual" or "collision"
        mesh_format: "STL" or "DAE"
        meshes_dir: Directory where mesh files should be saved
        simplify: Whether to simplify mesh (for collision)
        decimation_ratio: Simplification ratio if simplify=True
        dry_run: If True, return expected path without exporting

    Returns:
        Path to exported mesh file, or None if export failed

    """
    if obj is None or obj.type != "MESH":
        return None

    # Generate filename
    filename = get_mesh_filename(link_name, geometry_type, mesh_format)
    filepath = meshes_dir / filename

    if dry_run:
        return filepath

    # Create a temporary clone with transforms applied for mesh export.
    # Data-level copying is used to avoid dependencies on Blender's operator context
    # and viewport visibility states.
    temp_export_obj = obj.copy()
    temp_export_obj.data = obj.data.copy()

    # Temporarily unhide the duplication (not the original) for transform application
    temp_export_obj.hide_viewport = False

    # Link to the same collections as the original
    for col in obj.users_collection:
        col.objects.link(temp_export_obj)

    # Select and make active for transform application
    bpy.ops.object.select_all(action="DESELECT")
    temp_export_obj.select_set(True)
    bpy.context.view_layer.objects.active = temp_export_obj

    # CRITICAL FIX: Clear parent to ensure world-space positioning
    # Without this, setting location=(0,0,0) moves to parent's local origin,
    # which causes the parent's world transform to be baked into the mesh
    if temp_export_obj.parent:
        # Store the world matrix before unparenting
        world_matrix = temp_export_obj.matrix_world.copy()
        # Clear parent (keep transform)
        temp_export_obj.parent = None
        # Restore world matrix to maintain current world position
        temp_export_obj.matrix_world = world_matrix

    # Apply scale transform to bake dimensions into mesh
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    # Reset location and rotation to world origin
    # Now that parent is cleared, this truly centers at (0,0,0) in world space
    temp_export_obj.location = (0, 0, 0)
    temp_export_obj.rotation_euler = (0, 0, 0)

    # Force update
    if hasattr(bpy.context, "view_layer"):
        bpy.context.view_layer.update()

    # Simplify if requested for collision
    simplified_obj = None
    export_obj = temp_export_obj

    if simplify and geometry_type == "collision":
        simplified_obj = create_simplified_mesh(temp_export_obj, decimation_ratio)
        if simplified_obj:
            export_obj = simplified_obj

    # Export based on format
    success = False
    if mesh_format.upper() == "STL":
        success = export_mesh_stl(export_obj, filepath)
    elif mesh_format.upper() == "OBJ":
        success = export_mesh_obj(export_obj, filepath)
    elif mesh_format.upper() == "DAE":
        success = export_mesh_dae(export_obj, filepath)
    elif mesh_format.upper() == "GLB":
        success = export_mesh_glb(export_obj, filepath)
    else:
        # Unknown format, default to OBJ
        logger.warning(f"Unknown mesh format '{mesh_format}', defaulting to OBJ")
        filepath = filepath.with_suffix(".obj")
        success = export_mesh_obj(export_obj, filepath)

    # Clean up temporary objects
    if simplified_obj:
        bpy.data.objects.remove(simplified_obj, do_unlink=True)
    if temp_export_obj:
        bpy.data.objects.remove(temp_export_obj, do_unlink=True)

    return filepath if success else None
