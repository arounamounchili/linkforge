"""Procedural collision geometry and mesh generation logic for robot links.

This module provides functions for detecting, generating, and updating collision
geometry (both primitives and compound decimated convex hulls) from visual meshes.
"""

from __future__ import annotations

import typing

import bmesh
import bpy
import mathutils
from bpy.types import Context

from ..adapters.geometry_extractor import detect_primitive_type
from ..constants import (
    GEOM_AUTO,
    PROP_LINK,
    SUFFIX_COLLISION,
    SUFFIX_VISUAL,
)
from ..core import get_logger
from ..core.constants import (
    GEOM_BOX,
    GEOM_CYLINDER,
    GEOM_MESH,
    GEOM_SPHERE,
)
from ..properties.geom_props import PROP_GEOM
from ..properties.link_props import LinkPropertyGroup
from ..utils.mode_guard import context_and_mode_guard
from ..utils.scene_utils import sync_object_collections

logger = get_logger(__name__)


def regenerate_collision_mesh(
    link_obj: bpy.types.Object, collision_type: str, context: Context
) -> None:
    """Helper to regenerate collision mesh for a link from its visuals.

    Args:
        link_obj: The link object (Empty)
        collision_type: Type of collision ("auto", "box", "sphere", "cylinder", "mesh")
        context: Blender context
    """
    if (
        not link_obj
        or not hasattr(link_obj, PROP_LINK)
        or not typing.cast(LinkPropertyGroup, getattr(link_obj, PROP_LINK)).is_robot_link
    ):
        return

    # Filter out non-mesh visual children (e.g. empties)
    visual_children = [
        c for c in link_obj.children if SUFFIX_VISUAL in c.name.lower() and c.type == "MESH"
    ]
    if not visual_children:
        return

    # Delete existing collision meshes for this link
    existing_collisions = [c for c in link_obj.children if SUFFIX_COLLISION in c.name.lower()]
    hide_viewport = True
    if existing_collisions:
        # Preserve visibility of the first existing collision
        hide_viewport = existing_collisions[0].hide_viewport
        for col_obj in existing_collisions:
            bpy.data.objects.remove(col_obj, do_unlink=True)

    # Create new collision
    new_col = create_collision_for_link(link_obj, collision_type, context)
    if new_col:
        new_col.hide_viewport = hide_viewport


def create_collision_for_link(
    link_obj: bpy.types.Object, collision_type: str, context: Context
) -> bpy.types.Object | None:
    """Create collision geometry for a link.

    For links with multiple visual children, this creates a compound collision
    by merging all visuals into a single collision mesh (industry best practice).

    Args:
        link_obj: The link object (Empty)
        collision_type: Type of collision ("auto", "box", "sphere", "cylinder", "mesh")
        context: Blender context

    Returns:
        The created collision object, or None if failed
    """
    visual_children = [
        c for c in link_obj.children if SUFFIX_VISUAL in c.name.lower() and c.type == "MESH"
    ]

    # If no explicit visual children, check if the link object itself is a mesh
    if not visual_children and link_obj.type == "MESH":
        visual_children = [link_obj]

    if not visual_children:
        return None

    lf = typing.cast(LinkPropertyGroup, getattr(link_obj, PROP_LINK))
    link_name = lf.link_name or link_obj.name

    # Production-grade fix: Wrap all collision operators in a context and mode guard
    with context_and_mode_guard(context):
        # Remove existing collision objects to prevent duplicates
        existing_collisions = [c for c in link_obj.children if SUFFIX_COLLISION in c.name.lower()]
        for col in existing_collisions:
            bpy.data.objects.remove(col, do_unlink=True)

        # Determine collision type
        if collision_type == GEOM_AUTO:
            # For multiple visuals, always use mesh simplification (compound collision)
            if len(visual_children) > 1:
                collision_type = GEOM_MESH
            else:
                # Single visual - try to detect primitive
                detected = detect_primitive_type(visual_children[0])
                collision_type = detected if detected else GEOM_MESH

        # Determine collision type and generate geometry
        local_offset = mathutils.Vector((0, 0, 0))
        if collision_type in (GEOM_BOX, GEOM_CYLINDER, GEOM_SPHERE):
            # Primitives only make sense for single visuals
            collision_obj, local_offset = _create_primitive_collision(
                visual_children[0], collision_type, link_name, context
            )
            reference_visual = visual_children[0]
        else:  # MESH
            # Merge ALL visuals into compound collision (geometry is baked link-local)
            collision_obj = _create_mesh_collision_compound(visual_children, link_obj, context)
            # For compound mesh, we use the link itself as the local reference frame
            # Since merged geometry is already in link-local coordinates, the local matrix must be Identity
            reference_visual = None
            local_offset = mathutils.Vector((0, 0, 0))

        if collision_obj is None:
            return None

        # Parent to link using Strict Alignment
        collision_obj.parent = link_obj

        if reference_visual:
            # PRIMITIVE: Align with visual x local offset
            collision_obj.matrix_parent_inverse = reference_visual.matrix_parent_inverse.copy()
            collision_obj.matrix_local = (
                reference_visual.matrix_local @ mathutils.Matrix.Translation(local_offset)
            )
            # CRITICAL: Match World Dimensions LAST to override any scale from matrix_local
            collision_obj.dimensions = reference_visual.dimensions.copy()
        else:
            # MESH: Already baked link-local, just reset transforms
            collision_obj.matrix_parent_inverse.identity()
            collision_obj.matrix_local.identity()
            collision_obj.scale = (1, 1, 1)  # Scale was already baked into geometry

        # IMPORTANT: Ensure collision is synchronized with the link's collection hierarchy
        sync_object_collections(collision_obj, link_obj)
        if not collision_obj.users_collection and context.collection:
            context.collection.objects.link(collision_obj)

        if collision_obj.data and hasattr(collision_obj.data, "materials"):
            collision_obj.data.materials.clear()

        collision_obj.display_type = "WIRE"
        collision_obj.show_in_front = True
        collision_obj.rotation_mode = "XYZ"

        geom_props = getattr(collision_obj, PROP_GEOM, None)
        if geom_props:
            geom_props.geom_role = "COLLISION"
            geom_props.geometry_type = collision_type
        collision_obj.hide_viewport = False
        collision_obj.hide_render = True

        return collision_obj


def _create_primitive_collision(
    visual_obj: bpy.types.Object, prim_type: str, link_name: str, context: Context
) -> tuple[bpy.types.Object | None, mathutils.Vector]:
    """Create primitive collision geometry aligned with geometry center.

    Returns:
        tuple: (collision_obj, local_center_offset)
    """
    # Calculate geometric center from bounding box in local space
    # This allows correctly centering primitives even if the mesh origin is offset
    local_bbox = [mathutils.Vector(v) for v in visual_obj.bound_box]
    local_center = sum(local_bbox, mathutils.Vector((0, 0, 0))) / 8.0

    # Create primitive at world origin initially
    # We create them at unit size for predictable scaling via dimensions
    ops = getattr(context, "ops", None) or bpy.ops
    if prim_type == GEOM_BOX:
        # Create cube (1x1x1)
        ops.mesh.primitive_cube_add(size=1.0, location=(0, 0, 0))
    elif prim_type == GEOM_SPHERE:
        # Create sphere (radius 0.5 = 1m diameter)
        ops.mesh.primitive_uv_sphere_add(radius=0.5, location=(0, 0, 0))
    elif prim_type == GEOM_CYLINDER:
        # Create cylinder (radius 0.5, depth 1.0 = 1x1x1 volume)
        ops.mesh.primitive_cylinder_add(radius=0.5, depth=1.0, location=(0, 0, 0))
    else:
        return None, mathutils.Vector((0, 0, 0))

    collision_obj = getattr(context, "active_object", bpy.context.active_object)

    # CRITICAL: Match World Pose (Location/Rotation) first
    # We include local_center offset to align primitive with specific geometry volume
    if collision_obj:
        collision_obj.matrix_world = visual_obj.matrix_world @ mathutils.Matrix.Translation(
            local_center
        )

        # CRITICAL: Match World Dimensions exactly
        # Setting dimensions AFTER matrix_world ensures it overrides any scale inherited from visual
        collision_obj.dimensions = visual_obj.dimensions.copy()

        # Apply scale to bake dimensions into geometry (Scale 1.0 standard)
        collision_obj.hide_viewport = False
        ops.object.transform_apply(location=False, rotation=False, scale=True)

    # Name it
    if collision_obj:
        collision_obj.name = f"{link_name}_collision"
        collision_obj.rotation_mode = "XYZ"

    return collision_obj, local_center


def _merge_visual_meshes(
    visual_objects: list[bpy.types.Object], link_obj: bpy.types.Object, context: Context
) -> bpy.types.Object | None:
    """Merge multiple visual meshes into a single temporary mesh for compound collision.

    This creates a compound mesh that represents all visual geometry in the
    LOCAL space of the link, which is then used to generate a single
    accurate collision mesh aligned with the link origin.

    Args:
        visual_objects: List of visual mesh objects to merge
        link_obj: The parent link object (coordinate frame reference)
        context: Blender context

    Returns:
        Merged mesh object (temporary, caller must clean up)
    """
    if not visual_objects:
        return None

    # Log collision generation for debugging and user feedback
    logger.debug(
        f"Compound collision: merging {len(visual_objects)} visual mesh(es) for {link_obj.name}"
    )

    # Create clones of visual objects using data-level duplication to ensure
    # robustness against object visibility states in the viewport.
    duplicates = []
    for visual_obj in visual_objects:
        if not visual_obj.data:
            continue

        dup = visual_obj.copy()
        # Create unique mesh data duplicate
        dup.data = visual_obj.data.copy()

        # Ensure temporary duplicate is visible for join operation
        dup.hide_viewport = False

        # Link to the same collections as the original
        for col in visual_obj.users_collection:
            col.objects.link(dup)

        # Apply transforms to bake local position (relative to link) into geometry
        dup.parent = None
        dup.matrix_world = link_obj.matrix_world.inverted() @ visual_obj.matrix_world

        # Select and make active for transform application
        ops = getattr(context, "ops", None) or bpy.ops
        ops.object.select_all(action="DESELECT")
        dup.select_set(True)
        vl = getattr(context, "view_layer", bpy.context.view_layer)
        if vl:
            vl.objects.active = dup

        ops.object.transform_apply(location=True, rotation=True, scale=True)

        duplicates.append(dup)

    # If no valid meshes found, return None
    if not duplicates:
        return None

    # If only one visual, return it directly
    if len(duplicates) == 1:
        return duplicates[0]

    # Multiple visuals - join them into single mesh
    ops = getattr(context, "ops", None) or bpy.ops
    ops.object.select_all(action="DESELECT")
    for dup in duplicates:
        dup.select_set(True)
    vl = getattr(context, "view_layer", bpy.context.view_layer)
    if vl and duplicates:
        vl.objects.active = duplicates[0]

    # Join into single mesh
    ops.object.join()
    merged_obj = getattr(context, "active_object", bpy.context.active_object)
    if not merged_obj:
        return None

    # CRITICAL: Align merged object with link world frame
    # Vertices were baked relative to Link, so the object definition must be AT the Link.
    merged_obj.matrix_world = link_obj.matrix_world.copy()

    logger.debug(
        f"Compound collision created: {merged_obj.name} "
        f"({len(typing.cast(bpy.types.Mesh, merged_obj.data).vertices)} vertices)"
    )

    return merged_obj


def _create_mesh_collision_compound(
    visual_objects: list[bpy.types.Object], link_obj: bpy.types.Object, context: Context
) -> bpy.types.Object | None:
    """Create compound simplified mesh collision from multiple visual meshes.

    This merges all visual children into a single collision mesh, following
    industry best practices (ROS, Gazebo, MoveIt).

    Args:
        visual_objects: List of visual mesh objects to merge
        link_obj: The parent link object
        context: Blender context

    Returns:
        Collision object with compound simplified mesh
    """
    # Merge all visual meshes into compound mesh (baked relative to link)
    merged_obj = _merge_visual_meshes(visual_objects, link_obj, context)

    if merged_obj is None:
        return None

    # Name it
    merged_obj.name = f"{link_obj.name}_collision"

    # Store original visibility state to restore later
    old_hide_viewport = merged_obj.hide_viewport
    old_hide_render = merged_obj.hide_render

    # Ensure merged_obj is visible and active for mesh simplification
    merged_obj.hide_viewport = False
    merged_obj.hide_render = False

    # Apply mesh simplification to the merged mesh
    merged_mesh = typing.cast(bpy.types.Mesh, merged_obj.data)
    bm = bmesh.new()
    bm.from_mesh(merged_mesh)

    # Convex Hull operation (native BMesh API)
    bmesh.ops.convex_hull(bm, input=list(bm.verts))

    # Clear original data and load new hull back to mesh data
    bm.to_mesh(merged_mesh)
    bm.free()
    merged_mesh.update()

    # Add decimation modifier for live quality adjustment
    quality_ratio = 0.5

    decimate_mod = typing.cast(
        bpy.types.DecimateModifier, merged_obj.modifiers.new(name="Decimate", type="DECIMATE")
    )
    decimate_mod.ratio = quality_ratio
    decimate_mod.decimate_type = "COLLAPSE"

    # Restore properties
    merged_obj.name = f"{link_obj.name}_collision"
    merged_obj.rotation_mode = "XYZ"

    # Clear materials from collision mesh
    if merged_obj.data and hasattr(merged_obj.data, "materials"):
        merged_obj.data.materials.clear()

    # Strict Alignment Parenting
    merged_obj.parent = link_obj
    merged_obj.matrix_parent_inverse.identity()
    merged_obj.matrix_local.identity()
    merged_obj.scale = (1, 1, 1)

    merged_obj.display_type = "WIRE"
    merged_obj.show_in_front = True
    merged_obj.hide_viewport = old_hide_viewport
    merged_obj.hide_render = old_hide_render

    # Persist collision type for UI consistency
    geom_props = getattr(merged_obj, PROP_GEOM, None)
    if geom_props:
        geom_props.geom_role = "COLLISION"
        geom_props.geometry_type = GEOM_MESH

    # Ensure it's in the same collections as the link
    sync_object_collections(merged_obj, link_obj)

    bpy.ops.object.select_all(action="DESELECT")
    link_obj.select_set(True)
    vl = context.view_layer
    if vl:
        vl.objects.active = link_obj

    return merged_obj
