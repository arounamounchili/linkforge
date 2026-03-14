"""Operators for managing robot links."""

from __future__ import annotations

import contextlib
import time
import typing

from ...linkforge_core.logging_config import get_logger
from ..properties.link_props import sanitize_urdf_name
from ..utils.context import context_and_mode_guard
from ..utils.decorators import OperatorReturn, safe_execute
from ..utils.scene_utils import clear_stats_cache

if typing.TYPE_CHECKING:
    from bpy.types import Context, Operator

    bpy: typing.Any
    bmesh: typing.Any
    mathutils: typing.Any

    from ..properties.link_props import LinkPropertyGroup
else:
    import bmesh
    import bpy
    import mathutils

    # Runtime fallback for mock environments where bpy.types might be partially loaded.
    Context = typing.Any
    Operator = getattr(getattr(bpy, "types", object), "Operator", object)

logger = get_logger(__name__)

# Global state for debounced collision preview updates
_preview_pending_object = None
_preview_last_request_time = 0.0

# Debounce delay for collision preview updates (in seconds)
COLLISION_PREVIEW_DEBOUNCE_DELAY = 0.3


def schedule_collision_preview_update(obj: bpy.types.Object) -> None:
    """Schedule a debounced collision preview update.

    This prevents excessive regeneration during slider interaction by
    waiting 0.3 seconds after the LAST change before updating.
    """
    # pylint: disable=global-statement
    global _preview_pending_object, _preview_last_request_time
    _preview_pending_object = obj
    _preview_last_request_time = time.time()

    # Register new timer with debounce delay
    if not bpy.app.timers.is_registered(execute_collision_preview_update):
        bpy.app.timers.register(
            execute_collision_preview_update, first_interval=COLLISION_PREVIEW_DEBOUNCE_DELAY
        )


@typing.no_type_check
def execute_collision_preview_update() -> None | float:
    """Execute the actual collision mesh update after debounce delay."""
    # pylint: disable=global-statement
    global _preview_pending_object, _preview_last_request_time

    if _preview_pending_object is None:
        return None

    # Implement TRUE debounce: if a new request happened recently, reschedule
    time_since_last = time.time() - _preview_last_request_time
    if time_since_last < COLLISION_PREVIEW_DEBOUNCE_DELAY:
        # Return remaining time to wait
        return COLLISION_PREVIEW_DEBOUNCE_DELAY - time_since_last

    obj = _preview_pending_object
    _preview_pending_object = None

    # Check if object still exists and is in the scene
    if not obj or obj.name not in bpy.data.objects:
        return None

    # Find collision object
    collision_obj = next((c for c in obj.children if "_collision" in c.name.lower()), None)
    if collision_obj is None:
        return None

    # Use bpy.context.view_layer reliably
    if not bpy.context.view_layer:
        return None

    # Check if it's a primitive (don't regenerate primitives)
    from ..adapters.blender_to_core import detect_primitive_type

    primitive_type = detect_primitive_type(collision_obj)
    if primitive_type is not None:
        return None

    # Check if it's imported from URDF (don't regenerate imported collisions)
    if collision_obj.get("imported_from_urdf", False):
        return None

    # Regenerate collision mesh with new quality
    # The collision_type is stored on the collision_obj itself
    collision_type = collision_obj.get("collision_geometry_type", "MESH")
    regenerate_collision_mesh(obj, str(collision_type), bpy.context)

    return None  # All caught up


def regenerate_collision_mesh(
    link_obj: bpy.types.Object, collision_type: str, context: Context
) -> None:
    """Helper to regenerate collision mesh for a link from its visuals.

    Args:
        link_obj: The link object (Empty)
        collision_type: Type of collision ("AUTO", "BOX", "SPHERE", "CYLINDER", "MESH")
        context: Blender context
    """
    if (
        not link_obj
        or not hasattr(link_obj, "linkforge")
        or not typing.cast("LinkPropertyGroup", getattr(link_obj, "linkforge")).is_robot_link
    ):
        return

    # Filter out non-mesh visual children (e.g. empties)
    visual_children = [
        c for c in link_obj.children if "_visual" in c.name.lower() and c.type == "MESH"
    ]
    if not visual_children:
        return

    # Delete existing collision meshes for this link
    existing_collisions = [c for c in link_obj.children if "_collision" in c.name.lower()]
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
        collision_type: Type of collision ("AUTO", "BOX", "SPHERE", "CYLINDER", "MESH")
        context: Blender context

    Returns:
        The created collision object, or None if failed
    """
    # Find ALL visual children (not just first one)
    visual_children = [
        c for c in link_obj.children if "_visual" in c.name.lower() and c.type == "MESH"
    ]

    if not visual_children:
        return None

    lf = typing.cast("LinkPropertyGroup", getattr(link_obj, "linkforge"))
    link_name = lf.link_name or link_obj.name

    # Production-grade fix: Wrap all collision operators in a context and mode guard
    with context_and_mode_guard(context):
        # Remove existing collision objects to prevent duplicates
        existing_collisions = [c for c in link_obj.children if "_collision" in c.name.lower()]
        for col in existing_collisions:
            bpy.data.objects.remove(col, do_unlink=True)

        # Import here to avoid circular dependency
        from ..adapters.blender_to_core import detect_primitive_type

        # Determine collision type
        if collision_type == "AUTO":
            # For multiple visuals, always use mesh simplification (compound collision)
            if len(visual_children) > 1:
                collision_type = "MESH"
            else:
                # Single visual - try to detect primitive
                detected = detect_primitive_type(visual_children[0])
                collision_type = detected if detected else "MESH"

        # Determine collision type and generate geometry
        local_offset = mathutils.Vector((0, 0, 0))
        if collision_type in ("BOX", "SPHERE", "CYLINDER"):
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

        # Align with strict precision
        if reference_visual:
            # PRIMITIVE: Align with visual x local offset
            collision_obj.matrix_parent_inverse = reference_visual.matrix_parent_inverse.copy()
            collision_obj.matrix_local = (
                reference_visual.matrix_local @ mathutils.Matrix.Translation(local_offset)
            )
        else:
            # MESH: Already baked link-local, just reset transforms
            collision_obj.matrix_parent_inverse.identity()
            collision_obj.matrix_local.identity()

        collision_obj.scale = (1, 1, 1)  # Scale was already baked into geometry

        # IMPORTANT: Ensure collision is actually a child in the collection hierarchy
        if context.collection and collision_obj.name not in context.collection.objects:
            context.collection.objects.link(collision_obj)

        if collision_obj.data and hasattr(collision_obj.data, "materials"):
            collision_obj.data.materials.clear()

        collision_obj.display_type = "WIRE"
        collision_obj.show_in_front = True
        collision_obj.rotation_mode = "XYZ"
        collision_obj["collision_geometry_type"] = collision_type
        collision_obj.hide_viewport = True
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
    if prim_type == "BOX":
        # Create cube (1x1x1)
        bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0, 0, 0))
    elif prim_type == "SPHERE":
        # Create sphere (radius 0.5 = 1m diameter)
        bpy.ops.mesh.primitive_uv_sphere_add(radius=0.5, location=(0, 0, 0))
    elif prim_type == "CYLINDER":
        # Create cylinder (radius 0.5, depth 1.0 = 1x1x1 volume)
        bpy.ops.mesh.primitive_cylinder_add(radius=0.5, depth=1.0, location=(0, 0, 0))
    else:
        return None, mathutils.Vector((0, 0, 0))

    collision_obj = context.active_object

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
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

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
        # Vertex_Link = Link_World_Inv @ Vertex_World
        # We unparent first to ensure matrix_world correctly represents the local space
        dup.parent = None
        dup.matrix_world = link_obj.matrix_world.inverted() @ visual_obj.matrix_world

        # Select and make active for transform application
        bpy.ops.object.select_all(action="DESELECT")
        dup.select_set(True)
        vl = context.view_layer
        if vl:
            vl.objects.active = dup

        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

        duplicates.append(dup)

    # If no valid meshes found, return None
    if not duplicates:
        return None

    # If only one visual, return it directly
    if len(duplicates) == 1:
        return duplicates[0]

    # Multiple visuals - join them into single mesh
    bpy.ops.object.select_all(action="DESELECT")
    for dup in duplicates:
        dup.select_set(True)
    vl = context.view_layer
    if vl and duplicates:
        vl.objects.active = duplicates[0]

    # Join into single mesh
    bpy.ops.object.join()
    merged_obj = context.active_object
    if not merged_obj:
        return None

    # CRITICAL: Align merged object with link world frame
    # Vertices were baked relative to Link, so the object definition must be AT the Link.
    merged_obj.matrix_world = link_obj.matrix_world.copy()

    logger.debug(
        f"Compound collision created: {merged_obj.name} ({len(typing.cast(bpy.types.Mesh, merged_obj.data).vertices)} vertices)"
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

    # Store original visibility state to restore later
    old_hide_viewport = merged_obj.hide_viewport
    old_hide_render = merged_obj.hide_render

    # Ensure merged_obj is visible and active for mesh simplification
    merged_obj.hide_viewport = False
    merged_obj.hide_render = False

    # Apply mesh simplification to the merged mesh
    # ensures that BMesh processing is robust regardless of viewport mode.
    # We use high-performance BMesh API to avoid Object/Edit mode flickering.
    bm = bmesh.new()
    bm.from_mesh(typing.cast(bpy.types.Mesh, merged_obj.data))

    # Convex Hull operation (native BMesh API)
    bmesh.ops.convex_hull(bm, input=bm.verts)

    # Clear original data and load new hull back to mesh data
    bm.to_mesh(typing.cast(bpy.types.Mesh, merged_obj.data))
    bm.free()
    merged_obj.data.update()

    # Add decimation modifier for live quality adjustment
    lf = typing.cast("LinkPropertyGroup", getattr(link_obj, "linkforge"))
    quality_ratio = lf.collision_quality / 100.0

    decimate_mod = typing.cast(
        bpy.types.DecimateModifier, merged_obj.modifiers.new(name="Decimate", type="DECIMATE")
    )
    decimate_mod.ratio = quality_ratio
    decimate_mod.decimate_type = "COLLAPSE"

    # We do NOT apply the modifier here. Keeping it alive allows the
    # update_collision_quality_realtime callback to provide instant feedback.

    # Restore properties
    if merged_obj:
        merged_obj.name = f"{link_obj.name}_collision"
        merged_obj.rotation_mode = "XYZ"

        # Clear materials from collision mesh (collision doesn't need materials)
        if merged_obj.data and hasattr(merged_obj.data, "materials"):
            merged_obj.data.materials.clear()

    # Strict Alignment Parenting
    # For Mesh (Simplified), we align with the link origin because geometry is already local
    if merged_obj:
        merged_obj.parent = link_obj
        merged_obj.matrix_parent_inverse.identity()
        merged_obj.matrix_local.identity()
        merged_obj.scale = (1, 1, 1)  # Scale baked

        merged_obj.display_type = "WIRE"
        merged_obj.show_in_front = True
        merged_obj.hide_viewport = old_hide_viewport
        merged_obj.hide_render = old_hide_render

        # Persist collision type for UI consistency
        merged_obj["collision_geometry_type"] = "MESH"

        # Ensure it's in the same collection
        for collection in merged_obj.users_collection:
            collection.objects.unlink(merged_obj)
        for collection in link_obj.users_collection:
            if merged_obj.name not in collection.objects:
                collection.objects.link(merged_obj)

    bpy.ops.object.select_all(action="DESELECT")
    link_obj.select_set(True)
    vl = context.view_layer
    if vl:
        vl.objects.active = link_obj

    return merged_obj


def calculate_inertia_for_link(link_obj: bpy.types.Object) -> bool:
    """Calculate inertia tensor for a link.

    Args:
        link_obj: The link object (Empty)

    Returns:
        True if successful, False otherwise
    """
    if (
        not link_obj
        or not hasattr(link_obj, "linkforge")
        or not typing.cast("LinkPropertyGroup", getattr(link_obj, "linkforge")).is_robot_link
    ):
        return False

    lf = typing.cast("LinkPropertyGroup", getattr(link_obj, "linkforge"))

    # Import here to avoid circular dependency
    from ...linkforge_core.models.geometry import Box, Cylinder, Sphere
    from ...linkforge_core.physics import calculate_inertia, calculate_mesh_inertia_from_triangles
    from ..adapters.blender_to_core import extract_mesh_triangles

    # Calculate inertia from child meshes (new architecture: link Empty + children)
    try:
        # Find collision children (preferred for inertia calculation)
        collision_children = [
            child
            for child in link_obj.children
            if child.type == "MESH" and "_collision" in child.name.lower()
        ]

        # If no collision, use visual meshes
        if not collision_children:
            target_children = [
                child
                for child in link_obj.children
                if child.type == "MESH" and "_visual" in child.name.lower()
            ]
        else:
            target_children = collision_children

        if not target_children:
            return False

        # If multiple meshes, we need to combine them (complex)
        # For now, just use the first one or the largest one
        # Ideally we should sum them up properly
        target_obj = target_children[0]

        # Get mass (user input)
        mass = lf.mass
        if mass <= 0:
            mass = 1.0  # Default to 1kg if not set

        # Try to detect primitive type first (faster/cleaner)
        from ..adapters.blender_to_core import detect_primitive_type

        prim_type = detect_primitive_type(target_obj)

        if prim_type:
            # Use primitive calculation
            dims = target_obj.dimensions
            # Scale dimensions by object scale
            # dims are already in world space if applied, but here we want local dimensions
            # Actually dimensions property includes scale.

            # Primitive calculation expects dimensions
            if prim_type == "BOX":
                # Convert mathutils.Vector to core Vector3
                from ...linkforge_core.models.geometry import Vector3

                size = Vector3(dims.x, dims.y, dims.z)
                tensor = calculate_inertia(Box(size=size), mass)
            elif prim_type == "SPHERE":
                radius = max(dims[0], dims[1], dims[2]) / 2.0
                tensor = calculate_inertia(Sphere(radius=radius), mass)
            elif prim_type == "CYLINDER":
                radius = max(dims[0], dims[1]) / 2.0
                length = dims[2]
                tensor = calculate_inertia(Cylinder(radius=radius, length=length), mass)
            else:
                # Fallback to mesh
                res = extract_mesh_triangles(target_obj)
                if res:
                    verts, faces = res
                    tensor = calculate_mesh_inertia_from_triangles(verts, faces, mass)
                else:
                    return False
        else:
            # Use mesh integration
            res = extract_mesh_triangles(target_obj)
            if res:
                verts, faces = res
                tensor = calculate_mesh_inertia_from_triangles(verts, faces, mass)
            else:
                return False

        # Update properties
        lf.inertia_ixx = tensor.ixx
        lf.inertia_iyy = tensor.iyy
        lf.inertia_izz = tensor.izz
        lf.inertia_ixy = tensor.ixy
        lf.inertia_ixz = tensor.ixz
        lf.inertia_iyz = tensor.iyz

        return True

    except Exception as e:
        logger.error(f"Error calculating inertia for {link_obj.name}: {e}", exc_info=True)
        return False


class LINKFORGE_OT_add_empty_link(Operator):
    """Add a new robot link frame (virtual link) at the 3D cursor.

    This operator creates a new Blender Empty object configured as a
    LinkForge Robot Link at the current cursor position, initializing
    standard visual axes and link property defaults.
    """

    bl_idname = "linkforge.add_empty_link"
    bl_label = "Add Empty Link"
    bl_description = "Create a new empty link frame at the 3D cursor position"
    bl_options = {"REGISTER", "UNDO"}

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        from ..preferences import get_addon_prefs

        scene = context.scene
        if not scene:
            return {"CANCELLED"}

        # Initialize default size and prefix
        empty_size = 0.1
        link_name = "base_link"

        addon_prefs = get_addon_prefs(context)
        if addon_prefs:
            empty_size = getattr(addon_prefs, "link_empty_size", empty_size)

        # Create Empty object as link frame
        empty = bpy.data.objects.new(link_name, None)
        empty.empty_display_type = "PLAIN_AXES"
        empty.empty_display_size = empty_size

        # Add to scene
        if context.collection and empty.name not in context.collection.objects:
            context.collection.objects.link(empty)
        elif bpy.context.collection and empty.name not in bpy.context.collection.objects:
            bpy.context.collection.objects.link(empty)
        empty.rotation_mode = "XYZ"

        # Place at 3D cursor
        empty.location = scene.cursor.location.copy()
        # Rotation matched to cursor too for convenience
        empty.rotation_euler = scene.cursor.rotation_euler.copy()

        # Mark as robot link
        typing.cast("LinkPropertyGroup", getattr(empty, "linkforge")).is_robot_link = True

        # Select the new link
        bpy.ops.object.select_all(action="DESELECT")
        empty.select_set(True)
        vl = context.view_layer
        if vl:
            vl.objects.active = empty
        elif bpy.context.view_layer:
            bpy.context.view_layer.objects.active = empty

        # Ensure name is sanitized
        typing.cast("LinkPropertyGroup", getattr(empty, "linkforge")).link_name = empty.name

        clear_stats_cache()
        self.report({"INFO"}, f"Added virtual link frame '{empty.name}' at cursor.")
        return {"FINISHED"}


class LINKFORGE_OT_create_link_from_mesh(Operator):
    """Create a robot link from a selected mesh object.

    This operator converts a standard Blender mesh into a LinkForge Robot
    Link by creating a parent Empty frame and establishing the required
    hierarchy and naming conventions for URDF export.
    """

    bl_idname = "linkforge.create_link_from_mesh"
    bl_label = "Create Link from Mesh"
    bl_description = (
        "Convert selected mesh to a robot link (auto-creates Empty parent and proper naming)"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False

        # Only allow if object is selected
        if not obj.select_get():
            return False

        # Only allow mesh objects
        if obj.type != "MESH":
            return False

        # Don't allow if already a link
        if (
            hasattr(obj, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj, "linkforge")).is_robot_link
        ):
            return False

        # Don't allow if already a visual/collision child of a link
        return bool(
            not (
                obj.parent
                and hasattr(obj.parent, "linkforge")
                and typing.cast("LinkPropertyGroup", getattr(obj.parent, "linkforge")).is_robot_link
                and ("_visual" in obj.name.lower() or "_collision" in obj.name.lower())
            )
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        mesh_obj = context.active_object
        if not mesh_obj:
            return {"CANCELLED"}
        original_name = mesh_obj.name

        # Sanitize name for URDF
        link_name = sanitize_urdf_name(original_name)

        # Ensure we have a valid name
        if not link_name:
            link_name = "link"

        from ..preferences import get_addon_prefs

        # Initialize default size
        empty_size = 0.1

        addon_prefs = get_addon_prefs(context)
        if addon_prefs:
            empty_size = getattr(addon_prefs, "link_empty_size", empty_size)

        # Rename mesh FIRST to free up the name for the Empty
        # This prevents Blender from auto-renaming the Empty to "name.001"
        mesh_obj.name = f"{link_name}_visual"

        # Create Empty object as link frame
        empty = bpy.data.objects.new(link_name, None)
        empty.empty_display_type = "PLAIN_AXES"
        empty.empty_display_size = empty_size
        # Add to scene
        if context.collection and empty.name not in context.collection.objects:
            context.collection.objects.link(empty)

        # Position Empty at mesh world pose precisely
        with context_and_mode_guard(context):
            empty.matrix_world = mesh_obj.matrix_world.copy()
            # Ensure Link is Scale (1,1,1)
            empty.scale = (1, 1, 1)
            empty.rotation_mode = "XYZ"

            # Parent mesh to Empty with STRICT properties for URDF compatibility:
            # 1. Clear existing parent relationships (e.g. from previous imports) to prevent dependency cycles
            mesh_obj.parent_type = "OBJECT"
            mesh_obj.parent_bone = ""

            # Remove Armature modifiers if present (LinkForge links are rigid bodies)
            for mod in mesh_obj.modifiers:
                if mod.type == "ARMATURE":
                    mesh_obj.modifiers.remove(mod)

            # 2. Parent Inverse = Identity (No hidden transforms)
            # 3. Local Location/Rotation = 0 (Visual matches Link frame)
            # 4. Local Scale = Original Mesh Scale (Preserves visual size)

            mesh_obj.parent = empty
            mesh_obj.matrix_parent_inverse.identity()

            # CRITICAL: Force XYZ mode BEFORE zeroing rotations to ensure absolute precision
            mesh_obj.rotation_mode = "XYZ"
            mesh_obj.location = (0, 0, 0)
            mesh_obj.rotation_euler = (0, 0, 0)
            # mesh_obj.scale is already correct (it was S, parent is 1, so S stays S)

            # NOTE: We do NOT use set_parent_keep_transform here because we WANT
            # to effectively "zero out" the local transform relative to the frame we just matched.

            # Mark Empty as robot link
            link_props = typing.cast("LinkPropertyGroup", getattr(empty, "linkforge"))
            link_props.is_robot_link = True
            link_props.link_name = link_name

            # Set default mass
            link_props.mass = 1.0

            # Auto-calculate inertia enabled by default
            link_props.use_auto_inertia = True

            # Select the new link Empty
            bpy.ops.object.select_all(action="DESELECT")
            empty.select_set(True)
            if context.view_layer:
                context.view_layer.objects.active = empty

        self.report(
            {"INFO"},
            f"Created link '{link_name}' with visual mesh. "
            f"Tip: Use 'Generate Collision' to add collision geometry.",
        )
        clear_stats_cache()
        return {"FINISHED"}


class LINKFORGE_OT_generate_collision(Operator):
    """Generate collision geometry from visual geometry for the active link.

    This operator analyzes the visual mesh(es) of the selected link and
    automatically generates simplified collision geometry (primitive or mesh)
    based on the specified collision type.
    """

    bl_idname = "linkforge.generate_collision"
    bl_label = "Generate Collision"
    bl_description = (
        "Auto-generate collision geometry from visual mesh (uses primitives when possible)"
    )
    bl_options = {"REGISTER", "UNDO"}

    collision_type: bpy.props.EnumProperty(  # type: ignore
        name="Collision Type",
        description="Type of collision geometry to generate",
        items=[
            (
                "AUTO",
                "Auto-Detect",
                "Automatically detect primitive shape or use mesh simplification",
            ),
            ("BOX", "Bounding Box", "Use axis-aligned bounding box"),
            ("SPHERE", "Bounding Sphere", "Use bounding sphere"),
            ("CYLINDER", "Bounding Cylinder", "Cylindrical bounding volume around the mesh"),
            (
                "MESH",
                "Mesh (Simplified)",
                "Generate simplified mesh from visual geometry",
            ),
        ],
        default="AUTO",
    )

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False

        # Allow if object is a link
        if (
            hasattr(obj, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj, "linkforge")).is_robot_link
        ):
            return True

        # Allow if object is visual/collision child
        return bool(
            obj.parent
            and hasattr(obj.parent, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, "linkforge")).is_robot_link
            and ("_visual" in obj.name.lower() or "_collision" in obj.name.lower())
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        # Find all robot links
        links = [
            o
            for o in bpy.data.objects
            if hasattr(o, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(o, "linkforge")).is_robot_link
        ]

        if not links:
            self.report({"WARNING"}, "No robot links found")
            return {"CANCELLED"}

        obj = context.active_object
        if not obj:
            return {"CANCELLED"}

        # If selected object is visual or collision child, use parent link
        link_obj = obj
        if (
            obj.parent
            and hasattr(obj.parent, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, "linkforge")).is_robot_link
            and ("_visual" in obj.name.lower() or "_collision" in obj.name.lower())
        ):
            link_obj = obj.parent

        if not link_obj:
            return {"CANCELLED"}

        # Determine collision type
        # Priority: Operator property (if changed in redo) > Link property > Default "AUTO"
        collision_type = self.collision_type
        if collision_type == "AUTO" and hasattr(link_obj, "linkforge"):
            # If operator is AUTO (default), check if link has specific setting
            # Note: Link property also defaults to AUTO, so this works out
            collision_type = typing.cast(
                "LinkPropertyGroup", getattr(link_obj, "linkforge")
            ).collision_type

        # Create collision
        collision_obj = create_collision_for_link(link_obj, collision_type, context)

        if collision_obj is None:
            self.report({"ERROR"}, "Failed to generate collision geometry")
            return {"CANCELLED"}

        # Restore selection (fall back to link if original object was deleted)
        vl = context.view_layer
        try:
            obj.select_set(True)
            if vl:
                vl.objects.active = obj
        except ReferenceError:
            if link_obj:
                link_obj.select_set(True)
                if vl:
                    vl.objects.active = link_obj

        lp = typing.cast("LinkPropertyGroup", getattr(link_obj, "linkforge"))
        self.report({"INFO"}, f"Generated '{collision_type}' collision for '{lp.link_name}'")
        clear_stats_cache()
        return {"FINISHED"}


class LINKFORGE_OT_generate_collision_all(Operator):
    """Generate collision geometry for all robot links in the scene.

    This operator performs a batch collision generation for every object
    marked as a LinkForge Robot Link, using each link's stored collision
    type preferences.
    """

    bl_idname = "linkforge.generate_collision_all"
    bl_label = "Generate All Collisions"
    bl_description = "Generate collision geometry for all robot links in the scene"
    bl_options = {"REGISTER", "UNDO"}

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        scene = context.scene
        if not scene:
            return {"FINISHED"}

        count = 0
        failed = 0

        # Iterate over all objects in scene
        for obj in scene.objects:
            # Check if it's a robot link
            if (
                hasattr(obj, "linkforge")
                and typing.cast("LinkPropertyGroup", getattr(obj, "linkforge")).is_robot_link
            ):
                # Resolve primary mesh to use for detection
                visual_children = [c for c in obj.children if "_visual" in c.name.lower()]
                if visual_children:
                    collision_type = typing.cast(
                        "LinkPropertyGroup", getattr(obj, "linkforge")
                    ).collision_type
                    if create_collision_for_link(obj, collision_type, context):
                        count += 1
                    else:
                        failed += 1

        if failed > 0:
            self.report({"WARNING"}, f"Failed to generate collision for {failed} links")

        clear_stats_cache()
        return {"FINISHED"}


class LINKFORGE_OT_toggle_collision_visibility(Operator):
    """Toggle collision geometry visibility in the 3D viewport.

    This operator recursively toggles the visibility state of all collision
    mesh children for the selected robot link(s).
    """

    bl_idname = "linkforge.toggle_collision_visibility"
    bl_label = "Toggle Collision Visibility"
    bl_description = "Show/hide collision geometry in the viewport"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False

        # Allow if object is a link with collision children
        if (
            hasattr(obj, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj, "linkforge")).is_robot_link
        ):
            collision_children = [c for c in obj.children if "_collision" in c.name.lower()]
            return len(collision_children) > 0

        # Allow if object is visual/collision child
        if (
            obj.parent
            and hasattr(obj.parent, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, "linkforge")).is_robot_link
        ):
            collision_children = [c for c in obj.parent.children if "_collision" in c.name.lower()]
            return len(collision_children) > 0

        return False

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        obj = context.active_object
        if obj is None:
            return {"CANCELLED"}

        # Toggle visibility
        if (
            hasattr(obj, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj, "linkforge")).is_robot_link
        ):
            # It's a link - toggle all its collision children
            for child in obj.children:
                if "_collision" in child.name.lower():
                    child.hide_viewport = not child.hide_viewport
                    child.hide_render = child.hide_viewport  # Keep render state consistent
        else:
            # It's a visual/collision child - toggle its parent's collisions
            if obj.parent and hasattr(obj.parent, "linkforge"):
                for child in obj.parent.children:
                    if "_collision" in child.name.lower():
                        child.hide_viewport = not child.hide_viewport
                        child.hide_render = child.hide_viewport  # Keep render state consistent

        return {"FINISHED"}


class LINKFORGE_OT_calculate_inertia(Operator):
    """Calculate the inertia tensor from link geometry and mass.

    This operator utilizes the Core inertia calculator to derive the
    moment of inertia and center of mass for the selected link based on its
    visual and collision volumes.
    """

    bl_idname = "linkforge.calculate_inertia"
    bl_label = "Calculate Inertia"
    bl_description = "Auto-calculate inertia tensor from object geometry and mass"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False

        # If selected object is a visual/collision child, check parent
        if (
            obj.parent
            and hasattr(obj.parent, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, "linkforge")).is_robot_link
        ):
            return True

        return bool(
            hasattr(obj, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj, "linkforge")).is_robot_link
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        obj = context.active_object
        if not obj:
            return {"CANCELLED"}

        # Resolve target link
        link_obj = obj
        if (
            obj.parent
            and hasattr(obj.parent, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, "linkforge")).is_robot_link
        ):
            link_obj = obj.parent

        if not link_obj:
            return {"CANCELLED"}

        success = calculate_inertia_for_link(link_obj)

        if success:
            link_name = typing.cast("LinkPropertyGroup", getattr(link_obj, "linkforge")).link_name
            self.report({"INFO"}, f"Calculated inertia for '{link_name}'")
            return {"FINISHED"}
        else:
            self.report({"WARNING"}, "Failed to calculate inertia (check geometry/mass)")
            return {"CANCELLED"}


class LINKFORGE_OT_calculate_inertia_all(Operator):
    """Calculate the inertia tensor for all robot links in the scene.

    This operator performs a batch inertia calculation for every LinkForge
    Robot Link, updating their mass and inertial properties based on their
    active geometry.
    """

    bl_idname = "linkforge.calculate_inertia_all"
    bl_label = "Calculate All Inertias"
    bl_description = "Auto-calculate inertia tensor for all robot links in the scene"
    bl_options = {"REGISTER", "UNDO"}

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        scene = context.scene
        if not scene:
            return {"FINISHED"}

        count = 0
        failed = 0

        # Iterate over all objects in scene
        for obj in scene.objects:
            # Check if it's a robot link
            if (
                hasattr(obj, "linkforge")
                and typing.cast("LinkPropertyGroup", getattr(obj, "linkforge")).is_robot_link
            ):
                if calculate_inertia_for_link(obj):
                    count += 1
                else:
                    # Only count as failed if it had visual/collision mesh but failed
                    has_mesh = any(
                        c.type == "MESH"
                        and ("_visual" in c.name.lower() or "_collision" in c.name.lower())
                        for c in obj.children
                    )
                    if has_mesh:
                        failed += 1

        if count > 0:
            self.report({"INFO"}, f"Calculated inertia for {count} links")
        elif failed > 0:
            self.report({"WARNING"}, f"Failed to calculate inertia for {failed} links")
        else:
            self.report({"INFO"}, "No links found needing inertia calculation")

        return {"FINISHED"}


class LINKFORGE_OT_remove_link(Operator):
    """Remove link properties and revert to standard mesh"""

    bl_idname = "linkforge.remove_link"
    bl_label = "Remove Link"
    bl_description = "Revert this link back to a standard mesh (deletes collision geometry)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False

        # Allow if object is a robot link
        if (
            hasattr(obj, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj, "linkforge")).is_robot_link
        ):
            return True

        # Allow if object is a visual/collision child of a link
        return bool(
            obj.parent
            and hasattr(obj.parent, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, "linkforge")).is_robot_link
            and ("_visual" in obj.name.lower() or "_collision" in obj.name.lower())
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        obj = context.active_object
        if not obj:
            return {"CANCELLED"}

        # Resolve link object if child is selected
        link_obj = obj
        if (
            obj.parent
            and hasattr(obj.parent, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, "linkforge")).is_robot_link
            and ("_visual" in obj.name.lower() or "_collision" in obj.name.lower())
        ):
            link_obj = obj.parent

        if not link_obj:
            return {"CANCELLED"}

        lp = typing.cast("LinkPropertyGroup", getattr(link_obj, "linkforge"))
        link_name = lp.link_name or link_obj.name

        # Find visual child
        visual_children = [
            c for c in link_obj.children if "_visual" in c.name.lower() and c.type == "MESH"
        ]

        if not visual_children:
            # VIRTUAL LINK / EMPTY FRAME - Robust handling
            # If no visual mesh, we simply delete the collision children and the frame itself
            collision_children = [c for c in link_obj.children if "_collision" in c.name.lower()]
            for col in collision_children:
                bpy.data.objects.remove(col, do_unlink=True)

            bpy.data.objects.remove(link_obj, do_unlink=True)
            self.report({"INFO"}, f"Removed virtual link frame '{link_name}'")
            return {"FINISHED"}

        # Resolution logic
        with context_and_mode_guard(context):
            # Restore ALL visual objects
            # We unparent them and keep their world transforms
            for visual_obj in visual_children:
                original_world_matrix = visual_obj.matrix_world.copy()
                visual_obj.parent = None
                visual_obj.matrix_world = original_world_matrix

                # Restore name (remove _visual suffix / link prefix)
                if visual_obj.name.endswith("_visual"):
                    visual_obj.name = visual_obj.name[:-7]
                elif visual_obj.name.startswith(f"{link_name}_visual"):
                    visual_obj.name = link_name

            # Delete collision objects
            collision_children = [c for c in link_obj.children if "_collision" in c.name.lower()]
            for col in collision_children:
                bpy.data.objects.remove(col, do_unlink=True)

            # Delete the link empty
            bpy.data.objects.remove(link_obj, do_unlink=True)

            # Force update to ensure name namespace is freed in Blender
            if context.view_layer:
                context.view_layer.update()

            # Select the (first) restored visual object for consistency
            if visual_children and context.view_layer:
                bpy.ops.object.select_all(action="DESELECT")
                visual_children[0].select_set(True)
                context.view_layer.objects.active = visual_children[0]

        msg = f"Removed link '{link_name}'. Restored {len(visual_children)} mesh(es)."
        self.report({"INFO"}, msg)
        clear_stats_cache()
        return {"FINISHED"}


class LINKFORGE_OT_add_material_slot(Operator):
    """Add a material slot to the visual mesh of a link"""

    bl_idname = "linkforge.add_material_slot"
    bl_label = "Add Material Slot"
    bl_description = "Add a material slot to the visual mesh so a material can be assigned"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False

        # Allow if object is a link
        if (
            hasattr(obj, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj, "linkforge")).is_robot_link
        ):
            return True

        # Allow if object is a visual child
        return bool(
            obj.parent
            and hasattr(obj.parent, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, "linkforge")).is_robot_link
            and "_visual" in obj.name.lower()
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        obj = context.active_object

        # If selected object is link, find visual child
        if (
            obj
            and hasattr(obj, "linkforge")
            and typing.cast("LinkPropertyGroup", getattr(obj, "linkforge")).is_robot_link
        ):
            visual_children = [
                c for c in obj.children if "_visual" in c.name.lower() and c.type == "MESH"
            ]
            if not visual_children:
                self.report({"ERROR"}, "No visual mesh found for this link")
                return {"CANCELLED"}
            visual_obj = visual_children[0]
            link_name = typing.cast(typing.Any, obj).linkforge.link_name
        else:
            # Selected object is the visual mesh
            if not obj or not obj.parent:
                return {"CANCELLED"}
            visual_obj = obj
            link_name = typing.cast(typing.Any, obj.parent).linkforge.link_name

        # Append new material slot
        typing.cast(bpy.types.Mesh, visual_obj.data).materials.append(None)

        # Create and assign a default material immediately for better UX
        mat_name = f"{link_name}_material"
        mat = bpy.data.materials.get(mat_name)
        if not mat:
            mat = bpy.data.materials.new(name=mat_name)
            mat.use_nodes = True

        typing.cast(bpy.types.Mesh, visual_obj.data).materials[0] = mat

        self.report({"INFO"}, "Created new material slot with default material")
        return {"FINISHED"}


def update_collision_quality_realtime(
    obj: bpy.types.Object, collision_obj: bpy.types.Object
) -> None:
    """Update collision quality ratio in realtime via Decimate modifier.

    Args:
        obj: The main link object.
        collision_obj: The generated collision object.
    """
    if not collision_obj or not obj:
        return

    # FAST PATH: If we have a Decimate modifier, just update the ratio
    # This provides instant feedback without expensive mesh regeneration
    lf = typing.cast("LinkPropertyGroup", getattr(obj, "linkforge"))
    quality_ratio = lf.collision_quality / 100.0

    decimate_mod = next((m for m in collision_obj.modifiers if m.type == "DECIMATE"), None)
    if decimate_mod and isinstance(decimate_mod, bpy.types.DecimateModifier):
        decimate_mod.ratio = quality_ratio
    elif collision_obj.type == "MESH":
        # FALLBACK IMPROVEMENT: If modifier is missing but object exists, try adding it first
        # This is much faster than full _regenerate_ and avoids one potential "jump".
        decimate_mod = typing.cast(
            bpy.types.DecimateModifier,
            collision_obj.modifiers.new(name="Decimate", type="DECIMATE"),
        )
        decimate_mod.ratio = quality_ratio
        decimate_mod.decimate_type = "COLLAPSE"
    else:
        # ABSOLUTE FALLBACK: schedule a full regeneration for primitives or missing objects.
        schedule_collision_preview_update(obj)


# Registration
classes = [
    LINKFORGE_OT_add_empty_link,
    LINKFORGE_OT_create_link_from_mesh,
    LINKFORGE_OT_generate_collision,
    LINKFORGE_OT_generate_collision_all,
    LINKFORGE_OT_toggle_collision_visibility,
    LINKFORGE_OT_calculate_inertia,
    LINKFORGE_OT_calculate_inertia_all,
    LINKFORGE_OT_remove_link,
    LINKFORGE_OT_add_material_slot,
]


def register() -> None:
    """Register operators."""
    for cls in classes:
        with contextlib.suppress(ValueError):
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister operators."""
    for cls in reversed(classes):
        with contextlib.suppress(RuntimeError):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
