"""Operators for managing robot links."""

from __future__ import annotations

import bpy
import mathutils
from bpy.types import Operator

from ...core.logging_config import get_logger
from ..properties.link_props import sanitize_urdf_name

logger = get_logger(__name__)

# Global state for debounced collision preview updates
_preview_update_timer = None
_preview_pending_object = None

# Debounce delay for collision preview updates (in seconds)
COLLISION_PREVIEW_DEBOUNCE_DELAY = 0.3


def schedule_collision_preview_update(obj):
    """Schedule a debounced collision preview update.

    This prevents excessive regeneration during slider interaction by
    waiting 0.3 seconds after the last change before updating.
    """
    global _preview_update_timer, _preview_pending_object

    # Cancel existing timer
    if _preview_update_timer is not None:
        try:
            bpy.app.timers.unregister(_preview_update_timer)
        except Exception:
            pass

    # Store object reference
    _preview_pending_object = obj

    # Register new timer with debounce delay
    _preview_update_timer = bpy.app.timers.register(
        execute_collision_preview_update, first_interval=COLLISION_PREVIEW_DEBOUNCE_DELAY
    )


def execute_collision_preview_update():
    """Execute the actual collision mesh update after debounce delay."""
    global _preview_update_timer, _preview_pending_object

    if _preview_pending_object is None:
        return None

    obj = _preview_pending_object
    _preview_pending_object = None
    _preview_update_timer = None

    # Check if object still exists
    if obj.name not in bpy.data.objects:
        return None

    # Find collision object
    collision_obj = next((c for c in obj.children if "_collision" in c.name.lower()), None)
    if collision_obj is None:
        return None

    # Check if it's a primitive (don't regenerate primitives)
    from ..utils.converters import detect_primitive_type

    primitive_type = detect_primitive_type(collision_obj)
    if primitive_type is not None:
        return None

    # Regenerate collision mesh with new quality
    regenerate_collision_mesh(obj, collision_obj)

    return None  # Don't repeat timer


def regenerate_collision_mesh(link_obj, collision_obj):
    """Regenerate collision mesh based on current quality setting.

    This is called during live preview updates to show the user
    how the collision mesh will look with the current quality setting.
    """

    # Get quality from link properties
    quality = link_obj.linkforge.collision_quality

    # Find ALL visual meshes (for compound collision)
    visual_children = [
        c for c in link_obj.children if "_visual" in c.name.lower() and c.type == "MESH"
    ]
    if not visual_children:
        return

    link_name = link_obj.linkforge.link_name or link_obj.name

    # Store collision object's current state
    old_hide_viewport = collision_obj.hide_viewport
    old_hide_render = collision_obj.hide_render
    old_location = collision_obj.location.copy()
    old_rotation = collision_obj.rotation_euler.copy()

    # Remove old collision
    bpy.data.objects.remove(collision_obj, do_unlink=True)

    # Create new compound convex hull collision from ALL visuals
    merged_obj = _merge_visual_meshes(visual_children, bpy.context)

    if merged_obj is None:
        return

    # Clear parent temporarily
    merged_obj.parent = None
    merged_obj.location = old_location
    merged_obj.rotation_euler = old_rotation

    # Apply convex hull to merged mesh
    bpy.context.view_layer.objects.active = merged_obj
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.convex_hull()
    bpy.ops.object.mode_set(mode="OBJECT")

    # Apply decimation based on quality
    if quality < 100.0:
        # Add decimate modifier
        mod = merged_obj.modifiers.new(name="Decimate", type="DECIMATE")
        mod.ratio = quality / 100.0
        mod.use_collapse_triangulate = True

        # Apply modifier
        bpy.context.view_layer.objects.active = merged_obj
        bpy.ops.object.modifier_apply(modifier=mod.name)

    # Clear materials (collision doesn't need materials)
    if merged_obj.data and hasattr(merged_obj.data, "materials"):
        merged_obj.data.materials.clear()

    # Restore properties
    merged_obj.name = f"{link_name}_collision"

    # Strict Alignment Parenting
    merged_obj.parent = link_obj
    merged_obj.matrix_parent_inverse.identity()
    merged_obj.location = visual_children[0].location
    merged_obj.rotation_euler = visual_children[0].rotation_euler
    merged_obj.scale = (1, 1, 1)  # Scale baked

    merged_obj.display_type = "WIRE"
    merged_obj.show_in_front = True
    merged_obj.hide_viewport = old_hide_viewport
    merged_obj.hide_render = old_hide_render

    # Persist collision type for UI consistency
    merged_obj["collision_geometry_type"] = "CONVEX_HULL"

    # Ensure it's in the same collection
    for collection in merged_obj.users_collection:
        collection.objects.unlink(merged_obj)
    for collection in link_obj.users_collection:
        if merged_obj.name not in collection.objects:
            collection.objects.link(merged_obj)

    # IMPORTANT: Restore selection to the parent link
    # This ensures subsequent slider changes work correctly
    bpy.ops.object.select_all(action="DESELECT")
    link_obj.select_set(True)
    bpy.context.view_layer.objects.active = link_obj


def create_collision_for_link(link_obj, collision_type, context):
    """Create collision geometry for a link.

    For links with multiple visual children, this creates a compound collision
    by merging all visuals into a single collision mesh (industry best practice).

    Args:
        link_obj: The link object (Empty)
        collision_type: Type of collision ("AUTO", "BOX", "SPHERE", "CYLINDER", "CONVEX_HULL")
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

    link_name = link_obj.linkforge.link_name or link_obj.name

    # Remove existing collision objects to prevent duplicates
    existing_collisions = [c for c in link_obj.children if "_collision" in c.name.lower()]
    for col in existing_collisions:
        bpy.data.objects.remove(col, do_unlink=True)

    # Import here to avoid circular dependency
    from ..utils.converters import detect_primitive_type

    # Determine collision type
    if collision_type == "AUTO":
        # For multiple visuals, always use convex hull (compound collision)
        if len(visual_children) > 1:
            collision_type = "CONVEX_HULL"
        else:
            # Single visual - try to detect primitive
            detected = detect_primitive_type(visual_children[0])
            collision_type = detected if detected else "CONVEX_HULL"

    # Generate collision mesh
    if collision_type in ("BOX", "SPHERE", "CYLINDER"):
        # Primitives only make sense for single visuals
        collision_obj = _create_primitive_collision(
            visual_children[0], collision_type, link_name, context
        )
        reference_visual = visual_children[0]
    else:  # CONVEX_HULL
        # Merge ALL visuals into compound collision
        collision_obj = _create_convex_hull_collision_compound(visual_children, link_name, context)
        # Use first visual as reference for positioning
        reference_visual = visual_children[0]

    if collision_obj is None:
        return None

    # Parent to link using Strict Alignment
    # We want Collision to exactly match Visual in Local space relative to Link
    # This ensures URDF exporter (which reads Local) gets correct relative values

    collision_obj.parent = link_obj
    collision_obj.matrix_parent_inverse.identity()

    # Copy local transforms from reference visual to collision
    # (Since we used Identity inverse, 'Location' property aligns perfectly to Parent info)
    collision_obj.location = reference_visual.location
    collision_obj.rotation_euler = reference_visual.rotation_euler
    collision_obj.scale = (1, 1, 1)  # Scale was baked into geometry

    # IMPORTANT: Ensure collision is actually a child in the collection hierarchy
    # If collision was created in a different collection, it won't show as child in outliner
    # Make sure collision is in the same collection as the link
    for collection in collision_obj.users_collection:
        collection.objects.unlink(collision_obj)
    for collection in link_obj.users_collection:
        if collision_obj.name not in collection.objects:
            collection.objects.link(collision_obj)

    # Clear materials from collision mesh (collision doesn't need materials)
    # Materials may be inherited from duplicated visual mesh
    if collision_obj.data and hasattr(collision_obj.data, "materials"):
        collision_obj.data.materials.clear()

    # Make collision visually distinct (wireframe + semi-transparent)
    collision_obj.display_type = "WIRE"
    collision_obj.show_in_front = True  # X-ray mode

    # Store the collision type for accurate export (prevents auto-detection failures)
    collision_obj["collision_geometry_type"] = collision_type

    # Hide by default (clean viewport)
    collision_obj.hide_viewport = True
    collision_obj.hide_render = True

    return collision_obj


def _create_primitive_collision(visual_obj, prim_type, link_name, context):
    """Create primitive collision geometry."""
    # Get visual's dimensions and parent scale
    # visual_obj.dimensions gives world-space size (includes parent scale)
    # We need to create collision at LOCAL size (divide by parent scale)
    # because it will be parented to the link and inherit parent scale
    world_dims = visual_obj.dimensions.copy()

    # Get parent scale to convert world dims to local dims
    if visual_obj.parent:
        parent_scale = visual_obj.parent.scale
        # Divide world dimensions by parent scale to get local dimensions
        local_dims = mathutils.Vector(
            (
                world_dims.x / parent_scale.x,
                world_dims.y / parent_scale.y,
                world_dims.z / parent_scale.z,
            )
        )
    else:
        local_dims = world_dims

    if prim_type == "BOX":
        # Create cube at local size (will be scaled by parent when parented)
        bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0, 0, 0))
        collision_obj = context.active_object
        collision_obj.dimensions = local_dims

    elif prim_type == "SPHERE":
        # Create sphere at local size
        radius = max(local_dims) / 2.0
        bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=(0, 0, 0))
        collision_obj = context.active_object

    elif prim_type == "CYLINDER":
        # Create cylinder at local size
        radius = max(local_dims.x, local_dims.y) / 2.0
        depth = local_dims.z
        bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=depth, location=(0, 0, 0))
        collision_obj = context.active_object

    else:
        return None

    # Apply scale to bake dimensions into geometry
    # This ensures collision has scale=1.0 with geometry at local size
    # When parented to link, it will inherit parent's scale to reach correct world size
    bpy.context.view_layer.objects.active = collision_obj
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    # Name it
    collision_obj.name = f"{link_name}_collision"

    # NOTE: location/rotation are set by caller using Strict Alignment relative to Visual

    return collision_obj


def _merge_visual_meshes(visual_objects, context):
    """Merge multiple visual meshes into a single temporary mesh for compound collision.

    This creates a compound mesh that represents all visual geometry,
    which is then used to generate a single accurate collision hull.
    This follows industry best practices (ROS, Gazebo, MoveIt).

    Args:
        visual_objects: List of visual mesh objects to merge
        context: Blender context

    Returns:
        Merged mesh object (temporary, caller must clean up)
    """
    if not visual_objects:
        return None

    # Log collision generation for debugging and user feedback
    logger.info(f"Compound collision: merging {len(visual_objects)} visual mesh(es)")
    for i, vis in enumerate(visual_objects):
        logger.info(f"  [{i + 1}] {vis.name}")

    # Duplicate all visual objects
    duplicates = []
    for visual_obj in visual_objects:
        bpy.ops.object.select_all(action="DESELECT")
        visual_obj.select_set(True)
        context.view_layer.objects.active = visual_obj
        bpy.ops.object.duplicate()
        dup = context.active_object

        # Clear parent and apply transforms to bake world position into geometry
        dup.parent = None
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

        duplicates.append(dup)

    # If only one visual, return it directly
    if len(duplicates) == 1:
        return duplicates[0]

    # Multiple visuals - join them into single mesh
    bpy.ops.object.select_all(action="DESELECT")
    for dup in duplicates:
        dup.select_set(True)
    context.view_layer.objects.active = duplicates[0]

    # Join into single mesh
    bpy.ops.object.join()
    merged_obj = context.active_object
    logger.info(
        f"Compound collision created: {merged_obj.name} ({len(merged_obj.data.vertices)} vertices)"
    )

    return merged_obj


def _create_convex_hull_collision_compound(visual_objects, link_name, context):
    """Create compound convex hull collision from multiple visual meshes.

    This merges all visual children into a single collision mesh, following
    industry best practices (ROS, Gazebo, MoveIt).

    Args:
        visual_objects: List of visual mesh objects to merge
        link_name: Name of the link
        context: Blender context

    Returns:
        Collision object with compound convex hull
    """
    # Merge all visual meshes into compound mesh
    merged_obj = _merge_visual_meshes(visual_objects, context)

    if merged_obj is None:
        return None

    # Apply convex hull to the merged mesh
    bpy.context.view_layer.objects.active = merged_obj
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.convex_hull()
    bpy.ops.object.mode_set(mode="OBJECT")

    # Name it
    merged_obj.name = f"{link_name}_collision"

    return merged_obj


def calculate_inertia_for_link(link_obj):
    """Calculate inertia tensor for a link.

    Args:
        link_obj: The link object (Empty)

    Returns:
        True if successful, False otherwise
    """
    props = link_obj.linkforge

    # Import here to avoid circular dependency
    from ...core.physics import calculate_inertia, calculate_mesh_inertia_from_triangles
    from ..utils.converters import extract_mesh_triangles

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
        mass = props.mass
        if mass <= 0:
            mass = 1.0  # Default to 1kg if not set

        # Try to detect primitive type first (faster/cleaner)
        from ..utils.converters import detect_primitive_type

        prim_type = detect_primitive_type(target_obj)

        if prim_type:
            # Use primitive calculation
            dims = target_obj.dimensions
            # Scale dimensions by object scale
            # dims are already in world space if applied, but here we want local dimensions
            # Actually dimensions property includes scale.

            # Primitive calculation expects dimensions
            if prim_type == "BOX":
                tensor = calculate_inertia("box", mass, size=dims)
            elif prim_type == "SPHERE":
                radius = max(dims) / 2.0
                tensor = calculate_inertia("sphere", mass, radius=radius)
            elif prim_type == "CYLINDER":
                radius = max(dims.x, dims.y) / 2.0
                length = dims.z
                tensor = calculate_inertia("cylinder", mass, radius=radius, length=length)
            else:
                # Fallback to mesh
                triangles = extract_mesh_triangles(target_obj)
                tensor = calculate_mesh_inertia_from_triangles(triangles, mass)
        else:
            # Use mesh integration
            triangles = extract_mesh_triangles(target_obj)
            tensor = calculate_mesh_inertia_from_triangles(triangles, mass)

        # Update properties
        props.inertia_ixx = tensor.ixx
        props.inertia_iyy = tensor.iyy
        props.inertia_izz = tensor.izz
        props.inertia_ixy = tensor.ixy
        props.inertia_ixz = tensor.ixz
        props.inertia_iyz = tensor.iyz

        return True

    except Exception as e:
        logger.error(f"Error calculating inertia for {link_obj.name}: {e}", exc_info=True)
        return False


class LINKFORGE_OT_create_link_from_mesh(Operator):
    """Create a robot link from selected mesh object"""

    bl_idname = "linkforge.create_link_from_mesh"
    bl_label = "Create Link"
    bl_description = (
        "Convert selected mesh to a robot link (auto-creates Empty parent and proper naming)"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
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
        if obj.linkforge.is_robot_link:
            return False

        # Don't allow if already a visual/collision child of a link
        if obj.parent and hasattr(obj.parent, "linkforge"):
            if obj.parent.linkforge.is_robot_link:
                if "_visual" in obj.name.lower() or "_collision" in obj.name.lower():
                    return False

        return True

    def execute(self, context):
        """Execute the operator."""
        mesh_obj = context.active_object
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
        context.collection.objects.link(empty)

        # Position Empty at mesh origin
        empty.location = mesh_obj.location.copy()
        empty.rotation_euler = mesh_obj.rotation_euler.copy()
        # Ensure Link is Scale (1,1,1)
        empty.scale = (1, 1, 1)

        # Parent mesh to Empty with STRICT properties for URDF compatibility:
        # 1. Parent Inverse = Identity (No hidden transforms)
        # 2. Local Location/Rotation = 0 (Visual matches Link frame)
        # 3. Local Scale = Original Mesh Scale (Preserves visual size)

        mesh_obj.parent = empty
        mesh_obj.matrix_parent_inverse.identity()

        mesh_obj.location = (0, 0, 0)
        mesh_obj.rotation_euler = (0, 0, 0)
        # mesh_obj.scale is already correct (it was S, parent is 1, so S stays S)

        # NOTE: We do NOT use set_parent_keep_transform here because we WANT
        # to effectively "zero out" the local transform relative to the frame we just matched.

        # Mark Empty as robot link
        empty.linkforge.is_robot_link = True
        empty.linkforge.link_name = link_name

        # Set default mass
        empty.linkforge.mass = 1.0

        # Auto-calculate inertia enabled by default
        empty.linkforge.use_auto_inertia = True

        # Select the new link Empty
        bpy.ops.object.select_all(action="DESELECT")
        empty.select_set(True)
        context.view_layer.objects.active = empty

        self.report(
            {"INFO"},
            f"Created link '{link_name}' with visual mesh. "
            f"Tip: Use 'Generate Collision' to add collision geometry.",
        )
        return {"FINISHED"}


class LINKFORGE_OT_generate_collision(Operator):
    """Generate collision geometry from visual geometry"""

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
            ("AUTO", "Auto-Detect", "Automatically detect primitive shape or use convex hull"),
            ("BOX", "Bounding Box", "Use axis-aligned bounding box"),
            ("SPHERE", "Bounding Sphere", "Use bounding sphere"),
            ("CYLINDER", "Bounding Cylinder", "Use bounding cylinder"),
            ("CONVEX_HULL", "Convex Hull", "Generate convex hull from mesh"),
        ],
        default="AUTO",
    )

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False

        # Allow if object is a link with visual children
        if obj.linkforge.is_robot_link:
            visual_children = [c for c in obj.children if "_visual" in c.name.lower()]
            return len(visual_children) > 0

        # Allow if object is a visual child (will use parent link)
        if obj.parent and hasattr(obj.parent, "linkforge"):
            if obj.parent.linkforge.is_robot_link:
                # Allow if visual OR collision child (better UX for regeneration)
                if "_visual" in obj.name.lower() or "_collision" in obj.name.lower():
                    return True

        return False

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object

        # If selected object is visual or collision child, use parent link
        link_obj = obj
        if obj.parent and hasattr(obj.parent, "linkforge"):
            if obj.parent.linkforge.is_robot_link:
                if "_visual" in obj.name.lower() or "_collision" in obj.name.lower():
                    link_obj = obj.parent

        # Determine collision type
        # Priority: Operator property (if changed in redo) > Link property > Default "AUTO"
        collision_type = self.collision_type
        if collision_type == "AUTO" and hasattr(link_obj.linkforge, "collision_type"):
            # If operator is AUTO (default), check if link has specific setting
            # Note: Link property also defaults to AUTO, so this works out
            collision_type = link_obj.linkforge.collision_type

        # Create collision
        collision_obj = create_collision_for_link(link_obj, collision_type, context)

        if collision_obj is None:
            self.report({"ERROR"}, "Failed to generate collision geometry")
            return {"CANCELLED"}

        # Select the link (not collision)
        bpy.ops.object.select_all(action="DESELECT")
        link_obj.select_set(True)
        context.view_layer.objects.active = link_obj

        link_name = link_obj.linkforge.link_name or link_obj.name
        self.report(
            {"INFO"},
            f"Generated collision for '{link_name}' (hidden by default)",
        )
        return {"FINISHED"}


class LINKFORGE_OT_generate_collision_all(Operator):
    """Generate collision geometry for ALL links in the scene"""

    bl_idname = "linkforge.generate_collision_all"
    bl_label = "Generate All Collisions"
    bl_description = "Generate collision geometry for all robot links in the scene"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        """Execute the operator."""
        count = 0
        failed = 0

        # Iterate over all objects in scene
        for obj in context.scene.objects:
            # Check if it's a robot link
            if hasattr(obj, "linkforge") and obj.linkforge.is_robot_link:
                # Regenerate collision to ensure consistency
                # (Previously checked has_collision, but we always regenerate now)

                # Use link's preferred type
                collision_type = obj.linkforge.collision_type

                result = create_collision_for_link(obj, collision_type, context)
                if result:
                    count += 1
                else:
                    # Only count as failed if it had visual mesh but failed
                    visual_children = [c for c in obj.children if "_visual" in c.name.lower()]
                    if visual_children:
                        failed += 1

        if count > 0:
            self.report({"INFO"}, f"Generated collision for {count} links")
        elif failed > 0:
            self.report({"WARNING"}, f"Failed to generate collision for {failed} links")
        else:
            self.report({"INFO"}, "No links found needing collision generation")

        return {"FINISHED"}


class LINKFORGE_OT_toggle_collision_visibility(Operator):
    """Toggle collision geometry visibility in viewport"""

    bl_idname = "linkforge.toggle_collision_visibility"
    bl_label = "Toggle Collision Visibility"
    bl_description = "Show/hide collision geometry in the viewport"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False

        # Allow if object is a link with collision children
        if obj.linkforge.is_robot_link:
            collision_children = [c for c in obj.children if "_collision" in c.name.lower()]
            return len(collision_children) > 0

        # Allow if object is visual/collision child
        if obj.parent and hasattr(obj.parent, "linkforge"):
            if obj.parent.linkforge.is_robot_link:
                collision_children = [
                    c for c in obj.parent.children if "_collision" in c.name.lower()
                ]
                return len(collision_children) > 0

        return False

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object

        # Get the link object
        link_obj = obj
        if obj.parent and hasattr(obj.parent, "linkforge"):
            if obj.parent.linkforge.is_robot_link:
                link_obj = obj.parent

        # Find all collision children
        collision_children = [c for c in link_obj.children if "_collision" in c.name.lower()]

        if not collision_children:
            return {"CANCELLED"}

        # Toggle visibility (check first collision's state)
        new_state = not collision_children[0].hide_viewport

        for collision in collision_children:
            collision.hide_viewport = new_state
            collision.hide_render = new_state

        # No report needed - visibility change is immediately visible to user
        return {"FINISHED"}


class LINKFORGE_OT_calculate_inertia(Operator):
    """Calculate inertia tensor from object geometry"""

    bl_idname = "linkforge.calculate_inertia"
    bl_label = "Calculate Inertia"
    bl_description = "Auto-calculate inertia tensor from object geometry and mass"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False

        # If selected object is a visual/collision child, check parent
        if obj.parent and hasattr(obj.parent, "linkforge"):
            if obj.parent.linkforge.is_robot_link and not obj.linkforge.is_robot_link:
                return True

        return obj.linkforge.is_robot_link

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object

        # If selected object is a visual/collision child, use parent link
        if obj.parent and hasattr(obj.parent, "linkforge"):
            if obj.parent.linkforge.is_robot_link and not obj.linkforge.is_robot_link:
                obj = obj.parent

        if calculate_inertia_for_link(obj):
            self.report({"INFO"}, f"Calculated inertia for '{obj.name}'")
            return {"FINISHED"}
        else:
            self.report({"ERROR"}, "Failed to calculate inertia")
            return {"CANCELLED"}


class LINKFORGE_OT_calculate_inertia_all(Operator):
    """Calculate inertia for ALL links in the scene"""

    bl_idname = "linkforge.calculate_inertia_all"
    bl_label = "Calculate All Inertias"
    bl_description = "Auto-calculate inertia tensor for all robot links in the scene"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        """Execute the operator."""
        count = 0
        failed = 0

        # Iterate over all objects in scene
        for obj in context.scene.objects:
            # Check if it's a robot link
            if hasattr(obj, "linkforge") and obj.linkforge.is_robot_link:
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
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False

        # Allow if object is a robot link
        if obj.linkforge.is_robot_link:
            return True

        # Allow if object is a visual/collision child of a link
        if obj.parent and hasattr(obj.parent, "linkforge"):
            if obj.parent.linkforge.is_robot_link:
                if "_visual" in obj.name.lower() or "_collision" in obj.name.lower():
                    return True

        return False

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object

        # Resolve link object if child is selected
        link_obj = obj
        if obj.parent and hasattr(obj.parent, "linkforge"):
            if obj.parent.linkforge.is_robot_link:
                if "_visual" in obj.name.lower() or "_collision" in obj.name.lower():
                    link_obj = obj.parent

        link_name = link_obj.linkforge.link_name or link_obj.name

        # Find visual child
        visual_children = [
            c for c in link_obj.children if "_visual" in c.name.lower() and c.type == "MESH"
        ]

        if not visual_children:
            self.report({"ERROR"}, "No visual mesh found to restore")
            return {"CANCELLED"}

        visual_obj = visual_children[0]

        # Store world transform of visual object
        world_matrix = visual_obj.matrix_world.copy()

        # Unparent visual object
        visual_obj.parent = None
        visual_obj.matrix_world = world_matrix

        # Delete collision objects
        collision_children = [c for c in link_obj.children if "_collision" in c.name.lower()]
        for col in collision_children:
            bpy.data.objects.remove(col, do_unlink=True)

        # Delete the link empty FIRST to free up the name
        bpy.data.objects.remove(link_obj, do_unlink=True)

        # Force update to ensure name is freed
        if context.view_layer:
            context.view_layer.update()

        # Restore name (remove _visual suffix)
        # Now that Empty is gone, we can safely use the original name
        if visual_obj.name.endswith("_visual"):
            visual_obj.name = visual_obj.name[:-7]  # Remove last 7 chars "_visual"
        elif visual_obj.name.startswith(f"{link_name}_visual"):
            visual_obj.name = link_name

        # Select the restored visual object
        bpy.ops.object.select_all(action="DESELECT")
        visual_obj.select_set(True)
        context.view_layer.objects.active = visual_obj

        self.report({"INFO"}, f"Removed link properties. Restored mesh: '{visual_obj.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_add_material_slot(Operator):
    """Add a material slot to the visual mesh of a link"""

    bl_idname = "linkforge.add_material_slot"
    bl_label = "Add Material Slot"
    bl_description = "Add a material slot to the visual mesh so a material can be assigned"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False

        # Allow if object is a link
        if hasattr(obj, "linkforge") and obj.linkforge.is_robot_link:
            return True

        # Allow if object is a visual child
        if obj.parent and hasattr(obj.parent, "linkforge"):
            if obj.parent.linkforge.is_robot_link and "_visual" in obj.name.lower():
                return True

        return False

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object

        # If selected object is link, find visual child
        if obj.linkforge.is_robot_link:
            visual_children = [
                c for c in obj.children if "_visual" in c.name.lower() and c.type == "MESH"
            ]
            if not visual_children:
                self.report({"ERROR"}, "No visual mesh found for this link")
                return {"CANCELLED"}
            visual_obj = visual_children[0]
            link_name = obj.linkforge.link_name
        else:
            # Selected object is the visual mesh
            visual_obj = obj
            link_name = obj.parent.linkforge.link_name

        # Append new material slot
        visual_obj.data.materials.append(None)

        # Create and assign a default material immediately for better UX
        mat_name = f"{link_name}_material"
        mat = bpy.data.materials.get(mat_name)
        if not mat:
            mat = bpy.data.materials.new(name=mat_name)
            mat.use_nodes = True

        visual_obj.data.materials[0] = mat

        self.report({"INFO"}, "Created new material slot with default material")
        return {"FINISHED"}


# Registration
classes = [
    LINKFORGE_OT_create_link_from_mesh,
    LINKFORGE_OT_generate_collision,
    LINKFORGE_OT_generate_collision_all,
    LINKFORGE_OT_toggle_collision_visibility,
    LINKFORGE_OT_calculate_inertia,
    LINKFORGE_OT_calculate_inertia_all,
    LINKFORGE_OT_remove_link,
    LINKFORGE_OT_add_material_slot,
]


def register():
    """Register operators."""
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    """Unregister operators."""
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
