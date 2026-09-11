"""Operators for managing robot links."""

from __future__ import annotations

import contextlib
import time
import typing

import bpy
from bpy.types import Context, Operator

from ..constants import (
    DEFAULT_LINK_GIZMO_SIZE,
    GEOM_AUTO,
    PROP_LINK,
    SUFFIX_COLLISION,
    SUFFIX_VISUAL,
)
from ..core import InertiaTensor, Vector3, get_logger
from ..core.constants import (
    GEOM_BOX,
    GEOM_CYLINDER,
    GEOM_MESH,
    GEOM_SPHERE,
)
from ..logic.collision_builder import (
    _create_mesh_collision_compound as _create_mesh_collision_compound,
)
from ..logic.collision_builder import (
    _create_primitive_collision as _create_primitive_collision,
)
from ..logic.collision_builder import (
    _merge_visual_meshes as _merge_visual_meshes,
)
from ..logic.collision_builder import (
    create_collision_for_link as create_collision_for_link,
)
from ..logic.collision_builder import (
    regenerate_collision_mesh as regenerate_collision_mesh,
)
from ..properties.geom_props import PROP_GEOM
from ..properties.link_props import LinkPropertyGroup, sanitize_name
from ..utils.decorators import OperatorReturn, safe_execute
from ..utils.mode_guard import context_and_mode_guard
from ..utils.scene_utils import clear_stats_cache

logger = get_logger(__name__)

# Debounce state for real-time collision preview updates
_preview_pending_object: bpy.types.Object | None = None
_preview_last_request_time: float = 0.0
COLLISION_PREVIEW_DEBOUNCE_DELAY: float = 0.3


def schedule_collision_preview_update(obj: bpy.types.Object) -> None:
    """Debounce timer callback to update collision preview.

    Registers `execute_collision_preview_update` as a timer if not already registered.
    """
    global _preview_pending_object, _preview_last_request_time
    _preview_pending_object = obj
    _preview_last_request_time = time.time()

    if not bpy.app.timers.is_registered(execute_collision_preview_update):
        bpy.app.timers.register(
            execute_collision_preview_update,
            first_interval=COLLISION_PREVIEW_DEBOUNCE_DELAY,
        )


def execute_collision_preview_update() -> float | None:
    """Timer callback for collision preview debounce.

    Returns:
        Remaining delay to wait, or None to unregister the timer.
    """
    global _preview_pending_object, _preview_last_request_time
    if _preview_pending_object is None:
        return None

    elapsed = time.time() - _preview_last_request_time
    if elapsed < COLLISION_PREVIEW_DEBOUNCE_DELAY:
        return max(COLLISION_PREVIEW_DEBOUNCE_DELAY - elapsed, 0.01)

    obj = _preview_pending_object
    _preview_pending_object = None

    if not obj or not hasattr(obj, PROP_LINK):
        return None

    try:
        from ..adapters.geometry_extractor import detect_primitive_type

        collision_type = detect_primitive_type(obj)
        regenerate_collision_mesh(obj, str(collision_type), bpy.context)
    except Exception as e:
        logger.error(f"Error during debounce collision preview update: {e}")

    return None


def calculate_inertia_for_link(link_obj: bpy.types.Object) -> bool:
    """Calculate inertia tensor for a link.

    Args:
        link_obj: The link object (Empty)

    Returns:
        True if successful, False otherwise
    """
    if (
        not link_obj
        or not hasattr(link_obj, PROP_LINK)
        or not typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK)).is_robot_link
    ):
        return False

    lf = typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK))

    # Import here to avoid circular dependency
    from ..adapters.blender_to_core import extract_mesh_triangles
    from ..core import Box, Cylinder, Sphere, calculate_inertia, validate_mesh_topology
    from ..core.physics import calculate_mesh_inertia_from_triangles

    # Calculate inertia from child meshes (new architecture: link Empty + children)
    try:
        # Find collision children (preferred for inertia calculation)
        collision_children = [
            child
            for child in link_obj.children
            if child.type == "MESH" and SUFFIX_COLLISION in child.name.lower()
        ]

        # If no collision, use visual meshes
        if not collision_children:
            target_children = [
                child
                for child in link_obj.children
                if child.type == "MESH" and SUFFIX_VISUAL in child.name.lower()
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

        geom_props = getattr(target_obj, PROP_GEOM, None)
        if geom_props and getattr(geom_props, "geometry_type", None) in (
            GEOM_BOX,
            GEOM_CYLINDER,
            GEOM_SPHERE,
        ):
            prim_type = geom_props.geometry_type
        else:
            from ..adapters.blender_to_core import detect_primitive_type

            prim_type = detect_primitive_type(target_obj)

        tensor = None

        if prim_type and prim_type != "mesh":
            # Use primitive calculation
            dims = target_obj.dimensions

            # Primitive calculation expects dimensions
            if prim_type == GEOM_BOX:
                # Convert mathutils.Vector to core Vector3
                size = Vector3(dims.x, dims.y, dims.z)
                tensor = calculate_inertia(Box(size=size), mass)
            elif prim_type == GEOM_SPHERE:
                radius = max(dims[0], dims[1], dims[2]) / 2.0
                tensor = calculate_inertia(Sphere(radius=radius), mass)
            elif prim_type == GEOM_CYLINDER:
                radius = max(dims[0], dims[1]) / 2.0
                length = dims[2]
                tensor = calculate_inertia(Cylinder(radius=radius, length=length), mass)
        else:
            # Mesh fallback
            res = extract_mesh_triangles(target_obj, as_numpy=False)
            if res:
                verts, faces = res

                # Ensure mesh is not empty to avoid physics engine crashes
                if not verts or not faces:
                    logger.warning(
                        f"Skipping inertia calculation for link '{link_obj.name}': "
                        f"Visual object '{target_obj.name}' has no geometry (mesh is empty)."
                    )
                    return False

                # Mandatory topology validation for mesh inertia
                validate_mesh_topology(verts, faces, name=target_obj.name)
                tensor = calculate_mesh_inertia_from_triangles(verts, faces, mass)

        if tensor is not None:
            # Final validation for type-checker
            t: InertiaTensor = tensor
            # Update Link properties (explicitly cast to float to avoid Blender property set errors)
            lf.inertia_ixx = float(t.ixx)
            lf.inertia_iyy = float(t.iyy)
            lf.inertia_izz = float(t.izz)
            lf.inertia_ixy = float(t.ixy)
            lf.inertia_ixz = float(t.ixz)
            lf.inertia_iyz = float(t.iyz)
            return True

        return False

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
        empty_size = DEFAULT_LINK_GIZMO_SIZE
        link_name = "base_link"

        addon_prefs = get_addon_prefs(context)
        if addon_prefs:
            empty_size = getattr(addon_prefs, "link_empty_size", empty_size)

        # Create Empty object as link frame
        data = getattr(context, "data", None) or bpy.data
        empty = data.objects.new(link_name, None)
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
        lf = typing.cast("LinkPropertyGroup", getattr(empty, PROP_LINK))
        lf.is_robot_link = True
        logger.debug(f"add_empty_link set {empty.name}.is_robot_link to {lf.is_robot_link}")

        # Select the new link
        ops = getattr(context, "ops", None) or bpy.ops
        ops.object.select_all(action="DESELECT")
        empty.select_set(True)
        vl = getattr(context, "view_layer", bpy.context.view_layer)
        if vl:
            vl.objects.active = empty

        # Ensure name is sanitized
        typing.cast("LinkPropertyGroup", getattr(empty, PROP_LINK)).link_name = empty.name

        clear_stats_cache()
        self.report({"INFO"}, f"Added virtual link frame '{empty.name}' at cursor.")
        return {"FINISHED"}


class LINKFORGE_OT_create_link_from_mesh(Operator):
    """Create a robot link from a selected mesh object.

    This operator converts a standard Blender mesh into a LinkForge Robot
    Link by creating a parent Empty frame and establishing the required
    hierarchy and naming conventions for robot model export.
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
            hasattr(obj, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj, PROP_LINK)).is_robot_link
        ):
            return False

        # Don't allow if already a visual/collision child of a link
        return not (
            obj.parent
            and hasattr(obj.parent, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, PROP_LINK)).is_robot_link
            and (SUFFIX_VISUAL in obj.name.lower() or SUFFIX_COLLISION in obj.name.lower())
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        mesh_obj = context.active_object
        if not mesh_obj:
            return {"CANCELLED"}
        original_name = mesh_obj.name

        # Sanitize name for robot model compatibility
        link_name = sanitize_name(original_name)

        # Ensure we have a valid name
        if not link_name:
            link_name = "link"

        from ..preferences import get_addon_prefs

        # Initialize default size
        empty_size = DEFAULT_LINK_GIZMO_SIZE

        addon_prefs = get_addon_prefs(context)
        if addon_prefs:
            empty_size = getattr(addon_prefs, "link_empty_size", empty_size)

        # Rename mesh FIRST to free up the name for the Empty
        # This prevents Blender from auto-renaming the Empty to "name.001"
        mesh_obj.name = f"{link_name}{SUFFIX_VISUAL}"

        # Create Empty object as link frame
        data = getattr(context, "data", None) or bpy.data
        empty = data.objects.new(link_name, None)
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

            # Parent mesh to Empty with STRICT properties for model compatibility:
            # Clear existing parent relationships (e.g. from previous imports) to prevent dependency cycles
            mesh_obj.parent_type = "OBJECT"
            mesh_obj.parent_bone = ""

            # Remove Armature modifiers if present (LinkForge links are rigid bodies)
            for mod in mesh_obj.modifiers:
                if mod.type == "ARMATURE":
                    mesh_obj.modifiers.remove(mod)

            # Parent Inverse = Identity (No hidden transforms)
            # Local Location/Rotation = 0 (Visual matches Link frame)
            # Local Scale = Original Mesh Scale (Preserves visual size)

            mesh_obj.parent = empty
            mesh_obj.matrix_parent_inverse.identity()

            # CRITICAL: Force XYZ mode BEFORE zeroing rotations to ensure absolute precision
            mesh_obj.rotation_mode = "XYZ"
            mesh_obj.location = (0, 0, 0)
            mesh_obj.rotation_euler = (0, 0, 0)
            # mesh_obj.scale is already correct (it was S, parent is 1, so S stays S)

            # Mark Empty as robot link
            link_props = typing.cast(LinkPropertyGroup, getattr(empty, PROP_LINK))
            link_props.is_robot_link = True
            link_props.link_name = link_name

            # Set default mass
            link_props.mass = 1.0

            # Auto-calculate inertia enabled by default
            link_props.use_auto_inertia = True

            # Detect and store geometry properties on the mesh
            from ..adapters.blender_to_core import detect_primitive_type

            detected = detect_primitive_type(mesh_obj) or GEOM_MESH
            geom_props = getattr(mesh_obj, PROP_GEOM, None)
            if geom_props:
                geom_props.geometry_type = detected
                geom_props.geom_role = "VISUAL"

            # Select the new link Empty
            ops = getattr(context, "ops", None) or bpy.ops
            ops.object.select_all(action="DESELECT")
            empty.select_set(True)
            if context.view_layer is not None:
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
        "Auto-generate collision geometry from visual mesh. "
        "Requires at least one child mesh with " + SUFFIX_VISUAL + " suffix."
    )
    bl_options = {"REGISTER", "UNDO"}

    collision_type: bpy.props.EnumProperty(  # type: ignore
        name="Collision Type",
        description="Type of collision geometry to generate",
        items=[
            (
                GEOM_AUTO,
                "Auto-Detect",
                "Automatically detect primitive shape or use mesh simplification",
            ),
            (GEOM_BOX, "Bounding Box", "Use axis-aligned bounding box"),
            (GEOM_SPHERE, "Bounding Sphere", "Use bounding sphere"),
            (GEOM_CYLINDER, "Bounding Cylinder", "Cylindrical bounding volume around the mesh"),
            (
                GEOM_MESH,
                "Mesh (Simplified)",
                "Generate simplified mesh from visual geometry",
            ),
        ],
        default=GEOM_AUTO,
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
            hasattr(obj, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj, PROP_LINK)).is_robot_link
        ):
            return True

        # Allow if object is visual/collision child
        return bool(
            obj.parent
            and hasattr(obj.parent, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, PROP_LINK)).is_robot_link
            and (SUFFIX_VISUAL in obj.name.lower() or SUFFIX_COLLISION in obj.name.lower())
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        # Find all robot links
        links = [
            o
            for o in bpy.data.objects
            if hasattr(o, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(o, PROP_LINK)).is_robot_link
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
            and hasattr(obj.parent, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, PROP_LINK)).is_robot_link
            and (SUFFIX_VISUAL in obj.name.lower() or SUFFIX_COLLISION in obj.name.lower())
        ):
            link_obj = obj.parent

        if not link_obj:
            return {"CANCELLED"}

        # Determine collision type
        # Priority: Operator property (if changed in redo) > Link property > Default GEOM_AUTO
        collision_type = self.collision_type

        # Create collision
        collision_obj = create_collision_for_link(link_obj, collision_type, context)

        if collision_obj is None:
            # Check if it failed because of missing visuals
            visual_children = [
                c for c in link_obj.children if SUFFIX_VISUAL in c.name.lower() and c.type == "MESH"
            ]
            if not visual_children:
                self.report({"ERROR"}, "No visual meshes found. Cannot generate collision.")
            else:
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

        lp = typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK))
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
                hasattr(obj, PROP_LINK)
                and typing.cast("LinkPropertyGroup", getattr(obj, PROP_LINK)).is_robot_link
            ):
                # Resolve primary mesh to use for detection
                visual_children = [c for c in obj.children if SUFFIX_VISUAL in c.name.lower()]
                if visual_children:
                    collision_type = GEOM_AUTO
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
            hasattr(obj, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj, PROP_LINK)).is_robot_link
        ):
            collision_children = [c for c in obj.children if SUFFIX_COLLISION in c.name.lower()]
            return len(collision_children) > 0

        # Allow if object is visual/collision child
        if (
            obj.parent
            and hasattr(obj.parent, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, PROP_LINK)).is_robot_link
        ):
            collision_children = [
                c for c in obj.parent.children if SUFFIX_COLLISION in c.name.lower()
            ]
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
            hasattr(obj, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj, PROP_LINK)).is_robot_link
        ):
            # It's a link - toggle all its collision children
            for child in obj.children:
                if SUFFIX_COLLISION in child.name.lower():
                    child.hide_viewport = not child.hide_viewport
                    child.hide_render = child.hide_viewport  # Keep render state consistent
        else:
            # It's a visual/collision child - toggle its parent's collisions
            if obj.parent and hasattr(obj.parent, PROP_LINK):
                for child in obj.parent.children:
                    if SUFFIX_COLLISION in child.name.lower():
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
            and hasattr(obj.parent, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, PROP_LINK)).is_robot_link
        ):
            return True

        return bool(
            hasattr(obj, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj, PROP_LINK)).is_robot_link
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
            and hasattr(obj.parent, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, PROP_LINK)).is_robot_link
        ):
            link_obj = obj.parent

        if not link_obj:
            return {"CANCELLED"}

        success = calculate_inertia_for_link(link_obj)

        if success:
            link_name = typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK)).link_name
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
                hasattr(obj, PROP_LINK)
                and typing.cast("LinkPropertyGroup", getattr(obj, PROP_LINK)).is_robot_link
            ):
                if calculate_inertia_for_link(obj):
                    count += 1
                else:
                    # Only count as failed if it had visual/collision mesh but failed
                    has_mesh = any(
                        c.type == "MESH"
                        and (SUFFIX_VISUAL in c.name.lower() or SUFFIX_COLLISION in c.name.lower())
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


def _resolve_active_link(context: Context) -> bpy.types.Object | None:
    """Resolve the active link object from selection."""
    obj = context.active_object
    if not obj:
        return None
    if (
        hasattr(obj, PROP_LINK)
        and typing.cast("LinkPropertyGroup", getattr(obj, PROP_LINK)).is_robot_link
    ):
        return obj
    if (
        obj.parent
        and hasattr(obj.parent, PROP_LINK)
        and typing.cast("LinkPropertyGroup", getattr(obj.parent, PROP_LINK)).is_robot_link
    ):
        return obj.parent

    # If active object is not a link (e.g. it's a loose mesh), check selected objects
    for sel_obj in context.selected_objects:
        if (
            hasattr(sel_obj, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(sel_obj, PROP_LINK)).is_robot_link
        ):
            return sel_obj
        if (
            sel_obj.parent
            and hasattr(sel_obj.parent, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(sel_obj.parent, PROP_LINK)).is_robot_link
        ):
            return sel_obj.parent

    return None


class LINKFORGE_OT_assign_as_visual(Operator):
    """Parent selected mesh(es) to the active link as visual geometry."""

    bl_idname = "linkforge.assign_as_visual"
    bl_label = "Assign Selected as Visual"
    bl_description = "Parent selected mesh(es) to the active link as visual geometry"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        link_obj = _resolve_active_link(context)
        return link_obj is not None and any(
            o.type == "MESH" and o != link_obj and o.parent != link_obj
            for o in context.selected_objects
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        link_obj = _resolve_active_link(context)
        if not link_obj:
            return {"CANCELLED"}

        meshes = [
            o
            for o in context.selected_objects
            if o.type == "MESH" and o != link_obj and o.parent != link_obj
        ]
        if not meshes:
            self.report({"WARNING"}, "Select at least one loose mesh to assign")
            return {"CANCELLED"}

        lf = typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK))
        link_name = lf.link_name or link_obj.name

        from ..adapters.blender_to_core import detect_primitive_type

        for mesh in meshes:
            detected = detect_primitive_type(mesh) or GEOM_MESH

            idx = sum(1 for c in link_obj.children if SUFFIX_VISUAL in c.name.lower())
            suffix = f"_{idx}" if idx > 0 else ""
            mesh.name = f"{link_name}{SUFFIX_VISUAL}{suffix}"

            mesh.parent = link_obj
            mesh.matrix_parent_inverse = link_obj.matrix_world.inverted()
            mesh.rotation_mode = "XYZ"

            geom_props = getattr(mesh, PROP_GEOM, None)
            if geom_props:
                geom_props.geometry_type = detected
                geom_props.geom_role = "VISUAL"

        self.report({"INFO"}, f"Assigned {len(meshes)} visual(s) to '{link_name}'")
        clear_stats_cache()
        return {"FINISHED"}


class LINKFORGE_OT_assign_as_collision(Operator):
    """Parent selected mesh(es) to the active link as collision geometry."""

    bl_idname = "linkforge.assign_as_collision"
    bl_label = "Assign Selected as Collision"
    bl_description = "Parent selected mesh(es) to the active link as collision geometry"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        link_obj = _resolve_active_link(context)
        return link_obj is not None and any(
            o.type == "MESH" and o != link_obj and o.parent != link_obj
            for o in context.selected_objects
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        link_obj = _resolve_active_link(context)
        if not link_obj:
            return {"CANCELLED"}

        meshes = [
            o
            for o in context.selected_objects
            if o.type == "MESH" and o != link_obj and o.parent != link_obj
        ]
        if not meshes:
            self.report({"WARNING"}, "Select at least one loose mesh to assign")
            return {"CANCELLED"}

        lf = typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK))
        link_name = lf.link_name or link_obj.name

        from ..adapters.blender_to_core import detect_primitive_type

        for mesh in meshes:
            detected = detect_primitive_type(mesh) or GEOM_MESH

            idx = sum(1 for c in link_obj.children if SUFFIX_COLLISION in c.name.lower())
            suffix = f"_{idx}" if idx > 0 else ""
            mesh.name = f"{link_name}{SUFFIX_COLLISION}{suffix}"

            mesh.parent = link_obj
            mesh.matrix_parent_inverse = link_obj.matrix_world.inverted()
            mesh.rotation_mode = "XYZ"

            mesh.display_type = "WIRE"
            mesh.show_in_front = True
            mesh.hide_render = True

            geom_props = getattr(mesh, PROP_GEOM, None)
            if geom_props:
                geom_props.geometry_type = detected
                geom_props.geom_role = "COLLISION"
                geom_props.collision_quality = 100.0

        self.report({"INFO"}, f"Assigned {len(meshes)} collision(s) to '{link_name}'")
        clear_stats_cache()
        return {"FINISHED"}


class LINKFORGE_OT_set_active_geometry(Operator):
    """Set the active geometry index and select it."""

    bl_idname = "linkforge.set_active_geometry"
    bl_label = "Set Active Geometry"
    bl_description = "Set this geometry as active and select it in the viewport"
    bl_options = {"REGISTER", "UNDO"}

    geometry_type: bpy.props.EnumProperty(  # type: ignore
        items=[
            ("VISUAL", "Visual", ""),
            ("COLLISION", "Collision", ""),
        ],
        name="Geometry Type",
    )
    index: bpy.props.IntProperty(name="Index", default=0)  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        return _resolve_active_link(context) is not None

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        link_obj = _resolve_active_link(context)
        if not link_obj:
            return {"CANCELLED"}

        lf = typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK))

        if self.geometry_type == "VISUAL":
            lf.active_visual_index = self.index
            from ..properties.link_props import update_active_visual

            update_active_visual(lf, context)
        elif self.geometry_type == "COLLISION":
            lf.active_collision_index = self.index
            from ..properties.link_props import update_active_collision

            update_active_collision(lf, context)

        return {"FINISHED"}


class LINKFORGE_OT_remove_visual(Operator):
    """Remove the active visual mesh from the link."""

    bl_idname = "linkforge.remove_visual"
    bl_label = "Remove Visual"
    bl_description = "Remove the active visual mesh from the link"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        link_obj = _resolve_active_link(context)
        if not link_obj:
            return False
        lf = typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK))
        return hasattr(lf, "active_visual_index") and lf.active_visual_index >= 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        link_obj = _resolve_active_link(context)
        if not link_obj:
            return {"CANCELLED"}

        lf = typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK))
        visuals = [c for c in link_obj.children if SUFFIX_VISUAL in c.name.lower()]
        idx = getattr(lf, "active_visual_index", 0)

        if 0 <= idx < len(visuals):
            obj_to_remove = visuals[idx]

            # Keep world transforms when unparenting
            original_world_matrix = obj_to_remove.matrix_world.copy()
            obj_to_remove.parent = None
            obj_to_remove.matrix_world = original_world_matrix

            # Clean up name
            link_name = lf.link_name or link_obj.name
            if obj_to_remove.name.startswith(f"{link_name}{SUFFIX_VISUAL}"):
                obj_to_remove.name = obj_to_remove.name.replace(SUFFIX_VISUAL, "")

            # Adjust index if necessary
            if idx >= len(visuals) - 1 and idx > 0:
                lf.active_visual_index = idx - 1

            self.report({"INFO"}, f"Removed visual '{obj_to_remove.name}'")
            clear_stats_cache()

        return {"FINISHED"}


class LINKFORGE_OT_remove_collision(Operator):
    """Remove the active collision mesh from the link."""

    bl_idname = "linkforge.remove_collision"
    bl_label = "Remove Collision"
    bl_description = "Remove the active collision mesh from the link"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        link_obj = _resolve_active_link(context)
        if not link_obj:
            return False
        lf = typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK))
        return hasattr(lf, "active_collision_index") and lf.active_collision_index >= 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        link_obj = _resolve_active_link(context)
        if not link_obj:
            return {"CANCELLED"}

        lf = typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK))
        collisions = [c for c in link_obj.children if SUFFIX_COLLISION in c.name.lower()]
        idx = getattr(lf, "active_collision_index", 0)

        if 0 <= idx < len(collisions):
            obj_to_remove = collisions[idx]
            bpy.data.objects.remove(obj_to_remove, do_unlink=True)

            # Adjust index if necessary
            if idx >= len(collisions) - 1 and idx > 0:
                lf.active_collision_index = idx - 1

            self.report({"INFO"}, "Removed collision geometry")
            clear_stats_cache()

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
            hasattr(obj, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj, PROP_LINK)).is_robot_link
        ):
            return True

        # Allow if object is a visual/collision child of a link
        return bool(
            obj.parent
            and hasattr(obj.parent, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, PROP_LINK)).is_robot_link
            and (SUFFIX_VISUAL in obj.name.lower() or SUFFIX_COLLISION in obj.name.lower())
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
            and hasattr(obj.parent, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, PROP_LINK)).is_robot_link
            and (SUFFIX_VISUAL in obj.name.lower() or SUFFIX_COLLISION in obj.name.lower())
        ):
            link_obj = obj.parent

        if not link_obj:
            return {"CANCELLED"}

        lp = typing.cast("LinkPropertyGroup", getattr(link_obj, PROP_LINK))
        link_name = lp.link_name or link_obj.name

        # Find visual child
        visual_children = [
            c for c in link_obj.children if SUFFIX_VISUAL in c.name.lower() and c.type == "MESH"
        ]

        if not visual_children:
            # VIRTUAL LINK / EMPTY FRAME - Robust handling
            # If no visual mesh, we simply delete the collision children and the frame itself
            collision_children = [
                c for c in link_obj.children if SUFFIX_COLLISION in c.name.lower()
            ]
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
                if visual_obj.name.endswith(SUFFIX_VISUAL):
                    visual_obj.name = visual_obj.name[:-7]
                elif visual_obj.name.startswith(f"{link_name}_visual"):
                    visual_obj.name = link_name

            # Delete collision objects
            collision_children = [
                c for c in link_obj.children if SUFFIX_COLLISION in c.name.lower()
            ]
            for col in collision_children:
                bpy.data.objects.remove(col, do_unlink=True)

            # Delete the link empty
            bpy.data.objects.remove(link_obj, do_unlink=True)

            # Force update to ensure name namespace is freed in Blender
            if context.view_layer is not None:
                context.view_layer.update()

            # Select the (first) restored visual object for consistency
            if visual_children and context.view_layer is not None:
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
            hasattr(obj, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj, PROP_LINK)).is_robot_link
        ):
            return True

        # Allow if object is a visual child
        return bool(
            obj.parent
            and hasattr(obj.parent, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj.parent, PROP_LINK)).is_robot_link
            and SUFFIX_VISUAL in obj.name.lower()
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator."""
        obj = context.active_object

        # If selected object is link, find visual child
        if (
            obj
            and hasattr(obj, PROP_LINK)
            and typing.cast("LinkPropertyGroup", getattr(obj, PROP_LINK)).is_robot_link
        ):
            visual_children = [
                c for c in obj.children if SUFFIX_VISUAL in c.name.lower() and c.type == "MESH"
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
        if visual_obj.data and hasattr(visual_obj.data, "materials"):
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

    geom_props = getattr(collision_obj, PROP_GEOM, None)
    if not geom_props:
        return

    # PRIMITIVE INVARIANCE GUARANTEE:
    # Do not decimate primitive shapes (Box, Sphere, Cylinder). Decimation corrupts their bounding geometry
    # and distorts their representation in the 3D viewport. If any Decimate modifier exists, cleanly remove it.
    if geom_props.geometry_type != GEOM_MESH:
        decimate_mod = next((m for m in collision_obj.modifiers if m.type == "DECIMATE"), None)
        if decimate_mod:
            collision_obj.modifiers.remove(decimate_mod)
        return

    quality_ratio = geom_props.collision_quality / 100.0

    decimate_mod = next((m for m in collision_obj.modifiers if m.type == "DECIMATE"), None)
    if decimate_mod and isinstance(decimate_mod, bpy.types.DecimateModifier):
        decimate_mod.ratio = quality_ratio
    elif collision_obj.type == "MESH":
        # FALLBACK IMPROVEMENT: If modifier is missing but object exists, try adding it first
        # This is much faster than full _regenerate_ and avoids one potential "jump".
        decimate_mod = typing.cast(
            typing.Any,
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
    LINKFORGE_OT_assign_as_visual,
    LINKFORGE_OT_assign_as_collision,
    LINKFORGE_OT_set_active_geometry,
    LINKFORGE_OT_remove_visual,
    LINKFORGE_OT_remove_collision,
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
    logger.debug(f"link_ops.register called with {len(classes)} classes")
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
