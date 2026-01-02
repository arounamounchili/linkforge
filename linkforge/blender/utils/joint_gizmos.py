"""3D gizmos for visualizing joint axes in the viewport.

This module provides RViz-style RGB axis visualization for robot joints:
- Red = X axis (with arrow head)
- Green = Y axis (with arrow head)
- Blue = Z axis (with arrow head)

Each axis is drawn as a solid colored line with an arrow cone at the tip,
matching the professional appearance of RViz.
"""

from __future__ import annotations

import math

import bpy
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils import Vector

from ..preferences import get_addon_prefs

# Global drawing handle
_draw_handle = None


def generate_arrow_cone_vertices(
    origin: Vector, direction: Vector, length: float, cone_ratio: float = 0.2
) -> tuple[list, list]:
    """Generate vertices for an arrow cone at the tip of an axis.

    Args:
        origin: Start point of the arrow
        direction: Direction vector (normalized)
        length: Total length of axis
        cone_ratio: Ratio of cone length to total length (default 0.2 = 20%)

    Returns:
        Tuple of (positions, indices) for triangle drawing

    """
    cone_length = length * cone_ratio
    cone_radius = cone_length * 0.3  # Cone base radius
    shaft_end = length * (1.0 - cone_ratio)

    # Calculate cone tip and base center
    tip = origin + (direction * length)
    base_center = origin + (direction * shaft_end)

    # Create perpendicular vectors for cone base circle
    if abs(direction.x) < 0.9:
        perp1 = direction.cross(Vector((1, 0, 0))).normalized()
    else:
        perp1 = direction.cross(Vector((0, 1, 0))).normalized()
    perp2 = direction.cross(perp1).normalized()

    # Generate cone base circle vertices (8 segments for smooth appearance)
    num_segments = 8
    base_vertices = []
    for i in range(num_segments):
        angle = (2 * math.pi * i) / num_segments
        vertex = base_center + (perp1 * math.cos(angle) + perp2 * math.sin(angle)) * cone_radius
        base_vertices.append(vertex[:])

    positions = [tip[:]]  # Tip is vertex 0
    positions.extend(base_vertices)  # Base vertices are 1 to num_segments
    positions.append(base_center[:])  # Base center is last vertex

    # Generate triangle indices for cone
    indices = []
    tip_idx = 0
    center_idx = len(positions) - 1

    # Side triangles (from tip to base edge)
    for i in range(num_segments):
        next_i = (i + 1) % num_segments
        indices.extend([tip_idx, i + 1, next_i + 1])

    # Base triangles (filling the base)
    for i in range(num_segments):
        next_i = (i + 1) % num_segments
        indices.extend([center_idx, next_i + 1, i + 1])

    return positions, indices


def generate_axis_geometry(obj, axis_length: float = 0.2) -> dict:
    """Generate geometry data for RGB axes with arrow heads (RViz style).

    Args:
        obj: Blender Empty object with joint properties
        axis_length: Length of axis in Blender units

    Returns:
        Dictionary with line and triangle data for drawing

    """
    if not obj or obj.type != "EMPTY":
        return {"lines": [], "line_colors": [], "tris": [], "tri_colors": []}

    # Get joint origin in world space
    origin = obj.matrix_world.translation

    # Get joint rotation matrix
    rotation_matrix = obj.matrix_world.to_3x3()

    # Define axis directions in local space
    local_axes = {
        "x": Vector((1.0, 0.0, 0.0)),
        "y": Vector((0.0, 1.0, 0.0)),
        "z": Vector((0.0, 0.0, 1.0)),
    }

    # Define RGB colors for each axis (RViz convention)
    axis_colors = {
        "x": (1.0, 0.0, 0.0, 1.0),  # Red
        "y": (0.0, 1.0, 0.0, 1.0),  # Green
        "z": (0.0, 0.0, 1.0, 1.0),  # Blue
    }

    line_positions = []
    line_colors = []
    tri_positions = []
    tri_colors = []

    # Generate geometry for each axis
    for axis_name, local_dir in local_axes.items():
        # Transform axis direction to world space
        world_dir = rotation_matrix @ local_dir
        world_dir.normalize()

        color = axis_colors[axis_name]

        # Generate shaft line (from origin to 80% of length)
        shaft_length = axis_length * 0.8
        shaft_end = origin + (world_dir * shaft_length)
        line_positions.extend([origin[:], shaft_end[:]])
        line_colors.extend([color, color])

        # Generate arrow cone at tip
        cone_positions, cone_indices = generate_arrow_cone_vertices(
            origin, world_dir, axis_length, cone_ratio=0.2
        )

        # Add cone triangles
        for idx in cone_indices:
            tri_positions.append(cone_positions[idx])
            tri_colors.append(color)

    return {
        "lines": line_positions,
        "line_colors": line_colors,
        "tris": tri_positions,
        "tri_colors": tri_colors,
    }


def draw_joint_axes():
    """Draw RGB axes for all joint objects in the scene.

    Draws RViz-style arrows with colored shafts and arrow heads.
    """
    _draw_internal()


def _draw_internal():
    context = bpy.context
    scene = context.scene

    # Get preferences
    show_axes = True
    axis_length = 0.2

    addon_prefs = get_addon_prefs(context)

    if addon_prefs:
        show_axes = getattr(addon_prefs, "show_joint_axes", show_axes)
        axis_length = getattr(addon_prefs, "joint_empty_size", axis_length)

    if not show_axes:
        return

    # Check for region_data to prevent crashes in non-3D View contexts
    if not hasattr(context, "region_data") or context.region_data is None:
        return

    all_line_positions = []
    all_line_colors = []
    all_tri_positions = []
    all_tri_colors = []

    for obj in scene.objects:
        if obj.type == "EMPTY":
            # Check if this is a joint Empty
            if hasattr(obj, "linkforge_joint") and obj.linkforge_joint.is_robot_joint:
                # Generate axis geometry for this joint
                axis_data = generate_axis_geometry(obj, axis_length)
                all_line_positions.extend(axis_data["lines"])
                all_line_colors.extend(axis_data["line_colors"])
                all_tri_positions.extend(axis_data["tris"])
                all_tri_colors.extend(axis_data["tri_colors"])

    if not all_line_positions:
        return

    # Set up GPU state - ALWAYS draw over meshes for RViz style
    gpu.state.depth_test_set("ALWAYS")
    gpu.state.blend_set("ALPHA")

    # Draw lines (shafts)
    if all_line_positions:
        # Blender 4.3+ renamed 3D_FLAT_COLOR to FLAT_COLOR
        shader_name = "FLAT_COLOR" if bpy.app.version >= (4, 3, 0) else "3D_FLAT_COLOR"
        shader = gpu.shader.from_builtin(shader_name)
        batch = batch_for_shader(
            shader,
            "LINES",
            {"pos": all_line_positions, "color": all_line_colors},
        )
        matrix = gpu.matrix.get_projection_matrix() @ gpu.matrix.get_model_view_matrix()

        gpu.state.line_width_set(4.0)
        shader.bind()
        shader.uniform_float("ModelViewProjectionMatrix", matrix)
        batch.draw(shader)

    # Draw triangles (arrow cones)
    if all_tri_positions:
        shader_name = "FLAT_COLOR" if bpy.app.version >= (4, 3, 0) else "3D_FLAT_COLOR"
        shader = gpu.shader.from_builtin(shader_name)
        batch = batch_for_shader(
            shader,
            "TRIS",
            {"pos": all_tri_positions, "color": all_tri_colors},
        )
        matrix = gpu.matrix.get_projection_matrix() @ gpu.matrix.get_model_view_matrix()
        shader.bind()
        shader.uniform_float("ModelViewProjectionMatrix", matrix)
        batch.draw(shader)

    # Reset GPU state
    gpu.state.line_width_set(1.0)
    gpu.state.blend_set("NONE")
    gpu.state.depth_test_set("NONE")


def fix_existing_joints(dummy=None):
    """Fix display type for existing joints.

    Args:
        dummy: Unused parameter (Blender handlers pass filepath string, we ignore it)

    """
    # Get current scene from context
    try:
        scene = bpy.context.scene
    except (AttributeError, RuntimeError):
        return

    # Get preferred empty size from addon preferences
    empty_size = 0.2  # Default fallback
    addon_prefs = get_addon_prefs()
    if addon_prefs:
        empty_size = getattr(addon_prefs, "joint_empty_size", empty_size)

    for obj in scene.objects:
        if (
            obj.type == "EMPTY"
            and hasattr(obj, "linkforge_joint")
            and obj.linkforge_joint.is_robot_joint
        ):
            # Ensure ARROWS type (RGB colored axes - Red=X, Green=Y, Blue=Z)
            if obj.empty_display_type != "ARROWS":
                obj.empty_display_type = "ARROWS"
            # Set display size from preferences
            obj.empty_display_size = empty_size


def fix_current_scene():
    """Timer callback to fix joints in the current scene after registration.

    This runs once after the addon registers to fix any existing joints
    in the currently open scene. Returns None to prevent the timer from repeating.
    """
    fix_existing_joints()
    update_viz_handle(bpy.context)
    return None  # Don't repeat


def register():
    """Register the joint axes visualization components."""
    # Fix joints and restore draw handler state when file is loaded
    if fix_existing_joints not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(fix_existing_joints)

    # Also fix current scene and restore draw handler on registration
    bpy.app.timers.register(fix_current_scene, first_interval=0.1)


def unregister():
    """Unregister the joint axes visualization components."""
    global _draw_handle

    # Remove load handler
    if fix_existing_joints in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(fix_existing_joints)

    # Remove draw handler if it exists
    if _draw_handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, "WINDOW")
        _draw_handle = None


def update_viz_handle(context):
    """Enable or disable the draw handler based on user preferences.

    This is called by the preference update function.
    """
    global _draw_handle

    # Get preferences
    addon_prefs = get_addon_prefs(context)

    show_axes = getattr(addon_prefs, "show_joint_axes", False) if addon_prefs else False

    if show_axes and _draw_handle is None:
        # Register handler only when needed
        _draw_handle = bpy.types.SpaceView3D.draw_handler_add(
            draw_joint_axes, (), "WINDOW", "POST_VIEW"
        )
    elif not show_axes and _draw_handle is not None:
        # Remove handler when not in use to save memory
        bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, "WINDOW")
        _draw_handle = None

    # Force redraw
    for window in context.window_manager.windows:
        for area in window.screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()


if __name__ == "__main__":
    register()
