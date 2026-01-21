"""3D gizmos for visualizing Center of Mass and Inertia Frame in the viewport.

This module provides visualization for the Center of Mass (Inertia Origin)
when users opt for manual inertia configuration.

Visualization Style:
- Orange/White Axis System (Principal Axes of Inertia)
- Dashed line connecting COM to link origin
- Only visible when "Auto-Calculate Inertia" is DISABLED
"""

from __future__ import annotations

import bpy
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils import Matrix, Vector

_builtin_shader_name = None


def get_shader():
    """Get the builtin shader."""
    global _builtin_shader_name
    if _builtin_shader_name is None:
        _builtin_shader_name = "FLAT_COLOR"
    return gpu.shader.from_builtin(_builtin_shader_name)


# Global drawing handle
_draw_handle = None


def generate_inertia_axes_geometry(obj, axis_length: float = 0.3) -> dict:
    """Generate geometry data for Inertia Axes (Orange/White style).

    Args:
        obj: Blender Object (Link)
        axis_length: Length of axis in Blender units

    Returns:
        Dictionary with line data for drawing
    """
    if not obj:
        return {"lines": [], "line_colors": []}

    props = obj.linkforge

    # Get manual inertia origin relative to link
    # The property inertia_origin_xyz is in LINK LOCAL space
    com_local_pos = Vector(props.inertia_origin_xyz)
    com_local_rot = Vector(props.inertia_origin_rpy)

    # Transform to World Space
    # Link World Matrix
    link_matrix = obj.matrix_world

    # COM World Position
    com_world_pos = link_matrix @ com_local_pos

    # Calculate COM World Rotation
    # We combine the Link's rotation with the Manual Inertia Rotation
    # 1. Start with Link Rotation
    # 2. Apply Manual RPY Rotation (XYZ Euler)
    manual_rot_matrix = (
        Matrix.Rotation(com_local_rot.x, 4, "X")
        @ Matrix.Rotation(com_local_rot.y, 4, "Y")
        @ Matrix.Rotation(com_local_rot.z, 4, "Z")
    )

    # Combine: Local Inertia Frame -> Link Frame -> World Frame
    # To get just the direction vectors, we can rotate unit vectors
    inertia_rotation_world = link_matrix.to_3x3() @ manual_rot_matrix.to_3x3()

    # Define axis directions for the Inertia Frame
    axes = {
        "x": Vector((1.0, 0.0, 0.0)),
        "y": Vector((0.0, 1.0, 0.0)),
        "z": Vector((0.0, 0.0, 1.0)),
    }

    # Style: Orange/White Theme
    # X = Orange
    # Y = White
    # Z = Light Orange? Or maybe just X=Orange, Y=Orange, Z=Orange to denote "Inertia Box"
    # Actually, Principal Axes usually imply a coordinate system.
    # Let's stick to the docs description: "Orange/White Axis".
    # Let's make:
    # X: Bright Orange
    # Y: White
    # Z: Light Orange
    colors = {
        "x": (1.0, 0.5, 0.0, 1.0),  # Orange
        "y": (1.0, 1.0, 1.0, 1.0),  # White
        "z": (1.0, 0.7, 0.2, 1.0),  # Light Orange
    }

    line_positions = []
    line_colors = []

    # 1. Draw connecting line from Link Origin to COM (Dashed style simulation)
    # We simulate dashed line by drawing small segments or just a thinner line with lower alpha
    link_origin = link_matrix.translation
    line_positions.extend([link_origin[:], com_world_pos[:]])
    line_colors.extend([(1.0, 1.0, 1.0, 0.5), (1.0, 1.0, 1.0, 0.5)])  # Semi-transparent white

    # 2. Draw Principal Axes at COM
    for axis_name, local_dir in axes.items():
        # Rotate axis to world space
        world_dir = inertia_rotation_world @ local_dir
        world_dir.normalize()

        end_pos = com_world_pos + (world_dir * axis_length)

        line_positions.extend([com_world_pos[:], end_pos[:]])
        line_colors.extend([colors[axis_name], colors[axis_name]])

    return {
        "lines": line_positions,
        "line_colors": line_colors,
    }


def draw_inertia_gizmos():
    """Draw Inertia gizmos for selected link if Auto-Inertia is OFF."""
    context = bpy.context

    # Only draw if we have a selected object that is a Link
    obj = context.active_object
    if not obj or not obj.select_get():
        return

    # Check if it's a robot link
    if not hasattr(obj, "linkforge") or not obj.linkforge.is_robot_link:
        return

    # Only draw if Manual Inertia is active (Auto-Calculate is OFF)
    if obj.linkforge.use_auto_inertia:
        return

    # Check properties
    # Just draw it, no need for extra global toggle unless requested
    # Users disable auto-inertia specifically to edit this, so we should always show it.

    axis_data = generate_inertia_axes_geometry(obj)

    if not axis_data["lines"]:
        return

    # Draw
    shader = get_shader()
    batch = batch_for_shader(
        shader,
        "LINES",
        {"pos": axis_data["lines"], "color": axis_data["line_colors"]},
    )

    matrix = gpu.matrix.get_projection_matrix() @ gpu.matrix.get_model_view_matrix()

    gpu.state.line_width_set(2.0)
    gpu.state.depth_test_set("ALWAYS")  # Always show on top (like X-Ray) for visibility
    gpu.state.blend_set("ALPHA")

    shader.bind()
    shader.uniform_float("ModelViewProjectionMatrix", matrix)
    batch.draw(shader)

    gpu.state.blend_set("NONE")
    gpu.state.depth_test_set("NONE")
    gpu.state.line_width_set(1.0)


def ensure_inertia_handler():
    """Ensure the inertia visualization draw handler is registered.

    This should be called when Manual Inertia is enabled or when a file is loaded
    with Manual Inertia links. It is safe to call multiple times.
    """
    global _draw_handle
    if _draw_handle is None:
        _draw_handle = bpy.types.SpaceView3D.draw_handler_add(
            draw_inertia_gizmos, (), "WINDOW", "POST_VIEW"
        )


def check_manual_inertia_on_load(dummy=None):
    """Check if any link has Manual Inertia on file load."""
    try:
        scene = bpy.context.scene
    except (AttributeError, RuntimeError):
        return

    # Scan scene for any link with manual inertia
    for obj in scene.objects:
        if hasattr(obj, "linkforge") and obj.linkforge.is_robot_link:
            if not obj.linkforge.use_auto_inertia:
                ensure_inertia_handler()
                return  # Found one, we're done


def register():
    """Register inertia visualization components."""
    # Register load handler to scan for manual inertia usage on file open
    if check_manual_inertia_on_load not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(check_manual_inertia_on_load)


def unregister():
    """Unregister inertia visualization components."""
    global _draw_handle

    # Remove load handler
    if check_manual_inertia_on_load in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(check_manual_inertia_on_load)

    # Remove draw handler
    if _draw_handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, "WINDOW")
        _draw_handle = None
