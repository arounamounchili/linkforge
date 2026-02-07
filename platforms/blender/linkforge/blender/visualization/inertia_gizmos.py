"""3D gizmos for visualizing Center of Mass and Inertia Frames in the viewport.

This module provides high-visibility overlays for the Center of Mass (CoM)
and Inertia Frame (Principal Axes) when manual inertia is configured.

Visualization Style:
- Orange/White Axis System (Principal Axes of Inertia)
- Yellow Wireframe Sphere (Center of Mass Marker, RViz/Gazebo style)
- Semi-transparent line connecting CoM to link origin
- Permanently visible for objects with manual inertia (when enabled in preferences)
"""

from __future__ import annotations

import math
from typing import Any

import bpy
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils import Matrix, Vector

from ..preferences import get_addon_prefs


def get_shader() -> Any:
    """Get the builtin shader."""
    return gpu.shader.from_builtin("FLAT_COLOR")


# Global drawing handle
_draw_handle: Any = None


def generate_inertia_axes_geometry(obj: Any, axis_length: float = 0.1) -> dict[str, list[Any]]:
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

    # Style: Orange/White Theme (Principal Axes)
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

    # 3. Draw Center of Mass Sphere (Standard Robotics Style)
    # We draw 3 orthogonal rings to form a wireframe sphere

    sphere_radius = axis_length * 0.2
    segments = 16
    sphere_color = (1.0, 1.0, 0.0, 1.0)  # Yellow for Mass

    # Local circle points
    circle_points = []
    for i in range(segments + 1):
        angle = 2 * math.pi * i / segments
        circle_points.append((math.cos(angle) * sphere_radius, math.sin(angle) * sphere_radius))

    # Rings in local inertia frame
    rings = [
        # XY Plane Ring
        [(x, y, 0.0) for x, y in circle_points],
        # XZ Plane Ring
        [(x, 0.0, y) for x, y in circle_points],
        # YZ Plane Ring
        [(0.0, x, y) for x, y in circle_points],
    ]

    for ring_points in rings:
        # Transform ring points to world space
        world_ring_points = []
        for p in ring_points:
            local_vec = Vector(p)
            world_vec = inertia_rotation_world @ local_vec
            world_ring_points.append(com_world_pos + world_vec)

        # Add line segments
        for i in range(len(world_ring_points) - 1):
            line_positions.extend([world_ring_points[i][:], world_ring_points[i + 1][:]])
            line_colors.extend([sphere_color, sphere_color])

    return {
        "lines": line_positions,
        "line_colors": line_colors,
    }


def draw_inertia_gizmos() -> None:
    """Draw Inertia frames and CoM spheres for all visible links with manual inertia."""
    try:
        context = bpy.context

        # Check global visibility preference
        show_gizmos = True
        gizmo_size = 0.1

        try:
            prefs = get_addon_prefs(context)
            if prefs:
                show_gizmos = prefs.show_inertia_gizmos
                gizmo_size = prefs.inertia_gizmo_size
        except Exception:
            # Fallback if preferences access fails (safe default)
            pass

        if not show_gizmos:
            return

        objects_to_draw = context.visible_objects

        if not objects_to_draw:
            return

        shader = get_shader()

        # Collect all geometry to batch draw (performance optimization)
        all_line_positions = []
        all_line_colors = []

        for obj in objects_to_draw:
            # Check if it's a robot link
            if not hasattr(obj, "linkforge") or not obj.linkforge.is_robot_link:
                continue

            # Only draw if Manual Inertia is active (Auto-Calculate is OFF)
            if obj.linkforge.use_auto_inertia:
                continue

            axis_data = generate_inertia_axes_geometry(obj, axis_length=gizmo_size)
            if axis_data["lines"]:
                all_line_positions.extend(axis_data["lines"])
                all_line_colors.extend(axis_data["line_colors"])

        if not all_line_positions:
            return

        # Draw everything in one batch
        batch = batch_for_shader(
            shader,
            "LINES",
            {"pos": all_line_positions, "color": all_line_colors},
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
    except Exception:
        # Prevent Blender from unregistering the handler due to error
        pass


def tag_redraw() -> None:
    """Force redraw of all 3D views."""
    context = bpy.context
    if not hasattr(context, "window_manager") or not context.window_manager:
        return

    for window in context.window_manager.windows:
        for area in window.screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()


def ensure_inertia_handler() -> None:
    """Ensure the inertia visualization draw handler is registered.

    This should be called when Manual Inertia is enabled or when a file is loaded
    with Manual Inertia links. It is safe to call multiple times.
    """
    global _draw_handle
    if _draw_handle is None:
        _draw_handle = bpy.types.SpaceView3D.draw_handler_add(
            draw_inertia_gizmos, (), "WINDOW", "POST_VIEW"
        )
        tag_redraw()


def check_manual_inertia_on_load(dummy: Any = None) -> float | None:
    """Check if any link has Manual Inertia on file load or registration."""
    try:
        scene = bpy.context.scene
    except (AttributeError, RuntimeError):
        return None

    # Scan scene for any link with manual inertia
    found_manual = False
    for obj in scene.objects:
        if (
            hasattr(obj, "linkforge")
            and obj.linkforge.is_robot_link
            and not obj.linkforge.use_auto_inertia
        ):
            found_manual = True
            break

    if found_manual:
        ensure_inertia_handler()

    return None  # For timer compliance


def register() -> None:
    """Register inertia visualization components."""
    # Register load handler to scan for manual inertia usage on file open
    if check_manual_inertia_on_load not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(check_manual_inertia_on_load)

    # Also check current scene immediately (handles "enable addon" case)
    # Use timer to let context initialize if needed
    bpy.app.timers.register(check_manual_inertia_on_load, first_interval=0.1)


def unregister() -> None:
    """Unregister inertia visualization components."""
    global _draw_handle

    # Remove load handler
    if check_manual_inertia_on_load in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(check_manual_inertia_on_load)

    # Remove draw handler
    if _draw_handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, "WINDOW")
        _draw_handle = None
        tag_redraw()
