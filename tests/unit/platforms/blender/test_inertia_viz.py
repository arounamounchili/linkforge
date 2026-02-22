import bpy
from linkforge.blender.visualization.inertia_gizmos import (
    check_manual_inertia_on_load,
    draw_inertia_gizmos,
    ensure_inertia_handler,
    generate_inertia_axes_geometry,
    register,
    unregister,
)


def test_generate_inertia_axes_geometry_values():
    """Test correct geometry generation logic with a real object (Pure Logic)."""
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    obj = bpy.context.active_object
    obj.linkforge.inertia_origin_xyz = (1.0, 0.0, 0.0)

    data = generate_inertia_axes_geometry(obj)

    # Expect 104 points (3 axes + 1 connector + 3 rings)
    assert len(data["lines"]) == 104
    assert len(data["line_colors"]) == 104


def test_draw_inertia_gizmos_execution():
    """Execute the draw function to ensure no Python-level errors occur."""
    # 1. Setup real objects
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    obj1 = bpy.context.active_object
    obj1.linkforge.is_robot_link = True
    obj1.linkforge.use_auto_inertia = False
    obj1.select_set(True)

    # 2. Call draw (headless)
    # In a headless environment, this might do nothing or raise a Context/GPU error.
    # We just want to ensure our *logic* doesn't crash (e.g., AttributeErrors).
    try:
        draw_inertia_gizmos()
    except Exception as e:
        # If it's a GPU error (e.g. no context), that's expected in headless.
        # But we caught it, so we executed the logic leading up to it.
        # If it's a logic error (AttributeError), fail.
        if "gpu" in str(e).lower() or "context" in str(e).lower():
            pass
        else:
            raise e


def test_ensure_inertia_handler_logic():
    """Test handler registration logic directly."""
    # Clear any existing handle
    import linkforge.blender.visualization.inertia_gizmos as ig

    old_handle = ig._draw_handle
    ig._draw_handle = None

    try:
        ensure_inertia_handler()
        assert ig._draw_handle is not None

        # Verify idempotency
        handle = ig._draw_handle
        ensure_inertia_handler()
        assert ig._draw_handle == handle
    finally:
        # Restore state to avoid polluting other tests
        if ig._draw_handle:
            import contextlib

            with contextlib.suppress(ValueError):
                bpy.types.SpaceView3D.draw_handler_remove(ig._draw_handle, "WINDOW")
        ig._draw_handle = old_handle


def test_check_manual_inertia_on_load_logic():
    """Test scanning of real objects checks."""
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    obj = bpy.context.active_object
    obj.linkforge.is_robot_link = True
    obj.linkforge.use_auto_inertia = False

    check_manual_inertia_on_load(None)


def test_lifecycle_register_unregister():
    """Test register and unregister functions safely."""
    # This interacts with real bpy.app.handlers
    register()
    unregister()
