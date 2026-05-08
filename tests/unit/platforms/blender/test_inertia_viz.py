import bpy
from linkforge.blender.visualization.inertia_gizmos import (
    check_manual_inertia_on_load,
    draw_inertia_gizmos,
    ensure_inertia_handler,
    generate_inertia_axes_geometry,
    register,
    unregister,
)

from tests.blender_test_utils import (
    create_test_object,
    safe_get_linkforge,
)


def test_generate_inertia_axes_geometry_values(scene) -> None:
    """Test correct geometry generation logic with a real object (Pure Logic)."""
    obj = create_test_object("inertia_test_obj", None, scene)
    assert obj is not None
    safe_get_linkforge(obj).inertia_origin_xyz = (1.0, 0.0, 0.0)

    data = generate_inertia_axes_geometry(obj)

    # Expect 104 points (3 axes + 1 connector + 3 rings)
    assert len(data["lines"]) == 104
    assert len(data["line_colors"]) == 104


def test_draw_inertia_gizmos_execution(scene) -> None:
    """Execute the draw function to ensure no Python-level errors occur."""
    # Setup real objects
    obj1 = create_test_object("obj1", None, scene)
    assert obj1 is not None
    props = safe_get_linkforge(obj1)
    props.is_robot_link = True
    props.use_auto_inertia = False
    obj1.select_set(True)

    # Call draw (headless)
    try:
        draw_inertia_gizmos()
    except Exception as e:
        if "gpu" in str(e).lower() or "context" in str(e).lower():
            pass
        else:
            raise e


def test_ensure_inertia_handler_logic(scene) -> None:
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
        # Restore state
        if ig._draw_handle:
            import contextlib

            with contextlib.suppress(ValueError):
                bpy.types.SpaceView3D.draw_handler_remove(ig._draw_handle, "WINDOW")
        ig._draw_handle = old_handle


def test_check_manual_inertia_on_load_logic(scene) -> None:
    """Test scanning of real objects checks."""
    obj = create_test_object("obj_load", None, scene)
    assert obj is not None
    props = safe_get_linkforge(obj)
    props.is_robot_link = True
    props.use_auto_inertia = False

    check_manual_inertia_on_load(None)


def test_lifecycle_register_unregister(scene) -> None:
    """Test register and unregister functions safely."""
    register()
    unregister()
