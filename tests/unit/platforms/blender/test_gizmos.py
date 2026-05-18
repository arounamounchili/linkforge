"""Unit tests for Center of Mass, Inertia Frame, and Joint Axes 3D visualization gizmos."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import bpy
import linkforge.blender.visualization.inertia_gizmos as inertia_gizmos
import linkforge.blender.visualization.joint_gizmos as joint_gizmos
import pytest
from mathutils import Vector

from tests.blender_test_utils import (
    cleanup_blender_scene,
    create_robot_link,
    safe_get_joint,
    safe_get_linkforge,
    safe_update,
)


class TestInertiaGizmos:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_generate_inertia_axes_geometry_empty(self) -> None:
        """Verify generate_inertia_axes_geometry handles None/empty objects gracefully."""
        res = inertia_gizmos.generate_inertia_axes_geometry(None)
        assert res["lines"] == []
        assert res["line_colors"] == []

    def test_generate_inertia_axes_geometry_valid_link(self, scene) -> None:
        """Verify generate_inertia_axes_geometry produces correct axis and CoM lines."""
        link_obj = create_robot_link("com_link", scene, with_visual=True, with_collision=False)
        lf = safe_get_linkforge(link_obj)
        lf.inertia_origin_xyz = (0.1, 0.2, 0.3)
        lf.inertia_origin_rpy = (0.0, 0.0, 0.0)
        safe_update(scene)

        res = inertia_gizmos.generate_inertia_axes_geometry(link_obj, axis_length=0.5)
        # There should be lines for origin-to-CoM segment + CoM axes + wireframe circles
        assert len(res["lines"]) > 0
        assert len(res["line_colors"]) == len(res["lines"])

    def test_draw_inertia_gizmos_preferences_and_execution(self, scene) -> None:
        """Test drawing lifecycle with various preference combinations."""
        gpu = inertia_gizmos.gpu

        # Scenario 1: Hidden by preference
        with patch("linkforge.blender.visualization.inertia_gizmos.get_addon_prefs") as mock_prefs:
            prefs = MagicMock()
            prefs.show_inertia_gizmos = False
            mock_prefs.return_value = prefs

            # Should return early
            assert inertia_gizmos.draw_inertia_gizmos() is None

        # Scenario 2: Enabled preference, but no manual inertia objects
        with (
            patch("linkforge.blender.visualization.inertia_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.get_robot_statistics"
            ) as mock_stats,
        ):
            prefs = MagicMock()
            prefs.show_inertia_gizmos = True
            prefs.inertia_gizmo_size = 0.2
            mock_prefs.return_value = prefs

            stats = MagicMock()
            stats.manual_inertia_objects = []
            mock_stats.return_value = stats

            assert inertia_gizmos.draw_inertia_gizmos() is None

        # Scenario 3: Enabled with manual inertia objects (full GPU draw branch)
        link_obj = create_robot_link("manual_link", scene, with_visual=True, with_collision=False)
        lf = safe_get_linkforge(link_obj)
        lf.is_robot_link = True
        lf.inertia_origin_xyz = (0.0, 0.0, 0.0)
        safe_update(scene)

        gpu.state.depth_test_set.reset_mock()
        gpu.state.blend_set.reset_mock()
        gpu.state.line_width_set.reset_mock()

        with (
            patch("linkforge.blender.visualization.inertia_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.get_robot_statistics"
            ) as mock_stats,
        ):
            prefs = MagicMock()
            prefs.show_inertia_gizmos = True
            prefs.inertia_gizmo_size = 0.1
            mock_prefs.return_value = prefs

            stats = MagicMock()
            stats.manual_inertia_objects = [link_obj]
            mock_stats.return_value = stats

            inertia_gizmos.draw_inertia_gizmos()

            assert gpu.state.depth_test_set.called
            assert gpu.state.blend_set.called
            assert gpu.state.line_width_set.called

    def test_draw_inertia_gizmos_handles_deleted_object_reference_error(self, scene) -> None:
        """Verify draw loops ignore objects that raise ReferenceError."""
        with (
            patch("linkforge.blender.visualization.inertia_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.get_robot_statistics"
            ) as mock_stats,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.generate_inertia_axes_geometry",
                side_effect=ReferenceError("Object deleted"),
            ),
        ):
            prefs = MagicMock()
            prefs.show_inertia_gizmos = True
            mock_prefs.return_value = prefs

            stats = MagicMock()
            stats.manual_inertia_objects = [MagicMock()]
            mock_stats.return_value = stats

            # Should not raise exception
            inertia_gizmos.draw_inertia_gizmos()

    def test_ensure_inertia_handler(self) -> None:
        """Verify SpaceView3D draw handler registration/tag_redraw."""
        with (
            patch.object(
                bpy.types.SpaceView3D, "draw_handler_add", return_value="dummy_handle"
            ) as mock_add,
            patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw") as mock_redraw,
        ):
            inertia_gizmos._draw_handle = None
            inertia_gizmos.ensure_inertia_handler()
            assert inertia_gizmos._draw_handle == "dummy_handle"
            assert mock_add.called
            assert mock_redraw.called

    def test_check_manual_inertia_on_load(self, scene) -> None:
        """Verify check_manual_inertia_on_load triggers ensure_inertia_handler appropriately."""
        # Empty scene
        with patch(
            "linkforge.blender.visualization.inertia_gizmos.ensure_inertia_handler"
        ) as mock_ensure:
            inertia_gizmos.check_manual_inertia_on_load()
            assert not mock_ensure.called

        # Scene with manual inertia objects
        link_obj = create_robot_link("load_link", scene, with_visual=True, with_collision=False)
        lf = safe_get_linkforge(link_obj)
        lf.is_robot_link = True
        safe_update(scene)

        with (
            patch(
                "linkforge.blender.visualization.inertia_gizmos.get_robot_statistics"
            ) as mock_stats,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.ensure_inertia_handler"
            ) as mock_ensure,
        ):
            stats = MagicMock()
            stats.manual_inertia_objects = [link_obj]
            mock_stats.return_value = stats

            inertia_gizmos.check_manual_inertia_on_load()
            assert mock_ensure.called

    def test_register_unregister_lifecycle(self) -> None:
        """Verify register and unregister handlers setup and cleanup correctly."""
        # Clear existing
        if inertia_gizmos.check_manual_inertia_on_load in bpy.app.handlers.load_post:
            bpy.app.handlers.load_post.remove(inertia_gizmos.check_manual_inertia_on_load)

        inertia_gizmos.register()
        assert inertia_gizmos.check_manual_inertia_on_load in bpy.app.handlers.load_post

        with patch.object(bpy.types.SpaceView3D, "draw_handler_remove") as mock_remove:
            inertia_gizmos._draw_handle = "some_handle"
            inertia_gizmos.unregister()
            assert inertia_gizmos._draw_handle is None
            assert mock_remove.called
            assert inertia_gizmos.check_manual_inertia_on_load not in bpy.app.handlers.load_post


class TestJointGizmos:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_generate_arrow_cone_vertices(self) -> None:
        """Test calculation of cone tips, perpendicular vectors, and triangle base circle indices."""
        origin = Vector((0.0, 0.0, 0.0))
        direction = Vector((0.0, 0.0, 1.0))
        length = 1.0

        positions, indices = joint_gizmos.generate_arrow_cone_vertices(origin, direction, length)
        # Tip vertex should be at (0, 0, 1)
        assert list(positions[0]) == [0.0, 0.0, 1.0]
        # 1 base tip vertex + 8 base segment circle vertices + 1 base center vertex = 10 vertices
        assert len(positions) == 10
        assert len(indices) > 0

    def test_generate_axis_geometry_empty(self) -> None:
        """Verify generate_axis_geometry handles None/non-empty objects gracefully."""
        res = joint_gizmos.generate_axis_geometry(None)  # type: ignore
        assert res["lines"] == []
        assert res["line_colors"] == []

    def test_generate_axis_geometry_valid_joint(self, scene) -> None:
        """Verify generate_axis_geometry returns correct RGB shafts and cones."""
        joint_obj = bpy.data.objects.new("test_joint", None)
        scene.collection.objects.link(joint_obj)
        joint_obj.type = "EMPTY"
        safe_update(scene)

        res = joint_gizmos.generate_axis_geometry(joint_obj, axis_length=0.2)
        # 3 shafts (X, Y, Z lines) + 3 cones (triangles)
        assert len(res["lines"]) == 6  # 3 segments * 2 points
        assert len(res["line_colors"]) == 6
        assert len(res["tris"]) > 0

    def test_draw_joint_axes_preferences_and_rendering(self, scene) -> None:
        """Verify joint axes overlay rendering logic branches."""
        gpu = joint_gizmos.gpu

        # Scenario 1: Disabled
        with patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs:
            prefs = MagicMock()
            prefs.show_joint_axes = False
            mock_prefs.return_value = prefs

            assert joint_gizmos.draw_joint_axes() is None

        # Scenario 2: Enabled but no joints in statistics
        with (
            patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.joint_gizmos.get_robot_statistics"
            ) as mock_stats,
        ):
            prefs = MagicMock()
            prefs.show_joint_axes = True
            prefs.joint_empty_size = 0.2
            mock_prefs.return_value = prefs

            stats = MagicMock()
            stats.joint_objects = []
            mock_stats.return_value = stats

            assert joint_gizmos.draw_joint_axes() is None

        # Scenario 3: Enabled with active joint objects (full triangles / batch rendering draw)
        joint_obj = bpy.data.objects.new("rendered_joint", None)
        scene.collection.objects.link(joint_obj)
        joint_obj.type = "EMPTY"
        safe_update(scene)

        gpu.state.depth_test_set.reset_mock()
        gpu.state.blend_set.reset_mock()
        gpu.state.line_width_set.reset_mock()

        with (
            patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.joint_gizmos.get_robot_statistics"
            ) as mock_stats,
        ):
            prefs = MagicMock()
            prefs.show_joint_axes = True
            prefs.joint_empty_size = 0.3
            mock_prefs.return_value = prefs

            stats = MagicMock()
            stats.joint_objects = [joint_obj]
            mock_stats.return_value = stats

            joint_gizmos.draw_joint_axes()

            assert gpu.state.depth_test_set.called
            assert gpu.state.blend_set.called
            assert gpu.state.line_width_set.called

    def test_draw_joint_axes_handles_deleted_object_reference_error(self, scene) -> None:
        """Verify draw loop handles ReferenceError gracefully."""
        with (
            patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.joint_gizmos.get_robot_statistics"
            ) as mock_stats,
            patch(
                "linkforge.blender.visualization.joint_gizmos.generate_axis_geometry",
                side_effect=ReferenceError("Deleted"),
            ),
        ):
            prefs = MagicMock()
            prefs.show_joint_axes = True
            mock_prefs.return_value = prefs

            stats = MagicMock()
            stats.joint_objects = [MagicMock()]
            mock_stats.return_value = stats

            # Should not raise exception
            joint_gizmos.draw_joint_axes()

    def test_fix_existing_joints(self, scene) -> None:
        """Verify existing joint empty display type and size alignment."""
        joint_obj = bpy.data.objects.new("existing_joint", None)
        scene.collection.objects.link(joint_obj)
        joint_obj.type = "EMPTY"
        joint_obj.empty_display_type = "CONE"

        # Enable joint status
        jp = safe_get_joint(joint_obj)
        jp.is_robot_joint = True
        safe_update(scene)

        with patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs:
            prefs = MagicMock()
            prefs.joint_empty_size = 0.5
            mock_prefs.return_value = prefs

            joint_gizmos.fix_existing_joints()

            assert joint_obj.empty_display_type == "PLAIN_AXES"
            assert joint_obj.empty_display_size == 0.5

    def test_update_viz_handle_lifecycle(self, scene) -> None:
        """Verify SpaceView3D custom drawing handler addition and removal on viz updates."""
        # Case 1: Toggle ON
        with (
            patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs,
            patch.object(
                bpy.types.SpaceView3D, "draw_handler_add", return_value="dummy_joint_handle"
            ) as mock_add,
        ):
            prefs = MagicMock()
            prefs.show_joint_axes = True
            mock_prefs.return_value = prefs

            bpy.app.driver_namespace.clear()
            joint_gizmos.update_viz_handle(bpy.context)

            assert bpy.app.driver_namespace["linkforge_joint_gizmo_handler"] == "dummy_joint_handle"
            assert mock_add.called

        # Case 2: Toggle OFF
        with (
            patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs,
            patch.object(bpy.types.SpaceView3D, "draw_handler_remove") as mock_remove,
        ):
            prefs = MagicMock()
            prefs.show_joint_axes = False
            mock_prefs.return_value = prefs

            bpy.app.driver_namespace["linkforge_joint_gizmo_handler"] = "dummy_joint_handle"
            joint_gizmos.update_viz_handle(bpy.context)

            assert "linkforge_joint_gizmo_handler" not in bpy.app.driver_namespace
            assert mock_remove.called

    def test_register_unregister_lifecycle(self) -> None:
        """Verify joint visualization register/unregister post load and timer setups."""
        # Unregister existing first
        if joint_gizmos.fix_existing_joints in bpy.app.handlers.load_post:
            bpy.app.handlers.load_post.remove(joint_gizmos.fix_existing_joints)

        joint_gizmos.register()
        assert joint_gizmos.fix_existing_joints in bpy.app.handlers.load_post

        with patch.object(bpy.types.SpaceView3D, "draw_handler_remove") as mock_remove:
            bpy.app.driver_namespace["linkforge_joint_gizmo_handler"] = "dummy_handle"
            joint_gizmos.unregister()
            assert "linkforge_joint_gizmo_handler" not in bpy.app.driver_namespace
            assert mock_remove.called
            assert joint_gizmos.fix_existing_joints not in bpy.app.handlers.load_post


class TestVisualizationExtra:
    def test_inertia_gizmos_fallback_shader(self) -> None:
        """Verify fallback modern shader lookup for draw_inertia_gizmos."""
        orig_from_builtin = inertia_gizmos.gpu.shader.from_builtin
        mock_shader_obj = MagicMock()

        def custom_from_builtin(shader_name):
            if shader_name == "UNIFORM_COLOR":
                raise Exception("Not supported")
            elif shader_name == "3D_FLAT_COLOR":
                return mock_shader_obj
            return orig_from_builtin(shader_name)

        with (
            patch("linkforge.blender.visualization.inertia_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.get_robot_statistics"
            ) as mock_stats,
            patch("gpu.shader.from_builtin", side_effect=custom_from_builtin),
        ):
            prefs = MagicMock()
            prefs.show_inertia_gizmos = True
            mock_prefs.return_value = prefs

            stats = MagicMock()
            stats.link_objects = [MagicMock()]
            mock_stats.return_value = stats

            inertia_gizmos.draw_inertia_gizmos()

    def test_inertia_gizmos_draw_no_objects(self) -> None:
        """Verify draw_inertia_gizmos early exits if no link objects exist."""
        with (
            patch("linkforge.blender.visualization.inertia_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.get_robot_statistics"
            ) as mock_stats,
        ):
            prefs = MagicMock()
            prefs.show_inertia_gizmos = True
            mock_prefs.return_value = prefs

            stats = MagicMock()
            stats.link_objects = []
            mock_stats.return_value = stats

            assert inertia_gizmos.draw_inertia_gizmos() is None

    def test_inertia_gizmos_draw_fallback_prefs_error(self) -> None:
        """Verify draw_inertia_gizmos handles preferences lookup failures."""
        with patch(
            "linkforge.blender.visualization.inertia_gizmos.get_addon_prefs", side_effect=Exception
        ):
            assert inertia_gizmos.draw_inertia_gizmos() is None

    def test_inertia_gizmos_draw_exception_handling(self) -> None:
        """Verify draw_inertia_gizmos catches and logs general exceptions gracefully."""
        with (
            patch("linkforge.blender.visualization.inertia_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.get_robot_statistics",
                side_effect=ValueError("Test"),
            ),
        ):
            prefs = MagicMock()
            prefs.show_inertia_gizmos = True
            mock_prefs.return_value = prefs

            # Should not raise exception
            inertia_gizmos.draw_inertia_gizmos()

    def test_inertia_gizmos_tag_redraw_edge_cases(self) -> None:
        """Verify tag_redraw handles missing windows or screens."""

        class MockArea:
            type = "VIEW_3D"

            def __init__(self):
                self.redraw_called = False

            def tag_redraw(self):
                self.redraw_called = True

        class MockScreen:
            def __init__(self, areas):
                self.areas = areas

        class MockWindow:
            def __init__(self, screen):
                self.screen = screen

        class MockWindowManager:
            def __init__(self, windows):
                self.windows = windows

        class MockContext:
            def __init__(self, wm):
                self.window_manager = wm

        mock_area = MockArea()
        screen1 = MockScreen([mock_area])
        win1 = MockWindow(screen1)
        win2 = MockWindow(None)

        mock_wm = MockWindowManager([win1, win2])
        mock_ctx = MockContext(mock_wm)

        with patch("bpy.context", mock_ctx):
            inertia_gizmos.tag_redraw()
            assert mock_area.redraw_called

    def test_joint_gizmos_fallback_shader(self) -> None:
        """Verify fallback modern shader lookup for draw_joint_axes."""
        orig_from_builtin = joint_gizmos.gpu.shader.from_builtin
        mock_shader_obj = MagicMock()

        def custom_from_builtin(shader_name):
            if (
                shader_name == "POLYLINE_UNIFORM_COLOR"
                or shader_name == "3D_POLYLINE_UNIFORM_COLOR"
            ):
                raise Exception("Not supported")
            elif shader_name == "FLAT_COLOR":
                return mock_shader_obj
            return orig_from_builtin(shader_name)

        with (
            patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.joint_gizmos.get_robot_statistics"
            ) as mock_stats,
            patch("gpu.shader.from_builtin", side_effect=custom_from_builtin),
        ):
            prefs = MagicMock()
            prefs.show_joint_axes = True
            mock_prefs.return_value = prefs

            stats = MagicMock()
            stats.joint_objects = [MagicMock()]
            mock_stats.return_value = stats

            joint_gizmos.draw_joint_axes()

    def test_joint_gizmos_draw_no_region_data(self) -> None:
        """Verify draw_joint_axes returns early if region_data is missing."""
        mock_context = MagicMock()
        mock_context.region_data = None
        with patch("bpy.context", mock_context):
            assert joint_gizmos.draw_joint_axes() is None

    def test_joint_gizmos_draw_no_scene(self) -> None:
        """Verify draw_joint_axes returns early if scene is missing from context."""
        mock_context = MagicMock()
        mock_context.scene = None
        with patch("bpy.context", mock_context):
            assert joint_gizmos.draw_joint_axes() is None

    def test_joint_gizmos_fix_existing_joints_no_scene(self) -> None:
        """Verify fix_existing_joints handles missing scene/preferences cleanly."""

        class MockContextNoScene:
            preferences = MagicMock()
            scene = None

        with patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs:
            prefs = MagicMock()
            prefs.joint_empty_size = 0.5
            mock_prefs.return_value = prefs

            joint_gizmos.fix_existing_joints(MockContextNoScene())

    def test_joint_gizmos_fix_existing_joints_exceptions(self) -> None:
        """Verify fix_existing_joints catches exceptions during iteration."""
        mock_context = MagicMock()
        mock_context.scene.objects = None

        joint_gizmos.fix_existing_joints(mock_context)

    def test_joint_gizmos_main_entrypoint(self) -> None:
        """Test running joint_gizmos.py as __main__."""
        import runpy

        with (
            patch("bpy.app.handlers.load_post", []),
            patch("bpy.utils.register_class") as mock_reg,
        ):
            runpy.run_module("linkforge.blender.visualization.joint_gizmos", run_name="__main__")


class TestGizmosExtra:
    def test_inertia_gizmos_fallback_shader_extra(self) -> None:
        """Verify get_shader in inertia_gizmos falls back to 3D_FLAT_COLOR on exception."""
        inertia_gizmos._builtin_shader_name = None
        orig_from_builtin = inertia_gizmos.gpu.shader.from_builtin

        def mock_from_builtin(name):
            if name == "FLAT_COLOR":
                raise Exception("FLAT_COLOR not supported")
            return orig_from_builtin(name)

        with patch(
            "linkforge.blender.visualization.inertia_gizmos.gpu.shader.from_builtin",
            side_effect=mock_from_builtin,
        ):
            shader = inertia_gizmos.get_shader()
            assert inertia_gizmos._builtin_shader_name == "3D_FLAT_COLOR"

        # Reset caching
        inertia_gizmos._builtin_shader_name = None

    def test_inertia_gizmos_draw_no_prefs(self, scene) -> None:
        """Verify draw_inertia_gizmos handles None preferences gracefully (183->190)."""
        with (
            patch(
                "linkforge.blender.visualization.inertia_gizmos.get_addon_prefs", return_value=None
            ),
            patch(
                "linkforge.blender.visualization.inertia_gizmos.get_robot_statistics"
            ) as mock_stats,
        ):
            stats = MagicMock()
            stats.manual_inertia_objects = []
            mock_stats.return_value = stats

            # Should run without throwing
            inertia_gizmos.draw_inertia_gizmos()

    def test_inertia_gizmos_draw_empty_visible_objects(self) -> None:
        """Verify draw_inertia_gizmos exits early when context.visible_objects is empty (196)."""
        mock_ctx = MagicMock()
        mock_ctx.visible_objects = []
        with (
            patch("bpy.context", mock_ctx),
            patch("linkforge.blender.visualization.inertia_gizmos.get_addon_prefs") as mock_prefs,
        ):
            prefs = MagicMock()
            prefs.show_inertia_gizmos = True
            mock_prefs.return_value = prefs

            assert inertia_gizmos.draw_inertia_gizmos() is None

    def test_inertia_gizmos_draw_empty_lines_branch(self, scene) -> None:
        """Verify draw_inertia_gizmos handles empty generated lines branch (210->207)."""
        link_obj = create_robot_link("dummy_link", scene)
        with (
            patch("linkforge.blender.visualization.inertia_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.get_robot_statistics"
            ) as mock_stats,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.generate_inertia_axes_geometry",
                return_value={"lines": [], "line_colors": []},
            ),
        ):
            prefs = MagicMock()
            prefs.show_inertia_gizmos = True
            mock_prefs.return_value = prefs

            stats = MagicMock()
            stats.manual_inertia_objects = [link_obj]
            mock_stats.return_value = stats

            inertia_gizmos.draw_inertia_gizmos()

    def test_inertia_gizmos_tag_redraw_no_wm(self) -> None:
        """Verify tag_redraw returns early if window_manager is missing (249)."""

        class MockContextNoWM:
            window_manager = None

        with patch("bpy.context", MockContextNoWM()):
            assert inertia_gizmos.tag_redraw() is None

    def test_inertia_gizmos_tag_redraw_non_view3d(self) -> None:
        """Verify tag_redraw skips non VIEW_3D areas (256->255)."""

        class MockArea:
            type = "PROPERTIES"

        class MockScreen:
            areas = [MockArea()]

        class MockWindow:
            screen = MockScreen()

        class MockWM:
            windows = [MockWindow()]

        class MockCtx:
            window_manager = MockWM()

        with patch("bpy.context", MockCtx()):
            inertia_gizmos.tag_redraw()

    def test_inertia_gizmos_check_manual_inertia_scene_exception(self) -> None:
        """Verify check_manual_inertia_on_load handles scene exceptions gracefully (279-281)."""

        class BadContext:
            @property
            def scene(self):
                raise RuntimeError("No scene today")

        with patch("bpy.context", BadContext()):
            assert inertia_gizmos.check_manual_inertia_on_load() is None

    def test_inertia_gizmos_check_manual_inertia_scene_none(self) -> None:
        """Verify check_manual_inertia_on_load when scene is None (279)."""

        class MockCtxNoScene:
            scene = None

        with patch("bpy.context", MockCtxNoScene()):
            assert inertia_gizmos.check_manual_inertia_on_load() is None

    def test_inertia_gizmos_registration_coverage(self) -> None:
        """Verify register/unregister double call branches (294->300, 308->312, 312->exit)."""
        # Register when already registered
        inertia_gizmos.register()
        inertia_gizmos.register()

        # Test ensure_inertia_handler twice to hit _draw_handle is not None branch
        inertia_gizmos.ensure_inertia_handler()
        inertia_gizmos.ensure_inertia_handler()

        # Test unregister when check_manual_inertia_on_load is not in load_post
        if inertia_gizmos.check_manual_inertia_on_load in bpy.app.handlers.load_post:
            bpy.app.handlers.load_post.remove(inertia_gizmos.check_manual_inertia_on_load)

        # Unregister when _draw_handle is None
        inertia_gizmos._draw_handle = None
        inertia_gizmos.unregister()

        # Restore state
        inertia_gizmos.register()

    def test_joint_gizmos_fallback_shader_extra(self) -> None:
        """Verify get_shader in joint_gizmos falls back to 3D_FLAT_COLOR on exception."""
        joint_gizmos._builtin_shader_name = None
        orig_from_builtin = joint_gizmos.gpu.shader.from_builtin

        def mock_from_builtin(name):
            if name == "FLAT_COLOR":
                raise Exception("FLAT_COLOR not supported")
            return orig_from_builtin(name)

        with patch(
            "linkforge.blender.visualization.joint_gizmos.gpu.shader.from_builtin",
            side_effect=mock_from_builtin,
        ):
            shader = joint_gizmos.get_shader()
            assert joint_gizmos._builtin_shader_name == "3D_FLAT_COLOR"

        # Reset caching
        joint_gizmos._builtin_shader_name = None

    def test_joint_gizmos_draw_no_prefs(self) -> None:
        """Verify draw_joint_axes handles Falsy preferences gracefully (200->204)."""
        mock_ctx = MagicMock()
        mock_ctx.scene = None
        with (
            patch("bpy.context", mock_ctx),
            patch(
                "linkforge.blender.visualization.joint_gizmos.get_addon_prefs", return_value=None
            ),
        ):
            joint_gizmos.draw_joint_axes()

    def test_joint_gizmos_draw_empty_tris_only(self) -> None:
        """Verify draw_joint_axes with empty tris branch (259->273)."""
        mock_ctx = MagicMock()
        mock_scene = MagicMock()
        mock_ctx.scene = mock_scene
        mock_ctx.region_data = MagicMock()

        with (
            patch("bpy.context", mock_ctx),
            patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs,
            patch(
                "linkforge.blender.visualization.joint_gizmos.get_robot_statistics"
            ) as mock_stats,
            patch(
                "linkforge.blender.visualization.joint_gizmos.generate_axis_geometry"
            ) as mock_gen,
        ):
            prefs = MagicMock()
            prefs.show_joint_axes = True
            mock_prefs.return_value = prefs

            stats = MagicMock()
            stats.joint_objects = [MagicMock()]
            mock_stats.return_value = stats

            # Lines exist, but tris are empty
            mock_gen.return_value = {
                "lines": [Vector((0, 0, 0)), Vector((0, 0, 1))],
                "line_colors": [(1, 1, 1, 1), (1, 1, 1, 1)],
                "tris": [],
                "tri_colors": [],
            }

            joint_gizmos.draw_joint_axes()

    def test_joint_gizmos_fix_existing_joints_no_scene_attr(self) -> None:
        """Verify fix_existing_joints handles scene AttributeError gracefully (288-289)."""

        class BadContext:
            @property
            def scene(self):
                raise AttributeError("No scene")

        with patch("bpy.context", BadContext()):
            joint_gizmos.fix_existing_joints()

    def test_joint_gizmos_fix_existing_joints_no_addon_prefs(self) -> None:
        """Verify fix_existing_joints handles None addon preferences branch (294->297)."""

        class MockScene:
            objects = []

        class MockCtx:
            scene = MockScene()

        with (
            patch(
                "linkforge.blender.visualization.joint_gizmos.get_addon_prefs", return_value=None
            ),
            patch("bpy.context", MockCtx()),
        ):
            joint_gizmos.fix_existing_joints()

    def test_joint_gizmos_fix_existing_joints_none_scene(self) -> None:
        """Verify fix_existing_joints exits early if scene is None (298)."""

        class MockCtx:
            scene = None

        with (
            patch(
                "linkforge.blender.visualization.joint_gizmos.get_addon_prefs",
                return_value=MagicMock(),
            ),
            patch("bpy.context", MockCtx()),
        ):
            joint_gizmos.fix_existing_joints()

    def test_joint_gizmos_update_viz_handle_no_wm(self) -> None:
        """Verify update_viz_handle exits early when window_manager is None (381->exit)."""

        class MockContextNoWM:
            window_manager = None
            preferences = None

        joint_gizmos.update_viz_handle(MockContextNoWM())

    def test_joint_gizmos_update_viz_handle_redundant(self) -> None:
        """Verify update_viz_handle branch when show_axes is True and current_handler is not None (370->381)."""

        class MockArea:
            type = "VIEW_3D"

            def tag_redraw(self):
                pass

        class MockScreen:
            areas = [MockArea()]

        class MockWindow:
            screen = MockScreen()

        class MockWM:
            windows = [MockWindow()]

        class MockCtx:
            window_manager = MockWM()
            preferences = MagicMock()

        with (
            patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs,
            patch.dict(bpy.app.driver_namespace, {"linkforge_joint_gizmo_handler": MagicMock()}),
        ):
            prefs = MagicMock()
            prefs.show_joint_axes = True
            mock_prefs.return_value = prefs

            joint_gizmos.update_viz_handle(MockCtx())

    def test_joint_gizmos_update_viz_handle_not_in_namespace(self) -> None:
        """Verify update_viz_handle branch when handler is removed but not in namespace dictionary (377->381)."""

        class MockArea:
            type = "VIEW_3D"

            def tag_redraw(self):
                pass

        class MockScreen:
            areas = [MockArea()]

        class MockWindow:
            screen = MockScreen()

        class MockWM:
            windows = [MockWindow()]

        class MockCtx:
            window_manager = MockWM()
            preferences = MagicMock()

        mock_handler = MagicMock()

        # Mock dictionary to return key during get, but claim it doesn't contain it in "in" check
        class TrickDict(dict):
            def __contains__(self, key):
                return False

        trick_ns = TrickDict({"linkforge_joint_gizmo_handler": mock_handler})

        with (
            patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs,
            patch("bpy.types.SpaceView3D.draw_handler_remove"),
            patch("bpy.app.driver_namespace", trick_ns),
        ):
            prefs = MagicMock()
            prefs.show_joint_axes = False
            mock_prefs.return_value = prefs

            joint_gizmos.update_viz_handle(MockCtx())

    def test_joint_gizmos_update_viz_handle_redraw(self) -> None:
        """Verify update_viz_handle tags 3D areas for redraw (383-385)."""

        class MockArea:
            type = "VIEW_3D"

            def __init__(self):
                self.redraw_called = False

            def tag_redraw(self):
                self.redraw_called = True

        mock_area = MockArea()

        class MockScreen:
            areas = [mock_area]

        class MockWindow:
            screen = MockScreen()

        class MockWM:
            windows = [MockWindow()]

        class MockCtx:
            window_manager = MockWM()
            preferences = MagicMock()

        with patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs:
            prefs = MagicMock()
            prefs.show_joint_axes = True
            mock_prefs.return_value = prefs

            joint_gizmos.update_viz_handle(MockCtx())
            assert mock_area.redraw_called

    def test_joint_gizmos_registration_coverage(self) -> None:
        """Verify register/unregister double call and unregister when handler is None (325->329, 335->339, 340->exit, 347->exit)."""
        # Call register twice to test fix_existing_joints already in load_post
        joint_gizmos.register()
        joint_gizmos.register()

        # Test unregister when fix_existing_joints is not in load_post
        if joint_gizmos.fix_existing_joints in bpy.app.handlers.load_post:
            bpy.app.handlers.load_post.remove(joint_gizmos.fix_existing_joints)

        # Mock driver_namespace to have no handler
        with patch.dict(bpy.app.driver_namespace, {}, clear=True):
            joint_gizmos.unregister()

        # Test unregister when handler is not None but not in namespace dictionary during deletion
        mock_handler = MagicMock()

        class TrickDict(dict):
            def __contains__(self, key):
                return False

        trick_ns = TrickDict({"linkforge_joint_gizmo_handler": mock_handler})
        with (
            patch("bpy.types.SpaceView3D.draw_handler_remove"),
            patch("bpy.app.driver_namespace", trick_ns),
        ):
            joint_gizmos.unregister()

    def test_joint_gizmos_update_viz_handle_non_view3d_area(self) -> None:
        """Verify update_viz_handle when an area is not VIEW_3D (383->382)."""

        class MockArea:
            type = "PROPERTIES"

            def tag_redraw(self):
                pass

        class MockScreen:
            areas = [MockArea()]

        class MockWindow:
            screen = MockScreen()

        class MockWM:
            windows = [MockWindow()]

        class MockCtx:
            window_manager = MockWM()
            preferences = MagicMock()

        with patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs") as mock_prefs:
            prefs = MagicMock()
            prefs.show_joint_axes = True
            mock_prefs.return_value = prefs

            joint_gizmos.update_viz_handle(MockCtx())
