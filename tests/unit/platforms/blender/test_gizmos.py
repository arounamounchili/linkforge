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
