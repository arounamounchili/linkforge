"""Unit tests for display operators, bounding box auto-fit, and viewport overlay UI."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import bpy
from linkforge.blender.constants import PROP_ROBOT
from linkforge.blender.operators.display_ops import (
    LINKFORGE_OT_auto_fit_gizmo_sizes,
    LINKFORGE_OT_toggle_collisions,
    register,
    unregister,
)
from linkforge.blender.panels.forge_panel import LINKFORGE_PT_forge
from linkforge.blender.utils.property_helpers import get_link_props
from linkforge.blender.utils.scene_utils import auto_fit_robot_gizmos, calculate_robot_bounds
from mathutils import Vector


class TestDisplayOperators:
    """Test suite for LinkForge display operators."""

    def test_auto_fit_operator_no_scene(self) -> None:
        """Verify auto_fit returns CANCELLED when no scene exists."""
        op = LINKFORGE_OT_auto_fit_gizmo_sizes()
        context = MagicMock()
        context.scene = None

        result = op.execute(context)
        assert result == {"CANCELLED"}

    def test_auto_fit_operator_no_robot(self, scene: bpy.types.Scene) -> None:
        """Verify auto_fit returns CANCELLED when no robot links are present."""
        op = LINKFORGE_OT_auto_fit_gizmo_sizes()
        context = MagicMock()
        context.scene = scene

        result = op.execute(context)
        assert result == {"CANCELLED"}

    def test_auto_fit_operator_success(self, scene: bpy.types.Scene) -> None:
        """Verify auto_fit executes successfully when a robot is present."""
        link_obj = bpy.data.objects.new("test_link", None)
        scene.collection.objects.link(link_obj)
        link_obj.type = "EMPTY"
        link_props = get_link_props(link_obj)
        link_props.is_robot_link = True
        link_obj.location = (1.0, 2.0, 3.0)

        op = LINKFORGE_OT_auto_fit_gizmo_sizes()
        context = MagicMock()
        context.scene = scene

        with (
            patch("linkforge.blender.preferences.get_addon_prefs") as mock_prefs,
            patch("linkforge.blender.preferences.update_joint_empty_size"),
            patch("linkforge.blender.preferences.update_link_empty_size"),
            patch("linkforge.blender.preferences.update_sensor_empty_size"),
            patch("linkforge.blender.preferences.update_inertia_size"),
        ):
            prefs = MagicMock()
            mock_prefs.return_value = prefs
            result = op.execute(context)
            assert result == {"FINISHED"}
            assert prefs.joint_empty_size > 0

    def test_toggle_collisions_operator(self, scene: bpy.types.Scene) -> None:
        """Verify toggle_collisions flips show_collisions on the scene."""
        op = LINKFORGE_OT_toggle_collisions()
        context = MagicMock()
        context.scene = scene

        robot_props = getattr(scene, PROP_ROBOT)
        assert robot_props.show_collisions is False

        # First toggle -> True
        result = op.execute(context)
        assert result == {"FINISHED"}
        assert robot_props.show_collisions is True

        # Second toggle -> False
        result = op.execute(context)
        assert result == {"FINISHED"}
        assert robot_props.show_collisions is False

    def test_toggle_collisions_no_scene(self) -> None:
        """Verify toggle_collisions handles null scene gracefully."""
        op = LINKFORGE_OT_toggle_collisions()
        context = MagicMock()
        context.scene = None

        result = op.execute(context)
        assert result == {"CANCELLED"}

    def test_register_unregister_display_ops(self) -> None:
        """Verify registration and unregistration of display operators."""
        # Multiple calls should not raise exceptions
        register()
        unregister()

    def test_register_handles_value_error(self) -> None:
        """Verify register handles ValueError when class is already registered."""
        with (
            patch("bpy.utils.register_class") as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            # First register call raises ValueError, subsequent call succeeds
            mock_reg.side_effect = [ValueError("already registered"), None, None]
            register()
            assert mock_unreg.call_count >= 1


class TestAutoFitHeuristics:
    """Test suite for calculate_robot_bounds and auto_fit_robot_gizmos."""

    def test_calculate_robot_bounds_empty(self) -> None:
        """Verify bounds calculation handles null or empty scenes."""
        assert calculate_robot_bounds(None) is None
        empty_scene = MagicMock()
        empty_scene.objects = []
        assert calculate_robot_bounds(empty_scene) is None

    def test_calculate_robot_bounds_with_meshes(self, scene: bpy.types.Scene) -> None:
        """Verify bounds calculation includes child mesh bounding boxes."""
        link_obj = bpy.data.objects.new("base_link", None)
        scene.collection.objects.link(link_obj)
        link_props = get_link_props(link_obj)
        link_props.is_robot_link = True
        link_obj.location = (0.0, 0.0, 0.0)

        # Add child visual mesh with bound box
        mesh_child = bpy.data.objects.new("base_link_visual", None)
        mesh_child.type = "MESH"
        mesh_child.parent = link_obj
        mesh_child.bound_box = [
            (-0.5, -0.5, -0.5),
            (-0.5, -0.5, 0.5),
            (-0.5, 0.5, 0.5),
            (-0.5, 0.5, -0.5),
            (0.5, -0.5, -0.5),
            (0.5, -0.5, 0.5),
            (0.5, 0.5, 0.5),
            (0.5, 0.5, -0.5),
        ]
        scene.collection.objects.link(mesh_child)

        bounds = calculate_robot_bounds(scene)
        assert bounds is not None
        min_corner, max_corner, diagonal = bounds
        assert diagonal > 0.0

    def test_auto_fit_robot_gizmos_fallback(self, scene: bpy.types.Scene) -> None:
        """Verify auto_fit falls back to default size when diagonal is near zero."""
        link_obj = bpy.data.objects.new("point_link", None)
        scene.collection.objects.link(link_obj)
        link_props = get_link_props(link_obj)
        link_props.is_robot_link = True

        with (
            patch("linkforge.blender.utils.scene_utils.calculate_robot_bounds") as mock_calc,
            patch("linkforge.blender.preferences.get_addon_prefs") as mock_prefs,
        ):
            mock_calc.return_value = (Vector((0, 0, 0)), Vector((0, 0, 0)), 0.0001)
            prefs = MagicMock()
            mock_prefs.return_value = prefs

            result = auto_fit_robot_gizmos(scene)
            assert result is not None
            recommended_size, _ = result
            assert recommended_size == 0.05

    def test_auto_fit_robot_gizmos_proportional_and_clamping(self, scene: bpy.types.Scene) -> None:
        """Verify auto_fit scales proportionally and clamps to min/max boundaries."""
        with (
            patch("linkforge.blender.utils.scene_utils.calculate_robot_bounds") as mock_calc,
            patch("linkforge.blender.preferences.get_addon_prefs") as mock_prefs,
            patch("linkforge.blender.preferences.update_joint_empty_size"),
            patch("linkforge.blender.preferences.update_link_empty_size"),
            patch("linkforge.blender.preferences.update_sensor_empty_size"),
            patch("linkforge.blender.preferences.update_inertia_size"),
        ):
            prefs = MagicMock()
            mock_prefs.return_value = prefs

            # Normal size (2.0m diagonal -> 2.0 * 0.05 = 0.10m)
            mock_calc.return_value = (Vector((0, 0, 0)), Vector((2, 0, 0)), 2.0)
            res = auto_fit_robot_gizmos(scene)
            assert res is not None
            assert round(res[0], 2) == 0.10
            assert prefs.joint_empty_size == res[0]

            # Tiny robot (1cm diagonal -> raw 0.0005m -> clamped to 0.005m)
            mock_calc.return_value = (Vector((0, 0, 0)), Vector((0.01, 0, 0)), 0.01)
            res_tiny = auto_fit_robot_gizmos(scene)
            assert res_tiny is not None
            assert res_tiny[0] == 0.005

            # Huge robot (50m diagonal -> raw 2.5m -> clamped to 0.5m)
            mock_calc.return_value = (Vector((0, 0, 0)), Vector((50, 0, 0)), 50.0)
            res_huge = auto_fit_robot_gizmos(scene)
            assert res_huge is not None
            assert res_huge[0] == 0.5

    def test_auto_fit_robot_gizmos_none_bounds_and_no_prefs(self, scene: bpy.types.Scene) -> None:
        """Verify auto_fit returns None on missing bounds and works when prefs is None."""
        with patch("linkforge.blender.utils.scene_utils.calculate_robot_bounds") as mock_calc:
            mock_calc.return_value = None
            assert auto_fit_robot_gizmos(scene) is None

        with (
            patch("linkforge.blender.utils.scene_utils.calculate_robot_bounds") as mock_calc,
            patch("linkforge.blender.preferences.get_addon_prefs") as mock_prefs,
        ):
            mock_calc.return_value = (Vector((0, 0, 0)), Vector((1, 0, 0)), 1.0)
            mock_prefs.return_value = None
            res = auto_fit_robot_gizmos(scene)
            assert res is not None
            assert res[0] == 0.05

    def test_calculate_robot_bounds_child_without_bound_box(self, scene: bpy.types.Scene) -> None:
        """Verify calculate_robot_bounds falls back to translation when bound_box is missing."""
        link_obj = bpy.data.objects.new("link_with_empty_child", None)
        scene.collection.objects.link(link_obj)
        link_props = get_link_props(link_obj)
        link_props.is_robot_link = True
        link_obj.location = (0.0, 0.0, 0.0)

        # Child without bound_box attribute (plain empty)
        child_empty = bpy.data.objects.new("child_empty", None)
        child_empty.parent = link_obj
        child_empty.location = (1.0, 2.0, 3.0)
        scene.collection.objects.link(child_empty)

        bounds = calculate_robot_bounds(scene)
        assert bounds is not None
        _, _, diagonal = bounds
        assert diagonal > 0.0

    def test_calculate_robot_bounds_with_none_in_stats(self, scene: bpy.types.Scene) -> None:
        """Verify calculate_robot_bounds handles None entries in link_objects."""
        with patch("linkforge.blender.utils.scene_utils.get_robot_statistics") as mock_stats:
            stats = MagicMock()
            stats.link_objects = {"link1": None}
            mock_stats.return_value = stats
            assert calculate_robot_bounds(scene) is None


class TestForgePanelDisplayUI:
    """Test suite for the embedded Viewport Overlays section in Forge panel."""

    def test_forge_panel_draw_overlays(self, scene: bpy.types.Scene) -> None:
        """Verify Forge panel renders the Viewport Overlays controls."""
        panel = LINKFORGE_PT_forge()
        context = MagicMock()
        context.scene = scene

        # Layout mock to collect calls
        class MockLayout:
            def __init__(self) -> None:
                self.box_count = 0
                self.row_count = 0
                self.operators = []
                self.props = []
                self.labels = []
                self.scale_y = 1.0

            def box(self) -> MockLayout:
                self.box_count += 1
                return self

            def row(self, align: bool = False) -> MockLayout:
                self.row_count += 1
                return self

            def separator(self) -> None:
                pass

            def label(self, text: str = "", icon: str = "NONE") -> None:
                self.labels.append(text)

            def prop(self, data: object, property_name: str, **kwargs: object) -> None:
                self.props.append(property_name)

            def operator(self, op_name: str, **kwargs: object) -> None:
                self.operators.append(op_name)

        mock_layout = MockLayout()
        panel.layout = mock_layout

        with patch("linkforge.blender.panels.forge_panel.get_addon_prefs") as mock_prefs:
            prefs = MagicMock()
            mock_prefs.return_value = prefs

            panel.draw(context)

            assert "linkforge.auto_fit_gizmo_sizes" in mock_layout.operators
            assert "show_collisions" in mock_layout.props
            assert "show_joint_axes" in mock_layout.props
            assert "show_inertia_gizmos" in mock_layout.props
            assert "joint_empty_size" in mock_layout.props
