"""Unit tests for Blender Physics, Inertia gizmos, and Inertial origin."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest
from linkforge.blender.adapters.translator import LinkTranslator
from linkforge.core import RobotBuilder

from tests.blender_test_utils import create_test_object, safe_get_linkforge

# Inertia Visualization (Gizmos)


class TestInertiaGizmos:
    def test_draw_inertia_gizmos_disabled(self, mocker, scene, blender_context) -> None:
        """Test draw function when gizmos are disabled in preferences."""
        mock_prefs = MagicMock()
        mock_prefs.show_inertia_gizmos = False
        mocker.patch(
            "linkforge.blender.visualization.inertia_gizmos.get_addon_prefs",
            return_value=mock_prefs,
        )

        mock_batch = mocker.patch("linkforge.blender.visualization.inertia_gizmos.batch_for_shader")

        from linkforge.blender.visualization.inertia_gizmos import draw_inertia_gizmos

        draw_inertia_gizmos()

        mock_batch.assert_not_called()

    def test_ensure_inertia_handler(self, mocker, scene, blender_context) -> None:
        """Test inertia draw handler registration."""
        mock_add = mocker.patch("bpy.types.SpaceView3D.draw_handler_add")
        mocker.patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw")

        import linkforge.blender.visualization.inertia_gizmos as ig_module

        ig_module._draw_handle = None  # Reset

        from linkforge.blender.visualization.inertia_gizmos import ensure_inertia_handler

        ensure_inertia_handler()

        mock_add.assert_called_once()


# Inertial Origin Extraction


class TestInertialOrigin:
    def test_inertial_origin_extraction(self, scene, blender_context) -> None:
        """Verify that inertial origin properties are correctly converted."""
        obj = create_test_object("test_link", None, scene)

        props = safe_get_linkforge(obj)
        props.is_robot_link = True
        props.mass = 1.0
        props.use_auto_inertia = False
        from mathutils import Vector

        props.inertia_origin_xyz = Vector((1.2, 3.4, 5.6))
        props.inertia_origin_rpy = Vector((0.1, 0.2, 0.3))
        # Set minimal valid inertia
        props.inertia_ixx = 1.0
        props.inertia_iyy = 1.0
        props.inertia_izz = 1.0

        builder = RobotBuilder("test_robot")
        lb = LinkTranslator().translate(obj, builder, blender_context)
        if lb:
            lb.commit()

        link = builder.robot.get_link("test_link")
        assert link is not None, "Link 'test_link' not found in robot model"
        assert link.inertial is not None
        assert pytest.approx(link.inertial.origin.xyz.x) == 1.2
        assert pytest.approx(link.inertial.origin.rpy.z) == 0.3


class TestSimulationProperties:
    """Tests for advanced simulation physics properties."""

    def test_scientific_proxies(self) -> None:
        """Test scientific notation conversion logic."""

        # Create a mock instance (dataclass-like behavior for testing logic)
        class MockProps:
            def __init__(self):
                self.kp = 0.0
                self.kd = 0.0

        from typing import Any

        mock: Any = MockProps()
        from linkforge.blender.properties.link_props import (
            get_kd_scientific,
            get_kp_scientific,
            set_kd_scientific,
            set_kp_scientific,
        )

        # Test KP (Large value)
        mock.kp = 1.0e12
        assert get_kp_scientific(mock) == "1.00e+12"

        set_kp_scientific(mock, "5.5e+09")
        assert mock.kp == 5.5e9
        assert get_kp_scientific(mock) == "5.50e+09"

        # Test KD (Small value)
        mock.kd = 0.01
        assert get_kd_scientific(mock) == "1.00e-02"

        set_kd_scientific(mock, "1.0")
        assert mock.kd == 1.0
        assert get_kd_scientific(mock) == "1.00e+00"
