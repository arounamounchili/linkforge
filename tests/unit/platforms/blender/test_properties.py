"""Unit tests for Blender Properties, Validation, and Preferences."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.preferences import (
    LinkForgePreferences,
    get_addon_id,
    get_addon_prefs,
    update_inertia_size,
    update_inertia_visibility,
    update_joint_empty_size,
    update_link_empty_size,
    update_sensor_empty_size,
)
from linkforge.blender.preferences import (
    register as prefs_register,
)
from linkforge.blender.preferences import (
    unregister as prefs_unregister,
)
from linkforge.blender.utils.property_helpers import (
    find_property_owner,
    get_joint_props,
    get_link_props,
    get_robot_props,
    get_sensor_props,
)

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_sensor,
    safe_get_validation,
)

# Property Helpers


class TestPropertyHelpers:
    def test_find_property_owner(self, scene, blender_context) -> None:
        """Test finding the owner object of a PropertyGroup."""
        obj = create_test_object("test_owner", None, scene)
        props = safe_get_linkforge(obj)

        owner = find_property_owner(bpy.context, props, "linkforge")
        assert owner == obj

    def test_get_props_edge_cases(self, scene, blender_context) -> None:
        """Test edge cases for get_*_props helpers."""
        assert get_joint_props(None) is None
        assert get_link_props(None) is None
        assert get_sensor_props(None) is None
        assert get_robot_props(None) is None


# Validation Properties


class TestValidationProperties:
    def test_validation_issue_line_splitting(self, scene, blender_context) -> None:
        """Test correctly splitting long messages and suggestions into lines."""
        wm = bpy.context.window_manager
        res = safe_get_validation(wm)
        res.clear()

        err = res.errors.add()
        err.message = "This is a very long message that should be split into multiple lines."

        # Verify splitting logic (assuming 60 chars limit)
        lines = err.message_lines
        assert len(lines) >= 1
        for line in lines:
            assert len(line) <= 60

    def test_validation_result_clearing(self, scene, blender_context) -> None:
        """Test clearing validation results."""
        wm = bpy.context.window_manager
        res = safe_get_validation(wm)
        res.has_results = True
        res.clear()
        assert res.has_results is False

    def test_validation_issue_properties(self, scene, blender_context) -> None:
        """Test the properties and helper methods of ValidationIssueProperty and ValidationResultProperty."""
        wm = bpy.context.window_manager
        res = safe_get_validation(wm)
        res.clear()

        # Add error
        err = res.errors.add()
        err.title = "Test Error"
        err.message = "Error message"
        err.suggestion = "Do this to fix it"
        err.affected_objects = "obj1,obj2"

        assert err.has_suggestion is True
        assert err.has_objects is True
        assert err.objects_str == "obj1,obj2"
        assert len(err.suggestion_lines) >= 1

        # Add warning
        warn = res.warnings.add()
        warn.title = "Test Warning"
        warn.message = "Warning message"
        warn.suggestion = ""
        warn.affected_objects = ""

        assert warn.has_suggestion is False
        assert warn.has_objects is False
        assert warn.objects_str == ""
        assert warn.suggestion_lines == []

        # Test index getters
        res.error_count = 1
        res.warning_count = 1
        assert res.get_error(0) == err
        assert res.get_warning(0) == warn

    def test_validation_properties_registration(self) -> None:
        """Test register and unregister functions for validation properties."""
        from linkforge.blender.properties.validation_props import (
            register as val_register,
        )
        from linkforge.blender.properties.validation_props import (
            unregister as val_unregister,
        )

        with (
            patch("bpy.utils.register_class") as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            val_register()
            assert mock_reg.called

            val_unregister()
            assert mock_unreg.called


# Addon Preferences


class TestPreferences:
    def test_update_joint_empty_size(self, scene, blender_context) -> None:
        """Test that updating joint size in prefs affects scene objects."""
        obj = create_test_object("test_joint_size", None, scene)

        # Ensure we are testing the linkforge joint props
        safe_get_joint(obj).is_robot_joint = True
        obj.empty_display_size = 0.1

        mock_prefs = MagicMock()
        mock_prefs.joint_empty_size = 0.5

        with patch("linkforge.blender.visualization.joint_gizmos.update_viz_handle"):
            update_joint_empty_size(mock_prefs, bpy.context)

        assert obj.empty_display_size == pytest.approx(0.5)

    def test_update_sensor_empty_size(self, scene, blender_context) -> None:
        """Test that updating sensor empty size in prefs affects scene objects."""
        obj = create_test_object("test_sensor_size", None, scene)
        safe_get_sensor(obj).is_robot_sensor = True
        obj.empty_display_size = 0.1

        mock_prefs = MagicMock()
        mock_prefs.sensor_empty_size = 0.6

        update_sensor_empty_size(mock_prefs, bpy.context)
        assert obj.empty_display_size == pytest.approx(0.6)

    def test_update_link_empty_size(self, scene, blender_context) -> None:
        """Test that updating link empty size in prefs affects scene objects."""
        obj = create_test_object("test_link_size", None, scene)
        safe_get_linkforge(obj).is_robot_link = True
        obj.empty_display_size = 0.1

        mock_prefs = MagicMock()
        mock_prefs.link_empty_size = 0.7

        update_link_empty_size(mock_prefs, bpy.context)
        assert obj.empty_display_size == pytest.approx(0.7)

    def test_update_inertia_visibility_and_size(self, scene, blender_context) -> None:
        """Test that updating inertia visibility and size tags redraw."""
        mock_prefs = MagicMock()
        mock_prefs.show_inertia_gizmos = True

        with (
            patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw") as mock_redraw,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.ensure_inertia_handler"
            ) as mock_ensure,
        ):
            update_inertia_visibility(mock_prefs, bpy.context)
            mock_redraw.assert_called_once()
            mock_ensure.assert_called_once()

        with patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw") as mock_redraw:
            update_inertia_size(mock_prefs, bpy.context)
            mock_redraw.assert_called_once()

    def test_get_addon_id_and_prefs(self, scene, blender_context) -> None:
        """Test resolving addon ID and retrieving preferences."""
        addon_id = get_addon_id()
        assert addon_id == "linkforge"
        assert bpy.context.preferences is not None

        # Mock context.preferences.addons.get to return None so get_addon_prefs returns None
        with patch.object(bpy.context.preferences.addons, "get", return_value=None):
            prefs = get_addon_prefs(bpy.context)
            assert prefs is None

        # Mock context.preferences.addons.get to return an addon with preferences
        mock_addon = MagicMock()
        mock_prefs = LinkForgePreferences()
        mock_addon.preferences = mock_prefs
        with patch.object(bpy.context.preferences.addons, "get", return_value=mock_addon):
            prefs = get_addon_prefs(bpy.context)
            assert prefs == mock_prefs

    def test_preferences_draw(self, scene, blender_context) -> None:
        """Test drawing the preferences layout."""
        prefs = LinkForgePreferences()
        prefs.layout = MagicMock()
        prefs.show_inertia_gizmos = True
        prefs.show_joint_axes = True

        # Call draw, shouldn't raise any exception
        prefs.draw(bpy.context)
        assert prefs.layout.box.called

    def test_preferences_registration(self) -> None:
        """Test register and unregister functions for preferences."""
        with (
            patch("bpy.utils.register_class") as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            prefs_register()
            assert mock_reg.called

            prefs_unregister()
            assert mock_unreg.called
