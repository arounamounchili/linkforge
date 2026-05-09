"""Unit tests for Blender Properties, Validation, and Preferences."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.preferences import (
    update_joint_empty_size,
)
from linkforge.blender.utils.property_helpers import find_property_owner

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
