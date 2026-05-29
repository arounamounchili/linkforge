"""Integration tests for the robot validation workflow in Blender."""

from __future__ import annotations

import unittest.mock

import bpy
from linkforge.blender.constants import PROP_VALIDATION
from linkforge.blender.operators.export_ops import LINKFORGE_OT_validate_robot

from tests.blender_test_utils import (
    create_test_object,
    safe_get_linkforge,
    safe_get_linkforge_scene,
)


class TestValidationWorkflow:
    def test_disconnected_links_validation_workflow(self, blender_clean_scene) -> None:
        """Verify that a scene with disconnected links correctly triggers validation failures via the validation operator."""
        scene = bpy.context.scene

        # Set up robot name
        robot_props = safe_get_linkforge_scene(scene)
        robot_props.robot_name = "my_disconnected_robot"

        # Setup base_link (root)
        base_obj = create_test_object("base_link", None, scene)
        safe_get_linkforge(base_obj).is_robot_link = True
        safe_get_linkforge(base_obj).link_name = "base_link"

        # Setup a disconnected link
        island_obj = create_test_object("island_link", None, scene)
        safe_get_linkforge(island_obj).is_robot_link = True
        safe_get_linkforge(island_obj).link_name = "island_link"

        # Setup validation property group on window manager
        wm = bpy.context.window_manager
        assert wm is not None

        # Run validation operator
        op = LINKFORGE_OT_validate_robot()
        op.report = unittest.mock.MagicMock()

        result = op.execute(bpy.context)

        # The operator execution should fail (CANCELLED) because there are disconnected links (multiple roots)
        assert result == {"CANCELLED"}

        # Assert that the window manager's validation properties are correctly populated
        validation_props = getattr(wm, PROP_VALIDATION)
        assert validation_props.has_results is True
        assert validation_props.is_valid is False
        assert validation_props.error_count == 1

        # The error should be related to Multiple root links
        assert len(validation_props.errors) == 1
        error_item = validation_props.errors[0]
        assert "Multiple root links" in error_item.title
        assert "base_link" in error_item.message or "island_link" in error_item.message
