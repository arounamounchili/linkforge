"""Unit tests for LinkForge Blender UI panels."""

from __future__ import annotations

from unittest.mock import MagicMock

import bpy
from linkforge.blender.constants import PROP_VALIDATION
from linkforge.blender.panels.export_panel import LINKFORGE_PT_export_panel
from linkforge.blender.panels.forge_panel import LINKFORGE_PT_forge
from linkforge.blender.panels.joint_panel import LINKFORGE_PT_joints
from linkforge.blender.panels.link_panel import LINKFORGE_PT_links
from linkforge.blender.panels.sensor_panel import LINKFORGE_PT_perceive

from tests.blender_test_utils import create_robot_link


class TestExportPanel:
    def test_export_panel_draw_no_links(self, scene, blender_context) -> None:
        """Test drawing the export panel when no links are present."""
        panel = LINKFORGE_PT_export_panel()
        mock_layout = MagicMock()
        panel.layout = mock_layout

        panel.draw(bpy.context)

        # Should show "No robot in scene" when empty
        mock_layout.box.return_value.label.assert_any_call(text="No robot in scene", icon="INFO")

    def test_export_panel_draw_with_links(self, scene, blender_context) -> None:
        """Test drawing the export panel when links are present."""
        # Create a link to trigger the main UI
        create_robot_link("base_link", scene)

        panel = LINKFORGE_PT_export_panel()
        mock_layout = MagicMock()
        panel.layout = mock_layout

        # Mock sub-boxes/rows to return mock objects that also have label/prop/etc
        mock_box = MagicMock()
        mock_layout.box.return_value = mock_box
        mock_row = MagicMock()
        mock_layout.row.return_value = mock_row

        panel.draw(bpy.context)

        # Should show Properties and Export Configuration
        mock_box.label.assert_any_call(text="Properties", icon="ARMATURE_DATA")
        mock_box.label.assert_any_call(text="Export Configuration", icon="EXPORT")

    def test_export_panel_draw_validation_results(self, scene, blender_context) -> None:
        """Test drawing validation results in the export panel."""
        create_robot_link("base_link", scene)

        # Setup mock validation results in window manager
        wm = bpy.context.window_manager
        validation = getattr(wm, PROP_VALIDATION)
        validation.has_results = True
        validation.is_valid = False
        validation.error_count = 1
        error = validation.errors.add()
        error.title = "Test Error"
        error.message = "Something is wrong"
        validation.show_errors = True

        panel = LINKFORGE_PT_export_panel()
        mock_layout = MagicMock()
        panel.layout = mock_layout
        mock_box = MagicMock()
        mock_layout.box.return_value = mock_box

        panel.draw(bpy.context)

        # Should show error count
        mock_box.prop.assert_any_call(
            validation, "show_errors", toggle=True, text="Show 1 Error(s)", icon="TRIA_DOWN"
        )


class TestForgePanel:
    def test_forge_panel_draw(self, scene, blender_context) -> None:
        """Test drawing the forge panel."""
        panel = LINKFORGE_PT_forge()
        mock_layout = MagicMock()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        # Forge panel shows link creation tools
        mock_layout.label.assert_any_call(text="Create robot structure:", icon="TOOL_SETTINGS")


class TestLinkPanel:
    def test_link_panel_draw_no_selection(self, scene, blender_context) -> None:
        """Test drawing the link panel with no selection."""
        panel = LINKFORGE_PT_links()
        mock_layout = MagicMock()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.box.return_value.label.assert_any_call(text="Link Creation", icon="PLUS")

    def test_link_panel_draw_with_link(self, scene, blender_context) -> None:
        """Test drawing the link panel with a link selected."""
        link_obj = create_robot_link("base_link", scene)
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        panel = LINKFORGE_PT_links()
        mock_layout = MagicMock()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.box.return_value.label.assert_any_call(text="Link: base_link", icon="LINKED")


class TestJointPanel:
    def test_joint_panel_draw(self, scene, blender_context) -> None:
        """Test drawing the joint panel."""
        panel = LINKFORGE_PT_joints()
        mock_layout = MagicMock()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.box.return_value.row.return_value.operator.assert_any_call(
            "linkforge.create_joint", icon="ADD", text="Create Joint"
        )


class TestSensorPanel:
    def test_sensor_panel_draw(self, scene, blender_context) -> None:
        """Test drawing the sensor (perceive) panel."""
        panel = LINKFORGE_PT_perceive()
        mock_layout = MagicMock()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.box.return_value.row.return_value.operator.assert_any_call(
            "linkforge.create_sensor", icon="ADD", text="Create Sensor"
        )
