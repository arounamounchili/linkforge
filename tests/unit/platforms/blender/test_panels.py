"""Unit tests for LinkForge Blender UI panels."""

from __future__ import annotations

from unittest.mock import MagicMock

import bpy
import pytest
from linkforge.blender.constants import PROP_ROBOT, PROP_VALIDATION
from linkforge.blender.panels.control_panel import (
    LINKFORGE_MT_add_control_joint,
    LINKFORGE_PT_control,
    LINKFORGE_UL_ros2_control_joints,
)
from linkforge.blender.panels.export_panel import LINKFORGE_PT_export_panel
from linkforge.blender.panels.forge_panel import LINKFORGE_PT_forge
from linkforge.blender.panels.joint_panel import LINKFORGE_PT_joints
from linkforge.blender.panels.link_panel import LINKFORGE_PT_links
from linkforge.blender.panels.sensor_panel import LINKFORGE_PT_perceive

from tests.blender_test_utils import (
    create_robot_joint,
    create_robot_link,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge_scene,
    safe_get_sensor,
)


@pytest.fixture
def mock_layout() -> MagicMock:
    """A unified layout mock that returns itself for nested UI builder calls."""
    layout = MagicMock()
    layout.box.return_value = layout
    layout.row.return_value = layout
    layout.column.return_value = layout
    layout.grid_flow.return_value = layout
    return layout


class TestExportPanel:
    def test_export_panel_draw_no_links(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the export panel when no links are present."""
        panel = LINKFORGE_PT_export_panel()
        panel.layout = mock_layout

        panel.draw(bpy.context)

        # Should show "No robot in scene" when empty
        mock_layout.label.assert_any_call(text="No robot in scene", icon="INFO")

    def test_export_panel_draw_with_links(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the export panel when links are present."""
        create_robot_link("base_link", scene)

        panel = LINKFORGE_PT_export_panel()
        panel.layout = mock_layout

        panel.draw(bpy.context)

        # Should show Properties and Export Configuration
        mock_layout.label.assert_any_call(text="Properties", icon="ARMATURE_DATA")
        mock_layout.label.assert_any_call(text="Export Configuration", icon="EXPORT")

    def test_export_panel_draw_validation_results(
        self, scene, blender_context, mock_layout
    ) -> None:
        """Test drawing validation results in the export panel."""
        create_robot_link("base_link", scene)

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
        panel.layout = mock_layout

        panel.draw(bpy.context)

        mock_layout.prop.assert_any_call(
            validation, "show_errors", toggle=True, text="Show 1 Error(s)", icon="TRIA_DOWN"
        )

    def test_export_panel_draw_warning_results(self, scene, blender_context, mock_layout) -> None:
        """Test drawing warnings in validation results in the export panel."""
        create_robot_link("base_link", scene)

        wm = bpy.context.window_manager
        validation = getattr(wm, PROP_VALIDATION)
        validation.has_results = True
        validation.is_valid = True
        validation.warning_count = 1
        warning = validation.warnings.add()
        warning.title = "Test Warning"
        warning.message = "Something is slightly wrong"
        validation.show_warnings = True

        panel = LINKFORGE_PT_export_panel()
        panel.layout = mock_layout

        panel.draw(bpy.context)

        mock_layout.prop.assert_any_call(
            validation, "show_warnings", toggle=True, text="Show 1 Warning(s)", icon="TRIA_DOWN"
        )

    def test_export_panel_draw_advanced_xacro(self, scene, blender_context, mock_layout) -> None:
        """Test export panel drawing combinations for advanced XACRO settings."""
        create_robot_link("base_link", scene)
        props = getattr(scene, PROP_ROBOT)
        props.export_format = "XACRO"
        props.xacro_advanced_mode = True

        panel = LINKFORGE_PT_export_panel()
        panel.layout = mock_layout

        panel.draw(bpy.context)

        mock_layout.prop.assert_any_call(props, "xacro_extract_materials")

    def test_export_panel_draw_component_browser_with_search(
        self, scene, blender_context, mock_layout
    ) -> None:
        """Test export panel component browser with dynamic search filtering."""
        base = create_robot_link("base_link", scene)
        child = create_robot_link("child_link", scene)
        joint = create_robot_joint("test_joint", base, child, scene)

        # Add a sensor
        sensor = create_test_object("test_sensor", None, scene)
        sp = safe_get_sensor(sensor)
        sp.is_robot_sensor = True
        sensor.parent = base

        props = getattr(scene, PROP_ROBOT)
        props.show_kinematic_tree = True
        props.component_browser_search = "base"

        panel = LINKFORGE_PT_export_panel()
        panel.layout = mock_layout

        panel.draw(bpy.context)

        mock_layout.prop.assert_any_call(
            props, "component_browser_search", text="", icon="VIEWZOOM"
        )


class TestForgePanel:
    def test_forge_panel_draw(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the forge panel."""
        panel = LINKFORGE_PT_forge()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.label.assert_any_call(text="Create robot structure:", icon="TOOL_SETTINGS")

    def test_forge_panel_draw_importing(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the forge panel when active importing is occurring."""
        props = safe_get_linkforge_scene(scene)
        props.is_importing = True
        props.import_status = "Importing model..."

        panel = LINKFORGE_PT_forge()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.label.assert_any_call(text="Importing model...", icon="URL")
        mock_layout.prop.assert_any_call(
            props,
            "abort_import",
            text="Stop Import",
            toggle=True,
            icon="CANCEL",
        )


class TestLinkPanel:
    def test_link_panel_draw_no_selection(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the link panel with no selection."""
        panel = LINKFORGE_PT_links()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.label.assert_any_call(text="Link Creation", icon="PLUS")

    def test_link_panel_draw_with_link(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the link panel with a link selected."""
        link_obj = create_robot_link("base_link", scene)
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        panel = LINKFORGE_PT_links()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.label.assert_any_call(text="Link: base_link", icon="LINKED")

    def test_link_panel_draw_detailed_branches(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the link panel with virtual link and manual inertia branches."""
        from linkforge.blender.utils.property_helpers import get_link_props

        # 1. Test virtual link status (no child geometry)
        link_obj = create_robot_link("base_link", scene, with_visual=False, with_collision=False)
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        panel = LINKFORGE_PT_links()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.label.assert_any_call(text="Status: Virtual Frame (No Geometry)", icon="INFO")

        # 2. Test manual inertia input fields (use_auto_inertia = False) on non-virtual link
        non_virtual_link = create_robot_link(
            "non_virtual_link", scene, with_visual=True, with_collision=False
        )
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = non_virtual_link
        non_virtual_link.select_set(True)
        link_obj.select_set(False)

        props = get_link_props(non_virtual_link)
        assert props is not None
        props.use_auto_inertia = False

        panel.draw(bpy.context)
        mock_layout.prop.assert_any_call(props, "inertia_ixx", text="Ixx")
        mock_layout.prop.assert_any_call(props, "inertia_origin_xyz", text="")

        # 3. Test when visual/collision child of a link is active (falls back to parent properties)
        child_visual = create_test_object("non_virtual_link_visual", None, scene)
        child_visual.parent = non_virtual_link

        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = child_visual
        child_visual.select_set(True)
        link_obj.select_set(False)

        panel.draw(bpy.context)
        # Should fall back to parent and render its title
        mock_layout.label.assert_any_call(text="Link: non_virtual_link", icon="LINKED")


class TestJointPanel:
    def test_joint_panel_draw(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the joint panel."""
        panel = LINKFORGE_PT_joints()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.operator.assert_any_call(
            "linkforge.create_joint", icon="ADD", text="Create Joint"
        )

    def test_joint_panel_draw_editing_joint(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the joint panel when editing an active joint."""
        base = create_robot_link("base_link", scene)
        child = create_robot_link("child_link", scene)
        joint_obj = create_robot_joint("test_joint", base, child, scene)

        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = joint_obj
        joint_obj.select_set(True)

        panel = LINKFORGE_PT_joints()
        panel.layout = mock_layout

        panel.draw(bpy.context)

        mock_layout.label.assert_any_call(text="Joint: test_joint", icon="EMPTY_ARROWS")
        mock_layout.prop.assert_any_call(safe_get_joint(joint_obj), "joint_name")

    def test_joint_panel_draw_joint_types_and_toggles(
        self, scene, blender_context, mock_layout
    ) -> None:
        """Test drawing different joint configurations and toggles."""
        base = create_robot_link("base_link", scene)
        child = create_robot_link("child_link", scene)
        joint_obj = create_robot_joint("test_joint", base, child, scene)

        jp = safe_get_joint(joint_obj)
        jp.joint_type = "REVOLUTE"
        jp.axis = "CUSTOM"
        jp.use_dynamics = True
        jp.use_mimic = True
        jp.use_safety_controller = True
        jp.use_calibration = True

        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = joint_obj
        joint_obj.select_set(True)

        panel = LINKFORGE_PT_joints()
        panel.layout = mock_layout

        panel.draw(bpy.context)

        mock_layout.prop.assert_any_call(jp, "limit_lower")
        mock_layout.prop.assert_any_call(jp, "dynamics_damping")
        mock_layout.prop.assert_any_call(jp, "mimic_joint")
        mock_layout.prop.assert_any_call(jp, "safety_soft_lower_limit")


class TestSensorPanel:
    def test_sensor_panel_draw(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the sensor (perceive) panel in create mode."""
        panel = LINKFORGE_PT_perceive()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.operator.assert_any_call(
            "linkforge.create_sensor", icon="ADD", text="Create Sensor"
        )

    def test_sensor_panel_draw_editing_sensor(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the sensor panel when editing a sensor."""
        base = create_robot_link("base_link", scene)
        sensor_obj = create_test_object("test_sensor", None, scene)
        sp = safe_get_sensor(sensor_obj)
        sp.is_robot_sensor = True
        sp.sensor_name = "test_sensor"
        sp.sensor_type = "CAMERA"
        sp.use_noise = True
        sp.use_gazebo_plugin = True
        sensor_obj.parent = base

        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = sensor_obj
        sensor_obj.select_set(True)

        panel = LINKFORGE_PT_perceive()
        panel.layout = mock_layout

        panel.draw(bpy.context)

        mock_layout.label.assert_any_call(text="Sensor: test_sensor", icon="OUTLINER_OB_CAMERA")
        mock_layout.prop.assert_any_call(sp, "sensor_name")
        mock_layout.prop.assert_any_call(sp, "noise_mean")
        mock_layout.prop.assert_any_call(sp, "plugin_filename")

    def test_sensor_panel_draw_sensor_types(self, scene, blender_context, mock_layout) -> None:
        """Test sensor panel specific settings for lidar and contact sensor types."""
        base = create_robot_link("base_link", scene)
        sensor_obj = create_test_object("test_sensor", None, scene)
        sp = safe_get_sensor(sensor_obj)
        sp.is_robot_sensor = True
        sp.sensor_type = "LIDAR"
        sensor_obj.parent = base

        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = sensor_obj
        sensor_obj.select_set(True)

        panel = LINKFORGE_PT_perceive()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.prop.assert_any_call(sp, "lidar_horizontal_samples")

        sp.sensor_type = "CONTACT"
        panel.draw(bpy.context)
        mock_layout.prop.assert_any_call(sp, "contact_collision")


class TestControlPanel:
    def test_control_panel_draw_disabled(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the control panel when disabled."""
        props = safe_get_linkforge_scene(scene)
        props.use_ros2_control = False

        panel = LINKFORGE_PT_control()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.label.assert_any_call(
            text="Enable ROS 2 Control to configure settings.", icon="INFO"
        )

    def test_control_panel_draw_enabled(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the control panel when enabled with parameters and lists."""
        props = safe_get_linkforge_scene(scene)
        props.use_ros2_control = True

        p = props.ros2_control_parameters.add()
        p.name = "test_param"
        p.value = "test_val"
        props.show_ros2_control_parameters = True

        base = create_robot_link("base", scene)
        child = create_robot_link("child", scene)
        joint_obj = create_robot_joint("j1", base, child, scene)

        joint_item = props.ros2_control_joints.add()
        joint_item.name = "j1"
        joint_item.joint_obj = joint_obj
        joint_item.cmd_position = True

        j_param = joint_item.parameters.add()
        j_param.name = "joint_p"
        j_param.value = "joint_v"
        joint_item.show_parameters = True

        props.ros2_control_active_joint_index = 0

        panel = LINKFORGE_PT_control()
        panel.layout = mock_layout

        panel.draw(bpy.context)

        mock_layout.prop.assert_any_call(
            props, "use_ros2_control", text="Use ROS2 Control", icon="CHECKMARK"
        )
        mock_layout.prop.assert_any_call(p, "value", text="")

    def test_ui_list_draw_item(self, scene, blender_context) -> None:
        """Test UI list drawing for control joints."""
        props = safe_get_linkforge_scene(scene)
        item = props.ros2_control_joints.add()
        item.name = "test_joint"
        item.cmd_position = True
        item.cmd_velocity = True

        ul = LINKFORGE_UL_ros2_control_joints()
        mock_layout = MagicMock()
        mock_row = MagicMock()
        mock_layout.row.return_value = mock_row

        ul.layout_type = "DEFAULT"
        ul.draw_item(
            bpy.context,
            mock_layout,
            props.ros2_control_joints,
            item,
            None,
            props,
            "ros2_control_active_joint_index",
            0,
            0,
        )

        mock_row.label.assert_any_call(text="test_joint", icon="EMPTY_AXIS")
        mock_row.label.assert_any_call(text="[P/V]", icon="NONE")

    def test_add_control_joint_menu(self, scene, blender_context, mock_layout) -> None:
        """Test add control joint dropdown menu populating from kinematic tree."""
        base = create_robot_link("base", scene)
        child = create_robot_link("child", scene)
        joint_obj = create_robot_joint("test_joint_1", base, child, scene)

        menu = LINKFORGE_MT_add_control_joint()
        menu.layout = mock_layout

        menu.draw(bpy.context)

        mock_layout.operator.assert_any_call(
            "linkforge.add_ros2_control_joint", text="test_joint_1"
        )


class TestRobotOperators:
    def test_select_tree_object_operator(self, scene, blender_context) -> None:
        """Test select_tree_object operator execution and target lookup."""
        from linkforge.blender.panels.robot_panel import LINKFORGE_OT_select_tree_object

        link_obj = create_robot_link("test_link", scene)

        op = LINKFORGE_OT_select_tree_object()
        op.object_name = "test_link"
        op.object_type = "link"

        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        # Test object not found fallback
        op.object_name = "nonexistent"
        res_nonexistent = op.execute(bpy.context)
        assert res_nonexistent == {"CANCELLED"}

    def test_select_root_link_operator(self, scene, blender_context) -> None:
        """Test select_root_link operator execution."""
        from linkforge.blender.panels.robot_panel import LINKFORGE_OT_select_root_link

        op = LINKFORGE_OT_select_root_link()
        res = op.execute(bpy.context)
        # No links created yet, should cancel
        assert res == {"CANCELLED"}

        # Create link, now should succeed
        create_robot_link("base_link", scene)
        res_success = op.execute(bpy.context)
        assert res_success == {"FINISHED"}

    def test_clear_component_search_operator(self, scene, blender_context) -> None:
        """Test clear_component_search operator poll and execute."""
        from linkforge.blender.panels.robot_panel import LINKFORGE_OT_clear_component_search

        op = LINKFORGE_OT_clear_component_search()
        props = safe_get_linkforge_scene(scene)

        props.component_browser_search = ""
        assert op.poll(bpy.context) is False

        props.component_browser_search = "search_val"
        assert op.poll(bpy.context) is True

        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        assert props.component_browser_search == ""


class TestGlobalPanels:
    def test_panels_global_registration(self) -> None:
        """Test global register and unregister functions for panels package."""
        from unittest.mock import patch

        from linkforge.blender.panels import (
            register as panels_register,
        )
        from linkforge.blender.panels import (
            unregister as panels_unregister,
        )

        with (
            patch("bpy.utils.register_class") as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            panels_register()
            assert mock_reg.called

            panels_unregister()
            assert mock_unreg.called
