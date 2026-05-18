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

    def test_export_panel_draw_detailed_branches(self, scene, blender_context, mock_layout) -> None:
        """Test all missing branches inside export_panel.py."""
        # scene is None path
        panel = LINKFORGE_PT_export_panel()

        class MockContextNone:
            scene = None

        assert panel.draw(MockContextNone()) is None

        # layout is None path
        class MockContextLayoutNone:
            scene = bpy.context.scene
            window_manager = bpy.context.window_manager

        panel.layout = None
        assert panel.draw(MockContextLayoutNone()) is None

        # Restore layout
        panel.layout = mock_layout

        # Create objects to pass num_links > 0
        create_robot_link("base_link", scene)

        # Mock errors & warnings with codes, messages, suggestions, and affected objects
        wm = bpy.context.window_manager
        validation = getattr(wm, PROP_VALIDATION)
        validation.has_results = True
        validation.is_valid = False
        validation.error_count = 1
        validation.warning_count = 1
        validation.show_errors = True
        validation.show_warnings = True

        err = validation.errors.add()
        err.title = "Mock Error"
        err.error_code = "ERR_MOCK"
        err.message = (
            "This is a very long error message to test line wrapping logic in the issue panel"
        )
        err.affected_objects = "base_link"
        err.suggestion = "This is a suggestion for fixing the mock error"

        warn = validation.warnings.add()
        warn.title = "Mock Warning"
        warn.error_code = "WARN_MOCK"
        warn.message = "Warning message description goes here"
        warn.affected_objects = "base_link"
        warn.suggestion = "Fix warning suggestion"

        panel.draw(bpy.context)

        # Assert UI elements were drawn
        mock_layout.label.assert_any_call(text="  Affected: base_link", icon="OBJECT_DATA")

        # Component browser filtering empty matches path
        props = getattr(scene, PROP_ROBOT)
        props.show_kinematic_tree = True
        props.component_browser_search = "nonexistent"
        panel.draw(bpy.context)
        mock_layout.label.assert_any_call(text="No matches", icon="INFO")


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

    def test_forge_panel_draw_missing_branches(self, scene, blender_context, mock_layout) -> None:
        """Test drawing the forge panel with empty layout/row/box checks to cover False branches."""
        panel = LINKFORGE_PT_forge()

        # layout is None check
        panel.layout = None
        assert panel.draw(bpy.context) is None

        # context.scene is None check
        panel.layout = mock_layout
        from unittest.mock import MagicMock

        mock_ctx = MagicMock()
        mock_ctx.scene = None
        assert panel.draw(mock_ctx) is None

        # box is None branch check
        props = safe_get_linkforge_scene(scene)
        props.is_importing = True
        mock_layout.box.return_value = None
        panel.draw(bpy.context)

        # row is None branch check inside importing (first row is None)
        mock_layout.box.return_value = mock_layout
        mock_layout.row.side_effect = None
        mock_layout.row.return_value = None
        panel.draw(bpy.context)

        # row is None branch check inside importing (second row is None)
        mock_layout.row.side_effect = [mock_layout, None]
        panel.draw(bpy.context)

        # row is None branch check inside non-importing
        props.is_importing = False
        mock_layout.row.side_effect = None
        mock_layout.row.return_value = None
        panel.draw(bpy.context)


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

        # Test virtual link status (no child geometry)
        link_obj = create_robot_link("base_link", scene, with_visual=False, with_collision=False)
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        panel = LINKFORGE_PT_links()
        panel.layout = mock_layout

        panel.draw(bpy.context)
        mock_layout.label.assert_any_call(text="Status: Virtual Frame (No Geometry)", icon="INFO")

        # Test manual inertia input fields (use_auto_inertia = False) on non-virtual link
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

        # Test when visual/collision child of a link is active (falls back to parent properties)
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

        base = create_robot_link("base", scene)
        child = create_robot_link("child", scene)
        joint_obj = create_robot_joint("test_joint_1", base, child, scene)
        item.joint_obj = joint_obj

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

        mock_row.label.assert_any_call(text="test_joint_1", icon="EMPTY_AXIS")
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

    def test_control_panel_detailed_branches(self, scene, blender_context, mock_layout) -> None:
        """Test all missing branches inside control_panel.py."""
        panel = LINKFORGE_PT_control()

        # layout and scene None checks
        class MockContextNone:
            scene = None

        panel.layout = None
        assert panel.draw(MockContextNone()) is None

        # Restore layout
        panel.layout = mock_layout

        # get_robot_props(scene) is None check
        from unittest.mock import patch

        with patch("linkforge.blender.panels.control_panel.get_robot_props") as mock_get:
            mock_get.return_value = None
            assert panel.draw(bpy.context) is None

        # ros2_control_type == "sensor" check
        props = safe_get_linkforge_scene(scene)
        props.use_ros2_control = True
        props.ros2_control_type = "sensor"

        # Override prop_type to bypass hardcoded mock defaults
        from linkforge.blender.properties.control_props import (
            Ros2ControlJointProperty,
            Ros2ControlParameterProperty,
        )

        props.ros2_control_joints.prop_type = Ros2ControlJointProperty

        joint_item = props.ros2_control_joints.add()
        joint_item.name = "sensor_joint"
        # Add parameter with show_parameters = False
        joint_item.parameters.prop_type = Ros2ControlParameterProperty
        joint_item.parameters.add()
        joint_item.show_parameters = False

        # Add global parameter with show_ros2_control_parameters = False
        props.ros2_control_parameters.prop_type = Ros2ControlParameterProperty
        props.ros2_control_parameters.add()
        props.show_ros2_control_parameters = False

        panel.draw(bpy.context)

        # Empty ros2_control_joints draw fallback path
        props.ros2_control_joints.clear()
        panel.draw(bpy.context)

        # active_idx out of bounds draw fallback path
        props.ros2_control_joints.prop_type = Ros2ControlJointProperty
        props.ros2_control_joints.add()
        props.ros2_control_active_joint_index = 5
        panel.draw(bpy.context)

        # Reset active index and add parameters to cover 139-157
        props.ros2_control_active_joint_index = 0
        active_joint = props.ros2_control_joints[0]
        active_joint.parameters.prop_type = Ros2ControlParameterProperty
        p_item = active_joint.parameters.add()
        p_item.name = "test_param"
        p_item.value = "test_val"
        active_joint.show_parameters = True
        panel.draw(bpy.context)

        # Clear global parameters to cover 192->215 False
        props.ros2_control_parameters.clear()
        panel.draw(bpy.context)

        # UIList draw_item with GRID and cmd_effort
        ul = LINKFORGE_UL_ros2_control_joints()
        ul.layout_type = "GRID"
        # Create item with empty interfaces (cmd_position/velocity/effort = False)
        joint_item = props.ros2_control_joints[0]
        joint_item.cmd_position = False
        joint_item.cmd_velocity = False
        joint_item.cmd_effort = False
        ul.draw_item(
            bpy.context,
            mock_layout,
            props.ros2_control_joints,
            joint_item,
            None,
            props,
            "ros2_control_active_joint_index",
            0,
            0,
        )

        # UIList draw_item with DEFAULT layout and empty interfaces to cover (67->exit True)
        ul.layout_type = "DEFAULT"
        ul.draw_item(
            bpy.context,
            mock_layout,
            props.ros2_control_joints,
            joint_item,
            None,
            props,
            "ros2_control_active_joint_index",
            0,
            0,
        )

        # UIList draw_item with UNKNOWN layout type to cover (70->exit True)
        ul.layout_type = "UNKNOWN"
        ul.draw_item(
            bpy.context,
            mock_layout,
            props.ros2_control_joints,
            joint_item,
            None,
            props,
            "ros2_control_active_joint_index",
            0,
            0,
        )

        # UIList draw_item with cmd_effort = True
        ul.layout_type = "DEFAULT"
        joint_item.cmd_effort = True
        ul.draw_item(
            bpy.context,
            mock_layout,
            props.ros2_control_joints,
            joint_item,
            None,
            props,
            "ros2_control_active_joint_index",
            0,
            0,
        )

        # Menu draw edge cases and already-added branch
        menu = LINKFORGE_MT_add_control_joint()

        # Context layout and scene None checks
        menu.layout = None
        assert menu.draw(MockContextNone()) is None

        # Restore layout
        menu.layout = mock_layout

        # get_robot_props None check
        with patch("linkforge.blender.panels.control_panel.get_robot_props") as mock_get:
            mock_get.return_value = None
            assert menu.draw(bpy.context) is None

        # Create joint in scene and pre-add it to ros2_control_joints to hit (289->285)
        base = create_robot_link("base", scene)
        child = create_robot_link("child", scene)
        joint_obj = create_robot_joint("test_joint_1", base, child, scene)

        # Add it to the control joints list so it's considered "already added"
        props.ros2_control_joints.clear()
        added_item = props.ros2_control_joints.add()
        added_item.name = "test_joint_1"
        added_item.joint_obj = joint_obj

        # Now drawing the menu will skip adding it since it's already there
        menu.draw(bpy.context)


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

        # Test view_layer is None path
        mock_ctx = MagicMock()
        mock_ctx.scene = scene
        mock_ctx.view_layer = None
        assert op.execute(mock_ctx) == {"FINISHED"}

        # Test object not found fallback
        op.object_name = "nonexistent"
        res_nonexistent = op.execute(bpy.context)
        assert res_nonexistent == {"CANCELLED"}

    def test_select_root_link_operator(self, scene, blender_context) -> None:
        """Test select_root_link operator execution."""
        from unittest.mock import MagicMock, patch

        from linkforge.blender.panels.robot_panel import LINKFORGE_OT_select_root_link

        op = LINKFORGE_OT_select_root_link()
        res = op.execute(bpy.context)
        # No links created yet, should cancel
        assert res == {"CANCELLED"}

        # Create link, now should succeed
        create_robot_link("base_link", scene)
        res_success = op.execute(bpy.context)
        assert res_success == {"FINISHED"}

        # Test view_layer is None path
        mock_ctx = MagicMock()
        mock_ctx.scene = scene
        mock_ctx.view_layer = None
        assert op.execute(mock_ctx) == {"FINISHED"}

        # Test when root object does not exist in scene objects
        with patch("linkforge.blender.panels.robot_panel.build_tree_from_stats") as mock_build:
            mock_build.return_value = (None, "nonexistent_root", {}, {})
            assert op.execute(bpy.context) == {"FINISHED"}

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

    def test_panels_as_main(self) -> None:
        """Test running each panel module as __main__."""
        import importlib.util
        import sys
        from unittest.mock import patch

        from linkforge.blender import panels

        for name in [
            "control_panel",
            "export_panel",
            "forge_panel",
            "joint_panel",
            "link_panel",
            "robot_panel",
            "sensor_panel",
        ]:
            module = getattr(panels, name)
            spec = importlib.util.spec_from_file_location("__main__", module.__file__)
            if spec and spec.loader:
                main_mod = importlib.util.module_from_spec(spec)
                main_mod.__package__ = "linkforge.blender.panels"
                # Mock register to avoid actual bpy operations
                with (
                    patch("bpy.utils.register_class") as mock_reg,
                    patch("bpy.utils.unregister_class"),
                ):
                    sys.modules["__main__"] = main_mod
                    spec.loader.exec_module(main_mod)


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

        # Force a ValueError on the very first register_class call to test the fallback unregister/register path
        already_registered = False

        def mock_register(cls):
            nonlocal already_registered
            if not already_registered:
                already_registered = True
                raise ValueError("Already registered")
            return None

        with (
            patch("bpy.utils.register_class", side_effect=mock_register) as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            panels_register()
            assert mock_reg.called
            assert mock_unreg.called

            panels_unregister()

    def test_module_registrations_and_contexts(self) -> None:
        """Test registration and error contexts for all panel modules."""
        from unittest.mock import patch

        from linkforge.blender import panels
        from linkforge.blender.panels.robot_panel import (
            LINKFORGE_OT_clear_component_search,
            LINKFORGE_OT_select_root_link,
            LINKFORGE_OT_select_tree_object,
        )

        for name in [
            "control_panel",
            "export_panel",
            "forge_panel",
            "joint_panel",
            "link_panel",
            "robot_panel",
            "sensor_panel",
        ]:
            module = getattr(panels, name)
            already_registered = False

            def mock_register(cls):
                nonlocal already_registered
                if not already_registered:
                    already_registered = True
                    raise ValueError("Already registered")
                return None

            with (
                patch("bpy.utils.register_class", side_effect=mock_register) as mock_reg,
                patch("bpy.utils.unregister_class") as mock_unreg,
            ):
                module.register()
                assert mock_reg.called
                module.unregister()

        # Test operator execute with context.scene = None
        class MockContextSceneNone:
            scene = None

        op_select = LINKFORGE_OT_select_tree_object()
        assert op_select.execute(MockContextSceneNone()) == {"CANCELLED"}

        op_root = LINKFORGE_OT_select_root_link()
        assert op_root.execute(MockContextSceneNone()) == {"CANCELLED"}

        op_clear = LINKFORGE_OT_clear_component_search()
        assert op_clear.execute(MockContextSceneNone()) == {"CANCELLED"}


class TestPanelsExtra:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        from tests.blender_test_utils import cleanup_blender_scene

        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_export_panel_browser_no_select_box(self, scene, mock_layout) -> None:
        """Test component browser exits early if box creation fails."""
        from linkforge.blender.panels.export_panel import LINKFORGE_PT_export_panel

        create_robot_link("base_link", scene)
        panel = LINKFORGE_PT_export_panel()
        mock_layout.box.return_value = None
        panel.layout = mock_layout
        assert panel.draw(bpy.context) is None

    def test_export_panel_browser_falsy_search(self, scene, mock_layout) -> None:
        """Test component browser with blank/falsy search term."""
        from linkforge.blender.panels.export_panel import LINKFORGE_PT_export_panel

        base = create_robot_link("base_link", scene)
        child = create_robot_link("child_link", scene)
        create_robot_joint("test_joint", base, child, scene)

        sensor = create_test_object("test_sensor", None, scene)
        sp = safe_get_sensor(sensor)
        sp.is_robot_sensor = True
        sensor.parent = base

        props = getattr(scene, PROP_ROBOT)
        props.show_kinematic_tree = True
        props.component_browser_search = ""

        panel = LINKFORGE_PT_export_panel()
        panel.layout = mock_layout
        panel.draw(bpy.context)

        mock_layout.label.assert_any_call(text="Links (2):", icon="MESH_CUBE")
        mock_layout.label.assert_any_call(text="Joints (1):", icon="EMPTY_AXIS")
        mock_layout.label.assert_any_call(text="Sensors (1):", icon="TRACKER")

    def test_joint_panel_missing_branches(self, scene, mock_layout) -> None:
        """Test joint panel layout/scene is None and draw callbacks."""
        from linkforge.blender.panels.joint_panel import LINKFORGE_PT_joints

        panel = LINKFORGE_PT_joints()
        panel.layout = None
        assert panel.draw(bpy.context) is None

        class MockContextSceneNone:
            scene = None
            active_object = None

        panel.layout = mock_layout
        assert panel.draw(MockContextSceneNone()) is None

        base = create_robot_link("base", scene)
        child = create_robot_link("child", scene)
        joint_obj = create_robot_joint("j1", base, child, scene)

        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = joint_obj
        joint_obj.select_set(True)

        jp = safe_get_joint(joint_obj)
        jp.joint_type = "PRISMATIC"
        jp.axis = "X"
        jp.use_dynamics = False
        jp.use_mimic = False
        jp.use_safety_controller = False
        jp.use_calibration = False

        panel.draw(bpy.context)
        mock_layout.prop.assert_any_call(jp, "joint_type")

    def test_sensor_panel_missing_branches(self, scene, mock_layout) -> None:
        """Test sensor panel layout/scene is None and sensor configurations."""
        from linkforge.blender.panels.sensor_panel import LINKFORGE_PT_perceive

        panel = LINKFORGE_PT_perceive()
        panel.layout = None
        assert panel.draw(bpy.context) is None

        class MockContextSceneNone:
            scene = None
            active_object = None

        panel.layout = mock_layout
        assert panel.draw(MockContextSceneNone()) is None

        base = create_robot_link("base", scene)
        sensor_obj = create_test_object("sensor1", None, scene)
        sp = safe_get_sensor(sensor_obj)
        sp.is_robot_sensor = True
        sp.sensor_type = "LIDAR"
        sp.use_noise = False
        sp.use_gazebo_plugin = False
        sensor_obj.parent = base

        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = sensor_obj
        sensor_obj.select_set(True)

        panel.draw(bpy.context)
        mock_layout.prop.assert_any_call(sp, "sensor_type")

    def test_link_panel_layout_none(self) -> None:
        """Verify link panel exits early if layout is missing."""
        from linkforge.blender.panels.link_panel import LINKFORGE_PT_links

        panel = LINKFORGE_PT_links()
        panel.layout = None
        assert panel.draw(bpy.context) is None

    def test_link_panel_mesh_simplification_slider(self, scene, mock_layout) -> None:
        """Verify link panel collision quality simplification slider checks."""
        from unittest.mock import patch

        from linkforge.blender.panels.link_panel import LINKFORGE_PT_links

        link_obj = create_robot_link("base_link", scene, with_visual=True, with_collision=True)
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        from tests.blender_test_utils import safe_get_linkforge

        props = safe_get_linkforge(link_obj)
        props.collision_type = "auto"

        stats = MagicMock()
        collision_mesh_obj = MagicMock()
        collision_mesh_obj.get.return_value = False
        stats.geometry_stats = {"base_link": (collision_mesh_obj, "mesh", False)}

        panel = LINKFORGE_PT_links()
        panel.layout = mock_layout

        with patch("linkforge.blender.panels.link_panel.get_robot_statistics", return_value=stats):
            panel.draw(bpy.context)
            mock_layout.prop.assert_any_call(
                props, "collision_quality", text="Collision Quality", slider=True
            )

    def test_link_panel_material_node_tree(self, scene, mock_layout) -> None:
        """Verify link panel material slot template and BSDF node checks."""
        from linkforge.blender.panels.link_panel import LINKFORGE_PT_links

        link_obj = create_robot_link("base_link", scene, with_cube=True)
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        from tests.blender_test_utils import safe_get_linkforge

        props = safe_get_linkforge(link_obj)
        props.use_material = True

        panel = LINKFORGE_PT_links()
        panel.layout = mock_layout

        visual_obj = next(c for c in link_obj.children if "visual" in c.name)
        mat = bpy.data.materials.new("TestMaterial")
        mat.use_nodes = True
        visual_obj.data.materials.append(mat)

        node = MagicMock()
        node.type = "BSDF_PRINCIPLED"
        node.inputs = {"Base Color": MagicMock()}
        mat.node_tree.nodes = [node]

        panel.draw(bpy.context)
        mock_layout.template_ID.assert_called()
        mock_layout.prop.assert_any_call(node.inputs["Base Color"], "default_value", text="")

    def test_link_panel_simulation_advanced(self, scene, mock_layout) -> None:
        """Verify link panel simulation properties inputs."""
        from linkforge.blender.panels.link_panel import LINKFORGE_PT_links

        link_obj = create_robot_link("base_link", scene)
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        from tests.blender_test_utils import safe_get_linkforge

        props = safe_get_linkforge(link_obj)
        props.use_simulation_props = True

        panel = LINKFORGE_PT_links()
        panel.layout = mock_layout
        panel.draw(bpy.context)

        mock_layout.prop.assert_any_call(props, "self_collide")
        mock_layout.prop.assert_any_call(props, "kp_ui", text="kp (Stiffness)")
