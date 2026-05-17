"""Unit tests for Blender Export and Validation operators."""

from __future__ import annotations

import os
from unittest.mock import MagicMock, patch

import bpy
from linkforge.blender.constants import PROP_ROBOT, PROP_VALIDATION
from linkforge.blender.operators.export_ops import (
    LINKFORGE_OT_export_robot_model,
    LINKFORGE_OT_validate_robot,
    working_directory,
)
from linkforge.blender.operators.export_ops import (
    register as export_register,
)
from linkforge.blender.operators.export_ops import (
    unregister as export_unregister,
)


class TestExportOperators:
    def test_working_directory_context_manager(self, tmp_path) -> None:
        """Test the working_directory context manager changes cwd temporarily."""
        original_cwd = os.getcwd()
        with working_directory(tmp_path):
            assert os.getcwd() == str(tmp_path.resolve())
        assert os.getcwd() == original_cwd

    def test_export_robot_model_check(self) -> None:
        """Test check method returns True only if scene has PROP_ROBOT."""
        op = LINKFORGE_OT_export_robot_model()

        # Scenario 1: scene is None
        mock_context = MagicMock()
        mock_context.scene = None
        assert op.check(mock_context) is False

        # Scenario 2: scene is not None, but does not have PROP_ROBOT
        mock_scene = MagicMock(spec=[])
        mock_context.scene = mock_scene
        assert op.check(mock_context) is False

        # Scenario 3: scene is not None, has PROP_ROBOT
        mock_scene = MagicMock()
        setattr(mock_scene, PROP_ROBOT, MagicMock())
        mock_context.scene = mock_scene
        assert op.check(mock_context) is True

    def test_export_robot_model_invoke(self) -> None:
        """Test invoke sets the correct suffix based on export format."""
        op = LINKFORGE_OT_export_robot_model()
        event = MagicMock()
        mock_context = MagicMock()

        # Missing PROP_ROBOT
        mock_scene = MagicMock(spec=[])
        mock_context.scene = mock_scene
        assert op.invoke(mock_context, event) == {"CANCELLED"}

        # XACRO format
        mock_props = MagicMock()
        mock_props.export_format = "XACRO"
        mock_scene = MagicMock()
        setattr(mock_scene, PROP_ROBOT, mock_props)
        mock_context.scene = mock_scene
        with patch("bpy_extras.io_utils.ExportHelper.invoke", return_value={"RUNNING_MODAL"}):
            res = op.invoke(mock_context, event)
            assert res == {"RUNNING_MODAL"}
            assert op.filename_ext == ".xacro"

        # URDF format
        mock_props.export_format = "URDF"
        with patch("bpy_extras.io_utils.ExportHelper.invoke", return_value={"RUNNING_MODAL"}):
            res = op.invoke(mock_context, event)
            assert res == {"RUNNING_MODAL"}
            assert op.filename_ext == ".urdf"

    def test_export_robot_model_execute_missing_props(self) -> None:
        """Test execute cancels if PROP_ROBOT is missing."""
        op = LINKFORGE_OT_export_robot_model()
        mock_context = MagicMock()
        mock_scene = MagicMock(spec=[])
        mock_context.scene = mock_scene

        res = op.execute(mock_context)
        assert res == {"CANCELLED"}

    @patch("linkforge.blender.adapters.blender_to_core.scene_to_robot")
    @patch("linkforge.core.URDFGenerator")
    def test_export_robot_model_execute_urdf_success(
        self, mock_generator_cls, mock_scene_to_robot, tmp_path, scene, blender_context
    ) -> None:
        """Test successful export to URDF format."""
        filepath = tmp_path / "robot.urdf"
        op = LINKFORGE_OT_export_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        # Mock robot properties
        mock_props = MagicMock()
        mock_props.export_format = "URDF"
        mock_props.mesh_directory_name = "meshes"
        mock_props.validate_before_export = True
        mock_props.export_meshes = True
        mock_props.use_ros2_control = True
        setattr(scene, PROP_ROBOT, mock_props)

        # Mock robot parsing and validation
        mock_robot = MagicMock()
        mock_robot.name = "test_robot"
        mock_scene_to_robot.return_value = (mock_robot, MagicMock(issues=[]))

        mock_val_res = MagicMock()
        mock_val_res.is_valid = True

        with patch("linkforge.core.RobotValidator.validate", return_value=mock_val_res):
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}
            op.report.assert_any_call(
                {"INFO"}, f"Exported URDF to {filepath} (meshes in {filepath.parent / 'meshes'})"
            )

    @patch("linkforge.blender.adapters.blender_to_core.scene_to_robot")
    @patch("linkforge.core.XACROGenerator")
    def test_export_robot_model_execute_xacro_success(
        self, mock_generator_cls, mock_scene_to_robot, tmp_path, scene, blender_context
    ) -> None:
        """Test successful export to XACRO format."""
        filepath = tmp_path / "robot.xacro"
        op = LINKFORGE_OT_export_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        # Mock robot properties
        mock_props = MagicMock()
        mock_props.export_format = "XACRO"
        mock_props.mesh_directory_name = "meshes"
        mock_props.validate_before_export = False
        mock_props.export_meshes = False
        mock_props.xacro_extract_materials = True
        mock_props.xacro_extract_dimensions = True
        mock_props.xacro_generate_macros = True
        mock_props.xacro_split_files = True
        mock_props.use_ros2_control = False
        setattr(scene, PROP_ROBOT, mock_props)

        # Mock robot parsing
        mock_robot = MagicMock()
        mock_robot.name = "xacro_robot"
        mock_scene_to_robot.return_value = (mock_robot, MagicMock(issues=[]))

        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        op.report.assert_any_call(
            {"INFO"}, f"Exported XACRO to {filepath} (meshes in {filepath.parent / 'meshes'})"
        )

    @patch("linkforge.blender.adapters.blender_to_core.scene_to_robot")
    def test_export_robot_model_execute_validation_failure(
        self, mock_scene_to_robot, tmp_path, scene, blender_context
    ) -> None:
        """Test export cancels if validation fails when validate_before_export is active."""
        filepath = tmp_path / "robot.urdf"
        op = LINKFORGE_OT_export_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        # Mock robot properties
        mock_props = MagicMock()
        mock_props.export_format = "URDF"
        mock_props.mesh_directory_name = "meshes"
        mock_props.validate_before_export = True
        setattr(scene, PROP_ROBOT, mock_props)

        # Mock validation result with failures
        mock_scene_to_robot.return_value = (MagicMock(), MagicMock(issues=[]))
        mock_val_res = MagicMock()
        mock_val_res.is_valid = False
        mock_val_res.error_count = 2

        with patch("linkforge.core.RobotValidator.validate", return_value=mock_val_res):
            res = op.execute(bpy.context)
            assert res == {"CANCELLED"}
            op.report.assert_any_call(
                {"ERROR"}, "Cannot export: 2 validation error(s). Run validation to see details."
            )

    @patch("linkforge.blender.adapters.blender_to_core.scene_to_robot")
    def test_export_robot_model_execute_build_error(
        self, mock_scene_to_robot, tmp_path, scene, blender_context
    ) -> None:
        """Test export handles robot build errors gracefully."""
        filepath = tmp_path / "robot.urdf"
        op = LINKFORGE_OT_export_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        # Mock robot properties
        mock_props = MagicMock()
        mock_props.export_format = "URDF"
        mock_props.validate_before_export = True
        setattr(scene, PROP_ROBOT, mock_props)

        mock_scene_to_robot.side_effect = RuntimeError("Invalid link connections")

        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}
        op.report.assert_any_call(
            {"ERROR"}, "Failed to build robot model: Invalid link connections"
        )


class TestValidateRobotOperator:
    def test_validate_robot_missing_validation_prop(self) -> None:
        """Test validate operator cancels if validation properties are missing from wm."""
        op = LINKFORGE_OT_validate_robot()
        op.report = MagicMock()

        mock_context = MagicMock()
        mock_context.window_manager = MagicMock(spec=[])

        res = op.execute(mock_context)
        assert res == {"CANCELLED"}
        op.report.assert_any_call({"ERROR"}, "Validation system not initialized")

    @patch("linkforge.blender.adapters.blender_to_core.scene_to_robot")
    def test_validate_robot_build_crash(self, mock_scene_to_robot, scene, blender_context) -> None:
        """Test validate operator handles build crashes gracefully."""
        op = LINKFORGE_OT_validate_robot()
        op.report = MagicMock()

        wm = bpy.context.window_manager
        assert wm is not None
        mock_val_prop = MagicMock()
        setattr(wm, PROP_VALIDATION, mock_val_prop)

        mock_scene_to_robot.side_effect = RuntimeError("Fatal parsing error")

        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}
        op.report.assert_any_call({"ERROR"}, "Model build crashed: Fatal parsing error")
        assert mock_val_prop.is_valid is False
        assert mock_val_prop.error_count == 1

    @patch("linkforge.blender.adapters.blender_to_core.scene_to_robot")
    def test_validate_robot_success_clean(
        self, mock_scene_to_robot, scene, blender_context
    ) -> None:
        """Test validate operator reports success when robot is fully valid."""
        op = LINKFORGE_OT_validate_robot()
        op.report = MagicMock()

        wm = bpy.context.window_manager
        assert wm is not None
        mock_val_prop = MagicMock()
        mock_val_prop.errors = MagicMock()
        mock_val_prop.warnings = MagicMock()
        setattr(wm, PROP_VALIDATION, mock_val_prop)

        # Mock robot and validation result
        mock_robot = MagicMock()
        mock_robot.name = "clean_robot"
        mock_robot.links = ["link1", "link2"]
        mock_robot.joints = ["joint1"]
        mock_robot.degrees_of_freedom = 1
        mock_scene_to_robot.return_value = (mock_robot, MagicMock(issues=[]))

        mock_val_res = MagicMock()
        mock_val_res.is_valid = True
        mock_val_res.has_warnings = False
        mock_val_res.errors = []
        mock_val_res.warnings = []

        with patch("linkforge.core.RobotValidator.validate", return_value=mock_val_res):
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}
            op.report.assert_any_call(
                {"INFO"}, "Robot 'clean_robot' is valid! (2 links, 1 joints, 1 DOF)"
            )

    @patch("linkforge.blender.adapters.blender_to_core.scene_to_robot")
    def test_validate_robot_with_warnings_and_errors(
        self, mock_scene_to_robot, scene, blender_context
    ) -> None:
        """Test validate operator handles validation warnings and errors correctly."""
        op = LINKFORGE_OT_validate_robot()
        op.report = MagicMock()

        wm = bpy.context.window_manager
        assert wm is not None
        mock_val_prop = MagicMock()
        setattr(wm, PROP_VALIDATION, mock_val_prop)

        # Mock robot and validation result with errors/warnings
        mock_robot = MagicMock()
        mock_robot.name = "warn_err_robot"
        mock_scene_to_robot.return_value = (mock_robot, MagicMock(issues=[]))

        # Mock error
        mock_error = MagicMock()
        mock_error.title = "Circular mimic"
        mock_error.message = "Circular chain detected"
        mock_error.suggestion = "Remove mimic cycle"
        mock_error.affected_objects = ["joint1"]
        mock_error.code = MagicMock(name="CIRCULAR_MIMIC_CHAIN")

        # Mock warning
        mock_warning = MagicMock()
        mock_warning.title = "No limits"
        mock_warning.message = "Continuous joint limit warning"
        mock_warning.suggestion = "Add limits"
        mock_warning.affected_objects = ["joint2"]
        mock_warning.code = MagicMock(name="LIMIT_MISSING")

        mock_val_res = MagicMock()
        mock_val_res.is_valid = False
        mock_val_res.has_warnings = True
        mock_val_res.error_count = 1
        mock_val_res.warning_count = 1
        mock_val_res.errors = [mock_error]
        mock_val_res.warnings = [mock_warning]

        with patch("linkforge.core.RobotValidator.validate", return_value=mock_val_res):
            res = op.execute(bpy.context)
            assert res == {"CANCELLED"}
            op.report.assert_any_call(
                {"ERROR"}, "Validation failed. Found 1 error(s). Please check the Validation Panel."
            )

    def test_registration(self) -> None:
        """Test register and unregister functions for export operator and package-level operators."""
        from linkforge.blender import operators as operators_pkg

        with (
            patch("bpy.utils.register_class") as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            export_register()
            assert mock_reg.called

            export_unregister()
            assert mock_unreg.called

            # Cover package-level
            operators_pkg.register()
            operators_pkg.unregister()
