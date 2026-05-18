"""Unit tests for Blender Import operators."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import bpy
from linkforge.blender.operators.import_ops import (
    LINKFORGE_OT_import_robot_model,
)
from linkforge.blender.operators.import_ops import (
    register as import_register,
)
from linkforge.blender.operators.import_ops import (
    unregister as import_unregister,
)
from linkforge.core.exceptions import RobotParserError, XacroDetectedError


class TestImportRobotModelOperator:
    def test_check_method(self, blender_context) -> None:
        """Test check method returns True."""
        op = LINKFORGE_OT_import_robot_model()
        assert op.check(bpy.context) is True

    @patch("linkforge.core.URDFParser")
    @patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder")
    def test_execute_urdf_success(
        self, mock_builder, mock_parser, tmp_path, scene, blender_context
    ) -> None:
        """Test execute operator with a valid URDF file."""
        filepath = tmp_path / "robot.urdf"
        filepath.write_text("<robot name='test_robot'/>")

        # Mock parsed robot model
        mock_robot = MagicMock()
        mock_robot.name = "test_robot"
        mock_robot.links = ["base_link"]
        mock_robot.joints = []
        mock_parser.return_value.parse.return_value = mock_robot

        # Mock validation result
        mock_val_res = MagicMock()
        mock_val_res.is_valid = True
        mock_val_res.has_warnings = False

        op = LINKFORGE_OT_import_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        with (
            patch("linkforge.core.RobotValidator.validate", return_value=mock_val_res),
            patch("linkforge.blender.preferences.get_addon_prefs", return_value=None),
        ):
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}
            op.report.assert_any_call(
                {"INFO"}, "Started background import of URDF: 'test_robot'..."
            )

    @patch("linkforge.core.XacroResolver")
    @patch("linkforge.core.URDFParser")
    @patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder")
    def test_execute_xacro_success(
        self, mock_builder, mock_parser, mock_xacro_resolver, tmp_path, scene, blender_context
    ) -> None:
        """Test execute operator with a valid XACRO file."""
        filepath = tmp_path / "robot.xacro"
        filepath.write_text("<robot name='xacro_robot'/>")

        # Mock Xacro resolve
        mock_xacro_resolver.return_value.resolve_file.return_value = "<robot name='xacro_robot'/>"

        # Mock parsed robot
        mock_robot = MagicMock()
        mock_robot.name = "xacro_robot"
        mock_robot.links = ["base_link"]
        mock_robot.joints = []
        mock_parser.return_value.parse_string.return_value = mock_robot

        # Mock validation
        mock_val_res = MagicMock()
        mock_val_res.is_valid = True
        mock_val_res.has_warnings = False

        op = LINKFORGE_OT_import_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        with (
            patch("linkforge.core.RobotValidator.validate", return_value=mock_val_res),
            patch("linkforge.blender.preferences.get_addon_prefs", return_value=None),
        ):
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}
            op.report.assert_any_call(
                {"INFO"}, "Started background import of XACRO: 'xacro_robot'..."
            )

    @patch("linkforge.core.URDFParser")
    def test_execute_directory_auto_detect(
        self, mock_parser, tmp_path, scene, blender_context
    ) -> None:
        """Test directory selection auto-detects robot file."""
        dir_path = tmp_path / "robot_dir"
        dir_path.mkdir()
        urdf_file = dir_path / "robot_dir.urdf"
        urdf_file.write_text("<robot name='dir_robot'/>")

        # Mock parsed robot
        mock_robot = MagicMock()
        mock_robot.name = "dir_robot"
        mock_robot.links = ["base_link"]
        mock_robot.joints = []
        mock_parser.return_value.parse.return_value = mock_robot

        # Mock validation
        mock_val_res = MagicMock()
        mock_val_res.is_valid = True
        mock_val_res.has_warnings = False

        op = LINKFORGE_OT_import_robot_model()
        op.filepath = str(dir_path)
        op.report = MagicMock()

        with (
            patch("linkforge.core.RobotValidator.validate", return_value=mock_val_res),
            patch("linkforge.blender.preferences.get_addon_prefs", return_value=None),
            patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder"),
        ):
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}
            op.report.assert_any_call({"INFO"}, "Auto-detected robot description: robot_dir.urdf")

    def test_execute_directory_no_file(self, tmp_path, scene, blender_context) -> None:
        """Test directory selection with no obvious robot file cancels execution."""
        dir_path = tmp_path / "empty_dir"
        dir_path.mkdir()

        op = LINKFORGE_OT_import_robot_model()
        op.filepath = str(dir_path)
        op.report = MagicMock()

        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}
        op.report.assert_any_call(
            {"ERROR"},
            "Directory selected but no obvious robot file found. Please select a .urdf or .xacro file directly.",
        )

    def test_execute_file_not_found(self, scene, blender_context) -> None:
        """Test execution cancels when the selected file does not exist."""
        op = LINKFORGE_OT_import_robot_model()
        op.filepath = "/nonexistent/path/robot.urdf"
        op.report = MagicMock()

        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}
        op.report.assert_any_call({"ERROR"}, "File not found: /nonexistent/path/robot.urdf")

    @patch("linkforge.core.URDFParser")
    def test_execute_urdf_to_xacro_fallback(
        self, mock_parser, tmp_path, scene, blender_context
    ) -> None:
        """Test fallback to XACRO parser when XacroDetectedError is raised."""
        filepath = tmp_path / "robot.urdf"
        filepath.write_text("<robot xmlns:xacro='...'/>")

        # Raise XacroDetectedError on URDF parse, then mock XACRO resolution success
        mock_parser.return_value.parse.side_effect = XacroDetectedError("Xacro elements detected")

        # Mock Xacro resolve
        mock_robot = MagicMock()
        mock_robot.name = "fallback_robot"
        mock_robot.links = ["base_link"]
        mock_robot.joints = []
        mock_parser.return_value.parse_string.return_value = mock_robot

        # Mock validation
        mock_val_res = MagicMock()
        mock_val_res.is_valid = True
        mock_val_res.has_warnings = False

        op = LINKFORGE_OT_import_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        with (
            patch("linkforge.core.XacroResolver.resolve_file", return_value="<xml/>"),
            patch("linkforge.core.RobotValidator.validate", return_value=mock_val_res),
            patch("linkforge.blender.preferences.get_addon_prefs", return_value=None),
            patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder"),
        ):
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}
            op.report.assert_any_call(
                {"WARNING"},
                "Detected XACRO content in robot model file. Switching to XACRO parser...",
            )

    @patch("linkforge.core.URDFParser")
    def test_execute_parser_error(self, mock_parser, tmp_path, scene, blender_context) -> None:
        """Test execution handles RobotParserError gracefully."""
        filepath = tmp_path / "invalid.urdf"
        filepath.write_text("<robot>")

        mock_parser.return_value.parse.side_effect = RobotParserError("Malformed XML")

        op = LINKFORGE_OT_import_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}
        op.report.assert_any_call({"ERROR"}, "URDF Parsing failed: Malformed XML")

    @patch("linkforge.core.URDFParser")
    def test_execute_empty_robot_model(self, mock_parser, tmp_path, scene, blender_context) -> None:
        """Test execution cancels when the robot model contains no links or joints."""
        filepath = tmp_path / "empty.urdf"
        filepath.write_text("<robot/>")

        mock_robot = MagicMock()
        mock_robot.name = "empty_robot"
        mock_robot.links = []
        mock_robot.joints = []
        mock_parser.return_value.parse.return_value = mock_robot

        op = LINKFORGE_OT_import_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}
        op.report.assert_any_call(
            {"ERROR"},
            "The file 'empty.urdf' contains no links or joints. It may be a macro-only XACRO file. Please import the top-level robot description instead.",
        )

    @patch("linkforge.core.URDFParser")
    def test_execute_validation_warnings_and_errors(
        self, mock_parser, tmp_path, scene, blender_context
    ) -> None:
        """Test reporting of validation errors and warnings."""
        filepath = tmp_path / "warn.urdf"
        filepath.write_text("<robot name='warn_robot'/>")

        mock_robot = MagicMock()
        mock_robot.name = "warn_robot"
        mock_robot.links = ["base_link"]
        mock_robot.joints = []
        mock_parser.return_value.parse.return_value = mock_robot

        # Create validation issues
        mock_issue = MagicMock()
        mock_issue.message = "Low mass detected"

        mock_val_res = MagicMock()
        mock_val_res.is_valid = False
        mock_val_res.errors = [mock_issue]
        mock_val_res.error_count = 1

        op = LINKFORGE_OT_import_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        with (
            patch("linkforge.core.RobotValidator.validate", return_value=mock_val_res),
            patch("linkforge.blender.preferences.get_addon_prefs", return_value=None),
            patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder"),
        ):
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}
            op.report.assert_any_call({"WARNING"}, "Validation Error: Low mass detected")

    def test_import_ops_invalid_context(self) -> None:
        """Verify import operator handles invalid context gracefully."""
        op = LINKFORGE_OT_import_robot_model()

        class MockContextNoScene:
            scene = None

        assert op.execute(MockContextNoScene()) == {"CANCELLED"}

    def test_registration(self, mocker) -> None:
        """Test register and unregister functions for import operator."""
        import linkforge.blender.operators.import_ops as import_ops

        with (
            patch("bpy.utils.register_class") as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            import_register()
            assert mock_reg.called

            import_unregister()
            assert mock_unreg.called

        # Test register double-registration with ValueError fallback
        mock_reg_err = mocker.patch(
            "bpy.utils.register_class", side_effect=[ValueError("Already registered"), None]
        )
        mock_unreg_err = mocker.patch("bpy.utils.unregister_class")
        import_ops.register()
        assert mock_reg_err.call_count > 0
        assert mock_unreg_err.call_count > 0

        # Run __main__ entrypoint
        import runpy

        with patch.object(import_ops, "__name__", "__main__"):
            runpy.run_module("linkforge.blender.operators.import_ops")

    @patch("linkforge.core.URDFParser")
    def test_execute_directory_single_valid_file(
        self, mock_parser, tmp_path, scene, blender_context
    ) -> None:
        """Test directory selection auto-detects single robot file when it is the only valid one."""
        dir_path = tmp_path / "single_file_dir"
        dir_path.mkdir()
        urdf_file = dir_path / "my_custom_name.urdf"
        urdf_file.write_text("<robot name='single_robot'/>")

        mock_robot = MagicMock()
        mock_robot.name = "single_robot"
        mock_robot.links = ["base_link"]
        mock_robot.joints = []
        mock_parser.return_value.parse.return_value = mock_robot

        mock_val_res = MagicMock()
        mock_val_res.is_valid = True
        mock_val_res.has_warnings = False

        op = LINKFORGE_OT_import_robot_model()
        op.filepath = str(dir_path)
        op.report = MagicMock()

        with (
            patch("linkforge.core.RobotValidator.validate", return_value=mock_val_res),
            patch("linkforge.blender.preferences.get_addon_prefs", return_value=None),
            patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder"),
        ):
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}
            op.report.assert_any_call(
                {"INFO"}, "Auto-detected single robot file: my_custom_name.urdf"
            )

    @patch("linkforge.core.URDFParser")
    def test_execute_unexpected_exception(
        self, mock_parser, tmp_path, scene, blender_context
    ) -> None:
        """Verify import operator traps unexpected non-LinkForge exceptions during parsing."""
        filepath = tmp_path / "robot.urdf"
        filepath.write_text("<xml/>")

        mock_parser.return_value.parse.side_effect = Exception("General OS read error")

        op = LINKFORGE_OT_import_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}
        op.report.assert_any_call({"ERROR"}, "Unexpected internal error: General OS read error")

    @patch("linkforge.core.URDFParser")
    def test_execute_validation_warnings_only(
        self, mock_parser, tmp_path, scene, blender_context
    ) -> None:
        """Test reporting of validation warnings when robot is valid but has warnings."""
        filepath = tmp_path / "warn_only.urdf"
        filepath.write_text("<robot name='warn_only_robot'/>")

        mock_robot = MagicMock()
        mock_robot.name = "warn_only_robot"
        mock_robot.links = ["base_link"]
        mock_robot.joints = []
        mock_parser.return_value.parse.return_value = mock_robot

        mock_val_res = MagicMock()
        mock_val_res.is_valid = True
        mock_val_res.has_warnings = True
        mock_val_res.warning_count = 3

        op = LINKFORGE_OT_import_robot_model()
        op.filepath = str(filepath)
        op.report = MagicMock()

        with (
            patch("linkforge.core.RobotValidator.validate", return_value=mock_val_res),
            patch("linkforge.blender.preferences.get_addon_prefs", return_value=None),
            patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder"),
        ):
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}
            op.report.assert_any_call({"INFO"}, "Imported robot 'warn_only_robot' with 3 warnings.")
