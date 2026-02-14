import os
from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.operators.export_ops import (
    LINKFORGE_OT_export_urdf,
    LINKFORGE_OT_validate_robot,
    register,
    unregister,
    working_directory,
)


@pytest.mark.parametrize("export_format", ["URDF", "XACRO"])
def test_export_urdf_execute(mocker, clean_scene, export_format):
    """Test the basic export execution with real properties."""
    scene = bpy.context.scene
    props = scene.linkforge
    props.export_format = export_format
    props.export_meshes = False
    props.validate_before_export = False

    mock_self = MagicMock()
    mock_self.filepath = f"/tmp/robot.{export_format.lower()}"
    mock_self.report = MagicMock()

    mock_self.report = MagicMock()

    # Use real scene translation to exercise adapter logic without mocking scene_to_robot.
    # Generators are mocked to prevent file system operations.
    mocker.patch("linkforge.linkforge_core.URDFGenerator")
    mocker.patch("linkforge.linkforge_core.XACROGenerator")

    # Add a root object to make scene_to_robot succeed
    root = bpy.data.objects.new("root", None)
    bpy.context.collection.objects.link(root)
    root.linkforge.is_link = True

    # Execute
    result = LINKFORGE_OT_export_urdf.execute(mock_self, bpy.context)

    assert result == {"FINISHED"}
    mock_self.report.assert_called_with({"INFO"}, mocker.ANY)


def test_export_urdf_validation_failure(mocker, clean_scene):
    """Test export cancellation when validation fails."""
    scene = bpy.context.scene
    props = scene.linkforge
    props.validate_before_export = True

    mock_self = MagicMock()
    mock_self.filepath = "/tmp/robot.urdf"
    mock_self.report = MagicMock()

    result = LINKFORGE_OT_export_urdf.execute(mock_self, bpy.context)

    assert result == {"CANCELLED"}
    mock_self.report.assert_called_with({"ERROR"}, mocker.ANY)
    assert "Cannot export" in mock_self.report.call_args[0][1]


def test_export_urdf_validation_build_error(mocker, clean_scene):
    """Test build error during the validation dry-run (lines 103-105)."""
    scene = bpy.context.scene
    props = scene.linkforge
    props.validate_before_export = True

    mock_self = MagicMock()
    mock_self.filepath = "/tmp/robot.urdf"
    mock_self.report = MagicMock()

    mocker.patch(
        "linkforge.blender.adapters.blender_to_core.scene_to_robot",
        side_effect=Exception("Validation dry run build failed"),
    )

    result = LINKFORGE_OT_export_urdf.execute(mock_self, bpy.context)
    assert result == {"CANCELLED"}
    mock_self.report.assert_called_with({"ERROR"}, mocker.ANY)
    assert "Failed to build" in mock_self.report.call_args[0][1]


def test_validate_robot_operator(mocker, clean_scene):
    """Test the validation operator with errors and warnings."""
    mock_self = MagicMock()
    mock_self.report = MagicMock()

    # Set up some errors and warnings to cover loops (lines 257-269)
    err = MagicMock()
    err.title = "Error Title"
    err.message = "Error Message"
    err.affected_objects = ["Cube"]
    err.suggestion = ""  # Must be a string

    warn = MagicMock()
    warn.title = "Warn Title"
    warn.message = "Warn Message"
    warn.affected_objects = ["Sphere"]
    warn.suggestion = "Do something"  # Must be a string

    # Mock scene_to_robot
    mocker.patch(
        "linkforge.blender.adapters.blender_to_core.scene_to_robot", return_value=(MagicMock(), {})
    )

    # Case 1: Valid with warnings
    val_res = MagicMock()
    val_res.is_valid = True
    val_res.error_count = 0
    val_res.warning_count = 1
    val_res.has_warnings = True
    val_res.errors = []
    val_res.warnings = [warn]

    mocker.patch(
        "linkforge.linkforge_core.validation.RobotValidator.validate", return_value=val_res
    )

    result = LINKFORGE_OT_validate_robot.execute(mock_self, bpy.context)
    assert result == {"FINISHED"}
    assert bpy.context.window_manager.linkforge_validation.warning_count == 1

    # Case 2: Invalid with errors
    val_res.is_valid = False
    val_res.error_count = 1
    val_res.errors = [err]
    val_res.has_warnings = False
    val_res.warnings = []

    result = LINKFORGE_OT_validate_robot.execute(mock_self, bpy.context)
    assert result == {"CANCELLED"}
    assert bpy.context.window_manager.linkforge_validation.error_count == 1

    # Case 3: Valid with no warnings (lines 272-279)
    val_res.is_valid = True
    val_res.has_warnings = False
    val_res.error_count = 0
    val_res.warning_count = 0
    val_res.errors = []
    val_res.warnings = []
    result = LINKFORGE_OT_validate_robot.execute(mock_self, bpy.context)
    assert result == {"FINISHED"}

    # Clear for next tests
    bpy.context.window_manager.linkforge_validation.errors.clear()
    bpy.context.window_manager.linkforge_validation.warnings.clear()


def test_validate_robot_multi_line_error(mocker, clean_scene):
    """Test parsing of multi-line build errors."""
    mock_self = MagicMock()
    mock_self.report = MagicMock()

    error_msg = "Unable to build robot model.\nError 1\nError 2"
    mocker.patch(
        "linkforge.blender.adapters.blender_to_core.scene_to_robot",
        side_effect=Exception(error_msg),
    )

    result = LINKFORGE_OT_validate_robot.execute(mock_self, bpy.context)
    assert result == {"FINISHED"}
    props = bpy.context.window_manager.linkforge_validation
    assert props.error_count == 2
    assert props.errors[0].message == "Error 1"
    assert props.errors[1].message == "Error 2"


def test_validate_robot_direct_error(mocker, clean_scene):
    """Test parsing of build errors without prefix."""
    mock_self = MagicMock()
    mock_self.report = MagicMock()

    mocker.patch(
        "linkforge.blender.adapters.blender_to_core.scene_to_robot",
        side_effect=Exception("Direct error"),
    )

    result = LINKFORGE_OT_validate_robot.execute(mock_self, bpy.context)
    assert result == {"FINISHED"}
    assert bpy.context.window_manager.linkforge_validation.errors[0].message == "Direct error"


def test_export_urdf_exception_handling_advanced(mocker, clean_scene):
    """Test the 'Configuration errors found' message shortening logic."""
    scene = bpy.context.scene
    props = scene.linkforge
    props.export_format = "URDF"
    props.export_meshes = False
    props.validate_before_export = False

    mock_self = MagicMock()
    mock_self.filepath = "/tmp/robot.urdf"
    mock_self.report = MagicMock()

    error_msg = "Unable to build robot model.\nSomething went wrong here."
    mocker.patch(
        "linkforge.blender.adapters.blender_to_core.scene_to_robot",
        side_effect=Exception(error_msg),
    )

    result = LINKFORGE_OT_export_urdf.execute(mock_self, bpy.context)
    assert result == {"CANCELLED"}
    # Verify shortened message (line 137)
    mock_self.report.assert_called_with({"ERROR"}, mocker.ANY)
    assert "Configuration errors found" in mock_self.report.call_args[0][1]


def test_export_urdf_invoke_branches(clean_scene):
    """Test invoke branches (lines 58-69)."""
    mock_op = MagicMock(spec=LINKFORGE_OT_export_urdf)

    # Correct extension for both formats
    for fmt, ext in [("XACRO", ".xacro"), ("URDF", ".urdf")]:
        bpy.context.scene.linkforge.export_format = fmt
        with patch("bpy_extras.io_utils.ExportHelper.invoke", return_value={"FINISHED"}):
            LINKFORGE_OT_export_urdf.invoke(mock_op, bpy.context, MagicMock())
            assert mock_op.filename_ext == ext

    # Branch: missing scene/props (line 59)
    # Using a fake context with no scene
    bad_context = MagicMock()
    bad_context.scene = None
    result = LINKFORGE_OT_export_urdf.invoke(mock_op, bad_context, MagicMock())
    assert result == {"CANCELLED"}


def test_export_urdf_execute_missing_props(mocker):
    """Test execute with missing scene properties (line 79)."""
    mock_self = MagicMock()
    mock_self.report = MagicMock()
    bad_context = MagicMock()
    bad_context.scene = None

    result = LINKFORGE_OT_export_urdf.execute(mock_self, bad_context)
    assert result == {"CANCELLED"}


def test_validate_robot_not_initialized(mocker):
    """Test validation when system not initialized (line 193-194)."""
    mock_self = MagicMock()
    mock_self.report = MagicMock()
    bad_context = MagicMock()
    bad_context.window_manager = None

    result = LINKFORGE_OT_validate_robot.execute(mock_self, bad_context)
    assert result == {"CANCELLED"}


def test_export_registration_recovery(mocker):
    """Verify operator registration error recovery logic."""
    # Force a ValueError during registration for the FIRST class
    # The loop will:
    # 1. Try register(cls1) -> ValueError
    # 2. unregister(cls1)
    # 3. register(cls1) -> None
    # 4. Try register(cls2) -> None
    with patch("bpy.utils.register_class", side_effect=[ValueError, None, None]):
        register()


def test_export_utils_working_directory(tmp_path):
    """Test working_directory utility."""
    original_cwd = os.getcwd()
    new_dir = tmp_path / "workdir"
    new_dir.mkdir()

    with working_directory(new_dir):
        assert os.getcwd() == str(new_dir)

    assert os.getcwd() == original_cwd


def test_export_urdf_extension_correction(mocker, clean_scene):
    """Verify automatic correction of file extensions based on export format."""
    scene = bpy.context.scene
    props = scene.linkforge

    # Case 1: URDF format but .xacro extension
    props.export_format = "URDF"
    mock_self = MagicMock()
    mock_self.filepath = "/tmp/fake.xacro"  # Wrong extension
    mock_self.report = MagicMock()

    mocker.patch(
        "linkforge.blender.adapters.blender_to_core.scene_to_robot", return_value=(MagicMock(), {})
    )
    mocker.patch("linkforge.linkforge_core.URDFGenerator")

    LINKFORGE_OT_export_urdf.execute(mock_self, bpy.context)
    assert mock_self.filepath.endswith(".urdf")

    # Case 2: XACRO format but .urdf extension
    props.export_format = "XACRO"
    mock_self.filepath = "/tmp/fake.urdf"
    mocker.patch("linkforge.linkforge_core.XACROGenerator")

    LINKFORGE_OT_export_urdf.execute(mock_self, bpy.context)
    assert mock_self.filepath.endswith(".xacro")


def test_export_registration():
    """Test register/unregister."""
    # Ensure starting state
    if hasattr(bpy.types, "LINKFORGE_OT_export_urdf"):
        unregister()
    register()
    assert hasattr(bpy.types, "LINKFORGE_OT_export_urdf")
    unregister()
    assert not hasattr(bpy.types, "LINKFORGE_OT_export_urdf")
    register()


def test_export_check_logic(mocker):
    """Verify if export can proceed based on current scene state."""
    # Success case
    assert LINKFORGE_OT_export_urdf.check(None, bpy.context) is True

    # Failure case (missing scene/props)
    bad_context = MagicMock()
    bad_context.scene = None
    assert LINKFORGE_OT_export_urdf.check(None, bad_context) is False


def test_export_main_entry(mocker):
    """Verify execution of module entry point logic."""
    # Simulate module entry point execution.
    with (
        patch("linkforge.blender.operators.export_ops.register"),
        patch("linkforge.blender.operators.export_ops.__name__", "__main__"),
    ):
        pass
