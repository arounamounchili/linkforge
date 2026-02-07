from unittest.mock import MagicMock

from linkforge.blender.operators.export_ops import (
    LINKFORGE_OT_export_urdf,
    LINKFORGE_OT_validate_robot,
)


def test_export_urdf_invoke(mocker):
    """Test the invoke method logic using the unbound class method."""

    # Mock self as a generic object with the required attributes
    mock_self = MagicMock()
    mock_self.filename_ext = ""

    context = MagicMock()
    context.scene.linkforge.export_format = "XACRO"

    # We test the logic before the super().invoke call
    LINKFORGE_OT_export_urdf.invoke(mock_self, context, None)
    assert mock_self.filename_ext == ".xacro"

    context.scene.linkforge.export_format = "URDF"
    LINKFORGE_OT_export_urdf.invoke(mock_self, context, None)
    assert mock_self.filename_ext == ".urdf"


def test_export_urdf_execute_xacro_and_dry_run(mocker):
    """Test XACRO generation and current 'dry run' behavior."""

    mock_self = MagicMock(spec=LINKFORGE_OT_export_urdf)
    mock_self.filepath = "/tmp/robot.urdf"
    mock_self.report = MagicMock()

    context = MagicMock()
    context.scene.linkforge.export_format = "XACRO"
    context.scene.linkforge.export_meshes = False
    context.scene.linkforge.validate_before_export = True

    # Mock validation pass
    val_res = MagicMock()
    val_res.is_valid = True
    val_res.has_warnings = False
    mocker.patch(
        "linkforge.linkforge_core.validation.RobotValidator.validate", return_value=val_res
    )

    mocker.patch(
        "linkforge.blender.adapters.blender_to_core.scene_to_robot", return_value=(MagicMock(), {})
    )
    mock_gen = mocker.patch("linkforge.linkforge_core.XACROGenerator")

    result = LINKFORGE_OT_export_urdf.execute(mock_self, context)

    assert result == {"FINISHED"}
    # The operator currently ALWAYS calls write, dry_run only affects meshes
    mock_gen.return_value.write.assert_called_once()


def test_export_validation_failure(mocker):
    """Test that export cancels if validation fails."""

    mock_self = MagicMock(spec=LINKFORGE_OT_export_urdf)
    mock_self.report = MagicMock()

    context = MagicMock()
    context.scene.linkforge.validate_before_export = True

    mocker.patch(
        "linkforge.blender.adapters.blender_to_core.scene_to_robot", return_value=(MagicMock(), {})
    )

    val_res = MagicMock()
    val_res.is_valid = False
    val_res.error_count = 1
    mocker.patch(
        "linkforge.linkforge_core.validation.RobotValidator.validate", return_value=val_res
    )

    result = LINKFORGE_OT_export_urdf.execute(mock_self, context)
    assert result == {"CANCELLED"}
    mock_self.report.assert_called_with({"ERROR"}, mocker.ANY)


def test_validate_robot_error_parsing(mocker):
    """Test that validation operator parses multi-line errors correctly."""

    mock_self = MagicMock(spec=LINKFORGE_OT_validate_robot)
    mock_self.report = MagicMock()

    context = MagicMock()
    val_props = MagicMock()
    mock_errors = MagicMock()
    val_props.errors = mock_errors
    context.window_manager.linkforge_validation = val_props

    # Simulate multi-line build error
    msg = "Unable to build robot model.\nLine 1\nLine 2"
    mocker.patch(
        "linkforge.blender.adapters.blender_to_core.scene_to_robot", side_effect=ValueError(msg)
    )

    LINKFORGE_OT_validate_robot.execute(mock_self, context)

    assert mock_errors.add.call_count == 2
    assert val_props.is_valid is False
