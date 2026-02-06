from unittest.mock import MagicMock

from linkforge.blender.operators.export_ops import (
    LINKFORGE_OT_export_urdf,
    LINKFORGE_OT_validate_robot,
)

# NOTE: We can't instantiate BPY operators directly.
# We test the logic by mocking the internals or using the registered operator via bpy.ops.


def test_export_urdf_logic_paths(mocker):
    """Test the logic inside export execute by mocking the scene environment."""
    # We test the execute method by patching its dependencies and passing a mock 'self'

    mock_self = MagicMock(spec=LINKFORGE_OT_export_urdf)
    mock_self.filepath = "/tmp/robot.urdf"
    mock_self.report = MagicMock()

    context = MagicMock()
    context.scene.linkforge.export_format = "URDF"
    context.scene.linkforge.export_meshes = False
    context.scene.linkforge.validate_before_export = False

    mocker.patch("linkforge.blender.converters.scene_to_robot", return_value=(MagicMock(), {}))
    mock_gen = mocker.patch("linkforge.linkforge_core.URDFGenerator")

    # Call the unbound method
    result = LINKFORGE_OT_export_urdf.execute(mock_self, context)

    assert result == {"FINISHED"}
    mock_gen.return_value.write.assert_called_once()


def test_validate_robot_logic_paths(mocker):
    """Test the validation operator logic."""

    mock_self = MagicMock(spec=LINKFORGE_OT_validate_robot)
    mock_self.report = MagicMock()

    context = MagicMock()
    val_props = MagicMock()
    context.window_manager.linkforge_validation = val_props

    mocker.patch("linkforge.blender.converters.scene_to_robot", return_value=(MagicMock(), {}))
    mock_validator = mocker.patch("linkforge.linkforge_core.validation.RobotValidator")

    res = MagicMock()
    res.is_valid = True
    res.has_warnings = False
    res.errors = []
    res.warnings = []
    mock_validator.return_value.validate.return_value = res

    result = LINKFORGE_OT_validate_robot.execute(mock_self, context)

    assert result == {"FINISHED"}
    assert val_props.is_valid is True
