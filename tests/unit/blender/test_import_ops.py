from unittest.mock import MagicMock

from linkforge.blender.operators.import_ops import LINKFORGE_OT_import_urdf


def test_import_urdf_logic_paths(mocker, tmp_path):
    """Test import operator logic by calling the unbound method."""
    mock_self = MagicMock(spec=LINKFORGE_OT_import_urdf)
    mock_self.report = MagicMock()

    urdf_file = tmp_path / "test.urdf"
    urdf_file.write_text("<robot name='test'/>")
    mock_self.filepath = str(urdf_file)

    context = MagicMock()
    mocker.patch("linkforge.linkforge_core.parsers.URDFParser")
    mocker.patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder")
    mocker.patch(
        "linkforge.linkforge_core.validation.security.find_sandbox_root", return_value=tmp_path
    )

    # Call invoke directly to test the logic (execute calls self.invoke which is mocked on the instance)
    result = LINKFORGE_OT_import_urdf.invoke(mock_self, context, None)
    assert result == {"FINISHED"}


def test_import_invalid_path_logic(mocker):
    """Test handling of invalid paths."""
    mock_self = MagicMock(spec=LINKFORGE_OT_import_urdf)
    mock_self.filepath = "/non/existent/path.urdf"
    mock_self.report = MagicMock()

    context = MagicMock()
    # Call invoke directly to test the logic
    result = LINKFORGE_OT_import_urdf.invoke(mock_self, context, None)
    assert result == {"CANCELLED"}
    mock_self.report.assert_called_with({"ERROR"}, mocker.ANY)
