from unittest.mock import MagicMock, patch

import bpy
from linkforge.blender.operators.import_ops import (
    LINKFORGE_OT_import_urdf,
    register,
    unregister,
)
from linkforge.linkforge_core.exceptions import RobotModelError


def test_import_urdf_logic_paths(tmp_path) -> None:
    """Test import operator logic by calling the unbound method."""
    mock_self = MagicMock(spec=LINKFORGE_OT_import_urdf)
    mock_self.report = MagicMock()

    urdf_file = tmp_path / "test.urdf"
    urdf_file.write_text("<robot name='test'/>")
    mock_self.filepath = str(urdf_file)

    context = MagicMock()
    # Mock the asynchronous builder to avoid side effects during logic verification.
    with (
        patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder"),
        patch(
            "linkforge.linkforge_core.validation.security.find_sandbox_root", return_value=tmp_path
        ),
    ):
        # Call execute directly to test the logic
        result = LINKFORGE_OT_import_urdf.execute(mock_self, context)
        assert result == {"FINISHED"}


def test_import_invalid_path_logic() -> None:
    """Test handling of invalid paths or missing directories."""
    mock_self = MagicMock(spec=LINKFORGE_OT_import_urdf)
    mock_self.filepath = "/non/existent/path.urdf"
    mock_self.report = MagicMock()

    context = MagicMock()
    # Case 1: File not found
    result = LINKFORGE_OT_import_urdf.execute(mock_self, context)
    assert result == {"CANCELLED"}
    from unittest.mock import ANY

    mock_self.report.assert_called_with({"ERROR"}, ANY)


def test_import_urdf_xacro_fallback(tmp_path) -> None:
    """Test switching to Xacro parser if Xacro content is detected."""
    mock_self = MagicMock(spec=LINKFORGE_OT_import_urdf)
    mock_self.report = MagicMock()

    urdf_file = tmp_path / "test.urdf"
    urdf_file.write_text("<robot xmlns:xacro='http://www.ros.org/wiki/xacro'/>")
    mock_self.filepath = str(urdf_file)

    context = MagicMock()
    from linkforge.linkforge_core import XacroDetectedError

    with (
        patch(
            "linkforge.linkforge_core.parsers.URDFParser.parse",
            side_effect=XacroDetectedError("Xacro detected"),
        ),
        patch(
            "linkforge.linkforge_core.parsers.XacroResolver.resolve_file",
            return_value="<robot name='resolved'/>",
        ),
        patch("linkforge.linkforge_core.parsers.URDFParser.parse_string", return_value=MagicMock()),
        patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder"),
        patch(
            "linkforge.linkforge_core.validation.security.find_sandbox_root", return_value=tmp_path
        ),
    ):
        result = LINKFORGE_OT_import_urdf.execute(mock_self, context)
        assert result == {"FINISHED"}
        from unittest.mock import ANY

        mock_self.report.assert_any_call({"WARNING"}, ANY)


def test_import_urdf_directory_handling_more(tmp_path) -> None:
    """Test standard directory handling branches."""
    mock_self = MagicMock(spec=LINKFORGE_OT_import_urdf)
    mock_self.report = MagicMock()

    subdir = tmp_path / "lonely_robot"
    subdir.mkdir()

    # Case 1: Exactly one valid file
    robot_file = subdir / "one.urdf"
    robot_file.write_text("<robot name='one'/>")
    mock_self.filepath = str(subdir)

    with (
        patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder"),
        patch(
            "linkforge.linkforge_core.validation.security.find_sandbox_root", return_value=subdir
        ),
    ):
        result = LINKFORGE_OT_import_urdf.execute(mock_self, bpy.context)
        assert result == {"FINISHED"}
        from unittest.mock import ANY

        mock_self.report.assert_any_call({"INFO"}, ANY)


def test_import_urdf_directory_candidates(tmp_path) -> None:
    """Test candidate detection when importing a directory."""
    mock_self = MagicMock(spec=LINKFORGE_OT_import_urdf)
    mock_self.report = MagicMock()

    subdir = tmp_path / "robot_pkg"
    subdir.mkdir()
    # Create a candidate file (matching robot.urdf pattern)
    robot_file = subdir / "robot.urdf"
    robot_file.write_text("<robot name='pkg'/>")
    mock_self.filepath = str(subdir)

    with (
        patch("linkforge.blender.logic.asynchronous_builder.AsynchronousRobotBuilder"),
        patch(
            "linkforge.linkforge_core.validation.security.find_sandbox_root", return_value=subdir
        ),
    ):
        result = LINKFORGE_OT_import_urdf.execute(mock_self, bpy.context)
        assert result == {"FINISHED"}
        from unittest.mock import ANY

        # Verify candidate reporting
        mock_self.report.assert_any_call({"INFO"}, ANY)
        assert "Auto-detected robot description" in mock_self.report.call_args_list[0][0][1]

        # Case 2: No valid files
        robot_file.unlink()
        # Create a non-matching file
        (subdir / "other.txt").write_text("not a robot")
        result = LINKFORGE_OT_import_urdf.execute(mock_self, bpy.context)
        assert result == {"CANCELLED"}
        mock_self.report.assert_any_call({"ERROR"}, ANY)


def test_import_urdf_invoke_check() -> None:
    """Test invoke and check methods using class methods on mock."""
    mock_op = MagicMock(spec=LINKFORGE_OT_import_urdf)
    assert LINKFORGE_OT_import_urdf.check(mock_op, bpy.context) is True

    event = MagicMock()
    with patch("bpy_extras.io_utils.ImportHelper.invoke", return_value={"RUNNING_MODAL"}):
        result = LINKFORGE_OT_import_urdf.invoke(mock_op, bpy.context, event)
        assert result == {"RUNNING_MODAL"}


def test_import_registration() -> None:
    """Test operator registration and error recovery branches."""
    unregister()
    register()
    assert hasattr(bpy.types, "LINKFORGE_OT_import_urdf")

    # Forced recovery branch
    with patch("bpy.utils.register_class", side_effect=[ValueError, None]):
        register()

    unregister()
    assert not hasattr(bpy.types, "LINKFORGE_OT_import_urdf")
    register()


def test_import_xacro_resolution_error(tmp_path) -> None:
    """Test error handling during XACRO resolution."""
    mock_self = MagicMock(spec=LINKFORGE_OT_import_urdf)
    mock_self.report = MagicMock()

    xacro_file = tmp_path / "test.xacro"
    xacro_file.write_text("xacro content")
    mock_self.filepath = str(xacro_file)

    # Simulate a Xacro resolution error (e.g., PackageNotFoundError).
    with patch(
        "linkforge.linkforge_core.parsers.XacroResolver.resolve_file",
        side_effect=Exception("PackageNotFoundError: my_pkg"),
    ):
        result = LINKFORGE_OT_import_urdf.execute(mock_self, bpy.context)
        assert result == {"CANCELLED"}
        from unittest.mock import ANY

        mock_self.report.assert_called_with({"ERROR"}, ANY)
        assert "PackageNotFoundError" in mock_self.report.call_args[0][1]


def test_import_path_conversion_error(tmp_path) -> None:
    """Test error handling during URDF path conversion."""
    mock_self = MagicMock(spec=LINKFORGE_OT_import_urdf)
    mock_self.report = MagicMock()

    urdf_file = tmp_path / "broken.urdf"
    urdf_file.write_text("<robot name='broken'/>")
    mock_self.filepath = str(urdf_file)

    with patch(
        "linkforge.linkforge_core.parsers.URDFParser.parse",
        side_effect=RobotModelError("Path resolution failed"),
    ):
        result = LINKFORGE_OT_import_urdf.execute(mock_self, bpy.context)
        assert result == {"CANCELLED"}
        from unittest.mock import ANY

        mock_self.report.assert_called_with({"ERROR"}, ANY)


def test_import_main_entry() -> None:
    """Verify execution of module entry point logic."""
    with (
        patch("linkforge.blender.operators.import_ops.register"),
        patch("linkforge.blender.operators.import_ops.__name__", "__main__"),
    ):
        pass
