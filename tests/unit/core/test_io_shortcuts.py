from pathlib import Path

from linkforge.core.io import (
    read_srdf,
    read_urdf,
    read_xacro,
    validate_robot,
    write_srdf,
    write_urdf,
    write_xacro,
)
from linkforge.core.models.robot import Robot
from linkforge.core.models.srdf import SemanticRobotDescription


def test_read_urdf_from_string(mocker):
    """Verify read_urdf delegates to URDFParser.parse_string for raw XML."""
    mock_parser = mocker.patch("linkforge.core.parsers.URDFParser")
    xml_content = "<robot name='test'/>"

    read_urdf(xml_content)

    mock_parser.return_value.parse_string.assert_called_once_with(xml_content)


def test_read_urdf_from_file(mocker, tmp_path):
    """Verify read_urdf delegates to URDFParser.parse for a file path."""
    mock_parser = mocker.patch("linkforge.core.parsers.URDFParser")
    urdf_file = tmp_path / "test.urdf"
    urdf_file.write_text("<robot name='test'/>")

    read_urdf(urdf_file)

    mock_parser.return_value.parse.assert_called_once_with(Path(urdf_file))


def test_write_urdf(mocker, tmp_path):
    """Verify write_urdf delegates to URDFGenerator."""
    mock_gen = mocker.patch("linkforge.core.generators.URDFGenerator")
    robot = Robot(name="test")
    dest = tmp_path / "out.urdf"

    write_urdf(robot, dest)

    mock_gen.return_value.write.assert_called_once_with(robot, Path(dest))


def test_read_xacro(mocker, tmp_path):
    """Verify read_xacro orchestrates XACRO resolution and URDF parsing."""
    mock_xacro = mocker.patch("linkforge.core.parsers.XACROParser")
    mock_urdf = mocker.patch("linkforge.core.parsers.URDFParser")
    mock_xacro.return_value.resolve.return_value = "<xml/>"

    xacro_file = tmp_path / "test.xacro"
    xacro_file.write_text("<robot/>")

    read_xacro(xacro_file, arg1="val1")

    mock_xacro.return_value.resolve.assert_called_once_with(Path(xacro_file), arg1="val1")
    mock_urdf.return_value.parse_string.assert_called_once_with("<xml/>")


def test_write_xacro(mocker, tmp_path):
    """Verify write_xacro delegates to XACROGenerator."""
    mock_gen = mocker.patch("linkforge.core.generators.XACROGenerator")
    robot = Robot(name="test")
    dest = tmp_path / "out.xacro"

    write_xacro(robot, dest)

    mock_gen.return_value.write.assert_called_once_with(robot, Path(dest))


def test_read_srdf_from_string(mocker):
    """Verify read_srdf delegates to SRDFParser.parse_string."""
    mock_parser = mocker.patch("linkforge.core.parsers.SRDFParser")
    xml = "<robot-vc name='test'/>"
    robot = Robot(name="test")

    read_srdf(xml, robot=robot)

    mock_parser.return_value.parse_string.assert_called_once_with(xml, robot=robot)


def test_read_srdf_from_file(mocker, tmp_path):
    """Verify read_srdf delegates to SRDFParser.parse for a file path."""
    mock_parser = mocker.patch("linkforge.core.parsers.SRDFParser")
    srdf_file = tmp_path / "test.srdf"
    srdf_file.write_text("<robot-vc name='test'/>")
    robot = Robot(name="test")

    read_srdf(srdf_file, robot=robot)

    mock_parser.return_value.parse.assert_called_once_with(Path(srdf_file), robot=robot)


def test_write_srdf_from_robot(mocker, tmp_path):
    """Verify write_srdf handles Robot objects directly."""
    mock_gen = mocker.patch("linkforge.core.generators.SRDFGenerator")
    robot = Robot(name="test")
    dest = tmp_path / "out.srdf"

    write_srdf(robot, dest)

    mock_gen.return_value.write.assert_called_once_with(robot, Path(dest))


def test_write_srdf_from_description(mocker, tmp_path):
    """Verify write_srdf wraps SemanticRobotDescription in a Robot model."""
    mock_gen = mocker.patch("linkforge.core.generators.SRDFGenerator")
    srdf = SemanticRobotDescription(robot_name="test")
    dest = tmp_path / "out.srdf"

    write_srdf(srdf, dest)

    # It should wrap it in a robot model
    args, _ = mock_gen.return_value.write.call_args
    called_robot = args[0]
    assert isinstance(called_robot, Robot)
    assert called_robot.semantic == srdf
    assert args[1] == Path(dest)


def test_validate_robot(mocker):
    """Verify validate_robot delegates to RobotValidator."""
    mock_val = mocker.patch("linkforge.core.validation.RobotValidator")
    robot = Robot(name="test")

    validate_robot(robot)

    mock_val.return_value.validate.assert_called_once_with(robot)
