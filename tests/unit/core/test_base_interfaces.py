import pytest
from linkforge_core.base import RobotGenerator, RobotGeneratorError, RobotParserError
from linkforge_core.models.robot import Robot
from linkforge_core.parsers.urdf_parser import URDFParser


class MockStringGenerator(RobotGenerator[str]):
    def generate(self, robot: Robot, **kwargs) -> str:
        suffix = kwargs.get("suffix", "")
        return f"Robot: {robot.name}{suffix}"


class MockBinaryGenerator(RobotGenerator[bytes]):
    def generate(self, robot: Robot, **kwargs) -> bytes:
        return b"\x00\x01\x02"

    # Needs no override for write(), but uses default bytes support


def test_auto_directory_creation(tmp_path):
    robot = Robot(name="test_bot")
    generator = MockStringGenerator()

    deep_path = tmp_path / "a" / "b" / "c" / "robot.txt"
    generator.write(robot, deep_path)

    assert deep_path.exists()
    assert deep_path.read_text() == "Robot: test_bot"


def test_kwargs_in_generate(tmp_path):
    robot = Robot(name="test_bot")
    generator = MockStringGenerator()

    content = generator.generate(robot, suffix="!!!")
    assert content == "Robot: test_bot!!!"


def test_binary_write_support(tmp_path):
    robot = Robot(name="test_bot")
    generator = MockBinaryGenerator()

    bin_path = tmp_path / "robot.bin"
    generator.write(robot, bin_path)

    assert bin_path.exists()
    assert bin_path.read_bytes() == b"\x00\x01\x02"


def test_robot_metadata_and_version():
    robot = Robot(name="test_bot", version="2.0", metadata={"author": "Antigravity"})
    assert robot.version == "2.0"
    assert robot.metadata["author"] == "Antigravity"


def test_custom_exception_wrapping(tmp_path):
    class ErrorGenerator(RobotGenerator[str]):
        def generate(self, robot: Robot, **kwargs) -> str:
            raise RuntimeError("Something went wrong internals")

    generator = ErrorGenerator()
    robot = Robot(name="fail")

    with pytest.raises(RobotGeneratorError) as excinfo:
        generator.write(robot, tmp_path / "fail.txt")

    assert "Something went wrong internals" in str(excinfo.value)


def test_parser_detects_xacro_in_urdf():
    parser = URDFParser()
    xacro_content = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"></robot>'

    with pytest.raises(RobotParserError) as excinfo:
        parser.parse_string(xacro_content)

    assert "XACRO file detected" in str(excinfo.value)


def test_xacro_parser_basic(tmp_path):
    from linkforge_core.parsers.xacro_parser import XACROParser

    xacro_path = tmp_path / "robot.xacro"
    xacro_path.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_bot"><link name="base_link"/></robot>'
    )

    parser = XACROParser()
    robot = parser.parse(xacro_path)

    assert robot.name == "xacro_bot"
    assert any(link.name == "base_link" for link in robot.links)
