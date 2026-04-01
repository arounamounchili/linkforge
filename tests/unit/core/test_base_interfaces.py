import pytest
from linkforge_core.base import (
    LinkForgeError,
    RobotGenerator,
    RobotGeneratorError,
    RobotParser,
    XacroDetectedError,
)
from linkforge_core.models.robot import Robot
from linkforge_core.parsers.urdf_parser import URDFParser


class TestStringGenerator(RobotGenerator[str]):
    def generate(self, robot: Robot, **kwargs) -> str:
        suffix = kwargs.get("suffix", "")
        return f"Robot: {robot.name}{suffix}"


class TestBinaryGenerator(RobotGenerator[bytes]):
    def generate(self, robot: Robot, **kwargs) -> bytes:
        return b"\x00\x01\x02"

    # Needs no override for write(), but uses default bytes support


class TestUnsupportedTypeGenerator(RobotGenerator[dict]):
    def generate(self, robot: Robot, **kwargs) -> dict:
        return {"name": robot.name}


class TestLinkForgeErrorGenerator(RobotGenerator[str]):
    def generate(self, robot: Robot, **kwargs) -> str:
        raise LinkForgeError("Custom LinkForge error")


def test_auto_directory_creation(tmp_path) -> None:
    robot = Robot(name="test_bot")
    generator = TestStringGenerator()

    deep_path = tmp_path / "a" / "b" / "c" / "robot.txt"
    generator.write(robot, deep_path)

    assert deep_path.exists()
    assert deep_path.read_text() == "Robot: test_bot"


def test_kwargs_in_generate(tmp_path) -> None:
    robot = Robot(name="test_bot")
    generator = TestStringGenerator()

    content = generator.generate(robot, suffix="!!!")
    assert content == "Robot: test_bot!!!"


def test_binary_write_support(tmp_path) -> None:
    robot = Robot(name="test_bot")
    generator = TestBinaryGenerator()

    bin_path = tmp_path / "robot.bin"
    generator.write(robot, bin_path)

    assert bin_path.exists()
    assert bin_path.read_bytes() == b"\x00\x01\x02"


def test_robot_metadata_and_version() -> None:
    robot = Robot(name="test_bot", version="2.0", metadata={"author": "Antigravity"})
    assert robot.version == "2.0"
    assert robot.metadata["author"] == "Antigravity"


def test_custom_exception_wrapping(tmp_path) -> None:
    class ErrorGenerator(RobotGenerator[str]):
        def generate(self, robot: Robot, **kwargs) -> str:
            raise RuntimeError("Something went wrong internals")

    generator = ErrorGenerator()
    robot = Robot(name="fail")

    with pytest.raises(RobotGeneratorError) as excinfo:
        generator.write(robot, tmp_path / "fail.txt")

    assert "Something went wrong internals" in str(excinfo.value)


def test_linkforge_error_passthrough(tmp_path) -> None:
    """Test that LinkForgeError is re-raised without wrapping."""
    generator = TestLinkForgeErrorGenerator()
    robot = Robot(name="test")

    with pytest.raises(LinkForgeError) as excinfo:
        generator.write(robot, tmp_path / "test.txt")

    assert "Custom LinkForge error" in str(excinfo.value)
    assert not isinstance(excinfo.value, RobotGeneratorError)


def test_unsupported_content_type(tmp_path) -> None:
    """Test that unsupported content types raise RobotGeneratorError."""
    generator = TestUnsupportedTypeGenerator()
    robot = Robot(name="test")

    with pytest.raises(RobotGeneratorError) as excinfo:
        generator.write(robot, tmp_path / "test.txt")

    assert "<class 'dict'>" in str(excinfo.value)
    assert "TestUnsupportedTypeGenerator" in str(excinfo.value)


def test_parser_detects_xacro_in_urdf() -> None:
    parser = URDFParser()
    xacro_content = (
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:macro name="test"/></robot>'
    )

    with pytest.raises(XacroDetectedError) as excinfo:
        parser.parse_string(xacro_content)

    assert "XACRO file detected" in str(excinfo.value)


def test_xacro_parser_basic(tmp_path) -> None:
    from linkforge_core.parsers.xacro_parser import XACROParser

    xacro_path = tmp_path / "robot.xacro"
    xacro_path.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_bot"><link name="base_link"/></robot>'
    )

    parser = XACROParser()
    robot = parser.parse(xacro_path)

    assert robot.name == "xacro_bot"
    assert any(link.name == "base_link" for link in robot.links)


def test_abstract_generator_cannot_instantiate() -> None:
    """Test that abstract RobotGenerator cannot be instantiated."""
    with pytest.raises(TypeError):
        RobotGenerator()  # type: ignore


def test_abstract_parser_cannot_instantiate() -> None:
    """Test that abstract RobotParser cannot be instantiated."""
    with pytest.raises(TypeError):
        RobotParser()  # type: ignore
