"""Unit tests for the Abstract Resource Resolver system."""

from pathlib import Path

import pytest
from linkforge_core.base import FileSystemResolver, NetworkResolver
from linkforge_core.models.geometry import Mesh
from linkforge_core.models.robot import Robot


def test_file_system_resolver_absolute_path(tmp_path: Path):
    """Test that FileSystemResolver correctly resolves an existing absolute path."""
    test_file = tmp_path / "test_mesh.stl"
    test_file.write_text("dummy mesh data")

    resolver = FileSystemResolver()
    resolved = resolver.resolve(str(test_file))

    assert resolved == test_file.absolute()
    assert resolved.exists()


def test_file_system_resolver_non_existent():
    """Test that FileSystemResolver raises FileNotFoundError for non-existent files."""
    resolver = FileSystemResolver()
    with pytest.raises(FileNotFoundError, match="Could not resolve resource"):
        resolver.resolve("/non/existent/path/mesh.stl")


def test_network_resolver_local_fallback(tmp_path: Path):
    """Test that NetworkResolver falls back to FileSystemResolver for local paths."""
    test_file = tmp_path / "localmesh.obj"
    test_file.write_text("data")

    resolver = NetworkResolver()
    resolved = resolver.resolve(str(test_file))

    assert resolved == test_file.absolute()


def test_network_resolver_unimplemented_uri():
    """Test that NetworkResolver raises NotImplementedError for external URIs."""
    resolver = NetworkResolver()

    with pytest.raises(NotImplementedError, match="not yet implemented"):
        resolver.resolve("https://example.com/robot.stl")

    with pytest.raises(NotImplementedError, match="not yet implemented"):
        resolver.resolve("s3://bucket/mesh.dae")


def test_robot_resource_resolution(tmp_path: Path):
    """Test that the Robot model correctly uses its resolver."""
    test_file = tmp_path / "robot_mesh.stl"
    test_file.write_text("mesh")

    # Default resolver (FileSystemResolver)
    robot = Robot(name="test_bot")
    resolved = robot.resolve_resource(str(test_file))
    assert resolved == test_file.absolute()

    # Using the helper via Mesh model context (this mirrors how platform adapters will use it)
    mesh = Mesh(resource=str(test_file))
    assert robot.resolve_resource(mesh.resource) == test_file.absolute()


def test_robot_custom_resolver():
    """Test that a Robot can be initialized with a custom resolver."""

    class MockResolver:
        def resolve(self, uri: str, relative_to: Path | None = None) -> Path:
            return Path("/mock/resolved") / uri

    robot = Robot(name="mock_bot", resource_resolver=MockResolver())
    resolved = robot.resolve_resource("some_uri")

    assert resolved == Path("/mock/resolved/some_uri")

    # Verify relative_to is passed (conceptually, MockResolver doesn't use it but it shouldn't crash)
    resolved_rel = robot.resolve_resource("some_uri", relative_to=Path("/tmp"))
    assert resolved_rel == Path("/mock/resolved/some_uri")
