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


def test_filesystem_resolver_errors_and_fallbacks(tmp_path):
    """Test resolution successes, failures, and relative path fallbacks."""
    resolver = FileSystemResolver()

    # Create dummy files for success cases
    pkg_file = tmp_path / "my_pkg" / "test.urdf"
    pkg_file.parent.mkdir()
    pkg_file.write_text("<robot/>")

    abs_file = tmp_path / "abs_test.urdf"
    abs_file.write_text("<robot/>")

    # package:// success
    import os

    old_rpp = os.environ.get("ROS_PACKAGE_PATH")
    os.environ["ROS_PACKAGE_PATH"] = str(tmp_path)
    try:
        assert resolver.resolve("package://my_pkg/test.urdf") == pkg_file.absolute()

        # package:// failure
        with pytest.raises(FileNotFoundError, match="Could not resolve package resource"):
            resolver.resolve("package://non_existent_pkg/file.urdf")
    finally:
        if old_rpp:
            os.environ["ROS_PACKAGE_PATH"] = old_rpp
        else:
            del os.environ["ROS_PACKAGE_PATH"]

    # file:// success
    assert resolver.resolve(f"file://{abs_file.absolute()}") == abs_file.absolute()

    # file:// failure
    with pytest.raises(FileNotFoundError, match="Could not resolve file URI"):
        resolver.resolve("file:///non_existent_absolute_path/file.urdf")

    # Relative path resolution with relative_to
    rel_dir = tmp_path / "subdir"
    rel_dir.mkdir()
    target_file = rel_dir / "test.txt"
    target_file.write_text("hello")

    assert resolver.resolve("test.txt", relative_to=rel_dir) == target_file.absolute()

    # Fallback to current directory
    old_cwd = os.getcwd()
    os.chdir(str(tmp_path))
    try:
        (tmp_path / "local_file_for_fallback.txt").write_text("local")
        assert (
            resolver.resolve("local_file_for_fallback.txt")
            == (tmp_path / "local_file_for_fallback.txt").absolute()
        )

        # Total failure case
        with pytest.raises(FileNotFoundError, match="Could not resolve resource"):
            resolver.resolve("completely_missing.urdf")
    finally:
        os.chdir(old_cwd)


def test_resolver_current_directory_fallback(tmp_path):
    """Verify that the file resolver correctly falls back to CWD if relative resolution fails."""
    resolver = FileSystemResolver()
    test_file = Path("cwd_test_file_robust.txt")
    test_file.write_text("cwd content")

    try:
        # Resolve from a different subdirectory, which should fall back to CWD
        other_dir = tmp_path / "temporary_resolution_dir"
        other_dir.mkdir()
        res = resolver.resolve("cwd_test_file_robust.txt", relative_to=other_dir)
        assert res.name == "cwd_test_file_robust.txt"
        assert res.is_absolute()
    finally:
        if test_file.exists():
            test_file.unlink()
