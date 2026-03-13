import os
import tempfile
from pathlib import Path
from unittest.mock import patch

import pytest
from linkforge_core.base import FileSystemResolver
from linkforge_core.utils.path_utils import resolve_package_path


@pytest.fixture
def mock_workspace():
    """Create a mock workspace for testing hybrid resolution."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_path = Path(tmp_dir)

        # workspace/
        #   robot_pkg/
        #     package.xml
        #     urdf/
        #       robot.urdf
        #     meshes/
        #       base.stl

        pkg_dir = tmp_path / "robot_pkg"
        pkg_dir.mkdir()
        (pkg_dir / "package.xml").touch()

        urdf_dir = pkg_dir / "urdf"
        urdf_dir.mkdir()

        mesh_dir = pkg_dir / "meshes"
        mesh_dir.mkdir()
        (mesh_dir / "base.stl").touch()

        yield urdf_dir, pkg_dir


def test_resolve_package_path_upward(mock_workspace) -> None:
    urdf_dir, pkg_dir = mock_workspace
    uri = "package://robot_pkg/meshes/base.stl"

    # Test upward search
    resolved = resolve_package_path(uri, urdf_dir)
    assert resolved is not None
    assert resolved.name == "base.stl"
    assert "robot_pkg" in resolved.parts
    assert resolved.exists()


def test_resolve_package_path_ros_env(mock_workspace) -> None:
    urdf_dir, pkg_dir = mock_workspace
    uri = "package://robot_pkg/meshes/base.stl"

    # Mock ROS_PACKAGE_PATH
    with patch.dict(os.environ, {"ROS_PACKAGE_PATH": str(pkg_dir.parent)}):
        resolved = resolve_package_path(uri, Path("/tmp"))  # Start from unrelated dir
        assert resolved is not None
        assert resolved.exists()
        assert str(resolved).startswith(str(pkg_dir))


def test_resolve_mesh_path_package_uri(mock_workspace) -> None:
    urdf_dir, pkg_dir = mock_workspace
    uri_path = Path("package://robot_pkg/meshes/base.stl")

    resolved = FileSystemResolver().resolve(str(uri_path), relative_to=urdf_dir)
    assert resolved.exists()
    assert resolved.is_absolute()


def test_resolve_mesh_path_relative(mock_workspace) -> None:
    urdf_dir, pkg_dir = mock_workspace
    rel_path = "../meshes/base.stl"

    resolved = FileSystemResolver().resolve(rel_path, relative_to=urdf_dir)
    assert resolved.exists()
    # Resolve to remove .. for comparison
    assert resolved.resolve() == (pkg_dir / "meshes" / "base.stl").resolve()
