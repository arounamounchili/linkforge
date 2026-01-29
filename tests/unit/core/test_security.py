"""Tests for security validation functions."""

from pathlib import Path

import pytest
from linkforge_core.validation.security import validate_mesh_path


class TestValidateMeshPath:
    """Tests for validate_mesh_path function."""

    def test_valid_relative_path(self, tmp_path):
        """Test valid relative path inside URDF directory."""
        urdf_dir = tmp_path / "robot"
        urdf_dir.mkdir()
        mesh_path = Path("meshes/test.stl")

        resolved = validate_mesh_path(mesh_path, urdf_dir)
        expected = (urdf_dir / mesh_path).resolve()
        assert resolved == expected

    def test_allow_parent_traversal(self, tmp_path):
        """Test allowing '..' to go to sibling directory."""
        # Structure:
        # /root/urdf/robot.urdf
        # /root/meshes/test.stl
        root = tmp_path
        urdf_dir = root / "urdf"
        urdf_dir.mkdir()
        meshes_dir = root / "meshes"
        meshes_dir.mkdir()

        mesh_path = Path("../meshes/test.stl")

        resolved = validate_mesh_path(mesh_path, urdf_dir)
        expected = (meshes_dir / "test.stl").resolve()
        assert resolved == expected

    def test_block_system_paths(self, tmp_path):
        """Test that paths resolving to system directories are blocked."""
        urdf_dir = tmp_path

        # Try to access /etc/passwd using valid relative path syntax
        # We need enough .. to get to root.
        # On mac/linux, 10 levels should be enough
        mesh_path = Path("../../../../../../../../../etc/passwd")

        with pytest.raises(ValueError, match="restricted system location"):
            validate_mesh_path(mesh_path, urdf_dir)

    def test_absolute_path_default_blocked(self, tmp_path):
        """Test that absolute paths are blocked by default."""
        urdf_dir = tmp_path
        mesh_path = Path("/tmp/test.stl")

        with pytest.raises(ValueError, match="Absolute path.*not allowed"):
            validate_mesh_path(mesh_path, urdf_dir)

    def test_absolute_path_allowed(self, tmp_path):
        """Test allowing absolute paths when requested."""
        urdf_dir = tmp_path
        # Use a safe absolute path (temp dir)
        mesh_path = (tmp_path / "safe.stl").resolve()

        resolved = validate_mesh_path(mesh_path, urdf_dir, allow_absolute=True)
        assert resolved == mesh_path

    def test_url_encoded_path(self, tmp_path):
        """Test that URL-encoded paths are decoded."""
        from linkforge_core.validation.security import validate_mesh_path

        urdf_dir = tmp_path / "robot"
        urdf_dir.mkdir()

        # URL-encoded path (space as %20)
        encoded_path = "meshes/my%20file.stl"
        result = validate_mesh_path(encoded_path, urdf_dir)

        # Should decode to normal path
        assert "my file.stl" in str(result)
