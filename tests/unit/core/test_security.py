"""Tests for security validation functions."""

from pathlib import Path

import pytest
from linkforge_core.validation.security import find_sandbox_root, validate_mesh_path


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

    def test_allow_parent_traversal_blocked(self, tmp_path):
        """Test blocking '..' traversal that escapes URDF directory."""
        # Structure:
        # /root/urdf/robot.urdf
        # /root/meshes/test.stl
        root = tmp_path
        urdf_dir = root / "urdf"
        urdf_dir.mkdir()
        meshes_dir = root / "meshes"
        meshes_dir.mkdir()

        mesh_path = Path("../meshes/test.stl")

        with pytest.raises(ValueError, match="attempts to escape the sandbox root"):
            validate_mesh_path(mesh_path, urdf_dir)

    def test_allow_parent_traversal_with_sandbox_root(self, tmp_path):
        """Test allowing '..' traversal when it stays within sandbox_root."""
        # /root/urdf/ (urdf_dir)
        # /root/meshes/ (target)
        root = tmp_path.resolve()
        urdf_dir = root / "urdf"
        urdf_dir.mkdir()
        meshes_dir = root / "meshes"
        meshes_dir.mkdir()

        mesh_path = Path("../meshes/test.stl")

        # PASS: providing the top-level root as sandbox
        resolved = validate_mesh_path(mesh_path, urdf_dir, sandbox_root=root)
        assert resolved == (meshes_dir / "test.stl").resolve()

    def test_escape_sandbox_root_blocked(self, tmp_path):
        """Test blocking traversal even with sandbox_root if it escapes the root."""
        root = tmp_path.resolve()
        package = root / "package"
        package.mkdir()
        urdf_dir = package / "urdf"
        urdf_dir.mkdir()

        # Attempt to escape 'package' folder into 'root'
        mesh_path = Path("../../outside.stl")

        with pytest.raises(ValueError, match="attempts to escape the sandbox root"):
            validate_mesh_path(mesh_path, urdf_dir, sandbox_root=package)

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


def test_find_sandbox_root(tmp_path):
    """Test the sandbox root detection logic."""
    # 1. Standard URDF folder structure
    package_root = tmp_path / "my_robot"
    urdf_dir = package_root / "urdf"
    urdf_dir.mkdir(parents=True)
    robot_file = urdf_dir / "robot.urdf"

    assert find_sandbox_root(robot_file) == package_root

    # 1.5. XACRO folder parent detection
    # If the parent folder is literally named 'xacro', it should go up one level
    xacro_dir = package_root / "xacro"
    xacro_dir.mkdir()
    xacro_file = xacro_dir / "robot.xacro"
    assert find_sandbox_root(xacro_file) == package_root

    # 2. Package.xml detection
    other_root = tmp_path / "other_pkg"
    sub_dir = other_root / "subdir" / "deep"
    sub_dir.mkdir(parents=True)
    (other_root / "package.xml").touch()
    robot_file_2 = sub_dir / "robot.urdf"

    # It should find the package.xml root from the grandparent
    assert find_sandbox_root(robot_file_2) == other_root

    # 3. Test loop termination (root reached)
    # If we are at the root, it should just return the parent
    root_file = tmp_path / "root.urdf"
    assert find_sandbox_root(root_file) == tmp_path

    # 4. Fallback to parent
    random_dir = tmp_path / "random"
    random_dir.mkdir()
    random_file = random_dir / "test.urdf"
    assert find_sandbox_root(random_file) == random_dir


def test_security_sandbox_root_loop(tmp_path):
    """Test find_sandbox_root loop termination (depth limit)."""
    # Create a structure deep enough to hit the 5-level limit
    # root/a/b/c/d/e/f/file.urdf
    deep_path = tmp_path / "a/b/c/d/e/f"
    deep_path.mkdir(parents=True)
    urdf = deep_path / "file.urdf"
    urdf.touch()

    # This should traverse up 5 times and stop, returning the current parent if no package.xml found
    # Current parent of file.urdf is 'f'
    # It checks f/package.xml, e/package.xml, d/package.xml, c/package.xml, b/package.xml (5 times)
    # Returns 'f' (deep_path) as fallback
    root = find_sandbox_root(urdf)
    assert root == deep_path


def test_find_sandbox_root_recursion_break():
    """Test recursion break at root (check_path.parent == check_path)."""

    class FakePath:
        def __init__(self, name="root"):
            self.name = name

        @property
        def parent(self):
            return self  # Root behavior

        def __truediv__(self, other):
            return FakePath(f"{self.name}/{other}")

        def exists(self):
            return False  # package.xml never exists

    # Actually, find_sandbox_root takes filepath.
    # It does `current = filepath.parent`.
    # So simulate filepath
    class FakeFile:
        def __init__(self, parent_obj):
            self.p = parent_obj

        @property
        def parent(self):
            return self.p

        def resolve(self):
            return self

    fake_root = FakePath()
    fake_filepath = FakeFile(fake_root)

    # Pass fake_filepath to find_sandbox_root
    # It sets current = fake_filepath.parent (fake_root)
    # Loop 1: check package.xml (False), check parent==self (True) -> break.
    # Returns current (fake_root).

    # We need to type ignore or ensure it quacks like Path
    res = find_sandbox_root(fake_filepath)
    assert res == fake_root
