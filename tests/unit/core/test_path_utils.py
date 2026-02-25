from linkforge_core.utils.path_utils import resolve_package_path


def test_resolve_package_path_invalid_uri(tmp_path):
    """Test invalid URIs return None."""
    assert resolve_package_path("invalid://uri", tmp_path) is None
    assert resolve_package_path("package://", tmp_path) is None


def test_resolve_package_path_standalone_structure(tmp_path):
    """Test resolving in a standalone folder structure (no ROS env)."""
    # Structure:
    # /tmp/pkg_root/package.xml
    # /tmp/pkg_root/meshes/model.stl
    # /tmp/pkg_root/urdf/robot.urdf  <-- start_dir

    pkg_root = tmp_path / "my_robot_pkg"
    pkg_root.mkdir()
    (pkg_root / "package.xml").touch()

    meshes = pkg_root / "meshes"
    meshes.mkdir()
    model = meshes / "model.stl"
    model.touch()

    urdf_dir = pkg_root / "urdf"
    urdf_dir.mkdir()

    # Case 1: Detect package via package.xml parent
    resolved = resolve_package_path("package://my_robot_pkg/meshes/model.stl", urdf_dir)
    assert resolved is not None
    assert resolved.resolve() == model.resolve()


def test_resolve_package_path_folder_name_match(tmp_path):
    """Test resolving based on folder name matching package name."""
    # Structure:
    # /tmp/my_robot_pkg/meshes/model.stl
    # /tmp/my_robot_pkg/urdf/robot.urdf

    pkg_root = tmp_path / "my_robot_pkg"
    pkg_root.mkdir()

    meshes = pkg_root / "meshes"
    meshes.mkdir()
    model = meshes / "model.stl"
    model.touch()

    urdf_dir = pkg_root / "urdf"
    urdf_dir.mkdir()

    resolved = resolve_package_path("package://my_robot_pkg/meshes/model.stl", urdf_dir)
    assert resolved is not None
    assert resolved.resolve() == model.resolve()


def test_resolve_ros_package_path(tmp_path, monkeypatch):
    """Test resolving via ROS_PACKAGE_PATH environment variable."""
    # Mock a ROS workspace
    ws = tmp_path / "ros_ws"
    ws.mkdir()
    pkg = ws / "ros_pkg"
    pkg.mkdir()
    (pkg / "mesh.stl").touch()

    # Mock environment
    monkeypatch.setenv("ROS_PACKAGE_PATH", str(ws))

    # Test valid lookup
    resolved = resolve_package_path("package://ros_pkg/mesh.stl", tmp_path)
    assert resolved is not None
    assert resolved.resolve() == (pkg / "mesh.stl").resolve()

    # Test invalid path in ROS_PACKAGE_PATH
    monkeypatch.setenv("ROS_PACKAGE_PATH", f"{ws}:/invalid/path")
    resolved = resolve_package_path("package://ros_pkg/mesh.stl", tmp_path)
    assert resolved is not None
    assert resolved.resolve() == (pkg / "mesh.stl").resolve()


def test_resolve_uri_formats(tmp_path):
    """Test variations of package URI prefixes."""
    pkg = tmp_path / "pkg"
    pkg.mkdir()
    (pkg / "file.txt").touch()

    # Simple folder match
    assert (
        resolve_package_path("package://pkg/file.txt", pkg).resolve()
        == (pkg / "file.txt").resolve()
    )
    assert (
        resolve_package_path("package:/pkg/file.txt", pkg).resolve() == (pkg / "file.txt").resolve()
    )
    assert (
        resolve_package_path("package:pkg/file.txt", pkg).resolve() == (pkg / "file.txt").resolve()
    )


def test_resolve_max_depth_exceeded(tmp_path):
    """Test that we don't go up forever."""
    # Deeply nested folder without matching package
    deep = tmp_path / "a/b/c/d/e/f/g/h/i/j/k"
    deep.mkdir(parents=True)

    resolved = resolve_package_path("package://unknown_pkg/mesh.stl", deep)
    assert resolved is None


def test_resolve_package_path_edge_cases_extended(tmp_path, monkeypatch):
    """Test extended edge cases for comprehensive coverage."""

    # 1. Start dir is a file
    # /tmp/pkg/robot.urdf
    pkg = tmp_path / "pkg"
    pkg.mkdir()
    urdf = pkg / "robot.urdf"
    urdf.touch()
    (pkg / "mesh.stl").touch()

    # Should resolve relative to file's parent
    res = resolve_package_path("package://pkg/mesh.stl", urdf)
    assert res is not None
    assert res.resolve() == (pkg / "mesh.stl").resolve()

    # 2. Package.xml fallback (folder name mismatch)
    # /tmp/renamed_pkg/package.xml
    # package name sought: "original_pkg"
    # This relies on the heuristic: if we find ANY package.xml, assume it's the root.
    renamed = tmp_path / "renamed_pkg"
    renamed.mkdir()
    (renamed / "package.xml").touch()
    (renamed / "mesh.stl").touch()

    # Searching for "original_pkg" inside "renamed_pkg"
    # It should traverse up, see "renamed_pkg" (no match), see package.xml (match fallback)
    res = resolve_package_path("package://original_pkg/mesh.stl", renamed / "urdf")
    assert res is not None
    assert res.resolve() == (renamed / "mesh.stl").resolve()

    # 3. ROS_PACKAGE_PATH with empty entry and self-reference
    # ROS_PACKAGE_PATH = "/path/to/ws::/path/to/pkg"
    # The empty entry '::' should be skipped
    # The entry that IS the package root should be caught

    target_pkg = tmp_path / "target_pkg"
    target_pkg.mkdir()
    (target_pkg / "mesh.stl").touch()

    # Set ROS_PACKAGE_PATH with empty segment and direct path to package
    monkeypatch.setenv("ROS_PACKAGE_PATH", f":{str(target_pkg)}:")  # path:

    res = resolve_package_path("package://target_pkg/mesh.stl", tmp_path)
    assert res is not None
    assert res.resolve() == (target_pkg / "mesh.stl").resolve()


def test_resolve_package_path_root_recursion():
    """Test recursion break at root (curr.parent == curr)."""

    # Create a fake path object that acts like root
    class FakePath:
        def __init__(self, name="root"):
            self.name = name

        def resolve(self):
            return self

        @property
        def parent(self):
            return self  # Root behavior: parent is self

        def is_file(self):
            return False

        def __truediv__(self, other):
            return FakePath(f"{self.name}/{other}")

        def exists(self):
            return False

    fake_root = FakePath()
    # Should loop, find nothing, hit root check, and break/return None
    assert resolve_package_path("package://missing/file.stl", fake_root) is None
