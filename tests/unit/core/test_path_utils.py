"""Unit tests for path and resource resolution utilities."""

from __future__ import annotations

from pathlib import Path

from linkforge.core._utils.path_utils import _extract_package_name, resolve_package_path


def test_extract_package_name(tmp_path: Path) -> None:
    """Test extracting package name from XML content."""
    pkg_xml = tmp_path / "package.xml"

    # Standard package.xml
    pkg_xml.write_text("<package><name>test_pkg</name></package>", encoding="utf-8")
    assert _extract_package_name(pkg_xml) == "test_pkg"

    # With whitespace
    pkg_xml.write_text("<package>\n  <name>  spaced_pkg  </name>\n</package>", encoding="utf-8")
    assert _extract_package_name(pkg_xml) == "spaced_pkg"

    # Invalid file
    assert _extract_package_name(tmp_path / "nonexistent.xml") is None


def test_resolve_package_path_case_a(tmp_path: Path) -> None:
    """Test Case A: Folder name matches package name."""
    pkg_dir = tmp_path / "franka_description"
    pkg_dir.mkdir()
    mesh_file = pkg_dir / "meshes" / "link0.stl"
    mesh_file.parent.mkdir()
    mesh_file.write_text("data")

    # Search from a deeper subfolder
    result = resolve_package_path(
        "package://franka_description/meshes/link0.stl", start_dir=pkg_dir / "urdf"
    )
    assert result == mesh_file


def test_resolve_package_path_case_b_success(tmp_path: Path) -> None:
    """Test Case B: Folder renamed, but package.xml name matches."""
    pkg_dir = tmp_path / "franka-main"
    pkg_dir.mkdir()
    (pkg_dir / "package.xml").write_text("<package><name>franka_description</name></package>")
    mesh_file = pkg_dir / "meshes" / "link0.stl"
    mesh_file.parent.mkdir()
    mesh_file.write_text("data")

    result = resolve_package_path(
        "package://franka_description/meshes/link0.stl", start_dir=pkg_dir / "urdf"
    )
    assert result == mesh_file


def test_resolve_package_path_case_b_failure(tmp_path: Path) -> None:
    """Test Case B Failure: Stops false-positive matching of wrong packages."""
    # Structure:
    # /workspace
    #   ├── wrong_pkg (package.xml name="wrong")
    #   └── urdf/ (where we search from)
    ws = tmp_path / "workspace"
    ws.mkdir()
    wrong_pkg = ws / "wrong_pkg"
    wrong_pkg.mkdir()
    (wrong_pkg / "package.xml").write_text("<package><name>wrong</name></package>")

    search_dir = ws / "urdf"
    search_dir.mkdir()

    # Verify that we don't return a path if the package.xml name is different
    # from the requested package name (even if the folder exists).
    result = resolve_package_path("package://right_pkg/mesh.stl", start_dir=search_dir)
    assert result is None


def test_resolve_package_path_uri_variations(tmp_path: Path) -> None:
    # Test different uri scheme variations (package:/ and package:)
    pkg_dir = tmp_path / "franka_description"
    pkg_dir.mkdir()
    mesh_file = pkg_dir / "meshes" / "link0.stl"
    mesh_file.parent.mkdir()
    mesh_file.write_text("data")

    # package:/
    result = resolve_package_path("package:/franka_description/meshes/link0.stl", start_dir=pkg_dir)
    assert result == mesh_file

    # package:
    result = resolve_package_path("package:franka_description/meshes/link0.stl", start_dir=pkg_dir)
    assert result == mesh_file

    # Invalid scheme
    assert (
        resolve_package_path("invalid://franka_description/meshes/link0.stl", start_dir=pkg_dir)
        is None
    )

    # Empty path remainder
    assert resolve_package_path("package://", start_dir=pkg_dir) is None


def test_resolve_package_path_additional_paths(tmp_path: Path) -> None:
    pkg_dir = tmp_path / "franka_description"
    pkg_dir.mkdir()
    mesh_file = pkg_dir / "meshes" / "link0.stl"
    mesh_file.parent.mkdir()
    mesh_file.write_text("data")

    # Match in additional search paths
    result = resolve_package_path(
        "package://franka_description/meshes/link0.stl",
        start_dir=tmp_path,
        additional_search_paths=[tmp_path],
    )
    assert result == mesh_file

    # Match when additional search path IS the package itself
    result = resolve_package_path(
        "package://franka_description/meshes/link0.stl",
        start_dir=tmp_path,
        additional_search_paths=[pkg_dir],
    )
    assert result == mesh_file


def test_resolve_package_path_additional_paths_no_match(tmp_path: Path) -> None:
    pkg_dir = tmp_path / "franka_description"
    pkg_dir.mkdir()
    mesh_file = pkg_dir / "meshes" / "link0.stl"
    mesh_file.parent.mkdir()
    mesh_file.write_text("data")

    # We pass an additional search path that does NOT match (e.g. a different folder),
    # but the package is found via standard start_dir parent traversal.
    non_matching = tmp_path / "other_dir"
    non_matching.mkdir()

    result = resolve_package_path(
        "package://franka_description/meshes/link0.stl",
        start_dir=pkg_dir,
        additional_search_paths=[non_matching],
    )
    assert result == mesh_file


def test_resolve_package_path_ros_env(tmp_path: Path, monkeypatch) -> None:
    pkg_dir = tmp_path / "franka_description"
    pkg_dir.mkdir()
    mesh_file = pkg_dir / "meshes" / "link0.stl"
    mesh_file.parent.mkdir()
    mesh_file.write_text("data")

    # Set ROS_PACKAGE_PATH env
    monkeypatch.setenv("ROS_PACKAGE_PATH", f" :{tmp_path}")

    # Resolve from ROS path
    result = resolve_package_path(
        "package://franka_description/meshes/link0.stl",
        start_dir=tmp_path,
    )
    assert result == mesh_file

    # Set ROS_PACKAGE_PATH to be the package itself
    monkeypatch.setenv("ROS_PACKAGE_PATH", str(pkg_dir))
    result = resolve_package_path(
        "package://franka_description/meshes/link0.stl",
        start_dir=tmp_path,
    )
    assert result == mesh_file


def test_resolve_package_path_ros_env_no_match(tmp_path: Path, monkeypatch) -> None:
    pkg_dir = tmp_path / "franka_description"
    pkg_dir.mkdir()
    mesh_file = pkg_dir / "meshes" / "link0.stl"
    mesh_file.parent.mkdir()
    mesh_file.write_text("data")

    # ROS_PACKAGE_PATH has a non-matching directory, then standard lookup continues to start_dir
    non_matching = tmp_path / "other_dir"
    non_matching.mkdir()

    monkeypatch.setenv("ROS_PACKAGE_PATH", str(non_matching))

    result = resolve_package_path(
        "package://franka_description/meshes/link0.stl",
        start_dir=pkg_dir,
    )
    assert result == mesh_file


def test_resolve_package_path_start_dir_is_file_and_root_break(tmp_path: Path) -> None:
    pkg_dir = tmp_path / "franka_description"
    pkg_dir.mkdir()
    dummy_file = pkg_dir / "dummy.urdf"
    dummy_file.write_text("")

    # If start_dir is a file, it resolves to parent
    result = resolve_package_path(
        "package://franka_description/dummy.urdf",
        start_dir=dummy_file,
    )
    assert result == dummy_file

    # Root break test (searches all the way up to root)
    result = resolve_package_path(
        "package://nonexistent_package/dummy.urdf",
        start_dir=Path("/"),
    )
    assert result is None


def test_extract_package_name_exception(tmp_path: Path) -> None:
    # Passing a directory instead of a file triggers exception in open()
    assert _extract_package_name(tmp_path) is None


def test_normalize_uri_to_path() -> None:
    from linkforge.core._utils.path_utils import normalize_uri_to_path

    # Windows-style file:// URI
    win_uri = "file:///C:/mesh.stl"
    assert normalize_uri_to_path(win_uri) == Path("C:/mesh.stl")

    # Posix-style file:// URI
    posix_uri = "file:///path/mesh.stl"
    assert normalize_uri_to_path(posix_uri) == Path("/path/mesh.stl")

    # Plain path string
    plain = "/path/mesh.stl"
    assert normalize_uri_to_path(plain) == Path("/path/mesh.stl")


def test_get_export_path(tmp_path: Path) -> None:
    from linkforge.core._utils.path_utils import get_export_path

    # Preserve package://
    assert (
        get_export_path("package://franka_description/mesh.stl")
        == "package://franka_description/mesh.stl"
    )
    assert (
        get_export_path("package:/franka_description/mesh.stl")
        == "package:/franka_description/mesh.stl"
    )

    # file:// URI making relative
    base_dir = tmp_path / "workspace"
    base_dir.mkdir()
    target_file = base_dir / "mesh.stl"

    file_uri = f"file://{target_file.absolute()}"
    assert get_export_path(file_uri, relative_to=base_dir) == "mesh.stl"

    # file:// URI NOT relative (outside)
    outside_dir = tmp_path / "outside"
    outside_dir.mkdir()
    outside_file = outside_dir / "mesh.stl"
    outside_uri = f"file://{outside_file.absolute()}"
    assert get_export_path(outside_uri, relative_to=base_dir) == outside_uri

    # file:// URI without relative_to
    assert get_export_path(file_uri) == file_uri

    # Standard path making relative
    assert get_export_path(str(target_file.absolute()), relative_to=base_dir) == "mesh.stl"

    # Standard path NOT relative (outside)
    assert get_export_path(str(outside_file.absolute()), relative_to=base_dir) == str(
        outside_file.absolute()
    )

    # Standard path without relative_to
    assert get_export_path(str(target_file.absolute())) == str(target_file.absolute())
