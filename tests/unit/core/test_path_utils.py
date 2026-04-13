"""Unit tests for path and resource resolution utilities."""

from __future__ import annotations

from pathlib import Path

from linkforge_core.utils.path_utils import _extract_package_name, resolve_package_path


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
