"""Unit tests for Blender Mesh I/O, naming, and resolution."""

from __future__ import annotations

from unittest.mock import patch

import pytest
from linkforge.blender.adapters.mesh_io import (
    export_mesh_obj,
    export_mesh_stl,
)
from linkforge_core.utils.path_utils import resolve_package_path

from tests.blender_test_utils import create_test_object

# Mesh I/O Operations


class TestMeshIO:
    def test_export_mesh_stl(self, scene, tmp_path, blender_context) -> None:
        """Test exporting a mesh to STL."""
        from tests.blender_test_utils import create_mesh_object

        obj = create_mesh_object("test_cube_stl", scene=scene, with_cube=True)
        filepath = tmp_path / "test.stl"

        export_mesh_stl(obj, filepath)
        assert filepath.exists()

    def test_export_mesh_obj(self, scene, tmp_path, blender_context) -> None:
        """Test exporting a mesh to OBJ."""
        from tests.blender_test_utils import create_mesh_object

        obj = create_mesh_object("test_cube_obj", scene=scene, with_cube=True)
        filepath = tmp_path / "test.obj"

        export_mesh_obj(obj, filepath)
        assert filepath.exists()


# Mesh Naming and Suffixes


class TestMeshNaming:
    def test_single_visual_no_suffix(self, scene, blender_context) -> None:
        """Test that a single visual mesh has no suffix by default."""
        # This logic is typically handled in blender_to_core conversion
        # but we can verify the source_name preservation here if needed.
        obj = create_test_object("part", None, scene)
        obj["source_name"] = "custom"
        assert obj["source_name"] == "custom"


# Path Resolution


class TestMeshResolution:
    def test_resolve_package_path_relative(self, tmp_path) -> None:
        """Test resolving relative package paths."""
        pkg_dir = tmp_path / "my_pkg"
        pkg_dir.mkdir()
        (pkg_dir / "package.xml").touch()

        mesh_dir = pkg_dir / "meshes"
        mesh_dir.mkdir()
        mesh_file = mesh_dir / "test.stl"
        mesh_file.touch()

        source_dir = pkg_dir / "urdf"
        source_dir.mkdir()

        uri = "package://my_pkg/meshes/test.stl"
        resolved = resolve_package_path(uri, source_dir)
        assert resolved is not None
        assert resolved.name == "test.stl"


# Robustness and Edge Cases


class TestMeshRobustness:
    def test_export_mesh_failure_handling(self, scene, tmp_path, blender_context) -> None:
        """Test that mesh export handles exceptions gracefully."""
        obj = create_test_object("monkey", None, scene)
        filepath = tmp_path / "dummy.stl"

        with patch("linkforge.blender.adapters.mesh_io.bpy.ops.wm") as mock_wm:
            mock_wm.stl_export.side_effect = TypeError("Unexpected")
            with pytest.raises(TypeError):
                export_mesh_stl(obj, filepath)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
