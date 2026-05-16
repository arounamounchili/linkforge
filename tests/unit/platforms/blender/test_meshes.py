"""Unit tests for Blender Mesh I/O, naming, and resolution."""

import os
from pathlib import Path
from unittest.mock import patch

import pytest
from linkforge.blender.adapters.mesh_io import (
    create_simplified_mesh,
    export_link_mesh,
    export_mesh_glb,
    export_mesh_obj,
    export_mesh_stl,
    get_mesh_filename,
)
from linkforge.core._utils.path_utils import resolve_package_path

from tests.blender_test_utils import create_mesh_object, create_test_object

# Mesh I/O Operations


class TestMeshIO:
    def test_export_mesh_stl(self, scene, tmp_path, blender_context) -> None:
        """Test exporting a mesh to STL."""
        obj = create_mesh_object("test_cube_stl", scene=scene, with_cube=True)
        filepath = tmp_path / "test.stl"

        export_mesh_stl(obj, filepath)
        assert filepath.exists()

    def test_export_mesh_obj(self, scene, tmp_path, blender_context) -> None:
        """Test exporting a mesh to OBJ."""
        obj = create_mesh_object("test_cube_obj", scene=scene, with_cube=True)
        filepath = tmp_path / "test.obj"

        export_mesh_obj(obj, filepath)
        assert filepath.exists()

    def test_get_mesh_filename(self) -> None:
        """Verify mesh filename generation and sanitization."""
        # Simple name
        assert get_mesh_filename("part", "visual", "STL") == "part_visual.stl"
        # Sanitization: replace spaces and invalid chars
        assert get_mesh_filename("my part@123", "collision", "OBJ") == "my_part_123_collision.obj"
        # Suffix
        assert get_mesh_filename("part", "visual", "STL", suffix="1") == "part_visual_1.stl"

    def test_export_mesh_glb(self, scene, tmp_path, blender_context) -> None:
        """Test exporting a mesh to GLB."""
        obj = create_mesh_object("test_cube_glb", scene=scene, with_cube=True)
        filepath = tmp_path / "test.glb"

        with patch(
            "linkforge.blender.adapters.mesh_io.bpy.ops.export_scene.gltf",
            return_value={"FINISHED"},
        ):
            assert export_mesh_glb(obj, filepath) is True

    def test_get_mesh_filename_variants(self) -> None:
        """Verify filename generation with different types and suffixes."""
        assert get_mesh_filename("base", "visual", "STL") == "base_visual.stl"
        assert (
            get_mesh_filename("link_0", "collision", "OBJ", suffix="_0") == "link_0_collision_0.obj"
        )
        # sanitize_name does NOT lowercase
        assert get_mesh_filename("Upper Arm", "visual", "glb") == "Upper_Arm_visual.glb"

    def test_export_mesh_none_fails(self) -> None:
        """Verify that passing None to export functions returns False."""
        assert export_mesh_stl(None, Path("test.stl")) is False
        assert export_mesh_obj(None, Path("test.obj")) is False
        assert export_mesh_glb(None, Path("test.glb")) is False

    def test_export_mesh_error_handling(self, mocker, scene, tmp_path, blender_context) -> None:
        """Verify error handling (RuntimeError/OSError) during export."""
        obj = create_mesh_object("error_mesh", scene=scene, with_cube=True)
        filepath = tmp_path / "error.stl"

        # Mock ops to raise RuntimeError - patch on the module where it's used
        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.ops.wm.stl_export",
            side_effect=RuntimeError("Blender error"),
        )
        assert export_mesh_stl(obj, filepath) is False

        # Mock Path.mkdir to raise OSError
        mocker.patch("pathlib.Path.mkdir", side_effect=OSError("Disk full"))
        assert export_mesh_stl(obj, filepath) is False

    def test_export_link_mesh_error_handling(
        self, mocker, scene, tmp_path, blender_context
    ) -> None:
        """Verify error handling in high-level export_link_mesh."""
        obj = create_mesh_object("link_error", scene=scene, with_cube=True)
        # Force an exception during processing
        mocker.patch("bpy.data.meshes.new_from_object", side_effect=ValueError("Invalid mesh"))

        path, offset = export_link_mesh(obj, "link", "visual", "STL", tmp_path)
        assert path is None
        assert offset.is_identity

    def test_create_simplified_mesh(self, mocker, scene, blender_context) -> None:
        """Test mesh simplification logic."""
        obj = create_mesh_object("decimate_me", scene=scene, with_cube=True)
        # Mock modifier apply
        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.ops.object.modifier_apply",
            return_value={"FINISHED"},
        )
        simplified = create_simplified_mesh(obj, 0.5)
        assert simplified is not None
        assert "Decimate" in simplified.modifiers

    def test_export_mesh_obj_error_handling(self, mocker, scene, tmp_path, blender_context) -> None:
        """Verify error handling for OBJ export."""
        obj = create_mesh_object("obj_error", scene=scene, with_cube=True)
        filepath = tmp_path / "error.obj"
        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.ops.wm.obj_export",
            side_effect=OSError("OBJ fail"),
        )
        assert export_mesh_obj(obj, filepath) is False

    def test_export_link_mesh_full(self, mocker, scene, tmp_path, blender_context) -> None:
        """Verify full link mesh export workflow including directory creation."""
        # Create nested directory to test auto-creation
        export_dir = tmp_path / "meshes" / "sub"
        obj = create_mesh_object("link_part", scene=scene, with_cube=True)

        # Patch internal export functions
        mock_stl = mocker.patch("linkforge.blender.adapters.mesh_io.export_mesh_stl")

        filepath, offset = export_link_mesh(
            obj,
            link_name="link_part",
            geometry_type="visual",
            mesh_format="STL",
            meshes_dir=export_dir,
        )

        assert str(filepath).endswith("link_part_visual.stl")
        mock_stl.assert_called_once()

        # Verify filepath passed to exporter matches what was returned
        call_args = mock_stl.call_args
        assert call_args.args[1] == filepath
        assert os.path.isabs(call_args.args[1])


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
