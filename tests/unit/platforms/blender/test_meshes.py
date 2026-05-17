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


class TestMeshExhaustiveCoverage:
    def test_export_mesh_view_layer_none(self, mocker, scene, tmp_path) -> None:
        """Verify export and simplification functions handle view_layer being None gracefully."""
        import bpy

        obj = create_mesh_object("test_cube_view_none", scene=scene, with_cube=True)
        filepath = tmp_path / "test.stl"

        # Patch view_layer to be None in the mesh_io module
        mocker.patch("linkforge.blender.adapters.mesh_io.bpy.context.view_layer", None)
        mocker.patch("linkforge.blender.adapters.mesh_io.bpy.ops.wm.stl_export")
        mocker.patch("linkforge.blender.adapters.mesh_io.bpy.ops.wm.obj_export")
        mocker.patch("linkforge.blender.adapters.mesh_io.bpy.ops.export_scene.gltf")
        mocker.patch("linkforge.blender.adapters.mesh_io.bpy.ops.object.modifier_apply")

        # 1. STL export
        assert export_mesh_stl(obj, filepath) is True
        # 2. OBJ export
        assert export_mesh_obj(obj, filepath.with_suffix(".obj")) is True
        # 3. GLB export
        assert export_mesh_glb(obj, filepath.with_suffix(".glb")) is True
        # 4. Simplification
        simplified = create_simplified_mesh(obj, 0.5)
        assert simplified is not None
        # Clean up the simplified mesh from blender database
        if simplified:
            bpy.data.objects.remove(simplified, do_unlink=True)

    def test_export_mesh_glb_error_handling(self, mocker, scene, tmp_path) -> None:
        """Verify GLB export error handling for RuntimeError/OSError and TypeError."""
        obj = create_mesh_object("glb_error_mesh", scene=scene, with_cube=True)
        filepath = tmp_path / "test.glb"

        # Test RuntimeError/OSError fallback
        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.ops.export_scene.gltf",
            side_effect=RuntimeError("GLB fail"),
        )
        assert export_mesh_glb(obj, filepath) is False

        # Test unexpected TypeError/AttributeError
        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.ops.export_scene.gltf",
            side_effect=TypeError("Unexpected GLB error"),
        )
        with pytest.raises(TypeError, match="Unexpected GLB error"):
            export_mesh_glb(obj, filepath)

    def test_export_mesh_stl_unexpected_critical_exception(self, mocker, scene, tmp_path) -> None:
        """Verify STL export critical unexpected exceptions are propagated."""
        obj = create_mesh_object("stl_critical_mesh", scene=scene, with_cube=True)
        filepath = tmp_path / "test.stl"

        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.ops.wm.stl_export",
            side_effect=Exception("Critical error"),
        )
        with pytest.raises(Exception, match="Critical error"):
            export_mesh_stl(obj, filepath)

    def test_export_mesh_obj_unexpected_critical_exception(self, mocker, scene, tmp_path) -> None:
        """Verify OBJ export critical unexpected exceptions are propagated."""
        obj = create_mesh_object("obj_critical_mesh", scene=scene, with_cube=True)
        filepath = tmp_path / "test.obj"

        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.ops.wm.obj_export",
            side_effect=Exception("Critical error"),
        )
        with pytest.raises(Exception, match="Critical error"):
            export_mesh_obj(obj, filepath)

    def test_create_simplified_mesh_non_mesh_fails(self, scene) -> None:
        """Verify create_simplified_mesh returns None for non-MESH objects or None input."""
        assert create_simplified_mesh(None, 0.5) is None
        empty_obj = create_test_object("empty_obj", None, scene)
        assert create_simplified_mesh(empty_obj, 0.5) is None

    def test_export_link_mesh_non_mesh_fails(self, tmp_path) -> None:
        """Verify export_link_mesh returns None and Identity matrix for non-MESH objects or None input."""
        path, offset = export_link_mesh(None, "link", "visual", "STL", tmp_path)
        assert path is None
        assert offset.is_identity

    def test_export_link_mesh_depsgraph_provided(self, mocker, scene, tmp_path) -> None:
        """Verify export_link_mesh uses the provided depsgraph if passed."""
        import bpy

        obj = create_mesh_object("depsgraph_mesh", scene=scene, with_cube=True)
        depsgraph = bpy.context.evaluated_depsgraph_get()

        mocker.patch("linkforge.blender.adapters.mesh_io.bpy.ops.wm.stl_export")
        path, offset = export_link_mesh(
            obj, "depsgraph_link", "visual", "STL", tmp_path, depsgraph=depsgraph
        )
        assert path is not None

    def test_export_link_mesh_no_translation_needed(self, mocker, scene, tmp_path) -> None:
        """Verify export_link_mesh handles meshes already centered at origin (local_center <= EPSILON)."""
        obj = create_mesh_object("centered_mesh", scene=scene, with_cube=True)
        # Center vertices at (0,0,0) so bound_box is perfectly symmetric
        for vert in obj.data.vertices:
            vert.co = (0.0, 0.0, 0.0)

        mocker.patch("linkforge.blender.adapters.mesh_io.bpy.ops.wm.stl_export")
        path, offset = export_link_mesh(obj, "centered_link", "visual", "STL", tmp_path)
        assert path is not None

    def test_export_link_mesh_formats_and_fallbacks(self, mocker, scene, tmp_path) -> None:
        """Verify export_link_mesh handles various formats (OBJ, GLB, and unknown fallbacks)."""
        obj = create_mesh_object("formats_mesh", scene=scene, with_cube=True)

        mock_obj = mocker.patch("linkforge.blender.adapters.mesh_io.bpy.ops.wm.obj_export")
        mock_glb = mocker.patch("linkforge.blender.adapters.mesh_io.bpy.ops.export_scene.gltf")

        # Test OBJ format
        path_obj, _ = export_link_mesh(obj, "link_obj", "visual", "OBJ", tmp_path)
        assert path_obj is not None
        assert path_obj.suffix == ".obj"
        mock_obj.assert_called_once()

        # Test GLB format
        path_glb, _ = export_link_mesh(obj, "link_glb", "visual", "GLB", tmp_path)
        assert path_glb is not None
        assert path_glb.suffix == ".glb"
        mock_glb.assert_called_once()

        # Test Unknown format fallback
        path_fallback, _ = export_link_mesh(obj, "link_fallback", "visual", "PLY", tmp_path)
        assert path_fallback is not None
        assert path_fallback.suffix == ".obj"

    def test_export_link_mesh_export_failure(self, scene, tmp_path) -> None:
        """Verify export_link_mesh returns None when the underlying exporter fails."""
        obj = create_mesh_object("fail_mesh", scene=scene, with_cube=True)

        with patch("linkforge.blender.adapters.mesh_io.export_mesh_stl", return_value=False):
            path, offset = export_link_mesh(obj, "fail_link", "visual", "STL", tmp_path)
            assert path is None
            assert offset.is_identity

    def test_export_link_mesh_cleanup_on_exception(self, mocker, scene, tmp_path) -> None:
        """Verify export_link_mesh cleans up temporary meshes and objects when an exception is raised."""
        obj = create_mesh_object("cleanup_mesh", scene=scene, with_cube=True)

        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.data.meshes.new_from_object",
            side_effect=RuntimeError("Simulated failure"),
        )
        path, offset = export_link_mesh(obj, "cleanup_link", "visual", "STL", tmp_path)
        assert path is None
        assert offset.is_identity

    def test_export_mesh_obj_type_error_handling(self, mocker, scene, tmp_path) -> None:
        """Verify OBJ export handles TypeError and raises it."""
        obj = create_mesh_object("obj_type_error_mesh", scene=scene, with_cube=True)
        filepath = tmp_path / "test.obj"

        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.ops.wm.obj_export",
            side_effect=TypeError("OBJ type error"),
        )
        with pytest.raises(TypeError, match="OBJ type error"):
            export_mesh_obj(obj, filepath)

    def test_export_link_mesh_dry_run(self, scene, tmp_path) -> None:
        """Verify export_link_mesh returns expected filepath and matrix immediately in dry_run mode."""
        obj = create_mesh_object("dry_run_mesh", scene=scene, with_cube=True)
        filepath, matrix = export_link_mesh(
            obj, "dry_link", "visual", "STL", tmp_path, dry_run=True
        )
        assert filepath is not None
        assert filepath.name == "dry_link_visual.stl"
        assert matrix == obj.matrix_world

    def test_export_link_mesh_with_simplification_and_centering(
        self, mocker, scene, tmp_path
    ) -> None:
        """Verify export_link_mesh performs centering (local_center > EPSILON) and simplification."""
        from mathutils import Vector

        obj = create_mesh_object("complex_mesh", scene=scene, with_cube=True)
        # Shift the bound_box center to (2.0, 2.0, 2.0)
        obj.bound_box = [Vector((2.0, 2.0, 2.0))] * 8

        mocker.patch("linkforge.blender.adapters.mesh_io.bpy.ops.wm.stl_export", return_value=True)
        # Mock modifier apply for simplify
        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.ops.object.modifier_apply",
            return_value={"FINISHED"},
        )

        filepath, offset = export_link_mesh(
            obj, "complex_link", "collision", "STL", tmp_path, simplify=True, decimation_ratio=0.5
        )
        assert filepath is not None
        # Bounding box center should be shifted by 2 on the X axis, so offset should reflect that
        assert abs(offset.translation.x - 2.0) < 1e-5

    def test_export_link_mesh_finally_cleanup_different_data(self, mocker, scene, tmp_path) -> None:
        """Verify the finally block clean up path when final_mesh_data is different from temp_export_obj.data."""
        import bpy

        obj = create_mesh_object("diff_data_mesh", scene=scene, with_cube=True)

        # We want final_mesh_data.transform to throw an exception so that final_mesh_data != temp_export_obj.data
        # when the finally block is executed.
        original_new_from_object = bpy.data.meshes.new_from_object

        def mock_new_from_object(*args, **kwargs):
            mesh_data = original_new_from_object(*args, **kwargs)

            # Inject directly into __dict__ to bypass the MockPropertyGroup attributes intercept
            def raise_err(*args, **kwargs):
                raise RuntimeError("Simulated transform error")

            mesh_data.__dict__["transform"] = raise_err
            return mesh_data

        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.data.meshes.new_from_object",
            side_effect=mock_new_from_object,
        )

        path, offset = export_link_mesh(obj, "diff_data_link", "visual", "STL", tmp_path)
        assert path is None
        assert offset.is_identity

    def test_export_mesh_glb_generic_exception(self, mocker, scene, tmp_path) -> None:
        """Verify GLB export generic unexpected Exception is caught and propagated (covers line 253-255)."""
        obj = create_mesh_object("glb_generic_err_mesh", scene=scene, with_cube=True)
        filepath = tmp_path / "test.glb"

        mocker.patch(
            "linkforge.blender.adapters.mesh_io.bpy.ops.export_scene.gltf",
            side_effect=Exception("Generic GLB error"),
        )
        with pytest.raises(Exception, match="Generic GLB error"):
            export_mesh_glb(obj, filepath)

    def test_export_link_mesh_simplification_returns_none(self, mocker, scene, tmp_path) -> None:
        """Verify export_link_mesh handles create_simplified_mesh returning None (covers 370->374 false branch)."""
        from mathutils import Vector

        obj = create_mesh_object("simplify_none_mesh", scene=scene, with_cube=True)
        obj.bound_box = [Vector((1.0, 1.0, 1.0))] * 8

        mocker.patch("linkforge.blender.adapters.mesh_io.create_simplified_mesh", return_value=None)
        mocker.patch("linkforge.blender.adapters.mesh_io.export_mesh_stl", return_value=True)

        filepath, offset = export_link_mesh(
            obj,
            "simplify_none_link",
            "collision",
            "STL",
            tmp_path,
            simplify=True,
            decimation_ratio=0.5,
        )
        assert filepath is not None

    def test_export_link_mesh_finally_cleanup_no_data(self, mocker, scene, tmp_path) -> None:
        """Verify finally cleanup block when simplified_obj.data and temp_export_obj.data are None (covers 403->405 and 409->411 false branches)."""
        import bpy

        obj = create_mesh_object("no_data_mesh", scene=scene, with_cube=True)

        mocker.patch("linkforge.blender.adapters.mesh_io.bpy.ops.object.modifier_apply")

        def mock_export_stl(export_obj, filepath):
            # Locate all temporary copy objects in bpy.data.objects and clear their data
            for o in list(bpy.data.objects):
                if "copy" in o.name or "decimate" in o.name:
                    o.data = None
            return True

        mocker.patch("linkforge.blender.adapters.mesh_io.export_mesh_stl", mock_export_stl)

        filepath, offset = export_link_mesh(
            obj, "no_data_link", "collision", "STL", tmp_path, simplify=True, decimation_ratio=0.5
        )
        assert filepath is not None
