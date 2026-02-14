from unittest.mock import patch

import bpy
import pytest
from linkforge.blender.adapters.mesh_io import (
    create_simplified_mesh,
    export_link_mesh,
    export_mesh_glb,
    export_mesh_obj,
    export_mesh_stl,
)
from mathutils import Matrix


def test_mesh_io_simplification(clean_scene):
    """Test create_simplified_mesh and decimation."""
    bpy.ops.mesh.primitive_cube_add()
    o = bpy.context.active_object

    # 1. Non-mesh input
    assert create_simplified_mesh(None, 0.5) is None
    e = bpy.data.objects.new("Empty", None)
    assert create_simplified_mesh(e, 0.5) is None

    # 2. Success path
    simplified = create_simplified_mesh(o, 0.1)
    assert simplified is not None
    assert simplified.name != o.name
    # Clean up
    bpy.data.objects.remove(simplified, do_unlink=True)


def test_export_mesh_error_handling(clean_scene, tmp_path):
    """Hit error paths in mesh exporters using Path mocks."""
    bpy.ops.mesh.primitive_monkey_add()
    o = bpy.context.active_object
    filepath = tmp_path / "protected" / "mesh.stl"

    # Patch mkdir to prevent export from even starting (cleanest trigger)
    with patch(
        "linkforge.blender.adapters.mesh_io.Path.mkdir", side_effect=OSError("Permission Denied")
    ):
        assert export_mesh_stl(o, filepath) is False
        assert export_mesh_obj(o, filepath) is False
        assert export_mesh_glb(o, filepath) is False


def test_export_link_mesh_success_and_centering(clean_scene, tmp_path):
    """Verify combined centering and export logic."""
    bpy.ops.mesh.primitive_cube_add(location=(1, 2, 3))
    o = bpy.context.active_object
    # Ensure active for mode_set later
    bpy.context.view_layer.objects.active = o
    o.select_set(True)

    # Scale it to verify scale baking
    o.scale = (2, 2, 2)

    # 1. STL Success
    path, mat = export_link_mesh(o, "Link", "visual", "STL", tmp_path)
    assert path is not None
    assert path.exists()
    # Check that the returned matrix includes the original world position
    assert mat.translation.x == pytest.approx(1.0)
    assert mat.translation.y == pytest.approx(2.0)
    assert mat.translation.z == pytest.approx(3.0)

    # 2. Centering Logic Verification (Offset mesh)
    # Re-select for mode set
    bpy.ops.object.select_all(action="DESELECT")
    o.select_set(True)
    bpy.context.view_layer.objects.active = o

    # Move vertices far from origin in edit mode
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.transform.translate(value=(10, 0, 0))
    bpy.ops.object.mode_set(mode="OBJECT")

    path2, mat2 = export_link_mesh(o, "Offset", "visual", "STL", tmp_path)
    assert path2 is not None
    # geom_world_matrix should now include the +10 offset
    assert mat2.translation.x == pytest.approx(11.0)  # 1 (obj) + 10 (vertex offset)

    # 3. OBJ Success
    path_obj, _ = export_link_mesh(o, "Link", "visual", "OBJ", tmp_path)
    assert path_obj is not None and path_obj.exists()

    # 4. GLB Success
    path_glb, _ = export_link_mesh(o, "Link", "visual", "GLB", tmp_path)
    assert path_glb is not None and path_glb.exists()

    # 5. Unknown format fallback
    with patch("linkforge.blender.adapters.mesh_io.export_mesh_obj", return_value=True) as mock_obj:
        path3, _ = export_link_mesh(o, "Link", "visual", "UNKNOWN", tmp_path)
        assert mock_obj.called
        assert path3 is not None

    # 8. Success with simplification (Hits all cleanup lines in finalize)
    # Patch export_mesh_stl to return True so we hit the full loop
    # 9. Success with simplification (Hits all cleanup lines in finalize)
    # Patch format-specific exporter to return True
    with patch("linkforge.blender.adapters.mesh_io.export_mesh_stl", return_value=True):
        # simplify=True triggers create_simplified_mesh and cleanup in finally
        path, _ = export_link_mesh(
            o, "Link", "collision", "STL", tmp_path, simplify=True, decimation_ratio=0.1
        )
        assert path is not None

    # 11. OBJ/GLB failure triggers in export_link_mesh loop
    # Patch the lower-level exporter called within the loop
    with patch(
        "linkforge.blender.adapters.mesh_io.export_mesh_obj", side_effect=RuntimeError("OBJ Forced")
    ):
        path, center_matrix = export_link_mesh(o, "Link", "visual", "OBJ", tmp_path)
        assert path is None
        # In error case, matrix should be identity or whatever it was before
        assert isinstance(center_matrix, Matrix)

    with patch(
        "linkforge.blender.adapters.mesh_io.export_mesh_glb", side_effect=RuntimeError("GLB Forced")
    ):
        path, center_matrix = export_link_mesh(o, "Link", "visual", "GLB", tmp_path)
        assert path is None
