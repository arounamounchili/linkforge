from pathlib import Path

import bpy
import pytest
from linkforge.blender.mesh_export import (
    create_simplified_mesh,
    export_link_mesh,
    export_mesh_glb,
    export_mesh_obj,
    export_mesh_stl,
    get_mesh_filename,
)
from mathutils import Vector


def test_get_mesh_filename():
    """Test mesh filename generation with various inputs."""
    assert get_mesh_filename("base_link", "visual", "STL") == "base_link_visual.stl"
    assert (
        get_mesh_filename("base_link", "collision", "OBJ", suffix="_0")
        == "base_link_collision_0.obj"
    )
    assert get_mesh_filename("Link A", "visual", "glb") == "Link_A_visual.glb"
    assert get_mesh_filename("link", "visual", "STL", suffix="My Mesh") == "link_visualMy_Mesh.stl"


def test_export_mesh_internal_dispatch_logic():
    """Test that individual export functions handle None inputs safely."""
    assert export_mesh_stl(None, Path("/tmp/none.stl")) is False
    assert export_mesh_obj(None, Path("/tmp/none.obj")) is False
    assert export_mesh_glb(None, Path("/tmp/none.glb")) is False


def test_export_link_mesh_logic(mocker):
    """Test that export_link_mesh correctly calculates the geometric offset."""
    bpy.ops.mesh.primitive_cube_add(size=2.0, location=(5.0, 5.0, 5.0))
    obj = bpy.context.active_object

    # Shift vertices by (1, 0, 0)
    for vert in obj.data.vertices:
        vert.co += Vector((1, 0, 0))

    bpy.context.view_layer.update()

    # Mock high-level functions to verify the complex centering and dispatch math
    mocker.patch("linkforge.blender.mesh_export.export_mesh_stl", return_value=True)
    mocker.patch("linkforge.blender.mesh_export.export_mesh_obj", return_value=True)
    mocker.patch("linkforge.blender.mesh_export.export_mesh_glb", return_value=True)

    meshes_dir = Path("/tmp")

    # 1. STL path
    path, mat = export_link_mesh(obj, "link", "visual", "STL", meshes_dir)
    assert path.suffix == ".stl"
    assert tuple(mat.translation) == pytest.approx((6.0, 5.0, 5.0))

    # 2. OBJ path
    path, _ = export_link_mesh(obj, "link", "collision", "OBJ", meshes_dir)
    assert path.suffix == ".obj"

    # 3. GLB path
    path, _ = export_link_mesh(obj, "link", "visual", "GLB", meshes_dir)
    assert path.suffix == ".glb"

    bpy.data.objects.remove(obj, do_unlink=True)


def test_create_simplified_mesh():
    """Test simplification coverage."""
    bpy.ops.mesh.primitive_uv_sphere_add()
    obj = bpy.context.active_object

    # Test None
    assert create_simplified_mesh(None, 0.5) is None

    # Test non-mesh
    bpy.ops.object.empty_add()
    empty = bpy.context.active_object
    assert create_simplified_mesh(empty, 0.5) is None

    simplified = create_simplified_mesh(obj, 0.5)
    assert simplified is not None

    bpy.data.objects.remove(simplified, do_unlink=True)
    bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.objects.remove(empty, do_unlink=True)
