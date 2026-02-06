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


def test_export_mesh_operator_success(mocker):
    """Test success paths for OBJ and GLB using module-level mocks."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    path = Path("/tmp/test.obj")

    # Mock the operators in the module namespace to avoid real C calls
    mocker.patch("linkforge.blender.mesh_export.bpy.ops.wm.obj_export")
    mocker.patch("linkforge.blender.mesh_export.bpy.ops.export_scene.gltf")

    assert export_mesh_obj(obj, path) is True
    assert export_mesh_glb(obj, path.with_suffix(".glb")) is True

    bpy.data.objects.remove(obj, do_unlink=True)


def test_export_link_mesh_logic(mocker):
    """Test that export_link_mesh correctly calculates the geometric offset."""
    bpy.ops.mesh.primitive_cube_add(size=2.0, location=(5.0, 5.0, 5.0))
    obj = bpy.context.active_object

    # Shift vertices
    for vert in obj.data.vertices:
        vert.co += Vector((1, 0, 0))

    bpy.context.view_layer.update()

    mocker.patch("linkforge.blender.mesh_export.export_mesh_stl", return_value=True)
    mocker.patch("linkforge.blender.mesh_export.export_mesh_obj", return_value=True)
    mocker.patch("linkforge.blender.mesh_export.export_mesh_glb", return_value=True)

    meshes_dir = Path("/tmp")

    # STL
    path, mat = export_link_mesh(obj, "link", "visual", "STL", meshes_dir)
    assert path.suffix == ".stl"
    assert tuple(mat.translation) == pytest.approx((6.0, 5.0, 5.0))

    # Fallback and Simplification
    path, _ = export_link_mesh(obj, "link", "visual", "FOO", meshes_dir)
    assert path.suffix == ".obj"  # Default

    path, _ = export_link_mesh(obj, "link", "collision", "STL", meshes_dir, simplify=True)
    assert path is not None

    bpy.data.objects.remove(obj, do_unlink=True)


def test_create_simplified_mesh():
    """Test simplification coverage."""
    bpy.ops.mesh.primitive_uv_sphere_add()
    obj = bpy.context.active_object

    assert create_simplified_mesh(None, 0.5) is None
    bpy.ops.object.empty_add()
    empty = bpy.context.active_object
    assert create_simplified_mesh(empty, 0.5) is None

    simplified = create_simplified_mesh(obj, 0.5)
    assert simplified is not None

    bpy.data.objects.remove(simplified, do_unlink=True)
    bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.objects.remove(empty, do_unlink=True)


def test_export_link_mesh_error_dispatch(mocker):
    """Test that export_link_mesh returns None on sub-function failure."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    mocker.patch("linkforge.blender.mesh_export.export_mesh_stl", return_value=False)
    path, mat = export_link_mesh(obj, "l", "v", "STL", Path("/tmp"))
    assert path is None

    bpy.data.objects.remove(obj, do_unlink=True)
