import bpy
from linkforge.blender.mesh_export import (
    create_simplified_mesh,
    export_link_mesh,
    export_mesh_glb,
    export_mesh_obj,
    export_mesh_stl,
    get_mesh_filename,
)


def test_export_mesh_stl_success(tmp_path):
    """Test successful STL export."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    export_path = tmp_path / "test_cube.stl"
    result = export_mesh_stl(obj, export_path)

    assert result is True
    assert export_path.exists()
    assert export_path.stat().st_size > 0


def test_export_mesh_stl_invalid_object(tmp_path):
    """Test STL export failure with None object."""
    export_path = tmp_path / "invalid.stl"
    result = export_mesh_stl(None, export_path)
    assert result is False


def test_export_mesh_obj_success(tmp_path):
    """Test successful OBJ export."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    export_path = tmp_path / "test_cube.obj"
    result = export_mesh_obj(obj, export_path)

    assert result is True
    assert export_path.exists()
    assert export_path.stat().st_size > 0


def test_export_mesh_obj_invalid_object(tmp_path):
    """Test OBJ export failure with None object."""
    export_path = tmp_path / "invalid.obj"
    result = export_mesh_obj(None, export_path)
    assert result is False


def test_export_mesh_glb_success(tmp_path):
    """Test successful GLB export."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    export_path = tmp_path / "test_cube.glb"
    result = export_mesh_glb(obj, export_path)

    assert result is True
    assert export_path.exists()
    assert export_path.stat().st_size > 0


def test_export_mesh_glb_invalid_object(tmp_path):
    """Test GLB export failure with None object."""
    export_path = tmp_path / "invalid.glb"
    result = export_mesh_glb(None, export_path)
    assert result is False


def test_create_simplified_mesh():
    """Test mesh simplification with decimation."""
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16)
    obj = bpy.context.active_object
    original_poly_count = len(obj.data.polygons)

    simplified = create_simplified_mesh(obj, decimation_ratio=0.5)

    assert simplified is not None
    assert simplified.type == "MESH"
    assert len(simplified.data.polygons) < original_poly_count

    # Cleanup
    bpy.data.objects.remove(simplified, do_unlink=True)


def test_create_simplified_mesh_invalid_object():
    """Test mesh simplification with invalid object."""
    result = create_simplified_mesh(None, 0.5)
    assert result is None


def test_create_simplified_mesh_non_mesh():
    """Test mesh simplification with non-mesh object."""
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object

    result = create_simplified_mesh(obj, 0.5)
    assert result is None


def test_get_mesh_filename():
    """Test mesh filename generation."""
    assert get_mesh_filename("base_link", "visual", "STL") == "base_link_visual.stl"
    assert get_mesh_filename("arm_link", "collision", "OBJ") == "arm_link_collision.obj"
    assert get_mesh_filename("wheel", "collision", "GLB") == "wheel_collision.glb"


def test_export_link_mesh_stl(tmp_path):
    """Test export_link_mesh with STL format."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    result = export_link_mesh(
        obj=obj,
        link_name="test_link",
        geometry_type="visual",
        mesh_format="STL",
        meshes_dir=tmp_path,
    )

    assert result is not None
    assert result.exists()
    assert result.name == "test_link_visual.stl"


def test_export_link_mesh_with_simplification(tmp_path):
    """Test export_link_mesh with mesh simplification."""
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16)
    obj = bpy.context.active_object

    result = export_link_mesh(
        obj=obj,
        link_name="collision_link",
        geometry_type="collision",
        mesh_format="STL",
        meshes_dir=tmp_path,
        simplify=True,
        decimation_ratio=0.3,
    )

    assert result is not None
    assert result.exists()


def test_export_link_mesh_dry_run(tmp_path):
    """Test export_link_mesh in dry run mode."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    result = export_link_mesh(
        obj=obj,
        link_name="dry_run_link",
        geometry_type="visual",
        mesh_format="OBJ",
        meshes_dir=tmp_path,
        dry_run=True,
    )

    assert result is not None
    assert result == tmp_path / "dry_run_link_visual.obj"
    assert not result.exists()  # Should not actually export


def test_export_link_mesh_invalid_object(tmp_path):
    """Test export_link_mesh with None object."""
    result = export_link_mesh(
        obj=None,
        link_name="invalid",
        geometry_type="visual",
        mesh_format="STL",
        meshes_dir=tmp_path,
    )

    assert result is None


def test_export_link_mesh_non_mesh_object(tmp_path):
    """Test export_link_mesh with non-mesh object."""
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object

    result = export_link_mesh(
        obj=obj,
        link_name="empty_obj",
        geometry_type="visual",
        mesh_format="STL",
        meshes_dir=tmp_path,
    )

    assert result is None


def test_export_link_mesh_unknown_format(tmp_path):
    """Test export_link_mesh with unknown format defaults to OBJ."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    result = export_link_mesh(
        obj=obj,
        link_name="unknown_format",
        geometry_type="visual",
        mesh_format="UNKNOWN",
        meshes_dir=tmp_path,
    )

    assert result is not None
    assert result.suffix == ".obj"  # Should default to OBJ
