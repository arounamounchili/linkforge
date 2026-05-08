import bpy
from linkforge.blender.adapters.mesh_io import export_link_mesh


def test_high_fidelity_multi_geometry_export_suffixes(tmp_path, scene) -> None:
    """Verify that the unique suffixes correctly generate distinct collision/visual filenames."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None

    # Export 1
    p1, _ = export_link_mesh(
        obj=obj,
        link_name="base_link",
        geometry_type="visual",
        mesh_format="STL",
        meshes_dir=tmp_path,
        suffix="_0",
    )
    assert p1 is not None
    assert p1.name == "base_link_visual_0.stl"

    # Export 2 (mimicking second visual)
    p2, _ = export_link_mesh(
        obj=obj,
        link_name="base_link",
        geometry_type="visual",
        mesh_format="STL",
        meshes_dir=tmp_path,
        suffix="_1",
    )
    assert p2 is not None
    assert p2.name == "base_link_visual_1.stl"


def test_filename_sanitization_conformity(tmp_path, scene) -> None:
    """Verify that LinkForge aggressively sanitizes link names to ensure robot model filename compatibility."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None

    # link_name with spaces and dots
    p, _ = export_link_mesh(
        obj=obj,
        link_name="my link.001",
        geometry_type="visual",
        mesh_format="STL",
        meshes_dir=tmp_path,
    )
    assert p is not None
    # sanitize_name("my link.001") -> "my_link_001"
    assert "my_link_001" in p.name
    assert " " not in p.name
    assert "." not in p.stem


def test_mesh_export_suffix_hardening(tmp_path, scene) -> None:
    """Verify that injected suffixes are hardened against illegal characters."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None

    p, _ = export_link_mesh(
        obj=obj,
        link_name="base_link",
        geometry_type="visual",
        mesh_format="STL",
        meshes_dir=tmp_path,
        suffix="_invalid name!",
    )
    assert p is not None
    # The suffix is already sanitized in converters.py before calling export_link_mesh
    # If we pass an unsanitized one here for testing, the generator should handle it.
    assert "!" not in p.name
    assert " " not in p.name
