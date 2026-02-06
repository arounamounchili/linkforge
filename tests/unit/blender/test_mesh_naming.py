from pathlib import Path

import bpy
from linkforge.blender.converters import blender_link_to_core_with_origin


def test_single_visual_no_suffix():
    """Test that a single visual mesh has no suffix."""
    # Setup link and one visual child
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "test_link"
    link_obj.linkforge.is_robot_link = True

    bpy.ops.mesh.primitive_cube_add()
    visual_obj = bpy.context.active_object
    visual_obj.name = "test_link_visual"
    visual_obj.parent = link_obj

    # Subdivide to make it a complex mesh (not a primitive)
    bpy.ops.object.modifier_add(type="SUBSURF")
    bpy.ops.object.modifier_apply(modifier="Subdivision")

    # Convert to core
    core_link = blender_link_to_core_with_origin(link_obj, meshes_dir=Path("/tmp"))

    # Assert
    assert len(core_link.visuals) == 1
    # The Mesh geometry should have a filepath "test_link_visual.stl" (no _0)
    assert hasattr(core_link.visuals[0].geometry, "filepath")
    assert core_link.visuals[0].geometry.filepath.name == "test_link_visual.stl"


def test_multiple_visuals_with_suffix():
    """Test that multiple visual meshes have suffixes."""
    # Setup link and two visual children
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "multi_link"
    link_obj.linkforge.is_robot_link = True

    bpy.ops.mesh.primitive_cube_add()
    visual_a = bpy.context.active_object
    visual_a.name = "multi_link_visual_a"
    visual_a.parent = link_obj
    bpy.ops.object.modifier_add(type="SUBSURF")
    bpy.ops.object.modifier_apply(modifier="Subdivision")

    bpy.ops.mesh.primitive_cube_add()
    visual_b = bpy.context.active_object
    visual_b.name = "multi_link_visual_b"
    visual_b.parent = link_obj
    bpy.ops.object.modifier_add(type="SUBSURF")
    bpy.ops.object.modifier_apply(modifier="Subdivision")

    # Convert to core
    core_link = blender_link_to_core_with_origin(link_obj, meshes_dir=Path("/tmp"))

    # Assert
    assert len(core_link.visuals) == 2
    # Filenames should be "multi_link_visual_0.stl" and "multi_link_visual_1.stl"
    filenames = [v.geometry.filepath.name for v in core_link.visuals]
    assert "multi_link_visual_0.stl" in filenames
    assert "multi_link_visual_1.stl" in filenames


def test_urdf_name_preservation():
    """Test that urdf_name is preserved even for single meshes."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "name_link"
    link_obj.linkforge.is_robot_link = True

    bpy.ops.mesh.primitive_cube_add()
    visual_obj = bpy.context.active_object
    visual_obj.name = "name_link_visual"
    visual_obj.parent = link_obj
    visual_obj["urdf_name"] = "custom_part"

    # Subdivide
    bpy.ops.object.modifier_add(type="SUBSURF")
    bpy.ops.object.modifier_apply(modifier="Subdivision")

    # Convert to core
    core_link = blender_link_to_core_with_origin(link_obj, meshes_dir=Path("/tmp"))

    # Assert
    assert len(core_link.visuals) == 1
    # Filename should be "name_link_visual_custom_part.stl"
    assert hasattr(core_link.visuals[0].geometry, "filepath")
    assert core_link.visuals[0].geometry.filepath.name == "name_link_visual_custom_part.stl"
