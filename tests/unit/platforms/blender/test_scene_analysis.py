"""Unified scene analysis and caching tests for LinkForge Blender.

These tests ensure the high-performance architectural heart of the Blender
platform remains robust and 100% verified.
"""

from unittest.mock import patch

import bpy
from linkforge.blender.utils.scene_utils import clear_stats_cache, get_robot_statistics


def test_get_robot_statistics_cache_hit():
    """Test that statistics are successfully retrieved from the frame-level cache."""
    clear_stats_cache()

    # Setup scene
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    obj.linkforge.is_robot_link = True
    obj.linkforge.link_name = "cached_link"
    obj.linkforge.mass = 1.0

    # First call - populates cache
    stats1 = get_robot_statistics(bpy.context.scene)
    assert stats1.num_links == 1

    # Second call - should hit cache (O(1) retrieval)
    stats2 = get_robot_statistics(bpy.context.scene)
    assert stats1 is stats2  # Identity check proves cache hit

    # Third call with force_refresh - should NOT hit cache
    stats3 = get_robot_statistics(bpy.context.scene, force_refresh=True)
    assert stats1 is not stats3
    assert stats3.num_links == 1


def test_get_robot_statistics_manual_inertia():
    """Test detection of objects requiring manual inertia gizmos."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    obj.linkforge.is_robot_link = True
    obj.linkforge.use_auto_inertia = False
    obj.linkforge.link_name = "manual_link"

    stats = get_robot_statistics(bpy.context.scene, force_refresh=True)
    assert len(stats.manual_inertia_objects) == 1
    assert stats.manual_inertia_objects[0] == obj


def test_get_robot_statistics_geometry_detection_urdf_tag():
    """Test geometry detection via explicit urdf_geometry_type tag."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    link.linkforge.is_robot_link = True
    link.linkforge.link_name = "tag_link"

    # Add collision child
    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    coll.name = "tag_link_collision"
    coll.parent = link
    coll["urdf_geometry_type"] = "SPHERE"

    stats = get_robot_statistics(bpy.context.scene, force_refresh=True)
    assert "tag_link" in stats.geometry_stats
    obj, gtype, is_prim = stats.geometry_stats["tag_link"]
    assert gtype == "SPHERE"
    assert is_prim is True


def test_get_robot_statistics_geometry_detection_stored_type():
    """Test geometry detection via stored collision_geometry_type tag."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    link.linkforge.is_robot_link = True
    link.linkforge.link_name = "stored_link"

    # Add collision child
    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    coll.name = "stored_link_collision"
    coll.parent = link
    coll["collision_geometry_type"] = "BOX"

    stats = get_robot_statistics(bpy.context.scene, force_refresh=True)
    assert "stored_link" in stats.geometry_stats
    obj, gtype, is_prim = stats.geometry_stats["stored_link"]
    assert gtype == "BOX"
    assert is_prim is True


def test_get_robot_statistics_geometry_detection_heuristic():
    """Test geometry detection via heuristic topological analysis."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    link.linkforge.is_robot_link = True
    link.linkforge.link_name = "heuristic_link"

    # Add collision child (standard cube)
    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    coll.name = "heuristic_link_collision"
    coll.parent = link

    stats = get_robot_statistics(bpy.context.scene, force_refresh=True)
    assert "heuristic_link" in stats.geometry_stats
    obj, gtype, is_prim = stats.geometry_stats["heuristic_link"]
    assert gtype == "BOX"
    assert is_prim is True


def test_get_robot_statistics_geometry_detection_non_primitive():
    """Test heuristic fallback to MESH for a complex (non-primitive) object."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    link.linkforge.is_robot_link = True
    link.linkforge.link_name = "complex_link"

    # Create child
    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    coll.name = "complex_link_collision"
    coll.parent = link

    # Subdivide to make it a non-primitive mesh
    bpy.context.view_layer.objects.active = coll
    bpy.ops.object.modifier_add(type="SUBSURF")
    bpy.ops.object.modifier_apply(modifier="Subdivision")

    stats = get_robot_statistics(bpy.context.scene, force_refresh=True)
    assert "complex_link" in stats.geometry_stats
    _, gtype, is_prim = stats.geometry_stats["complex_link"]
    assert gtype == "MESH"
    assert is_prim is False


def test_get_robot_statistics_heuristic_error_handling():
    """Test robustness when heuristic detection fails."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    link.linkforge.is_robot_link = True
    link.linkforge.link_name = "error_link"

    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    coll.name = "error_link_collision"
    coll.parent = link

    with patch(
        "linkforge.blender.utils.scene_utils.detect_primitive_type", side_effect=ValueError("Boom")
    ):
        stats = get_robot_statistics(bpy.context.scene, force_refresh=True)
        assert "error_link" in stats.geometry_stats
        _, gtype, _ = stats.geometry_stats["error_link"]
        assert gtype == "MESH"


def test_get_robot_statistics_geometry_detection_mesh_tag():
    """Test geometry detection forcing MESH type via stored tag."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    link.linkforge.is_robot_link = True
    link.linkforge.link_name = "mesh_link"

    # Add collision child
    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    coll.name = "mesh_link_collision"
    coll.parent = link
    coll["collision_geometry_type"] = "MESH"

    stats = get_robot_statistics(bpy.context.scene, force_refresh=True)
    assert "mesh_link" in stats.geometry_stats
    _, gtype, is_prim = stats.geometry_stats["mesh_link"]
    assert gtype == "MESH"
    assert is_prim is False


def test_get_robot_statistics_stale_cache_recovery():
    """Test recovery from stale cache when an object is deleted in the same frame."""
    clear_stats_cache()

    # Setup link and collision
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    link.linkforge.is_robot_link = True
    link.linkforge.link_name = "stale_link"

    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    coll.name = "stale_link_collision"
    coll.parent = link
    coll["collision_geometry_type"] = "BOX"

    # Populate cache
    stats1 = get_robot_statistics(bpy.context.scene)
    assert len(stats1.geometry_stats) == 1

    # Simulate operator deleting the object (same frame)
    # We don't change frame or object count significantly here to force same cache key
    # (Actually object count changes, but let's assume it stays same if replaced)
    bpy.data.objects.remove(coll, do_unlink=True)

    # Add a dummy object so count is the same
    bpy.ops.mesh.primitive_cube_add()
    dummy = bpy.context.active_object
    dummy.name = "dummy"

    # Next call should detect ReferenceError in cached stats and recompute
    stats2 = get_robot_statistics(bpy.context.scene)
    assert stats2 is not stats1  # Should NOT be the same object
    assert len(stats2.geometry_stats) == 0  # Collision is gone
