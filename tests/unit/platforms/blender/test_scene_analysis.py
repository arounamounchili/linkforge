"""Unified scene analysis and caching tests for LinkForge Blender.

These tests ensure the high-performance architectural heart of the Blender
platform remains robust and 100% verified.
"""

from unittest.mock import patch

import bpy
from linkforge.blender.utils.scene_utils import clear_stats_cache, get_robot_statistics

from tests.blender_test_utils import (
    create_robot_link,
    create_test_object,
    safe_get_linkforge,
    safe_get_sensor,
    safe_get_transmission,
    setup_2_link_arm,
)


def test_get_robot_statistics_cache_hit(scene) -> None:
    """Test that statistics are successfully retrieved from the frame-level cache."""
    clear_stats_cache()

    # Setup scene
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None
    safe_get_linkforge(obj).is_robot_link = True
    safe_get_linkforge(obj).link_name = "cached_link"
    safe_get_linkforge(obj).mass = 1.0

    # First call - populates cache
    stats1 = get_robot_statistics(scene)
    assert stats1.num_links == 1

    # Second call - should hit cache (O(1) retrieval)
    stats2 = get_robot_statistics(scene)
    assert stats1 is stats2  # Identity check proves cache hit

    # Third call with force_refresh - should NOT hit cache
    stats3 = get_robot_statistics(scene, force_refresh=True)
    assert stats1 is not stats3
    assert stats3.num_links == 1


def test_get_robot_statistics_manual_inertia(scene) -> None:
    """Test detection of objects requiring manual inertia gizmos."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None
    safe_get_linkforge(obj).is_robot_link = True
    safe_get_linkforge(obj).use_auto_inertia = False
    safe_get_linkforge(obj).link_name = "manual_link"

    stats = get_robot_statistics(scene, force_refresh=True)
    assert len(stats.manual_inertia_objects) == 1
    assert stats.manual_inertia_objects[0] == obj


def test_get_robot_statistics_geometry_detection_urdf_tag(scene) -> None:
    """Test geometry detection via explicit source_geometry_type tag."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    assert link is not None
    safe_get_linkforge(link).is_robot_link = True
    safe_get_linkforge(link).link_name = "tag_link"

    # Add collision child
    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    assert coll is not None
    coll.name = "tag_link_collision"
    coll.parent = link
    coll["source_geometry_type"] = "SPHERE"

    stats = get_robot_statistics(scene, force_refresh=True)
    assert "tag_link" in stats.geometry_stats
    obj, gtype, is_prim = stats.geometry_stats["tag_link"]
    assert gtype == "SPHERE"
    assert is_prim is True


def test_get_robot_statistics_geometry_detection_stored_type(scene) -> None:
    """Test geometry detection via stored collision_geometry_type tag."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    assert link is not None
    safe_get_linkforge(link).is_robot_link = True
    safe_get_linkforge(link).link_name = "stored_link"

    # Add collision child
    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    assert coll is not None
    coll.name = "stored_link_collision"
    coll.parent = link
    coll["collision_geometry_type"] = "BOX"

    stats = get_robot_statistics(scene, force_refresh=True)
    assert "stored_link" in stats.geometry_stats
    obj, gtype, is_prim = stats.geometry_stats["stored_link"]
    assert gtype == "BOX"
    assert is_prim is True


def test_get_robot_statistics_geometry_detection_heuristic(scene) -> None:
    """Test geometry detection via heuristic topological analysis."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    assert link is not None
    safe_get_linkforge(link).is_robot_link = True
    safe_get_linkforge(link).link_name = "heuristic_link"

    # Add collision child (standard cube)
    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    assert coll is not None
    coll.name = "heuristic_link_collision"
    coll.parent = link

    stats = get_robot_statistics(scene, force_refresh=True)
    assert "heuristic_link" in stats.geometry_stats
    obj, gtype, is_prim = stats.geometry_stats["heuristic_link"]
    assert gtype == "BOX"
    assert is_prim is True


def test_get_robot_statistics_geometry_detection_non_primitive(scene) -> None:
    """Test heuristic fallback to MESH for a complex (non-primitive) object."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    assert link is not None
    safe_get_linkforge(link).is_robot_link = True
    safe_get_linkforge(link).link_name = "complex_link"

    # Create child
    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    assert coll is not None
    coll.name = "complex_link_collision"
    coll.parent = link

    # Subdivide to make it a non-primitive mesh
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.objects.active = coll
    bpy.ops.object.modifier_add(type="SUBSURF")
    bpy.ops.object.modifier_apply(modifier="Subdivision")

    stats = get_robot_statistics(scene, force_refresh=True)
    assert "complex_link" in stats.geometry_stats
    _, gtype, is_prim = stats.geometry_stats["complex_link"]
    assert gtype == "MESH"
    assert is_prim is False


def test_get_robot_statistics_heuristic_error_handling(scene) -> None:
    """Test robustness when heuristic detection fails."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    assert link is not None
    safe_get_linkforge(link).is_robot_link = True
    safe_get_linkforge(link).link_name = "error_link"

    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    assert coll is not None
    coll.name = "error_link_collision"
    coll.parent = link

    with patch(
        "linkforge.blender.utils.scene_utils.detect_primitive_type", side_effect=ValueError("Boom")
    ):
        stats = get_robot_statistics(scene, force_refresh=True)
        assert "error_link" in stats.geometry_stats
        _, gtype, _ = stats.geometry_stats["error_link"]
        assert gtype == "MESH"


def test_get_robot_statistics_joint_mapping(scene) -> None:
    """Test that joints correctly map parents to children."""
    parent, joint, child = setup_2_link_arm(scene)
    safe_get_linkforge(parent).link_name = "parent"
    safe_get_linkforge(child).link_name = "child"

    stats = get_robot_statistics(scene, force_refresh=True)
    assert len(stats.joint_objects) == 1
    assert stats.joint_objects[0] == joint
    assert stats.joints_map["child"][0] == "parent"


def test_get_robot_statistics_geometry_detection_mesh_tag(scene) -> None:
    """Test geometry detection forcing MESH type via stored tag."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    assert link is not None
    safe_get_linkforge(link).is_robot_link = True
    safe_get_linkforge(link).link_name = "mesh_link"

    # Add collision child
    bpy.ops.mesh.primitive_cube_add()
    coll = bpy.context.active_object
    assert coll is not None
    coll.name = "mesh_link_collision"
    coll.parent = link
    coll["source_geometry_type"] = "MESH"

    stats = get_robot_statistics(scene, force_refresh=True)
    assert "mesh_link" in stats.geometry_stats
    _, gtype, is_prim = stats.geometry_stats["mesh_link"]
    assert gtype == "MESH"
    assert is_prim is False


def test_get_robot_statistics_multiple_links(scene) -> None:
    """Test detection of multiple links and robots."""
    bpy.ops.mesh.primitive_cube_add()
    l1 = bpy.context.active_object
    assert l1 is not None
    safe_get_linkforge(l1).is_robot_link = True
    safe_get_linkforge(l1).link_name = "link1"

    bpy.ops.mesh.primitive_cube_add()
    l2 = bpy.context.active_object
    assert l2 is not None
    safe_get_linkforge(l2).is_robot_link = True
    safe_get_linkforge(l2).link_name = "link2"

    stats = get_robot_statistics(scene, force_refresh=True)
    assert stats.num_links == 2
    assert "link1" in stats.link_objects
    assert "link2" in stats.link_objects


def test_get_robot_statistics_unnamed_links(scene) -> None:
    """Test handling of links that haven't been assigned a name yet."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    assert link is not None
    safe_get_linkforge(link).is_robot_link = True
    # Link name is empty or default

    stats = get_robot_statistics(scene, force_refresh=True)
    # It should still be detected as a link, using object name as fallback
    assert stats.num_links == 1


def test_get_robot_statistics_transmission_detection(scene) -> None:
    """Test detection of transmissions attached to joints."""
    parent, joint, child = setup_2_link_arm(scene)

    # Create transmission object
    trans_obj = create_test_object("trans", None, scene)
    assert trans_obj is not None
    trans_props = safe_get_transmission(trans_obj)
    trans_props.is_robot_transmission = True
    trans_props.transmission_name = "test_trans"
    trans_props.joint_name = joint

    stats = get_robot_statistics(scene, force_refresh=True)
    assert len(stats.transmission_objects) == 1
    assert stats.transmission_objects[0] == trans_obj


def test_get_robot_statistics_sensor_detection(scene) -> None:
    """Test detection of sensors attached to links."""
    link = create_robot_link("sensor_link", scene)
    assert link is not None

    # Create sensor object
    sensor_obj = create_test_object("sensor", None, scene)
    assert sensor_obj is not None
    sensor_props = safe_get_sensor(sensor_obj)
    sensor_props.is_robot_sensor = True
    sensor_props.sensor_name = "test_sensor"
    sensor_props.attached_link = link

    stats = get_robot_statistics(scene, force_refresh=True)
    assert len(stats.sensor_objects) == 1
    assert stats.sensor_objects[0] == sensor_obj
