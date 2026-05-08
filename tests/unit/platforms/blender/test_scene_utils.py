"""Tests for scene utility functions."""

from __future__ import annotations

import bpy
from linkforge.blender.utils.scene_utils import (
    build_tree_from_stats,
    get_robot_statistics,
    is_robot_joint,
    is_robot_link,
    is_robot_sensor,
    is_robot_transmission,
    move_to_collection,
)

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_sensor,
    safe_get_transmission,
)


def test_is_robot_link_with_valid_link(scene) -> None:
    """Test is_robot_link returns True for valid robot link object."""
    obj = create_test_object("test_link", None, scene)
    safe_get_linkforge(obj).is_robot_link = True
    safe_get_linkforge(obj).link_name = "test_link"

    assert is_robot_link(obj) is True


def test_is_robot_link_with_non_link(scene) -> None:
    """Test is_robot_link returns False for non-link object."""
    obj = create_test_object("non_link", None, scene)

    assert is_robot_link(obj) is False


def test_is_robot_link_with_none(scene) -> None:
    """Test is_robot_link handles None input without throwing an error."""
    assert is_robot_link(None) is False


def test_is_robot_joint_with_valid_joint(scene) -> None:
    """Test is_robot_joint returns True for valid robot_joint object."""
    obj = create_test_object("test_joint", None, scene)
    safe_get_joint(obj).is_robot_joint = True
    safe_get_joint(obj).joint_name = "test_joint"
    safe_get_joint(obj).joint_type = "REVOLUTE"

    assert is_robot_joint(obj) is True


def test_is_robot_joint_with_mesh_object(scene) -> None:
    """Test is_robot_joint returns False for objects that cannot be robot_joints."""
    obj = create_test_object("mesh_obj", None, scene)

    assert is_robot_joint(obj) is False


def test_is_robot_joint_with_empty_not_marked(scene) -> None:
    """Test is_robot_joint returns False for non robot_joint objects."""
    obj = create_test_object("unmarked_empty", None, scene)

    assert is_robot_joint(obj) is False


def test_is_robot_sensor_with_valid_sensor(scene) -> None:
    """Test is_robot_sensor returns True for valid robot_sensor object."""
    obj = create_test_object("test_sensor", None, scene)
    safe_get_sensor(obj).is_robot_sensor = True
    safe_get_sensor(obj).sensor_name = "test_sensor"
    safe_get_sensor(obj).sensor_type = "CAMERA"

    assert is_robot_sensor(obj) is True


def test_is_robot_sensor_with_mesh_object(scene) -> None:
    """Test is_robot_sensor returns False for objects that cannot be robot_sensors."""
    obj = create_test_object("mesh_sensor", None, scene)

    assert is_robot_sensor(obj) is False


def test_is_robot_sensor_with_empty_not_marked(scene) -> None:
    """Test is_robot_sensor returns False for non robot_sensor objects."""
    obj = create_test_object("unmarked_sensor", None, scene)

    assert is_robot_sensor(obj) is False


def test_is_robot_transmission_with_valid_transmission(scene) -> None:
    """Test is_robot_transmission returns True for valid transmission."""
    # Create an empty and mark it as a transmission
    obj = create_test_object("test_transmission", None, scene)
    safe_get_transmission(obj).is_robot_transmission = True
    safe_get_transmission(obj).transmission_name = "test_transmission"

    assert is_robot_transmission(obj) is True


def test_is_robot_transmission_with_unmarked_object(scene) -> None:
    """Test is_robot_transmission returns False for unmarked object."""
    # Create an object but don't mark it as a transmission
    obj = create_test_object("unmarked_mesh", None, scene)

    assert is_robot_transmission(obj) is False


def test_move_to_collection_basic(scene) -> None:
    """Test moving an object to a new collection."""
    # Clean scene
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create a cube
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Create a new collection
    target_collection = bpy.data.collections.new("TestCollection")
    scene.collection.children.link(target_collection)

    # Move object to new collection
    move_to_collection(obj, target_collection)

    # Verify object is in target collection
    assert obj in target_collection.objects[:]
    # Verify object is not in scene root collection
    assert obj not in scene.collection.objects[:]


def test_move_to_collection_already_in_target(scene) -> None:
    """Test moving an object to its current collection."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create collection and object
    target_collection = bpy.data.collections.new("TestCollection")
    scene.collection.children.link(target_collection)

    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Move to target once
    move_to_collection(obj, target_collection)

    # Move again (should be idempotent)
    move_to_collection(obj, target_collection)

    # Should still be in target collection only once
    assert obj in target_collection.objects[:]
    assert list(obj.users_collection) == [target_collection]


def test_move_to_collection_none_object(scene) -> None:
    """Test with None object."""
    target_collection = bpy.data.collections.new("TestCollection")
    scene.collection.children.link(target_collection)

    # Should not raise error
    move_to_collection(None, target_collection)


def test_move_to_collection_none_collection(scene) -> None:
    """Test with None collection."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Should not raise error
    move_to_collection(obj, None)


def test_move_to_collection_both_none(scene) -> None:
    """Test with both None."""
    move_to_collection(None, None)
    # Should not raise error


def test_move_to_collection_multiple_collections(scene) -> None:
    """Test moving object that exists in multiple collections."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create object
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Create multiple collections and link object
    coll1 = bpy.data.collections.new("Collection1")
    coll2 = bpy.data.collections.new("Collection2")
    target_coll = bpy.data.collections.new("TargetCollection")

    scene.collection.children.link(coll1)
    scene.collection.children.link(coll2)
    scene.collection.children.link(target_coll)

    coll1.objects.link(obj)
    coll2.objects.link(obj)

    # Move to target
    move_to_collection(obj, target_coll)

    # Should only be in target collection now
    assert obj in target_coll.objects[:]
    assert obj not in coll1.objects[:]
    assert obj not in coll2.objects[:]


def test_get_robot_statistics_empty_scene(scene) -> None:
    """Test get_robot_statistics with empty scene returns zeros."""
    stats = get_robot_statistics(scene)

    assert stats.num_links == 0
    assert stats.total_mass == 0.0
    assert stats.total_dof == 0
    assert len(stats.link_objects) == 0
    assert len(stats.joint_objects) == 0
    assert len(stats.sensor_objects) == 0
    assert len(stats.transmission_objects) == 0
    assert stats.root_link is None


def test_get_robot_statistics_none_scene(scene) -> None:
    """Test get_robot_statistics with None returns zeros."""
    stats = get_robot_statistics(None)

    assert stats.num_links == 0
    assert stats.total_mass == 0.0
    assert stats.total_dof == 0
    assert len(stats.link_objects) == 0
    assert len(stats.joint_objects) == 0
    assert len(stats.sensor_objects) == 0
    assert len(stats.transmission_objects) == 0
    assert stats.root_link is None


def test_get_robot_statistics_with_links(scene) -> None:
    """Test get_robot_statistics counts links and calculates mass."""
    link1 = create_test_object("base_link", None, scene)
    link1.location = (0, 0, 0)
    safe_get_linkforge(link1).is_robot_link = True
    safe_get_linkforge(link1).link_name = "base_link"
    safe_get_linkforge(link1).mass = 5.0

    link2 = create_test_object("body_link", None, scene)
    link2.location = (1, 0, 0)
    safe_get_linkforge(link2).is_robot_link = True
    safe_get_linkforge(link2).link_name = "body_link"
    safe_get_linkforge(link2).mass = 10.0

    link3 = create_test_object("gripper_link", None, scene)
    link3.location = (2, 0, 0)
    safe_get_linkforge(link3).is_robot_link = True
    safe_get_linkforge(link3).link_name = "gripper_link"
    safe_get_linkforge(link3).mass = 2.5

    stats = get_robot_statistics(scene)

    assert stats.num_links == 3
    assert stats.total_mass == 17.5  # 5.0 + 10.0 + 2.5
    assert stats.total_dof == 0  # no joints yet
    assert len(stats.link_objects) == 3
    assert "base_link" in stats.link_objects
    assert "body_link" in stats.link_objects
    assert "gripper_link" in stats.link_objects


def test_get_robot_statistics_dof_calculation(scene) -> None:
    """Test get_robot_statistics correctly calculates DOF for different joint types."""
    parent_link = create_test_object("parent_link", None, scene)
    parent_link.location = (0, 0, 0)
    safe_get_linkforge(parent_link).is_robot_link = True
    safe_get_linkforge(parent_link).link_name = "parent_link"

    bpy.ops.mesh.primitive_cube_add(location=(1, 0, 0))
    child_link = bpy.context.active_object
    child_link.name = "child_link"
    child_link.linkforge.is_robot_link = True
    child_link.linkforge.link_name = "child_link"

    # REVOLUTE joint: 1 DOF
    joint1 = create_test_object("revolute_joint", None, scene)
    joint1.location = (0.5, 0, 0)
    safe_get_joint(joint1).is_robot_joint = True
    safe_get_joint(joint1).joint_name = "revolute_joint"
    safe_get_joint(joint1).joint_type = "REVOLUTE"
    safe_get_joint(joint1).parent_link = parent_link
    safe_get_joint(joint1).child_link = child_link

    # PRISMATIC joint: 1 DOF
    joint2 = create_test_object("prismatic_joint", None, scene)
    joint2.location = (1.5, 0, 0)
    safe_get_joint(joint2).is_robot_joint = True
    safe_get_joint(joint2).joint_name = "prismatic_joint"
    safe_get_joint(joint2).joint_type = "PRISMATIC"

    # PLANAR joint: 2 DOF
    joint3 = create_test_object("planar_joint", None, scene)
    joint3.location = (2.5, 0, 0)
    safe_get_joint(joint3).is_robot_joint = True
    safe_get_joint(joint3).joint_name = "planar_joint"
    safe_get_joint(joint3).joint_type = "PLANAR"

    # FIXED joint: 0 DOF
    joint4 = create_test_object("fixed_joint", None, scene)
    joint4.location = (3.5, 0, 0)
    safe_get_joint(joint4).is_robot_joint = True
    safe_get_joint(joint4).joint_name = "fixed_joint"
    safe_get_joint(joint4).joint_type = "FIXED"

    stats = get_robot_statistics(scene)

    assert stats.total_dof == 4  # 1 + 1 + 2 + 0
    assert len(stats.joint_objects) == 4


def test_get_robot_statistics_root_link_detection(scene) -> None:
    """Test get_robot_statistics correctly identifies root link."""
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
    base_link = bpy.context.active_object
    base_link.name = "base_link"
    base_link.linkforge.is_robot_link = True
    base_link.linkforge.link_name = "base_link"

    link1 = create_test_object("link1", None, scene)
    link1.location = (1, 0, 0)
    safe_get_linkforge(link1).is_robot_link = True
    safe_get_linkforge(link1).link_name = "link1"

    link2 = create_test_object("link2", None, scene)
    link2.location = (2, 0, 0)
    safe_get_linkforge(link2).is_robot_link = True
    safe_get_linkforge(link2).link_name = "link2"

    # base_link -> link1
    joint1 = create_test_object("joint1", None, scene)
    joint1.location = (0.5, 0, 0)
    safe_get_joint(joint1).is_robot_joint = True
    safe_get_joint(joint1).joint_name = "joint1"
    safe_get_joint(joint1).joint_type = "REVOLUTE"
    safe_get_joint(joint1).parent_link = base_link
    safe_get_joint(joint1).child_link = link1

    joint2 = create_test_object("joint2", None, scene)
    joint2.location = (1.5, 0, 0)
    safe_get_joint(joint2).is_robot_joint = True
    safe_get_joint(joint2).joint_name = "joint2"
    safe_get_joint(joint2).joint_type = "REVOLUTE"
    safe_get_joint(joint2).parent_link = link1
    safe_get_joint(joint2).child_link = link2

    stats = get_robot_statistics(scene)

    # root shld be base_link (not a child in any joint)
    assert stats.root_link is not None
    assert stats.root_link[0] == "base_link"
    assert stats.root_link[1] == base_link


def test_get_robot_statistics_with_sensors_and_transmissions(scene) -> None:
    """Test get_robot_statistics counts sensors and transmissions."""
    link = create_test_object("sensor_link", None, scene)
    safe_get_linkforge(link).is_robot_link = True
    safe_get_linkforge(link).link_name = "sensor_link"

    sensor = create_test_object("camera_sensor", None, scene)
    sensor.location = (1, 0, 0)
    safe_get_sensor(sensor).is_robot_sensor = True
    safe_get_sensor(sensor).sensor_name = "camera_sensor"
    safe_get_sensor(sensor).sensor_type = "CAMERA"

    transmission = create_test_object("transmission1", None, scene)
    transmission.location = (2, 0, 0)
    safe_get_transmission(transmission).is_robot_transmission = True
    safe_get_transmission(transmission).transmission_name = "transmission1"

    stats = get_robot_statistics(scene)

    assert stats.num_links == 1
    assert len(stats.sensor_objects) == 1
    assert len(stats.transmission_objects) == 1
    assert stats.sensor_objects[0] == sensor
    assert stats.transmission_objects[0] == transmission


def test_build_tree_from_stats_basic(scene) -> None:
    """Test build_tree_from_stats creates tree and joints mapping from stats."""
    base = create_test_object("base_link", None, scene)
    safe_get_linkforge(base).is_robot_link = True
    safe_get_linkforge(base).link_name = "base_link"

    child = create_test_object("child_link", None, scene)
    child.location = (1, 0, 0)
    safe_get_linkforge(child).is_robot_link = True
    safe_get_linkforge(child).link_name = "child_link"

    joint = create_test_object("joint1", None, scene)
    joint.location = (0.5, 0, 0)
    safe_get_joint(joint).is_robot_joint = True
    safe_get_joint(joint).joint_name = "joint1"
    safe_get_joint(joint).joint_type = "REVOLUTE"
    safe_get_joint(joint).parent_link = base
    safe_get_joint(joint).child_link = child

    stats = get_robot_statistics(scene)
    tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)

    assert root_link == "base_link"
    assert "base_link" in tree
    children = tree["base_link"]
    assert any(c[0] == "child_link" and c[1] == "joint1" and c[2] == "REVOLUTE" for c in children)
    assert ("base_link", "child_link") in joints_dict
    assert "base_link" in links_dict and "child_link" in links_dict


def test_build_tree_from_stats_single_link(scene) -> None:
    """Test build_tree_from_stats handles no joint scene."""
    only = create_test_object("only_link", None, scene)
    only.location = (2, 0, 0)
    safe_get_linkforge(only).is_robot_link = True
    safe_get_linkforge(only).link_name = "only_link"

    stats = get_robot_statistics(scene)
    tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)

    assert root_link == "only_link"
    assert "only_link" in tree
    assert tree["only_link"] == []
    assert joints_dict == {}


def test_build_tree_from_stats_parent_not_in_tree(scene) -> None:
    """If a joint refs a parent that is not a robot_link, it should be ignored."""
    # invalid parent
    parent_nonlink = create_test_object("maybe_parent", None, scene)
    parent_nonlink.location = (10, 0, 0)
    safe_get_linkforge(parent_nonlink).is_robot_link = False
    safe_get_linkforge(parent_nonlink).link_name = "maybe_parent"

    child = create_test_object("real_child", None, scene)
    child.location = (11, 0, 0)
    safe_get_linkforge(child).is_robot_link = True
    safe_get_linkforge(child).link_name = "real_child"

    joint = create_test_object("joint1", None, scene)
    joint.location = (10.5, 0, 0)
    safe_get_joint(joint).is_robot_joint = True
    safe_get_joint(joint).joint_name = "joint1"
    safe_get_joint(joint).joint_type = "REVOLUTE"
    safe_get_joint(joint).parent_link = parent_nonlink
    safe_get_joint(joint).child_link = child

    stats = get_robot_statistics(scene)
    tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)

    # parent w/o robot_link props should not be in tree
    assert "maybe_parent" not in tree
    # joint shld not be in joints_dict since parent is invalid
    assert ("maybe_parent", "real_child") not in joints_dict


def test_build_tree_from_stats_no_root_when_all_links_are_children(scene) -> None:
    """If every link appears as a child in joints_map, root_link should be None."""
    a = create_test_object("link_a", None, scene)
    a.location = (20, 0, 0)
    safe_get_linkforge(a).is_robot_link = True
    safe_get_linkforge(a).link_name = "link_a"

    b = create_test_object("link_b", None, scene)
    b.location = (21, 0, 0)
    safe_get_linkforge(b).is_robot_link = True
    safe_get_linkforge(b).link_name = "link_b"

    j1 = create_test_object("j1", None, scene)
    j1.location = (20.5, 0, 0)
    safe_get_joint(j1).is_robot_joint = True
    safe_get_joint(j1).joint_name = "j1"
    safe_get_joint(j1).joint_type = "REVOLUTE"
    safe_get_joint(j1).parent_link = b
    safe_get_joint(j1).child_link = a

    j2 = create_test_object("j2", None, scene)
    j2.location = (21.5, 0, 0)
    safe_get_joint(j2).is_robot_joint = True
    safe_get_joint(j2).joint_name = "j2"
    safe_get_joint(j2).joint_type = "REVOLUTE"
    safe_get_joint(j2).parent_link = a
    safe_get_joint(j2).child_link = b

    stats = get_robot_statistics(scene)
    tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)

    assert root_link is None


def test_get_robot_statistics_excludes_invalid_mass(scene) -> None:
    """Test that links with <0 mass do not add up to total_mass.
    If a link has invalid its still counted in num_links but its mass is
    ignored in total_mass, since its not a valid physical link."""
    link1 = create_test_object("valid_link", None, scene)
    link1.location = (0, 0, 0)
    safe_get_linkforge(link1).is_robot_link = True
    safe_get_linkforge(link1).link_name = "valid_link"
    safe_get_linkforge(link1).mass = 10.0

    link2 = create_test_object("zero_mass_link", None, scene)
    link2.location = (1, 0, 0)
    safe_get_linkforge(link2).is_robot_link = True
    safe_get_linkforge(link2).link_name = "zero_mass_link"
    safe_get_linkforge(link2).mass = 0.0

    link3 = create_test_object("negative_mass_link", None, scene)
    link3.location = (2, 0, 0)
    safe_get_linkforge(link3).is_robot_link = True
    safe_get_linkforge(link3).link_name = "negative_mass_link"
    safe_get_linkforge(link3).mass = -5.0

    stats = get_robot_statistics(scene)

    assert stats.num_links == 3
    assert "valid_link" in stats.link_objects
    assert "zero_mass_link" in stats.link_objects
    assert "negative_mass_link" in stats.link_objects

    assert stats.total_mass == 10.0  # 10 + 0 (ignored) + (-5)(ignored)


def test_get_robot_statistics_joint_with_none_parent(scene) -> None:
    """Test that joints with None parent_link are counted but not added to joints_map.

    If parent_link is None, the joint shld be counted but not create a parent-child relation in joints_map.
    """
    child_link = create_test_object("child_link", None, scene)
    child_link.location = (0, 0, 0)
    safe_get_linkforge(child_link).is_robot_link = True
    safe_get_linkforge(child_link).link_name = "child_link"
    safe_get_linkforge(child_link).mass = 5.0

    # invalid parent
    joint = create_test_object("world_joint", None, scene)
    joint.location = (0.5, 0, 0)
    safe_get_joint(joint).is_robot_joint = True
    safe_get_joint(joint).joint_name = "world_joint"
    safe_get_joint(joint).joint_type = "FIXED"
    safe_get_joint(joint).child_link = child_link
    safe_get_joint(joint).parent_link = None

    stats = get_robot_statistics(scene)

    # Joint shld be counted as existing
    assert len(stats.joint_objects) == 1
    assert stats.joint_objects[0] == joint

    # invalid parent -> shld not create a mapping in joints_map
    assert len(stats.joints_map) == 0

    assert stats.root_link is not None
    assert stats.root_link[0] == "child_link"
    assert stats.root_link[1] == child_link

    assert stats.num_links == 1
    assert stats.total_mass == 5.0


def test_get_robot_statistics_joint_with_empty_link_names(scene) -> None:
    """Test that joints with empty link_name strings are counted but not added to joints_map.

    If parent or child has an empty link_name, the joint should be counted
    but not create a relation in joints_map.
    """
    parent_link = create_test_object("parent_link", None, scene)
    parent_link.location = (0, 0, 0)
    safe_get_linkforge(parent_link).is_robot_link = True
    safe_get_linkforge(parent_link).link_name = ""  # no name link
    safe_get_linkforge(parent_link).mass = 10.0

    # child has valid link name
    child_link = create_test_object("child_link", None, scene)
    child_link.location = (1, 0, 0)
    safe_get_linkforge(child_link).is_robot_link = True
    safe_get_linkforge(child_link).link_name = "child_link"
    safe_get_linkforge(child_link).mass = 5.0

    joint1 = create_test_object("joint_empty_parent", None, scene)
    joint1.location = (0.5, 0, 0)
    safe_get_joint(joint1).is_robot_joint = True
    safe_get_joint(joint1).joint_name = "joint_empty_parent"
    safe_get_joint(joint1).joint_type = "REVOLUTE"
    safe_get_joint(joint1).parent_link = parent_link  # shld cause it to be ignored
    safe_get_joint(joint1).child_link = child_link

    # valid named parent
    valid_parent = create_test_object("valid_parent", None, scene)
    valid_parent.location = (2, 0, 0)
    safe_get_linkforge(valid_parent).is_robot_link = True
    safe_get_linkforge(valid_parent).link_name = "valid_parent"
    safe_get_linkforge(valid_parent).mass = 8.0

    # invalid child (empty link_name)
    empty_child = create_test_object("empty_child", None, scene)
    empty_child.location = (3, 0, 0)
    safe_get_linkforge(empty_child).is_robot_link = True
    safe_get_linkforge(empty_child).link_name = ""  # Empty string
    safe_get_linkforge(empty_child).mass = 3.0

    joint2 = create_test_object("joint_empty_child", None, scene)
    joint2.location = (2.5, 0, 0)
    safe_get_joint(joint2).is_robot_joint = True
    safe_get_joint(joint2).joint_name = "joint_empty_child"
    safe_get_joint(joint2).joint_type = "PRISMATIC"
    safe_get_joint(joint2).parent_link = valid_parent
    safe_get_joint(joint2).child_link = empty_child

    stats = get_robot_statistics(scene)

    assert len(stats.joint_objects) == 2
    assert joint1 in stats.joint_objects
    assert joint2 in stats.joint_objects

    # joints_map shld contain the mapping based on the object names coz link_name is empty
    assert len(stats.joints_map) == 2
    assert stats.joints_map.get("child_link")[0] == "parent_link"
    assert stats.joints_map.get("child_link")[1] == joint1
    # child has empty link_name -> keyed by its object name
    assert stats.joints_map.get("empty_child")[0] == "valid_parent"
    assert stats.joints_map.get("empty_child")[1] == joint2

    # links shld be counted even if they have empty link_name
    assert stats.num_links == 4
    assert stats.total_mass == 26.0  # 10 + 5 + 8 + 3

    # links shld be acc via their names (or object names if link_name is empty)
    assert "child_link" in stats.link_objects
    assert "valid_parent" in stats.link_objects
    assert "parent_link" in stats.link_objects
    assert "empty_child" in stats.link_objects
