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


def test_is_robot_link_with_valid_link():
    """Test is_robot_link returns True for valid robot link object."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    obj.linkforge.is_robot_link = True
    obj.linkforge.link_name = "test_link"

    assert is_robot_link(obj) is True


def test_is_robot_link_with_non_link():
    """Test is_robot_link returns False for non-link object."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    assert is_robot_link(obj) is False


def test_is_robot_link_with_none():
    """Test is_robot_link handles None input without throwing an error."""
    assert is_robot_link(None) is False


def test_is_robot_joint_with_valid_joint():
    """Test is_robot_joint returns True for valid robot_joint object."""
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    obj = bpy.context.active_object
    obj.linkforge_joint.is_robot_joint = True
    obj.linkforge_joint.joint_name = "test_joint"
    obj.linkforge_joint.joint_type = "REVOLUTE"

    assert is_robot_joint(obj) is True


def test_is_robot_joint_with_mesh_object():
    """Test is_robot_joint returns False for objects that cannot be robot_joints."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    assert is_robot_joint(obj) is False


def test_is_robot_joint_with_empty_not_marked():
    """Test is_robot_joint returns False for non robot_joint objects."""
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    obj = bpy.context.active_object

    assert is_robot_joint(obj) is False


def test_is_robot_sensor_with_valid_sensor():
    """Test is_robot_sensor returns True for valid robot_sensor object."""
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    obj = bpy.context.active_object
    obj.linkforge_sensor.is_robot_sensor = True
    obj.linkforge_sensor.sensor_name = "test_sensor"
    obj.linkforge_sensor.sensor_type = "CAMERA"

    assert is_robot_sensor(obj) is True


def test_is_robot_sensor_with_mesh_object():
    """Test is_robot_sensor returns False for objects that cannot be robot_sensors."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    assert is_robot_sensor(obj) is False


def test_is_robot_sensor_with_empty_not_marked():
    """Test is_robot_sensor returns False for non robot_sensor objects."""
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    obj = bpy.context.active_object

    assert is_robot_sensor(obj) is False


def test_is_robot_transmission_with_valid_transmission():
    """Test is_robot_transmission returns True for valid transmission."""
    # Create an empty and mark it as a transmission
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    obj = bpy.context.active_object
    obj.linkforge_transmission.is_robot_transmission = True
    obj.linkforge_transmission.transmission_name = "test_transmission"

    assert is_robot_transmission(obj) is True


def test_is_robot_transmission_with_unmarked_object():
    """Test is_robot_transmission returns False for unmarked object."""
    # Create an object but don't mark it as a transmission
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    assert is_robot_transmission(obj) is False


def test_move_to_collection_basic():
    """Test moving an object to a new collection."""
    # Clean scene
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create a cube
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Create a new collection
    target_collection = bpy.data.collections.new("TestCollection")
    bpy.context.scene.collection.children.link(target_collection)

    # Move object to new collection
    move_to_collection(obj, target_collection)

    # Verify object is in target collection
    assert obj in target_collection.objects[:]
    # Verify object is not in scene root collection
    assert obj not in bpy.context.scene.collection.objects[:]


def test_move_to_collection_already_in_target():
    """Test moving an object to its current collection."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create collection and object
    target_collection = bpy.data.collections.new("TestCollection")
    bpy.context.scene.collection.children.link(target_collection)

    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Move to target once
    move_to_collection(obj, target_collection)

    # Move again (should be idempotent)
    move_to_collection(obj, target_collection)

    # Should still be in target collection only once
    assert obj in target_collection.objects[:]
    assert list(obj.users_collection) == [target_collection]


def test_move_to_collection_none_object():
    """Test with None object."""
    target_collection = bpy.data.collections.new("TestCollection")
    bpy.context.scene.collection.children.link(target_collection)

    # Should not raise error
    move_to_collection(None, target_collection)


def test_move_to_collection_none_collection():
    """Test with None collection."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Should not raise error
    move_to_collection(obj, None)


def test_move_to_collection_both_none():
    """Test with both None."""
    move_to_collection(None, None)
    # Should not raise error


def test_move_to_collection_multiple_collections():
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

    bpy.context.scene.collection.children.link(coll1)
    bpy.context.scene.collection.children.link(coll2)
    bpy.context.scene.collection.children.link(target_coll)

    coll1.objects.link(obj)
    coll2.objects.link(obj)

    # Move to target
    move_to_collection(obj, target_coll)

    # Should only be in target collection now
    assert obj in target_coll.objects[:]
    assert obj not in coll1.objects[:]
    assert obj not in coll2.objects[:]


def test_get_robot_statistics_empty_scene():
    """Test get_robot_statistics with empty scene returns zeros."""
    stats = get_robot_statistics(bpy.context.scene)

    assert stats.num_links == 0
    assert stats.total_mass == 0.0
    assert stats.total_dof == 0
    assert len(stats.link_objects) == 0
    assert len(stats.joint_objects) == 0
    assert len(stats.sensor_objects) == 0
    assert len(stats.transmission_objects) == 0
    assert stats.root_link is None


def test_get_robot_statistics_none_scene():
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


def test_get_robot_statistics_with_links():
    """Test get_robot_statistics counts links and calculates mass."""
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
    link1 = bpy.context.active_object
    link1.name = "base_link"
    link1.linkforge.is_robot_link = True
    link1.linkforge.link_name = "base_link"
    link1.linkforge.mass = 5.0

    bpy.ops.mesh.primitive_cube_add(location=(1, 0, 0))
    link2 = bpy.context.active_object
    link2.name = "body_link"
    link2.linkforge.is_robot_link = True
    link2.linkforge.link_name = "body_link"
    link2.linkforge.mass = 10.0

    bpy.ops.mesh.primitive_cube_add(location=(2, 0, 0))
    link3 = bpy.context.active_object
    link3.name = "gripper_link"
    link3.linkforge.is_robot_link = True
    link3.linkforge.link_name = "gripper_link"
    link3.linkforge.mass = 2.5

    stats = get_robot_statistics(bpy.context.scene)

    assert stats.num_links == 3
    assert stats.total_mass == 17.5  # 5.0 + 10.0 + 2.5
    assert stats.total_dof == 0  # no joints yet
    assert len(stats.link_objects) == 3
    assert "base_link" in stats.link_objects
    assert "body_link" in stats.link_objects
    assert "gripper_link" in stats.link_objects


def test_get_robot_statistics_dof_calculation():
    """Test get_robot_statistics correctly calculates DOF for different joint types."""
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
    parent_link = bpy.context.active_object
    parent_link.name = "parent_link"
    parent_link.linkforge.is_robot_link = True
    parent_link.linkforge.link_name = "parent_link"

    bpy.ops.mesh.primitive_cube_add(location=(1, 0, 0))
    child_link = bpy.context.active_object
    child_link.name = "child_link"
    child_link.linkforge.is_robot_link = True
    child_link.linkforge.link_name = "child_link"

    # REVOLUTE joint: 1 DOF
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0.5, 0, 0))
    joint1 = bpy.context.active_object
    joint1.linkforge_joint.is_robot_joint = True
    joint1.linkforge_joint.joint_name = "revolute_joint"
    joint1.linkforge_joint.joint_type = "REVOLUTE"
    joint1.linkforge_joint.parent_link = parent_link
    joint1.linkforge_joint.child_link = child_link

    # PRISMATIC joint: 1 DOF
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(1.5, 0, 0))
    joint2 = bpy.context.active_object
    joint2.linkforge_joint.is_robot_joint = True
    joint2.linkforge_joint.joint_name = "prismatic_joint"
    joint2.linkforge_joint.joint_type = "PRISMATIC"

    # PLANAR joint: 2 DOF
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(2.5, 0, 0))
    joint3 = bpy.context.active_object
    joint3.linkforge_joint.is_robot_joint = True
    joint3.linkforge_joint.joint_name = "planar_joint"
    joint3.linkforge_joint.joint_type = "PLANAR"

    # FIXED joint: 0 DOF
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(3.5, 0, 0))
    joint4 = bpy.context.active_object
    joint4.linkforge_joint.is_robot_joint = True
    joint4.linkforge_joint.joint_name = "fixed_joint"
    joint4.linkforge_joint.joint_type = "FIXED"

    stats = get_robot_statistics(bpy.context.scene)

    assert stats.total_dof == 4  # 1 + 1 + 2 + 0
    assert len(stats.joint_objects) == 4


def test_get_robot_statistics_root_link_detection():
    """Test get_robot_statistics correctly identifies root link."""
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
    base_link = bpy.context.active_object
    base_link.name = "base_link"
    base_link.linkforge.is_robot_link = True
    base_link.linkforge.link_name = "base_link"

    bpy.ops.mesh.primitive_cube_add(location=(1, 0, 0))
    link1 = bpy.context.active_object
    link1.name = "link1"
    link1.linkforge.is_robot_link = True
    link1.linkforge.link_name = "link1"

    bpy.ops.mesh.primitive_cube_add(location=(2, 0, 0))
    link2 = bpy.context.active_object
    link2.name = "link2"
    link2.linkforge.is_robot_link = True
    link2.linkforge.link_name = "link2"

    # base_link -> link1
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0.5, 0, 0))
    joint1 = bpy.context.active_object
    joint1.linkforge_joint.is_robot_joint = True
    joint1.linkforge_joint.joint_name = "joint1"
    joint1.linkforge_joint.joint_type = "REVOLUTE"
    joint1.linkforge_joint.parent_link = base_link
    joint1.linkforge_joint.child_link = link1

    # link1 -> link2
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(1.5, 0, 0))
    joint2 = bpy.context.active_object
    joint2.linkforge_joint.is_robot_joint = True
    joint2.linkforge_joint.joint_name = "joint2"
    joint2.linkforge_joint.joint_type = "REVOLUTE"
    joint2.linkforge_joint.parent_link = link1
    joint2.linkforge_joint.child_link = link2

    stats = get_robot_statistics(bpy.context.scene)

    # root shld be base_link (not a child in any joint)
    assert stats.root_link is not None
    assert stats.root_link[0] == "base_link"
    assert stats.root_link[1] == base_link


def test_get_robot_statistics_with_sensors_and_transmissions():
    """Test get_robot_statistics counts sensors and transmissions."""
    bpy.ops.mesh.primitive_cube_add()
    link = bpy.context.active_object
    link.linkforge.is_robot_link = True
    link.linkforge.link_name = "sensor_link"

    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(1, 0, 0))
    sensor = bpy.context.active_object
    sensor.linkforge_sensor.is_robot_sensor = True
    sensor.linkforge_sensor.sensor_name = "camera_sensor"
    sensor.linkforge_sensor.sensor_type = "CAMERA"

    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(2, 0, 0))
    transmission = bpy.context.active_object
    transmission.linkforge_transmission.is_robot_transmission = True
    transmission.linkforge_transmission.transmission_name = "transmission1"

    stats = get_robot_statistics(bpy.context.scene)

    assert stats.num_links == 1
    assert len(stats.sensor_objects) == 1
    assert len(stats.transmission_objects) == 1
    assert stats.sensor_objects[0] == sensor
    assert stats.transmission_objects[0] == transmission


def test_build_tree_from_stats_basic():
    """Test build_tree_from_stats creates tree and joints mapping from stats."""
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
    base = bpy.context.active_object
    base.name = "base_link"
    base.linkforge.is_robot_link = True
    base.linkforge.link_name = "base_link"

    bpy.ops.mesh.primitive_cube_add(location=(1, 0, 0))
    child = bpy.context.active_object
    child.name = "child_link"
    child.linkforge.is_robot_link = True
    child.linkforge.link_name = "child_link"

    # parent=base_link, child=child_link
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0.5, 0, 0))
    joint = bpy.context.active_object
    joint.linkforge_joint.is_robot_joint = True
    joint.linkforge_joint.joint_name = "joint1"
    joint.linkforge_joint.joint_type = "REVOLUTE"
    joint.linkforge_joint.parent_link = base
    joint.linkforge_joint.child_link = child

    stats = get_robot_statistics(bpy.context.scene)
    tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)

    assert root_link == "base_link"
    assert "base_link" in tree
    children = tree["base_link"]
    assert any(c[0] == "child_link" and c[1] == "joint1" and c[2] == "REVOLUTE" for c in children)
    assert ("base_link", "child_link") in joints_dict
    assert "base_link" in links_dict and "child_link" in links_dict


def test_build_tree_from_stats_single_link():
    """Test build_tree_from_stats handles no joint scene."""
    bpy.ops.mesh.primitive_cube_add(location=(2, 0, 0))
    only = bpy.context.active_object
    only.name = "only_link"
    only.linkforge.is_robot_link = True
    only.linkforge.link_name = "only_link"

    stats = get_robot_statistics(bpy.context.scene)
    tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)

    assert root_link == "only_link"
    assert "only_link" in tree
    assert tree["only_link"] == []
    assert joints_dict == {}


def test_build_tree_from_stats_parent_not_in_tree():
    """If a joint refs a parent that is not a robot_link, it should be ignored."""
    # invalid parent
    bpy.ops.mesh.primitive_cube_add(location=(10, 0, 0))
    parent_nonlink = bpy.context.active_object
    parent_nonlink.name = "maybe_parent"
    parent_nonlink.linkforge.is_robot_link = False
    parent_nonlink.linkforge.link_name = "maybe_parent"

    # valid child
    bpy.ops.mesh.primitive_cube_add(location=(11, 0, 0))
    child = bpy.context.active_object
    child.name = "real_child"
    child.linkforge.is_robot_link = True
    child.linkforge.link_name = "real_child"

    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(10.5, 0, 0))
    joint = bpy.context.active_object
    joint.linkforge_joint.is_robot_joint = True
    joint.linkforge_joint.joint_name = "joint1"
    joint.linkforge_joint.joint_type = "REVOLUTE"
    joint.linkforge_joint.parent_link = parent_nonlink
    joint.linkforge_joint.child_link = child

    stats = get_robot_statistics(bpy.context.scene)
    tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)

    # parent w/o robot_link props should not be in tree
    assert "maybe_parent" not in tree
    # joint shld not be in joints_dict since parent is invalid
    assert ("maybe_parent", "real_child") not in joints_dict


def test_build_tree_from_stats_no_root_when_all_links_are_children():
    """If every link appears as a child in joints_map, root_link should be None."""
    bpy.ops.mesh.primitive_cube_add(location=(20, 0, 0))
    a = bpy.context.active_object
    a.name = "link_a"
    a.linkforge.is_robot_link = True
    a.linkforge.link_name = "link_a"

    bpy.ops.mesh.primitive_cube_add(location=(21, 0, 0))
    b = bpy.context.active_object
    b.name = "link_b"
    b.linkforge.is_robot_link = True
    b.linkforge.link_name = "link_b"

    # a is child of b
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(20.5, 0, 0))
    j1 = bpy.context.active_object
    j1.linkforge_joint.is_robot_joint = True
    j1.linkforge_joint.joint_name = "j1"
    j1.linkforge_joint.joint_type = "REVOLUTE"
    j1.linkforge_joint.parent_link = b
    j1.linkforge_joint.child_link = a

    # b is child of a (=cycle)
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(21.5, 0, 0))
    j2 = bpy.context.active_object
    j2.linkforge_joint.is_robot_joint = True
    j2.linkforge_joint.joint_name = "j2"
    j2.linkforge_joint.joint_type = "REVOLUTE"
    j2.linkforge_joint.parent_link = a
    j2.linkforge_joint.child_link = b

    stats = get_robot_statistics(bpy.context.scene)
    tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)

    assert root_link is None


def test_get_robot_statistics_excludes_invalid_mass():
    """Test that links with <0 mass do not add up to total_mass.
    If a link has invalid its still counted in num_links but its mass is
    ignored in total_mass, since its not a valid physical link."""
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
    link1 = bpy.context.active_object
    link1.name = "valid_link"
    link1.linkforge.is_robot_link = True
    link1.linkforge.link_name = "valid_link"
    link1.linkforge.mass = 10.0

    bpy.ops.mesh.primitive_cube_add(location=(1, 0, 0))
    link2 = bpy.context.active_object
    link2.name = "zero_mass_link"
    link2.linkforge.is_robot_link = True
    link2.linkforge.link_name = "zero_mass_link"
    link2.linkforge.mass = 0.0

    bpy.ops.mesh.primitive_cube_add(location=(2, 0, 0))
    link3 = bpy.context.active_object
    link3.name = "negative_mass_link"
    link3.linkforge.is_robot_link = True
    link3.linkforge.link_name = "negative_mass_link"
    link3.linkforge.mass = -5.0

    stats = get_robot_statistics(bpy.context.scene)

    assert stats.num_links == 3
    assert "valid_link" in stats.link_objects
    assert "zero_mass_link" in stats.link_objects
    assert "negative_mass_link" in stats.link_objects

    assert stats.total_mass == 10.0  # 10 + 0 (ignored) + (-5)(ignored)


def test_get_robot_statistics_joint_with_none_parent():
    """Test that joints with None parent_link are counted but not added to joints_map.

    If parent_link is None, the joint shld be counted but not create a parent-child relation in joints_map.
    """
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
    child_link = bpy.context.active_object
    child_link.name = "child_link"
    child_link.linkforge.is_robot_link = True
    child_link.linkforge.link_name = "child_link"
    child_link.linkforge.mass = 5.0

    # invalid parent
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0.5, 0, 0))
    joint = bpy.context.active_object
    joint.linkforge_joint.is_robot_joint = True
    joint.linkforge_joint.joint_name = "world_joint"
    joint.linkforge_joint.joint_type = "FIXED"
    joint.linkforge_joint.child_link = child_link
    joint.linkforge_joint.parent_link = None

    stats = get_robot_statistics(bpy.context.scene)

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


def test_get_robot_statistics_joint_with_empty_link_names():
    """Test that joints with empty link_name strings are counted but not added to joints_map.

    If parent or child has an empty link_name, the joint should be counted
    but not create a relation in joints_map.
    """
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
    parent_link = bpy.context.active_object
    parent_link.name = "parent_link"
    parent_link.linkforge.is_robot_link = True
    parent_link.linkforge.link_name = ""  # no name link
    parent_link.linkforge.mass = 10.0

    # child has valid link name
    bpy.ops.mesh.primitive_cube_add(location=(1, 0, 0))
    child_link = bpy.context.active_object
    child_link.name = "child_link"
    child_link.linkforge.is_robot_link = True
    child_link.linkforge.link_name = "child_link"
    child_link.linkforge.mass = 5.0

    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0.5, 0, 0))
    joint1 = bpy.context.active_object
    joint1.linkforge_joint.is_robot_joint = True
    joint1.linkforge_joint.joint_name = "joint_empty_parent"
    joint1.linkforge_joint.joint_type = "REVOLUTE"
    joint1.linkforge_joint.parent_link = parent_link  # shld cause it to be ignored
    joint1.linkforge_joint.child_link = child_link

    # valid named parent
    bpy.ops.mesh.primitive_cube_add(location=(2, 0, 0))
    valid_parent = bpy.context.active_object
    valid_parent.name = "valid_parent"
    valid_parent.linkforge.is_robot_link = True
    valid_parent.linkforge.link_name = "valid_parent"
    valid_parent.linkforge.mass = 8.0

    # invalid child (empty link_name)
    bpy.ops.mesh.primitive_cube_add(location=(3, 0, 0))
    empty_child = bpy.context.active_object
    empty_child.name = "empty_child"
    empty_child.linkforge.is_robot_link = True
    empty_child.linkforge.link_name = ""  # Empty string
    empty_child.linkforge.mass = 3.0

    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(2.5, 0, 0))
    joint2 = bpy.context.active_object
    joint2.linkforge_joint.is_robot_joint = True
    joint2.linkforge_joint.joint_name = "joint_empty_child"
    joint2.linkforge_joint.joint_type = "PRISMATIC"
    joint2.linkforge_joint.parent_link = valid_parent
    joint2.linkforge_joint.child_link = empty_child

    stats = get_robot_statistics(bpy.context.scene)

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
