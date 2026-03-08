"""Tests for scene utility functions."""

from __future__ import annotations

import bpy
from linkforge.blender.utils.scene_utils import (
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
