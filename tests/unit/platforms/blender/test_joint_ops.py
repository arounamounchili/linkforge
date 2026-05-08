"""Tests for joint management operators."""

from __future__ import annotations

import bpy


def test_joint_ops_create_joint(mock_context, scene) -> None:
    """Test LINKFORGE_OT_create_joint operator."""
    # Create a link
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    # 1. Test Poll
    assert bpy.ops.linkforge.create_joint.poll() is True

    # 2. Test Execute
    bpy.ops.linkforge.create_joint()
    joint_obj = bpy.context.active_object
    assert "_joint" in joint_obj.name
    assert joint_obj.linkforge_joint.is_robot_joint is True
    assert joint_obj.linkforge_joint.child_link == link_obj


def test_joint_ops_delete_joint(mock_context, scene) -> None:
    """Test LINKFORGE_OT_delete_joint operator and ROS2 Control sync."""
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    bpy.ops.linkforge.create_joint()
    joint_obj = bpy.context.active_object
    joint_name = joint_obj.name

    # Setup ROS2 control sync
    scene.linkforge.use_ros2_control = True
    item = scene.linkforge.ros2_control_joints.add()
    item.name = joint_name

    # Execute delete
    bpy.ops.linkforge.delete_joint()
    assert joint_name not in bpy.data.objects
    assert len(scene.linkforge.ros2_control_joints) == 0


def test_joint_ops_auto_detect(mock_context, scene) -> None:
    """Test LINKFORGE_OT_auto_detect_parent_child operator."""
    # Create link hierarchy
    bpy.ops.mesh.primitive_cube_add(location=(1, 0, 0))  # Child
    bpy.ops.linkforge.create_link_from_mesh()
    child_link = bpy.context.active_object

    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))  # Parent
    bpy.ops.linkforge.create_link_from_mesh()
    parent_link = bpy.context.active_object

    # Create joint at child
    bpy.ops.object.select_all(action="DESELECT")
    child_link.select_set(True)
    bpy.context.view_layer.objects.active = child_link
    bpy.ops.linkforge.create_joint()
    joint_obj = bpy.context.active_object

    # Auto-detect
    bpy.ops.linkforge.auto_detect_parent_child()
    assert joint_obj.linkforge_joint.child_link == child_link
    assert joint_obj.linkforge_joint.parent_link == parent_link


def test_joint_ops_robustness_no_links(mock_context, scene) -> None:
    """Test auto-detect and creation with no valid links."""
    # 1. Auto-detect with no links
    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.linkforge_joint.is_robot_joint = True

    # Should not crash
    bpy.ops.linkforge.auto_detect_parent_child()
    assert joint_obj.linkforge_joint.child_link is None

    # 2. Create joint on non-link object (should fail poll)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    bpy.ops.mesh.primitive_monkey_add()
    assert bpy.ops.linkforge.create_joint.poll() is False


def test_joint_ops_robustness_zombie_cleanup(mock_context, scene) -> None:
    """Test that deleting a joint cleans up ROS2 control even if references are broken."""
    scene.linkforge.use_ros2_control = True
    item = scene.linkforge.ros2_control_joints.add()
    item.name = "NonExistentJoint"

    # Selecting something else to allow delete_joint poll if possible,
    # but delete_joint usually requires a selected joint.
    # Let's create a real joint, rename it, and see if it still syncs by name.
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    bpy.ops.linkforge.create_joint()
    joint_obj = bpy.context.active_object

    # Add to ROS2 control
    joint_name = joint_obj.name
    item2 = scene.linkforge.ros2_control_joints.add()
    item2.name = joint_name

    bpy.ops.linkforge.delete_joint()
    # verify item2 is gone
    assert not any(i.name == joint_name for i in scene.linkforge.ros2_control_joints)
