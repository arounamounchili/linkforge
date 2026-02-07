"""Tests for joint_ops module."""

import bpy


def test_create_joint_from_link():
    """Test creating a joint from a selected link."""
    # Clean scene
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create a link
    bpy.ops.object.empty_add(location=(1, 2, 3))
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.link_name = "test_link"

    # Create joint
    link_obj.select_set(True)
    bpy.ops.linkforge.create_joint()

    # Verify joint was created
    joint_obj = bpy.context.active_object
    assert joint_obj != link_obj
    assert joint_obj.linkforge_joint.is_robot_joint
    # Note: Joints are not necessarily parented to links


def test_create_joint_poll_no_selection():
    """Test that poll fails with no selection."""
    bpy.ops.object.select_all(action="DESELECT")

    from linkforge.blender.operators.joint_ops import LINKFORGE_OT_create_joint

    assert not LINKFORGE_OT_create_joint.poll(bpy.context)


def test_create_joint_poll_non_link():
    """Test that poll fails when non-link is selected."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create non-link object
    bpy.ops.mesh.primitive_cube_add()
    cube = bpy.context.active_object
    cube.select_set(True)

    from linkforge.blender.operators.joint_ops import LINKFORGE_OT_create_joint

    assert not LINKFORGE_OT_create_joint.poll(bpy.context)


def test_delete_joint():
    """Test deleting a joint."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create link and joint
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True

    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.linkforge_joint.is_robot_joint = True
    joint_obj.linkforge_joint.joint_name = "test_joint"

    # Store name before deletion
    joint_name = joint_obj.name

    # Delete joint
    joint_obj.select_set(True)
    bpy.ops.linkforge.delete_joint()

    # Verify joint was deleted
    assert joint_name not in bpy.data.objects


def test_delete_joint_poll_no_selection():
    """Test that delete poll fails with no selection."""
    bpy.ops.object.select_all(action="DESELECT")

    from linkforge.blender.operators.joint_ops import LINKFORGE_OT_delete_joint

    assert not LINKFORGE_OT_delete_joint.poll(bpy.context)


def test_delete_joint_poll_non_joint():
    """Test that delete poll fails when non-joint is selected."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.select_set(True)

    from linkforge.blender.operators.joint_ops import LINKFORGE_OT_delete_joint

    assert not LINKFORGE_OT_delete_joint.poll(bpy.context)


def test_auto_detect_parent_child():
    """Test auto-detecting parent and child links based on distance."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create two links at different distances
    bpy.ops.object.empty_add(location=(0, 0, 0))
    link_a = bpy.context.active_object
    link_a.linkforge.is_robot_link = True
    link_a.linkforge.link_name = "link_a"

    bpy.ops.object.empty_add(location=(1, 0, 0))
    link_b = bpy.context.active_object
    link_b.linkforge.is_robot_link = True
    link_b.linkforge.link_name = "link_b"

    # Create joint closer to link_a (at 0.1, 0, 0)
    bpy.ops.object.empty_add(location=(0.1, 0, 0))
    joint_obj = bpy.context.active_object
    joint_obj.linkforge_joint.is_robot_joint = True

    # Auto-detect - should find both links
    joint_obj.select_set(True)
    bpy.ops.linkforge.auto_detect_parent_child()

    # Verify links were detected (closest is child, second is parent)
    assert joint_obj.linkforge_joint.child_link == link_a  # Closest
    assert joint_obj.linkforge_joint.parent_link == link_b  # Second closest


def test_auto_detect_poll_no_selection():
    """Test that auto-detect poll fails with no selection."""
    bpy.ops.object.select_all(action="DESELECT")

    from linkforge.blender.operators.joint_ops import LINKFORGE_OT_auto_detect_parent_child

    assert not LINKFORGE_OT_auto_detect_parent_child.poll(bpy.context)


def test_auto_detect_poll_non_joint():
    """Test that auto-detect poll fails for non-joint."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.select_set(True)

    from linkforge.blender.operators.joint_ops import LINKFORGE_OT_auto_detect_parent_child

    assert not LINKFORGE_OT_auto_detect_parent_child.poll(bpy.context)


def test_create_joint_naming():
    """Test that joint names are properly generated."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.link_name = "base"

    link_obj.select_set(True)
    bpy.ops.linkforge.create_joint()

    joint_obj = bpy.context.active_object
    assert "base_joint" in joint_obj.name.lower()
