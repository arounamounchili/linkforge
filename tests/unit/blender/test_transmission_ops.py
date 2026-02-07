"""Tests for transmission_ops module."""

import bpy


def test_create_transmission_from_joint():
    """Test creating a transmission from a selected joint."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create a joint
    bpy.ops.object.empty_add(location=(1, 2, 3))
    joint_obj = bpy.context.active_object
    joint_obj.linkforge_joint.is_robot_joint = True
    joint_obj.linkforge_joint.joint_name = "test_joint"

    # Create transmission
    joint_obj.select_set(True)
    bpy.ops.linkforge.create_transmission()

    # Verify transmission was created
    trans_obj = bpy.context.active_object
    assert trans_obj != joint_obj
    assert trans_obj.linkforge_transmission.is_robot_transmission
    assert trans_obj.parent == joint_obj


def test_create_transmission_poll_no_selection():
    """Test that poll fails with no selection."""
    bpy.ops.object.select_all(action="DESELECT")

    from linkforge.blender.operators.transmission_ops import LINKFORGE_OT_create_transmission

    assert not LINKFORGE_OT_create_transmission.poll(bpy.context)


def test_create_transmission_poll_non_joint():
    """Test that poll fails when non-joint is selected."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.select_set(True)

    from linkforge.blender.operators.transmission_ops import LINKFORGE_OT_create_transmission

    assert not LINKFORGE_OT_create_transmission.poll(bpy.context)


def test_create_transmission_default_type():
    """Test that transmission is created with default SIMPLE type."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.linkforge_joint.is_robot_joint = True
    joint_obj.linkforge_joint.joint_name = "revolute_joint"

    joint_obj.select_set(True)
    bpy.ops.linkforge.create_transmission()

    trans_obj = bpy.context.active_object
    assert trans_obj.linkforge_transmission.transmission_type == "SIMPLE"


def test_create_transmission_joint_link():
    """Test that transmission is linked to the joint."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.linkforge_joint.is_robot_joint = True
    joint_obj.linkforge_joint.joint_name = "test_joint"

    joint_obj.select_set(True)
    bpy.ops.linkforge.create_transmission()

    trans_obj = bpy.context.active_object
    assert trans_obj.linkforge_transmission.joint_name == joint_obj


def test_delete_transmission():
    """Test deleting a transmission."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create transmission
    bpy.ops.object.empty_add()
    trans_obj = bpy.context.active_object
    trans_obj.linkforge_transmission.is_robot_transmission = True
    trans_obj.linkforge_transmission.transmission_name = "test_trans"

    # Store name before deletion
    trans_name = trans_obj.name

    # Delete transmission
    trans_obj.select_set(True)
    bpy.ops.linkforge.delete_transmission()

    # Verify transmission was deleted
    assert trans_name not in bpy.data.objects


def test_delete_transmission_poll_no_selection():
    """Test that delete poll fails with no selection."""
    bpy.ops.object.select_all(action="DESELECT")

    from linkforge.blender.operators.transmission_ops import LINKFORGE_OT_delete_transmission

    assert not LINKFORGE_OT_delete_transmission.poll(bpy.context)


def test_delete_transmission_poll_non_transmission():
    """Test that delete poll fails for non-transmission objects."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.select_set(True)

    from linkforge.blender.operators.transmission_ops import LINKFORGE_OT_delete_transmission

    assert not LINKFORGE_OT_delete_transmission.poll(bpy.context)


def test_transmission_naming():
    """Test that transmission names are properly generated."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.linkforge_joint.is_robot_joint = True
    joint_obj.linkforge_joint.joint_name = "wheel_joint"

    joint_obj.select_set(True)
    bpy.ops.linkforge.create_transmission()

    trans_obj = bpy.context.active_object
    assert "wheel_joint_trans" in trans_obj.name.lower()


def test_transmission_location_at_joint():
    """Test that transmission is created at joint's origin (0,0,0 relative)."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add(location=(5, 10, 15))
    joint_obj = bpy.context.active_object
    joint_obj.linkforge_joint.is_robot_joint = True

    joint_obj.select_set(True)
    bpy.ops.linkforge.create_transmission()

    trans_obj = bpy.context.active_object
    # Should be at 0,0,0 relative to parent joint
    assert abs(trans_obj.location[0]) < 0.001
    assert abs(trans_obj.location[1]) < 0.001
    assert abs(trans_obj.location[2]) < 0.001
