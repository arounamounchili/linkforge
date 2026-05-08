import bpy

from tests.blender_test_utils import (
    create_mesh_object,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge_scene,
)


def test_joint_ops_create_joint(mock_context, scene) -> None:
    """Test LINKFORGE_OT_create_joint operator."""
    # Create a link
    obj = create_mesh_object("link", scene)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    # Test Poll
    assert bpy.ops.linkforge.create_joint.poll() is True

    # Test Execute
    bpy.ops.linkforge.create_joint()
    joint_obj = bpy.context.active_object
    assert "_joint" in joint_obj.name
    assert safe_get_joint(joint_obj).is_robot_joint is True
    assert safe_get_joint(joint_obj).child_link == link_obj


def test_joint_ops_delete_joint(mock_context, scene) -> None:
    """Test LINKFORGE_OT_delete_joint operator and ROS2 Control sync."""
    obj = create_mesh_object("link_del", scene)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.linkforge.create_link_from_mesh()
    bpy.ops.linkforge.create_joint()
    joint_obj = bpy.context.active_object
    joint_name = joint_obj.name

    # Setup ROS2 control sync
    lf_scene = safe_get_linkforge_scene(scene)
    lf_scene.use_ros2_control = True
    item = lf_scene.ros2_control_joints.add()
    item.name = joint_name

    # Execute delete
    bpy.ops.linkforge.delete_joint()
    assert joint_name not in bpy.data.objects
    assert len(scene.linkforge.ros2_control_joints) == 0


def test_joint_ops_auto_detect(mock_context, scene) -> None:
    """Test LINKFORGE_OT_auto_detect_parent_child operator."""
    # Create link hierarchy
    child_link_mesh = create_mesh_object("child", scene)
    child_link_mesh.location = (1, 0, 0)
    bpy.context.view_layer.objects.active = child_link_mesh
    child_link_mesh.select_set(True)
    bpy.ops.linkforge.create_link_from_mesh()
    child_link = bpy.context.active_object  # The newly created Empty

    parent_link_mesh = create_mesh_object("parent", scene)
    parent_link_mesh.location = (0, 0, 0)
    bpy.context.view_layer.objects.active = parent_link_mesh
    parent_link_mesh.select_set(True)
    bpy.ops.linkforge.create_link_from_mesh()
    parent_link = bpy.context.active_object  # The newly created Empty

    # Create joint at child
    bpy.ops.object.select_all(action="DESELECT")
    child_link.select_set(True)
    bpy.context.view_layer.objects.active = child_link
    bpy.ops.linkforge.create_joint()
    joint_obj = bpy.context.active_object

    # Auto-detect
    bpy.ops.linkforge.auto_detect_parent_child()
    assert safe_get_joint(joint_obj).child_link == child_link
    assert safe_get_joint(joint_obj).parent_link == parent_link


def test_joint_ops_robustness_no_links(mock_context, scene) -> None:
    """Test auto-detect and creation with no valid links."""
    # Auto-detect with no links
    joint_obj = create_test_object("joint_lone", None, scene)
    bpy.context.view_layer.objects.active = joint_obj
    joint_obj.select_set(True)
    safe_get_joint(joint_obj).is_robot_joint = True

    # Should not crash
    bpy.ops.linkforge.auto_detect_parent_child()
    assert safe_get_joint(joint_obj).child_link is None

    # Create joint on non-link object (should fail poll)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    bpy.ops.mesh.primitive_monkey_add()
    assert bpy.ops.linkforge.create_joint.poll() is False


def test_joint_ops_robustness_zombie_cleanup(mock_context, scene) -> None:
    """Test that deleting a joint cleans up ROS2 control even if references are broken."""
    lf_scene = safe_get_linkforge_scene(scene)
    lf_scene.use_ros2_control = True
    item = lf_scene.ros2_control_joints.add()
    item.name = "NonExistentJoint"

    # Selecting something else to allow delete_joint poll if possible,
    # but delete_joint usually requires a selected joint.
    # Let's create a real joint, rename it, and see if it still syncs by name.
    obj = create_mesh_object("zombie_link", scene)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.linkforge.create_link_from_mesh()
    bpy.ops.linkforge.create_joint()
    joint_obj = bpy.context.active_object

    # Add to ROS2 control
    joint_name = joint_obj.name
    item2 = lf_scene.ros2_control_joints.add()
    item2.name = joint_name

    bpy.ops.linkforge.delete_joint()
    # verify item2 is gone
    assert not any(i.name == joint_name for i in lf_scene.ros2_control_joints)
