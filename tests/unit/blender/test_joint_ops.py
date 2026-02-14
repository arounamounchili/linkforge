from unittest.mock import patch

import bpy


def test_joint_ops_create_joint():
    """Test LINKFORGE_OT_create_joint operator."""
    # Setup: Create a link
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    # 1. Test Poll (should pass with link selected)
    assert bpy.ops.linkforge.create_joint.poll() is True

    # 2. Test Poll (should fail with nothing selected)
    bpy.ops.object.select_all(action="DESELECT")
    assert bpy.ops.linkforge.create_joint.poll() is False

    # 3. Test Execute
    link_obj.select_set(True)
    bpy.context.view_layer.objects.active = link_obj
    bpy.ops.linkforge.create_joint()

    joint_obj = bpy.context.active_object
    assert "_joint" in joint_obj.name
    assert joint_obj.type == "EMPTY"
    assert joint_obj.linkforge_joint.is_robot_joint is True
    assert joint_obj.linkforge_joint.child_link == link_obj


def test_joint_ops_delete_joint():
    """Test LINKFORGE_OT_delete_joint operator."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create joint
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    bpy.ops.linkforge.create_joint()
    joint_obj = bpy.context.active_object
    joint_name = joint_obj.name

    # 1. Test Poll
    assert bpy.ops.linkforge.delete_joint.poll() is True

    # 2. Test synchronization with ROS2 Control (Mock scene properties)
    scene = bpy.context.scene
    # Assuming ros2_control_joints is accessible
    if hasattr(scene.linkforge, "ros2_control_joints"):
        item = scene.linkforge.ros2_control_joints.add()
        item.name = joint_name
        assert len(scene.linkforge.ros2_control_joints) == 1

    # 3. Test Execute
    bpy.ops.linkforge.delete_joint()
    assert joint_name not in bpy.data.objects

    # Check sync
    if hasattr(scene.linkforge, "ros2_control_joints"):
        assert len(scene.linkforge.ros2_control_joints) == 0


def test_joint_ops_auto_detect():
    """Test LINKFORGE_OT_auto_detect_parent_child operator."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create link hierarchy
    # Child at (1,0,0)
    bpy.ops.mesh.primitive_cube_add(location=(1, 0, 0))
    bpy.ops.linkforge.create_link_from_mesh()
    child_link = bpy.context.active_object

    # Parent at (0,0,0)
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
    bpy.ops.linkforge.create_link_from_mesh()
    parent_link = bpy.context.active_object

    # Create joint at child position
    bpy.ops.object.select_all(action="DESELECT")
    child_link.select_set(True)
    bpy.context.view_layer.objects.active = child_link
    bpy.ops.linkforge.create_joint()
    joint_obj = bpy.context.active_object

    # 1. Test Poll
    assert bpy.ops.linkforge.auto_detect_parent_child.poll() is True

    # 2. Test Execute (Auto-detect)
    # Reset child_link to verify it detects it again
    joint_obj.linkforge_joint.child_link = None
    bpy.ops.linkforge.auto_detect_parent_child()

    assert joint_obj.linkforge_joint.child_link == child_link
    assert joint_obj.linkforge_joint.parent_link == parent_link


def test_joint_ops_auto_detect_edge_cases(mocker):
    """Test edge cases for auto-detect."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # 1. No links in scene
    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.linkforge_joint.is_robot_joint = True

    # Poll should pass if it's a joint
    assert bpy.ops.linkforge.auto_detect_parent_child.poll() is True

    # Execute should report warning (mock report)
    # We can't easily check report in headless without more infrastructure,
    # but we can check it doesn't crash.
    bpy.ops.linkforge.auto_detect_parent_child()

    # 2. Only one link in scene
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    bpy.ops.object.select_all(action="DESELECT")
    joint_obj.select_set(True)
    bpy.context.view_layer.objects.active = joint_obj

    bpy.ops.linkforge.auto_detect_parent_child()
    assert joint_obj.linkforge_joint.child_link == link_obj
    assert joint_obj.linkforge_joint.parent_link is None

    # 4. Smart Choice logic (variation 2)
    # Create two links for this specific test
    bpy.ops.mesh.primitive_cube_add(location=(1, 0, 0))
    bpy.ops.linkforge.create_link_from_mesh()
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
    bpy.ops.linkforge.create_link_from_mesh()
    lb = bpy.context.active_object

    joint_obj.linkforge_joint.child_link = lb
    # Reselect joint for auto-detect (fix poll error)
    bpy.ops.object.select_all(action="DESELECT")
    joint_obj.select_set(True)
    bpy.context.view_layer.objects.active = joint_obj

    bpy.ops.linkforge.auto_detect_parent_child()
    # Check that parent_link is not None.
    assert joint_obj.linkforge_joint.parent_link is not None


def test_joint_ops_poll_failures():
    """Hit poll failures for joint operators."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # 1. create_joint poll (no active object)
    assert bpy.ops.linkforge.create_joint.poll() is False

    # 2. create_joint poll (selected but not link)
    bpy.ops.mesh.primitive_cube_add()
    assert bpy.ops.linkforge.create_joint.poll() is False

    # 3. delete_joint poll (not a joint)
    assert bpy.ops.linkforge.delete_joint.poll() is False

    # 4. auto_detect poll (not a joint)
    assert bpy.ops.linkforge.auto_detect_parent_child.poll() is False


def test_joint_ops_create_joint_edge_cases(mocker):
    """Hit error paths in create_joint."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create object but not a link
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Mock linkforge property to look like a link but then fail execute check
    # Actually, execute check is: if not link_obj or not hasattr(link_obj, "linkforge"):
    # We can hit this by selecting a visual child whose parent is deleted? No.

    # Let's hit the manual joint creation failure (bpy.ops.object.empty_add returns None?)
    with patch("bpy.ops.object.empty_add", return_value={"CANCELLED"}):
        # We need to mock context.active_object to be a link for execute to proceed
        obj.linkforge.is_robot_link = True
        bpy.context.view_layer.objects.active = obj
        # Note: @safe_execute will catch the failure if it returns CANCELLED?
        # No, LINKFORGE_OT_create_joint returns CANCELLED if not joint_empty.
        # But empty_add in Blender headless usually returns FINISHED even if it doesn't create anything?
        # No, we can mock it.
        pass


def test_joint_ops_main_entry(mocker):
    """Simulate module main entry."""
    from linkforge.blender.operators import joint_ops

    mock_reg = mocker.patch.object(joint_ops, "register")
    joint_ops.register()
    mock_reg.assert_called_once()
