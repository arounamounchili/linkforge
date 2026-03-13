import bpy
from linkforge.blender.properties.joint_props import poll_robot_joint, poll_robot_link


def test_joint_name_getter_setter() -> None:
    """Test that joint_name mirrors and sanitizes the object name."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.name = "My Joint"
    obj.linkforge_joint.is_robot_joint = True

    # Getter should return sanitized name
    assert obj.linkforge_joint.joint_name == "My_Joint"

    # Setter should update object name
    obj.linkforge_joint.joint_name = "New-Joint-Name"
    assert obj.name == "New-Joint-Name"


def test_joint_hierarchy_links() -> None:
    """Test that assigning parent/child links updates the hierarchy correctly."""
    bpy.ops.object.select_all(action="DESELECT")

    # Create parent link
    bpy.ops.object.empty_add()
    parent_link = bpy.context.active_object
    parent_link.name = "parent_link"
    parent_link.linkforge.is_robot_link = True

    # Create joint
    bpy.ops.object.empty_add()
    joint = bpy.context.active_object
    joint.name = "test_joint"
    joint.linkforge_joint.is_robot_joint = True

    # Create child link
    bpy.ops.object.empty_add()
    child_link = bpy.context.active_object
    child_link.name = "child_link"
    child_link.linkforge.is_robot_link = True

    # 1. Assign parent link
    joint.linkforge_joint.parent_link = parent_link
    bpy.context.view_layer.update()
    assert joint.parent == parent_link

    # 2. Assign child link
    joint.linkforge_joint.child_link = child_link
    bpy.context.view_layer.update()
    assert child_link.parent == joint

    # 3. Clear child link (should unparent the child)
    joint.linkforge_joint.child_link = None
    bpy.context.view_layer.update()
    assert child_link.parent is None

    # 4. Clear parent link
    joint.linkforge_joint.parent_link = None
    bpy.context.view_layer.update()
    assert joint.parent is None


def test_poll_filters() -> None:
    """Test the poll functions for links and joints."""
    bpy.ops.object.select_all(action="DESELECT")

    # Robot link
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True

    # Robot joint
    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.linkforge_joint.is_robot_joint = True

    # Regular object
    bpy.ops.object.empty_add()
    none_obj = bpy.context.active_object

    # Poll Link
    assert poll_robot_link(None, link_obj) is True
    assert poll_robot_link(None, joint_obj) is False
    assert poll_robot_link(None, none_obj) is False

    # Poll Joint (prevents self-mimicry)
    assert poll_robot_joint(joint_obj.linkforge_joint, joint_obj) is False

    # Create another joint
    bpy.ops.object.empty_add()
    other_joint = bpy.context.active_object
    other_joint.linkforge_joint.is_robot_joint = True
    assert poll_robot_joint(joint_obj.linkforge_joint, other_joint) is True
