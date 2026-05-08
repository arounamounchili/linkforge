import bpy
from linkforge.blender.properties.joint_props import poll_robot_joint, poll_robot_link

from tests.blender_test_utils import create_test_object, safe_get_joint, safe_get_linkforge


def test_joint_name_getter_setter(scene) -> None:
    """Test that joint_name mirrors and sanitizes the object name."""
    obj = create_test_object("My Joint", None, scene)
    safe_get_joint(obj).is_robot_joint = True

    # Getter should return sanitized name
    assert safe_get_joint(obj).joint_name == "My_Joint"

    # Setter should update object name
    safe_get_joint(obj).joint_name = "New-Joint-Name"
    assert obj.name == "New-Joint-Name"


def test_joint_hierarchy_links(scene) -> None:
    """Test that assigning parent/child links updates the hierarchy correctly."""
    # Create parent link
    parent_link = create_test_object("parent_link", None, scene)
    safe_get_linkforge(parent_link).is_robot_link = True

    # Create joint
    joint = create_test_object("test_joint", None, scene)
    safe_get_joint(joint).is_robot_joint = True

    # Create child link
    child_link = create_test_object("child_link", None, scene)
    safe_get_linkforge(child_link).is_robot_link = True

    # Assign parent link
    safe_get_joint(joint).parent_link = parent_link
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.update()
    assert joint.parent == parent_link

    # Assign child link
    safe_get_joint(joint).child_link = child_link
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.update()
    assert child_link.parent == joint

    # Clear child link (should unparent the child)
    safe_get_joint(joint).child_link = None
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.update()
    assert child_link.parent is None

    # Clear parent link
    safe_get_joint(joint).parent_link = None
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.update()
    assert joint.parent is None


def test_poll_filters(scene) -> None:
    """Test the poll functions for links and joints."""
    # Robot link
    link_obj = create_test_object("LinkObj", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True

    # Robot joint
    joint_obj = create_test_object("JointObj", None, scene)
    safe_get_joint(joint_obj).is_robot_joint = True

    # Regular object
    none_obj = create_test_object("NoneObj", None, scene)

    # Poll Link
    assert poll_robot_link(None, link_obj) is True  # type: ignore
    assert poll_robot_link(None, joint_obj) is False  # type: ignore
    assert poll_robot_link(None, none_obj) is False  # type: ignore

    # Poll Joint (prevents self-mimicry)
    assert poll_robot_joint(safe_get_joint(joint_obj), joint_obj) is False

    # Create another joint
    other_joint = create_test_object("OtherJoint", None, scene)
    safe_get_joint(other_joint).is_robot_joint = True
    assert poll_robot_joint(safe_get_joint(joint_obj), other_joint) is True
