from mathutils import Vector

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_transmission,
)


def test_transmission_name_getter_setter(scene) -> None:
    """Test that transmission_name getter/setter work and sanitize names."""
    obj = create_test_object("Trans Name", None, scene)
    props = safe_get_transmission(obj)
    props.is_robot_transmission = True

    # Getter
    assert props.transmission_name == "Trans_Name"

    # Setter
    props.transmission_name = "New-Trans!"
    assert obj.name == "New-Trans_"


def test_transmission_hierarchy_simple(scene) -> None:
    """Test that a simple transmission is reparented to its joint."""
    # Create joint
    joint_obj = create_test_object("joint_obj", None, scene)
    safe_get_joint(joint_obj).is_robot_joint = True

    # Create transmission
    trans_obj = create_test_object("trans_obj", None, scene)
    props = safe_get_transmission(trans_obj)
    props.is_robot_transmission = True

    # Assign joint to transmission
    props.joint_name = joint_obj

    # Assert
    assert trans_obj.parent == joint_obj
    assert trans_obj.location == Vector((0, 0, 0))
    assert all(abs(c) < 1e-6 for c in trans_obj.rotation_euler)


def test_transmission_hierarchy_differential(scene) -> None:
    """Test that a differential transmission is reparented to its first joint."""
    # Create joints
    j1 = create_test_object("joint1", None, scene)
    safe_get_joint(j1).is_robot_joint = True

    j2 = create_test_object("joint2", None, scene)
    safe_get_joint(j2).is_robot_joint = True

    # Create transmission
    trans_obj = create_test_object("diff_trans", None, scene)
    props = safe_get_transmission(trans_obj)
    props.is_robot_transmission = True
    props.transmission_type = "DIFFERENTIAL"

    # Assign first joint
    props.joint1_name = j1

    # Assert
    assert trans_obj.parent == j1


def test_poll_robot_joint(scene) -> None:
    """Test that only robot joint objects are filtered."""
    from linkforge.blender.properties.transmission_props import poll_robot_joint

    # Joint object
    j_obj = create_test_object("j_obj", None, scene)
    safe_get_joint(j_obj).is_robot_joint = True

    # Non-joint object
    n_obj = create_test_object("n_obj", None, scene)

    # Create transmission to check poll
    trans_obj = create_test_object("trans", None, scene)
    props = safe_get_transmission(trans_obj)

    # poll_robot_joint(self, obj)
    assert poll_robot_joint(props, j_obj) is True
    assert poll_robot_joint(props, n_obj) is False
    assert poll_robot_joint(props, None) is False
