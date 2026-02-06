import bpy
from mathutils import Vector


def test_transmission_name_getter_setter():
    """Test that transmission_name getter/setter work and sanitize names."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.name = "Trans Name"
    obj.linkforge_transmission.is_robot_transmission = True

    # Getter
    assert obj.linkforge_transmission.transmission_name == "Trans_Name"

    # Setter
    obj.linkforge_transmission.transmission_name = "New-Trans!"
    assert obj.name == "New-Trans_"


def test_transmission_hierarchy_simple():
    """Test that a simple transmission is reparented to its joint."""
    bpy.ops.object.select_all(action="DESELECT")

    # Create joint
    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.name = "joint_obj"
    joint_obj.linkforge_joint.is_robot_joint = True

    # Create transmission
    bpy.ops.object.empty_add()
    trans_obj = bpy.context.active_object
    trans_obj.name = "trans_obj"
    trans_obj.linkforge_transmission.is_robot_transmission = True

    # Assign joint to transmission
    trans_obj.linkforge_transmission.joint_name = joint_obj

    # Assert
    assert trans_obj.parent == joint_obj
    assert trans_obj.location == Vector((0, 0, 0))
    assert all(abs(c) < 1e-6 for c in trans_obj.rotation_euler)


def test_transmission_hierarchy_differential():
    """Test that a differential transmission is reparented to its first joint."""
    bpy.ops.object.select_all(action="DESELECT")

    # Create joints
    bpy.ops.object.empty_add()
    j1 = bpy.context.active_object
    j1.name = "joint1"
    j1.linkforge_joint.is_robot_joint = True

    bpy.ops.object.empty_add()
    j2 = bpy.context.active_object
    j2.name = "joint2"
    j2.linkforge_joint.is_robot_joint = True

    # Create transmission
    bpy.ops.object.empty_add()
    trans_obj = bpy.context.active_object
    trans_obj.name = "diff_trans"
    trans_obj.linkforge_transmission.is_robot_transmission = True
    trans_obj.linkforge_transmission.transmission_type = "DIFFERENTIAL"

    # Assign first joint
    trans_obj.linkforge_transmission.joint1_name = j1

    # Assert
    assert trans_obj.parent == j1


def test_poll_robot_joint():
    """Test that only robot joint objects are filtered."""
    from linkforge.blender.properties.transmission_props import poll_robot_joint

    bpy.ops.object.select_all(action="DESELECT")

    # Joint object
    bpy.ops.object.empty_add()
    j_obj = bpy.context.active_object
    j_obj.linkforge_joint.is_robot_joint = True

    # Non-joint object
    bpy.ops.object.empty_add()
    n_obj = bpy.context.active_object

    # Create transmission to check poll
    bpy.ops.object.empty_add()
    trans_obj = bpy.context.active_object
    props = trans_obj.linkforge_transmission

    # poll_robot_joint(self, obj)
    assert poll_robot_joint(props, j_obj) is True
    assert poll_robot_joint(props, n_obj) is False
    assert poll_robot_joint(props, None) is False
