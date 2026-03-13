import bpy


def test_transmission_ops_create_transmission() -> None:
    """Test LINKFORGE_OT_create_transmission operator."""
    # Setup: Create link and joint
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    joint_obj = bpy.ops.linkforge.create_joint()
    joint_obj = bpy.context.active_object

    # 1. Test Poll (should pass with joint selected)
    assert bpy.ops.linkforge.create_transmission.poll() is True

    # 2. Test alignment logic (X axis)
    joint_obj.linkforge_joint.axis = "X"
    bpy.ops.linkforge.create_transmission()
    trans_x = bpy.context.active_object
    assert "_trans" in trans_x.name
    assert trans_x.parent == joint_obj

    # Reselect joint for next test
    bpy.ops.object.select_all(action="DESELECT")
    joint_obj.select_set(True)
    bpy.context.view_layer.objects.active = joint_obj

    # 4. Test alignment logic (Y axis)
    joint_obj.linkforge_joint.axis = "Y"
    bpy.ops.object.select_all(action="DESELECT")
    joint_obj.select_set(True)
    bpy.context.view_layer.objects.active = joint_obj
    bpy.ops.linkforge.create_transmission()

    # 5. Test alignment logic (Z axis)
    joint_obj.linkforge_joint.axis = "Z"
    bpy.ops.object.select_all(action="DESELECT")
    joint_obj.select_set(True)
    bpy.context.view_layer.objects.active = joint_obj
    bpy.ops.linkforge.create_transmission()

    trans_z = bpy.context.active_object
    assert trans_z.parent == joint_obj

    # 6. Test no axis alignment (axis set to CUSTOM with zero vector)
    # Reselect joint
    bpy.ops.object.select_all(action="DESELECT")
    joint_obj.select_set(True)
    bpy.context.view_layer.objects.active = joint_obj
    joint_obj.linkforge_joint.axis = "CUSTOM"
    joint_obj.linkforge_joint.custom_axis_x = 0
    joint_obj.linkforge_joint.custom_axis_y = 0
    joint_obj.linkforge_joint.custom_axis_z = 0
    bpy.ops.linkforge.create_transmission()


def test_transmission_ops_delete_transmission() -> None:
    """Test LINKFORGE_OT_delete_transmission operator."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Setup: Link -> Joint -> Transmission
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    bpy.ops.linkforge.create_joint()
    bpy.ops.linkforge.create_transmission()
    trans_obj = bpy.context.active_object
    trans_name = trans_obj.name

    # 1. Test Poll
    assert bpy.ops.linkforge.delete_transmission.poll() is True

    # 2. Test Execute
    bpy.ops.linkforge.delete_transmission()
    assert trans_name not in bpy.data.objects


def test_transmission_ops_poll_failures() -> None:
    """Hit poll failures for transmission operators."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # 1. No active object
    assert bpy.ops.linkforge.create_transmission.poll() is False

    # 2. Active but not a joint
    bpy.ops.mesh.primitive_cube_add()
    assert bpy.ops.linkforge.create_transmission.poll() is False


def test_transmission_ops_main_entry(mocker) -> None:
    """Simulate module main entry."""
    from linkforge.blender.operators import transmission_ops

    mock_reg = mocker.patch.object(transmission_ops, "register")
    transmission_ops.register()
    mock_reg.assert_called_once()
