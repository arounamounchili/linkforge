import bpy


def test_sensor_ops_create_sensor():
    """Test LINKFORGE_OT_create_sensor operator."""
    # Setup: Create a link and a joint
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    bpy.ops.linkforge.create_joint()

    # 1. Test Poll (should pass with link selected)
    link_obj.select_set(True)
    bpy.context.view_layer.objects.active = link_obj
    assert bpy.ops.linkforge.create_sensor.poll() is True

    # 2. Test Execute
    bpy.ops.linkforge.create_sensor()

    sensor_obj = bpy.context.active_object
    assert "_sensor" in sensor_obj.name
    assert sensor_obj.type == "EMPTY"
    assert sensor_obj.linkforge_sensor.is_robot_sensor is True
    assert sensor_obj.parent == link_obj


def test_sensor_ops_delete_sensor():
    """Test LINKFORGE_OT_delete_sensor operator."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create sensor
    bpy.ops.mesh.primitive_cube_add()
    bpy.ops.linkforge.create_link_from_mesh()
    bpy.ops.linkforge.create_sensor()
    sensor_obj = bpy.context.active_object
    sensor_name = sensor_obj.name

    # 1. Test Poll
    assert bpy.ops.linkforge.delete_sensor.poll() is True

    # 2. Test Execute
    bpy.ops.linkforge.delete_sensor()
    assert sensor_name not in bpy.data.objects


def test_sensor_ops_poll_failures():
    """Hit poll failures for sensor operators."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # 1. No active object
    assert bpy.ops.linkforge.create_sensor.poll() is False

    # 2. Active but not a joint
    bpy.ops.mesh.primitive_cube_add()
    assert bpy.ops.linkforge.create_sensor.poll() is False


def test_sensor_ops_main_entry(mocker):
    """Simulate module main entry."""
    from linkforge.blender.operators import sensor_ops

    mock_reg = mocker.patch.object(sensor_ops, "register")
    sensor_ops.register()
    mock_reg.assert_called_once()
