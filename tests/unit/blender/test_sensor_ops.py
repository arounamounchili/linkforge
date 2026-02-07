"""Tests for sensor_ops module."""

import bpy


def test_create_sensor_from_link():
    """Test creating a sensor from a selected link."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create a link
    bpy.ops.object.empty_add(location=(1, 2, 3))
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.link_name = "test_link"

    # Create sensor
    link_obj.select_set(True)
    bpy.ops.linkforge.create_sensor()

    # Verify sensor was created
    sensor_obj = bpy.context.active_object
    assert sensor_obj != link_obj
    assert sensor_obj.linkforge_sensor.is_robot_sensor
    assert sensor_obj.parent == link_obj
    assert sensor_obj.linkforge_sensor.attached_link == link_obj


def test_create_sensor_from_visual_child():
    """Test creating sensor when visual child of link is selected."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create link
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.link_name = "test_link"

    # Create visual mesh child
    bpy.ops.mesh.primitive_cube_add()
    visual_obj = bpy.context.active_object
    visual_obj.parent = link_obj

    # Create sensor from visual (should attach to link)
    visual_obj.select_set(True)
    bpy.ops.linkforge.create_sensor()

    sensor_obj = bpy.context.active_object
    assert sensor_obj.linkforge_sensor.is_robot_sensor
    assert sensor_obj.parent == link_obj


def test_create_sensor_poll_no_selection():
    """Test that poll fails with no selection."""
    bpy.ops.object.select_all(action="DESELECT")

    from linkforge.blender.operators.sensor_ops import LINKFORGE_OT_create_sensor

    assert not LINKFORGE_OT_create_sensor.poll(bpy.context)


def test_create_sensor_poll_non_link():
    """Test that poll fails when non-link object is selected."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.mesh.primitive_cube_add()
    cube = bpy.context.active_object
    cube.select_set(True)

    from linkforge.blender.operators.sensor_ops import LINKFORGE_OT_create_sensor

    assert not LINKFORGE_OT_create_sensor.poll(bpy.context)


def test_create_sensor_default_type():
    """Test that sensor is created with default camera type."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True

    link_obj.select_set(True)
    bpy.ops.linkforge.create_sensor()

    sensor_obj = bpy.context.active_object
    assert sensor_obj.linkforge_sensor.sensor_type == "CAMERA"


def test_delete_sensor():
    """Test deleting a sensor."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create sensor
    bpy.ops.object.empty_add()
    sensor_obj = bpy.context.active_object
    sensor_obj.linkforge_sensor.is_robot_sensor = True
    sensor_obj.linkforge_sensor.sensor_name = "test_sensor"

    # Store name before deletion
    sensor_name = sensor_obj.name

    # Delete sensor
    sensor_obj.select_set(True)
    bpy.ops.linkforge.delete_sensor()

    # Verify sensor was deleted
    assert sensor_name not in bpy.data.objects


def test_delete_sensor_poll_no_selection():
    """Test that delete poll fails with no selection."""
    bpy.ops.object.select_all(action="DESELECT")

    from linkforge.blender.operators.sensor_ops import LINKFORGE_OT_delete_sensor

    assert not LINKFORGE_OT_delete_sensor.poll(bpy.context)


def test_delete_sensor_poll_non_sensor():
    """Test that delete poll fails for non-sensor objects."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.select_set(True)

    from linkforge.blender.operators.sensor_ops import LINKFORGE_OT_delete_sensor

    assert not LINKFORGE_OT_delete_sensor.poll(bpy.context)


def test_sensor_naming():
    """Test that sensor names are properly generated."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.link_name = "base"

    link_obj.select_set(True)
    bpy.ops.linkforge.create_sensor()

    sensor_obj = bpy.context.active_object
    assert "base_sensor" in sensor_obj.name.lower()


def test_sensor_location_at_origin():
    """Test that sensor is created at link's origin (0,0,0 relative)."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add(location=(5, 10, 15))
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True

    link_obj.select_set(True)
    bpy.ops.linkforge.create_sensor()

    sensor_obj = bpy.context.active_object
    # Should be at 0,0,0 relative to parent
    assert abs(sensor_obj.location[0]) < 0.001
    assert abs(sensor_obj.location[1]) < 0.001
    assert abs(sensor_obj.location[2]) < 0.001
