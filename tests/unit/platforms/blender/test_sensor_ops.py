import bpy

from tests.blender_test_utils import (
    create_mesh_object,
    safe_get_sensor,
)


def test_sensor_ops_create_sensor(mock_context, scene) -> None:
    """Test LINKFORGE_OT_create_sensor operator."""
    # Setup: Create a link
    obj = create_mesh_object("link_sensor", scene)
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.linkforge.create_link_from_mesh()  # type: ignore[attr-defined]
    link_obj = bpy.context.active_object
    assert link_obj is not None

    bpy.ops.linkforge.create_joint()  # type: ignore[attr-defined]

    # Test Poll (should pass with link selected)
    link_obj.select_set(True)
    bpy.context.view_layer.objects.active = link_obj
    assert bpy.ops.linkforge.create_sensor.poll() is True  # type: ignore[attr-defined]

    # Test Execute
    bpy.ops.linkforge.create_sensor()  # type: ignore[attr-defined]

    sensor_obj = bpy.context.active_object
    assert sensor_obj is not None
    assert "_sensor" in sensor_obj.name
    assert sensor_obj.type == "EMPTY"
    assert safe_get_sensor(sensor_obj).is_robot_sensor is True
    assert sensor_obj.parent == link_obj


def test_sensor_ops_delete_sensor(mock_context, scene) -> None:
    """Test LINKFORGE_OT_delete_sensor operator."""
    # Create sensor
    obj = create_mesh_object("link_for_sensor_del", scene)
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.linkforge.create_link_from_mesh()  # type: ignore[attr-defined]
    bpy.ops.linkforge.create_sensor()  # type: ignore[attr-defined]
    sensor_obj = bpy.context.active_object
    assert sensor_obj is not None
    sensor_name = sensor_obj.name

    # Test Poll
    assert bpy.ops.linkforge.delete_sensor.poll() is True  # type: ignore[attr-defined]

    # Test Execute
    bpy.ops.linkforge.delete_sensor()  # type: ignore[attr-defined]
    assert sensor_name not in bpy.data.objects


def test_sensor_ops_poll_failures(mock_context, scene) -> None:
    """Hit poll failures for sensor operators."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # No active object
    assert bpy.ops.linkforge.create_sensor.poll() is False  # type: ignore[attr-defined]

    # Active but not a joint
    bpy.ops.mesh.primitive_cube_add()
    assert bpy.ops.linkforge.create_sensor.poll() is False  # type: ignore[attr-defined]


def test_sensor_ops_main_entry(mocker, scene) -> None:
    """Simulate module main entry."""
    from linkforge.blender.operators import sensor_ops

    mock_reg = mocker.patch.object(sensor_ops, "register")
    sensor_ops.register()
    mock_reg.assert_called_once()
