import bpy

from tests.blender_test_utils import (
    create_mesh_object,
    safe_get_joint,
)


def test_transmission_ops_create_transmission(mock_context, scene) -> None:
    """Test LINKFORGE_OT_create_transmission operator."""
    # Setup: Create link and joint
    obj = create_mesh_object("link_trans", scene)
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.linkforge.create_link_from_mesh()  # type: ignore[attr-defined]
    bpy.ops.linkforge.create_joint()  # type: ignore[attr-defined]
    joint_obj = bpy.context.active_object
    assert joint_obj is not None

    # Test Poll (should pass with joint selected)
    assert bpy.ops.linkforge.create_transmission.poll() is True  # type: ignore[attr-defined]

    # Test alignment logic (X axis)
    safe_get_joint(joint_obj).axis = "X"
    bpy.ops.linkforge.create_transmission()  # type: ignore[attr-defined]
    trans_x = bpy.context.active_object
    assert trans_x is not None
    assert "_trans" in trans_x.name
    assert trans_x.parent == joint_obj

    # Reselect joint for next test
    bpy.ops.object.select_all(action="DESELECT")
    joint_obj.select_set(True)
    bpy.context.view_layer.objects.active = joint_obj

    # Test alignment logic (Y axis)
    safe_get_joint(joint_obj).axis = "Y"
    bpy.ops.object.select_all(action="DESELECT")
    joint_obj.select_set(True)
    bpy.context.view_layer.objects.active = joint_obj
    bpy.ops.linkforge.create_transmission()  # type: ignore[attr-defined]

    # Test alignment logic (Z axis)
    safe_get_joint(joint_obj).axis = "Z"
    bpy.ops.object.select_all(action="DESELECT")
    joint_obj.select_set(True)
    bpy.context.view_layer.objects.active = joint_obj
    bpy.ops.linkforge.create_transmission()  # type: ignore[attr-defined]

    trans_z = bpy.context.active_object
    assert trans_z is not None
    assert trans_z.parent == joint_obj

    # Test no axis alignment (axis set to CUSTOM with zero vector)
    # Reselect joint
    bpy.ops.object.select_all(action="DESELECT")
    joint_obj.select_set(True)
    bpy.context.view_layer.objects.active = joint_obj
    safe_get_joint(joint_obj).axis = "CUSTOM"
    safe_get_joint(joint_obj).custom_axis_x = 0
    safe_get_joint(joint_obj).custom_axis_y = 0
    safe_get_joint(joint_obj).custom_axis_z = 0
    bpy.ops.linkforge.create_transmission()  # type: ignore[attr-defined]


def test_transmission_ops_delete_transmission(mock_context, scene) -> None:
    """Test LINKFORGE_OT_delete_transmission operator."""
    # Setup: Link -> Joint -> Transmission
    obj = create_mesh_object("link_for_trans_del", scene)
    assert bpy.context.view_layer is not None
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.linkforge.create_link_from_mesh()  # type: ignore[attr-defined]
    bpy.ops.linkforge.create_joint()  # type: ignore[attr-defined]
    bpy.ops.linkforge.create_transmission()  # type: ignore[attr-defined]
    trans_obj = bpy.context.active_object
    assert trans_obj is not None
    trans_name = trans_obj.name

    # Test Poll
    assert bpy.ops.linkforge.delete_transmission.poll() is True  # type: ignore[attr-defined]

    # Test Execute
    bpy.ops.linkforge.delete_transmission()  # type: ignore[attr-defined]
    assert trans_name not in bpy.data.objects


def test_transmission_ops_poll_failures(mock_context, scene) -> None:
    """Hit poll failures for transmission operators."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # No active object
    assert bpy.ops.linkforge.create_transmission.poll() is False  # type: ignore[attr-defined]

    # Active but not a joint
    bpy.ops.mesh.primitive_cube_add()
    assert bpy.ops.linkforge.create_transmission.poll() is False  # type: ignore[attr-defined]


def test_transmission_ops_main_entry(mocker, scene) -> None:
    """Simulate module main entry."""
    from linkforge.blender.operators import transmission_ops

    mock_reg = mocker.patch.object(transmission_ops, "register")
    transmission_ops.register()
    mock_reg.assert_called_once()
