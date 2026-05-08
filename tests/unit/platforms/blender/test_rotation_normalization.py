import bpy

from tests.blender_test_utils import safe_get_linkforge


def test_rotation_mode_normalization_on_add_link(scene) -> None:
    """Verify that adding a new empty link frame forces XYZ rotation mode."""
    # Add link
    bpy.ops.linkforge.add_empty_link()  # type: ignore[attr-defined]
    obj = bpy.context.active_object
    assert obj is not None

    assert safe_get_linkforge(obj).is_robot_link is True
    assert obj.rotation_mode == "XYZ"


def test_rotation_mode_normalization_on_create_from_mesh(scene) -> None:
    """Verify that converting a mesh with Quaternions to a Link forces XYZ mode."""
    # Create mesh with Quaternions
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None
    obj.rotation_mode = "QUATERNION"
    obj.rotation_quaternion = (0.707, 0.707, 0, 0)  # 90 deg X

    # Convert to link
    bpy.ops.linkforge.create_link_from_mesh()  # type: ignore[attr-defined]

    # The mesh becomes a child of the Empty
    link_obj = bpy.context.active_object  # The Empty
    assert link_obj is not None
    visual_obj = link_obj.children[0]

    assert link_obj.rotation_mode == "XYZ"
    assert visual_obj.rotation_mode == "XYZ"
    # Values should match the original rotation in Euler
    assert abs(visual_obj.rotation_euler.x) < 0.001
    assert abs(visual_obj.rotation_euler.y) < 0.001
    assert abs(visual_obj.rotation_euler.z) < 0.001
    # Because creation_from_mesh snaps Empty to Mesh and zeros visual local.
