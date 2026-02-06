import bpy
from linkforge.blender.operators.link_ops import create_collision_for_link


def test_collision_alignment_on_rotated_link():
    """Verify that generating collision for a rotated link avoids 'Inverse Rotation' offsets."""
    # 1. Setup a Link at a specific rotation (e.g. 90 deg X)
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True
    link_obj.rotation_mode = "XYZ"
    link_obj.rotation_euler = (1.5708, 0, 0)  # 90 deg X

    # 2. Add a Visual mesh at origin relative to link (Identity local)
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    visual_obj = bpy.context.active_object
    visual_obj.name = "test_visual"
    visual_obj.parent = link_obj
    visual_obj.matrix_parent_inverse.identity()
    visual_obj.location = (0, 0, 0)
    visual_obj.rotation_euler = (0, 0, 0)

    # 3. Generate Collision
    collision_obj = create_collision_for_link(link_obj, "CONVEX_HULL", bpy.context)

    # ASSERTIONS
    assert collision_obj is not None
    assert collision_obj.parent == link_obj

    # The core fix: the collision should be ALIGNED with the visual/link frame.
    # Its local transform should be Identity (or very close to it)
    # instead of containing the 'reverse' rotation of the link.
    local_pos = collision_obj.location
    local_rot = collision_obj.rotation_euler

    assert local_pos.length < 1e-5
    assert local_rot.x < 1e-5
    assert local_rot.y < 1e-5
    assert local_rot.z < 1e-5

    # Verify world matrix match
    assert (collision_obj.matrix_world - link_obj.matrix_world).to_translation().length < 1e-5
