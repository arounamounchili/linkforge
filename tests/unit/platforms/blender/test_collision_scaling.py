import bpy
from linkforge.blender.operators.link_ops import create_collision_for_link


def test_primitive_box_collision_scaling_fidelity() -> None:
    """Verify that a scaled cube results in a matching collision primitive.

    Prevents the 'half-size' bug where collision was 2x1.5x0.5 for a 4x3x1 cube.
    """
    # Create a 2m cube (default) and scale it to 4x3x1
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    visual_obj = bpy.context.active_object
    visual_obj.scale = (2.0, 1.5, 0.5)
    bpy.context.view_layer.update()

    # Make it a link
    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    # Generate BOX collision
    collision_obj = create_collision_for_link(link_obj, "BOX", bpy.context)

    # Assert dimensions match (4.0, 3.0, 1.0)
    assert abs(collision_obj.dimensions.x - 4.0) < 1e-5
    assert abs(collision_obj.dimensions.y - 3.0) < 1e-5
    assert abs(collision_obj.dimensions.z - 1.0) < 1e-5

    # Scale should be 1.0 because we bake it
    assert abs(collision_obj.scale.x - 1.0) < 1e-5


def test_primitive_sphere_collision_scaling_fidelity() -> None:
    """Verify sphere collision radius matches scaled visual dimensions."""
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0)  # 2m diameter
    visual_obj = bpy.context.active_object
    visual_obj.scale = (2.0, 2.0, 2.0)  # 4m volume
    bpy.context.view_layer.update()

    bpy.ops.linkforge.create_link_from_mesh()
    link_obj = bpy.context.active_object

    collision_obj = create_collision_for_link(link_obj, "SPHERE", bpy.context)

    # Diameter should be 4.0
    assert abs(collision_obj.dimensions.x - 4.0) < 1e-5
