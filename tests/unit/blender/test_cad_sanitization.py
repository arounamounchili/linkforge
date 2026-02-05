"""Integration test for CAD-style mesh sanitization during Link creation."""

import bpy
import pytest
from mathutils import Quaternion


def test_link_creation_sanitization_all_modes():
    """Test link creation from meshes with various rotation modes and ghost parenting."""
    modes_to_test = [
        ("QUATERNION", Quaternion((0.7071, 0.7071, 0, 0))),  # 90 deg X
        ("AXIS_ANGLE", (1.5708, 1, 0, 0)),  # 90 deg X
    ]

    for mode, rot_val in modes_to_test:
        # 1. Setup a "dirty" CAD-style mesh
        bpy.ops.mesh.primitive_cube_add()
        mesh_obj = bpy.context.active_object
        mesh_obj.name = f"CAD_Mesh_{mode}"

        # Set rotation mode and value
        mesh_obj.rotation_mode = mode
        if mode == "QUATERNION":
            mesh_obj.rotation_quaternion = rot_val
        elif mode == "AXIS_ANGLE":
            mesh_obj.rotation_axis_angle = rot_val

        # Simulate "Ghost Bone" state
        import contextlib

        with contextlib.suppress(TypeError):
            mesh_obj.parent_type = "BONE"

        # 2. Run LinkForge "Create Link from Mesh"
        bpy.ops.linkforge.create_link_from_mesh()

        # 3. Verify the result
        link_obj = bpy.context.active_object
        assert link_obj.type == "EMPTY"

        # Verify World Matrix Identity: The mesh should now follow the link frame exactly
        # We compare the world matrices to ensure no visual shift occurred
        for i in range(4):
            for j in range(4):
                assert mesh_obj.matrix_world[i][j] == pytest.approx(
                    link_obj.matrix_world[i][j], abs=1e-4
                )

        # Check parenting and sanitization
        assert mesh_obj.parent == link_obj
        assert mesh_obj.parent_type == "OBJECT"
        assert mesh_obj.parent_bone == ""

        # Check that mesh local transform is truly zeroed
        assert mesh_obj.rotation_mode == "XYZ"
        assert mesh_obj.location.x == pytest.approx(0.0)
        assert mesh_obj.location.y == pytest.approx(0.0)
        assert mesh_obj.location.z == pytest.approx(0.0)
        assert mesh_obj.rotation_euler.x == pytest.approx(0.0)
        assert mesh_obj.rotation_euler.y == pytest.approx(0.0)
        assert mesh_obj.rotation_euler.z == pytest.approx(0.0)
        assert mesh_obj.scale.x == pytest.approx(1.0)
        assert mesh_obj.scale.y == pytest.approx(1.0)
        assert mesh_obj.scale.z == pytest.approx(1.0)

        # Cleanup for next iteration
        bpy.data.objects.remove(link_obj, do_unlink=True)
        bpy.data.objects.remove(mesh_obj, do_unlink=True)


def test_root_link_orientation_preservation():
    """Test that the root link's world orientation is preserved in URDF origins."""
    from math import radians

    from linkforge.blender.converters import blender_link_to_core_with_origin

    # 1. Setup a rotated root link Empty
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    root_link_obj = bpy.context.active_object
    root_link_obj.name = "root_link"
    root_link_obj.linkforge.is_robot_link = True
    # Rotate 90 degrees around Y (stands up from lying down)
    root_link_obj.rotation_euler = (0, radians(90), 0)

    # 2. Add a visual child (already aligned with root in Blender)
    bpy.ops.mesh.primitive_cube_add()
    visual_obj = bpy.context.active_object
    visual_obj.name = "root_link_visual"
    visual_obj.parent = root_link_obj
    visual_obj.location = (1, 0, 0)  # Offset relative to root
    visual_obj.rotation_euler = (0, 0, 0)  # No additional rotation

    # CRITICAL: Force update view layer to calculate world matrices
    bpy.context.view_layer.update()

    # 3. Process as root link
    link_model = blender_link_to_core_with_origin(root_link_obj, is_root=True)

    # 4. Verify visual origin
    assert len(link_model.visuals) == 1
    origin = link_model.visuals[0].origin

    # The origin should now reflect the WORLD transform of the visual_obj
    # because the root link frame is exported as Identity.
    # Root(RotY 90) * Visual(LocX 1) = World(LocZ -1, RotY 90)
    assert origin.xyz.z == pytest.approx(-1.0, abs=1e-6)
    assert origin.xyz.x == pytest.approx(0.0, abs=1e-6)
    assert origin.rpy.y == pytest.approx(radians(90), abs=1e-6)

    # Cleanup
    bpy.data.objects.remove(root_link_obj, do_unlink=True)
    bpy.data.objects.remove(visual_obj, do_unlink=True)


if __name__ == "__main__":
    test_link_creation_sanitization_all_modes()
    test_root_link_orientation_preservation()
