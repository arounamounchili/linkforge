"""Unit tests for Blender Transmission operations, properties, and robustness."""

from __future__ import annotations

import bpy
import pytest

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_transmission,
)


class TestTransmissionOperations:
    def test_create_transmission(self, scene, blender_context) -> None:
        """Test creating a transmission for a joint."""
        # Setup Joint
        j = create_test_object("Joint", None, scene)
        safe_get_joint(j).is_robot_joint = True

        # Ensure active and selected
        bpy.context.view_layer.objects.active = j
        j.select_set(True)

        bpy.ops.linkforge.create_transmission()
        trans = bpy.context.active_object
        assert trans is not None
        assert trans.name == f"{j.name}_trans"
        assert trans.parent == j
        assert safe_get_transmission(trans).is_robot_transmission

    def test_transmission_defaults(self, scene, blender_context) -> None:
        """Test transmission property defaults."""
        trans = create_test_object("test_trans_defaults", None, scene)
        props = safe_get_transmission(trans)
        assert props.is_robot_transmission is False

    def test_delete_transmission(self, scene, blender_context) -> None:
        """Test deleting a transmission."""
        trans = create_test_object("Trans", None, scene)
        safe_get_transmission(trans).is_robot_transmission = True

        trans.select_set(True)
        bpy.context.view_layer.objects.active = trans

        bpy.ops.linkforge.delete_transmission()
        assert "Trans" not in bpy.data.objects


# Transmission Hierarchy and Logic


class TestTransmissionLogic:
    def test_transmission_hierarchy_simple(self, scene, blender_context) -> None:
        """Test that a simple transmission is reparented to its joint."""
        # Create joint
        joint_obj = create_test_object("joint_obj", None, scene)
        safe_get_joint(joint_obj).is_robot_joint = True

        # Create transmission
        trans_obj = create_test_object("trans_obj", None, scene)
        props = safe_get_transmission(trans_obj)
        props.is_robot_transmission = True

        # Assign joint to transmission (triggers update)
        props.joint_name = joint_obj

        # In mock environments, we must manually trigger the update callback
        from linkforge.blender.properties.transmission_props import update_transmission_hierarchy

        update_transmission_hierarchy(props, blender_context)

        assert trans_obj.parent == joint_obj
        assert list(trans_obj.location) == [0, 0, 0]

    def test_poll_robot_joint(self, scene, blender_context) -> None:
        """Test filtering for robot joint objects in UI polls."""
        from linkforge.blender.properties.transmission_props import poll_robot_joint

        bpy.ops.object.empty_add()
        bpy.context.active_object.name = "j_obj"
        j_obj = bpy.context.active_object
        safe_get_joint(j_obj).is_robot_joint = True

        bpy.ops.mesh.primitive_cube_add()
        bpy.context.active_object.name = "n_obj"
        n_obj = bpy.context.active_object

        bpy.ops.object.empty_add()
        bpy.context.active_object.name = "trans"
        trans_obj = bpy.context.active_object
        props = safe_get_transmission(trans_obj)

        assert poll_robot_joint(props, j_obj) is True
        assert poll_robot_joint(props, n_obj) is False


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
