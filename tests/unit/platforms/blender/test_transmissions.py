"""Unit tests for Blender Transmission operations, properties, and robustness."""

from __future__ import annotations

import bpy
import pytest
from mathutils import Vector

from tests.blender_test_utils import (
    safe_get_joint,
    safe_get_transmission,
)

# =============================================================================
# Transmission Operations
# =============================================================================


class TestTransmissionOperations:
    def test_create_transmission(self, scene) -> None:
        """Test creating a transmission for a joint."""
        # Setup Joint
        bpy.ops.object.empty_add()
        bpy.context.active_object.name = "Joint"
        j = bpy.context.active_object
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

    def test_delete_transmission(self, scene) -> None:
        """Test deleting a transmission."""
        bpy.ops.object.empty_add()
        bpy.context.active_object.name = "Trans"
        trans = bpy.context.active_object
        safe_get_transmission(trans).is_robot_transmission = True

        trans.select_set(True)
        bpy.context.view_layer.objects.active = trans

        bpy.ops.linkforge.delete_transmission()
        assert "Trans" not in bpy.data.objects


# =============================================================================
# Transmission Hierarchy and Logic
# =============================================================================


class TestTransmissionLogic:
    def test_transmission_hierarchy_simple(self, scene) -> None:
        """Test that a simple transmission is reparented to its joint."""
        # Create joint
        bpy.ops.object.empty_add()
        bpy.context.active_object.name = "joint_obj"
        joint_obj = bpy.context.active_object
        safe_get_joint(joint_obj).is_robot_joint = True

        # Create transmission
        bpy.ops.object.empty_add()
        bpy.context.active_object.name = "trans_obj"
        trans_obj = bpy.context.active_object
        props = safe_get_transmission(trans_obj)
        props.is_robot_transmission = True

        # Assign joint to transmission (triggers update)
        props.joint_name = joint_obj

        assert trans_obj.parent == joint_obj
        assert trans_obj.location == Vector((0, 0, 0))

    def test_poll_robot_joint(self, scene) -> None:
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
