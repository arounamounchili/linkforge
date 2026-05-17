"""Unit tests for Blender Transmission operations, properties, and robustness."""

from __future__ import annotations

from unittest.mock import PropertyMock, patch

import bpy
from linkforge.blender.operators.transmission_ops import (
    LINKFORGE_OT_create_transmission,
    LINKFORGE_OT_delete_transmission,
    create_transmission_for_joint,
)

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_transmission,
)


class TestTransmissionOperations:
    def test_create_transmission_operator_poll(self, mocker, scene, blender_context) -> None:
        """Test poll method of create transmission operator."""
        op = LINKFORGE_OT_create_transmission

        # Active object is None
        mocker.patch.object(
            type(bpy.context), "active_object", new_callable=PropertyMock, return_value=None
        )
        assert not op.poll(bpy.context)

        # Active object exists but not selected
        j = create_test_object("Joint", None, scene)
        safe_get_joint(j).is_robot_joint = True
        j.select_set(False)
        mocker.patch.object(
            type(bpy.context), "active_object", new_callable=PropertyMock, return_value=j
        )
        assert not op.poll(bpy.context)

        # Selected and is joint
        j.select_set(True)
        assert op.poll(bpy.context)

    def test_create_transmission_operator_execute(self, scene, blender_context) -> None:
        """Test execute method of create transmission operator."""
        j = create_test_object("Joint", None, scene)
        safe_get_joint(j).is_robot_joint = True

        blender_context.view_layer.objects.active = j
        j.select_set(True)

        op = LINKFORGE_OT_create_transmission()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        trans = blender_context.get_active_object()
        assert trans is not None
        assert trans.name == "Joint_trans"
        assert trans.parent == j
        assert safe_get_transmission(trans).is_robot_transmission

    def test_create_transmission_for_joint_axis_variations(self, scene, blender_context) -> None:
        """Test creating a transmission with X, Y, Z, and CUSTOM joint axes."""
        # Test axis X
        j_x = create_test_object("JointX", None, scene)
        jp_x = safe_get_joint(j_x)
        jp_x.is_robot_joint = True
        jp_x.axis = "X"
        assert create_transmission_for_joint(j_x, blender_context)

        # Test axis Y
        j_y = create_test_object("JointY", None, scene)
        jp_y = safe_get_joint(j_y)
        jp_y.is_robot_joint = True
        jp_y.axis = "Y"
        assert create_transmission_for_joint(j_y, blender_context)

        # Test axis Z
        j_z = create_test_object("JointZ", None, scene)
        jp_z = safe_get_joint(j_z)
        jp_z.is_robot_joint = True
        jp_z.axis = "Z"
        assert create_transmission_for_joint(j_z, blender_context)

        # Test axis CUSTOM
        j_c = create_test_object("JointCustom", None, scene)
        jp_c = safe_get_joint(j_c)
        jp_c.is_robot_joint = True
        jp_c.axis = "CUSTOM"
        jp_c.custom_axis_x = 0.707
        jp_c.custom_axis_y = 0.707
        jp_c.custom_axis_z = 0.0
        assert create_transmission_for_joint(j_c, blender_context)

    def test_create_transmission_for_joint_fallback(self, scene, blender_context) -> None:
        """Test fallback behavior when preferences are missing or joint_props is None."""
        # Setup Joint
        j = create_test_object("Joint", None, scene)
        safe_get_joint(j).is_robot_joint = True

        with patch("linkforge.blender.preferences.get_addon_prefs", return_value=None):
            assert create_transmission_for_joint(j, blender_context)

        # Fails when joint_props is not a valid robot joint
        j_invalid = create_test_object("InvalidJoint", None, scene)
        with patch(
            "linkforge.blender.operators.transmission_ops.get_joint_props", return_value=None
        ):
            assert not create_transmission_for_joint(j_invalid, blender_context)

    def test_transmission_defaults(self, scene, blender_context) -> None:
        """Test transmission property defaults."""
        trans = create_test_object("test_trans_defaults", None, scene)
        props = safe_get_transmission(trans)
        assert props.is_robot_transmission is False

    def test_delete_transmission_operator_poll(self, mocker, scene, blender_context) -> None:
        """Test poll method of delete transmission operator."""
        op = LINKFORGE_OT_delete_transmission

        # Active object is None
        mocker.patch.object(
            type(bpy.context), "active_object", new_callable=PropertyMock, return_value=None
        )
        assert not op.poll(bpy.context)

        # Active object exists but not selected
        trans = create_test_object("Trans", None, scene)
        safe_get_transmission(trans).is_robot_transmission = True
        trans.select_set(False)
        mocker.patch.object(
            type(bpy.context), "active_object", new_callable=PropertyMock, return_value=trans
        )
        assert not op.poll(bpy.context)

        # Selected and is transmission
        trans.select_set(True)
        assert op.poll(bpy.context)

    def test_delete_transmission_operator_execute(self, mocker, scene, blender_context) -> None:
        """Test execute method of delete transmission operator."""
        trans = create_test_object("Trans", None, scene)
        safe_get_transmission(trans).is_robot_transmission = True
        mocker.patch.object(
            type(bpy.context), "active_object", new_callable=PropertyMock, return_value=trans
        )

        op = LINKFORGE_OT_delete_transmission()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        assert trans.name not in [o.name for o in blender_context.data.objects]


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

        j_obj = create_test_object("j_obj", None, scene)
        safe_get_joint(j_obj).is_robot_joint = True
        blender_context.view_layer.objects.active = j_obj

        n_obj = create_test_object("n_obj", None, scene)

        trans_obj = create_test_object("trans", None, scene)
        blender_context.view_layer.objects.active = trans_obj
        props = safe_get_transmission(trans_obj)

        assert poll_robot_joint(props, j_obj) is True
        assert poll_robot_joint(props, n_obj) is False
