"""Unit tests for Blender Control (ROS 2) and Sensors."""

from __future__ import annotations

import runpy
import types
from unittest.mock import MagicMock, patch

import bpy
import linkforge.blender.operators.control_ops as control_ops
from linkforge.blender.operators.control_ops import (
    LINKFORGE_OT_add_ros2_control_joint,
    LINKFORGE_OT_add_ros2_control_parameter,
    LINKFORGE_OT_move_ros2_control_joint,
    LINKFORGE_OT_prune_ros2_control_joints,
    LINKFORGE_OT_purge_ros2_control_data,
    LINKFORGE_OT_remove_ros2_control_joint,
    LINKFORGE_OT_remove_ros2_control_parameter,
)
from linkforge.blender.panels.control_panel import LINKFORGE_UL_ros2_control_joints

from tests.blender_test_utils import (
    create_robot_joint,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_linkforge_scene,
    safe_get_sensor,
)
from tests.mock_bpy_env import MockCollection, MockPropertyGroup


class TestControlOperations:
    def test_add_ros2_control_joint_poll(self, scene, blender_context) -> None:
        """Test poll method of add joint operator."""
        op = LINKFORGE_OT_add_ros2_control_joint
        assert op.poll(bpy.context)

        # Missing robot properties in scene
        with patch("linkforge.blender.operators.control_ops.get_robot_props", return_value=None):
            assert not op.poll(bpy.context)

    def test_add_ros2_control_joint_execute(self, scene, blender_context) -> None:
        """Test executing addition of a ROS 2 control joint under various conditions."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_joints.clear()

        joint_obj = create_robot_joint("test_j", None, None, scene)
        joint_props = safe_get_joint(joint_obj)
        joint_props.is_robot_joint = True
        joint_props.joint_name = "test_j"

        op = LINKFORGE_OT_add_ros2_control_joint()
        op.joint_name = "test_j"

        # Successful execute
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        assert len(props.ros2_control_joints) == 1
        assert props.ros2_control_joints[0].name == "test_j"
        assert props.ros2_control_joints[0].joint_obj == joint_obj

        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}

    def test_remove_ros2_control_joint_poll_and_execute(self, scene, blender_context) -> None:
        """Test remove joint operator."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_joints.clear()

        op = LINKFORGE_OT_remove_ros2_control_joint

        # Poll fails when list is empty
        assert not op.poll(bpy.context)

        item = props.ros2_control_joints.add()
        item.name = "test_j"
        props.ros2_control_active_joint_index = 0

        # Poll passes
        assert op.poll(bpy.context)

        res = op().execute(bpy.context)
        assert res == {"FINISHED"}
        assert len(props.ros2_control_joints) == 0

        props.ros2_control_joints.add().name = "test_j"
        props.ros2_control_active_joint_index = 5
        res = op().execute(bpy.context)
        assert res == {"CANCELLED"}

    def test_move_ros2_control_joint(self, scene, blender_context) -> None:
        """Test moving/reordering joints in list."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_joints.clear()

        op = LINKFORGE_OT_move_ros2_control_joint

        # Poll fails with <= 1 joints
        assert not op.poll(bpy.context)

        props.ros2_control_joints.add().name = "j1"
        props.ros2_control_joints.add().name = "j2"

        # Poll passes
        assert op.poll(bpy.context)

        # Move DOWN
        props.ros2_control_active_joint_index = 0
        o = op()
        o.direction = "DOWN"
        res = o.execute(bpy.context)
        assert res == {"FINISHED"}
        assert props.ros2_control_active_joint_index == 1

        # Move UP
        o.direction = "UP"
        res = o.execute(bpy.context)
        assert res == {"FINISHED"}
        assert props.ros2_control_active_joint_index == 0

        # Invalid move directions or boundaries
        o.direction = "UP"  # Cannot move up from index 0
        res = o.execute(bpy.context)
        assert res == {"CANCELLED"}

    def test_add_ros2_control_parameter(self, scene, blender_context) -> None:
        """Test adding parameter to global or joint list."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_parameters.clear()
        props.ros2_control_joints.clear()

        op = LINKFORGE_OT_add_ros2_control_parameter()
        op.target = "GLOBAL"
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        assert len(props.ros2_control_parameters) == 1
        assert props.ros2_control_parameters[0].name == "param"

        joint = props.ros2_control_joints.add()
        joint.name = "j1"

        joint.parameters = MockCollection(prop_type=MockPropertyGroup)
        props.ros2_control_active_joint_index = 0

        op.target = "JOINT"
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        assert len(joint.parameters) == 1
        assert joint.parameters[0].name == "param"

    def test_remove_ros2_control_parameter(self, scene, blender_context) -> None:
        """Test removing parameter from global or joint list."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_parameters.clear()
        props.ros2_control_joints.clear()

        p1 = props.ros2_control_parameters.add()
        p1.name = "g1"

        op = LINKFORGE_OT_remove_ros2_control_parameter()
        op.target = "GLOBAL"
        op.index = 0
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        assert len(props.ros2_control_parameters) == 0

        joint = props.ros2_control_joints.add()
        joint.name = "j1"

        joint.parameters = MockCollection(prop_type=MockPropertyGroup)
        jp1 = joint.parameters.add()
        jp1.name = "jp1"
        props.ros2_control_active_joint_index = 0

        op.target = "JOINT"
        op.index = 0
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        assert len(joint.parameters) == 0

    def test_purge_ros2_control_data(self, scene, blender_context) -> None:
        """Test purging all ros2_control data from scene."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_joints.add().name = "j1"
        props.ros2_control_parameters.add().name = "p1"

        res = LINKFORGE_OT_purge_ros2_control_data().execute(bpy.context)
        assert res == {"FINISHED"}
        assert len(props.ros2_control_joints) == 0
        assert len(props.ros2_control_parameters) == 0

    def test_prune_ros2_control_joints(self, scene, blender_context) -> None:
        """Test pruning orphaned joints from ros2_control."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_joints.clear()

        # Valid joint
        joint_obj = create_robot_joint("valid_j", None, None, scene)
        joint_props = safe_get_joint(joint_obj)
        joint_props.is_robot_joint = True
        joint_props.joint_name = "valid_j"

        item_valid = props.ros2_control_joints.add()
        item_valid.name = "valid_j"
        item_valid.joint_obj = joint_obj

        # Orphaned joint (None pointer)
        item_orphan = props.ros2_control_joints.add()
        item_orphan.name = "deleted_j"
        item_orphan.joint_obj = None

        # Orphaned joint (pointer exists but unlinked from scene.objects, as when deleted in viewport)
        unlinked_obj = create_robot_joint("unlinked_j", None, None, scene)
        item_unlinked = props.ros2_control_joints.add()
        item_unlinked.name = "unlinked_j"
        item_unlinked.joint_obj = unlinked_obj
        # Remove from scene.objects
        scene.objects.remove(unlinked_obj)

        assert len(props.ros2_control_joints) == 3

        # Test UI list draw_item for missing indicator
        ui_list = LINKFORGE_UL_ros2_control_joints()
        ui_list.layout_type = "DEFAULT"
        mock_layout = MagicMock()
        mock_row = MagicMock()
        mock_layout.row.return_value = mock_row

        ui_list.draw_item(
            bpy.context, mock_layout, props, item_orphan, None, props, "ros2_control_joints", 1
        )
        mock_row.label.assert_any_call(text="deleted_j (Missing)", icon="ERROR")

        ui_list.draw_item(
            bpy.context, mock_layout, props, item_unlinked, None, props, "ros2_control_joints", 2
        )
        mock_row.label.assert_any_call(text="unlinked_j (Missing)", icon="ERROR")

        # Poll should be True while missing joints exist
        assert LINKFORGE_OT_prune_ros2_control_joints.poll(bpy.context)

        # Execute prune
        res = LINKFORGE_OT_prune_ros2_control_joints().execute(bpy.context)
        assert res == {"FINISHED"}
        assert len(props.ros2_control_joints) == 1
        assert props.ros2_control_joints[0].name == "valid_j"

        # Poll should be False once all missing joints are pruned
        assert not LINKFORGE_OT_prune_ros2_control_joints.poll(bpy.context)

        # Direct execute when no missing joints remain handles gracefully
        res2 = LINKFORGE_OT_prune_ros2_control_joints().execute(bpy.context)
        assert res2 == {"FINISHED"}

    def test_control_ops_exception_handling(self, scene) -> None:
        """Verify control ops handle invalid context or registration issues."""
        op = LINKFORGE_OT_add_ros2_control_joint()
        op.joint_name = "invalid_joint"

        mock_ctx_no_scene = types.SimpleNamespace(scene=None)

        assert op.execute(mock_ctx_no_scene) == {"CANCELLED"}

        assert not LINKFORGE_OT_remove_ros2_control_joint.poll(mock_ctx_no_scene)
        assert not LINKFORGE_OT_move_ros2_control_joint.poll(mock_ctx_no_scene)
        assert not LINKFORGE_OT_add_ros2_control_parameter.poll(mock_ctx_no_scene)
        assert not LINKFORGE_OT_remove_ros2_control_parameter.poll(mock_ctx_no_scene)
        assert not LINKFORGE_OT_purge_ros2_control_data.poll(mock_ctx_no_scene)
        assert not LINKFORGE_OT_prune_ros2_control_joints.poll(mock_ctx_no_scene)

        assert LINKFORGE_OT_remove_ros2_control_joint().execute(mock_ctx_no_scene) == {"CANCELLED"}
        assert LINKFORGE_OT_move_ros2_control_joint().execute(mock_ctx_no_scene) == {"CANCELLED"}
        assert LINKFORGE_OT_add_ros2_control_parameter().execute(mock_ctx_no_scene) == {"CANCELLED"}
        assert LINKFORGE_OT_remove_ros2_control_parameter().execute(mock_ctx_no_scene) == {
            "CANCELLED"
        }
        assert LINKFORGE_OT_purge_ros2_control_data().execute(mock_ctx_no_scene) == {"CANCELLED"}
        assert LINKFORGE_OT_prune_ros2_control_joints().execute(mock_ctx_no_scene) == {"CANCELLED"}

    def test_control_ops_parameter_boundaries(self, scene, blender_context) -> None:
        """Test parameter addition and removal boundary cases."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_parameters.clear()
        props.ros2_control_joints.clear()

        op_add = LINKFORGE_OT_add_ros2_control_parameter()
        op_add.target = "JOINT"
        assert op_add.execute(bpy.context) == {"FINISHED"}

        # Remove parameter from JOINT but no active joints
        op_rem = LINKFORGE_OT_remove_ros2_control_parameter()
        op_rem.target = "JOINT"
        op_rem.index = 0
        assert op_rem.execute(bpy.context) == {"FINISHED"}

        joint = props.ros2_control_joints.add()
        joint.name = "j1"

        joint.parameters = MockCollection(prop_type=MockPropertyGroup)
        props.ros2_control_active_joint_index = 0

        # Remove parameter with target "JOINT" and negative index (should default to last item, but list is empty)
        op_rem.index = -1
        assert op_rem.execute(bpy.context) == {"FINISHED"}

        props.ros2_control_parameters.add().name = "g1"
        op_rem.target = "GLOBAL"
        op_rem.index = 10
        assert op_rem.execute(bpy.context) == {"FINISHED"}

    def test_control_ops_registration_and_main(self, mocker) -> None:
        """Verify registration and unregistration loops including double-registration."""
        control_ops.unregister()

        mock_reg = mocker.patch(
            "bpy.utils.register_class",
            side_effect=[ValueError("Already registered")] + [None] * 10,
        )
        mock_unreg = mocker.patch("bpy.utils.unregister_class")
        control_ops.register()
        assert mock_reg.call_count > 0
        assert mock_unreg.call_count > 0

        with patch.object(control_ops, "__name__", "__main__"):
            runpy.run_module("linkforge.blender.operators.control_ops")


class TestSensorOperations:
    def test_create_sensor(self, scene, blender_context) -> None:
        """Test creating a sensor for a robot link."""
        link_obj = create_test_object("link_obj", None, scene=scene)
        safe_get_linkforge(link_obj).is_robot_link = True

        sensor_obj = create_test_object("link_obj_sensor", None, scene=scene)
        sensor_obj.parent = link_obj
        safe_get_sensor(sensor_obj).is_robot_sensor = True

        assert "_sensor" in sensor_obj.name
        assert safe_get_sensor(sensor_obj).is_robot_sensor
        assert sensor_obj.parent == link_obj
