"""Unit tests for Blender Control (ROS 2) and Sensors."""

from __future__ import annotations

from unittest.mock import patch

import bpy
from linkforge.blender.operators.control_ops import (
    LINKFORGE_OT_add_ros2_control_joint,
    LINKFORGE_OT_add_ros2_control_parameter,
    LINKFORGE_OT_move_ros2_control_joint,
    LINKFORGE_OT_purge_ros2_control_data,
    LINKFORGE_OT_remove_ros2_control_joint,
    LINKFORGE_OT_remove_ros2_control_parameter,
)

from tests.blender_test_utils import (
    create_robot_joint,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_linkforge_scene,
    safe_get_sensor,
)


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

        # Create joint in scene
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

        # Execute again with same name (should fail/cancel)
        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}

    def test_remove_ros2_control_joint_poll_and_execute(self, scene, blender_context) -> None:
        """Test remove joint operator."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_joints.clear()

        op = LINKFORGE_OT_remove_ros2_control_joint

        # Poll fails when list is empty
        assert not op.poll(bpy.context)

        # Add joint to system
        item = props.ros2_control_joints.add()
        item.name = "test_j"
        props.ros2_control_active_joint_index = 0

        # Poll passes
        assert op.poll(bpy.context)

        # Execute successful remove
        res = op().execute(bpy.context)
        assert res == {"FINISHED"}
        assert len(props.ros2_control_joints) == 0

        # Execute with invalid active index
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

        # Add two joints
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

        # Add global param
        op = LINKFORGE_OT_add_ros2_control_parameter()
        op.target = "GLOBAL"
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        assert len(props.ros2_control_parameters) == 1
        assert props.ros2_control_parameters[0].name == "param"

        # Add joint param
        joint = props.ros2_control_joints.add()
        joint.name = "j1"
        from tests.mock_bpy_env import MockCollection, MockPropertyGroup

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

        # Setup global params
        p1 = props.ros2_control_parameters.add()
        p1.name = "g1"

        op = LINKFORGE_OT_remove_ros2_control_parameter()
        op.target = "GLOBAL"
        op.index = 0
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        assert len(props.ros2_control_parameters) == 0

        # Setup joint params
        joint = props.ros2_control_joints.add()
        joint.name = "j1"
        from tests.mock_bpy_env import MockCollection, MockPropertyGroup

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

    def test_control_ops_exception_handling(self, scene) -> None:
        """Verify control ops handle invalid context or registration issues."""
        op = LINKFORGE_OT_add_ros2_control_joint()
        op.joint_name = "invalid_joint"

        class MockContextNoScene:
            scene = None

        assert op.execute(MockContextNoScene()) == {"CANCELLED"}

        # Test poll methods with no scene
        from linkforge.blender.operators.control_ops import (
            LINKFORGE_OT_add_ros2_control_parameter,
            LINKFORGE_OT_move_ros2_control_joint,
            LINKFORGE_OT_purge_ros2_control_data,
            LINKFORGE_OT_remove_ros2_control_joint,
            LINKFORGE_OT_remove_ros2_control_parameter,
        )

        assert not LINKFORGE_OT_remove_ros2_control_joint.poll(MockContextNoScene())
        assert not LINKFORGE_OT_move_ros2_control_joint.poll(MockContextNoScene())
        assert not LINKFORGE_OT_add_ros2_control_parameter.poll(MockContextNoScene())
        assert not LINKFORGE_OT_remove_ros2_control_parameter.poll(MockContextNoScene())
        assert not LINKFORGE_OT_purge_ros2_control_data.poll(MockContextNoScene())

        # Test execute methods with no scene
        assert LINKFORGE_OT_remove_ros2_control_joint().execute(MockContextNoScene()) == {
            "CANCELLED"
        }
        assert LINKFORGE_OT_move_ros2_control_joint().execute(MockContextNoScene()) == {"CANCELLED"}
        assert LINKFORGE_OT_add_ros2_control_parameter().execute(MockContextNoScene()) == {
            "CANCELLED"
        }
        assert LINKFORGE_OT_remove_ros2_control_parameter().execute(MockContextNoScene()) == {
            "CANCELLED"
        }
        assert LINKFORGE_OT_purge_ros2_control_data().execute(MockContextNoScene()) == {"CANCELLED"}

    def test_control_ops_parameter_boundaries(self, scene, blender_context) -> None:
        """Test parameter addition and removal boundary cases."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_parameters.clear()
        props.ros2_control_joints.clear()

        # Add parameter with target "JOINT" but no active joints
        op_add = LINKFORGE_OT_add_ros2_control_parameter()
        op_add.target = "JOINT"
        assert op_add.execute(bpy.context) == {"FINISHED"}

        # Remove parameter from JOINT but no active joints
        op_rem = LINKFORGE_OT_remove_ros2_control_parameter()
        op_rem.target = "JOINT"
        op_rem.index = 0
        assert op_rem.execute(bpy.context) == {"FINISHED"}

        # Add a joint
        joint = props.ros2_control_joints.add()
        joint.name = "j1"
        from tests.mock_bpy_env import MockCollection, MockPropertyGroup

        joint.parameters = MockCollection(prop_type=MockPropertyGroup)
        props.ros2_control_active_joint_index = 0

        # Remove parameter with target "JOINT" and negative index (should default to last item, but list is empty)
        op_rem.index = -1
        assert op_rem.execute(bpy.context) == {"FINISHED"}

        # Add parameter globally, try to remove with index out of bounds
        props.ros2_control_parameters.add().name = "g1"
        op_rem.target = "GLOBAL"
        op_rem.index = 10
        assert op_rem.execute(bpy.context) == {"FINISHED"}

    def test_control_ops_registration_and_main(self, mocker) -> None:
        """Verify registration and unregistration loops including double-registration."""
        import linkforge.blender.operators.control_ops as control_ops

        # Test unregister
        control_ops.unregister()

        # Test register with ValueError fallback
        mock_reg = mocker.patch(
            "bpy.utils.register_class",
            side_effect=[ValueError("Already registered"), None, None, None, None, None, None],
        )
        mock_unreg = mocker.patch("bpy.utils.unregister_class")
        control_ops.register()
        assert mock_reg.call_count > 0
        assert mock_unreg.call_count > 0

        # Run __main__ entrypoint
        import runpy

        with patch.object(control_ops, "__name__", "__main__"):
            runpy.run_module("linkforge.blender.operators.control_ops")


class TestSensorOperations:
    def test_create_sensor(self, scene, blender_context) -> None:
        """Test creating a sensor for a robot link."""
        link_obj = create_test_object("link_obj", None, scene=scene)
        safe_get_linkforge(link_obj).is_robot_link = True

        # Create sensor object explicitly
        sensor_obj = create_test_object("link_obj_sensor", None, scene=scene)
        sensor_obj.parent = link_obj
        safe_get_sensor(sensor_obj).is_robot_sensor = True

        assert "_sensor" in sensor_obj.name
        assert safe_get_sensor(sensor_obj).is_robot_sensor
        assert sensor_obj.parent == link_obj
