"""Integration tests for Blender ROS 2 Control."""

from __future__ import annotations

import bpy
from linkforge.blender.operators.export_ops import LINKFORGE_OT_export_robot_model

from tests.blender_test_utils import (
    create_robot_joint,
    create_robot_link,
    safe_get_linkforge_scene,
    safe_update,
)


class TestRos2ControlIntegration:
    def test_ros2_control_joint_export(self, blender_clean_scene, tmp_path) -> None:
        """Verify that ros2_control joint configuration is exported correctly."""
        scene = bpy.context.scene
        lf_scene = safe_get_linkforge_scene(scene)
        lf_scene.use_ros2_control = True
        lf_scene.export_format = "URDF"
        lf_scene.xacro_split_files = False

        base = create_robot_link("base", scene)
        child = create_robot_link("child", scene)
        joint = create_robot_joint("joint1", base, child, scene)

        # Configure control for this joint
        rc_joint = lf_scene.ros2_control_joints.add()
        rc_joint.name = "joint1"
        rc_joint.cmd_position = True
        rc_joint.state_position = True
        rc_joint.state_velocity = True

        safe_update()

        export_path = tmp_path / "control_test.urdf"

        class MockExportOp:
            filepath = str(export_path)

            def report(self, level, message):
                pass

        res = LINKFORGE_OT_export_robot_model.execute(MockExportOp(), bpy.context)
        assert res == {"FINISHED"}

        urdf_content = export_path.read_text()
        assert "<ros2_control" in urdf_content
        assert '<joint name="joint1">' in urdf_content
        assert '<command_interface name="position" />' in urdf_content
        assert '<state_interface name="position" />' in urdf_content
        assert '<state_interface name="velocity" />' in urdf_content
