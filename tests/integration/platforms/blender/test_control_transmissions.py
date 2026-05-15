"""Integration tests for Blender ROS 2 Control and Transmissions."""

from __future__ import annotations

import bpy
from linkforge.blender.operators.export_ops import LINKFORGE_OT_export_robot_model
from linkforge.blender.operators.transmission_ops import LINKFORGE_OT_create_transmission

from tests.blender_test_utils import (
    create_robot_joint,
    create_robot_link,
    safe_get_linkforge_scene,
    safe_update,
)


class TestControlTransmissionsIntegration:
    def test_ros2_control_joint_export(self, blender_clean_scene, tmp_path) -> None:
        """Verify that ros2_control joint configuration is exported correctly."""
        scene = bpy.context.scene
        lf_scene = safe_get_linkforge_scene(scene)
        lf_scene.use_ros2_control = True
        lf_scene.export_format = "URDF"
        lf_scene.xacro_split_files = False

        # Add a joint
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

    def test_transmission_creation_and_export(self, blender_clean_scene, tmp_path) -> None:
        """Verify that a transmission linking two joints is correctly exported."""
        scene = bpy.context.scene

        # Setup 2-link chain with 2 joints (simplified)
        base = create_robot_link("base", scene)
        l1 = create_robot_link("l1", scene)
        l2 = create_robot_link("l2", scene)

        j1 = create_robot_joint("j1", base, l1, scene)
        j2 = create_robot_joint("j2", l1, l2, scene)

        safe_update()

        # Create transmission
        # We need to select one of the joints or the scene
        if bpy.context.view_layer:
            bpy.context.view_layer.objects.active = j1
        else:
            bpy.context.active_object = j1
        j1.select_set(True)

        res = LINKFORGE_OT_create_transmission.execute(None, bpy.context)
        assert res == {"FINISHED"}

        # Find the transmission object
        trans_obj = next((o for o in bpy.data.objects if "_trans" in o.name), None)
        assert trans_obj is not None

        from tests.blender_test_utils import safe_get_transmission

        t_props = safe_get_transmission(trans_obj)
        t_props.transmission_type = "simple"
        t_props.joint_name = j1
        t_props.mechanical_reduction = 50.0

        safe_update()

        export_path = tmp_path / "trans_test.urdf"

        class MockExportOp:
            filepath = str(export_path)

            def report(self, level, message):
                pass

        lf_scene = safe_get_linkforge_scene(scene)
        lf_scene.export_format = "URDF"
        lf_scene.xacro_split_files = False

        res = LINKFORGE_OT_export_robot_model.execute(MockExportOp(), bpy.context)
        assert res == {"FINISHED"}

        urdf_content = export_path.read_text()
        assert "<transmission" in urdf_content
        assert '<joint name="j1">' in urdf_content
        assert "<mechanicalReduction>50</mechanicalReduction>" in urdf_content
