"""Integration tests for Blender MoveIt 2 SRDF Planning Panel and Export/Import."""

from __future__ import annotations

import bpy
from linkforge.blender.adapters import core_to_blender
from linkforge.blender.adapters.context import BlenderContext
from linkforge.blender.core import SRDFParser, URDFParser
from linkforge.blender.operators.export_ops import LINKFORGE_OT_export_robot_model

from tests.blender_test_utils import (
    create_robot_joint,
    create_robot_link,
    safe_get_joint,
    safe_get_linkforge_scene,
    safe_update,
)


class TestSRDFPlanningIntegration:
    """End-to-end integration tests for SRDF planning in Blender."""

    def test_srdf_full_workflow_and_export_roundtrip(self, blender_clean_scene, tmp_path) -> None:
        """Verify SRDF planning operators, translation, companion .srdf export, and import."""
        scene = bpy.context.scene
        lf_scene = safe_get_linkforge_scene(scene)
        lf_scene.robot_name = "moveit_arm"
        lf_scene.export_format = "URDF"
        lf_scene.export_meshes = False
        lf_scene.use_ros2_control = False

        base = create_robot_link("base_link", scene)
        wrist = create_robot_link("wrist_link", scene)
        wrist.location = (0, 0, 0.5)
        finger = create_robot_link("finger_link", scene)
        finger.location = (0, 0, 0.8)

        j_arm = create_robot_joint("arm_joint", base, wrist, scene, joint_type="revolute")
        jp_arm = safe_get_joint(j_arm, scene)
        jp_arm.limit_lower = -1.57
        jp_arm.limit_upper = 1.57

        j_finger = create_robot_joint("finger_joint", wrist, finger, scene, joint_type="prismatic")
        jp_finger = safe_get_joint(j_finger, scene)
        jp_finger.limit_lower = 0.0
        jp_finger.limit_upper = 0.04

        safe_update()

        # 1. Detect virtual joint root link via operator
        assert bpy.ops.linkforge.detect_virtual_joint_root() == {"FINISHED"}
        assert lf_scene.virtual_joint_child_link == base

        # 2. Add 'arm' planning group (CHAIN mode) and 'hand' planning group (JOINTS mode)
        assert bpy.ops.linkforge.add_planning_group() == {"FINISHED"}
        assert lf_scene.use_srdf is True
        arm_grp = lf_scene.planning_groups[0]
        arm_grp.name = "arm"
        arm_grp.group_type = "CHAIN"
        arm_grp.base_link = base
        arm_grp.tip_link = wrist

        assert bpy.ops.linkforge.add_planning_group() == {"FINISHED"}
        hand_grp = lf_scene.planning_groups[1]
        hand_grp.name = "hand"
        hand_grp.group_type = "JOINTS"
        lf_scene.planning_active_group_index = 1
        assert bpy.ops.linkforge.add_planning_group_joint(joint_name="finger_joint") == {"FINISHED"}
        assert len(hand_grp.joints) == 1

        # 3. Add End Effector
        assert bpy.ops.linkforge.add_end_effector() == {"FINISHED"}
        ee = lf_scene.end_effectors[0]
        ee.name = "gripper"
        ee.group = "hand"
        ee.parent_link = wrist
        ee.parent_group = "arm"

        # 4. Add Group State for 'arm' and verify dynamic joint syncing + limit clamping
        lf_scene.planning_active_group_index = 0
        assert bpy.ops.linkforge.add_group_state() == {"FINISHED"}
        home_state = lf_scene.group_states[0]
        home_state.name = "home"
        assert home_state.group == "arm"
        assert len(home_state.joint_values) == 1
        assert home_state.joint_values[0].joint_name == "arm_joint"
        home_state.joint_values[0].value = 5.0  # clamped to upper limit 1.57
        assert abs(home_state.joint_values[0].value - 1.57) < 1e-4

        # 5. Auto-disable adjacent collisions
        assert bpy.ops.linkforge.auto_disable_adjacent_collisions() == {"FINISHED"}
        assert len(lf_scene.disabled_collisions) == 2

        # 6. Export URDF + companion SRDF
        export_urdf = tmp_path / "moveit_arm.urdf"
        export_srdf = tmp_path / "moveit_arm.srdf"

        class MockExportOp:
            filepath = str(export_urdf)

            def report(self, level, message):
                pass

        res = LINKFORGE_OT_export_robot_model.execute(MockExportOp(), bpy.context)
        assert res == {"FINISHED"}
        assert export_urdf.is_file()
        assert export_srdf.is_file()

        srdf_text = export_srdf.read_text()
        assert (
            '<virtual_joint name="world_joint" type="fixed" parent_frame="world" child_link="base_link"'
            in srdf_text
        )
        assert '<chain base_link="base_link" tip_link="wrist_link"' in srdf_text
        assert (
            '<end_effector name="gripper" group="hand" parent_link="wrist_link" parent_group="arm"'
            in srdf_text
        )
        assert '<group_state name="home" group="arm">' in srdf_text
        assert (
            '<disable_collisions link1="base_link" link2="wrist_link" reason="Adjacent"'
            in srdf_text
        )

        # 7. Parse exported URDF + SRDF and verify round-trip back into Blender scene
        parsed_robot = URDFParser().parse(export_urdf)
        parsed_robot.semantic = SRDFParser().parse(export_srdf, robot=parsed_robot)

        assert bpy.ops.linkforge.purge_planning_data() == {"FINISHED"}
        assert lf_scene.use_srdf is False
        assert len(lf_scene.planning_groups) == 0

        core_to_blender.import_robot_to_scene(parsed_robot, export_urdf, BlenderContext())
        assert lf_scene.use_srdf is True
        assert len(lf_scene.planning_groups) == 2
        assert len(lf_scene.end_effectors) == 1
        assert len(lf_scene.group_states) == 1
        assert len(lf_scene.disabled_collisions) == 2
