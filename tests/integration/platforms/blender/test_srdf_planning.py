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

        # 1. Configure virtual joint root link
        lf_scene.use_virtual_joint = True
        lf_scene.virtual_joint_child_link = base
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

    def test_pose_preview_and_urdf_coordinate_preservation(
        self, blender_clean_scene, tmp_path
    ) -> None:
        """Verify pose preview articulates joints, viewport capture works, and URDF is never drifted."""
        scene = bpy.context.scene
        lf_scene = safe_get_linkforge_scene(scene)
        lf_scene.robot_name = "preview_bot"
        lf_scene.export_format = "URDF"
        lf_scene.export_meshes = False
        lf_scene.use_ros2_control = False

        base = create_robot_link("base_link", scene)
        wrist = create_robot_link("wrist_link", scene)
        wrist.location = (0, 0, 0.5)

        j_arm = create_robot_joint("arm_joint", base, wrist, scene, joint_type="revolute")
        jp_arm = safe_get_joint(j_arm, scene)
        jp_arm.limit_lower = -3.14
        jp_arm.limit_upper = 3.14

        safe_update()

        # 1. Setup planning group and group state
        assert bpy.ops.linkforge.add_planning_group() == {"FINISHED"}
        arm_grp = lf_scene.planning_groups[0]
        arm_grp.name = "arm"
        arm_grp.group_type = "CHAIN"
        arm_grp.base_link = base
        arm_grp.tip_link = wrist

        lf_scene.planning_active_group_index = 0
        assert bpy.ops.linkforge.add_group_state() == {"FINISHED"}
        state = lf_scene.group_states[0]
        state.name = "bent_pose"
        state.joint_values[0].value = 1.25

        # 2. Export baseline URDF at pristine rest pose
        rest_urdf = tmp_path / "rest.urdf"

        class MockRestExport:
            filepath = str(rest_urdf)

            def report(self, level, message):
                pass

        assert LINKFORGE_OT_export_robot_model.execute(MockRestExport(), bpy.context) == {
            "FINISHED"
        }
        rest_content = rest_urdf.read_text()

        # 3. Toggle group state preview (articulates j_arm in the 3D scene)
        assert bpy.ops.linkforge.toggle_group_state_preview(group_state_name="bent_pose") == {
            "FINISHED"
        }
        assert state.is_previewing is True
        assert lf_scene.preview_active_state_name == "bent_pose"
        assert abs(j_arm.matrix_local.to_euler().z - 1.25) < 1e-3

        # 4. Export robot while preview is actively engaged in 3D viewport
        preview_urdf = tmp_path / "during_preview.urdf"

        class MockPreviewExport:
            filepath = str(preview_urdf)

            def report(self, level, message):
                pass

        assert LINKFORGE_OT_export_robot_model.execute(MockPreviewExport(), bpy.context) == {
            "FINISHED"
        }
        # CRITICAL: URDF coordinate frames and joint origins must be 100% identical!
        assert preview_urdf.read_text() == rest_content
        # Rest pose must be automatically restored after export
        assert state.is_previewing is False
        assert lf_scene.preview_active_state_name == ""
        assert abs(j_arm.matrix_local.to_euler().z) < 1e-4

        # 5. Test direct 3D viewport capture
        from linkforge.blender.utils.pose_preview import cache_rest_transform
        from mathutils import Matrix

        cache_rest_transform(j_arm)
        j_arm.matrix_local = Matrix.Rotation(0.85, 4, "Z")
        safe_update()
        assert bpy.ops.linkforge.capture_group_state_pose() == {"FINISHED"}
        assert abs(state.joint_values[0].value - 0.85) < 1e-3
        # Restore rest pose cleanly
        assert bpy.ops.linkforge.restore_rest_pose() == {"FINISHED"}

    def test_subgroups_and_passive_joints_roundtrip(self, blender_clean_scene, tmp_path) -> None:
        """Verify subgroups and passive joints export to SRDF/ros2_control and roundtrip into scene."""
        scene = bpy.context.scene
        lf_scene = safe_get_linkforge_scene(scene)
        lf_scene.robot_name = "subgroup_bot"
        lf_scene.export_format = "URDF"
        lf_scene.export_meshes = False
        lf_scene.use_ros2_control = True

        base = create_robot_link("base_link", scene)
        arm = create_robot_link("arm_link", scene)
        arm.location = (0, 0, 0.5)
        hand = create_robot_link("hand_link", scene)
        hand.location = (0, 0, 1.0)
        finger = create_robot_link("finger_link", scene)
        finger.location = (0, 0, 1.2)

        j_arm = create_robot_joint("arm_joint", base, arm, scene, joint_type="revolute")
        safe_get_joint(j_arm, scene).limit_lower = -3.14
        safe_get_joint(j_arm, scene).limit_upper = 3.14

        _j_hand = create_robot_joint("hand_mount", arm, hand, scene, joint_type="fixed")

        j_finger = create_robot_joint("finger_joint", hand, finger, scene, joint_type="prismatic")
        jp_finger = safe_get_joint(j_finger, scene)
        jp_finger.limit_lower = 0.0
        jp_finger.limit_upper = 0.05
        # Mark joint as passive
        jp_finger.is_passive = True

        safe_update()

        # Add ROS 2 control interfaces for joints
        assert bpy.ops.linkforge.add_ros2_control_joint(joint_name="arm_joint") == {"FINISHED"}
        assert bpy.ops.linkforge.add_ros2_control_joint(joint_name="finger_joint") == {"FINISHED"}

        # Groups: arm (CHAIN), gripper (JOINTS), robot_system (SUBGROUPS)
        assert bpy.ops.linkforge.add_planning_group() == {"FINISHED"}
        grp_arm = lf_scene.planning_groups[0]
        grp_arm.name = "arm"
        grp_arm.group_type = "CHAIN"
        grp_arm.base_link = base
        grp_arm.tip_link = arm

        assert bpy.ops.linkforge.add_planning_group() == {"FINISHED"}
        grp_grip = lf_scene.planning_groups[1]
        grp_grip.name = "gripper"
        grp_grip.group_type = "JOINTS"
        lf_scene.planning_active_group_index = 1
        assert bpy.ops.linkforge.add_planning_group_joint(joint_name="finger_joint") == {"FINISHED"}

        assert bpy.ops.linkforge.add_planning_group() == {"FINISHED"}
        grp_sys = lf_scene.planning_groups[2]
        grp_sys.name = "robot_system"
        grp_sys.group_type = "SUBGROUPS"
        lf_scene.planning_active_group_index = 2
        assert bpy.ops.linkforge.add_planning_subgroup(subgroup_name="arm") == {"FINISHED"}
        assert bpy.ops.linkforge.add_planning_subgroup(subgroup_name="gripper") == {"FINISHED"}
        assert len(grp_sys.subgroups) == 2

        # Export URDF + companion SRDF
        export_urdf = tmp_path / "subgroup_bot.urdf"
        export_srdf = tmp_path / "subgroup_bot.srdf"

        class MockExportOp:
            filepath = str(export_urdf)

            def report(self, level, message):
                pass

        assert LINKFORGE_OT_export_robot_model.execute(MockExportOp(), bpy.context) == {"FINISHED"}

        urdf_text = export_urdf.read_text()
        srdf_text = export_srdf.read_text()

        # Verify SRDF: passive joint and subgroups
        assert '<passive_joint name="finger_joint"' in srdf_text
        assert '<group name="robot_system"' in srdf_text
        assert '<group name="arm"' in srdf_text
        assert '<group name="gripper"' in srdf_text

        # Verify ros2_control in URDF: finger_joint is passive so NO command interface!
        assert '<joint name="arm_joint">' in urdf_text
        assert '<command_interface name="position"' in urdf_text
        assert '<joint name="finger_joint">' in urdf_text
        # Passive joint must not have command interfaces
        assert '<joint name="finger_joint">\n      <command_interface' not in urdf_text

        # Parse and import back into scene
        parsed_robot = URDFParser().parse(export_urdf)
        parsed_robot.semantic = SRDFParser().parse(export_srdf, robot=parsed_robot)

        assert bpy.ops.linkforge.purge_planning_data() == {"FINISHED"}
        core_to_blender.import_robot_to_scene(parsed_robot, export_urdf, BlenderContext())

        # Verify imported properties in Blender scene
        assert lf_scene.use_srdf is True
        assert len(lf_scene.planning_groups) == 3
        imp_sys = next(g for g in lf_scene.planning_groups if g.name == "robot_system")
        assert imp_sys.group_type == "SUBGROUPS"
        sub_names = {sg.name for sg in imp_sys.subgroups}
        assert sub_names == {"arm", "gripper"}

        # Verify passive flag restored on finger_joint
        new_finger_joint = bpy.data.objects.get("finger_joint")
        assert new_finger_joint is not None
        assert safe_get_joint(new_finger_joint, scene).is_passive is True

    def test_sampled_collision_matrix_acm(self, blender_clean_scene) -> None:
        """Verify Monte Carlo kinematic collision sampler in live Blender scene."""
        from linkforge.blender.properties.planning_props import get_planning_link_name

        scene = bpy.context.scene
        lf_scene = safe_get_linkforge_scene(scene)
        lf_scene.robot_name = "acm_bot"

        l1 = create_robot_link(
            "link_1", scene, with_visual=True, with_collision=True, with_cube=True
        )
        l2 = create_robot_link(
            "link_2", scene, with_visual=True, with_collision=True, with_cube=True
        )
        l2.location = (0, 0, 1.0)
        l3 = create_robot_link(
            "link_3", scene, with_visual=True, with_collision=True, with_cube=True
        )
        l3.location = (0, 0, 2.0)

        j1 = create_robot_joint("j1", l1, l2, scene, joint_type="revolute")
        safe_get_joint(j1, scene).limit_lower = -1.57
        safe_get_joint(j1, scene).limit_upper = 1.57

        j2 = create_robot_joint("j2", l2, l3, scene, joint_type="revolute")
        safe_get_joint(j2, scene).limit_lower = -1.57
        safe_get_joint(j2, scene).limit_upper = 1.57

        safe_update()

        # Run collision matrix generator
        res = bpy.ops.linkforge.compute_collision_matrix(
            sample_count=50,
            clear_existing=True,
        )
        assert res == {"FINISHED"}
        assert len(lf_scene.disabled_collisions) > 0

        # Link 1 & Link 2 are adjacent
        reasons = {
            frozenset(
                [
                    get_planning_link_name(dc.link1, dc.link1_name),
                    get_planning_link_name(dc.link2, dc.link2_name),
                ]
            ): dc.reason
            for dc in lf_scene.disabled_collisions
        }
        assert frozenset(["link_1", "link_2"]) in reasons
        assert reasons[frozenset(["link_1", "link_2"])] == "Adjacent"
        assert frozenset(["link_2", "link_3"]) in reasons
        assert reasons[frozenset(["link_2", "link_3"])] == "Adjacent"
