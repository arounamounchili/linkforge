"""Comprehensive unit tests for Blender SRDF Planning properties, operators, panels, and adapters."""

from __future__ import annotations

import runpy
import types
import typing
from pathlib import Path
from unittest.mock import MagicMock, patch

import bpy
from linkforge.blender.adapters import blender_to_core, core_to_blender, translator
from linkforge.blender.adapters.context import BlenderContext
from linkforge.blender.core.models.robot import Robot
from linkforge.blender.core.models.srdf import (
    Chain,
    CollisionPair,
    EndEffector,
    GroupState,
    PlanningGroup,
    SemanticRobotDescription,
    VirtualJoint,
)
from linkforge.blender.core.validation import ValidationResult
from linkforge.blender.logic.asynchronous_builder import AsynchronousRobotBuilder
from linkforge.blender.operators import planning_ops
from linkforge.blender.operators.export_ops import LINKFORGE_OT_export_robot_model
from linkforge.blender.operators.import_ops import LINKFORGE_OT_import_robot_model
from linkforge.blender.operators.planning_ops import (
    LINKFORGE_OT_add_disabled_collision,
    LINKFORGE_OT_add_end_effector,
    LINKFORGE_OT_add_group_state,
    LINKFORGE_OT_add_planning_group,
    LINKFORGE_OT_add_planning_group_joint,
    LINKFORGE_OT_auto_disable_adjacent_collisions,
    LINKFORGE_OT_detect_virtual_joint_root,
    LINKFORGE_OT_move_disabled_collision,
    LINKFORGE_OT_move_end_effector,
    LINKFORGE_OT_move_group_state,
    LINKFORGE_OT_move_planning_group,
    LINKFORGE_OT_purge_planning_data,
    LINKFORGE_OT_remove_disabled_collision,
    LINKFORGE_OT_remove_end_effector,
    LINKFORGE_OT_remove_group_state,
    LINKFORGE_OT_remove_planning_group,
    LINKFORGE_OT_remove_planning_group_joint,
    LINKFORGE_OT_reset_group_state_values,
    LINKFORGE_OT_sync_group_state_joints,
)
from linkforge.blender.panels import planning_panel
from linkforge.blender.panels.planning_panel import (
    LINKFORGE_MT_add_planning_group_joint,
    LINKFORGE_PT_planning,
    LINKFORGE_UL_disabled_collisions,
    LINKFORGE_UL_end_effectors,
    LINKFORGE_UL_group_states,
    LINKFORGE_UL_planning_groups,
)
from linkforge.blender.properties import planning_props
from linkforge.blender.properties.planning_props import (
    GroupStateJointValueProperty,
    GroupStateProperty,
    PlanningGroupProperty,
    get_planning_joint_name,
    get_planning_link_name,
    has_planning_data,
    poll_planning_joint,
    poll_planning_link,
    resolve_planning_group_joints,
    sync_group_state_joints,
    update_collision_pair_links,
    update_end_effector_link,
    update_group_state_group,
    update_group_state_joint_value,
    update_planning_group_name,
    update_planning_group_topology,
    update_virtual_joint_child_link,
)

from tests.blender_test_utils import (
    create_robot_joint,
    create_robot_link,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_linkforge_scene,
)


def _setup_3_link_arm(scene: typing.Any) -> tuple[typing.Any, ...]:
    """Create base_link -> j1 -> link1 -> j2 -> link2 in the scene."""
    base = create_robot_link("base_link", scene, with_visual=False, with_collision=False)
    safe_get_linkforge(base, scene).link_name = "base_link"

    link1 = create_robot_link("link1", scene, with_visual=False, with_collision=False)
    safe_get_linkforge(link1, scene).link_name = "link1"

    link2 = create_robot_link("link2", scene, with_visual=False, with_collision=False)
    safe_get_linkforge(link2, scene).link_name = "link2"

    j1 = create_robot_joint("j1", base, link1, scene, joint_type="REVOLUTE")
    jp1 = safe_get_joint(j1, scene)
    jp1.joint_name = "j1"
    jp1.limit_lower = -1.5
    jp1.limit_upper = 1.5

    j2 = create_robot_joint("j2", link1, link2, scene, joint_type="REVOLUTE")
    jp2 = safe_get_joint(j2, scene)
    jp2.joint_name = "j2"
    jp2.limit_lower = -2.0
    jp2.limit_upper = 2.0

    return base, link1, link2, j1, j2


class TestPlanningPropsHelpers:
    """Test helper functions and callbacks in planning_props.py."""

    def test_polls_and_name_getters(self, scene) -> None:
        link_obj = create_robot_link("base", scene, with_visual=False, with_collision=False)
        safe_get_linkforge(link_obj, scene).link_name = "custom_base"

        joint_obj = create_robot_joint("j1", link_obj, link_obj, scene)
        safe_get_joint(joint_obj, scene).joint_name = "custom_j1"

        plain_obj = create_test_object("plain", None, scene=scene)
        visual_child = create_test_object("base_visual", None, scene=scene)
        visual_child.parent = link_obj

        assert poll_planning_link(None, link_obj) is True
        assert poll_planning_link(None, visual_child) is False
        assert poll_planning_link(None, plain_obj) is False
        assert poll_planning_joint(None, joint_obj) is True
        assert poll_planning_joint(None, plain_obj) is False

        assert get_planning_link_name(link_obj) == "custom_base"
        assert get_planning_link_name(visual_child) == "custom_base"
        assert get_planning_link_name(None, "fallback_link") == "fallback_link"
        assert get_planning_joint_name(joint_obj) == "custom_j1"
        assert get_planning_joint_name(None, "fallback_joint") == "fallback_joint"

        # Object without link/joint props falls back to obj.name
        safe_get_linkforge(plain_obj, scene).link_name = ""
        with patch("linkforge.blender.properties.planning_props.get_link_props", return_value=None):
            assert get_planning_link_name(plain_obj) == "plain"
        with patch(
            "linkforge.blender.properties.planning_props.get_joint_props", return_value=None
        ):
            assert get_planning_joint_name(plain_obj) == "plain"

    def test_has_planning_data_states(self, scene) -> None:
        assert has_planning_data(None) is False
        props = safe_get_linkforge_scene(scene)
        assert has_planning_data(props) is False

        # Virtual joint via name fallback
        props.use_virtual_joint = True
        props.virtual_joint_child_link_name = "base_link"
        assert has_planning_data(props) is True

        # Virtual joint disabled
        props.use_virtual_joint = False
        assert has_planning_data(props) is False

        # Virtual joint via object pointer
        link_obj = create_robot_link("base", scene, with_visual=False, with_collision=False)
        props.use_virtual_joint = True
        props.virtual_joint_child_link_name = ""
        props.virtual_joint_child_link = link_obj
        assert has_planning_data(props) is True
        props.virtual_joint_child_link = None

        # Planning groups / end effectors / group states / disabled collisions
        props.planning_groups.add().name = "arm"
        assert has_planning_data(props) is True
        props.planning_groups.clear()

        props.end_effectors.add().name = "ee"
        assert has_planning_data(props) is True
        props.end_effectors.clear()

        props.group_states.add().name = "home"
        assert has_planning_data(props) is True
        props.group_states.clear()

        props.disabled_collisions.add().name = "pair"
        assert has_planning_data(props) is True
        props.disabled_collisions.clear()

    def test_resolve_planning_group_joints_chain_and_joints_modes(self, scene) -> None:
        base, link1, link2, j1, j2 = _setup_3_link_arm(scene)

        # Add a fixed joint and a mimic joint to ensure they are skipped
        link3 = create_robot_link("link3", scene, with_visual=False, with_collision=False)
        safe_get_linkforge(link3, scene).link_name = "link3"
        j_fixed = create_robot_joint("j_fixed", link2, link3, scene, joint_type="FIXED")
        safe_get_joint(j_fixed, scene).joint_name = "j_fixed"

        # Non-joint object and incomplete joint in scene
        create_test_object("random_empty", None, scene=scene)
        j_incomplete = create_robot_joint("j_inc", None, None, scene)
        safe_get_joint(j_incomplete, scene).is_robot_joint = True

        grp = PlanningGroupProperty()
        grp.group_type = "CHAIN"
        grp.base_link = base
        grp.tip_link = link3

        assert resolve_planning_group_joints(None, grp) == []
        resolved = resolve_planning_group_joints(scene, grp)
        assert [name for name, _ in resolved] == ["j1", "j2"]

        # Unreachable chain (reversed base and tip)
        grp.base_link = link2
        grp.tip_link = base
        assert resolve_planning_group_joints(scene, grp) == []

        # Same base and tip
        grp.base_link = base
        grp.tip_link = base
        assert resolve_planning_group_joints(scene, grp) == []

        # Cycle guard in chain resolution
        safe_get_joint(j1, scene).parent_link = link2
        grp.base_link = link3
        grp.tip_link = link2
        assert resolve_planning_group_joints(scene, grp) == []
        safe_get_joint(j1, scene).parent_link = base

        # JOINTS mode (with object pointer, name fallback, duplicate, and fixed joint)
        grp.group_type = "JOINTS"
        item1 = grp.joints.add()
        item1.name = "j1"
        item1.joint_obj = j1

        item2 = grp.joints.add()
        item2.name = "j2"
        item2.joint_obj = None  # resolved by name from scene

        item_dup = grp.joints.add()
        item_dup.name = "j1"
        item_dup.joint_obj = j1

        item_fixed = grp.joints.add()
        item_fixed.name = "j_fixed"
        item_fixed.joint_obj = j_fixed

        item_ghost = grp.joints.add()
        item_ghost.name = "ghost_joint"
        item_ghost.joint_obj = None

        item_empty_name = grp.joints.add()
        item_empty_name.name = ""
        item_empty_name.joint_obj = None

        resolved_j = resolve_planning_group_joints(scene, grp)
        assert [name for name, _ in resolved_j] == ["j1", "j2", "ghost_joint"]

    def test_update_group_state_joint_value_clamping(self, scene) -> None:
        base, link1, _, j1, _ = _setup_3_link_arm(scene)
        jv = GroupStateJointValueProperty()

        # No joint_obj -> no-op
        jv.value = 5.0
        update_group_state_joint_value(jv, bpy.context)
        assert jv.value == 5.0

        # Non-robot joint -> no-op
        plain = create_test_object("plain", None, scene=scene)
        jv.joint_obj = plain
        update_group_state_joint_value(jv, bpy.context)
        assert jv.value == 5.0

        # Valid revolute joint with limits [-1.5, 1.5]
        jv.joint_obj = j1
        jv.value = 3.0
        update_group_state_joint_value(jv, bpy.context)
        assert jv.value == 1.5

        jv.value = -3.0
        update_group_state_joint_value(jv, bpy.context)
        assert jv.value == -1.5

        jv.value = 0.5
        update_group_state_joint_value(jv, bpy.context)
        assert jv.value == 0.5

        # Inverted limits (lower > upper) and continuous joint without limits -> no clamp
        jp1 = safe_get_joint(j1, scene)
        jp1.limit_lower = 2.0
        jp1.limit_upper = -2.0
        jv.value = 5.0
        update_group_state_joint_value(jv, bpy.context)
        assert jv.value == 5.0

        jp1.joint_type = "CONTINUOUS"
        jp1.use_limits = False
        update_group_state_joint_value(jv, bpy.context)
        assert jv.value == 5.0

    def test_sync_group_state_joints_and_callback(self, scene) -> None:
        base, _, link2, _, _ = _setup_3_link_arm(scene)
        props = safe_get_linkforge_scene(scene)

        grp = props.planning_groups.add()
        grp.name = "arm"
        grp.group_type = "CHAIN"
        grp.base_link = base
        grp.tip_link = link2

        state = GroupStateProperty()
        state.name = "home"

        assert sync_group_state_joints(state, None) == 0
        assert sync_group_state_joints(state, scene) == 0  # empty group name

        state.group = "missing_group"
        assert sync_group_state_joints(state, scene) == 0

        # Trigger via update callback on valid group
        state.group = "arm"
        update_group_state_group(state, bpy.context)
        assert len(state.joint_values) == 2
        assert state.joint_values[0].joint_name == "j1"
        assert state.joint_values[1].joint_name == "j2"

        # Preserve existing values on re-sync
        state.joint_values[0].value = 0.75
        count = sync_group_state_joints(state, scene)
        assert count == 2
        assert state.joint_values[0].value == 0.75

        # Empty group in callback -> no-op
        state.group = ""
        update_group_state_group(state, None)

        # Test update_planning_group_topology and update_planning_group_name
        scene_state = props.group_states.add()
        scene_state.name = "pose1"
        scene_state.group = "arm"
        other_state = props.group_states.add()
        other_state.name = "pose2"
        other_state.group = "other"

        ee1 = props.end_effectors.add()
        ee1.name = "ee1"
        ee1.group = "arm"
        ee1.parent_group = "arm"
        ee2 = props.end_effectors.add()
        ee2.name = "ee2"
        ee2.group = "other"
        ee2.parent_group = "other"

        # Changing tip_link triggers update_planning_group_topology
        grp.tip_link = base
        update_planning_group_topology(grp, bpy.context)
        assert len(scene_state.joint_values) == 0
        grp.tip_link = link2
        update_planning_group_topology(grp, None)
        assert len(scene_state.joint_values) == 2

        # Renaming group propagates to End Effectors and Group States
        grp.previous_name = "arm"
        grp.name = "manipulator"
        update_planning_group_name(grp, bpy.context)
        assert ee1.group == "manipulator"
        assert ee1.parent_group == "manipulator"
        assert ee2.group == "other"
        assert ee2.parent_group == "other"
        assert scene_state.group == "manipulator"
        assert other_state.group == "other"
        assert grp.previous_name == "manipulator"

        # Calling when name hasn't changed is a no-op
        update_planning_group_name(grp, bpy.context)

        # Null scene branches in callbacks
        with patch.object(bpy.context, "scene", None):
            update_planning_group_topology(grp, None)
            grp.previous_name = "manipulator"
            grp.name = "arm2"
            update_planning_group_name(grp, None)

        # Visual child mesh auto-redirection to parent link
        base_visual = create_test_object("base_visual", None, scene=scene)
        base_visual.parent = base
        props.virtual_joint_child_link = base_visual
        update_virtual_joint_child_link(props, bpy.context)
        assert props.virtual_joint_child_link == base
        update_virtual_joint_child_link(props, bpy.context)

        grp.base_link = base_visual
        update_planning_group_topology(grp, bpy.context)
        assert grp.base_link == base

        ee1.parent_link = base_visual
        update_end_effector_link(ee1, bpy.context)
        assert ee1.parent_link == base
        update_end_effector_link(ee1, bpy.context)

        dc = props.disabled_collisions.add()
        dc.link1 = base_visual
        update_collision_pair_links(dc, bpy.context)
        assert dc.link1 == base
        dc.link2 = base_visual
        update_collision_pair_links(dc, bpy.context)
        assert dc.link2 == base
        update_collision_pair_links(dc, bpy.context)

    def test_safe_redirect_pointer_gui_timer(self, scene) -> None:
        from linkforge.blender.properties.planning_props import _safe_redirect_pointer

        class DummyOwner:
            target = None

        owner = DummyOwner()
        target_obj = create_test_object("dummy_target", None, scene=scene)
        callbacks = []

        class MockTimers:
            @staticmethod
            def register(fn, first_interval=0.0):
                callbacks.append(fn)

        with (
            patch.object(bpy.app, "background", False, create=True),
            patch.object(bpy.app, "timers", MockTimers, create=True),
        ):
            _safe_redirect_pointer(owner, "target", target_obj)
            assert len(callbacks) == 1
            assert callbacks[0]() is None
            assert owner.target == target_obj

            class BrokenOwner:
                def __setattr__(self, name, val):
                    raise ReferenceError("Struct deleted")

            broken = BrokenOwner()
            _safe_redirect_pointer(broken, "target", target_obj)
            assert len(callbacks) == 2
            assert callbacks[1]() is None

    def test_planning_props_registration(self) -> None:
        planning_props.register()
        planning_props.unregister()
        planning_props.register()
        calls = 0

        def _raise_once(_cls):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise ValueError("already registered")

        with (
            patch.object(bpy.utils, "register_class", side_effect=_raise_once),
            patch.object(bpy.utils, "unregister_class"),
        ):
            planning_props.register()
        with patch.object(bpy.utils, "register_class"):
            runpy.run_module("linkforge.blender.properties.planning_props", run_name="__main__")


class TestPlanningOperators:
    """Test all operators in planning_ops.py."""

    def test_detect_virtual_joint_root(self, scene) -> None:
        props = safe_get_linkforge_scene(scene)
        assert LINKFORGE_OT_detect_virtual_joint_root.poll(bpy.context)

        # Empty scene -> CANCELLED
        op = LINKFORGE_OT_detect_virtual_joint_root()
        assert op.execute(bpy.context) == {"CANCELLED"}

        # With kinematic tree -> detects base_link
        base, _, _, _, _ = _setup_3_link_arm(scene)
        assert op.execute(bpy.context) == {"FINISHED"}
        assert props.virtual_joint_child_link == base
        assert props.virtual_joint_child_link_name == "base_link"
        assert props.use_virtual_joint is True

        # Missing scene
        assert op.execute(types.SimpleNamespace(scene=None)) == {"CANCELLED"}

    def test_planning_group_crud_and_move(self, scene) -> None:
        props = safe_get_linkforge_scene(scene)
        props.planning_groups.clear()

        assert not LINKFORGE_OT_remove_planning_group.poll(bpy.context)
        assert not LINKFORGE_OT_move_planning_group.poll(bpy.context)

        # Add groups
        assert LINKFORGE_OT_add_planning_group.poll(bpy.context)
        assert LINKFORGE_OT_add_planning_group().execute(bpy.context) == {"FINISHED"}
        assert props.planning_groups[0].name == "group"
        assert props.use_srdf is True

        assert LINKFORGE_OT_add_planning_group().execute(bpy.context) == {"FINISHED"}
        assert props.planning_groups[1].name == "group_2"
        assert LINKFORGE_OT_add_planning_group().execute(bpy.context) == {"FINISHED"}
        assert props.planning_groups[2].name == "group_3"
        assert LINKFORGE_OT_add_planning_group().execute(bpy.context) == {"FINISHED"}
        assert props.planning_groups[3].name == "group_4"

        # Move UP and DOWN
        assert LINKFORGE_OT_move_planning_group.poll(bpy.context)
        props.planning_active_group_index = 1
        mv = LINKFORGE_OT_move_planning_group()
        mv.direction = "UP"
        assert mv.execute(bpy.context) == {"FINISHED"}
        assert props.planning_active_group_index == 0

        mv.direction = "UP"
        assert mv.execute(bpy.context) == {"CANCELLED"}

        mv.direction = "DOWN"
        assert mv.execute(bpy.context) == {"FINISHED"}
        assert props.planning_active_group_index == 1

        # Out-of-range remove -> CANCELLED
        props.planning_active_group_index = 99
        assert LINKFORGE_OT_remove_planning_group().execute(bpy.context) == {"CANCELLED"}

        # Remove all groups -> auto-disables use_srdf when no other planning data exists
        while len(props.planning_groups) > 0:
            props.planning_active_group_index = 0
            assert LINKFORGE_OT_remove_planning_group().execute(bpy.context) == {"FINISHED"}
        assert props.use_srdf is False

    def test_planning_group_joints_add_and_remove(self, scene) -> None:
        _, _, _, j1, j2 = _setup_3_link_arm(scene)
        props = safe_get_linkforge_scene(scene)
        props.planning_groups.clear()

        assert not LINKFORGE_OT_add_planning_group_joint.poll(bpy.context)
        assert not LINKFORGE_OT_remove_planning_group_joint.poll(bpy.context)

        LINKFORGE_OT_add_planning_group().execute(bpy.context)
        grp = props.planning_groups[0]
        grp.group_type = "JOINTS"

        # Add a group state referencing this group and one referencing another group
        st = props.group_states.add()
        st.name = "open"
        st.group = grp.name
        st_other = props.group_states.add()
        st_other.name = "other_pose"
        st_other.group = "other_group"

        assert LINKFORGE_OT_add_planning_group_joint.poll(bpy.context)
        add_op = LINKFORGE_OT_add_planning_group_joint()
        add_op.joint_name = "j1"
        assert add_op.execute(bpy.context) == {"FINISHED"}
        assert len(grp.joints) == 1
        assert grp.joints[0].joint_obj == j1
        assert len(st.joint_values) == 1

        # Add second joint (tests loop over existing joints when not matching)
        add_op2 = LINKFORGE_OT_add_planning_group_joint()
        add_op2.joint_name = "j2"
        assert add_op2.execute(bpy.context) == {"FINISHED"}
        assert len(grp.joints) == 2

        # Duplicate joint -> CANCELLED
        assert add_op.execute(bpy.context) == {"CANCELLED"}

        # Remove joints
        assert LINKFORGE_OT_remove_planning_group_joint.poll(bpy.context)
        rem_op = LINKFORGE_OT_remove_planning_group_joint()
        rem_op.index = 1
        assert rem_op.execute(bpy.context) == {"FINISHED"}
        rem_op.index = 0
        assert rem_op.execute(bpy.context) == {"FINISHED"}
        assert len(grp.joints) == 0
        assert len(st.joint_values) == 0

        # Invalid index remove -> CANCELLED
        rem_op.index = 10
        assert rem_op.execute(bpy.context) == {"CANCELLED"}

        # Invalid active group index -> CANCELLED
        props.planning_active_group_index = 10
        assert not LINKFORGE_OT_remove_planning_group_joint.poll(bpy.context)
        assert add_op.execute(bpy.context) == {"CANCELLED"}
        assert rem_op.execute(bpy.context) == {"CANCELLED"}

    def test_end_effector_crud_and_move(self, scene) -> None:
        props = safe_get_linkforge_scene(scene)
        props.end_effectors.clear()

        assert not LINKFORGE_OT_remove_end_effector.poll(bpy.context)
        assert not LINKFORGE_OT_move_end_effector.poll(bpy.context)

        assert LINKFORGE_OT_add_end_effector.poll(bpy.context)
        assert LINKFORGE_OT_add_end_effector().execute(bpy.context) == {"FINISHED"}
        assert props.end_effectors[0].name == "end_effector"
        assert LINKFORGE_OT_add_end_effector().execute(bpy.context) == {"FINISHED"}
        assert props.end_effectors[1].name == "end_effector_2"

        assert LINKFORGE_OT_move_end_effector.poll(bpy.context)
        props.planning_active_ee_index = 1
        mv = LINKFORGE_OT_move_end_effector()
        mv.direction = "UP"
        assert mv.execute(bpy.context) == {"FINISHED"}
        assert mv.execute(bpy.context) == {"CANCELLED"}
        mv.direction = "DOWN"
        assert mv.execute(bpy.context) == {"FINISHED"}

        props.planning_active_ee_index = 99
        assert LINKFORGE_OT_remove_end_effector().execute(bpy.context) == {"CANCELLED"}

        props.planning_active_ee_index = 0
        assert LINKFORGE_OT_remove_end_effector().execute(bpy.context) == {"FINISHED"}
        assert LINKFORGE_OT_remove_end_effector().execute(bpy.context) == {"FINISHED"}
        assert len(props.end_effectors) == 0

    def test_group_state_crud_move_sync_and_reset(self, scene) -> None:
        base, _, link2, _, _ = _setup_3_link_arm(scene)
        props = safe_get_linkforge_scene(scene)
        props.planning_groups.clear()
        props.group_states.clear()

        assert not LINKFORGE_OT_remove_group_state.poll(bpy.context)
        assert not LINKFORGE_OT_move_group_state.poll(bpy.context)
        assert not LINKFORGE_OT_sync_group_state_joints.poll(bpy.context)
        assert not LINKFORGE_OT_reset_group_state_values.poll(bpy.context)

        # Add group state when no planning groups exist yet
        assert LINKFORGE_OT_add_group_state().execute(bpy.context) == {"FINISHED"}
        props.group_states.clear()

        # Create a planning group first
        grp = props.planning_groups.add()
        grp.name = "arm"
        grp.group_type = "CHAIN"
        grp.base_link = base
        grp.tip_link = link2
        props.planning_active_group_index = 99  # fallback to 0

        assert LINKFORGE_OT_add_group_state.poll(bpy.context)
        assert LINKFORGE_OT_add_group_state().execute(bpy.context) == {"FINISHED"}
        assert props.group_states[0].name == "pose"
        assert props.group_states[0].group == "arm"
        assert len(props.group_states[0].joint_values) == 2

        props.planning_active_group_index = 0
        assert LINKFORGE_OT_add_group_state().execute(bpy.context) == {"FINISHED"}
        assert props.group_states[1].name == "pose_2"

        # Sync and Reset operators
        props.planning_active_state_index = 0
        assert LINKFORGE_OT_sync_group_state_joints.poll(bpy.context)
        assert LINKFORGE_OT_sync_group_state_joints().execute(bpy.context) == {"FINISHED"}

        props.group_states[0].joint_values[0].value = 1.0
        assert LINKFORGE_OT_reset_group_state_values.poll(bpy.context)
        assert LINKFORGE_OT_reset_group_state_values().execute(bpy.context) == {"FINISHED"}
        assert props.group_states[0].joint_values[0].value == 0.0

        # Move
        assert LINKFORGE_OT_move_group_state.poll(bpy.context)
        props.planning_active_state_index = 1
        mv = LINKFORGE_OT_move_group_state()
        mv.direction = "UP"
        assert mv.execute(bpy.context) == {"FINISHED"}
        assert mv.execute(bpy.context) == {"CANCELLED"}
        mv.direction = "DOWN"
        assert mv.execute(bpy.context) == {"FINISHED"}

        # Out of range cases
        props.planning_active_state_index = 99
        assert LINKFORGE_OT_sync_group_state_joints().execute(bpy.context) == {"CANCELLED"}
        assert LINKFORGE_OT_reset_group_state_values().execute(bpy.context) == {"CANCELLED"}
        assert LINKFORGE_OT_remove_group_state().execute(bpy.context) == {"CANCELLED"}

        # Remove all
        props.planning_groups.clear()
        props.planning_active_state_index = 0
        assert LINKFORGE_OT_remove_group_state().execute(bpy.context) == {"FINISHED"}
        assert LINKFORGE_OT_remove_group_state().execute(bpy.context) == {"FINISHED"}
        assert len(props.group_states) == 0

    def test_disabled_collisions_crud_move_auto_and_purge(self, scene) -> None:
        props = safe_get_linkforge_scene(scene)
        props.disabled_collisions.clear()

        # Auto-disable on empty scene -> 0 pairs
        assert LINKFORGE_OT_auto_disable_adjacent_collisions().execute(bpy.context) == {"FINISHED"}
        assert len(props.disabled_collisions) == 0

        base, _, _, _, _ = _setup_3_link_arm(scene)
        # Add an incomplete joint and a self-loop joint in scene, plus an incomplete pair in disabled_collisions
        j_inc = create_robot_joint("j_inc", None, None, scene)
        safe_get_joint(j_inc, scene).is_robot_joint = True
        j_self = create_robot_joint("j_self", base, base, scene)
        safe_get_joint(j_self, scene).is_robot_joint = True
        inc_pair = props.disabled_collisions.add()
        inc_pair.link1 = None
        inc_pair.link2 = None

        assert LINKFORGE_OT_remove_disabled_collision.poll(bpy.context)
        props.disabled_collisions.clear()
        assert not LINKFORGE_OT_remove_disabled_collision.poll(bpy.context)
        assert not LINKFORGE_OT_move_disabled_collision.poll(bpy.context)
        assert not LINKFORGE_OT_purge_planning_data.poll(bpy.context)

        # Add back incomplete pair before auto-disable
        inc_pair = props.disabled_collisions.add()
        inc_pair.link1 = None
        inc_pair.link2 = None

        # Auto-disable adjacent collisions
        assert LINKFORGE_OT_auto_disable_adjacent_collisions.poll(bpy.context)
        assert LINKFORGE_OT_auto_disable_adjacent_collisions().execute(bpy.context) == {"FINISHED"}
        assert len(props.disabled_collisions) == 3

        # Second run skips existing pairs (idempotent)
        assert LINKFORGE_OT_auto_disable_adjacent_collisions().execute(bpy.context) == {"FINISHED"}
        assert len(props.disabled_collisions) == 3

        # Manual add
        assert LINKFORGE_OT_add_disabled_collision.poll(bpy.context)
        assert LINKFORGE_OT_add_disabled_collision().execute(bpy.context) == {"FINISHED"}
        assert len(props.disabled_collisions) == 4

        # Move
        assert LINKFORGE_OT_move_disabled_collision.poll(bpy.context)
        props.planning_active_collision_index = 1
        mv = LINKFORGE_OT_move_disabled_collision()
        mv.direction = "UP"
        assert mv.execute(bpy.context) == {"FINISHED"}
        assert mv.execute(bpy.context) == {"CANCELLED"}
        mv.direction = "DOWN"
        assert mv.execute(bpy.context) == {"FINISHED"}

        # Remove
        props.planning_active_collision_index = 99
        assert LINKFORGE_OT_remove_disabled_collision().execute(bpy.context) == {"CANCELLED"}
        props.planning_active_collision_index = 2
        assert LINKFORGE_OT_remove_disabled_collision().execute(bpy.context) == {"FINISHED"}
        assert len(props.disabled_collisions) == 3

        # Purge all planning data
        assert LINKFORGE_OT_purge_planning_data.poll(bpy.context)
        assert LINKFORGE_OT_purge_planning_data().execute(bpy.context) == {"FINISHED"}
        assert len(props.disabled_collisions) == 0
        assert not has_planning_data(props)

        # Remove last collision pair sets use_srdf = False
        LINKFORGE_OT_add_disabled_collision().execute(bpy.context)
        props.use_srdf = True
        props.planning_active_collision_index = 0
        assert LINKFORGE_OT_remove_disabled_collision().execute(bpy.context) == {"FINISHED"}
        assert props.use_srdf is False

    def test_planning_ops_null_scene_and_registration(self) -> None:
        no_scene_ctx = types.SimpleNamespace(scene=None)
        for cls in planning_ops.classes:
            assert not cls.poll(no_scene_ctx)
            assert cls().execute(no_scene_ctx) == {"CANCELLED"}

        planning_ops.register()
        planning_ops.unregister()
        planning_ops.register()
        calls = 0

        def _raise_once(_cls):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise ValueError("already registered")

        with (
            patch.object(bpy.utils, "register_class", side_effect=_raise_once),
            patch.object(bpy.utils, "unregister_class"),
        ):
            planning_ops.register()
        with patch.object(bpy.utils, "register_class"):
            runpy.run_module("linkforge.blender.operators.planning_ops", run_name="__main__")


class TestPlanningPanelAndUILists:
    """Test UILists, Menu, and Panel drawing in planning_panel.py."""

    def test_ui_lists_default_and_grid_modes(self, scene) -> None:
        base, link1, link2, _, _ = _setup_3_link_arm(scene)
        props = safe_get_linkforge_scene(scene)

        grp_chain = props.planning_groups.add()
        grp_chain.name = "arm"
        grp_chain.group_type = "CHAIN"
        grp_chain.base_link = base
        grp_chain.tip_link = link2

        grp_joints = props.planning_groups.add()
        grp_joints.name = "hand"
        grp_joints.group_type = "JOINTS"

        ee = props.end_effectors.add()
        ee.name = "gripper"
        ee.group = "hand"
        ee.parent_link = link2

        st = props.group_states.add()
        st.name = "home"
        st.group = "arm"

        dc = props.disabled_collisions.add()
        dc.link1 = base
        dc.link2 = link1
        dc.reason = "Adjacent"

        for ul_cls, item in [
            (LINKFORGE_UL_planning_groups, grp_chain),
            (LINKFORGE_UL_planning_groups, grp_joints),
            (LINKFORGE_UL_end_effectors, ee),
            (LINKFORGE_UL_group_states, st),
            (LINKFORGE_UL_disabled_collisions, dc),
        ]:
            ul = ul_cls()
            layout = MagicMock()
            ul.layout_type = "DEFAULT"
            ul.draw_item(bpy.context, layout, props, item, None, props, "active", 0)
            ul.layout_type = "GRID"
            ul.draw_item(bpy.context, layout, props, item, None, props, "active", 0)
            ul.layout_type = "OTHER"
            ul.draw_item(bpy.context, layout, props, item, None, props, "active", 0)

    def test_add_planning_group_joint_menu(self, scene) -> None:
        _setup_3_link_arm(scene)
        props = safe_get_linkforge_scene(scene)
        menu = LINKFORGE_MT_add_planning_group_joint()
        menu.layout = MagicMock()

        # No active planning group
        props.planning_groups.clear()
        menu.draw(bpy.context)

        # With active planning group and available joints
        grp = props.planning_groups.add()
        grp.name = "hand"
        grp.group_type = "JOINTS"
        props.planning_active_group_index = 0
        menu.draw(bpy.context)

        # When all joints are already added
        for name in ("j1", "j2"):
            j_item = grp.joints.add()
            j_item.name = name
        menu.draw(bpy.context)

        # Null layout/scene or missing props
        menu.layout = None
        menu.draw(bpy.context)
        menu.layout = MagicMock()
        with patch("linkforge.blender.panels.planning_panel.get_robot_props", return_value=None):
            menu.draw(bpy.context)

    def test_planning_panel_draw_all_sections(self, scene) -> None:
        props = safe_get_linkforge_scene(scene)

        panel = LINKFORGE_PT_planning()
        panel.layout = MagicMock()

        # Draw on completely empty scene (num_links == 0 and no planning data)
        props.use_virtual_joint = False
        panel.draw(bpy.context)
        panel.layout.box.return_value = None
        panel.draw(bpy.context)
        panel.layout = MagicMock()

        base, link1, link2, j1, j2 = _setup_3_link_arm(scene)

        # Draw when robot exists in scene and no planning data exists (empty collections, all sections open)
        props.use_virtual_joint = False
        panel.draw(bpy.context)

        # Draw with populated items (CHAIN group, End Effector, Group State with joints, Disabled Collision)
        props.use_virtual_joint = True
        props.virtual_joint_child_link = base

        grp = props.planning_groups.add()
        grp.name = "arm"
        grp.group_type = "CHAIN"
        # First draw with unset base/tip links
        panel.draw(bpy.context)
        # Draw with unreachable chain (tip -> base)
        grp.base_link = link2
        grp.tip_link = base
        panel.draw(bpy.context)
        # Draw with valid chain (base -> tip)
        grp.base_link = base
        grp.tip_link = link2

        ee = props.end_effectors.add()
        ee.name = "gripper"
        ee.group = "arm"
        ee.parent_link = link2

        st = props.group_states.add()
        st.name = "home"
        st.group = "arm"
        jv = st.joint_values.add()
        jv.name = "j1"
        jv.joint_name = "j1"
        jv.joint_obj = j1
        jv.value = 0.0

        # Add a joint value without joint_obj and a continuous joint without limits
        jv_none = st.joint_values.add()
        jv_none.name = "j_unlinked"
        jv_none.joint_name = "j_unlinked"
        jv_none.joint_obj = None

        safe_get_joint(j2, scene).joint_type = "CONTINUOUS"
        safe_get_joint(j2, scene).use_limits = False
        jv_cont = st.joint_values.add()
        jv_cont.name = "j2"
        jv_cont.joint_name = "j2"
        jv_cont.joint_obj = j2

        dc = props.disabled_collisions.add()
        dc.link1 = base
        dc.link2 = link1

        panel.draw(bpy.context)

        # Draw with JOINTS group (empty and populated) and empty Group State
        grp.group_type = "JOINTS"
        st.joint_values.clear()
        panel.draw(bpy.context)

        j_item = grp.joints.add()
        j_item.name = "j1"
        j_item.joint_obj = j1
        props.use_virtual_joint = False
        panel.draw(bpy.context)

        # Draw with all sections collapsed
        props.show_virtual_joint = False
        props.show_planning_groups = False
        props.show_end_effectors = False
        props.show_group_states = False
        props.show_disabled_collisions = False
        panel.draw(bpy.context)

        # Null layout/props guards
        panel.layout = None
        panel.draw(bpy.context)
        panel.layout = MagicMock()
        with patch("linkforge.blender.panels.planning_panel.get_robot_props", return_value=None):
            panel.draw(bpy.context)

        planning_panel.register()
        planning_panel.unregister()
        planning_panel.register()
        calls = 0

        def _raise_once(_cls):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise ValueError("already registered")

        with (
            patch.object(bpy.utils, "register_class", side_effect=_raise_once),
            patch.object(bpy.utils, "unregister_class"),
        ):
            planning_panel.register()
        with patch.object(bpy.utils, "register_class"):
            runpy.run_module("linkforge.blender.panels.planning_panel", run_name="__main__")


class TestSemanticTranslationExportAndImport:
    """Test SemanticTranslator, export_ops SRDF writing, and import_ops/core_to_blender SRDF round-trip."""

    def test_semantic_translator_complete_and_incomplete_entries(self, scene) -> None:
        base, link1, link2, j1, j2 = _setup_3_link_arm(scene)
        props = safe_get_linkforge_scene(scene)
        props.robot_name = "planned_bot"

        # 1. Virtual joint
        props.use_virtual_joint = True
        props.virtual_joint_name = "world_joint"
        props.virtual_joint_type = "fixed"
        props.virtual_joint_parent_frame = "world"
        props.virtual_joint_child_link = base

        # 2. CHAIN group (with extra chains, links_csv, subgroups_csv) + JOINTS group
        grp_arm = props.planning_groups.add()
        grp_arm.name = "arm"
        grp_arm.group_type = "CHAIN"
        grp_arm.base_link = base
        grp_arm.tip_link = link2
        grp_arm.links_csv = "base_link"
        grp_arm.subgroups_csv = "hand"
        grp_arm.extra_chains_csv = "base_link:link1"

        grp_hand = props.planning_groups.add()
        grp_hand.name = "hand"
        grp_hand.group_type = "JOINTS"
        j_item = grp_hand.joints.add()
        j_item.name = "j2"
        j_item.joint_obj = j2

        # 3. End effector
        ee = props.end_effectors.add()
        ee.name = "gripper"
        ee.group = "hand"
        ee.parent_link = link2
        ee.parent_group = "arm"

        # 4. Group state
        st = props.group_states.add()
        st.name = "home"
        st.group = "arm"
        jv = st.joint_values.add()
        jv.name = "j1"
        jv.joint_name = "j1"
        jv.joint_obj = j1
        jv.value = 0.25

        # 5. Disabled collisions
        dc = props.disabled_collisions.add()
        dc.link1 = base
        dc.link2 = link1
        dc.reason = "Adjacent"

        lf_ctx = BlenderContext(bpy)
        robot, val_res = blender_to_core.scene_to_robot(lf_ctx, dry_run=True, raise_on_error=False)
        assert not val_res.errors
        assert robot.semantic is not None
        assert len(robot.semantic.virtual_joints) == 1
        assert len(robot.semantic.groups) == 2
        assert len(robot.semantic.end_effectors) == 1
        assert len(robot.semantic.group_states) == 1
        assert len(robot.semantic.disabled_collisions) == 1

        # Incomplete entries produce clear validation errors
        props.use_virtual_joint = True
        props.virtual_joint_child_link = None
        props.virtual_joint_child_link_name = ""

        grp_unnamed = props.planning_groups.add()
        grp_unnamed.name = ""

        grp_bad = props.planning_groups.add()
        grp_bad.name = "incomplete_chain"
        grp_bad.group_type = "CHAIN"
        grp_bad.base_link = base
        grp_bad.tip_link = None

        grp_empty = props.planning_groups.add()
        grp_empty.name = "empty_group"
        grp_empty.group_type = "JOINTS"

        ee_unnamed = props.end_effectors.add()
        ee_unnamed.name = ""

        ee_bad = props.end_effectors.add()
        ee_bad.name = "incomplete_ee"
        ee_bad.group = ""

        st_unnamed = props.group_states.add()
        st_unnamed.name = ""

        st_bad = props.group_states.add()
        st_bad.name = "incomplete_state"
        st_bad.group = ""

        dc_empty = props.disabled_collisions.add()
        dc_empty.link1 = None
        dc_empty.link2 = None

        dc_bad = props.disabled_collisions.add()
        dc_bad.link1 = base
        dc_bad.link2 = None

        _, val_res_bad = blender_to_core.scene_to_robot(lf_ctx, dry_run=True, raise_on_error=False)
        titles = {e.title for e in val_res_bad.errors}
        assert "Incomplete planning group chain" in titles
        assert "Empty planning group" in titles
        assert "Incomplete end effector" in titles
        assert "Incomplete group state" in titles
        assert "Incomplete disabled collision pair" in titles

        # Test with use_virtual_joint = False
        props.use_virtual_joint = False
        blender_to_core.scene_to_robot(lf_ctx, dry_run=True, raise_on_error=False)

        # Exception handling in SemanticTranslator
        sem_trans = translator.SemanticTranslator()
        bad_builder = MagicMock()
        type(bad_builder).semantic = property(
            lambda _self: (_ for _ in ()).throw(RuntimeError("boom"))
        )
        err_res = ValidationResult()
        sem_trans.translate(props, bad_builder, err_res)
        assert any(e.title == "SRDF semantic translation failed" for e in err_res.errors)

    def test_export_companion_srdf_and_import_round_trip(self, tmp_path: Path, scene) -> None:
        base, link1, link2, j1, j2 = _setup_3_link_arm(scene)
        props = safe_get_linkforge_scene(scene)
        props.robot_name = "srdf_bot"
        props.export_format = "XACRO"
        props.export_meshes = False
        props.validate_before_export = True
        props.use_ros2_control = False
        props.use_srdf = True
        props.use_virtual_joint = True
        props.virtual_joint_child_link = base

        grp = props.planning_groups.add()
        grp.name = "arm"
        grp.group_type = "CHAIN"
        grp.base_link = base
        grp.tip_link = link2

        dc = props.disabled_collisions.add()
        dc.link1 = base
        dc.link2 = link1
        dc.reason = "Adjacent"

        out_xacro = tmp_path / "srdf_bot.urdf.xacro"
        exp_op = LINKFORGE_OT_export_robot_model()
        exp_op.filepath = str(out_xacro)
        exp_op.report = MagicMock()

        res = exp_op.execute(bpy.context)
        assert res == {"FINISHED"}
        srdf_path = tmp_path / "srdf_bot.srdf"
        assert srdf_path.is_file()
        srdf_xml = srdf_path.read_text()
        assert '<group name="arm">' in srdf_xml
        assert '<virtual_joint name="world_joint"' in srdf_xml

        # Also test URDF export writing companion .srdf
        out_urdf = tmp_path / "srdf_bot.urdf"
        props.export_format = "URDF"
        exp_op.filepath = str(out_urdf)
        assert exp_op.execute(bpy.context) == {"FINISHED"}

        # Test importing the URDF + companion SRDF back via LINKFORGE_OT_import_robot_model
        imp_op = LINKFORGE_OT_import_robot_model()
        imp_op.filepath = str(out_urdf)
        imp_op.report = MagicMock()
        with patch.object(AsynchronousRobotBuilder, "start"):
            assert imp_op.execute(bpy.context) == {"FINISHED"}

        # Test companion SRDF parse error logs warning without aborting URDF import
        srdf_path.write_text("<invalid_xml")
        with patch.object(AsynchronousRobotBuilder, "start"):
            assert imp_op.execute(bpy.context) == {"FINISHED"}

        # Null guard on bind_semantic_scene_objects
        core_to_blender.bind_semantic_scene_objects(None, {}, {})

        # Test setup_scene_for_robot and bind_semantic_scene_objects with both CHAIN and JOINTS groups
        robot = Robot(name="roundtrip_bot")
        robot.semantic = SemanticRobotDescription(
            virtual_joints=[
                VirtualJoint(
                    name="vj",
                    type="planar",
                    parent_frame="odom",
                    child_link="base_link",
                )
            ],
            groups=[
                PlanningGroup(
                    name="arm",
                    links=("base_link",),
                    subgroups=("hand",),
                    chains=(
                        Chain(base_link="base_link", tip_link="link2"),
                        Chain(base_link="base_link", tip_link="link1"),
                    ),
                ),
                PlanningGroup(name="hand", joints=("j2",)),
                PlanningGroup(name="links_only_grp", links=("base_link",)),
            ],
            end_effectors=[
                EndEffector(
                    name="gripper",
                    group="hand",
                    parent_link="link2",
                    parent_group="arm",
                )
            ],
            group_states=[
                GroupState(name="home", group="arm", joint_values={"j1": (0.5,)}),
            ],
            disabled_collisions=[
                CollisionPair(link1="base_link", link2="link1", reason="CustomReason"),
            ],
        )

        lf_ctx = BlenderContext(bpy)
        core_to_blender.setup_scene_for_robot(lf_ctx, robot)
        core_to_blender.bind_semantic_scene_objects(
            props,
            {"base_link": base, "link1": link1, "link2": link2},
            {"j1": j1, "j2": j2},
        )

        assert props.use_srdf is True
        assert props.virtual_joint_child_link == base
        assert props.planning_groups[0].base_link == base
        assert props.planning_groups[0].tip_link == link2
        assert props.planning_groups[0].extra_chains_csv == "base_link:link1"
        assert props.planning_groups[1].joints[0].joint_obj == j2
        assert props.end_effectors[0].parent_link == link2
        assert props.group_states[0].joint_values[0].joint_obj == j1
        assert props.disabled_collisions[0].link1 == base
        assert props.disabled_collisions[0].reason == "User"
