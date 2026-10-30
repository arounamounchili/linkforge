"""Blender Operators for centralized SRDF (MoveIt 2 planning) configuration.

Provides operators for managing Virtual Joints, Planning Groups, End Effectors,
Group States, and Disabled Collisions in the LinkForge Planning panel.
"""

from __future__ import annotations

import bpy
from bpy.props import StringProperty
from bpy.types import Context, Operator

from ..core.constants import SRDF_REASON_ADJACENT
from ..properties.planning_props import (
    get_planning_joint_name,
    get_planning_link_name,
    has_planning_data,
    sync_group_state_joints,
)
from ..utils.decorators import OperatorReturn, safe_execute
from ..utils.property_helpers import get_joint_props, get_link_props, get_robot_props


def _unique_item_name(collection: object, base_name: str) -> str:
    """Generate a unique name within a Blender CollectionProperty."""
    existing = {
        str(getattr(item, "name", "")) for item in getattr(collection, "__iter__", lambda: ())()
    }
    if base_name not in existing:
        return base_name
    counter = 2
    while f"{base_name}_{counter}" in existing:
        counter += 1
    return f"{base_name}_{counter}"


class LINKFORGE_OT_detect_virtual_joint_root(Operator):
    """Automatically detect the robot's root link and assign it to the Virtual Joint."""

    bl_idname = "linkforge.detect_virtual_joint_root"
    bl_label = "Detect Root Link"
    bl_description = "Automatically detect the kinematic tree root link and assign it as Child Link"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Detect and assign the root link."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        child_links = set()
        link_objs = []
        for obj in scene.objects:
            if (lp := get_link_props(obj)) and lp.is_robot_link:
                link_objs.append(obj)
            if (jp := get_joint_props(obj)) and jp.is_robot_joint and jp.child_link:
                child_links.add(jp.child_link)

        root_candidates = [obj for obj in link_objs if obj not in child_links]
        if not root_candidates:
            self.report({"WARNING"}, "No root robot link found in the scene")
            return {"CANCELLED"}

        root_candidates.sort(key=lambda o: get_planning_link_name(o))
        root_obj = root_candidates[0]
        root_name = get_planning_link_name(root_obj)

        props.use_virtual_joint = True
        props.virtual_joint_child_link = root_obj
        props.virtual_joint_child_link_name = root_name
        props.use_srdf = True

        self.report({"INFO"}, f"Assigned root link '{root_name}' to virtual joint")
        return {"FINISHED"}


class LINKFORGE_OT_add_planning_group(Operator):
    """Add a new MoveIt 2 Planning Group."""

    bl_idname = "linkforge.add_planning_group"
    bl_label = "Add Planning Group"
    bl_description = "Add a new kinematic planning group (chain or joint collection)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Add a new planning group and auto-enable SRDF export."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        group_name = _unique_item_name(props.planning_groups, "group")

        item = props.planning_groups.add()
        item.name = group_name
        item.group_type = "CHAIN"

        props.planning_active_group_index = len(props.planning_groups) - 1
        props.use_srdf = True

        self.report({"INFO"}, f"Added planning group '{group_name}'")
        return {"FINISHED"}


class LINKFORGE_OT_remove_planning_group(Operator):
    """Remove the selected MoveIt 2 Planning Group."""

    bl_idname = "linkforge.remove_planning_group"
    bl_label = "Remove Planning Group"
    bl_description = "Remove the selected planning group"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.planning_groups) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Remove the active planning group."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_group_index
        if 0 <= index < len(props.planning_groups):
            group_name = props.planning_groups[index].name
            props.planning_groups.remove(index)
            props.planning_active_group_index = max(0, index - 1)
            if not has_planning_data(props):
                props.use_srdf = False
            self.report({"INFO"}, f"Removed planning group '{group_name}'")
            return {"FINISHED"}

        return {"CANCELLED"}


class LINKFORGE_OT_move_planning_group(Operator):
    """Move the selected Planning Group up or down in the list."""

    bl_idname = "linkforge.move_planning_group"
    bl_label = "Move Planning Group"
    bl_description = "Move planning group up or down in the list"
    bl_options = {"REGISTER", "UNDO"}

    direction: StringProperty()  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.planning_groups) > 1

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Move the active planning group."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_group_index
        if self.direction == "UP" and index > 0:
            new_index = index - 1
        elif self.direction == "DOWN" and index < len(props.planning_groups) - 1:
            new_index = index + 1
        else:
            return {"CANCELLED"}

        props.planning_groups.move(index, new_index)
        props.planning_active_group_index = new_index
        return {"FINISHED"}


class LINKFORGE_OT_add_planning_group_joint(Operator):
    """Add a joint to the active JOINTS-mode Planning Group."""

    bl_idname = "linkforge.add_planning_group_joint"
    bl_label = "Add Joint to Group"
    bl_description = "Add a joint from the robot's kinematic tree to the active planning group"
    bl_options = {"REGISTER", "UNDO"}

    joint_name: StringProperty(name="Joint Name")  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        idx = props.planning_active_group_index
        return bool(0 <= idx < len(props.planning_groups))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Add the joint to the active planning group."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        idx = props.planning_active_group_index
        if not (0 <= idx < len(props.planning_groups)):
            return {"CANCELLED"}

        group = props.planning_groups[idx]
        target_joint_obj = next(
            (
                obj
                for obj in scene.objects
                if obj.type == "EMPTY"
                and (jp := get_joint_props(obj))
                and jp.is_robot_joint
                and jp.joint_name == self.joint_name
            ),
            None,
        )

        for existing in group.joints:
            if existing.name == self.joint_name or (
                existing.joint_obj is not None
                and target_joint_obj is not None
                and existing.joint_obj == target_joint_obj
            ):
                self.report(
                    {"WARNING"},
                    f"Joint '{self.joint_name}' is already in group '{group.name}'",
                )
                return {"CANCELLED"}

        item = group.joints.add()
        item.name = self.joint_name
        item.joint_obj = target_joint_obj
        group.active_joint_index = len(group.joints) - 1

        # Refresh any group states referencing this group
        for state in props.group_states:
            if state.group == group.name:
                sync_group_state_joints(state, scene)

        self.report({"INFO"}, f"Added joint '{self.joint_name}' to group '{group.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_remove_planning_group_joint(Operator):
    """Remove the selected joint from the active JOINTS-mode Planning Group."""

    bl_idname = "linkforge.remove_planning_group_joint"
    bl_label = "Remove Joint from Group"
    bl_description = "Remove the selected joint from the active planning group"
    bl_options = {"REGISTER", "UNDO"}

    index: bpy.props.IntProperty(default=-1)  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        idx = props.planning_active_group_index
        if not (0 <= idx < len(props.planning_groups)):
            return False
        return len(props.planning_groups[idx].joints) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Remove a joint from the active planning group."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        idx = props.planning_active_group_index
        if not (0 <= idx < len(props.planning_groups)):
            return {"CANCELLED"}

        group = props.planning_groups[idx]
        j_idx = self.index if self.index >= 0 else group.active_joint_index
        if 0 <= j_idx < len(group.joints):
            j_name = get_planning_joint_name(
                group.joints[j_idx].joint_obj, group.joints[j_idx].name
            )
            group.joints.remove(j_idx)
            group.active_joint_index = max(0, j_idx - 1)

            for state in props.group_states:
                if state.group == group.name:
                    sync_group_state_joints(state, scene)

            self.report({"INFO"}, f"Removed joint '{j_name}' from group '{group.name}'")
            return {"FINISHED"}

        return {"CANCELLED"}


class LINKFORGE_OT_add_end_effector(Operator):
    """Add a new MoveIt 2 End Effector."""

    bl_idname = "linkforge.add_end_effector"
    bl_label = "Add End Effector"
    bl_description = "Add a new end effector definition"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Add an end effector."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        ee_name = _unique_item_name(props.end_effectors, "end_effector")
        item = props.end_effectors.add()
        item.name = ee_name

        props.planning_active_ee_index = len(props.end_effectors) - 1
        self.report({"INFO"}, f"Added end effector '{ee_name}'")
        return {"FINISHED"}


class LINKFORGE_OT_remove_end_effector(Operator):
    """Remove the selected MoveIt 2 End Effector."""

    bl_idname = "linkforge.remove_end_effector"
    bl_label = "Remove End Effector"
    bl_description = "Remove the selected end effector"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.end_effectors) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Remove the active end effector."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_ee_index
        if 0 <= index < len(props.end_effectors):
            ee_name = props.end_effectors[index].name
            props.end_effectors.remove(index)
            props.planning_active_ee_index = max(0, index - 1)
            if not has_planning_data(props):
                props.use_srdf = False
            self.report({"INFO"}, f"Removed end effector '{ee_name}'")
            return {"FINISHED"}

        return {"CANCELLED"}


class LINKFORGE_OT_move_end_effector(Operator):
    """Move the selected End Effector up or down in the list."""

    bl_idname = "linkforge.move_end_effector"
    bl_label = "Move End Effector"
    bl_description = "Move end effector up or down in the list"
    bl_options = {"REGISTER", "UNDO"}

    direction: StringProperty()  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.end_effectors) > 1

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Move the active end effector."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_ee_index
        if self.direction == "UP" and index > 0:
            new_index = index - 1
        elif self.direction == "DOWN" and index < len(props.end_effectors) - 1:
            new_index = index + 1
        else:
            return {"CANCELLED"}

        props.end_effectors.move(index, new_index)
        props.planning_active_ee_index = new_index
        return {"FINISHED"}


class LINKFORGE_OT_add_group_state(Operator):
    """Add a new named MoveIt 2 Group State (pose)."""

    bl_idname = "linkforge.add_group_state"
    bl_label = "Add Group State"
    bl_description = "Add a named joint configuration (e.g., home, ready) for a planning group"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Add a group state and sync its joints if a planning group is available."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        state_name = _unique_item_name(props.group_states, "pose")
        item = props.group_states.add()
        item.name = state_name

        if len(props.planning_groups) > 0:
            grp_idx = props.planning_active_group_index
            if not (0 <= grp_idx < len(props.planning_groups)):
                grp_idx = 0
            item.group = props.planning_groups[grp_idx].name
            sync_group_state_joints(item, scene)

        props.planning_active_state_index = len(props.group_states) - 1
        self.report({"INFO"}, f"Added group state '{state_name}'")
        return {"FINISHED"}


class LINKFORGE_OT_remove_group_state(Operator):
    """Remove the selected MoveIt 2 Group State."""

    bl_idname = "linkforge.remove_group_state"
    bl_label = "Remove Group State"
    bl_description = "Remove the selected group state"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.group_states) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Remove the active group state."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_state_index
        if 0 <= index < len(props.group_states):
            state_name = props.group_states[index].name
            props.group_states.remove(index)
            props.planning_active_state_index = max(0, index - 1)
            if not has_planning_data(props):
                props.use_srdf = False
            self.report({"INFO"}, f"Removed group state '{state_name}'")
            return {"FINISHED"}

        return {"CANCELLED"}


class LINKFORGE_OT_move_group_state(Operator):
    """Move the selected Group State up or down in the list."""

    bl_idname = "linkforge.move_group_state"
    bl_label = "Move Group State"
    bl_description = "Move group state up or down in the list"
    bl_options = {"REGISTER", "UNDO"}

    direction: StringProperty()  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.group_states) > 1

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Move the active group state."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_state_index
        if self.direction == "UP" and index > 0:
            new_index = index - 1
        elif self.direction == "DOWN" and index < len(props.group_states) - 1:
            new_index = index + 1
        else:
            return {"CANCELLED"}

        props.group_states.move(index, new_index)
        props.planning_active_state_index = new_index
        return {"FINISHED"}


class LINKFORGE_OT_sync_group_state_joints(Operator):
    """Synchronize the active Group State's joint sliders from its Planning Group."""

    bl_idname = "linkforge.sync_group_state_joints"
    bl_label = "Sync Joints from Group"
    bl_description = "Refresh the active pose's joint list from its selected planning group"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        idx = props.planning_active_state_index
        return 0 <= idx < len(props.group_states) and bool(props.group_states[idx].group)

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Sync active joints from the referenced planning group."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        idx = props.planning_active_state_index
        if not (0 <= idx < len(props.group_states)):
            return {"CANCELLED"}

        state = props.group_states[idx]
        count = sync_group_state_joints(state, scene)
        self.report({"INFO"}, f"Synced {count} active joint(s) for state '{state.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_reset_group_state_values(Operator):
    """Reset all joint positions in the active Group State to 0.0 (clamped to limits)."""

    bl_idname = "linkforge.reset_group_state_values"
    bl_label = "Reset Joint Values"
    bl_description = "Reset all joint positions in this pose to 0.0 (or nearest valid limit)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        idx = props.planning_active_state_index
        return 0 <= idx < len(props.group_states) and len(props.group_states[idx].joint_values) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Reset all joint values to 0.0."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        idx = props.planning_active_state_index
        if not (0 <= idx < len(props.group_states)):
            return {"CANCELLED"}

        state = props.group_states[idx]
        for jv in state.joint_values:
            jv.value = 0.0

        self.report({"INFO"}, f"Reset joint values for state '{state.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_add_disabled_collision(Operator):
    """Add a new Disabled Collision link pair."""

    bl_idname = "linkforge.add_disabled_collision"
    bl_label = "Add Disabled Collision"
    bl_description = "Add a link pair that MoveIt should ignore during collision checking"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Add a disabled collision pair."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        item = props.disabled_collisions.add()
        item.name = f"pair_{len(props.disabled_collisions)}"
        item.reason = SRDF_REASON_ADJACENT

        props.planning_active_collision_index = len(props.disabled_collisions) - 1
        self.report({"INFO"}, "Added disabled collision pair")
        return {"FINISHED"}


class LINKFORGE_OT_remove_disabled_collision(Operator):
    """Remove the selected Disabled Collision link pair."""

    bl_idname = "linkforge.remove_disabled_collision"
    bl_label = "Remove Disabled Collision"
    bl_description = "Remove the selected disabled collision pair"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.disabled_collisions) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Remove the active disabled collision pair."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_collision_index
        if 0 <= index < len(props.disabled_collisions):
            props.disabled_collisions.remove(index)
            props.planning_active_collision_index = max(0, index - 1)
            if not has_planning_data(props):
                props.use_srdf = False
            self.report({"INFO"}, "Removed disabled collision pair")
            return {"FINISHED"}

        return {"CANCELLED"}


class LINKFORGE_OT_move_disabled_collision(Operator):
    """Move the selected Disabled Collision pair up or down in the list."""

    bl_idname = "linkforge.move_disabled_collision"
    bl_label = "Move Disabled Collision"
    bl_description = "Move disabled collision pair up or down in the list"
    bl_options = {"REGISTER", "UNDO"}

    direction: StringProperty()  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.disabled_collisions) > 1

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Move the active disabled collision pair."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_collision_index
        if self.direction == "UP" and index > 0:
            new_index = index - 1
        elif self.direction == "DOWN" and index < len(props.disabled_collisions) - 1:
            new_index = index + 1
        else:
            return {"CANCELLED"}

        props.disabled_collisions.move(index, new_index)
        props.planning_active_collision_index = new_index
        return {"FINISHED"}


class LINKFORGE_OT_auto_disable_adjacent_collisions(Operator):
    """Walk the kinematic tree and disable collisions for all adjacent parent-child link pairs."""

    bl_idname = "linkforge.auto_disable_adjacent_collisions"
    bl_label = "Auto-Disable Adjacent Links"
    bl_description = (
        "Walk the kinematic tree and generate a disabled collision pair for every "
        "parent-child joint connection with reason 'Adjacent'"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Generate adjacent disabled collision pairs from scene joints."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        existing_pairs: set[tuple[str, str]] = set()
        for item in props.disabled_collisions:
            l1 = get_planning_link_name(item.link1, item.link1_name)
            l2 = get_planning_link_name(item.link2, item.link2_name)
            if l1 and l2:
                existing_pairs.add((l1, l2))
                existing_pairs.add((l2, l1))

        added_count = 0
        for obj in scene.objects:
            jp = get_joint_props(obj)
            if not jp or not jp.is_robot_joint:
                continue
            if not jp.parent_link or not jp.child_link:
                continue

            p_name = get_planning_link_name(jp.parent_link)
            c_name = get_planning_link_name(jp.child_link)
            if not p_name or not c_name or p_name == c_name:
                continue

            if (p_name, c_name) in existing_pairs:
                continue

            pair = props.disabled_collisions.add()
            pair.name = f"{p_name} <-> {c_name}"
            pair.link1 = jp.parent_link
            pair.link1_name = p_name
            pair.link2 = jp.child_link
            pair.link2_name = c_name
            pair.reason = SRDF_REASON_ADJACENT

            existing_pairs.add((p_name, c_name))
            existing_pairs.add((c_name, p_name))
            added_count += 1

        if len(props.disabled_collisions) > 0:
            props.planning_active_collision_index = len(props.disabled_collisions) - 1

        self.report({"INFO"}, f"Added {added_count} adjacent collision pair(s)")
        return {"FINISHED"}


class LINKFORGE_OT_purge_planning_data(Operator):
    """Clear all SRDF planning configuration from the scene."""

    bl_idname = "linkforge.purge_planning_data"
    bl_label = "Purge Planning Data"
    bl_description = "Clear all planning groups, end effectors, poses, and collision rules"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return has_planning_data(props)

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Clear all SRDF planning properties."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        props.planning_groups.clear()
        props.end_effectors.clear()
        props.group_states.clear()
        props.disabled_collisions.clear()

        props.planning_active_group_index = 0
        props.planning_active_ee_index = 0
        props.planning_active_state_index = 0
        props.planning_active_collision_index = 0

        props.virtual_joint_child_link = None
        props.virtual_joint_child_link_name = ""
        props.use_srdf = False

        self.report({"INFO"}, "Purged all SRDF planning data")
        return {"FINISHED"}


classes = [
    LINKFORGE_OT_detect_virtual_joint_root,
    LINKFORGE_OT_add_planning_group,
    LINKFORGE_OT_remove_planning_group,
    LINKFORGE_OT_move_planning_group,
    LINKFORGE_OT_add_planning_group_joint,
    LINKFORGE_OT_remove_planning_group_joint,
    LINKFORGE_OT_add_end_effector,
    LINKFORGE_OT_remove_end_effector,
    LINKFORGE_OT_move_end_effector,
    LINKFORGE_OT_add_group_state,
    LINKFORGE_OT_remove_group_state,
    LINKFORGE_OT_move_group_state,
    LINKFORGE_OT_sync_group_state_joints,
    LINKFORGE_OT_reset_group_state_values,
    LINKFORGE_OT_add_disabled_collision,
    LINKFORGE_OT_remove_disabled_collision,
    LINKFORGE_OT_move_disabled_collision,
    LINKFORGE_OT_auto_disable_adjacent_collisions,
    LINKFORGE_OT_purge_planning_data,
]


def register() -> None:
    """Register SRDF planning operators."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister SRDF planning operators."""
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
