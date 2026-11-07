"""Blender Operators for centralized SRDF (MoveIt 2 planning) configuration.

Provides operators for managing Virtual Joints, Planning Groups, End Effectors,
Group States, and Disabled Collisions in the LinkForge Planning panel.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import bpy
from bpy.props import IntProperty, StringProperty
from bpy.types import Context, Operator
from bpy_extras.io_utils import ImportHelper

from .. import core
from ..adapters.core_to_blender import (
    bind_semantic_scene_objects,
    populate_planning_properties_from_semantic,
)
from ..constants import DIRECTION_DOWN, DIRECTION_UP, EXT_SRDF
from ..core.constants import (
    PLANNING_GROUP_CHAIN,
    PLANNING_GROUP_JOINTS,
    PLANNING_GROUP_SUBGROUPS,
    SRDF_REASON_ADJACENT,
)
from ..logic.collision_matrix import compute_allowed_collision_matrix
from ..properties.planning_props import (
    get_existing_collision_pairs,
    get_group_subgroup_names,
    get_planning_joint_name,
    get_planning_link_name,
    has_planning_data,
    sync_group_state_joints,
)
from ..utils.decorators import OperatorReturn, safe_execute
from ..utils.property_helpers import get_joint_props, get_link_props, get_robot_props
from ..utils.scene_utils import get_robot_statistics


def _unique_item_name(collection: object, base_name: str) -> str:
    """Generate a unique name within a Blender CollectionProperty.

    Args:
        collection: The Blender collection property to check for existing names.
        base_name: Desired base name prefix (e.g., 'group', 'end_effector', 'pose').

    Returns:
        A unique item name that does not collide with existing items in the collection.
    """
    existing = {
        str(getattr(item, "name", "")) for item in getattr(collection, "__iter__", lambda: ())()
    }
    if base_name not in existing:
        return base_name
    counter = 2
    while f"{base_name}_{counter}" in existing:
        counter += 1
    return f"{base_name}_{counter}"


class LINKFORGE_OT_add_planning_group(Operator):
    """Add a new MoveIt 2 Planning Group."""

    bl_idname = "linkforge.add_planning_group"
    bl_label = "Add Planning Group"
    bl_description = "Add a new kinematic planning group (chain or joint collection)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if the scene has LinkForge properties initialized, False otherwise.
        """
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Add a new planning group and auto-enable SRDF export.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        group_name = _unique_item_name(props.planning_groups, "group")

        item = props.planning_groups.add()
        item.name = group_name
        item.group_type = PLANNING_GROUP_CHAIN

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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if at least one planning group exists, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.planning_groups) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Remove the active planning group.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if multiple planning groups exist, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.planning_groups) > 1

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Move the active planning group.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_group_index
        if self.direction == DIRECTION_UP and index > 0:
            new_index = index - 1
        elif self.direction == DIRECTION_DOWN and index < len(props.planning_groups) - 1:
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if a valid planning group is selected, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        idx = props.planning_active_group_index
        return bool(0 <= idx < len(props.planning_groups))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Add the joint to the active planning group.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if the active planning group has joints to remove, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        idx = props.planning_active_group_index
        if not (0 <= idx < len(props.planning_groups)):
            return False
        return len(props.planning_groups[idx].joints) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Remove a joint from the active planning group.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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


class LINKFORGE_OT_add_planning_subgroup(Operator):
    """Add another Planning Group as a subgroup to the active composite Planning Group."""

    bl_idname = "linkforge.add_planning_subgroup"
    bl_label = "Add Subgroup to Group"
    bl_description = "Add another planning group as a subgroup"
    bl_options = {"REGISTER", "UNDO"}

    subgroup_name: StringProperty(name="Subgroup Name")  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if multiple planning groups exist, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.planning_groups) > 1

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Add the requested subgroup name to the active group.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        idx = props.planning_active_group_index
        if not (0 <= idx < len(props.planning_groups)):
            return {"CANCELLED"}

        group = props.planning_groups[idx]
        if not self.subgroup_name:
            return {"CANCELLED"}

        if self.subgroup_name == group.name:
            self.report({"WARNING"}, "Cannot add a planning group as a subgroup of itself")
            return {"CANCELLED"}

        current_subs = get_group_subgroup_names(group)
        if self.subgroup_name in current_subs:
            self.report(
                {"WARNING"},
                f"'{self.subgroup_name}' is already a subgroup of '{group.name}'",
            )
            return {"CANCELLED"}

        current_subs.append(self.subgroup_name)
        group.subgroups_csv = ",".join(current_subs)
        if not any(getattr(sg, "name", "") == self.subgroup_name for sg in group.subgroups):
            sg_item = group.subgroups.add()
            sg_item.name = self.subgroup_name

        for state in props.group_states:
            if state.group == group.name:
                sync_group_state_joints(state, scene)

        self.report({"INFO"}, f"Added subgroup '{self.subgroup_name}' to group '{group.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_remove_planning_subgroup(Operator):
    """Remove a subgroup from the active composite Planning Group."""

    bl_idname = "linkforge.remove_planning_subgroup"
    bl_label = "Remove Subgroup from Group"
    bl_description = "Remove the selected subgroup from the active planning group"
    bl_options = {"REGISTER", "UNDO"}

    index: IntProperty(default=-1)  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if the active group has at least one subgroup, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        idx = props.planning_active_group_index
        if not (0 <= idx < len(props.planning_groups)):
            return False
        subs = get_group_subgroup_names(props.planning_groups[idx])
        return len(subs) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Remove the subgroup at the specified index.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        idx = props.planning_active_group_index
        if not (0 <= idx < len(props.planning_groups)):
            return {"CANCELLED"}

        group = props.planning_groups[idx]
        current_subs = get_group_subgroup_names(group)

        if not (0 <= self.index < len(current_subs)):
            return {"CANCELLED"}

        removed = current_subs.pop(self.index)
        group.subgroups_csv = ",".join(current_subs)
        for i, sg in enumerate(group.subgroups):
            if getattr(sg, "name", "") == removed:
                group.subgroups.remove(i)
                break

        for state in props.group_states:
            if state.group == group.name:
                sync_group_state_joints(state, scene)

        self.report({"INFO"}, f"Removed subgroup '{removed}' from group '{group.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_add_end_effector(Operator):
    """Add a new MoveIt 2 End Effector."""

    bl_idname = "linkforge.add_end_effector"
    bl_label = "Add End Effector"
    bl_description = "Add a new end effector definition"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if the scene has LinkForge properties initialized, False otherwise.
        """
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Add an end effector.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if at least one end effector exists, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.end_effectors) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Remove the active end effector.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if multiple end effectors exist, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.end_effectors) > 1

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Move the active end effector.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_ee_index
        if self.direction == DIRECTION_UP and index > 0:
            new_index = index - 1
        elif self.direction == DIRECTION_DOWN and index < len(props.end_effectors) - 1:
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if the scene has LinkForge properties initialized, False otherwise.
        """
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Add a group state and sync its joints if a planning group is available.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if at least one group state exists, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.group_states) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Remove the active group state.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if multiple group states exist, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.group_states) > 1

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Move the active group state.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_state_index
        if self.direction == DIRECTION_UP and index > 0:
            new_index = index - 1
        elif self.direction == DIRECTION_DOWN and index < len(props.group_states) - 1:
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if a group state with a valid group name is selected, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        idx = props.planning_active_state_index
        return bool(0 <= idx < len(props.group_states)) and bool(props.group_states[idx].group)

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Sync active joints from the referenced planning group.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if the active group state contains joint values, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        idx = props.planning_active_state_index
        return (
            bool(0 <= idx < len(props.group_states))
            and len(props.group_states[idx].joint_values) > 0
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Reset all joint values to 0.0.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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


class LINKFORGE_OT_toggle_group_state_preview(Operator):
    """Toggle live 3D viewport preview for the active Group State pose."""

    bl_idname = "linkforge.toggle_group_state_preview"
    bl_label = "Toggle Pose Preview"
    bl_description = "Toggle live non-destructive preview of this pose in the 3D viewport"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        idx = props.planning_active_state_index
        return bool(0 <= idx < len(props.group_states))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Toggle preview on active pose."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}
        idx = props.planning_active_state_index
        if not (0 <= idx < len(props.group_states)):
            return {"CANCELLED"}
        state = props.group_states[idx]
        state.is_previewing = not state.is_previewing

        from ..utils.pose_preview import apply_group_state_pose, restore_robot_rest_pose

        if state.is_previewing:
            for other in props.group_states:
                if other != state and getattr(other, "is_previewing", False):
                    other.is_previewing = False
            props.preview_active_state_name = str(state.name or "")
            apply_group_state_pose(scene, state)
        else:
            if props.preview_active_state_name == str(state.name or ""):
                props.preview_active_state_name = ""
            restore_robot_rest_pose(scene)
        return {"FINISHED"}


class LINKFORGE_OT_capture_group_state_pose(Operator):
    """Capture current joint angles and positions from the 3D viewport into this Group State."""

    bl_idname = "linkforge.capture_group_state_pose"
    bl_label = "Capture Scene Pose"
    bl_description = (
        "Read current joint rotations and displacements from the 3D viewport into this pose"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        idx = props.planning_active_state_index
        return (
            bool(0 <= idx < len(props.group_states))
            and len(props.group_states[idx].joint_values) > 0
        )

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Capture scene pose into active group state."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}
        idx = props.planning_active_state_index
        if not (0 <= idx < len(props.group_states)):
            return {"CANCELLED"}
        state = props.group_states[idx]
        from ..utils.pose_preview import capture_scene_pose_to_state

        count = capture_scene_pose_to_state(scene, state)
        self.report({"INFO"}, f"Captured {count} joint position(s) into state '{state.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_restore_rest_pose(Operator):
    """Restore all joints in the robot to their rest pose."""

    bl_idname = "linkforge.restore_rest_pose"
    bl_label = "Restore Rest Pose"
    bl_description = "Reset robot joints back to their pristine zero/rest configuration"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Restore all robot joints to rest pose."""
        scene = context.scene
        if not scene:
            return {"CANCELLED"}
        from ..utils.pose_preview import restore_robot_rest_pose

        restore_robot_rest_pose(scene)
        self.report({"INFO"}, "Restored robot to rest pose")
        return {"FINISHED"}


class LINKFORGE_OT_add_disabled_collision(Operator):
    """Add a new Disabled Collision link pair."""

    bl_idname = "linkforge.add_disabled_collision"
    bl_label = "Add Disabled Collision"
    bl_description = "Add a link pair that MoveIt should ignore during collision checking"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if the scene has LinkForge properties initialized, False otherwise.
        """
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Add a disabled collision pair.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if at least one disabled collision pair exists, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.disabled_collisions) > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Remove the active disabled collision pair.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if multiple disabled collision pairs exist, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return len(props.disabled_collisions) > 1

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Move the active disabled collision pair.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        index = props.planning_active_collision_index
        if self.direction == DIRECTION_UP and index > 0:
            new_index = index - 1
        elif self.direction == DIRECTION_DOWN and index < len(props.disabled_collisions) - 1:
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
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if the scene has LinkForge properties initialized, False otherwise.
        """
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Generate adjacent disabled collision pairs from scene joints.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        existing_pairs = get_existing_collision_pairs(props.disabled_collisions)

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


class LINKFORGE_OT_compute_collision_matrix(Operator):
    """Compute MoveIt-compatible Allowed Collision Matrix (ACM) via kinematic sampling."""

    bl_idname = "linkforge.compute_collision_matrix"
    bl_label = "Generate Collision Matrix (ACM)"
    bl_description = (
        "Generate MoveIt Allowed Collision Matrix using Monte Carlo kinematic sampling "
        "(detects Adjacent, Fixed Chains, Default, Never, and Always colliding pairs)"
    )
    bl_options = {"REGISTER", "UNDO"}

    sample_count: bpy.props.IntProperty(  # type: ignore
        name="Sample Count",
        description="Number of random joint configurations to sample",
        default=500,
        min=50,
        max=5000,
    )

    clear_existing: bpy.props.BoolProperty(  # type: ignore
        name="Clear Existing",
        description="Clear existing auto-generated disabled collision pairs before populating",
        default=True,
    )

    adjacent_only: bpy.props.BoolProperty(  # type: ignore
        name="Adjacent Only",
        description="Only detect adjacent and fixed link pairs (skip kinematic sampling)",
        default=False,
    )

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run."""
        return bool(context.scene and get_robot_props(context.scene))

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute collision matrix sampling and populate disabled collision pairs."""
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        if (
            hasattr(self, "properties")
            and self.properties is not None
            and hasattr(self.properties, "is_property_set")
            and self.properties.is_property_set("sample_count")
        ):
            sample_cnt = int(self.sample_count)
        elif getattr(props, "acm_sample_count", 0):
            sample_cnt = int(props.acm_sample_count)
        else:
            sample_cnt = int(getattr(self, "sample_count", 500))

        results = compute_allowed_collision_matrix(
            scene,
            sample_count=0 if self.adjacent_only else sample_cnt,
            include_adjacent=True,
            include_never=not self.adjacent_only,
            include_default=not self.adjacent_only,
            include_always=not self.adjacent_only,
        )

        if not results:
            self.report({"WARNING"}, "No robot link pairs found to compute collision matrix.")
            return {"CANCELLED"}

        if self.clear_existing:
            # Preserve user-defined custom pairs if any
            user_pairs = [
                (
                    getattr(p, "link1", None),
                    getattr(p, "link1_name", ""),
                    getattr(p, "link2", None),
                    getattr(p, "link2_name", ""),
                    getattr(p, "reason", "User"),
                )
                for p in props.disabled_collisions
                if getattr(p, "reason", "") == "User"
            ]
            props.disabled_collisions.clear()
            for l1_obj, l1_name, l2_obj, l2_name, reason in user_pairs:
                up = props.disabled_collisions.add()
                up.name = f"{l1_name} <-> {l2_name}"
                up.link1 = l1_obj
                up.link1_name = l1_name
                up.link2 = l2_obj
                up.link2_name = l2_name
                up.reason = reason

        # Map existing collision pairs to their collection items
        pair_item_map: dict[frozenset[str], Any] = {}
        for p in props.disabled_collisions:
            p_l1 = getattr(p, "link1_name", "")
            p_l2 = getattr(p, "link2_name", "")
            if p_l1 and p_l2:
                pair_item_map[frozenset([p_l1, p_l2])] = p

        link_objects = get_robot_statistics(scene).link_objects

        counts: dict[str, int] = {}
        added_count = 0
        updated_count = 0
        for l1_name, l2_name, reason in results:
            key = frozenset([l1_name, l2_name])
            if key in pair_item_map:
                existing_item = pair_item_map[key]
                if getattr(existing_item, "reason", "") == "User":
                    counts["User"] = counts.get("User", 0) + 1
                    continue
                if getattr(existing_item, "reason", "") != reason:
                    existing_item.reason = reason
                    updated_count += 1
                counts[reason] = counts.get(reason, 0) + 1
                continue

            pair = props.disabled_collisions.add()
            pair.name = f"{l1_name} <-> {l2_name}"
            pair.link1 = link_objects.get(l1_name)
            pair.link1_name = l1_name
            pair.link2 = link_objects.get(l2_name)
            pair.link2_name = l2_name
            pair.reason = reason

            pair_item_map[key] = pair
            counts[reason] = counts.get(reason, 0) + 1
            added_count += 1

        if len(props.disabled_collisions) > 0:
            props.planning_active_collision_index = len(props.disabled_collisions) - 1
            props.use_srdf = True

        detail_str = ", ".join(f"{r}: {c}" for r, c in sorted(counts.items()))
        report_msg = f"Collision matrix: {added_count} added"
        if updated_count > 0:
            report_msg += f", {updated_count} updated"
        report_msg += f" ({detail_str})"
        self.report({"INFO"}, report_msg)
        return {"FINISHED"}


class LINKFORGE_OT_purge_planning_data(Operator):
    """Clear all SRDF planning configuration from the scene."""

    bl_idname = "linkforge.purge_planning_data"
    bl_label = "Purge Planning Data"
    bl_description = "Clear all planning groups, end effectors, poses, and collision rules"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operator can run.

        Args:
            context: The current Blender context.

        Returns:
            True if any planning data exists to purge, False otherwise.
        """
        if not context.scene or not (props := get_robot_props(context.scene)):
            return False
        return has_planning_data(props)

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Clear all SRDF planning properties.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
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


class LINKFORGE_OT_import_srdf(Operator, ImportHelper):  # type: ignore[misc]
    """Import a MoveIt 2 SRDF file and configure Planning properties for the scene robot."""

    bl_idname = "linkforge.import_srdf"
    bl_label = "Import SRDF"
    bl_description = (
        "Import a MoveIt 2 SRDF file to configure planning groups, poses, and collisions"
    )
    bl_options = {"REGISTER", "UNDO"}

    filename_ext = EXT_SRDF
    filter_glob: StringProperty(  # type: ignore
        default=f"*{EXT_SRDF}",
        options={"HIDDEN"},
        maxlen=255,
    )
    filepath: StringProperty(subtype="FILE_PATH")  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Only allow importing SRDF when a robot exists in the scene.

        Args:
            context: The current Blender context.

        Returns:
            True if robot links exist in the scene, False otherwise.
        """
        scene = context.scene
        if not scene:
            return False
        stats = get_robot_statistics(scene)
        return stats.num_links > 0

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Parse and populate SRDF planning properties from the selected file.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene or not (props := get_robot_props(scene)):
            return {"CANCELLED"}

        srdf_path = Path(self.filepath)
        if not srdf_path.is_file():
            self.report({"ERROR"}, f"File not found: {srdf_path}")
            return {"CANCELLED"}

        try:
            semantic = core.SRDFParser().parse(srdf_path, validate=False)
        except Exception as e:
            self.report({"ERROR"}, f"Failed to parse SRDF '{srdf_path.name}': {e}")
            return {"CANCELLED"}

        link_objects: dict[str, bpy.types.Object] = {}
        joint_objects: dict[str, bpy.types.Object] = {}
        for obj in scene.objects:
            if (lp := get_link_props(obj)) and lp.is_robot_link:
                if lp.link_name:
                    link_objects[lp.link_name] = obj
                link_objects[obj.name] = obj
            if (jp := get_joint_props(obj)) and jp.is_robot_joint:
                if jp.joint_name:
                    joint_objects[jp.joint_name] = obj
                joint_objects[obj.name] = obj

        populate_planning_properties_from_semantic(props, semantic)
        bind_semantic_scene_objects(props, link_objects, joint_objects)

        # Mark passive joints in scene
        imported_passive_count = 0
        for pj in getattr(semantic, "passive_joints", ()):
            pj_name = pj.name
            if pj_name in joint_objects:
                j_obj = joint_objects[pj_name]
                jp = get_joint_props(j_obj)
                if jp:
                    jp.is_passive = True
                    imported_passive_count += 1

        # Count unresolved references
        unresolved_count = 0
        for grp in props.planning_groups:
            if grp.group_type == PLANNING_GROUP_CHAIN:
                if grp.base_link_name and not grp.base_link:
                    unresolved_count += 1
                if grp.tip_link_name and not grp.tip_link:
                    unresolved_count += 1
            elif grp.group_type == PLANNING_GROUP_JOINTS:
                for j in grp.joints:
                    if j.name and not j.joint_obj:
                        unresolved_count += 1
            elif grp.group_type == PLANNING_GROUP_SUBGROUPS:
                existing_group_names = {g.name for g in props.planning_groups}
                for s in str(grp.subgroups_csv or "").split(","):
                    sub = s.strip()
                    if sub and sub not in existing_group_names:
                        unresolved_count += 1

        for ee in props.end_effectors:
            if ee.parent_link_name and not ee.parent_link:
                unresolved_count += 1

        for dc in props.disabled_collisions:
            if dc.link1_name and not dc.link1:
                unresolved_count += 1
            if dc.link2_name and not dc.link2:
                unresolved_count += 1

        msg = (
            f"Imported SRDF '{srdf_path.name}': "
            f"{len(props.planning_groups)} group(s), "
            f"{len(props.end_effectors)} end effector(s), "
            f"{len(props.disabled_collisions)} collision rule(s)"
        )
        if unresolved_count > 0:
            self.report(
                {"WARNING"},
                f"{msg} ({unresolved_count} unresolved reference(s) - check Validate panel)",
            )
        else:
            self.report({"INFO"}, msg)

        return {"FINISHED"}


classes = [
    LINKFORGE_OT_add_planning_group,
    LINKFORGE_OT_remove_planning_group,
    LINKFORGE_OT_move_planning_group,
    LINKFORGE_OT_add_planning_group_joint,
    LINKFORGE_OT_remove_planning_group_joint,
    LINKFORGE_OT_add_planning_subgroup,
    LINKFORGE_OT_remove_planning_subgroup,
    LINKFORGE_OT_add_end_effector,
    LINKFORGE_OT_remove_end_effector,
    LINKFORGE_OT_move_end_effector,
    LINKFORGE_OT_add_group_state,
    LINKFORGE_OT_remove_group_state,
    LINKFORGE_OT_move_group_state,
    LINKFORGE_OT_sync_group_state_joints,
    LINKFORGE_OT_reset_group_state_values,
    LINKFORGE_OT_toggle_group_state_preview,
    LINKFORGE_OT_capture_group_state_pose,
    LINKFORGE_OT_restore_rest_pose,
    LINKFORGE_OT_add_disabled_collision,
    LINKFORGE_OT_remove_disabled_collision,
    LINKFORGE_OT_move_disabled_collision,
    LINKFORGE_OT_auto_disable_adjacent_collisions,
    LINKFORGE_OT_compute_collision_matrix,
    LINKFORGE_OT_purge_planning_data,
    LINKFORGE_OT_import_srdf,
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
