"""UI Panel for managing centralized MoveIt 2 SRDF planning configuration."""

from __future__ import annotations

import contextlib
import typing

import bpy

from ..constants import DIRECTION_DOWN, DIRECTION_UP
from ..core.constants import (
    JOINT_PRISMATIC,
    JOINT_REVOLUTE,
    PLANNING_GROUP_CHAIN,
    PLANNING_GROUP_JOINTS,
    PLANNING_GROUP_SUBGROUPS,
)
from ..properties.planning_props import (
    get_group_subgroup_names,
    get_planning_joint_name,
    get_planning_link_name,
    has_planning_data,
    inspect_chain_path,
)
from ..utils.property_helpers import get_joint_props, get_robot_props
from ..utils.scene_utils import get_robot_statistics


class LINKFORGE_UL_planning_groups(bpy.types.UIList):
    """UI List representation for MoveIt 2 SRDF Planning Groups."""

    def draw_item(
        self,
        _context: bpy.types.Context,
        layout: bpy.types.UILayout,
        _data: typing.Any,
        item: typing.Any,
        _icon: int | None,
        _active_data: typing.Any,
        _active_propname: str | None,
        _index: int | None = 0,
        _flt_flag: int | None = 0,
    ) -> None:
        """Draw a planning group row in the UIList.

        Args:
            _context: The current Blender context (unused, required by API).
            layout: The parent UILayout row.
            _data: Property group collection owner (unused, required by API).
            item: The PlanningGroupProperty item being rendered.
            _icon: Icon ID (unused, required by API).
            _active_data: Pointer to active collection data (unused, required by API).
            _active_propname: Name of active property (unused, required by API).
            _index: Index of the item in the collection.
            _flt_flag: UIList filter flags.
        """
        if self.layout_type in {"DEFAULT", "COMPACT"}:
            row = layout.row(align=True)
            row.label(text=item.name or "Unnamed", icon="ARMATURE_DATA")

            if item.group_type == PLANNING_GROUP_CHAIN:
                base_str = get_planning_link_name(item.base_link, item.base_link_name)
                tip_str = get_planning_link_name(item.tip_link, item.tip_link_name)
                if base_str and tip_str:
                    row.label(text=f"[Chain: {base_str} → {tip_str}]", icon="NONE")
                else:
                    row.label(text="[Chain: Unconfigured]", icon="NONE")
            elif item.group_type == PLANNING_GROUP_SUBGROUPS:
                subs = get_group_subgroup_names(item)
                sub_count = len(subs)
                s_str = "1 subgroup" if sub_count == 1 else f"{sub_count} subgroups"
                row.label(text=f"[Subgroups: {s_str}]", icon="NONE")
            else:
                n_joints = len(item.joints)
                joint_str = "1 joint" if n_joints == 1 else f"{n_joints} joints"
                row.label(text=f"[Collection: {joint_str}]", icon="NONE")
        elif self.layout_type == "GRID":
            layout.alignment = "CENTER"
            layout.label(text="", icon="ARMATURE_DATA")


class LINKFORGE_UL_end_effectors(bpy.types.UIList):
    """UI List representation for MoveIt 2 SRDF End Effectors."""

    def draw_item(
        self,
        _context: bpy.types.Context,
        layout: bpy.types.UILayout,
        _data: typing.Any,
        item: typing.Any,
        _icon: int | None,
        _active_data: typing.Any,
        _active_propname: str | None,
        _index: int | None = 0,
        _flt_flag: int | None = 0,
    ) -> None:
        """Draw an end effector row in the UIList.

        Args:
            _context: The current Blender context (unused, required by API).
            layout: The parent UILayout row.
            _data: Property group collection owner (unused, required by API).
            item: The EndEffectorProperty item being rendered.
            _icon: Icon ID (unused, required by API).
            _active_data: Pointer to active collection data (unused, required by API).
            _active_propname: Name of active property (unused, required by API).
            _index: Index of the item in the collection.
            _flt_flag: UIList filter flags.
        """
        if self.layout_type in {"DEFAULT", "COMPACT"}:
            row = layout.row(align=True)
            row.label(text=item.name or "Unnamed", icon="MOD_SKIN")
            p_link = get_planning_link_name(item.parent_link, item.parent_link_name)
            grp = item.group
            if grp and p_link:
                row.label(text=f"[{grp} @ {p_link}]", icon="NONE")
            elif grp:
                row.label(text=f"[{grp} @ Unassigned]", icon="NONE")
            elif p_link:
                row.label(text=f"[Unassigned @ {p_link}]", icon="NONE")
            else:
                row.label(text="[Unconfigured]", icon="NONE")
        elif self.layout_type == "GRID":
            layout.alignment = "CENTER"
            layout.label(text="", icon="MOD_SKIN")


class LINKFORGE_UL_group_states(bpy.types.UIList):
    """UI List representation for MoveIt 2 SRDF Group States (poses)."""

    def draw_item(
        self,
        _context: bpy.types.Context,
        layout: bpy.types.UILayout,
        _data: typing.Any,
        item: typing.Any,
        _icon: int | None,
        _active_data: typing.Any,
        _active_propname: str | None,
        _index: int | None = 0,
        _flt_flag: int | None = 0,
    ) -> None:
        """Draw a group state row in the UIList.

        Args:
            _context: The current Blender context (unused, required by API).
            layout: The parent UILayout row.
            _data: Property group collection owner (unused, required by API).
            item: The GroupStateProperty item being rendered.
            _icon: Icon ID (unused, required by API).
            _active_data: Pointer to active collection data (unused, required by API).
            _active_propname: Name of active property (unused, required by API).
            _index: Index of the item in the collection.
            _flt_flag: UIList filter flags.
        """
        if self.layout_type in {"DEFAULT", "COMPACT"}:
            row = layout.row(align=True)
            is_prev = getattr(item, "is_previewing", False)
            icon = "HIDE_OFF" if is_prev else "POSE_HLT"
            row.label(text=item.name or "Unnamed", icon=icon)
            grp = item.group or "Unassigned"
            preview_badge = " [Previewing]" if is_prev else ""
            row.label(text=f"[{grp} · {len(item.joint_values)} DOF]{preview_badge}", icon="NONE")
        elif self.layout_type == "GRID":
            layout.alignment = "CENTER"
            layout.label(text="", icon="POSE_HLT")


class LINKFORGE_UL_disabled_collisions(bpy.types.UIList):
    """UI List representation for MoveIt 2 SRDF Disabled Collision pairs."""

    def draw_item(
        self,
        _context: bpy.types.Context,
        layout: bpy.types.UILayout,
        _data: typing.Any,
        item: typing.Any,
        _icon: int | None,
        _active_data: typing.Any,
        _active_propname: str | None,
        _index: int | None = 0,
        _flt_flag: int | None = 0,
    ) -> None:
        """Draw a disabled collision pair row in the UIList.

        Args:
            _context: The current Blender context (unused, required by API).
            layout: The parent UILayout row.
            _data: Property group collection owner (unused, required by API).
            item: The CollisionPairProperty item being rendered.
            _icon: Icon ID (unused, required by API).
            _active_data: Pointer to active collection data (unused, required by API).
            _active_propname: Name of active property (unused, required by API).
            _index: Index of the item in the collection.
            _flt_flag: UIList filter flags.
        """
        if self.layout_type in {"DEFAULT", "COMPACT"}:
            row = layout.row(align=True)
            l1 = get_planning_link_name(item.link1, item.link1_name) or "?"
            l2 = get_planning_link_name(item.link2, item.link2_name) or "?"
            row.label(text=f"{l1} ↔ {l2}", icon="MOD_PHYSICS")
            row.label(text=f"[{item.reason}]", icon="NONE")
        elif self.layout_type == "GRID":
            layout.alignment = "CENTER"
            layout.label(text="", icon="MOD_PHYSICS")


class LINKFORGE_MT_add_planning_group_joint(bpy.types.Menu):
    """Menu to add joints from the scene to a JOINTS-mode Planning Group."""

    bl_label = "Add Joint to Group"

    def draw(self, context: bpy.types.Context) -> None:
        """Draw available scene joints not yet in the active planning group.

        Args:
            context: The current Blender context.
        """
        layout = self.layout
        scene = context.scene
        if not (layout and scene):
            return

        if not (props := get_robot_props(scene)):
            return

        idx = props.planning_active_group_index
        if not (0 <= idx < len(props.planning_groups)):
            layout.label(text="No active planning group")
            return

        group = props.planning_groups[idx]
        added_names = {item.name for item in group.joints}
        added_objs = {item.joint_obj for item in group.joints if item.joint_obj}

        joint_objs = []
        for obj in scene.objects:
            if (jp := get_joint_props(obj)) and jp.is_robot_joint:
                name = jp.joint_name
                if obj not in added_objs and name not in added_names:
                    joint_objs.append((name, obj))

        if not joint_objs:
            layout.label(text="No more joints available")
            return

        for name, _ in sorted(joint_objs):
            op = layout.operator("linkforge.add_planning_group_joint", text=name)
            op.joint_name = name


class LINKFORGE_MT_add_planning_subgroup(bpy.types.Menu):
    """Menu to add another Planning Group as a subgroup to the active composite group."""

    bl_label = "Add Subgroup to Group"

    def draw(self, context: bpy.types.Context) -> None:
        """Draw available planning groups not yet in the active group's subgroups.

        Args:
            context: The current Blender context.
        """
        layout = self.layout
        scene = context.scene
        if not (layout and scene):
            return

        if not (props := get_robot_props(scene)):
            return

        idx = props.planning_active_group_index
        if not (0 <= idx < len(props.planning_groups)):
            layout.label(text="No active planning group")
            return

        group = props.planning_groups[idx]
        current_subs = set(get_group_subgroup_names(group))

        available = [
            g.name
            for g in props.planning_groups
            if g.name and g.name != group.name and g.name not in current_subs
        ]

        if not available:
            layout.label(text="No other groups available")
            return

        for name in sorted(available):
            op = layout.operator("linkforge.add_planning_subgroup", text=name)
            op.subgroup_name = name


class LINKFORGE_PT_planning(bpy.types.Panel):
    """Panel for configuring MoveIt 2 SRDF planning semantics."""

    bl_label = "Planning"
    bl_description = (
        "Configure MoveIt 2 SRDF planning groups, end effectors, poses, and collision matrix"
    )
    bl_idname = "LINKFORGE_PT_planning"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 3
    bl_options = {"DEFAULT_CLOSED"}

    def draw_planning_group_details(
        self,
        layout: bpy.types.UILayout,
        group_item: typing.Any,
        scene: bpy.types.Scene | None = None,
    ) -> None:
        """Draw the detailed configuration box for the selected Planning Group.

        Args:
            layout: The parent UILayout to draw into.
            group_item: The PlanningGroupProperty item currently selected.
            scene: The active Blender scene, used for kinematic chain inspection.
        """
        inner = layout.box()
        inner.label(text=f"Group: {group_item.name}", icon="SETTINGS")

        col = inner.column(align=True)
        col.prop(group_item, "name")

        mode_row = inner.row(align=True)
        mode_row.prop(group_item, "group_type", expand=True)

        if group_item.group_type == PLANNING_GROUP_CHAIN:
            chain_col = inner.column(align=True)
            chain_col.prop(group_item, "base_link")
            chain_col.prop(group_item, "tip_link")
            base_name = get_planning_link_name(group_item.base_link, group_item.base_link_name)
            tip_name = get_planning_link_name(group_item.tip_link, group_item.tip_link_name)
            if base_name and tip_name:
                if base_name == tip_name:
                    inner.label(
                        text="Base Link and Tip Link must be different",
                        icon="ERROR",
                    )
                else:
                    active, non_active, found = inspect_chain_path(scene, group_item)
                    if active:
                        if len(active) == 1:
                            summary_text = f"Active Joints (1): {active[0][0]}"
                        elif len(active) == 2:
                            summary_text = f"Active Joints (2): {active[0][0]}, {active[1][0]}"
                        else:
                            summary_text = f"Active Joints ({len(active)} DOF): {active[0][0]} → {active[-1][0]}"
                        inner.label(text=summary_text, icon="CHECKMARK")
                    elif found and non_active:
                        inner.label(
                            text=f"Path has {len(non_active)} fixed/mimic joint(s) ({', '.join(non_active)})",
                            icon="INFO",
                        )
                        inner.label(
                            text="MoveIt requires actuated joints (revolute/prismatic/continuous)",
                            icon="FORWARD",
                        )
                    else:
                        inner.label(
                            text="No kinematic path connects Base Link to Tip Link",
                            icon="ERROR",
                        )
        elif group_item.group_type == PLANNING_GROUP_JOINTS:
            header = inner.row()
            header.label(text=f"Joints ({len(group_item.joints)})", icon="EMPTY_AXIS")
            header.menu("LINKFORGE_MT_add_planning_group_joint", icon="ADD", text="")

            if len(group_item.joints) == 0:
                inner.label(text="Add joints to this group using '+'", icon="INFO")
            else:
                j_col = inner.column(align=True)
                for i, j_item in enumerate(group_item.joints):
                    j_row = j_col.row(align=True)
                    j_name = get_planning_joint_name(j_item.joint_obj, j_item.name) or "Unnamed"
                    j_row.label(text=j_name, icon="DOT")
                    rem_op = j_row.operator(
                        "linkforge.remove_planning_group_joint", icon="REMOVE", text=""
                    )
                    rem_op.index = i
        else:  # PLANNING_GROUP_SUBGROUPS
            header = inner.row()
            subs = get_group_subgroup_names(group_item)
            header.label(text=f"Subgroups ({len(subs)})", icon="GROUP")
            header.menu("LINKFORGE_MT_add_planning_subgroup", icon="ADD", text="")

            if not subs:
                inner.label(text="Add subgroups to this group using '+'", icon="INFO")
            else:
                s_col = inner.column(align=True)
                for i, sub_name in enumerate(subs):
                    s_row = s_col.row(align=True)
                    s_row.label(text=sub_name, icon="DOT")
                    rem_op = s_row.operator(
                        "linkforge.remove_planning_subgroup", icon="REMOVE", text=""
                    )
                    rem_op.index = i

    def draw_end_effector_details(
        self,
        layout: bpy.types.UILayout,
        props: typing.Any,
        ee_item: typing.Any,
    ) -> None:
        """Draw the detailed configuration box for the selected End Effector.

        Args:
            layout: The parent UILayout to draw into.
            props: The global LinkForge scene property group.
            ee_item: The EndEffectorProperty item currently selected.
        """
        inner = layout.box()
        inner.label(text=f"End Effector: {ee_item.name}", icon="SETTINGS")

        col = inner.column(align=True)
        col.prop(ee_item, "name")
        col.prop_search(
            ee_item,
            "group",
            props,
            "planning_groups",
            text="Group",
            icon="ARMATURE_DATA",
        )
        col.prop(ee_item, "parent_link")
        col.prop_search(
            ee_item,
            "parent_group",
            props,
            "planning_groups",
            text="Parent Group",
            icon="ARMATURE_DATA",
        )

    def draw_group_state_details(
        self,
        layout: bpy.types.UILayout,
        props: typing.Any,
        state_item: typing.Any,
    ) -> None:
        """Draw the detailed configuration box and joint sliders for the selected Group State.

        Args:
            layout: The parent UILayout to draw into.
            props: The global LinkForge scene property group.
            state_item: The GroupStateProperty item currently selected.
        """
        inner = layout.box()
        head_row = inner.row(align=True)
        head_row.label(text=f"Pose: {state_item.name}", icon="SETTINGS")
        is_prev = getattr(state_item, "is_previewing", False)
        head_row.prop(
            state_item,
            "is_previewing",
            text="Preview Active" if is_prev else "Preview",
            icon="HIDE_OFF" if is_prev else "HIDE_ON",
            toggle=True,
        )

        col = inner.column(align=True)
        col.prop(state_item, "name")
        col.prop_search(
            state_item,
            "group",
            props,
            "planning_groups",
            text="Group",
            icon="ARMATURE_DATA",
        )

        if is_prev:
            banner = inner.box()
            b_row = banner.row(align=True)
            b_row.label(text="Previewing live in 3D Viewport", icon="CHECKMARK")
            b_row.operator("linkforge.restore_rest_pose", text="Exit Preview", icon="X")

        inner.separator()
        if len(state_item.joint_values) == 0:
            inner.label(text="Select a group or click Sync to load active joints.", icon="INFO")
        else:
            j_box = inner.box()
            j_head = j_box.row(align=True)
            j_head.label(text=f"Active Joints ({len(state_item.joint_values)})", icon="EMPTY_AXIS")
            j_head.operator("linkforge.sync_group_state_joints", text="Sync", icon="FILE_REFRESH")
            j_head.operator("linkforge.reset_group_state_values", text="Reset", icon="LOOP_BACK")
            j_head.operator("linkforge.capture_group_state_pose", text="Capture", icon="CAMERA")
            if is_prev:
                j_head.operator("linkforge.restore_rest_pose", text="Rest", icon="ARMATURE_DATA")

            sliders_col = j_box.column(align=True)
            for jv in state_item.joint_values:
                j_name = get_planning_joint_name(jv.joint_obj, jv.joint_name or jv.name)
                label_text = j_name
                if (
                    jv.joint_obj
                    and (jp := get_joint_props(jv.joint_obj))
                    and (
                        jp.joint_type.lower() in (JOINT_REVOLUTE, JOINT_PRISMATIC) or jp.use_limits
                    )
                ):
                    label_text = f"{j_name} [{jp.limit_lower:.2f}, {jp.limit_upper:.2f}]"
                sliders_col.prop(jv, "value", text=label_text, slider=True)

    def draw_collision_pair_details(
        self,
        layout: bpy.types.UILayout,
        pair_item: typing.Any,
    ) -> None:
        """Draw the detailed configuration box for the selected Disabled Collision pair.

        Args:
            layout: The parent UILayout to draw into.
            pair_item: The CollisionPairProperty item currently selected.
        """
        inner = layout.box()
        l1 = get_planning_link_name(pair_item.link1, pair_item.link1_name) or "?"
        l2 = get_planning_link_name(pair_item.link2, pair_item.link2_name) or "?"
        inner.label(text=f"Pair: {l1} ↔ {l2}", icon="SETTINGS")

        col = inner.column(align=True)
        col.prop(pair_item, "link1")
        col.prop(pair_item, "link2")
        col.prop(pair_item, "reason")

    def draw(self, context: bpy.types.Context) -> None:
        """Draw the Planning panel.

        Args:
            context: The current Blender context.
        """
        layout = self.layout
        scene = context.scene
        if not (layout and scene):
            return

        if not (props := get_robot_props(scene)):
            return

        stats = get_robot_statistics(scene)
        if stats.num_links == 0 and not has_planning_data(props):
            box = layout.box()
            if box:
                box.label(text="No robot in scene", icon="INFO")
                box.label(text="Create links in Forge panel to start", icon="FORWARD")
            return

        # Top utility bar for importing/resetting SRDF
        if has_planning_data(props):
            top_split = layout.split(factor=0.88, align=True)
            top_split.operator("linkforge.import_srdf", text="Import SRDF...", icon="IMPORT")
            top_split.operator("linkforge.purge_planning_data", text="", icon="TRASH")
        else:
            layout.operator("linkforge.import_srdf", text="Import SRDF...", icon="IMPORT")

        # === 1. VIRTUAL JOINT ===
        vj_box = layout.box()
        vj_header = vj_box.row(align=True)
        vj_left = vj_header.row(align=True)
        vj_left.alignment = "LEFT"
        vj_left.prop(
            props,
            "show_virtual_joint",
            text="Virtual Joint",
            icon="TRIA_DOWN" if props.show_virtual_joint else "TRIA_RIGHT",
            emboss=False,
        )
        vj_right = vj_header.row(align=True)
        vj_right.alignment = "RIGHT"
        vj_right.prop(props, "use_virtual_joint", text="")

        if props.show_virtual_joint:
            if props.use_virtual_joint:
                vj_col = vj_box.column(align=True)
                vj_col.prop(props, "virtual_joint_name")
                vj_col.prop(props, "virtual_joint_type")
                vj_col.prop(props, "virtual_joint_parent_frame")
                vj_col.prop(props, "virtual_joint_child_link")
            else:
                vj_box.label(text="Enable checkbox to attach robot root to world.", icon="INFO")

        # === 2. PLANNING GROUPS ===
        layout.separator()
        pg_box = layout.box()
        pg_header = pg_box.row(align=True)
        pg_header.alignment = "LEFT"
        pg_header.prop(
            props,
            "show_planning_groups",
            text=f"Planning Groups ({len(props.planning_groups)})",
            icon="TRIA_DOWN" if props.show_planning_groups else "TRIA_RIGHT",
            emboss=False,
        )

        if props.show_planning_groups:
            row = pg_box.row()
            row.template_list(
                "LINKFORGE_UL_planning_groups",
                "",
                props,
                "planning_groups",
                props,
                "planning_active_group_index",
                rows=3,
            )
            col = row.column(align=True)
            col.operator("linkforge.add_planning_group", icon="ADD", text="")
            col.operator("linkforge.remove_planning_group", icon="REMOVE", text="")
            col.separator()
            col.operator(
                "linkforge.move_planning_group", icon="TRIA_UP", text=""
            ).direction = DIRECTION_UP
            col.operator(
                "linkforge.move_planning_group", icon="TRIA_DOWN", text=""
            ).direction = DIRECTION_DOWN

            active_grp_idx = props.planning_active_group_index
            if 0 <= active_grp_idx < len(props.planning_groups):
                self.draw_planning_group_details(
                    pg_box, props.planning_groups[active_grp_idx], scene=scene
                )

        # === 3. END EFFECTORS ===
        layout.separator()
        ee_box = layout.box()
        ee_header = ee_box.row(align=True)
        ee_header.alignment = "LEFT"
        ee_header.prop(
            props,
            "show_end_effectors",
            text=f"End Effectors ({len(props.end_effectors)})",
            icon="TRIA_DOWN" if props.show_end_effectors else "TRIA_RIGHT",
            emboss=False,
        )

        if props.show_end_effectors:
            row = ee_box.row()
            row.template_list(
                "LINKFORGE_UL_end_effectors",
                "",
                props,
                "end_effectors",
                props,
                "planning_active_ee_index",
                rows=3,
            )
            col = row.column(align=True)
            col.operator("linkforge.add_end_effector", icon="ADD", text="")
            col.operator("linkforge.remove_end_effector", icon="REMOVE", text="")
            col.separator()
            col.operator(
                "linkforge.move_end_effector", icon="TRIA_UP", text=""
            ).direction = DIRECTION_UP
            col.operator(
                "linkforge.move_end_effector", icon="TRIA_DOWN", text=""
            ).direction = DIRECTION_DOWN

            active_ee_idx = props.planning_active_ee_index
            if 0 <= active_ee_idx < len(props.end_effectors):
                self.draw_end_effector_details(ee_box, props, props.end_effectors[active_ee_idx])

        # === 4. GROUP STATES ===
        layout.separator()
        gs_box = layout.box()
        gs_header = gs_box.row(align=True)
        gs_header.alignment = "LEFT"
        gs_header.prop(
            props,
            "show_group_states",
            text=f"Group States ({len(props.group_states)})",
            icon="TRIA_DOWN" if props.show_group_states else "TRIA_RIGHT",
            emboss=False,
        )

        if props.show_group_states:
            row = gs_box.row()
            row.template_list(
                "LINKFORGE_UL_group_states",
                "",
                props,
                "group_states",
                props,
                "planning_active_state_index",
                rows=3,
            )
            col = row.column(align=True)
            col.operator("linkforge.add_group_state", icon="ADD", text="")
            col.operator("linkforge.remove_group_state", icon="REMOVE", text="")
            col.separator()
            col.operator(
                "linkforge.move_group_state", icon="TRIA_UP", text=""
            ).direction = DIRECTION_UP
            col.operator(
                "linkforge.move_group_state", icon="TRIA_DOWN", text=""
            ).direction = DIRECTION_DOWN

            active_gs_idx = props.planning_active_state_index
            if 0 <= active_gs_idx < len(props.group_states):
                self.draw_group_state_details(gs_box, props, props.group_states[active_gs_idx])

        # === 5. DISABLED COLLISIONS ===
        layout.separator()
        dc_box = layout.box()
        dc_header = dc_box.row(align=True)
        dc_header.alignment = "LEFT"
        dc_header.prop(
            props,
            "show_disabled_collisions",
            text=f"Disabled Collisions ({len(props.disabled_collisions)})",
            icon="TRIA_DOWN" if props.show_disabled_collisions else "TRIA_RIGHT",
            emboss=False,
        )

        if props.show_disabled_collisions:
            auto_row = dc_box.row(align=True)
            auto_row.operator(
                "linkforge.compute_collision_matrix",
                text="Generate Matrix (ACM)",
                icon="MOD_PHYSICS",
            )
            auto_row.operator(
                "linkforge.auto_disable_adjacent_collisions",
                text="Adjacent Only",
                icon="FILE_REFRESH",
            )

            row = dc_box.row()
            row.template_list(
                "LINKFORGE_UL_disabled_collisions",
                "",
                props,
                "disabled_collisions",
                props,
                "planning_active_collision_index",
                rows=3,
            )
            col = row.column(align=True)
            col.operator("linkforge.add_disabled_collision", icon="ADD", text="")
            col.operator("linkforge.remove_disabled_collision", icon="REMOVE", text="")
            col.separator()
            col.operator(
                "linkforge.move_disabled_collision", icon="TRIA_UP", text=""
            ).direction = DIRECTION_UP
            col.operator(
                "linkforge.move_disabled_collision", icon="TRIA_DOWN", text=""
            ).direction = DIRECTION_DOWN

            active_dc_idx = props.planning_active_collision_index
            if 0 <= active_dc_idx < len(props.disabled_collisions):
                self.draw_collision_pair_details(dc_box, props.disabled_collisions[active_dc_idx])


classes = [
    LINKFORGE_UL_planning_groups,
    LINKFORGE_UL_end_effectors,
    LINKFORGE_UL_group_states,
    LINKFORGE_UL_disabled_collisions,
    LINKFORGE_MT_add_planning_group_joint,
    LINKFORGE_MT_add_planning_subgroup,
    LINKFORGE_PT_planning,
]


def register() -> None:
    """Register Planning panel and UILists."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister Planning panel and UILists."""
    for cls in reversed(classes):
        with contextlib.suppress(RuntimeError):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
