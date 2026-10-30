"""UI Panel for managing centralized MoveIt 2 SRDF planning configuration."""

from __future__ import annotations

import contextlib
import typing

import bpy

from ..core.constants import JOINT_PRISMATIC, JOINT_REVOLUTE
from ..properties.planning_props import (
    get_planning_joint_name,
    get_planning_link_name,
    has_planning_data,
    resolve_planning_group_joints,
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
        """Draw a planning group row in the UIList."""
        if self.layout_type in {"DEFAULT", "COMPACT"}:
            row = layout.row(align=True)
            row.label(text=item.name or "Unnamed", icon="ARMATURE_DATA")

            if item.group_type == "CHAIN":
                base_str = get_planning_link_name(item.base_link, item.base_link_name) or "?"
                tip_str = get_planning_link_name(item.tip_link, item.tip_link_name) or "?"
                row.label(text=f"[{base_str} → {tip_str}]", icon="NONE")
            else:
                row.label(text=f"[{len(item.joints)} joint(s)]", icon="NONE")
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
        """Draw an end effector row in the UIList."""
        if self.layout_type in {"DEFAULT", "COMPACT"}:
            row = layout.row(align=True)
            row.label(text=item.name or "Unnamed", icon="MOD_SKIN")
            p_link = get_planning_link_name(item.parent_link, item.parent_link_name) or "?"
            grp = item.group or "?"
            row.label(text=f"[{grp} @ {p_link}]", icon="NONE")
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
        """Draw a group state row in the UIList."""
        if self.layout_type in {"DEFAULT", "COMPACT"}:
            row = layout.row(align=True)
            row.label(text=item.name or "Unnamed", icon="POSE_HLT")
            grp = item.group or "Unassigned"
            row.label(text=f"[{grp} · {len(item.joint_values)} DOF]", icon="NONE")
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
        """Draw a disabled collision pair row in the UIList."""
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
        """Draw available scene joints not yet in the active planning group."""
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
        """Draw the detailed configuration box for the selected Planning Group."""
        inner = layout.box()
        inner.label(text=f"Group: {group_item.name}", icon="SETTINGS")

        col = inner.column(align=True)
        col.prop(group_item, "name")

        mode_row = inner.row(align=True)
        mode_row.prop(group_item, "group_type", expand=True)

        if group_item.group_type == "CHAIN":
            chain_col = inner.column(align=True)
            chain_col.prop(group_item, "base_link")
            chain_col.prop(group_item, "tip_link")
            base_name = get_planning_link_name(group_item.base_link, group_item.base_link_name)
            tip_name = get_planning_link_name(group_item.tip_link, group_item.tip_link_name)
            if base_name and tip_name:
                resolved = resolve_planning_group_joints(scene, group_item)
                j_box = inner.box()
                if resolved:
                    j_box.label(
                        text=f"Active Joints ({len(resolved)})",
                        icon="EMPTY_AXIS",
                    )
                    j_col = j_box.column(align=True)
                    for j_name, _ in resolved:
                        j_col.label(text=j_name, icon="DOT")
                else:
                    j_box.label(
                        text="No active joints found on path (Base → Tip)",
                        icon="INFO",
                    )
        else:
            j_box = inner.box()
            header = j_box.row()
            header.label(text=f"Joints ({len(group_item.joints)})", icon="EMPTY_AXIS")
            header.menu("LINKFORGE_MT_add_planning_group_joint", icon="ADD", text="")

            if len(group_item.joints) == 0:
                j_box.label(text="Add joints to this group using '+'", icon="INFO")
            else:
                for i, j_item in enumerate(group_item.joints):
                    j_row = j_box.row(align=True)
                    j_name = get_planning_joint_name(j_item.joint_obj, j_item.name) or "Unnamed"
                    j_row.label(text=j_name, icon="EMPTY_AXIS")
                    rem_op = j_row.operator(
                        "linkforge.remove_planning_group_joint", icon="REMOVE", text=""
                    )
                    rem_op.index = i

    def draw_end_effector_details(
        self,
        layout: bpy.types.UILayout,
        props: typing.Any,
        ee_item: typing.Any,
    ) -> None:
        """Draw the detailed configuration box for the selected End Effector."""
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
        """Draw the detailed configuration box and joint sliders for the selected Group State."""
        inner = layout.box()
        inner.label(text=f"Pose: {state_item.name}", icon="SETTINGS")

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

        inner.separator()
        if len(state_item.joint_values) == 0:
            inner.label(text="Select a group or click Sync to load active joints.", icon="INFO")
        else:
            j_box = inner.box()
            j_head = j_box.row(align=True)
            j_head.label(text=f"Active Joints ({len(state_item.joint_values)})", icon="EMPTY_AXIS")
            j_head.operator("linkforge.sync_group_state_joints", text="Sync", icon="FILE_REFRESH")
            j_head.operator("linkforge.reset_group_state_values", text="Reset", icon="LOOP_BACK")

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
        """Draw the detailed configuration box for the selected Disabled Collision pair."""
        inner = layout.box()
        l1 = get_planning_link_name(pair_item.link1, pair_item.link1_name) or "?"
        l2 = get_planning_link_name(pair_item.link2, pair_item.link2_name) or "?"
        inner.label(text=f"Pair: {l1} ↔ {l2}", icon="SETTINGS")

        col = inner.column(align=True)
        col.prop(pair_item, "link1")
        col.prop(pair_item, "link2")
        col.prop(pair_item, "reason")

    def draw(self, context: bpy.types.Context) -> None:
        """Draw the Planning panel."""
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

        # Top utility bar when planning data exists
        if has_planning_data(props):
            top_row = layout.row(align=True)
            top_row.label(text="MoveIt 2 SRDF Configured", icon="CHECKMARK")
            top_row.operator("linkforge.purge_planning_data", text="", icon="FILE_REFRESH")

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
                vj_box.operator(
                    "linkforge.detect_virtual_joint_root",
                    text="Detect Root Link",
                    icon="VIEWZOOM",
                )
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
            col.operator("linkforge.move_planning_group", icon="TRIA_UP", text="").direction = "UP"
            col.operator(
                "linkforge.move_planning_group", icon="TRIA_DOWN", text=""
            ).direction = "DOWN"

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
            col.operator("linkforge.move_end_effector", icon="TRIA_UP", text="").direction = "UP"
            col.operator(
                "linkforge.move_end_effector", icon="TRIA_DOWN", text=""
            ).direction = "DOWN"

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
            col.operator("linkforge.move_group_state", icon="TRIA_UP", text="").direction = "UP"
            col.operator("linkforge.move_group_state", icon="TRIA_DOWN", text="").direction = "DOWN"

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
                "linkforge.auto_disable_adjacent_collisions",
                text="Auto-Disable Adjacent Links",
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
            ).direction = "UP"
            col.operator(
                "linkforge.move_disabled_collision", icon="TRIA_DOWN", text=""
            ).direction = "DOWN"

            active_dc_idx = props.planning_active_collision_index
            if 0 <= active_dc_idx < len(props.disabled_collisions):
                self.draw_collision_pair_details(dc_box, props.disabled_collisions[active_dc_idx])


classes = [
    LINKFORGE_UL_planning_groups,
    LINKFORGE_UL_end_effectors,
    LINKFORGE_UL_group_states,
    LINKFORGE_UL_disabled_collisions,
    LINKFORGE_MT_add_planning_group_joint,
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
