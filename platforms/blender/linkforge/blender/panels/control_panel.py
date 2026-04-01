"""UI Panel for managing centralized robot control and transmissions."""

from __future__ import annotations

import contextlib
import typing

import bpy

from ..utils.scene_utils import build_tree_from_stats, get_robot_statistics


class LINKFORGE_UL_ros2_control_joints(bpy.types.UIList):
    """UI List representation for ROS 2 control joints.

    This class defines how individual joints are displayed in the centralized
    control interface, including status indicators for command interfaces.
    """

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
        """Draw an item in the list.

        Args:
            _context: The current Blender context (unused, required by API).
            layout: The current UILayout.
            _data: The property group being displayed (unused, required by API).
            item: The current list item.
            _icon: The icon for the item (unused, required by API).
            _active_data: Required by Blender API (pointing to the collection owner).
            _active_propname: Required by Blender API (name of the active property).
            _index: Current item index (unused, required by API).
            _flt_flag: Filter flag (unused, required by API).
        """
        # Note: _active_data and _active_propname are required by Blender's UIList.draw_item API
        # signature but are not used in this specific implementation.
        if self.layout_type in {"DEFAULT", "COMPACT"}:
            row = layout.row(align=True)

            # Use the joint's custom name if it exists, otherwise fall back to the object name
            display_name = (
                getattr(item.joint_obj, "linkforge_joint").joint_name
                if item.joint_obj
                else item.name
            )

            row.label(text=display_name, icon="EMPTY_AXIS")

            # Indicators for enabled interfaces
            interfaces = []
            if item.cmd_position:
                interfaces.append("P")
            if item.cmd_velocity:
                interfaces.append("V")
            if item.cmd_effort:
                interfaces.append("E")

            if interfaces:
                row.label(text=f"[{'/'.join(interfaces)}]", icon="NONE")

        elif self.layout_type == "GRID":
            layout.alignment = "CENTER"
            layout.label(text="", icon="EMPTY_AXIS")


class LINKFORGE_PT_control(bpy.types.Panel):
    """Panel for configuring motor control and centralized ros2_control."""

    bl_label = "Control"
    bl_description = "Step 3: Configure centralized ros2_control system and interfaces"
    bl_idname = "LINKFORGE_PT_control"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 2
    bl_options = {"DEFAULT_CLOSED"}

    def draw_joint_details(
        self, layout: bpy.types.UILayout, props: typing.Any, joint_item: typing.Any
    ) -> None:
        """Draw the detailed interface settings for a single joint.

        This helper method renders the command and state interface toggles,
        as well as joint-specific ROS 2 parameters, in a localized box.

        Args:
            layout: The parent UILayout to draw into.
            props: The global LinkForge scene property group.
            joint_item: The specific joint property item being configured.
        """
        inner = layout.box()

        # Use the joint's custom name if it exists, otherwise fall back to the object name
        display_name = (
            getattr(joint_item.joint_obj, "linkforge_joint").joint_name
            if joint_item.joint_obj
            else joint_item.name
        )

        inner.label(text=f"Config: {display_name}", icon="SETTINGS")

        # Hiding command interfaces for read-only sensor hardware
        row = inner.row()
        is_sensor = props.ros2_control_type == "sensor"

        if not is_sensor:
            col = row.column(align=True)
            col.label(text="Command Interfaces:")
            col.prop(joint_item, "cmd_position")
            col.prop(joint_item, "cmd_velocity")
            col.prop(joint_item, "cmd_effort")
        else:
            col = row.column(align=True)
            col.label(text="Command Interfaces:", icon="INFO")
            col.label(text="Disabled for Sensor system.")

        # State Interfaces
        col = row.column(align=True)
        col.label(text="State Interfaces:")
        col.prop(joint_item, "state_position")
        col.prop(joint_item, "state_velocity")
        col.prop(joint_item, "state_effort")

        # Joint Parameters
        inner.separator()
        param_box = inner.box()
        header = param_box.row()
        header.label(text="Joint Parameters", icon="PRESET")
        add_op = header.operator("linkforge.add_ros2_control_parameter", icon="ADD", text="")
        add_op.target = "JOINT"
        if len(joint_item.parameters) > 0:
            p_row = param_box.row()
            p_row.prop(
                joint_item,
                "show_parameters",
                text=f"Parameters ({len(joint_item.parameters)})",
                icon="TRIA_DOWN" if joint_item.show_parameters else "TRIA_RIGHT",
                emboss=False,
            )

            if joint_item.show_parameters:
                for i, param in enumerate(joint_item.parameters):
                    p_row = param_box.row(align=True)
                    p_row.prop(param, "name", text="")
                    p_row.prop(param, "value", text="")
                    rem_op = p_row.operator(
                        "linkforge.remove_ros2_control_parameter", icon="REMOVE", text=""
                    )
                    rem_op.target = "JOINT"
                    rem_op.index = i

    def draw(self, context: bpy.types.Context) -> None:
        """Draw the panel."""
        layout = self.layout
        scene = context.scene
        if not (layout and scene):
            return

        props = getattr(scene, "linkforge")

        # Master Toggle
        layout.prop(props, "use_ros2_control", text="Use ROS2 Control", icon="CHECKMARK")

        if not props.use_ros2_control:
            layout.label(text="Enable ROS 2 Control to configure settings.", icon="INFO")
            return

        # === 1. HARDWARE SYSTEM (Global) ===
        box = layout.box()
        box.label(text="Hardware System", icon="ARMATURE_DATA")

        col = box.column(align=True)
        col.prop(props, "ros2_control_name")
        col.prop(props, "ros2_control_type")
        col.prop(props, "hardware_plugin")

        # Global Parameters
        box.separator()
        p_header = box.row()
        p_header.label(text="Hardware Parameters", icon="PRESET")
        add_p = p_header.operator("linkforge.add_ros2_control_parameter", icon="ADD", text="")
        add_p.target = "GLOBAL"

        if len(props.ros2_control_parameters) > 0:
            p_box = box.box()
            p_row = p_box.row()
            p_row.prop(
                props,
                "show_ros2_control_parameters",
                text=f"Parameters ({len(props.ros2_control_parameters)})",
                icon="TRIA_DOWN" if props.show_ros2_control_parameters else "TRIA_RIGHT",
                emboss=False,
            )

            if props.show_ros2_control_parameters:
                for i, param in enumerate(props.ros2_control_parameters):
                    p_row = p_box.row(align=True)
                    p_row.prop(param, "name", text="")
                    p_row.prop(param, "value", text="")
                    rem_p = p_row.operator(
                        "linkforge.remove_ros2_control_parameter", icon="REMOVE", text=""
                    )
                    rem_p.target = "GLOBAL"
                    rem_p.index = i

        # === 2. INTERFACE LAYER (Joint Manager) ===
        layout.separator()
        box = layout.box()
        box.label(text="Joint Interfaces", icon="LINKED")

        # UI List for joints
        row = box.row()
        row.template_list(
            "LINKFORGE_UL_ros2_control_joints",
            "",
            props,
            "ros2_control_joints",
            props,
            "ros2_control_active_joint_index",
        )

        # List controls (Add/Remove from tree)
        col = row.column(align=True)

        # Add Menu: Shows joints from kinematic tree not yet added
        col.menu("LINKFORGE_MT_add_control_joint", icon="ADD", text="")
        col.operator("linkforge.remove_ros2_control_joint", icon="REMOVE", text="")
        col.separator()
        col.operator("linkforge.move_ros2_control_joint", icon="TRIA_UP", text="").direction = "UP"
        col.operator(
            "linkforge.move_ros2_control_joint", icon="TRIA_DOWN", text=""
        ).direction = "DOWN"
        col.separator()
        col.operator("linkforge.purge_ros2_control_data", icon="FILE_REFRESH", text="")

        # Settings for the selected joint
        if len(props.ros2_control_joints) > 0:
            # List of interfaces for each joint
            # Only draw details for the active joint
            active_idx = props.ros2_control_active_joint_index
            if 0 <= active_idx < len(props.ros2_control_joints):
                joint_item = props.ros2_control_joints[active_idx]
                self.draw_joint_details(layout, props, joint_item)

        # === 3. SIMULATION & GAZEBO ===
        layout.separator()
        box = layout.box()
        box.label(text="Gazebo Integration", icon="WORLD")

        box.prop(props, "gazebo_plugin_name")
        box.prop(props, "controllers_yaml_path")


class LINKFORGE_MT_add_control_joint(bpy.types.Menu):
    """Menu to add joints from the scene to ros2_control."""

    bl_label = "Add Joint"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        if not (layout and scene):
            return

        props = getattr(scene, "linkforge")

        # Get all joints from tree using centralized statistics
        stats = get_robot_statistics(scene)
        tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)

        # Get already added joint names and object references
        added_names = {item.name for item in props.ros2_control_joints}
        added_objs = {item.joint_obj for item in props.ros2_control_joints if item.joint_obj}

        joint_objs = []
        for obj in scene.objects:
            if (
                obj.type == "EMPTY"
                and hasattr(obj, "linkforge_joint")
                and getattr(obj, "linkforge_joint").is_robot_joint
            ):
                name = getattr(obj, "linkforge_joint").joint_name
                # Check if the exact object or the exact name was already added
                if obj not in added_objs and name not in added_names:
                    joint_objs.append((name, obj))

        if not joint_objs:
            layout.label(text="No more joints available")
            return

        for name, _ in sorted(joint_objs):
            op = layout.operator("linkforge.add_ros2_control_joint", text=name)
            op.joint_name = name


# Registration
classes = [
    LINKFORGE_UL_ros2_control_joints,
    LINKFORGE_PT_control,
    LINKFORGE_MT_add_control_joint,
]


def register() -> None:
    """Register panel."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister panel."""
    for cls in reversed(classes):
        with contextlib.suppress(RuntimeError):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
