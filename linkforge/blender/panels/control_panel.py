"""UI Panel for managing centralized robot control and transmissions."""

from __future__ import annotations

import bpy
from bpy.types import Context, Panel, UIList

from .robot_panel import build_tree_structure


class LINKFORGE_UL_ros2_control_joints(UIList):
    """UI List for ros2_control joints."""

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        """Draw an item in the list."""
        if self.layout_type in {"DEFAULT", "COMPACT"}:
            row = layout.row(align=True)
            row.label(text=item.name, icon="EMPTY_AXIS")

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


class LINKFORGE_PT_control(Panel):
    """Panel for configuring motor control and centralized ros2_control."""

    bl_label = "Control"
    bl_description = "Step 3: Configure centralized ros2_control system and interfaces"
    bl_idname = "LINKFORGE_PT_control"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 2
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context: Context):
        """Draw the panel."""
        layout = self.layout
        scene = context.scene
        props = scene.linkforge

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

        # Settings for the selected joint
        if 0 <= props.ros2_control_active_joint_index < len(props.ros2_control_joints):
            joint_item = props.ros2_control_joints[props.ros2_control_active_joint_index]

            inner = box.box()
            inner.label(text=f"Config: {joint_item.name}", icon="SETTINGS")

            # Command Interfaces
            row = inner.row()
            col = row.column(align=True)
            col.label(text="Command Interfaces:")
            col.prop(joint_item, "cmd_position")
            col.prop(joint_item, "cmd_velocity")
            col.prop(joint_item, "cmd_effort")

            # State Interfaces
            col = row.column(align=True)
            col.label(text="State Interfaces:")
            col.prop(joint_item, "state_position")
            col.prop(joint_item, "state_velocity")
            col.prop(joint_item, "state_effort")

        # === 3. SIMULATION & GAZEBO ===
        layout.separator()
        box = layout.box()
        box.label(text="Gazebo Integration", icon="WORLD")

        box.prop(props, "gazebo_plugin_name")
        box.prop(props, "controllers_yaml_path")


class LINKFORGE_MT_add_control_joint(bpy.types.Menu):
    """Menu to add joints from the scene to ros2_control."""

    bl_label = "Add Joint"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        props = scene.linkforge

        # Get all joints from tree
        tree, root_link, joints_dict, links_dict = build_tree_structure(scene)

        # Get already added joint names
        added_joints = {item.name for item in props.ros2_control_joints}

        joint_objs = []
        for obj in scene.objects:
            if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint:
                name = obj.linkforge_joint.joint_name
                if name not in added_joints:
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


def register():
    """Register panel."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister():
    """Unregister panel."""
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except RuntimeError:
            pass


if __name__ == "__main__":
    register()
