"""Operators for managing centralized ros2_control joints."""

from __future__ import annotations

import bpy
from bpy.props import StringProperty
from bpy.types import Context, Operator

from ..utils.decorators import safe_execute


class LINKFORGE_OT_add_ros2_control_joint(Operator):
    """Add a joint to the ros2_control system"""

    bl_idname = "linkforge.add_ros2_control_joint"
    bl_label = "Add Joint"
    bl_description = "Add a joint from the robot's kinematic tree to the control system"
    bl_options = {"REGISTER", "UNDO"}

    joint_name: StringProperty(name="Joint Name")  # type: ignore

    @classmethod
    def poll(cls, context: Context):
        """Check if operators can run."""
        return hasattr(context.scene, "linkforge")

    @safe_execute
    def execute(self, context: Context):
        """Execute the operator."""
        scene = context.scene
        props = scene.linkforge

        # Check if joint already exists in collection
        for item in props.ros2_control_joints:
            if item.name == self.joint_name:
                self.report(
                    {"WARNING"}, f"Joint '{self.joint_name}' is already in the control system"
                )
                return {"CANCELLED"}

        # Add to collection
        item = props.ros2_control_joints.add()
        item.name = self.joint_name

        # Set default interfaces
        item.cmd_position = True
        item.state_position = True
        item.state_velocity = True

        # Set as active index
        props.ros2_control_active_joint_index = len(props.ros2_control_joints) - 1

        self.report({"INFO"}, f"Added joint '{self.joint_name}' to control system")
        return {"FINISHED"}


class LINKFORGE_OT_remove_ros2_control_joint(Operator):
    """Remove a joint from the ros2_control system"""

    bl_idname = "linkforge.remove_ros2_control_joint"
    bl_label = "Remove Joint"
    bl_description = "Remove the selected joint from the control system"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context):
        """Check if operators can run."""
        props = getattr(context.scene, "linkforge", None)
        return props and len(props.ros2_control_joints) > 0

    @safe_execute
    def execute(self, context: Context):
        """Execute the operator."""
        props = context.scene.linkforge
        index = props.ros2_control_active_joint_index

        if 0 <= index < len(props.ros2_control_joints):
            joint_name = props.ros2_control_joints[index].name
            props.ros2_control_joints.remove(index)

            # Update active index
            props.ros2_control_active_joint_index = max(0, index - 1)

            self.report({"INFO"}, f"Removed joint '{joint_name}' from control system")
            return {"FINISHED"}

        return {"CANCELLED"}


class LINKFORGE_OT_move_ros2_control_joint(Operator):
    """Move a joint up or down in the control interface list"""

    bl_idname = "linkforge.move_ros2_control_joint"
    bl_label = "Move Joint"
    bl_description = "Move joint up or down in the list (cosmetic only)"
    bl_options = {"REGISTER", "UNDO"}

    direction: StringProperty()  # type: ignore

    @classmethod
    def poll(cls, context: Context):
        """Check if operators can run."""
        props = getattr(context.scene, "linkforge", None)
        return props and len(props.ros2_control_joints) > 1

    @safe_execute
    def execute(self, context: Context):
        """Execute the operator."""
        props = context.scene.linkforge
        index = props.ros2_control_active_joint_index
        new_index = index

        if self.direction == "UP" and index > 0:
            new_index = index - 1
        elif self.direction == "DOWN" and index < len(props.ros2_control_joints) - 1:
            new_index = index + 1
        else:
            return {"CANCELLED"}

        props.ros2_control_joints.move(index, new_index)
        props.ros2_control_active_joint_index = new_index
        return {"FINISHED"}


# Registration
classes = [
    LINKFORGE_OT_add_ros2_control_joint,
    LINKFORGE_OT_remove_ros2_control_joint,
    LINKFORGE_OT_move_ros2_control_joint,
]


def register() -> None:
    """Register operators."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister operators."""
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
