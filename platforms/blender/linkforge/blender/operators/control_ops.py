from __future__ import annotations

import typing

import bpy
from bpy.props import IntProperty, StringProperty

from ..utils.decorators import OperatorReturn, safe_execute

if typing.TYPE_CHECKING:
    from bpy.types import Context, Operator

    from ..properties.robot_props import RobotPropertyGroup
else:
    # Runtime fallback for mock environments where bpy.types might be partially loaded.
    Context = typing.Any
    Operator = getattr(getattr(bpy, "types", object), "Operator", object)


class LINKFORGE_OT_add_ros2_control_joint(Operator):
    """Add a joint to the ros2_control system.

    This operator allows users to select a joint from the robot's kinematic
    tree and include it in the ROS 2 control configuration, setting up
    default command and state interfaces.
    """

    bl_idname = "linkforge.add_ros2_control_joint"
    bl_label = "Add Joint"
    bl_description = "Add a joint from the robot's kinematic tree to the control system"
    bl_options = {"REGISTER", "UNDO"}

    joint_name: StringProperty(name="Joint Name")  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operators can run.

        Args:
            context: The current Blender context.

        Returns:
            True if the scene has LinkForge properties initialized.
        """
        return hasattr(context.scene, "linkforge")

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'}).
        """
        scene = context.scene
        if not scene:
            return {"CANCELLED"}
        props = typing.cast("RobotPropertyGroup", getattr(scene, "linkforge"))

        # Find the target joint object we intend to add
        target_joint_obj = next(
            (
                obj
                for obj in scene.objects
                if obj.type == "EMPTY"
                and hasattr(obj, "linkforge_joint")
                and obj.linkforge_joint.is_robot_joint
                and obj.linkforge_joint.joint_name == self.joint_name
            ),
            None,
        )

        # Check if joint or its specific object already exists in collection
        for item in props.ros2_control_joints:
            # Check if name exactly matches OR physical object exactly matches
            if item.name == self.joint_name or (
                item.joint_obj is not None
                and target_joint_obj is not None
                and item.joint_obj == target_joint_obj
            ):
                self.report(
                    {"WARNING"}, f"Joint '{self.joint_name}' is already in the control system"
                )
                return {"CANCELLED"}

        # Add to collection
        item = props.ros2_control_joints.add()
        item.name = self.joint_name

        item.joint_obj = target_joint_obj

        # Set default interfaces
        item.cmd_position = True
        item.state_position = True
        item.state_velocity = True

        # Set as active index
        props.ros2_control_active_joint_index = len(props.ros2_control_joints) - 1

        self.report({"INFO"}, f"Added joint '{self.joint_name}' to control system")
        return {"FINISHED"}


class LINKFORGE_OT_remove_ros2_control_joint(Operator):
    """Remove a joint from the ros2_control system.

    This operator removes the currently selected joint from the ROS 2
    control configuration list.
    """

    bl_idname = "linkforge.remove_ros2_control_joint"
    bl_label = "Remove Joint"
    bl_description = "Remove the selected joint from the control system"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operators can run.

        Args:
            context: The current Blender context.

        Returns:
            True if there are joints in the control system.
        """
        if not context.scene:
            return False
        props = getattr(context.scene, "linkforge", None)
        return bool(props and len(props.ros2_control_joints) > 0)

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state.
        """
        if not context.scene:
            return {"CANCELLED"}
        props = typing.cast("RobotPropertyGroup", getattr(context.scene, "linkforge"))
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
    """Move a joint up or down in the control interface list.

    This is a UI helper operator to reorder how joints appear in the
    LinkForge control panel.
    """

    bl_idname = "linkforge.move_ros2_control_joint"
    bl_label = "Move Joint"
    bl_description = "Move joint up or down in the list (cosmetic only)"
    bl_options = {"REGISTER", "UNDO"}

    direction: StringProperty()  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if operators can run.

        Args:
            context: The current Blender context.

        Returns:
            True if there are multiple joints to move.
        """
        if not context.scene:
            return False
        props = getattr(context.scene, "linkforge", None)
        return bool(props and len(props.ros2_control_joints) > 1)

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the operator.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state.
        """
        if not context.scene:
            return {"CANCELLED"}
        props = typing.cast("RobotPropertyGroup", getattr(context.scene, "linkforge"))
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


class LINKFORGE_OT_add_ros2_control_parameter(Operator):
    """Add a parameter to ros2_control (global or joint).

    This operator adds a new key-value pair to either the global hardware
    parameters or the parameters of the currently selected joint.
    """

    bl_idname = "linkforge.add_ros2_control_parameter"
    bl_label = "Add Parameter"
    bl_description = "Add a key-value parameter to the control system"
    bl_options = {"REGISTER", "UNDO"}

    target: StringProperty(default="GLOBAL")  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if the operator can be executed.

        Args:
            context: The current Blender context.

        Returns:
            True if LinkForge properties are initialized in the scene.
        """
        return hasattr(context.scene, "linkforge")

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the addition of a parameter.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state.
        """
        scene = context.scene
        props = typing.cast("RobotPropertyGroup", getattr(scene, "linkforge"))

        if self.target == "GLOBAL":
            param = props.ros2_control_parameters.add()
            param.name = "param"
            param.value = "0.0"
        else:
            # Joint-level
            index = props.ros2_control_active_joint_index
            if 0 <= index < len(props.ros2_control_joints):
                joint = props.ros2_control_joints[index]
                param = joint.parameters.add()
                param.name = "param"
                param.value = "0.0"

        return {"FINISHED"}


class LINKFORGE_OT_remove_ros2_control_parameter(Operator):
    """Remove a parameter from ros2_control.

    This operator deletes a key-value pair from either the global
    hardware parameters or a specific joint's parameter list.
    """

    bl_idname = "linkforge.remove_ros2_control_parameter"
    bl_label = "Remove Parameter"
    bl_description = "Remove a hardware or joint parameter"
    bl_options = {"REGISTER", "UNDO"}

    target: StringProperty(default="GLOBAL")  # type: ignore
    index: IntProperty(default=-1)  # type: ignore

    @classmethod
    def poll(cls, context: Context) -> bool:
        """Check if the operator can be executed.

        Args:
            context: The current Blender context.

        Returns:
            True if LinkForge properties are initialized.
        """
        return hasattr(context.scene, "linkforge")

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the removal of a parameter.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state.
        """
        scene = context.scene
        props = typing.cast("RobotPropertyGroup", getattr(scene, "linkforge"))

        if self.target == "GLOBAL":
            items = props.ros2_control_parameters
            idx = self.index
            if 0 <= idx < len(items):
                items.remove(idx)
        else:
            # Joint-level
            joint_idx = props.ros2_control_active_joint_index
            if 0 <= joint_idx < len(props.ros2_control_joints):
                joint = props.ros2_control_joints[joint_idx]
                items = joint.parameters
                idx = self.index if self.index >= 0 else len(items) - 1
                if 0 <= idx < len(items):
                    items.remove(idx)

        return {"FINISHED"}


# Registration
classes = [
    LINKFORGE_OT_add_ros2_control_joint,
    LINKFORGE_OT_remove_ros2_control_joint,
    LINKFORGE_OT_move_ros2_control_joint,
    LINKFORGE_OT_add_ros2_control_parameter,
    LINKFORGE_OT_remove_ros2_control_parameter,
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
