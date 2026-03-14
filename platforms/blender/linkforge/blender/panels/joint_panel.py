"""UI Panel for managing robot joints."""

from __future__ import annotations

import contextlib

import bpy
from bpy.types import Context, Panel


class LINKFORGE_PT_joints(Panel):
    """Panel for robot joint properties in 3D Viewport sidebar."""

    bl_label = "Joints"
    bl_idname = "LINKFORGE_PT_joints"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_parent_id = "LINKFORGE_PT_forge"
    bl_order = 2

    def draw(self, context: Context) -> None:
        """Draw the panel."""
        layout = self.layout
        if not layout:
            return

        obj = context.active_object

        # Check if selected object is a joint (edit mode vs create mode)
        is_joint = (
            obj
            and obj.select_get()
            and obj.type == "EMPTY"
            and hasattr(obj, "linkforge_joint")
            and getattr(obj, "linkforge_joint").is_robot_joint
        )

        # Only show Create button when NOT editing a joint
        if not is_joint:
            # Detect target link (either selected link or parent of selected visual)
            target_link = None
            if obj and obj.select_get():
                if hasattr(obj, "linkforge") and getattr(obj, "linkforge").is_robot_link:
                    target_link = obj
                elif (
                    obj.parent
                    and hasattr(obj.parent, "linkforge")
                    and getattr(obj.parent, "linkforge").is_robot_link
                ):
                    # Selected object is a visual/collision child of a link
                    target_link = obj.parent

            # Create Joint button (creation mode)
            box = layout.box()
            row = box.row()
            row.enabled = target_link is not None
            row.operator("linkforge.create_joint", icon="ADD", text="Create Joint")

        # Show joint properties only if a joint is selected (edit mode)
        if not is_joint or not obj:
            return

        props = getattr(obj, "linkforge_joint")

        # Joint properties
        box = layout.box()
        box.label(text=f"Joint: {obj.name}", icon="EMPTY_ARROWS")
        box.prop(props, "joint_name")

        # Joint type
        box.prop(props, "joint_type")

        # Parent and child
        box.separator()
        row = box.row()
        row.label(text="Connection:", icon="LINKED")
        row.operator("linkforge.auto_detect_parent_child", icon="AUTO", text="Detect")

        box.prop(props, "parent_link", icon="OUTLINER_OB_EMPTY")
        box.prop(props, "child_link", icon="OUTLINER_OB_EMPTY")

        # Joint axis (only for revolute, continuous, prismatic)
        if props.joint_type in {"REVOLUTE", "CONTINUOUS", "PRISMATIC"}:
            box.separator()
            box.label(text="Axis:", icon="ORIENTATION_GIMBAL")
            box.prop(props, "axis", expand=True)

            if props.axis == "CUSTOM":
                col = box.column(align=True)
                col.prop(props, "custom_axis_x", text="X")
                col.prop(props, "custom_axis_y", text="Y")
                col.prop(props, "custom_axis_z", text="Z")

        # Joint limits (dynamic based on joint type per URDF spec)
        if props.joint_type in {"REVOLUTE", "PRISMATIC"}:
            # REVOLUTE/PRISMATIC: Limits are REQUIRED (no checkbox)
            box.separator()
            col = box.column(align=True)
            col.label(text="Limits (Required):", icon="DRIVER_DISTANCE")
            col.prop(props, "limit_lower")
            col.prop(props, "limit_upper")
            col.prop(props, "limit_effort")
            col.prop(props, "limit_velocity")
        elif props.joint_type == "CONTINUOUS":
            # CONTINUOUS: Limits are OPTIONAL (show checkbox, only effort/velocity)
            box.separator()
            box.prop(props, "use_limits", text="Use Limits (Optional)")
            if props.use_limits:
                col = box.column(align=True)
                col.label(text="Effort & Velocity Limits:", icon="DRIVER_DISTANCE")
                col.prop(props, "limit_effort")
                col.prop(props, "limit_velocity")
        # FIXED/FLOATING/PLANAR: No limits section (not allowed per URDF spec)

        # Joint Dynamics settings (optional)
        box.separator()
        box.prop(props, "use_dynamics")
        if props.use_dynamics:
            col = box.column(align=True)
            col.prop(props, "dynamics_damping")
            col.prop(props, "dynamics_friction")

        # Joint Mimic settings (optional)
        box.separator()
        box.prop(props, "use_mimic")
        if props.use_mimic:
            box.prop(props, "mimic_joint")
            col = box.column(align=True)
            col.prop(props, "mimic_multiplier")
            col.prop(props, "mimic_offset")

        # Joint Safety Controller (optional)
        box.separator()
        box.prop(props, "use_safety_controller")
        if props.use_safety_controller:
            col = box.column(align=True)
            col.prop(props, "safety_soft_lower_limit")
            col.prop(props, "safety_soft_upper_limit")
            col.prop(props, "safety_k_position")
            col.prop(props, "safety_k_velocity")

        # Joint Calibration (optional)
        box.separator()
        box.prop(props, "use_calibration")
        if props.use_calibration:
            col = box.column(align=True)

            # Rising edge with its own toggle
            row = col.row(align=True)
            row.prop(props, "use_calibration_rising", text="")
            sub = row.row()
            sub.active = props.use_calibration_rising
            sub.prop(props, "calibration_rising", text="Rising")

            # Falling edge with its own toggle
            row = col.row(align=True)
            row.prop(props, "use_calibration_falling", text="")
            sub = row.row()
            sub.active = props.use_calibration_falling
            sub.prop(props, "calibration_falling", text="Falling")

        # Remove Joint button (Danger Zone)
        box.separator()
        box.separator()
        row = box.row()
        row.scale_y = 1.2  # Make it slightly bigger
        row.operator("linkforge.delete_joint", icon="TRASH", text="Remove Joint")


# Registration
classes = [
    LINKFORGE_PT_joints,
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
