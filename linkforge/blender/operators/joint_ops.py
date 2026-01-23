"""Operators for managing robot joints."""

from __future__ import annotations

import bpy
from bpy.types import Context, Operator

from ..properties.link_props import sanitize_urdf_name
from ..utils.decorators import safe_execute


class LINKFORGE_OT_create_joint(Operator):
    """Create a new robot joint at selected link's location"""

    bl_idname = "linkforge.create_joint"
    bl_label = "Create Joint"
    bl_description = "Create a new robot joint at the selected link's location and orientation"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context):
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False

        # Only allow if object is selected
        if not obj.select_get():
            return False

        # Allow if object is a link or a child of a link (visual mesh)
        return obj.linkforge.is_robot_link or (
            obj.parent and hasattr(obj.parent, "linkforge") and obj.parent.linkforge.is_robot_link
        )

    @safe_execute
    def execute(self, context: Context):
        """Execute the operator."""
        obj = context.active_object

        # Get the link object (either selected directly or parent of selected visual)
        link_obj = obj if obj.linkforge.is_robot_link else obj.parent

        # Get preferred empty size from addon preferences
        from ..preferences import get_addon_prefs

        addon_prefs = get_addon_prefs(context)

        # Initialize default size
        empty_size = 0.2

        if addon_prefs:
            empty_size = getattr(addon_prefs, "joint_empty_size", empty_size)

        # Get link object's world location and rotation
        location = link_obj.matrix_world.translation.copy()
        rotation = link_obj.matrix_world.to_euler()

        # Create Empty at link's location (ARROWS shows RGB colored axes)
        bpy.ops.object.empty_add(type="ARROWS", location=location)
        joint_empty = context.active_object
        joint_empty.name = f"{link_obj.linkforge.link_name}_joint"
        joint_empty.rotation_euler = rotation

        # Move joint to same collection as link (for clean organization)
        # Remove from all current collections
        for coll in list(joint_empty.users_collection):
            coll.objects.unlink(joint_empty)
        # Add to link's collection
        if link_obj.users_collection:
            parent_collection = link_obj.users_collection[0]
            parent_collection.objects.link(joint_empty)

        # Set display size from preferences
        joint_empty.empty_display_size = empty_size

        # Enable joint properties
        joint_empty.linkforge_joint.is_robot_joint = True
        joint_empty.linkforge_joint.joint_name = sanitize_urdf_name(joint_empty.name)

        # Set default joint type
        joint_empty.linkforge_joint.joint_type = "REVOLUTE"

        # Enable limits by default for REVOLUTE joints (they typically need them)
        # User can disable if not needed
        joint_empty.linkforge_joint.use_limits = True

        # Ensure matrices are up to date before triggering property callbacks
        # This prevents transform jumps when setting child_link (which sets parent)
        context.view_layer.update()

        # Auto-set child link to the selected link (parent must be set manually)
        joint_empty.linkforge_joint.child_link = link_obj

        self.report(
            {"INFO"},
            f"Created joint '{joint_empty.name}' for child '{link_obj.linkforge.link_name}' (set parent manually)",
        )
        return {"FINISHED"}


class LINKFORGE_OT_delete_joint(Operator):
    """Delete the selected joint Empty"""

    bl_idname = "linkforge.delete_joint"
    bl_label = "Remove Joint"
    bl_description = "Remove the selected joint from the robot structure"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: Context):
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False
        return obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint

    @safe_execute
    def execute(self, context: Context):
        """Execute the operator."""
        obj = context.active_object
        joint_name = obj.name

        # Remove from ROS2 Control list if present (Maintain Consistency)
        scene = context.scene
        if hasattr(scene, "linkforge") and hasattr(scene.linkforge, "ros2_control_joints"):
            rc_joints = scene.linkforge.ros2_control_joints
            # Find index by name
            idx_to_remove = -1
            for i, item in enumerate(rc_joints):
                if item.name == joint_name:
                    idx_to_remove = i
                    break

            if idx_to_remove >= 0:
                rc_joints.remove(idx_to_remove)
                self.report({"INFO"}, f"Removed '{joint_name}' from ROS2 Control")

        # Delete the Empty entirely
        bpy.data.objects.remove(obj, do_unlink=True)

        self.report({"INFO"}, f"Deleted joint '{joint_name}'")
        return {"FINISHED"}


class LINKFORGE_OT_auto_detect_parent_child(Operator):
    """Auto-detect parent and child links based on hierarchy"""

    bl_idname = "linkforge.auto_detect_parent_child"
    bl_label = "Auto-Detect Links"
    bl_description = "Automatically detect parent and child links from object hierarchy"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        if obj is None:
            return False
        if not obj.select_get():
            return False
        return obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint

    @safe_execute
    def execute(self, context):
        """Execute the operator."""
        joint_empty = context.active_object
        props = joint_empty.linkforge_joint

        # Find nearest links based on distance
        joint_loc = joint_empty.location
        links = [
            (obj, (obj.location - joint_loc).length)
            for obj in context.scene.objects
            if obj.linkforge.is_robot_link
        ]

        if not links:
            self.report({"WARNING"}, "No robot links found in scene")
            return {"CANCELLED"}

        # Sort by distance
        links.sort(key=lambda x: x[1])

        # Force property update to refresh enum items
        bpy.context.view_layer.update()

        # Try to set parent and child links with "Smart Choice" logic
        # Pro Rule: Joint Origin is usually coincident with Child Origin.
        # So Closest = Child, Second Closest = Parent.
        try:
            if len(links) >= 2:
                link_a = links[0][0]
                link_b = links[1][0]

                # If child is already set (standard workflow), keep it and find parent
                if props.child_link == link_a:
                    props.parent_link = link_b
                elif props.child_link == link_b:
                    props.parent_link = link_a
                else:
                    # Nothing set or something far away set - use defaults
                    props.child_link = link_a
                    props.parent_link = link_b

                self.report(
                    {"INFO"}, f"Connected: {props.parent_link.name} -> {props.child_link.name}"
                )
            elif len(links) == 1:
                # Only one link - must be child
                props.child_link = links[0][0]
                self.report({"INFO"}, f"Set child: {props.child_link.name} (No parent nearby)")
        except Exception as e:
            self.report({"WARNING"}, f"Auto-detect failed: {str(e)}")

        return {"FINISHED"}


# Registration
classes = [
    LINKFORGE_OT_create_joint,
    LINKFORGE_OT_delete_joint,
    LINKFORGE_OT_auto_detect_parent_child,
]


def register():
    """Register operators."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister():
    """Unregister operators."""
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except RuntimeError:
            pass


if __name__ == "__main__":
    register()
