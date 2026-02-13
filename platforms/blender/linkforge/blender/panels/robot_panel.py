"""UI Panel for robot-level properties and validation."""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.props import StringProperty
from bpy.types import Context, Object, Operator, Scene

from ..utils.decorators import safe_execute


def build_tree_structure(
    scene: Scene | None,
) -> tuple[
    dict[str, list[tuple[str, str, str]]],
    str | None,
    dict[tuple[str, str], Object],
    dict[str, Object],
]:
    """Build a kinematic tree from the scene objects.

    Iterates through all objects in the scene to reconstruct the parent-child relationships
    defined by LinkForge joints.

    Args:
        scene: The Blender scene to analyze.

    Returns:
        A tuple containing:
        - tree: Dictionary mapping parent link names to lists of (child_name, joint_name, joint_type).
        - root_link: The name of the root link (link with no parent), or None if not found.
        - joints: Dictionary mapping (parent_name, child_name) tuples to the joint Object.
        - links: Dictionary mapping link names to their Blender Objects.
    """
    if not scene:
        return {}, None, {}, {}
    # Collect all links
    links = {
        obj.linkforge.link_name: obj
        for obj in scene.objects
        if hasattr(obj, "linkforge") and typing.cast(typing.Any, obj).linkforge.is_robot_link
    }

    # Build parent->children mapping from joints
    tree: dict[str, list[tuple[str, str, str]]] = {link_name: [] for link_name in links}
    joints = {}
    root_link = None

    for obj in scene.objects:
        if (
            obj.type == "EMPTY"
            and hasattr(obj, "linkforge_joint")
            and typing.cast(typing.Any, obj).linkforge_joint.is_robot_joint
        ):
            props = typing.cast(typing.Any, obj).linkforge_joint
            parent = props.parent_link
            child = props.child_link

            if parent and child and parent in tree:
                tree[parent].append((child, props.joint_name, props.joint_type))
                joints[(parent, child)] = obj

    # Find root (link with no parent)
    all_children = set()
    for children_list in tree.values():
        for child, _, _ in children_list:
            all_children.add(child)

    # Root is a link that appears as parent but not as child
    for link_name in links:
        if link_name not in all_children:
            root_link = link_name
            break

    return tree, root_link, joints, links


class LINKFORGE_OT_select_tree_object(Operator):
    """Select object from kinematic tree."""

    bl_idname = "linkforge.select_tree_object"
    bl_label = "Select Object"
    bl_description = "Select this object in the 3D viewport"
    bl_options = {"REGISTER", "UNDO"}

    object_name: bpy.props.StringProperty(  # type: ignore
        name="Object Name", description="Name of the object to select"
    )
    object_type: bpy.props.StringProperty(  # type: ignore
        name="Object Type", description="Type of object (link/joint)", default="link"
    )
    joint_name = StringProperty(name="Joint Name")  # type: ignore[func-returns-value]
    parent_link = StringProperty(name="Parent Link")  # type: ignore[func-returns-value]
    child_link = StringProperty(name="Child Name")  # type: ignore[func-returns-value]

    @safe_execute
    def execute(self, context: Context) -> set[str]:
        """Execute the operator.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        # Find the object
        scene = context.scene
        if not scene:
            return {"CANCELLED"}
        obj = scene.objects.get(self.object_name)
        if not obj:
            self.report({"WARNING"}, f"Object '{self.object_name}' not found")
            return {"CANCELLED"}

        # Deselect all
        bpy.ops.object.select_all(action="DESELECT")

        # Select and activate the object
        # Select and activate the object
        obj.select_set(True)
        vl = context.view_layer
        if vl:
            vl.objects.active = obj

        return {"FINISHED"}


class LINKFORGE_OT_select_root_link(Operator):
    """Select the root link of the robot."""

    bl_idname = "linkforge.select_root_link"
    bl_label = "Select Root Link"
    bl_description = "Select the root link of the robot in the 3D viewport"
    bl_options = {"REGISTER", "UNDO"}

    @safe_execute
    def execute(self, context: Context) -> set[str]:
        """Execute the operator.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        scene = context.scene
        if not scene:
            return {"CANCELLED"}
        _, root_link, _, _ = build_tree_structure(scene)

        # Select the root link
        if root_link:
            root_obj = scene.objects.get(root_link)
            if root_obj:
                bpy.ops.object.select_all(action="DESELECT")
                root_obj.select_set(True)
                vl = context.view_layer
                if vl:
                    vl.objects.active = root_obj

            return {"FINISHED"}
        else:
            self.report({"WARNING"}, "No root link found for the robot.")
            return {"CANCELLED"}


# Registration
classes = [
    LINKFORGE_OT_select_tree_object,
    LINKFORGE_OT_select_root_link,
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
