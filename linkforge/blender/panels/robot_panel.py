"""UI Panel for robot-level properties and validation."""

from __future__ import annotations

import bpy
from bpy.types import Operator

from ..utils.decorators import safe_execute


def build_tree_structure(scene):
    """Build robot tree structure from scene objects.

    Returns:
        tuple: (tree dict, root_link str, joints dict, links dict)
            - tree: mapping parent links to list of (child_link, joint_name, joint_type)
            - root_link: name of root link
            - joints: dict mapping (parent, child) to joint object
            - links: dict mapping link_name to link object

    """
    # Collect all links
    links = {obj.linkforge.link_name: obj for obj in scene.objects if obj.linkforge.is_robot_link}

    # Build parent->children mapping from joints
    tree = {link_name: [] for link_name in links}
    joints = {}
    root_link = None

    for obj in scene.objects:
        if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint:
            props = obj.linkforge_joint
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

    object_name: bpy.props.StringProperty(
        name="Object Name", description="Name of the object to select"
    )  # type: ignore
    object_type: bpy.props.StringProperty(
        name="Object Type", description="Type of object (link, joint, sensor, transmission)"
    )  # type: ignore

    @safe_execute
    def execute(self, context):
        """Execute the operator."""
        # Find the object
        obj = context.scene.objects.get(self.object_name)
        if not obj:
            self.report({"WARNING"}, f"Object '{self.object_name}' not found")
            return {"CANCELLED"}

        # Deselect all
        bpy.ops.object.select_all(action="DESELECT")

        # Select and activate the object
        obj.select_set(True)
        context.view_layer.objects.active = obj

        return {"FINISHED"}


# Registration
classes = [
    LINKFORGE_OT_select_tree_object,
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
