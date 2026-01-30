"""Kinematics and hierarchy utilities for LinkForge."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...linkforge_core.models import Joint, Link


def sort_joints_topological(joints: list[Joint], links: list[Link]) -> list[Joint]:
    """Sort joints so parents are processed before children.

    This ensures that when building a hierarchy in Blender, the parent object
    always exists before the child object is parented to it.

    Args:
        joints: List of joint models to sort
        links: List of all link models in the robot

    Returns:
        Sorted list of joints
    """
    # Build a map of which links are children
    child_links = {j.child for j in joints}
    # Find root links (not children of any joint)
    root_links = {link.name for link in links if link.name not in child_links}

    # Build adjacency list: parent_link -> [joint, ...]
    children_of = {}
    for joint in joints:
        if joint.parent not in children_of:
            children_of[joint.parent] = []
        children_of[joint.parent].append(joint)

    # Traverse tree from roots, collecting joints in order
    sorted_joints = []
    visited = set()

    def visit(link_name):
        if link_name in visited:
            return
        visited.add(link_name)
        if link_name in children_of:
            for joint in children_of[link_name]:
                sorted_joints.append(joint)
                visit(joint.child)

    for root in root_links:
        visit(root)


def resolve_mimic_joints(joints: list[Joint], joint_objects: dict) -> None:
    """Resolve mimic joint pointers after all joint objects have been created.

    Args:
        joints: List of joint models
        joint_objects: Dictionary mapping joint names to Blender objects
    """
    for joint in joints:
        if joint.mimic and joint.name in joint_objects:
            joint_obj = joint_objects[joint.name]
            mimic_joint_obj = joint_objects.get(joint.mimic.joint)
            if mimic_joint_obj:
                joint_obj.linkforge_joint.mimic_joint = mimic_joint_obj
