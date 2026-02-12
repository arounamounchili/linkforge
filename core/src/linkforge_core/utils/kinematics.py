"""Generic kinematics utilities for LinkForge core."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..models import Joint, Link


def sort_joints_topological(joints: list[Joint], links: list[Link]) -> list[Joint]:
    """Sort joints so parents are processed before children.

    This ensures that when building a hierarchy, the parent structure
    always exists before the child is attached.

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
    children_of: dict[str, list[Joint]] = {}
    for joint in joints:
        if joint.parent not in children_of:
            children_of[joint.parent] = []
        children_of[joint.parent].append(joint)

    # Traverse tree from roots, collecting joints in order
    sorted_joints: list[Joint] = []
    visited: set[str] = set()

    def visit(link_name: str) -> None:
        if link_name in visited:
            return
        visited.add(link_name)
        if link_name in children_of:
            for joint in children_of[link_name]:
                sorted_joints.append(joint)
                visit(joint.child)

    for root in root_links:
        visit(root)

    return sorted_joints
