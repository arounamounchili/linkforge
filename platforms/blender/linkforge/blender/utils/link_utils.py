"""Utilities for robot links in Blender.

This module contains helper functions for managing link objects,
their naming conventions, and hierarchy.
"""

from __future__ import annotations


def should_rename_child(child_name: str, parent_old_name: str) -> bool:
    """Check if a child object was auto-named by LinkForge and should be synced.

    Only renames if the child follows the exact [parent_old_name]_visual or
    [parent_old_name]_collision convention. Custom names are preserved.

    Args:
        child_name: Current name of the child object.
        parent_old_name: The previous name of the parent link before the current rename.

    Returns:
        True if the child is a standard LinkForge visual/collision object that should be renamed.
    """
    prefix_v = f"{parent_old_name}_visual"
    prefix_c = f"{parent_old_name}_collision"

    # Case 1: Perfect match (e.g., "base_link_visual")
    # Case 2: Suffix match (e.g., "base_link_visual.001" or "base_link_visual_mesh")
    # We check if the name starts with the prefix and the character immediately
    # following the prefix is a standard Blender separator (. or _).
    is_visual = child_name.startswith(prefix_v) and (
        len(child_name) == len(prefix_v) or child_name[len(prefix_v)] in "._"
    )
    is_collision = child_name.startswith(prefix_c) and (
        len(child_name) == len(prefix_c) or child_name[len(prefix_c)] in "._"
    )

    return is_visual or is_collision
