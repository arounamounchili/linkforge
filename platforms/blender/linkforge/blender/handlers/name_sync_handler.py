"""Handler for synchronizing LinkForge names with Blender object names.

This ensures that renaming an object in the Outliner or duplicating it
automatically updates the corresponding LinkForge URDF identity.
"""

from __future__ import annotations

import typing

import bpy
from bpy.app.handlers import persistent

if typing.TYPE_CHECKING:
    from ..properties.joint_props import JointPropertyGroup
    from ..properties.link_props import LinkPropertyGroup


@persistent  # type: ignore[misc]
def on_depsgraph_update_post(_scene: bpy.types.Scene, _depsgraph: bpy.types.Depsgraph) -> None:
    """Detect name changes and duplication during depsgraph update."""
    for obj in bpy.data.objects:
        # Check Links
        if hasattr(obj, "linkforge"):
            lf: LinkPropertyGroup = obj.linkforge
            if lf.is_robot_link and obj.name != lf.link_name:
                lf.link_name = obj.name

        # Check Joints
        if hasattr(obj, "linkforge_joint"):
            jf: JointPropertyGroup = obj.linkforge_joint
            if jf.is_robot_joint and obj.name != jf.joint_name:
                jf.joint_name = obj.name


def register() -> None:
    """Register name sync handler."""
    if on_depsgraph_update_post not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(on_depsgraph_update_post)


def unregister() -> None:
    """Unregister name sync handler."""
    if on_depsgraph_update_post in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(on_depsgraph_update_post)
