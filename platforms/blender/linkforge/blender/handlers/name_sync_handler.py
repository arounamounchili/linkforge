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
    """Synchronize LinkForge identities when objects are renamed in the Outliner.

    This ensures that internal robot properties (link_name, joint_name) stay in
    sync with the Blender object names.

    Args:
        scene: The current Blender scene.
        _depsgraph: The dependency graph that triggered the update.
    """
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

        # Check Sensors
        if hasattr(obj, "linkforge_sensor"):
            sf = obj.linkforge_sensor
            if sf.is_robot_sensor and obj.name != sf.sensor_name:
                sf.sensor_name = obj.name

        # Check Transmissions
        if hasattr(obj, "linkforge_transmission"):
            tf = obj.linkforge_transmission
            if tf.is_robot_transmission and obj.name != tf.transmission_name:
                tf.transmission_name = obj.name


def register() -> None:
    """Register name sync handler."""
    if on_depsgraph_update_post not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(on_depsgraph_update_post)


def unregister() -> None:
    """Unregister name sync handler."""
    if on_depsgraph_update_post in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(on_depsgraph_update_post)
