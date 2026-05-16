"""Kinematics utilities for the Blender platform."""

from __future__ import annotations

import bpy
from linkforge.core import Joint

from .property_helpers import get_joint_props


def resolve_mimic_joints(joints: list[Joint], joint_objects: dict[str, bpy.types.Object]) -> None:
    """Resolve mimic joint pointers after all joint objects have been created.

    Args:
        joints: List of joint models
        joint_objects: Dictionary mapping joint names to Blender objects
    """
    for joint in joints:
        if joint.mimic and joint.name in joint_objects:
            joint_obj = joint_objects[joint.name]
            if jp := get_joint_props(joint_obj):
                # Use joint_objects map to find the mimic target object
                if joint.mimic.joint in joint_objects:
                    mimic_target_obj = joint_objects[joint.mimic.joint]
                    jp.mimic_joint = mimic_target_obj
                    jp.use_mimic = True
                jp.mimic_multiplier = joint.mimic.multiplier
                jp.mimic_offset = joint.mimic.offset
