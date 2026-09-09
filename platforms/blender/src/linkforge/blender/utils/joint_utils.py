"""Kinematics utilities for the Blender platform."""

from __future__ import annotations

import contextlib

import bpy

from ..core import Joint
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


def is_control_joint_missing(item: bpy.types.PropertyGroup, scene: bpy.types.Scene | None) -> bool:
    """Check if the physical joint object for a ros2_control joint is missing from the scene.

    Args:
        item: Ros2ControlJointProperty instance
        scene: Active Blender scene

    Returns:
        bool: True if the joint object no longer exists in the active scene.
    """
    if not scene:
        return True

    joint_obj = getattr(item, "joint_obj", None)
    if joint_obj is not None:
        try:
            if scene.objects.get(joint_obj.name) == joint_obj:
                jp = get_joint_props(joint_obj)
                return not (jp and jp.is_robot_joint)
        except (ReferenceError, AttributeError):
            return True
        return True

    # Fast O(1) name lookup in Blender C GHash (avoids iterating scene.objects)
    item_name = getattr(item, "name", "")
    if (
        item_name
        and (obj := scene.objects.get(item_name))
        and (jp := get_joint_props(obj))
        and jp.is_robot_joint
    ):
        with contextlib.suppress(Exception):
            item.joint_obj = obj  # type: ignore[attr-defined]
        return False

    return True
