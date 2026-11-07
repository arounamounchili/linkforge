"""Kinematics utilities for the Blender platform."""

from __future__ import annotations

import contextlib
from typing import Any

import bpy

from ..constants import (
    AXIS_CUSTOM,
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
)
from ..core import Joint
from ..core._utils.math_utils import normalize_vector
from ..core.constants import DEFAULT_AXIS_XYZ, SYLVESTER_TOLERANCE_EPSILON
from . import scene_utils
from .property_helpers import get_joint_props


def get_joint_axis_vector(
    jp: Any,
    normalize: bool = True,
) -> tuple[float, float, float]:
    """Return 3D axis vector for a robot joint in its local frame.

    Args:
        jp: JointPropertyGroup or object with axis attributes.
        normalize: Whether to normalize custom axis vectors.

    Returns:
        tuple[float, float, float]: Normalized or raw axis vector (x, y, z).
    """
    axis = getattr(jp, "axis", AXIS_Z)
    if axis == AXIS_X:
        return (1.0, 0.0, 0.0)
    if axis == AXIS_Y:
        return (0.0, 1.0, 0.0)
    if axis == AXIS_Z:
        return (0.0, 0.0, 1.0)
    if axis == AXIS_CUSTOM:
        cx = float(getattr(jp, "custom_axis_x", 0.0))
        cy = float(getattr(jp, "custom_axis_y", 0.0))
        cz = float(getattr(jp, "custom_axis_z", 1.0))
        if normalize:
            norm = normalize_vector(cx, cy, cz)
            return norm if any(norm) else (0.0, 0.0, 1.0)
        # Fallback for near-zero axis
        if all(abs(v) < SYLVESTER_TOLERANCE_EPSILON for v in (cx, cy, cz)):
            return DEFAULT_AXIS_XYZ
        return (cx, cy, cz)
    return (0.0, 0.0, 1.0)


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


def get_connected_joints_for_link(
    link_obj: bpy.types.Object | None,
    scene: bpy.types.Scene | None = None,
) -> tuple[bpy.types.Object | None, list[bpy.types.Object]]:
    """Return incoming parent joint and outgoing child joints for a given link object.

    Args:
        link_obj: The link object in Blender.
        scene: Optional active scene. If not provided, uses bpy.context.scene.

    Returns:
        tuple[bpy.types.Object | None, list[bpy.types.Object]]:
            (parent_joint, list_of_child_joints)
    """
    if link_obj is None:
        return None, []

    if scene is None:
        scene = getattr(bpy.context, "scene", None)
    if scene is None:
        return None, []

    parent_joint: bpy.types.Object | None = None
    child_joints: list[bpy.types.Object] = []

    stats = scene_utils.get_robot_statistics(scene)
    all_joints = getattr(stats, "joint_objects", []) or [
        o
        for o in getattr(scene, "objects", [])
        if (jp := get_joint_props(o)) and getattr(jp, "is_robot_joint", False)
    ]

    for j_obj in all_joints:
        try:
            jp = get_joint_props(j_obj)
            if not jp or not getattr(jp, "is_robot_joint", False):
                continue

            # Incoming joint (where this link is the child)
            if getattr(jp, "child_link", None) == link_obj:
                parent_joint = j_obj

            # Outgoing joint (where this link is the parent)
            if getattr(jp, "parent_link", None) == link_obj:
                child_joints.append(j_obj)
        except (ReferenceError, AttributeError):
            continue

    child_joints.sort(key=lambda x: getattr(x, "name", ""))
    return parent_joint, child_joints
