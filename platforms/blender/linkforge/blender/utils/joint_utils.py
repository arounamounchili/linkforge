"""Kinematics utilities for the Blender platform."""

from __future__ import annotations

from ...linkforge_core.models import Joint


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
