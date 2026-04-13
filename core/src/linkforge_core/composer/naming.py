"""Robust utilities for robot composition and model building.

This module provides helpers for safely adding elements to a Robot model,
handling common real-world issues like duplicate names or broken references.
"""

from __future__ import annotations

from dataclasses import replace
from typing import TYPE_CHECKING

from ..exceptions import RobotModelError, RobotValidationError, ValidationErrorCode
from ..logging_config import get_logger

if TYPE_CHECKING:
    from ..models.joint import Joint
    from ..models.link import Link
    from ..models.robot import Robot

logger = get_logger(__name__)


def add_link_with_renaming(robot: Robot, link: Link) -> None:
    """Add a link to the robot, handling naming collisions via renaming.

    If a link with the same name already exists, a suffix (e.g., _duplicate_1)
    is appended to the new link's name until a unique name is found.

    Args:
        robot: The robot model to add the link to.
        link: The link object to add.
    """
    try:
        robot.add_link(link)
    except RobotModelError as e:
        original_name = link.name
        counter = 1
        if isinstance(e, RobotValidationError) and e.code == ValidationErrorCode.DUPLICATE_NAME:
            while True:
                new_name = f"{original_name}_duplicate_{counter}"
                if not robot.has_link(new_name):
                    link = replace(link, name=new_name)
                    try:
                        robot.add_link(link)
                        logger.warning(f"Renamed duplicate link '{original_name}' to '{new_name}'")
                        break
                    except RobotModelError as inner_e:
                        if (
                            isinstance(inner_e, RobotValidationError)
                            and inner_e.code == ValidationErrorCode.DUPLICATE_NAME
                        ):
                            counter += 1
                            continue
                        logger.warning(f"Skipping invalid link '{original_name}': {inner_e}")
                        break
                else:
                    counter += 1
        else:
            logger.warning(f"Skipping invalid link '{original_name}': {e}")


def add_joint_with_renaming(robot: Robot, joint: Joint, fallback_name: str | None = None) -> None:
    """Add a joint to the robot, handling naming collisions and broken references.

    Naming collisions are resolved by appending a numeric suffix. This method also
    gracefully handles cases where the parent or child links are missing.

    Args:
        robot: The robot model to add the joint to.
        joint: The joint object to add.
        fallback_name: Optional name used for logging if joint.name is unavailable.
    """
    try:
        robot.add_joint(joint)
    except RobotModelError as e:
        joint_name = joint.name or fallback_name or "unnamed_joint"
        if isinstance(e, RobotValidationError) and e.code == ValidationErrorCode.DUPLICATE_NAME:
            original_name = joint_name
            counter = 1
            while True:
                new_name = f"{original_name}_duplicate_{counter}"
                if not robot.has_joint(new_name):
                    joint = replace(joint, name=new_name)
                    try:
                        robot.add_joint(joint)
                        logger.warning(f"Renamed duplicate joint '{original_name}' to '{new_name}'")
                        break
                    except RobotModelError as inner_e:
                        if (
                            isinstance(inner_e, RobotValidationError)
                            and inner_e.code == ValidationErrorCode.DUPLICATE_NAME
                        ):
                            counter += 1
                            continue
                        logger.warning(f"Skipping invalid joint '{original_name}': {inner_e}")
                        break
                else:
                    counter += 1
        else:
            logger.warning(f"Skipping invalid joint '{joint_name}': {e}")
