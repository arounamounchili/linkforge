"""General Blender utility functions for object and collection management."""

from __future__ import annotations

import contextlib
import typing
from dataclasses import dataclass, field
from typing import Any

import bpy


def is_robot_link(obj: Any) -> bool:
    """Check if blender obj is a robot_link.

    Args:
        obj: Blender object to be checked

    Returns:
        True if object has linkforge properties and is marked as robot_link
    """
    return hasattr(obj, "linkforge") and obj.linkforge.is_robot_link


def is_robot_joint(obj: Any) -> bool:
    """Check if blender obj is a robot_joint.

    Args:
        obj: Blender object to be checked

    Returns:
        True if object is EMPTY with linkforge_joint properties marked as robot_joint
    """
    return (
        getattr(obj, "type", None) == "EMPTY"
        and hasattr(obj, "linkforge_joint")
        and obj.linkforge_joint.is_robot_joint
    )


def is_robot_sensor(obj: Any) -> bool:
    """Check if blender obj is a robot_sensor.

    Args:
        obj: Blender object to check

    Returns:
        True if object is EMPTY with linkforge_sensor properties marked as robot_sensor
    """
    return (
        getattr(obj, "type", None) == "EMPTY"
        and hasattr(obj, "linkforge_sensor")
        and obj.linkforge_sensor.is_robot_sensor
    )


def is_robot_transmission(obj: Any) -> bool:
    """Check if blender obj is a robot_transmission.

    Args:
        obj: Blender object to check

    Returns:
        True if object has linkforge_transmission properties marked as robot_transmission
    """
    return (
        getattr(obj, "type", None) == "EMPTY"
        and hasattr(obj, "linkforge_transmission")
        and obj.linkforge_transmission.is_robot_transmission
    )


@dataclass(frozen=True)
class RobotSceneStatistics:
    """Statistics/Properties about robot components within a scene.

    Attributes:
        num_links: Total number of robot_link objects in scene
        total_mass: Sum of all robot_link masses (kg)
        total_dof: Total degrees of freedom from all robot_joint objects in scene
        link_objects: Mapping of robot_link names to their corresponding blender objects
        joint_objects: List of all robot_joint objects in scene
        sensor_objects: List of all robot_sensor objects in scene
        transmission_objects: List of all robot_transmission objects in scene
        root_link: Tuple of (link_name, object) for root link, or None if not found
    """

    num_links: int
    total_mass: float
    total_dof: int
    link_objects: dict[str, Any]
    joint_objects: list[Any]
    sensor_objects: list[Any]
    transmission_objects: list[Any]
    root_link: tuple[str, Any] | None
    # Map from child link name -> (parent link name, joint object)
    joints_map: dict[str, tuple[str, Any]] = field(default_factory=dict)


def get_robot_statistics(scene: Any) -> RobotSceneStatistics:
    """Analyze scene and setup robot statistics/properties.

    Categorizes all robot components (links, joints, sensors, transmissions)
    and calc total link mass and DOFs from all joints found.

    Args:
        scene: Blender scene to be analyzed

    Returns:
        RobotSceneStatistics with all pre-calculated properties. Empty if scene is None or has no objects.
    """
    link_objects: dict[str, Any] = {}
    joint_objects: list[Any] = []
    sensor_objects: list[Any] = []
    transmission_objects: list[Any] = []
    total_mass = 0.0
    total_dof = 0
    joints_map: dict[str, tuple[str, Any]] = {}  # child_name -> (parent_name, joint_obj)
    root_link: tuple[str, Any] | None = None

    # invalid scene/nothing to be checked
    if not scene or not hasattr(scene, "objects"):
        return RobotSceneStatistics(
            num_links=0,
            total_mass=0.0,
            total_dof=0,
            link_objects={},
            joint_objects=[],
            sensor_objects=[],
            transmission_objects=[],
            root_link=None,
        )
    # Map joint types to DOF contribution
    dof_map = {
        "FIXED": 0,
        "REVOLUTE": 1,
        "CONTINUOUS": 1,
        "PRISMATIC": 1,
        "PLANAR": 2,
        "FLOATING": 6,
    }

    for obj in scene.objects:
        if is_robot_link(obj):
            link_name = obj.linkforge.link_name if obj.linkforge.link_name else obj.name
            link_objects[link_name] = obj

            # NOTE: invalid mass values (<= 0) are ignored
            if obj.linkforge.mass > 0:
                total_mass += obj.linkforge.mass

        elif is_robot_joint(obj):
            joint_objects.append(obj)

            joint_type = obj.linkforge_joint.joint_type
            total_dof += dof_map.get(joint_type, 0)

            child = obj.linkforge_joint.child_link
            parent = obj.linkforge_joint.parent_link

            if child and hasattr(child, "linkforge"):
                child_name = child.linkforge.link_name if child.linkforge.link_name else child.name
                parent_name = ""
                if parent and hasattr(parent, "linkforge"):
                    parent_name = (
                        parent.linkforge.link_name if parent.linkforge.link_name else parent.name
                    )

                if child_name and parent_name:
                    joints_map[child_name] = (parent_name, obj)

        elif is_robot_sensor(obj):
            sensor_objects.append(obj)

        elif is_robot_transmission(obj):
            transmission_objects.append(obj)

    # get root link (link that is not a child in any joint)
    for link_name, obj in link_objects.items():
        if link_name not in joints_map:
            root_link = (link_name, obj)
            break

    return RobotSceneStatistics(
        num_links=len(link_objects),
        total_mass=total_mass,
        total_dof=total_dof,
        link_objects=link_objects,
        joint_objects=joint_objects,
        sensor_objects=sensor_objects,
        transmission_objects=transmission_objects,
        root_link=root_link,
        joints_map=joints_map,
    )


def build_tree_from_stats(
    stats: RobotSceneStatistics,
) -> tuple[
    dict[str, list[tuple[str, str, str]]], str | None, dict[tuple[str, str], Any], dict[str, Any]
]:
    """Build kinematic tree from precomputed RobotSceneStatistics.

    Returns: (tree, root_link_name, joints_dict, links_dict)
    - tree: parent -> list of (child_name, joint_name, joint_type)
    - root_link_name: name of root link or None
    - joints_dict: mapping (parent, child) -> joint object
    - links_dict: mapping link_name -> link object
    """
    links = stats.link_objects
    joints_map = stats.joints_map

    tree: dict[str, list[tuple[str, str, str]]] = {link_name: [] for link_name in links}
    joints: dict[tuple[str, str], Any] = {}

    for child_name, (parent_name, joint_obj) in joints_map.items():
        if parent_name in tree:
            props = typing.cast(typing.Any, joint_obj).linkforge_joint
            tree[parent_name].append((child_name, props.joint_name, props.joint_type))
            joints[(parent_name, child_name)] = joint_obj

    # find root
    all_children = set(joints_map.keys())
    root_link = None
    for link_name in links:
        if link_name not in all_children:
            root_link = link_name
            break

    return tree, root_link, joints, links


def move_to_collection(obj: bpy.types.Object, collection: bpy.types.Collection) -> None:
    """Safely move an object to a specific collection.

    This unlinks the object from all existing collections and links it to
     the target collection.

    Args:
        obj: Blender object to move
        collection: Target Blender collection
    """
    if not obj or not collection:
        return

    # Unlink from all current collections
    for coll in list(obj.users_collection):
        if coll != collection:
            coll.objects.unlink(obj)

    # Link to target if not already there
    if obj.name not in collection.objects:
        # Object might be already linked but not showing in collection.objects lookup yet
        with contextlib.suppress(RuntimeError):
            collection.objects.link(obj)
