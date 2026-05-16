"""General Blender utility functions for object and collection management."""

from __future__ import annotations

import contextlib
import os
import typing
from dataclasses import dataclass, field
from typing import Any

import bpy
from linkforge.core.constants import (
    GEOM_BOX,
    GEOM_CYLINDER,
    GEOM_MESH,
    GEOM_SPHERE,
    JOINT_CONTINUOUS,
    JOINT_FIXED,
    JOINT_FLOATING,
    JOINT_PLANAR,
    JOINT_PRISMATIC,
    JOINT_REVOLUTE,
)

from ..adapters.blender_to_core import detect_primitive_type
from ..constants import (
    GEOM_AUTO,
    SUFFIX_COLLISION,
    TAG_COLLISION_GEOM,
    TAG_SOURCE_GEOM,
)
from ..utils.property_helpers import (
    get_joint_props,
    get_link_props,
    get_sensor_props,
    get_transmission_props,
)

if typing.TYPE_CHECKING:
    pass


def is_robot_link(obj: Any) -> bool:
    """Check if blender obj is a robot_link.

    Args:
        obj: Blender object to be checked

    Returns:
        True if object has linkforge properties and is marked as robot_link
    """
    props = get_link_props(obj)
    return bool(props and props.is_robot_link)


def is_robot_joint(obj: Any) -> bool:
    """Check if blender obj is a robot_joint.

    Args:
        obj: Blender object to be checked

    Returns:
        True if object is EMPTY with linkforge_joint properties marked as robot_joint
    """
    return (
        getattr(obj, "type", None) == "EMPTY"
        and (props := get_joint_props(obj)) is not None
        and props.is_robot_joint
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
        and (props := get_sensor_props(obj)) is not None
        and props.is_robot_sensor
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
        and (props := get_transmission_props(obj)) is not None
        and props.is_robot_transmission
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
    # Map from link name -> (collision_obj, detected_type, is_primitive)
    geometry_stats: dict[str, tuple[Any, str, bool]] = field(default_factory=dict)
    # List of objects that need manual inertia gizmos
    manual_inertia_objects: list[Any] = field(default_factory=list)


# Internal cache for scene statistics to avoid redundant scans within the same frame
_stats_cache: dict[tuple[int, int, int], RobotSceneStatistics] = {}


def clear_stats_cache(_self: Any = None, _context: Any = None) -> None:
    """Clear the global scene statistics cache."""
    _stats_cache.clear()


# Map joint types to DOF contribution
JOINT_DOF_MAP = {
    JOINT_FIXED: 0,
    JOINT_REVOLUTE: 1,
    JOINT_CONTINUOUS: 1,
    JOINT_PRISMATIC: 1,
    JOINT_PLANAR: 2,
    JOINT_FLOATING: 6,
}


def get_robot_statistics(scene: Any, force_refresh: bool = False) -> RobotSceneStatistics:
    """Analyze scene and setup robot statistics/properties.

    Categorizes all robot components (links, joints, sensors, transmissions)
    and calculates total link mass and DOFs from all joints found.
    Uses frame-level caching to ensure the scene is scanned only once per frame.

    Args:
        scene: Blender scene to be analyzed
        force_refresh: If True, ignore cache and perform a full scan.

    Returns:
        RobotSceneStatistics with all pre-calculated properties.
    """
    link_objects: dict[str, Any] = {}
    joint_objects: list[Any] = []
    sensor_objects: list[Any] = []
    transmission_objects: list[Any] = []

    if scene:
        disable_cache = os.environ.get("LINKFORGE_DISABLE_CACHE", "0") == "1"

        objects = getattr(scene, "objects", [])
        try:
            obj_count = len(objects)
        except (TypeError, AttributeError):
            obj_count = 0

        cache_key = (
            id(scene),
            getattr(scene, "frame_current", 0),
            obj_count,
        )

        if not disable_cache and not force_refresh and cache_key in _stats_cache:
            cached_stats = _stats_cache[cache_key]
            # Defensive check: if an operator deleted an object in the same frame,
            # accessing it will raise a ReferenceError. We catch this and invalidate the cache.
            try:
                # 1. Validate link objects
                for link_obj in cached_stats.link_objects.values():
                    _ = link_obj.name
                # 2. Validate joint objects
                for joint_obj in cached_stats.joint_objects:
                    _ = joint_obj.name
                # 3. Validate sensor objects
                for sensor_obj in cached_stats.sensor_objects:
                    _ = sensor_obj.name
                # 4. Validate transmission objects
                for trans_obj in cached_stats.transmission_objects:
                    _ = trans_obj.name
                # 5. Validate geometry objects
                for geo_info in cached_stats.geometry_stats.values():
                    _ = geo_info[0].name
                # 6. Validate manual inertia objects
                for manual_obj in cached_stats.manual_inertia_objects:
                    _ = manual_obj.name

                return cached_stats
            except (ReferenceError, KeyError, AttributeError):
                del _stats_cache[cache_key]

    total_mass = 0.0
    total_dof = 0
    joints_map: dict[str, tuple[str, Any]] = {}
    geometry_stats: dict[str, tuple[Any, str, bool]] = {}
    manual_inertia_objects: list[Any] = []
    root_link: tuple[str, Any] | None = None

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
            joints_map={},
            geometry_stats={},
            manual_inertia_objects=[],
        )

    for obj in scene.objects:
        if (lp := get_link_props(obj)) and lp.is_robot_link:
            link_name = lp.link_name if lp.link_name else obj.name
            link_objects[link_name] = obj

            # NOTE: invalid mass values (<= 0) are ignored
            if lp.mass > 0:
                total_mass += lp.mass

            # Track manual inertia gizmos
            if not lp.use_auto_inertia:
                manual_inertia_objects.append(obj)

            # Heuristic Geometry Detection (moved from link_panel.py)
            collision_obj = next(
                (c for c in obj.children if SUFFIX_COLLISION in c.name.lower()), None
            )
            if collision_obj:
                detected_type = GEOM_MESH
                is_primitive = False

                # 1. Check explicit URDF tag
                if collision_obj.get(TAG_SOURCE_GEOM):
                    tag_val = str(collision_obj[TAG_SOURCE_GEOM]).lower()
                    detected_type = tag_val
                    is_primitive = tag_val in (GEOM_BOX, GEOM_CYLINDER, GEOM_SPHERE)
                # 2. Check generator tag
                else:
                    stored_type = collision_obj.get(TAG_COLLISION_GEOM, GEOM_AUTO)
                    if isinstance(stored_type, str):
                        stored_type = stored_type.lower()

                    if stored_type in (GEOM_BOX, GEOM_CYLINDER, GEOM_SPHERE):
                        detected_type = stored_type
                        is_primitive = True
                    elif stored_type == GEOM_MESH:
                        detected_type = GEOM_MESH
                    # 3. Fallback to heuristic
                    else:
                        try:
                            heuristic_type = detect_primitive_type(collision_obj)
                            if heuristic_type:
                                detected_type = heuristic_type
                                is_primitive = True
                        except Exception:
                            pass

                geometry_stats[link_name] = (collision_obj, detected_type, is_primitive)

        if is_robot_joint(obj):
            joint_objects.append(obj)

            if jp := get_joint_props(obj):
                joint_type = jp.joint_type
                total_dof += JOINT_DOF_MAP.get(joint_type, 0)

                child = jp.child_link
                parent = jp.parent_link

                if child and (props_child := get_link_props(child)):
                    child_name = props_child.link_name if props_child.link_name else child.name
                    parent_name = ""
                    if parent and (props_parent := get_link_props(parent)):
                        parent_name = (
                            props_parent.link_name if props_parent.link_name else parent.name
                        )

                    if child_name and parent_name:
                        joints_map[child_name] = (parent_name, obj)

        if is_robot_sensor(obj):
            sensor_objects.append(obj)

        if is_robot_transmission(obj):
            transmission_objects.append(obj)

    # get root link (link that is not a child in any joint)
    for link_name, obj in link_objects.items():
        if link_name not in joints_map:
            root_link = (link_name, obj)
            break

    stats = RobotSceneStatistics(
        num_links=len(link_objects),
        total_mass=total_mass,
        total_dof=total_dof,
        link_objects=link_objects,
        joint_objects=joint_objects,
        sensor_objects=sensor_objects,
        transmission_objects=transmission_objects,
        root_link=root_link,
        joints_map=joints_map,
        geometry_stats=geometry_stats,
        manual_inertia_objects=manual_inertia_objects,
    )

    # Update cache
    cache_key = (id(scene), getattr(scene, "frame_current", 0), len(getattr(scene, "objects", [])))
    _stats_cache[cache_key] = stats

    return stats


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
            try:
                if props := get_joint_props(joint_obj):
                    tree[parent_name].append((child_name, props.joint_name, props.joint_type))
                    joints[(parent_name, child_name)] = joint_obj
            except (ReferenceError, AttributeError):
                continue

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


def sync_object_collections(target_obj: bpy.types.Object, source_obj: bpy.types.Object) -> None:
    """Synchronize a target object's collection membership with a source object.

    This ensures that secondary components (collisions, sensors, etc.) always stay
    in the same Outliner collections as their parent Link or Robot frame,
    preventing "leaks" to the scene root.

    Args:
        target_obj: The object to be moved/linked.
        source_obj: The reference object whose collections should be matched.
    """
    if not target_obj or not source_obj:
        return

    # 1. Link to all collections where source_obj resides
    source_cols = list(source_obj.users_collection)
    if not source_cols:
        return

    for col in source_cols:
        if target_obj.name not in col.objects:
            col.objects.link(target_obj)

    # 2. Unlink from any collections that source_obj is NOT in
    # This cleans up the default Scene Collection link if it was created there
    for col in list(target_obj.users_collection):
        if col not in source_cols:
            col.objects.unlink(target_obj)
