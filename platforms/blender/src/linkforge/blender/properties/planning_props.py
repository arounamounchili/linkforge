"""Blender Property Groups for centralized SRDF (MoveIt 2 planning) configuration.

These properties are stored on the Scene and define the semantic planning
description (Virtual Joint, Planning Groups, End Effectors, Group States,
and Disabled Collisions) for MoveIt 2 and ROS 2 motion planning.
"""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.props import (
    CollectionProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import Context, PropertyGroup

from ..core.constants import (
    JOINT_FIXED,
    JOINT_PRISMATIC,
    JOINT_REVOLUTE,
    PI,
    SRDF_REASON_ADJACENT,
    SRDF_REASON_ALWAYS,
    SRDF_REASON_DEFAULT,
    SRDF_REASON_NEVER,
    SRDF_REASON_USER,
)
from ..utils.property_helpers import get_joint_props, get_link_props, get_robot_props


def _resolve_to_link_obj(obj: bpy.types.Object | None) -> bpy.types.Object | None:
    """Return the robot link object, or its parent link if obj is a visual/collision child."""
    if obj is None:
        return None
    if (lp := get_link_props(obj)) and lp.is_robot_link:
        return obj
    parent = getattr(obj, "parent", None)
    if parent is not None and (lp_p := get_link_props(parent)) and lp_p.is_robot_link:
        return typing.cast(bpy.types.Object, parent)
    return None


def poll_planning_link(_self: typing.Any, obj: bpy.types.Object) -> bool:
    """Filter to only allow robot link objects in pointer selection."""
    return bool((props := get_link_props(obj)) and props.is_robot_link)


def poll_planning_joint(_self: typing.Any, obj: bpy.types.Object) -> bool:
    """Filter to only allow robot joint objects in pointer selection."""
    return bool((props := get_joint_props(obj)) and props.is_robot_joint)


def get_planning_link_name(obj: bpy.types.Object | None, fallback: str = "") -> str:
    """Resolve the logical robot link name from a Blender object pointer or fallback."""
    link_obj = _resolve_to_link_obj(obj) or obj
    if link_obj is not None:
        lp = get_link_props(link_obj)
        return str(lp.link_name) if lp and lp.link_name else str(link_obj.name)
    return fallback


def get_planning_joint_name(obj: bpy.types.Object | None, fallback: str = "") -> str:
    """Resolve the logical robot joint name from a Blender object pointer or fallback."""
    if obj is not None:
        jp = get_joint_props(obj)
        return str(jp.joint_name) if jp and jp.joint_name else str(obj.name)
    return fallback


def has_planning_data(props: typing.Any) -> bool:
    """Check whether any SRDF semantic planning data is configured on the scene."""
    if not props:
        return False
    vj_obj = getattr(props, "virtual_joint_child_link", None)
    vj_name = getattr(props, "virtual_joint_child_link_name", "")
    has_vj_target = (vj_obj is not None and isinstance(getattr(vj_obj, "name", None), str)) or (
        isinstance(vj_name, str) and bool(vj_name.strip())
    )
    has_vj = bool(getattr(props, "use_virtual_joint", False) is True and has_vj_target)
    return bool(
        has_vj
        or len(getattr(props, "planning_groups", ())) > 0
        or len(getattr(props, "end_effectors", ())) > 0
        or len(getattr(props, "group_states", ())) > 0
        or len(getattr(props, "disabled_collisions", ())) > 0
    )


def resolve_planning_group_joints(
    scene: bpy.types.Scene | None,
    group: PlanningGroupProperty,
) -> list[tuple[str, bpy.types.Object | None]]:
    """Resolve active (non-fixed, non-mimic) joints belonging to a planning group.

    For ``CHAIN`` groups, walks the scene's kinematic tree from ``tip_link`` up to
    ``base_link`` and returns the actuated joints in ``base_link -> tip_link`` order.
    For ``JOINTS`` groups, returns the explicitly assigned joints in order.
    """
    if group.group_type == "JOINTS":
        resolved: list[tuple[str, bpy.types.Object | None]] = []
        seen: set[str] = set()
        for item in group.joints:
            j_obj = item.joint_obj
            if j_obj is None and scene is not None and item.name:
                for obj in scene.objects:
                    jp = get_joint_props(obj)
                    if (
                        jp
                        and jp.is_robot_joint
                        and (jp.joint_name == item.name or obj.name == item.name)
                    ):
                        j_obj = obj
                        break
            j_name = get_planning_joint_name(j_obj, fallback=item.name)
            if not j_name or j_name in seen:
                continue
            if (
                j_obj is not None
                and (jp := get_joint_props(j_obj))
                and (jp.joint_type.lower() == JOINT_FIXED or jp.use_mimic)
            ):
                continue
            seen.add(j_name)
            resolved.append((j_name, j_obj))
        return resolved

    # CHAIN mode: walk from tip_link up to base_link
    if scene is None:
        return []

    base_name = get_planning_link_name(group.base_link, fallback=group.base_link_name)
    tip_name = get_planning_link_name(group.tip_link, fallback=group.tip_link_name)
    if not base_name or not tip_name or base_name == tip_name:
        return []

    # Map child_link_name -> (parent_link_name, joint_obj, joint_props)
    child_to_joint: dict[str, tuple[str, bpy.types.Object, typing.Any]] = {}
    for obj in scene.objects:
        jp = get_joint_props(obj)
        if not jp or not jp.is_robot_joint:
            continue
        if not jp.parent_link or not jp.child_link:
            continue
        p_name = get_planning_link_name(jp.parent_link)
        c_name = get_planning_link_name(jp.child_link)
        child_to_joint[c_name] = (p_name, obj, jp)

    chain_joints: list[tuple[str, bpy.types.Object | None]] = []
    visited: set[str] = set()
    current = tip_name
    found = False

    while True:
        if current == base_name:
            found = True
            break
        if current in visited:
            break
        visited.add(current)
        entry = child_to_joint.get(current)
        if entry is None:
            break
        parent_name, j_obj, jp = entry
        if jp.joint_type.lower() != JOINT_FIXED and not jp.use_mimic:
            j_name = get_planning_joint_name(j_obj)
            chain_joints.append((j_name, j_obj))
        current = parent_name

    if not found:
        return []

    chain_joints.reverse()
    return chain_joints


def update_group_state_joint_value(
    self: GroupStateJointValueProperty,
    _context: Context | None,
) -> None:
    """Clamp joint state value to the joint's kinematic limits when applicable."""
    j_obj = self.joint_obj
    if j_obj is None:
        return
    jp = get_joint_props(j_obj)
    if not jp or not jp.is_robot_joint:
        return

    j_type = jp.joint_type.lower()
    if j_type in (JOINT_REVOLUTE, JOINT_PRISMATIC) or jp.use_limits:
        lower = float(jp.limit_lower)
        upper = float(jp.limit_upper)
        if lower <= upper:
            if self.value < lower:
                self.value = lower
            elif self.value > upper:
                self.value = upper


def sync_group_state_joints(
    state: GroupStateProperty,
    scene: bpy.types.Scene | None,
) -> int:
    """Synchronize a GroupStateProperty's joint_values collection with its PlanningGroup.

    Preserves existing joint values for joints that remain in the group and
    initializes newly added joints to 0.0 (clamped to joint limits).
    """
    if scene is None:
        return 0
    robot_props = get_robot_props(scene)
    if not robot_props or not state.group:
        return 0

    target_group: PlanningGroupProperty | None = None
    for grp in robot_props.planning_groups:
        if grp.name == state.group:
            target_group = grp
            break

    if target_group is None:
        return 0

    resolved_joints = resolve_planning_group_joints(scene, target_group)
    existing_values: dict[str, float] = {
        (jv.joint_name or jv.name): float(jv.value) for jv in state.joint_values
    }

    state.joint_values.clear()
    for j_name, j_obj in resolved_joints:
        item = state.joint_values.add()
        item.name = j_name
        item.joint_name = j_name
        item.joint_obj = j_obj
        item.value = existing_values.get(j_name, 0.0)

    return len(resolved_joints)


def update_group_state_group(
    self: GroupStateProperty,
    context: Context | None,
) -> None:
    """Automatically sync active joints when a GroupState's target group changes."""
    scene = getattr(context, "scene", None) if context else getattr(bpy.context, "scene", None)
    if scene is not None and self.group:
        sync_group_state_joints(self, scene)


def _safe_redirect_pointer(
    prop_owner: typing.Any,
    attr: str,
    target: bpy.types.Object | None,
) -> None:
    """Safely update a PointerProperty, deferring via timer in GUI mode to avoid RNA re-entrancy."""
    if not getattr(bpy.app, "background", False) and hasattr(bpy.app, "timers"):

        def _deferred() -> None:
            with contextlib.suppress(ReferenceError, Exception):
                setattr(prop_owner, attr, target)
            return None

        bpy.app.timers.register(_deferred, first_interval=0.0)
    else:
        setattr(prop_owner, attr, target)


def update_virtual_joint_child_link(
    self: typing.Any,
    _context: Context | None,
) -> None:
    """Redirect visual/collision child meshes to their parent robot link."""
    cur = getattr(self, "virtual_joint_child_link", None)
    resolved = _resolve_to_link_obj(cur)
    if resolved is not None and resolved != cur:
        _safe_redirect_pointer(self, "virtual_joint_child_link", resolved)


def update_planning_group_topology(
    self: PlanningGroupProperty,
    context: Context | None,
) -> None:
    """Redirect visual/collision child meshes and sync GroupStates referencing this group."""
    for attr in ("base_link", "tip_link"):
        cur = getattr(self, attr, None)
        resolved = _resolve_to_link_obj(cur)
        if resolved is not None and resolved != cur:
            _safe_redirect_pointer(self, attr, resolved)
            return

    scene = getattr(context, "scene", None) if context else getattr(bpy.context, "scene", None)
    if scene is not None and (props := get_robot_props(scene)):
        for state in props.group_states:
            if state.group == self.name:
                sync_group_state_joints(state, scene)


def update_end_effector_link(
    self: EndEffectorProperty,
    _context: Context | None,
) -> None:
    """Redirect visual/collision child meshes to their parent robot link."""
    resolved = _resolve_to_link_obj(self.parent_link)
    if resolved is not None and resolved != self.parent_link:
        _safe_redirect_pointer(self, "parent_link", resolved)


def update_collision_pair_links(
    self: CollisionPairProperty,
    _context: Context | None,
) -> None:
    """Redirect visual/collision child meshes to their parent robot link."""
    for attr in ("link1", "link2"):
        cur = getattr(self, attr, None)
        resolved = _resolve_to_link_obj(cur)
        if resolved is not None and resolved != cur:
            _safe_redirect_pointer(self, attr, resolved)
            return


def update_planning_group_name(
    self: PlanningGroupProperty,
    context: Context | None,
) -> None:
    """Propagate planning group renames to End Effectors and Group States."""
    old_name = getattr(self, "previous_name", "")
    new_name = self.name
    if old_name and old_name != new_name:
        scene = getattr(context, "scene", None) if context else getattr(bpy.context, "scene", None)
        if scene is not None and (props := get_robot_props(scene)):
            for ee in props.end_effectors:
                if ee.group == old_name:
                    ee.group = new_name
                if ee.parent_group == old_name:
                    ee.parent_group = new_name
            for state in props.group_states:
                if state.group == old_name:
                    state.group = new_name
    self.previous_name = new_name


class PlanningGroupJointProperty(PropertyGroup):
    """Property group for a joint member of a JOINTS-mode Planning Group."""

    name: StringProperty(  # type: ignore
        name="Joint Name",
        description="Name of the joint in the kinematic tree",
        default="",
    )

    joint_obj: PointerProperty(  # type: ignore
        name="Joint Object",
        type=bpy.types.Object,
        poll=poll_planning_joint,
        description="Reference to the physical joint object",
    )


class PlanningGroupProperty(PropertyGroup):
    """Property group for a MoveIt 2 SRDF Planning Group."""

    name: StringProperty(  # type: ignore
        name="Name",
        description="Unique name for the planning group (e.g., arm, hand)",
        default="group",
        update=update_planning_group_name,
    )

    previous_name: StringProperty(  # type: ignore
        name="Previous Name",
        description="Internal tracker used to propagate group renames to dependent SRDF items",
        default="",
    )

    group_type: EnumProperty(  # type: ignore
        name="Mode",
        description="How the planning group members are defined",
        items=[
            (
                "CHAIN",
                "Kinematic Chain",
                "Serial kinematic chain from Base Link to Tip Link (recommended for arms)",
            ),
            (
                "JOINTS",
                "Joint Collection",
                "Explicit collection of joints (recommended for multi-finger grippers and hands)",
            ),
        ],
        default="CHAIN",
        update=update_planning_group_topology,
    )

    base_link: PointerProperty(  # type: ignore
        name="Base Link",
        description="Start link of the kinematic chain (e.g., base_link or shoulder_link)",
        type=bpy.types.Object,
        poll=poll_planning_link,
        update=update_planning_group_topology,
    )

    base_link_name: StringProperty(  # type: ignore
        name="Base Link Name",
        description="Fallback base link name when object pointer is unresolved",
        default="",
    )

    tip_link: PointerProperty(  # type: ignore
        name="Tip Link",
        description="End link of the kinematic chain (e.g., wrist_link or tool0)",
        type=bpy.types.Object,
        poll=poll_planning_link,
        update=update_planning_group_topology,
    )

    tip_link_name: StringProperty(  # type: ignore
        name="Tip Link Name",
        description="Fallback tip link name when object pointer is unresolved",
        default="",
    )

    joints: CollectionProperty(type=PlanningGroupJointProperty)  # type: ignore
    active_joint_index: IntProperty(default=0)  # type: ignore

    # Preserved SRDF group members for lossless round-trip import
    links_csv: StringProperty(  # type: ignore
        name="Explicit Links",
        description="Comma-separated explicit link names preserved from SRDF import",
        default="",
    )

    subgroups_csv: StringProperty(  # type: ignore
        name="Subgroups",
        description="Comma-separated subgroup names preserved from SRDF import",
        default="",
    )

    extra_chains_csv: StringProperty(  # type: ignore
        name="Extra Chains",
        description="Additional base:tip pairs preserved from SRDF import",
        default="",
    )


class EndEffectorProperty(PropertyGroup):
    """Property group for a MoveIt 2 SRDF End Effector."""

    name: StringProperty(  # type: ignore
        name="Name",
        description="Unique name for the end effector (e.g., gripper)",
        default="end_effector",
    )

    group: StringProperty(  # type: ignore
        name="Group",
        description="Planning group that forms the end effector (e.g., hand)",
        default="",
    )

    parent_link: PointerProperty(  # type: ignore
        name="Parent Link",
        description="Robot link the end effector attaches to (e.g., wrist_link)",
        type=bpy.types.Object,
        poll=poll_planning_link,
        update=update_end_effector_link,
    )

    parent_link_name: StringProperty(  # type: ignore
        name="Parent Link Name",
        description="Fallback parent link name when object pointer is unresolved",
        default="",
    )

    parent_group: StringProperty(  # type: ignore
        name="Parent Group",
        description="Optional planning group that carries the end effector (e.g., arm)",
        default="",
    )


class GroupStateJointValueProperty(PropertyGroup):
    """Property group for a single joint's target position inside a Group State."""

    name: StringProperty(  # type: ignore
        name="Joint Name",
        default="",
    )

    joint_name: StringProperty(  # type: ignore
        name="Joint Name",
        description="Name of the joint in the planning group",
        default="",
    )

    joint_obj: PointerProperty(  # type: ignore
        name="Joint Object",
        type=bpy.types.Object,
        poll=poll_planning_joint,
        description="Reference to the physical joint object",
    )

    value: FloatProperty(  # type: ignore
        name="Position",
        description="Target joint position for this named pose (clamped to joint limits)",
        default=0.0,
        soft_min=-PI,
        soft_max=PI,
        precision=3,
        update=update_group_state_joint_value,
    )


class GroupStateProperty(PropertyGroup):
    """Property group for a MoveIt 2 SRDF named Group State (pose)."""

    name: StringProperty(  # type: ignore
        name="Name",
        description="Unique pose name (e.g., home, ready, folded)",
        default="pose",
    )

    group: StringProperty(  # type: ignore
        name="Group",
        description="Planning group this pose belongs to",
        default="",
        update=update_group_state_group,
    )

    joint_values: CollectionProperty(type=GroupStateJointValueProperty)  # type: ignore


class CollisionPairProperty(PropertyGroup):
    """Property group for a MoveIt 2 SRDF Disabled Collision pair."""

    name: StringProperty(  # type: ignore
        name="Collision Pair",
        default="",
    )

    link1: PointerProperty(  # type: ignore
        name="Link 1",
        description="First link in the disabled collision pair",
        type=bpy.types.Object,
        poll=poll_planning_link,
        update=update_collision_pair_links,
    )

    link1_name: StringProperty(  # type: ignore
        name="Link 1 Name",
        description="Fallback first link name when object pointer is unresolved",
        default="",
    )

    link2: PointerProperty(  # type: ignore
        name="Link 2",
        description="Second link in the disabled collision pair",
        type=bpy.types.Object,
        poll=poll_planning_link,
        update=update_collision_pair_links,
    )

    link2_name: StringProperty(  # type: ignore
        name="Link 2 Name",
        description="Fallback second link name when object pointer is unresolved",
        default="",
    )

    reason: EnumProperty(  # type: ignore
        name="Reason",
        description="Reason why collision checking is disabled between these links",
        items=[
            (
                SRDF_REASON_ADJACENT,
                "Adjacent",
                "Links are directly connected by a joint in the kinematic tree",
            ),
            (
                SRDF_REASON_NEVER,
                "Never",
                "Links can never collide within their kinematic joint limits",
            ),
            (
                SRDF_REASON_DEFAULT,
                "Default",
                "Links are in collision in the default robot configuration",
            ),
            (
                SRDF_REASON_ALWAYS,
                "Always",
                "Links are always in collision",
            ),
            (
                SRDF_REASON_USER,
                "User",
                "Collision checking manually disabled by the user",
            ),
        ],
        default=SRDF_REASON_ADJACENT,
    )


_CLASSES = [
    PlanningGroupJointProperty,
    PlanningGroupProperty,
    EndEffectorProperty,
    GroupStateJointValueProperty,
    GroupStateProperty,
    CollisionPairProperty,
]


def register() -> None:
    """Register SRDF planning property groups."""
    for cls in _CLASSES:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister SRDF planning property groups."""
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
