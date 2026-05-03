"""Semantic robot description models (SRDF).

This module provides data structures to represent MoveIt-style semantic information,
such as planning groups, poses, and collision filters.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from typing import Any

from ..exceptions import RobotValidationError, ValidationErrorCode


@dataclass(frozen=True)
class VirtualJoint:
    """Connects the robot to a fixed frame in the world.

    Attributes:
        name: Unique name for the virtual joint.
        type: Type of joint (e.g., 'fixed', 'planar', 'floating').
        parent_frame: Name of the parent coordinate frame (e.g., 'world').
        child_link: Name of the robot link attached to this joint.
    """

    name: str
    type: str
    parent_frame: str
    child_link: str

    def __post_init__(self) -> None:
        """Validate virtual joint."""
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY, "Virtual joint name cannot be empty"
            )
        if self.type not in ("fixed", "planar", "floating"):
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Invalid virtual joint type '{self.type}' (must be fixed, planar, or floating)",
                target="VirtualJointType",
                value=self.type,
            )

    def with_prefix(self, prefix: str) -> VirtualJoint:
        """Create a new virtual joint with prefixed name and child_link."""
        return replace(
            self,
            name=f"{prefix}{self.name}",
            child_link=f"{prefix}{self.child_link}",
        )


@dataclass(frozen=True)
class GroupState:
    """A named set of joint values for a planning group (a pose).

    Attributes:
        name: Unique name for this pose (e.g., 'home', 'folded').
        group: Name of the planning group this state applies to.
        joint_values: Dictionary mapping joint names to their target values.
            A joint can have multiple values (e.g., planar or floating joints).
    """

    name: str
    group: str
    joint_values: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate and normalize group state."""
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY, "Group state name cannot be empty"
            )
        if not self.group:
            raise RobotValidationError(ValidationErrorCode.NAME_EMPTY, "Group name cannot be empty")

        # Normalize and isolate joint values (ensure tuples)
        normalized = {}
        for k, v in self.joint_values.items():
            if isinstance(v, (list, set, tuple)):
                normalized[k] = tuple(v)
            elif isinstance(v, (int, float)):
                normalized[k] = (float(v),)
            else:
                normalized[k] = (v,)
        object.__setattr__(self, "joint_values", normalized)

    def with_prefix(self, prefix: str) -> GroupState:
        """Create a new group state with prefixed name, group, and joint names."""
        return replace(
            self,
            name=f"{prefix}{self.name}",
            group=f"{prefix}{self.group}",
            joint_values={f"{prefix}{k}": v for k, v in self.joint_values.items()},
        )


@dataclass(frozen=True)
class EndEffector:
    """Defines a planning group as an end effector.

    Attributes:
        name: Unique name for the end effector.
        group: The planning group that forms the end effector (e.g., 'hand').
        parent_link: The robot link the end effector is attached to.
        parent_group: Optional name of the group this end-effector belongs to.
    """

    name: str
    group: str
    parent_link: str
    parent_group: str | None = None

    def __post_init__(self) -> None:
        """Validate end effector."""
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY, "End effector name cannot be empty"
            )
        if not self.group:
            raise RobotValidationError(ValidationErrorCode.NAME_EMPTY, "Group name cannot be empty")

    def with_prefix(self, prefix: str) -> EndEffector:
        """Create a new end effector with prefixed name, group, and links."""
        return replace(
            self,
            name=f"{prefix}{self.name}",
            group=f"{prefix}{self.group}",
            parent_link=f"{prefix}{self.parent_link}",
            parent_group=f"{prefix}{self.parent_group}" if self.parent_group else None,
        )


@dataclass(frozen=True)
class PassiveJoint:
    """A joint that is not actuated but exists in the kinematic chain.

    Attributes:
        name: Name of the passive joint.
    """

    name: str

    def __post_init__(self) -> None:
        """Validate passive joint."""
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY, "Passive joint name cannot be empty"
            )

    def with_prefix(self, prefix: str) -> PassiveJoint:
        """Create a new passive joint with a prefixed name."""
        return replace(self, name=f"{prefix}{self.name}")


@dataclass(frozen=True)
class CollisionPair:
    """Represents a collision rule between two specific links.

    Can be used for both disabled and enabled collisions.

    Attributes:
        link1: Name of the first link.
        link2: Name of the second link.
        reason: Optional human-readable reason (e.g., 'Adjacent', 'Never').
    """

    link1: str
    link2: str
    reason: str | None = None

    def __post_init__(self) -> None:
        """Validate collision pair."""
        if not self.link1 or not self.link2:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY, "Collision link names cannot be empty"
            )
        if self.link1 == self.link2:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Cannot specify collisions for a link with itself ('{self.link1}')",
                target="CollisionPair",
            )

    def with_prefix(self, prefix: str) -> CollisionPair:
        """Create a new collision pair with prefixed link names."""
        return replace(
            self,
            link1=f"{prefix}{self.link1}",
            link2=f"{prefix}{self.link2}",
        )


@dataclass(frozen=True)
class Chain:
    """A kinematic chain defined by a base link and a tip link.

    Attributes:
        base_link: Name of the base link.
        tip_link: Name of the tip link.
    """

    base_link: str
    tip_link: str

    def __post_init__(self) -> None:
        """Validate chain."""
        if not self.base_link or not self.tip_link:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY, "Chain base and tip link names cannot be empty"
            )

    def with_prefix(self, prefix: str) -> Chain:
        """Create a new chain with prefixed base and tip links."""
        return replace(
            self,
            base_link=f"{prefix}{self.base_link}",
            tip_link=f"{prefix}{self.tip_link}",
        )


@dataclass(frozen=True)
class PlanningGroup:
    """A named collection of links, joints, or chains used for motion planning.

    Attributes:
        name: Unique name for the planning group (e.g., 'arm', 'gripper').
        links: List of link names included in the group.
        joints: List of joint names included in the group.
        chains: List of chains defining kinematic structure.
        subgroups: List of other planning group names to include.
    """

    name: str
    links: Sequence[str] = field(default_factory=tuple)
    joints: Sequence[str] = field(default_factory=tuple)
    chains: Sequence[Chain] = field(default_factory=tuple)
    subgroups: Sequence[str] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        """Validate planning group."""
        # Convert to tuples if they are lists
        object.__setattr__(self, "links", tuple(self.links))
        object.__setattr__(self, "joints", tuple(self.joints))
        object.__setattr__(self, "chains", tuple(self.chains))
        object.__setattr__(self, "subgroups", tuple(self.subgroups))

        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY, "Planning group name cannot be empty"
            )
        if not any([self.links, self.joints, self.chains, self.subgroups]):
            raise RobotValidationError(
                ValidationErrorCode.VALUE_EMPTY,
                f"Planning group '{self.name}' must contain at least one link, joint, chain, or subgroup",
                target="PlanningGroup",
            )

    def with_prefix(self, prefix: str) -> PlanningGroup:
        """Create a new planning group with prefixed name and sub-elements."""
        return replace(
            self,
            name=f"{prefix}{self.name}",
            links=tuple(f"{prefix}{link}" for link in self.links),
            joints=tuple(f"{prefix}{joint}" for joint in self.joints),
            chains=tuple(c.with_prefix(prefix) for c in self.chains),
            subgroups=tuple(f"{prefix}{subgroup}" for subgroup in self.subgroups),
        )


@dataclass(frozen=True)
class SrdfSphere:
    """A collision sphere approximation.

    Attributes:
        center_x: Center X coordinate.
        center_y: Center Y coordinate.
        center_z: Center Z coordinate.
        radius: Radius of the sphere.
    """

    center_x: float
    center_y: float
    center_z: float
    radius: float

    def __post_init__(self) -> None:
        if self.radius < 0:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                "Sphere radius cannot be negative",
                target="SrdfSphere",
            )


@dataclass(frozen=True)
class LinkSphereApproximation:
    """Sphere-based collision geometry for a link.

    Attributes:
        link: Name of the link.
        spheres: List of spheres approximating the link's collision geometry.
    """

    link: str
    spheres: Sequence[SrdfSphere] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if not self.link:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY, "Link sphere approximation requires a link name"
            )
        object.__setattr__(self, "spheres", tuple(self.spheres))

    def with_prefix(self, prefix: str) -> LinkSphereApproximation:
        """Create a new approximation with a prefixed link name."""
        return replace(self, link=f"{prefix}{self.link}")


@dataclass(frozen=True)
class JointProperty:
    """Key-value metadata for a joint.

    Attributes:
        joint_name: Name of the joint.
        property_name: Name of the property.
        value: Value of the property.
    """

    joint_name: str
    property_name: str
    value: str

    def __post_init__(self) -> None:
        if not self.joint_name or not self.property_name or not self.value:
            raise RobotValidationError(
                ValidationErrorCode.VALUE_EMPTY,
                "Joint property must have a joint_name, property_name, and value",
            )

    def with_prefix(self, prefix: str) -> JointProperty:
        """Create a new joint property with a prefixed joint name."""
        return replace(self, joint_name=f"{prefix}{self.joint_name}")


@dataclass(frozen=True)
class SemanticRobotDescription:
    """Container for all semantic information (SRDF).

    This class serves as the central point for MoveIt-compatible metadata
    that exists alongside the kinematic URDF description.

    Attributes:
        robot_name: Name of the robot.
        virtual_joints: Virtual joints connecting the robot to the world.
        groups: Planning groups.
        group_states: Named joint configurations for groups.
        end_effectors: End effector definitions.
        passive_joints: Joints ignored by planning.
        disabled_collisions: Collision pairs to disable.
        enabled_collisions: Collision pairs to explicitly enable.
        no_default_collision_links: Links to disable all default collisions for.
        link_sphere_approximations: Sphere approximations for collision checking.
        joint_properties: Metadata properties for joints.
    """

    robot_name: str = ""
    virtual_joints: Sequence[VirtualJoint] = field(default_factory=tuple)
    groups: Sequence[PlanningGroup] = field(default_factory=tuple)
    group_states: Sequence[GroupState] = field(default_factory=tuple)
    end_effectors: Sequence[EndEffector] = field(default_factory=tuple)
    passive_joints: Sequence[PassiveJoint] = field(default_factory=tuple)
    disabled_collisions: Sequence[CollisionPair] = field(default_factory=tuple)
    enabled_collisions: Sequence[CollisionPair] = field(default_factory=tuple)
    no_default_collision_links: Sequence[str] = field(default_factory=tuple)
    link_sphere_approximations: Sequence[LinkSphereApproximation] = field(default_factory=tuple)
    joint_properties: Sequence[JointProperty] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        """Ensure all fields are tuples."""
        object.__setattr__(self, "virtual_joints", tuple(self.virtual_joints))
        object.__setattr__(self, "groups", tuple(self.groups))
        object.__setattr__(self, "group_states", tuple(self.group_states))
        object.__setattr__(self, "end_effectors", tuple(self.end_effectors))
        object.__setattr__(self, "passive_joints", tuple(self.passive_joints))
        object.__setattr__(self, "disabled_collisions", tuple(self.disabled_collisions))
        object.__setattr__(self, "enabled_collisions", tuple(self.enabled_collisions))
        object.__setattr__(
            self, "no_default_collision_links", tuple(self.no_default_collision_links)
        )
        object.__setattr__(
            self, "link_sphere_approximations", tuple(self.link_sphere_approximations)
        )
        object.__setattr__(self, "joint_properties", tuple(self.joint_properties))

    def with_prefix(self, prefix: str) -> SemanticRobotDescription:
        """Create a new description with prefixed name and all sub-elements."""
        return replace(
            self,
            robot_name=f"{prefix}{self.robot_name}" if self.robot_name else "",
            virtual_joints=tuple(vj.with_prefix(prefix) for vj in self.virtual_joints),
            groups=tuple(g.with_prefix(prefix) for g in self.groups),
            group_states=tuple(gs.with_prefix(prefix) for gs in self.group_states),
            end_effectors=tuple(ee.with_prefix(prefix) for ee in self.end_effectors),
            passive_joints=tuple(pj.with_prefix(prefix) for pj in self.passive_joints),
            disabled_collisions=tuple(dc.with_prefix(prefix) for dc in self.disabled_collisions),
            enabled_collisions=tuple(ec.with_prefix(prefix) for ec in self.enabled_collisions),
            no_default_collision_links=tuple(
                f"{prefix}{link}" for link in self.no_default_collision_links
            ),
            link_sphere_approximations=tuple(
                lsa.with_prefix(prefix) for lsa in self.link_sphere_approximations
            ),
            joint_properties=tuple(jp.with_prefix(prefix) for jp in self.joint_properties),
        )

    def merge_with(self, other: SemanticRobotDescription) -> SemanticRobotDescription:
        """Merge another semantic description into this one, deduplicating elements."""
        # Deduplicate groups
        new_groups = list(self.groups)
        current_group_names = {g.name for g in new_groups}
        for g in other.groups:
            if g.name not in current_group_names:
                new_groups.append(g)
                current_group_names.add(g.name)

        # Deduplicate virtual joints
        new_vjoints = list(self.virtual_joints)
        current_vj_names = {vj.name for vj in new_vjoints}
        for vj in other.virtual_joints:
            if vj.name not in current_vj_names:
                new_vjoints.append(vj)
                current_vj_names.add(vj.name)

        # Deduplicate passive joints
        new_passive = list(self.passive_joints)
        current_pj_names = {pj.name for pj in new_passive}
        for pj in other.passive_joints:
            if pj.name not in current_pj_names:
                new_passive.append(pj)
                current_pj_names.add(pj.name)

        # Deduplicate collision pairs
        new_disabled = list(self.disabled_collisions)
        current_disabled = {frozenset([dc.link1, dc.link2]) for dc in new_disabled}
        for dc in other.disabled_collisions:
            if frozenset([dc.link1, dc.link2]) not in current_disabled:
                new_disabled.append(dc)
                current_disabled.add(frozenset([dc.link1, dc.link2]))

        new_enabled = list(self.enabled_collisions)
        current_enabled = {frozenset([ec.link1, ec.link2]) for ec in new_enabled}
        for ec in other.enabled_collisions:
            if frozenset([ec.link1, ec.link2]) not in current_enabled:
                new_enabled.append(ec)
                current_enabled.add(frozenset([ec.link1, ec.link2]))

        # Deduplicate other elements
        new_ee = list(self.end_effectors)
        current_ee_names = {ee.name for ee in new_ee}
        for ee in other.end_effectors:
            if ee.name not in current_ee_names:
                new_ee.append(ee)
                current_ee_names.add(ee.name)

        new_gs = list(self.group_states)
        current_gs_names = {gs.name for gs in new_gs}
        for gs in other.group_states:
            if gs.name not in current_gs_names:
                new_gs.append(gs)
                current_gs_names.add(gs.name)

        new_no_default = list(self.no_default_collision_links)
        current_no_default = set(new_no_default)
        for link_name in other.no_default_collision_links:
            if link_name not in current_no_default:
                new_no_default.append(link_name)
                current_no_default.add(link_name)

        new_lsa = list(self.link_sphere_approximations)
        current_lsa_links = {lsa.link for lsa in new_lsa}
        for lsa in other.link_sphere_approximations:
            if lsa.link not in current_lsa_links:
                new_lsa.append(lsa)
                current_lsa_links.add(lsa.link)

        new_jp = list(self.joint_properties)
        current_jp = {(jp.joint_name, jp.property_name) for jp in new_jp}
        for jp in other.joint_properties:
            if (jp.joint_name, jp.property_name) not in current_jp:
                new_jp.append(jp)
                current_jp.add((jp.joint_name, jp.property_name))

        return replace(
            self,
            groups=tuple(new_groups),
            virtual_joints=tuple(new_vjoints),
            passive_joints=tuple(new_passive),
            disabled_collisions=tuple(new_disabled),
            enabled_collisions=tuple(new_enabled),
            end_effectors=tuple(new_ee),
            group_states=tuple(new_gs),
            no_default_collision_links=tuple(new_no_default),
            link_sphere_approximations=tuple(new_lsa),
            joint_properties=tuple(new_jp),
        )
