"""Semantic robot description models (SRDF).

This module provides data structures to represent MoveIt-style semantic information,
such as planning groups, poses, and collision filters.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

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
    joint_values: dict[str, tuple[float, ...]] = field(default_factory=dict)

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
