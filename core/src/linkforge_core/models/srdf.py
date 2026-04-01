"""Semantic robot description models (SRDF).

This module provides data structures to represent MoveIt-style semantic information,
such as planning groups, poses, and collision filters.
"""

from __future__ import annotations

from dataclasses import dataclass, field


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


@dataclass(frozen=True)
class GroupState:
    """A named set of joint values for a planning group (a pose).

    Attributes:
        name: Unique name for this pose (e.g., 'home', 'folded').
        group: Name of the planning group this state applies to.
        joint_values: Dictionary mapping joint names to their target values.
    """

    name: str
    group: str
    joint_values: dict[str, float] = field(default_factory=dict)


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


@dataclass(frozen=True)
class PassiveJoint:
    """A joint that is not actuated but exists in the kinematic chain.

    Attributes:
        name: Name of the passive joint.
    """

    name: str


@dataclass(frozen=True)
class DisabledCollision:
    """Disables collision checking between two specific links.

    Attributes:
        link1: Name of the first link.
        link2: Name of the second link.
        reason: Optional human-readable reason (e.g., 'Adjacent', 'Never').
    """

    link1: str
    link2: str
    reason: str | None = None


@dataclass(frozen=True)
class PlanningGroup:
    """A named collection of links, joints, or chains used for motion planning.

    Attributes:
        name: Unique name for the planning group (e.g., 'arm', 'gripper').
        links: List of link names included in the group.
        joints: List of joint names included in the group.
        chains: List of (base_link, tip_link) tuples defining kinematic chains.
        subgroups: List of other planning group names to include.
    """

    name: str
    links: list[str] = field(default_factory=list)
    joints: list[str] = field(default_factory=list)
    chains: list[tuple[str, str]] = field(default_factory=list)
    subgroups: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class SemanticRobotDescription:
    """Container for all semantic information (SRDF).

    This class serves as the central point for MoveIt-compatible metadata
    that exists alongside the kinematic URDF description.
    """

    virtual_joints: list[VirtualJoint] = field(default_factory=list)
    groups: list[PlanningGroup] = field(default_factory=list)
    group_states: list[GroupState] = field(default_factory=list)
    end_effectors: list[EndEffector] = field(default_factory=list)
    passive_joints: list[PassiveJoint] = field(default_factory=list)
    disabled_collisions: list[DisabledCollision] = field(default_factory=list)
