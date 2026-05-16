"""Semantic properties builder for LinkForge Composer."""

from __future__ import annotations

from dataclasses import replace
from typing import TYPE_CHECKING

from ..constants import (
    SRDF_REASON_ADJACENT,
    SRDF_VJOIN_FIXED,
)
from ..models.srdf import (
    Chain,
    CollisionPair,
    EndEffector,
    GroupState,
    JointProperty,
    LinkSphereApproximation,
    PassiveJoint,
    PlanningGroup,
    SrdfSphere,
    VirtualJoint,
)

if TYPE_CHECKING:
    from .interfaces import IComposer


class SemanticBuilder:
    """Namespace for SRDF and MoveIt-specific semantic properties.

    Accessed via RobotBuilder.semantic.
    """

    def __init__(self, builder: IComposer) -> None:
        """Initialize semantic builder."""
        self._builder = builder

    def group(
        self,
        name: str,
        links: list[str] | None = None,
        joints: list[str] | None = None,
        chains: list[Chain] | None = None,
        subgroups: list[str] | None = None,
        base_link: str | None = None,
        tip_link: str | None = None,
    ) -> IComposer:
        """Define a planning group for MoveIt.

        Args:
            name: Unique name for the group.
            links: List of link names to include.
            joints: List of joint names to include.
            chains: List of (base, tip) tuples for kinematic chains.
            subgroups: List of other planning group names to include.
            base_link: Optional shorthand for chain base.
            tip_link: Optional shorthand for chain tip.

        Returns:
            The parent RobotBuilder instance.
        """
        # Define planning group
        final_chains = list(chains or [])
        if base_link and tip_link:
            final_chains.append(Chain(base_link=base_link, tip_link=tip_link))

        group = PlanningGroup(
            name=name,
            links=tuple(links or []),
            joints=tuple(joints or []),
            chains=tuple(final_chains),
            subgroups=tuple(subgroups or []),
        )

        semantic = self._builder.robot.semantic
        self._builder.robot.semantic = replace(semantic, groups=tuple(semantic.groups) + (group,))
        return self._builder

    def group_state(
        self, name: str, group: str, values: dict[str, float | tuple[float, ...]]
    ) -> IComposer:
        """Define a named state (e.g. 'home') for a planning group.

        Args:
            name: Unique name for the state.
            group: The group this state belongs to.
            values: Dictionary of joint names and their positions.

        Returns:
            The parent RobotBuilder instance.
        """
        normalized_values = {
            k: (v,) if isinstance(v, (int, float)) else tuple(v) for k, v in values.items()
        }
        state = GroupState(name=name, group=group, joint_values=normalized_values)
        semantic = self._builder.robot.semantic
        self._builder.robot.semantic = replace(
            semantic, group_states=tuple(semantic.group_states) + (state,)
        )
        return self._builder

    def end_effector(
        self, name: str, group: str, parent_link: str, parent_group: str | None = None
    ) -> IComposer:
        """Define an end effector for MoveIt.

        Args:
            name: Unique name for the end effector.
            group: The planning group representing the end effector.
            parent_link: The link it is attached to.
            parent_group: Optional parent group (e.g. 'arm').

        Returns:
            The parent RobotBuilder instance.
        """
        ee = EndEffector(name=name, group=group, parent_link=parent_link, parent_group=parent_group)
        semantic = self._builder.robot.semantic
        self._builder.robot.semantic = replace(
            semantic, end_effectors=tuple(semantic.end_effectors) + (ee,)
        )
        return self._builder

    def passive_joint(self, name: str) -> IComposer:
        """Mark a joint as passive (not actuated) for MoveIt.

        Args:
            name: Name of the joint to mark as passive.

        Returns:
            The parent RobotBuilder instance.
        """
        pj = PassiveJoint(name=name)
        semantic = self._builder.robot.semantic
        self._builder.robot.semantic = replace(
            semantic, passive_joints=tuple(semantic.passive_joints) + (pj,)
        )
        return self._builder

    def virtual_joint(
        self,
        name: str,
        child_link: str,
        parent_frame: str = "world",
        joint_type: str = SRDF_VJOIN_FIXED,
    ) -> IComposer:
        """Define a virtual joint connecting the robot to the world frame.

        Args:
            name: Unique joint name.
            child_link: The root link of the robot.
            parent_frame: The external frame (e.g., 'world', 'map').
            joint_type: Joint type (fixed, floating, planar).

        Returns:
            The parent RobotBuilder instance.
        """
        vj = VirtualJoint(
            name=name, type=joint_type, parent_frame=parent_frame, child_link=child_link
        )
        semantic = self._builder.robot.semantic
        self._builder.robot.semantic = replace(
            semantic, virtual_joints=tuple(semantic.virtual_joints) + (vj,)
        )
        return self._builder

    def disable_collisions(
        self, link1: str, link2: str, reason: str = SRDF_REASON_ADJACENT
    ) -> IComposer:
        """Instruct MoveIt to ignore collisions between two specific links.

        Args:
            link1, link2: Names of the links.
            reason: Explanation for disabling (e.g. 'Adjacent', 'Never').

        Returns:
            The parent RobotBuilder instance.
        """
        dc = CollisionPair(link1=link1, link2=link2, reason=reason)
        semantic = self._builder.robot.semantic
        self._builder.robot.semantic = replace(
            semantic, disabled_collisions=tuple(semantic.disabled_collisions) + (dc,)
        )
        return self._builder

    def enable_collisions(self, link1: str, link2: str, reason: str | None = None) -> IComposer:
        """Explicitly re-enable collision checking between two specific links.

        Args:
            link1, link2: Names of the links.
            reason: Optional explanation for enabling.

        Returns:
            The parent RobotBuilder instance.
        """
        ec = CollisionPair(link1=link1, link2=link2, reason=reason)
        semantic = self._builder.robot.semantic
        self._builder.robot.semantic = replace(
            semantic, enabled_collisions=tuple(semantic.enabled_collisions) + (ec,)
        )
        return self._builder

    def disable_default_collisions(self, link: str) -> IComposer:
        """Disable all default collisions for a specific link.

        Args:
            link: Name of the link.

        Returns:
            The parent RobotBuilder instance.
        """
        semantic = self._builder.robot.semantic
        self._builder.robot.semantic = replace(
            semantic,
            no_default_collision_links=tuple(semantic.no_default_collision_links) + (link,),
        )
        return self._builder

    def joint_property(self, joint_name: str, property_name: str, value: str) -> IComposer:
        """Add a custom property/metadata to a joint.

        Args:
            joint_name: Name of the joint.
            property_name: Name of the property.
            value: Property value as string.

        Returns:
            The parent RobotBuilder instance.
        """
        jp = JointProperty(joint_name=joint_name, property_name=property_name, value=value)
        semantic = self._builder.robot.semantic
        self._builder.robot.semantic = replace(
            semantic, joint_properties=tuple(semantic.joint_properties) + (jp,)
        )
        return self._builder

    def approximate_link_collision(self, link: str, spheres: list[SrdfSphere]) -> IComposer:
        """Add sphere-based collision approximation for a link.

        Args:
            link: Name of the link.
            spheres: List of SrdfSphere objects.

        Returns:
            The parent RobotBuilder instance.
        """
        lsa = LinkSphereApproximation(link=link, spheres=tuple(spheres))
        semantic = self._builder.robot.semantic
        self._builder.robot.semantic = replace(
            semantic,
            link_sphere_approximations=tuple(semantic.link_sphere_approximations) + (lsa,),
        )
        return self._builder
