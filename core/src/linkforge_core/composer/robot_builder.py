"""RobotBuilder API for LinkForge.

This module implements the 'Composer' which allows for both macro-assembly
(merging sub-robots) and micro-construction (programmatic link/joint building).
"""

from __future__ import annotations

from ..exceptions import RobotModelError, RobotValidationError, ValidationErrorCode
from ..models.geometry import Transform, Vector3
from ..models.joint import Joint, JointLimits, JointType
from ..models.link import Link
from ..models.robot import Robot


class RobotBuilder:
    """A high-level API to compose robots programmatically.

    Attributes:
        robot: The underlying robot model being composed.
    """

    def __init__(self, name: str | None = None, robot: Robot | None = None) -> None:
        """Initialize a new robot builder.

        Args:
            name: Name of the new robot (if creating from scratch).
            robot: Existing robot model to build upon.
        """
        if robot is not None:
            self.robot = robot
        elif name is not None:
            self.robot = Robot(name=name)
        else:
            raise RobotModelError("Either name or robot must be provided")  # noqa: TRY003

    def _add_link_with_joint(
        self,
        link: Link,
        parent: str,
        joint_name: str,
        joint_type: JointType,
        origin: Transform | None = None,
        axis: Vector3 | None = None,
        limits: JointLimits | None = None,
    ) -> None:
        """Internal helper to add a link and its connecting joint."""
        self.robot.add_link(link)
        joint = Joint(
            name=joint_name,
            type=joint_type,
            parent=parent,
            child=link.name,
            origin=origin or Transform.identity(),
            axis=axis,
            limits=limits,
        )
        self.robot.add_joint(joint)

    def attach(
        self,
        component: Robot,
        at_link: str,
        joint_name: str,
        prefix: str = "",
        joint_type: JointType = JointType.FIXED,
        origin: Transform | None = None,
        axis: Vector3 | None = None,
        limits: JointLimits | None = None,
    ) -> RobotBuilder:
        """Attach a sub-robot component to the current assembly.

        Delegates to `robot.merge()`.
        """
        self.robot.merge(
            component=component,
            at_link=at_link,
            joint_name=joint_name,
            prefix=prefix,
            joint_type=joint_type,
            origin=origin,
            axis=axis,
            limits=limits,
        )
        return self

    def add_link(self, name: str) -> LinkBuilder:
        """Begin building a new link programmatically."""
        link = Link(name=name)
        return LinkBuilder(self, link)

    def add_group(
        self,
        name: str,
        links: list[str] | None = None,
        joints: list[str] | None = None,
        chains: list[tuple[str, str]] | None = None,
    ) -> RobotBuilder:
        """Add a planning group for MoveIt."""
        self.robot.add_group(name=name, links=links, joints=joints, chains=chains)
        return self

    def disable_collisions(self, link1: str, link2: str, reason: str = "Adjacent") -> RobotBuilder:
        """Disable collision checking between two links."""
        self.robot.disable_collisions(link1=link1, link2=link2, reason=reason)
        return self

    def disable_all_collisions(self, links: list[str], reason: str = "Adjacent") -> RobotBuilder:
        """Disable collision checking between all pairs in the provided list."""
        self.robot.disable_all_collisions(links=links, reason=reason)
        return self

    def export_urdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Export the assembled robot to URDF XML."""
        return self.robot.export_urdf(validate=validate, pretty_print=pretty_print)

    def export_srdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Export the assembled semantic description to SRDF XML."""
        return self.robot.export_srdf(validate=validate, pretty_print=pretty_print)

    def build(self) -> Robot:
        """Return the completed Robot model."""
        return self.robot


class LinkBuilder:
    """Staged fluent builder for programmatic link and joint construction."""

    def __init__(self, builder: RobotBuilder, link: Link) -> None:
        self._builder = builder
        self._link = link
        self._pending_origin: Transform | None = None
        self._pending_parent: str | None = None
        self._pending_joint_name: str | None = None

    def with_mass(self, value: float) -> LinkBuilder:
        """Set the link's mass and calculate default inertia."""
        from dataclasses import replace

        from ..models.link import Inertial, InertiaTensor

        if self._link.inertial:
            new_inertial = replace(self._link.inertial, mass=value)
        else:
            new_inertial = Inertial(mass=value, inertia=InertiaTensor.zero())

        self._link = replace(self._link, inertial=new_inertial)
        return self

    def at_origin(
        self,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
    ) -> LinkBuilder:
        """Store a custom transform to use when connecting this link."""
        from ..models.geometry import Transform, Vector3

        self._pending_origin = Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy))
        return self

    def connect_to(self, parent: str, joint_name: str) -> LinkBuilder:
        """Stage the joint's topology (parent and name)."""
        self._pending_parent = parent
        self._pending_joint_name = joint_name
        return self

    def _get_connection_params(self, origin: Transform | None = None) -> tuple[str, str, Transform]:
        """Resolve parent, joint name and origin for finalization."""
        if self._pending_parent is None or self._pending_joint_name is None:
            raise RobotValidationError(
                ValidationErrorCode.GENERIC_FAILURE,
                "connect_to() must be called before finalizing the joint",
                target="LinkBuilder",
                value=self._link.name,
            )

        resolved_origin = origin if origin is not None else self._pending_origin
        return (
            self._pending_parent,
            self._pending_joint_name,
            resolved_origin or Transform.identity(),
        )

    def as_fixed(self, origin: Transform | None = None) -> RobotBuilder:
        """Finalize the connection as a fixed joint."""
        parent, name, resolved_origin = self._get_connection_params(origin)
        self._builder._add_link_with_joint(
            link=self._link,
            parent=parent,
            joint_name=name,
            joint_type=JointType.FIXED,
            origin=resolved_origin,
        )
        return self._builder

    def as_revolute(
        self,
        axis: Vector3,
        limits: JointLimits,
        origin: Transform | None = None,
    ) -> RobotBuilder:
        """Finalize the connection as a revolute joint."""
        parent, name, resolved_origin = self._get_connection_params(origin)
        self._builder._add_link_with_joint(
            link=self._link,
            parent=parent,
            joint_name=name,
            joint_type=JointType.REVOLUTE,
            origin=resolved_origin,
            axis=axis,
            limits=limits,
        )
        return self._builder

    def as_prismatic(
        self,
        axis: Vector3,
        limits: JointLimits,
        origin: Transform | None = None,
    ) -> RobotBuilder:
        """Finalize the connection as a prismatic (sliding) joint."""
        parent, name, resolved_origin = self._get_connection_params(origin)
        self._builder._add_link_with_joint(
            link=self._link,
            parent=parent,
            joint_name=name,
            joint_type=JointType.PRISMATIC,
            origin=resolved_origin,
            axis=axis,
            limits=limits,
        )
        return self._builder

    def as_continuous(self, axis: Vector3, origin: Transform | None = None) -> RobotBuilder:
        """Finalize the connection as a continuous (unlimited revolute) joint."""
        parent, name, resolved_origin = self._get_connection_params(origin)
        self._builder._add_link_with_joint(
            link=self._link,
            parent=parent,
            joint_name=name,
            joint_type=JointType.CONTINUOUS,
            origin=resolved_origin,
            axis=axis,
        )
        return self._builder

    def as_joint(
        self,
        joint_type: JointType,
        axis: Vector3 | None = None,
        limits: JointLimits | None = None,
        origin: Transform | None = None,
    ) -> RobotBuilder:
        """Generic finalization for any joint type."""
        parent, name, resolved_origin = self._get_connection_params(origin)
        self._builder._add_link_with_joint(
            link=self._link,
            parent=parent,
            joint_name=name,
            joint_type=joint_type,
            origin=resolved_origin,
            axis=axis,
            limits=limits,
        )
        return self._builder
