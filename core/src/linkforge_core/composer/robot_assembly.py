"""Hybrid RobotAssembly API for LinkForge.

This module implements the 'Composer' which allows for both macro-assembly
(attaching sub-robots) and micro-construction (programmatic link/joint building).
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ..exceptions import RobotValidationError, ValidationErrorCode
from ..generators.srdf_generator import SRDFGenerator
from ..generators.urdf_generator import URDFGenerator
from ..models.geometry import Transform, Vector3
from ..models.joint import Joint, JointLimits, JointType
from ..models.link import Link
from ..models.robot import Robot
from ..models.srdf import (
    DisabledCollision,
    PlanningGroup,
    SemanticRobotDescription,
)


@dataclass
class RobotAssembly:
    """A high-level API to compose robots from multiple components.

    Attributes:
        robot: The underlying robot model being composed.
        srdf: The semantic robot description (SRDF) for MoveIt support.
    """

    robot: Robot
    srdf: SemanticRobotDescription = field(default_factory=SemanticRobotDescription)

    def __post_init__(self) -> None:
        """Sync the root robot's semantic description if it exists."""
        if self.robot.semantic:
            self.srdf = self.robot.semantic
        else:
            self.robot.semantic = self.srdf

    @classmethod
    def create(cls, name: str) -> RobotAssembly:
        """Create a new empty robot assembly.

        Args:
            name: Name of the new robot.

        Returns:
            A new RobotAssembly instance.
        """
        return cls(robot=Robot(name=name))

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
        """Internal helper to add a link and its connecting joint.

        Args:
            link: The link to add.
            parent: Name of the parent link.
            joint_name: Name of the joint connecting them.
            joint_type: Type of the joint.
            origin: Optional transform.
            axis: Optional joint axis.
            limits: Optional joint limits.
        """
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
    ) -> RobotAssembly:
        """Attach a sub-robot component to the current assembly.

        Args:
            component: The robot model to attach.
            at_link: The link in the current assembly to attach to.
            joint_name: Name of the joint connecting the assembly to the component.
            prefix: Optional prefix to add to all elements in the component.
            joint_type: Type of the connecting joint (default: FIXED).
            origin: Optional transform for the joint.
            axis: Optional joint axis.
            limits: Optional joint limits.

        Returns:
            The assembly instance for chaining.
        """
        # 0. Early validation of attachment point
        if not self.robot.get_link(at_link):
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Attachment link '{at_link}' not found in assembly",
                target="Attach",
                value=at_link,
            )

        # 1. Deep copy the component to ensure isolation
        sub_robot = component.clone()

        # 2. Apply prefix if provided
        if prefix:
            sub_robot.prefix_all(prefix)
            joint_name = f"{prefix}{joint_name}"

        # 3. Identify the root link of the sub-robot
        root_link = sub_robot.get_root_link()
        if not root_link:
            raise RobotValidationError(
                ValidationErrorCode.NO_ROOT,
                f"No root link found in component '{component.name}'",
                target="Attach",
                value=component.name,
            )

        # 4. Merge links
        for link in sub_robot.links:
            self.robot.add_link(link)

        # 5. Merge joints
        for joint in sub_robot.joints:
            self.robot.add_joint(joint)

        # 6. Create the connecting joint
        connection = Joint(
            name=joint_name,
            type=joint_type,
            parent=at_link,
            child=root_link.name,
            origin=origin or Transform.identity(),
            axis=axis,
            limits=limits,
        )
        self.robot.add_joint(connection)

        # 7. Merge additional elements (sensors, transmissions, etc.)
        for sensor in sub_robot.sensors:
            self.robot.add_sensor(sensor)

        for trans in sub_robot.transmissions:
            self.robot.add_transmission(trans)

        for rc in sub_robot.ros2_controls:
            self.robot.add_ros2_control(rc)

        for gz in sub_robot.gazebo_elements:
            self.robot.add_gazebo_element(gz)

        # 8. Merge materials
        self.robot.materials.update(sub_robot.materials)

        # 9. Merge semantic data (SRDF)
        if sub_robot.semantic:
            self._merge_srdf(sub_robot.semantic)

        # 10. Validate kinematic integrity
        _ = (
            self.robot.graph
        )  # Accessing the property triggers validation of connectivity and cycles

        return self

    def _merge_srdf(self, other: SemanticRobotDescription) -> None:
        """Merge another SRDF description into the assembly's SRDF."""
        self.srdf.virtual_joints.extend(other.virtual_joints)
        self.srdf.groups.extend(other.groups)
        self.srdf.group_states.extend(other.group_states)
        self.srdf.end_effectors.extend(other.end_effectors)
        self.srdf.passive_joints.extend(other.passive_joints)
        self.srdf.disabled_collisions.extend(other.disabled_collisions)

    def add_link(self, name: str) -> LinkBuilder:
        """Begin building a new link programmatically.

        Args:
            name: Unique name for the link.

        Returns:
            A LinkBuilder instance for fluent construction.
        """
        link = Link(name=name)
        return LinkBuilder(self, link)

    def add_group(
        self,
        name: str,
        links: list[str] | None = None,
        joints: list[str] | None = None,
        chains: list[tuple[str, str]] | None = None,
    ) -> RobotAssembly:
        """Add a planning group for MoveIt.

        Args:
            name: Unique group name.
            links: List of link names.
            joints: List of joint names.
            chains: List of (base_link, tip_link) tuples.

        Returns:
            The assembly instance.
        """
        group = PlanningGroup(
            name=name, links=links or [], joints=joints or [], chains=chains or []
        )
        self.srdf.groups.append(group)
        return self

    def disable_collisions(self, link1: str, link2: str, reason: str = "Adjacent") -> RobotAssembly:
        """Disable collision checking between two links.

        Args:
            link1: First link name.
            link2: Second link name.
            reason: Reason for disabling (default: 'Adjacent').

        Returns:
            The assembly instance for chaining.
        """
        dc = DisabledCollision(link1=link1, link2=link2, reason=reason)
        self.srdf.disabled_collisions.append(dc)
        return self

    def disable_all_collisions(self, links: list[str], reason: str = "Adjacent") -> RobotAssembly:
        """Disable collision checking between all pairs in the provided list.

        Args:
            links: List of link names to disable collisions between.
            reason: Reason for disabling (default: 'Adjacent').

        Returns:
            The assembly instance for chaining.
        """
        import itertools

        for l1, l2 in itertools.combinations(links, 2):
            self.disable_collisions(l1, l2, reason)
        return self

    def export_urdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Export the assembled robot to URDF XML.

        Args:
            validate: Whether to run full kinematic validation (default: True).
            pretty_print: Whether to indent the XML (default: True).

        Returns:
            URDF XML string.
        """
        generator = URDFGenerator(pretty_print=pretty_print)
        return generator.generate(self.robot, validate=validate)

    def export_srdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Export the assembled semantic description to SRDF XML.

        Args:
            validate: Whether to validate (default: True).
            pretty_print: Whether to indent the XML (default: True).

        Returns:
            SRDF XML string.
        """
        generator = SRDFGenerator(pretty_print=pretty_print)
        return generator.generate(self.robot, validate=validate)


class LinkBuilder:
    """Staged fluent builder for programmatic link and joint construction.

    The builder accumulates state in discrete steps. Call connect_to() to
    stage the parent link and joint name, then finalize with one of the
    typed terminal methods (as_fixed, as_revolute, as_prismatic, etc.).

    Example:
        assembly.add_link("bracket") \\
            .with_mass(0.5) \\
            .at_origin(xyz=(0.1, 0, 0)) \\
            .connect_to("flange", "bracket_joint") \\
            .as_fixed()
    """

    def __init__(self, assembly: RobotAssembly, link: Link) -> None:
        """Initialize the builder.

        Args:
            assembly: The assembly this link will belong to.
            link: The link being built.
        """
        self._assembly = assembly
        self._link = link
        self._pending_origin: Transform | None = None
        self._pending_parent: str | None = None
        self._pending_joint_name: str | None = None

    def with_mass(self, value: float) -> LinkBuilder:
        """Set the link's mass and calculate default inertia.

        Args:
            value: Mass in kilograms.

        Returns:
            The builder instance for chaining.
        """
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
        """Store a custom transform to use when connecting this link.

        This transform will be applied to the connecting joint if no
        explicit origin is provided at connection time.

        Args:
            xyz: Translation as (x, y, z) in meters.
            rpy: Rotation as (roll, pitch, yaw) in radians.

        Returns:
            The builder instance for chaining.
        """
        from ..models.geometry import Transform, Vector3

        self._pending_origin = Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy))
        return self

    def connect_to(self, parent: str, joint_name: str) -> LinkBuilder:
        """Stage the joint's topology (parent and name).

        This is a configuration step. You must call one of the terminal
        ``as_*`` methods afterwards to finalize the connection.

        Args:
            parent: Name of the parent link in the assembly.
            joint_name: Unique name for the connecting joint.

        Returns:
            The builder instance for chaining.
        """
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

    def as_fixed(self, origin: Transform | None = None) -> RobotAssembly:
        """Finalize the connection as a fixed joint.

        Args:
            origin: Optional transform override.

        Returns:
            The parent RobotAssembly instance.
        """
        parent, name, resolved_origin = self._get_connection_params(origin)
        self._assembly._add_link_with_joint(
            link=self._link,
            parent=parent,
            joint_name=name,
            joint_type=JointType.FIXED,
            origin=resolved_origin,
        )
        return self._assembly

    def as_revolute(
        self,
        axis: Vector3,
        limits: JointLimits,
        origin: Transform | None = None,
    ) -> RobotAssembly:
        """Finalize the connection as a revolute joint.

        Args:
            axis: Rotation axis unit vector.
            limits: Joint position, effort, and velocity limits.
            origin: Optional transform override.

        Returns:
            The parent RobotAssembly instance.
        """
        parent, name, resolved_origin = self._get_connection_params(origin)
        self._assembly._add_link_with_joint(
            link=self._link,
            parent=parent,
            joint_name=name,
            joint_type=JointType.REVOLUTE,
            origin=resolved_origin,
            axis=axis,
            limits=limits,
        )
        return self._assembly

    def as_prismatic(
        self,
        axis: Vector3,
        limits: JointLimits,
        origin: Transform | None = None,
    ) -> RobotAssembly:
        """Finalize the connection as a prismatic (sliding) joint.

        Args:
            axis: Translation axis unit vector.
            limits: Joint position, effort, and velocity limits.
            origin: Optional transform override.

        Returns:
            The parent RobotAssembly instance.
        """
        parent, name, resolved_origin = self._get_connection_params(origin)
        self._assembly._add_link_with_joint(
            link=self._link,
            parent=parent,
            joint_name=name,
            joint_type=JointType.PRISMATIC,
            origin=resolved_origin,
            axis=axis,
            limits=limits,
        )
        return self._assembly

    def as_continuous(self, axis: Vector3, origin: Transform | None = None) -> RobotAssembly:
        """Finalize the connection as a continuous (unlimited revolute) joint.

        Args:
            axis: Rotation axis unit vector.
            origin: Optional transform override.

        Returns:
            The parent RobotAssembly instance.
        """
        parent, name, resolved_origin = self._get_connection_params(origin)
        self._assembly._add_link_with_joint(
            link=self._link,
            parent=parent,
            joint_name=name,
            joint_type=JointType.CONTINUOUS,
            origin=resolved_origin,
            axis=axis,
        )
        return self._assembly

    def as_joint(
        self,
        joint_type: JointType,
        axis: Vector3 | None = None,
        limits: JointLimits | None = None,
        origin: Transform | None = None,
    ) -> RobotAssembly:
        """Generic finalization for any joint type.

        Use this for rarely-used types (PLANAR, FLOATING) or custom extensions.

        Args:
            joint_type: The type of joint.
            axis: Optional axis vector.
            limits: Optional joint limits.
            origin: Optional transform override.

        Returns:
            The parent RobotAssembly instance.
        """
        parent, name, resolved_origin = self._get_connection_params(origin)
        self._assembly._add_link_with_joint(
            link=self._link,
            parent=parent,
            joint_name=name,
            joint_type=joint_type,
            origin=resolved_origin,
            axis=axis,
            limits=limits,
        )
        return self._assembly
