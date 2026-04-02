"""Hybrid RobotAssembly API for LinkForge.

This module implements the 'Composer' which allows for both macro-assembly
(attaching sub-robots) and micro-construction (programmatic link/joint building).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..exceptions import RobotValidationError
from ..models.joint import Joint, JointAxis, JointLimit, JointType
from ..models.link import Link
from ..models.robot import Robot
from ..models.srdf import DisabledCollision, PlanningGroup, SemanticRobotDescription


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

    def attach(
        self,
        component: Robot,
        at_link: str,
        joint_name: str,
        prefix: str = "",
        joint_type: JointType = JointType.FIXED,
        origin: Any | None = None,
        axis: JointAxis | None = None,
        limit: JointLimit | None = None,
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
            limit: Optional joint limits.

        Returns:
            The assembly instance for chaining.
        """
        # 1. Deep copy the component to ensure isolation
        sub_robot = component.clone()

        # 2. Apply prefix if provided
        if prefix:
            sub_robot.prefix_all(prefix)
            joint_name = f"{prefix}{joint_name}"

        # 3. Identify the root link of the sub-robot
        root_link = sub_robot.get_root_link()
        if not root_link:
            raise RobotValidationError("Attach", component.name, "No root link found in component")

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
            origin=origin,
            axis=axis,
            limit=limit,
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
            reason: Reason for disabling.

        Returns:
            The assembly instance.
        """
        dc = DisabledCollision(link1=link1, link2=link2, reason=reason)
        self.srdf.disabled_collisions.append(dc)
        return self


class LinkBuilder:
    """Fluent API for programmatic link construction."""

    def __init__(self, assembly: RobotAssembly, link: Link) -> None:
        self._assembly = assembly
        self._link = link

    def with_mass(self, value: float) -> LinkBuilder:
        """Set link mass."""
        self._link.mass = value
        return self

    def connect_to(
        self,
        parent: str,
        joint_name: str,
        joint_type: JointType = JointType.FIXED,
        origin: Any | None = None,
    ) -> RobotAssembly:
        """Finish link construction and connect it to a parent.

        Args:
            parent: Name of the parent link.
            joint_name: Name of the joint.
            joint_type: Type of the joint.
            origin: Optional transform.

        Returns:
            The parent RobotAssembly instance.
        """
        self._assembly.robot.add_link(self._link)
        joint = Joint(
            name=joint_name,
            type=joint_type,
            parent=parent,
            child=self._link.name,
            origin=origin,
        )
        self._assembly.robot.add_joint(joint)
        return self._assembly
