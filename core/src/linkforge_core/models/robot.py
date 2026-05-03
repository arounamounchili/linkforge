"""Central Robot model representing the LinkForge Intermediate Representation (IR).

This module provides the core `Robot` class, which serves as the central
hub for all kinematic, physical, and sensor data within the LinkForge ecosystem.
"""

from __future__ import annotations

import copy
import itertools
from collections.abc import Sequence
from dataclasses import InitVar, dataclass, field, replace
from pathlib import Path
from typing import Any

from ..base import FileSystemResolver, IResourceResolver
from ..exceptions import RobotValidationError, ValidationErrorCode
from ..utils.string_utils import is_valid_name
from .gazebo import GazeboElement
from .geometry import Transform, Vector3
from .graph import KinematicGraph
from .joint import Joint, JointLimits, JointType
from .link import Link
from .material import Material
from .ros2_control import Ros2Control
from .sensor import Sensor
from .srdf import (
    Chain,
    CollisionPair,
    JointProperty,
    LinkSphereApproximation,
    PlanningGroup,
    SemanticRobotDescription,
    SrdfSphere,
)
from .transmission import Transmission


@dataclass
class Robot:
    """Complete robot description containing links, joints, and metadata.

    The Robot class acts as the central hub of the LinkForge Intermediate
    Representation (IR). It maintains a collection of rigid bodies (Links)
    connected by kinematic constraints (Joints), along with sensors,
    transmissions, and format-specific metadata.

    Attributes:
        name: Unique identifier for the robot.
        version: LinkForge IR schema version (e.g., '1.1').
        materials: Global material library shared across links.
        metadata: Arbitrary dictionary for format-specific extensions.
        resource_resolver: Strategy for locating meshes and external files.

    Note:
        Uses O(1) hash map lookups for links and joints via internal indices.
        The kinematic structure (parent-child tree) is managed via the
        ``graph`` property.
    """

    name: str
    version: str = "1.1"  # LinkForge IR Version
    materials: dict[str, Material] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)
    resource_resolver: IResourceResolver = field(default_factory=FileSystemResolver)

    # Internal storage
    _links: list[Link] = field(default_factory=list, init=False)
    _joints: list[Joint] = field(default_factory=list, init=False)
    _sensors: list[Sensor] = field(default_factory=list, init=False)
    _transmissions: list[Transmission] = field(default_factory=list, init=False)
    _ros2_controls: list[Ros2Control] = field(default_factory=list, init=False)
    _gazebo_elements: list[GazeboElement] = field(default_factory=list, init=False)
    _semantic: SemanticRobotDescription = field(
        default_factory=SemanticRobotDescription, init=False
    )

    # Fast lookup indices (name -> object)
    _link_index: dict[str, Link] = field(default_factory=dict, init=False, repr=False)
    _joint_index: dict[str, Joint] = field(default_factory=dict, init=False, repr=False)
    _sensor_index: dict[str, Sensor] = field(default_factory=dict, init=False, repr=False)

    _graph_cache: KinematicGraph | None = field(default=None, init=False, repr=False)

    # Init args
    initial_links: InitVar[Sequence[Link] | None] = None
    initial_joints: InitVar[Sequence[Joint] | None] = None
    initial_sensors: InitVar[Sequence[Sensor] | None] = None
    initial_transmissions: InitVar[Sequence[Transmission] | None] = None
    initial_ros2_controls: InitVar[Sequence[Ros2Control] | None] = None
    initial_gazebo_elements: InitVar[Sequence[GazeboElement] | None] = None
    initial_semantic: InitVar[SemanticRobotDescription | None] = None

    def __post_init__(
        self,
        initial_links: Sequence[Link] | None,
        initial_joints: Sequence[Joint] | None,
        initial_sensors: Sequence[Sensor] | None = None,
        initial_transmissions: Sequence[Transmission] | None = None,
        initial_ros2_controls: Sequence[Ros2Control] | None = None,
        initial_gazebo_elements: Sequence[GazeboElement] | None = None,
        initial_semantic: SemanticRobotDescription | None = None,
    ) -> None:
        """Initialize and index the robot structure.

        This method validates the robot name and populates the internal
        storage with any components provided during instantiation.

        Args:
            initial_links: Links to add to the robot.
            initial_joints: Joints connecting the links.
            initial_sensors: Attached sensors (Lidar, Camera, etc.).
            initial_transmissions: Mechanical transmission definitions.
            initial_ros2_controls: Hardware interface configurations.
            initial_gazebo_elements: Simulation-specific metadata.
            initial_semantic: MoveIt/SRDF semantic metadata.

        Raises:
            RobotValidationError: If the robot name is empty or invalid.
        """
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "Robot name cannot be empty",
                target="RobotName",
                value=self.name,
            )

        # Validate naming convention
        if not is_valid_name(self.name):
            raise RobotValidationError(
                ValidationErrorCode.INVALID_NAME,
                "Invalid name format",
                target="RobotName",
                value=self.name,
            )

        # Initialize storage
        if initial_links:
            for link in initial_links:
                self.add_link(link)
        if initial_joints:
            for joint in initial_joints:
                self.add_joint(joint)
        if initial_sensors:
            for sensor in initial_sensors:
                self.add_sensor(sensor)
        if initial_transmissions:
            for trans in initial_transmissions:
                self.add_transmission(trans)
        if initial_ros2_controls:
            for ros2_ctrl in initial_ros2_controls:
                self.add_ros2_control(ros2_ctrl)
        if initial_gazebo_elements:
            for gz in initial_gazebo_elements:
                self.add_gazebo_element(gz)
        if initial_semantic:
            self._semantic = replace(initial_semantic, robot_name=self.name)
        else:
            self._semantic = replace(self._semantic, robot_name=self.name)

        self._reindex()

    def _reindex(self) -> None:
        """Rebuild internal lookup indices and clear cache.

        This is an internal maintenance method that ensures the O(1)
        lookup maps stay in sync with the list-based storage.

        Raises:
            RobotValidationError: If duplicate link or joint names are detected.
        """
        # Validate link names and build index
        self._link_index = {}
        for link in self._links:
            if link.name in self._link_index:
                raise RobotValidationError(
                    ValidationErrorCode.DUPLICATE_NAME,
                    f"Already exists: Link '{link.name}'",
                    target="Link",
                    value=link.name,
                )
            self._link_index[link.name] = link

        # Validate joint names and build index
        self._joint_index = {}
        for joint in self._joints:
            if joint.name in self._joint_index:
                raise RobotValidationError(
                    ValidationErrorCode.DUPLICATE_NAME,
                    f"Already exists: Joint '{joint.name}'",
                    target="Joint",
                    value=joint.name,
                )
            self._joint_index[joint.name] = joint

        self._sensor_index = {sensor.name: sensor for sensor in self._sensors}
        self._graph_cache = None

    def clone(self) -> Robot:
        """Create a deep copy of the robot.

        Returns:
            A new Robot instance with identical links, joints, and metadata.
        """
        return copy.deepcopy(self)

    def prefix_all(self, prefix: str) -> None:
        """Add a namespace prefix to all components in the robot.

        This is a recursive operation that updates names for links, joints,
        sensors, transmissions, ros2_control interfaces, and semantic data.
        It is primarily used during 'RobotBuilder.attach()' to prevent
        name collisions.

        Args:
            prefix: The string prefix to prepend (e.g., ``arm_``).
        """
        if not prefix:
            return

        # Update Materials (Global)
        self.materials = {f"{prefix}{k}": v.with_prefix(prefix) for k, v in self.materials.items()}

        # Update Components
        self._links = [link.with_prefix(prefix) for link in self._links]
        self._joints = [joint.with_prefix(prefix) for joint in self._joints]
        self._sensors = [sensor.with_prefix(prefix) for sensor in self._sensors]
        self._transmissions = [trans.with_prefix(prefix) for trans in self._transmissions]
        self._ros2_controls = [rc.with_prefix(prefix) for rc in self._ros2_controls]
        self._gazebo_elements = [ge.with_prefix(prefix) for ge in self._gazebo_elements]
        self._semantic = self._semantic.with_prefix(prefix)

        self._reindex()

    def add_link(self, link: Link) -> None:
        """Add a link to the robot and update indices.

        Args:
            link: The Link object to add.

        Raises:
            RobotValidationError: If a link with the same name already exists
                or if naming conventions are violated.
        """
        if link.name in self._link_index:
            raise RobotValidationError(
                ValidationErrorCode.DUPLICATE_NAME,
                f"Already exists: Link '{link.name}'",
                target="Link",
                value=link.name,
            )
        self._links.append(link)
        self._link_index[link.name] = link
        self._graph_cache = None

    def add_joint(self, joint: Joint) -> None:
        """Add a joint to the robot and update indices.

        Args:
            joint: The Joint object to add.

        Raises:
            RobotValidationError: If the joint name is a duplicate or if the
                referenced parent/child links do not exist.
        """
        if joint.name in self._joint_index:
            raise RobotValidationError(
                ValidationErrorCode.DUPLICATE_NAME,
                f"Already exists: Joint '{joint.name}'",
                target="Joint",
                value=joint.name,
            )

        # Validate parent and child links exist
        if joint.parent not in self._link_index:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Not found: Parent link '{joint.parent}'",
                target="ParentLink",
                value=joint.parent,
            )
        if joint.child not in self._link_index:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Not found: Child link '{joint.child}'",
                target="ChildLink",
                value=joint.child,
            )

        self._joints.append(joint)
        self._joint_index[joint.name] = joint
        self._graph_cache = None

    def resolve_resource(self, uri: str, relative_to: Path | None = None) -> Path:
        """Resolve a resource URI using the robot's configured resolver.

        Args:
            uri: The resource URI to resolve (e.g. mesh path, package://).
            relative_to: Optional base directory for relative path resolution.

        Returns:
            The resolved absolute Path.
        """
        return self.resource_resolver.resolve(uri, relative_to=relative_to)

    def get_link(self, name: str) -> Link | None:
        """Retrieve a link by name using the internal index.

        Args:
            name: The name of the link to find.

        Returns:
            The Link object if found, otherwise None.
        """
        return self._link_index.get(name)

    def link(self, name: str) -> Link:
        """Retrieve a link by name, raising an error if it does not exist.

        Args:
            name: The name of the link to find.

        Returns:
            The Link object.

        Raises:
            RobotValidationError: If the link is not found.
        """
        link_obj = self.get_link(name)
        if link_obj is None:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Link '{name}' not found in robot '{self.name}'",
                target="Link",
                value=name,
            )
        return link_obj

    def get_joint(self, name: str) -> Joint | None:
        """Retrieve a joint by name using the internal index.

        Args:
            name: The name of the joint to find.

        Returns:
            The Joint object if found, otherwise None.
        """
        return self._joint_index.get(name)

    def joint(self, name: str) -> Joint:
        """Retrieve a joint by name, raising an error if it does not exist.

        Args:
            name: The name of the joint to find.

        Returns:
            The Joint object.

        Raises:
            RobotValidationError: If the joint is not found.
        """
        joint_obj = self.get_joint(name)
        if joint_obj is None:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Joint '{name}' not found in robot '{self.name}'",
                target="Joint",
                value=name,
            )
        return joint_obj

    def has_link(self, name: str) -> bool:
        """Check if a link with the given name exists in the robot."""
        return name in self._link_index

    def has_joint(self, name: str) -> bool:
        """Check if a joint with the given name exists in the robot."""
        return name in self._joint_index

    def get_joints_for_link(self, link_name: str, as_parent: bool = True) -> list[Joint]:
        """Get all joints where the link is parent or child.

        Args:
            link_name: Name of the link
            as_parent: If True, get joints where link is parent; if False, where link is child

        Returns:
            List of matching joints.
        """
        if as_parent:
            return [joint for joint in self.joints if joint.parent == link_name]
        else:
            return [joint for joint in self.joints if joint.child == link_name]

    def add_sensor(self, sensor: Sensor) -> None:
        """Attach a sensor to the robot model.

        Args:
            sensor: The Sensor object to add.

        Raises:
            RobotValidationError: If the sensor name is a duplicate or
                referenced link does not exist.
        """
        if sensor.name in self._sensor_index:
            raise RobotValidationError(
                ValidationErrorCode.DUPLICATE_NAME,
                f"Already exists: Sensor '{sensor.name}'",
                target="Sensor",
                value=sensor.name,
            )

        # Validate that the link exists
        if sensor.link_name not in self._link_index:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Not found: Link '{sensor.link_name}'",
                target="LinkName",
                value=sensor.link_name,
            )

        self._sensors.append(sensor)
        self._sensor_index[sensor.name] = sensor

    def add_transmission(self, transmission: Transmission) -> None:
        """Define a mechanical transmission for one or more joints.

        Args:
            transmission: The Transmission definition to add.

        Raises:
            RobotValidationError: If the transmission name is a duplicate
                or referenced joints do not exist.
        """
        if any(t.name == transmission.name for t in self._transmissions):
            raise RobotValidationError(
                ValidationErrorCode.DUPLICATE_NAME,
                f"Already exists: Transmission '{transmission.name}'",
                target="Transmission",
                value=transmission.name,
            )

        # Validate that all referenced joints exist
        for trans_joint in transmission.joints:
            if trans_joint.name not in self._joint_index:
                raise RobotValidationError(
                    ValidationErrorCode.NOT_FOUND,
                    f"Not found: Joint '{trans_joint.name}'",
                    target="JointName",
                    value=trans_joint.name,
                )

        self._transmissions.append(transmission)

    def add_gazebo_element(self, element: GazeboElement) -> None:
        """Add simulation-specific metadata (Gazebo tags).

        Args:
            element: The GazeboElement definition.

        Raises:
            RobotValidationError: If the referenced link/joint does not exist.
        """
        # Validate reference if specified
        if (
            element.reference is not None
            and self.get_link(element.reference) is None
            and self.get_joint(element.reference) is None
        ):
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Not found: Gazebo reference '{element.reference}'",
                target="GazeboReference",
                value=element.reference,
            )

        self._gazebo_elements.append(element)

    def add_ros2_control(self, ros2_control: Ros2Control) -> None:
        """Register a ros2_control hardware system.

        Args:
            ros2_control: The hardware configuration to add.

        Raises:
            RobotValidationError: If the configuration name is a duplicate.
        """
        # Check for duplicate names
        if any(rc.name == ros2_control.name for rc in self._ros2_controls):
            raise RobotValidationError(
                ValidationErrorCode.DUPLICATE_NAME,
                f"Already exists: ROS2 control '{ros2_control.name}'",
                target="Ros2Control",
                value=ros2_control.name,
            )

        self._ros2_controls.append(ros2_control)

    @property
    def graph(self) -> KinematicGraph:
        """Get the formal kinematic graph representing the robot's structure.

        This is built on demand (and cached) to ensure it reflects the current state
        of links and joints with optimal performance.
        """
        if self._graph_cache is None:
            self._graph_cache = KinematicGraph(self._links, self._joints)
        return self._graph_cache

    def get_root_link(self) -> Link:
        """Get the root link of the kinematic tree.

        The root link is the one that is never a child in any joint.

        Returns:
            The root Link object.

        Raises:
            RobotValidationError: If no root link is found or multiple root links exist.
        """
        roots = self.graph.get_root_links()
        if not roots:
            raise RobotValidationError(
                ValidationErrorCode.NO_ROOT,
                "No root link found in the kinematic tree",
                target="Roots",
                value=0,
            )
        if len(roots) > 1:
            raise RobotValidationError(
                ValidationErrorCode.MULTIPLE_ROOTS,
                f"Multiple root links found ({len(roots)}): {roots}",
                target="Roots",
                value=len(roots),
            )

        # We can safely call get_link as roots[0] is guaranteed to be in the graph
        link = self.get_link(roots[0])
        if link is None:
            # This should be unreachable given graph integrity
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Root link '{roots[0]}' exists in graph but not in link index",
                target="Roots",
            )
        return link

    @property
    def has_cycle(self) -> bool:
        """Check for cycles in the kinematic tree."""
        return self.graph.has_cycle()

    @property
    def total_mass(self) -> float:
        """Calculate total mass of the robot."""
        return sum(link.mass for link in self.links)

    @property
    def degrees_of_freedom(self) -> int:
        """Calculate total degrees of freedom (actuated joints only)."""
        return sum(joint.degrees_of_freedom for joint in self.joints)

    @property
    def links(self) -> tuple[Link, ...]:
        """Get read-only view of links.

        Use ``add_link()`` to modify the robot structure.
        """
        return tuple(self._links)

    @property
    def joints(self) -> tuple[Joint, ...]:
        """Get read-only view of joints.

        Use ``add_joint()`` to modify the robot structure.
        """
        return tuple(self._joints)

    @property
    def sensors(self) -> tuple[Sensor, ...]:
        """Get read-only view of sensors."""
        return tuple(self._sensors)

    @property
    def transmissions(self) -> tuple[Transmission, ...]:
        """Get read-only view of transmissions."""
        return tuple(self._transmissions)

    @property
    def ros2_controls(self) -> tuple[Ros2Control, ...]:
        """Get read-only view of ROS2 Control configurations."""
        return tuple(self._ros2_controls)

    @property
    def gazebo_elements(self) -> tuple[GazeboElement, ...]:
        """Get read-only view of Gazebo elements."""
        return tuple(self._gazebo_elements)

    @property
    def semantic(self) -> SemanticRobotDescription:
        """Get semantic description (SRDF metadata) of the robot."""
        return self._semantic

    @semantic.setter
    def semantic(self, value: SemanticRobotDescription | None) -> None:
        """Set semantic description of the robot.

        Always syncs the internal robot_name to match this robot's name.
        """
        if value is None:
            self._semantic = replace(SemanticRobotDescription(), robot_name=self.name)
        else:
            self._semantic = replace(value, robot_name=self.name)

    def merge(
        self,
        component: Robot,
        at_link: str,
        joint_name: str,
        prefix: str = "",
        joint_type: JointType = JointType.FIXED,
        origin: Transform | None = None,
        axis: Vector3 | None = None,
        limits: JointLimits | None = None,
    ) -> Robot:
        """Merge a sub-robot (kinematic + semantic) into this robot in-place.

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
            The robot instance for chaining.

        Raises:
            RobotValidationError: If the attachment link is not found.
        """
        # Validation of attachment point
        if not self.get_link(at_link):
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Attachment link '{at_link}' not found in assembly",
                target="Attach",
                value=at_link,
            )

        # Deep copy the component to ensure isolation
        sub_robot = component.clone()

        # Apply prefix if provided
        if prefix:
            sub_robot.prefix_all(prefix)

        # Identify the root link of the sub-robot
        root_link = sub_robot.get_root_link()

        # Merge links and joints
        for link in sub_robot.links:
            self.add_link(link)

        for joint in sub_robot.joints:
            self.add_joint(joint)

        # Merge other components
        for sensor in sub_robot.sensors:
            self.add_sensor(sensor)

        for transmission in sub_robot.transmissions:
            self.add_transmission(transmission)

        for rc in sub_robot.ros2_controls:
            self.add_ros2_control(rc)

        for ge in sub_robot.gazebo_elements:
            self.add_gazebo_element(ge)

        # Merge materials (avoid duplicates by name)
        for name, mat in sub_robot.materials.items():
            if name not in self.materials:
                self.materials[name] = mat

        # Merge semantic data (SRDF)
        self._semantic = self._semantic.merge_with(sub_robot.semantic)

        # Create the connecting joint
        connection = Joint(
            name=joint_name,
            type=joint_type,
            parent=at_link,
            child=root_link.name,
            origin=origin or Transform.identity(),
            axis=axis,
            limits=limits,
        )
        self.add_joint(connection)

        # Merge additional elements (sensors, transmissions, etc.)
        for sensor in sub_robot.sensors:
            self.add_sensor(sensor)

        for trans in sub_robot.transmissions:
            self.add_transmission(trans)

        for rc in sub_robot.ros2_controls:
            self.add_ros2_control(rc)

        for gz in sub_robot.gazebo_elements:
            self.add_gazebo_element(gz)

        # Merge materials
        self.materials.update(sub_robot.materials)

        # Validate kinematic integrity (connectivity and cycles)
        _ = self.graph

        return self

    def add_group(
        self,
        name: str,
        links: list[str] | None = None,
        joints: list[str] | None = None,
        chains: list[Chain] | None = None,
        subgroups: list[str] | None = None,
        base_link: str | None = None,
        tip_link: str | None = None,
    ) -> Robot:
        """Add a planning group for MoveIt.

        Args:
            name: Unique group name.
            links: List of link names.
            joints: List of joint names.
            chains: List of (base_link, tip_link) tuples.
            subgroups: List of subgroup names.
            base_link: Shorthand for chain base.
            tip_link: Shorthand for chain tip.

        Returns:
            The robot instance for chaining.
        """
        # Add group
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
        self._semantic = replace(self._semantic, groups=tuple(self._semantic.groups) + (group,))
        return self

    def disable_collisions(self, link1: str, link2: str, reason: str = "Adjacent") -> Robot:
        """Disable collision checking between two links.

        Args:
            link1: First link name.
            link2: Second link name.
            reason: Reason for disabling (default: 'Adjacent').

        Returns:
            The robot instance for chaining.
        """
        # Disable collisions
        dc = CollisionPair(link1=link1, link2=link2, reason=reason)
        self._semantic = replace(
            self._semantic,
            disabled_collisions=tuple(self._semantic.disabled_collisions) + (dc,),
        )
        return self

    def disable_all_collisions(self, links: list[str], reason: str = "Adjacent") -> Robot:
        """Disable collision checking between all pairs in the provided list.

        Args:
            links: List of link names to disable collisions between.
            reason: Reason for disabling (default: 'Adjacent').

        Returns:
            The robot instance for chaining.
        """
        for l1, l2 in itertools.combinations(links, 2):
            self.disable_collisions(l1, l2, reason)
        return self

    def enable_collisions(self, link1: str, link2: str, reason: str | None = None) -> Robot:
        """Explicitly re-enable collision checking between two links.

        Args:
            link1: First link name.
            link2: Second link name.
            reason: Optional reason for enabling.

        Returns:
            The robot instance for chaining.
        """
        ec = CollisionPair(link1=link1, link2=link2, reason=reason)
        self._semantic = replace(
            self._semantic,
            enabled_collisions=tuple(self._semantic.enabled_collisions) + (ec,),
        )
        return self

    def disable_default_collisions(self, link: str) -> Robot:
        """Disable all default collisions for a specific link.

        Args:
            link: Link name.

        Returns:
            The robot instance for chaining.
        """
        self._semantic = replace(
            self._semantic,
            no_default_collision_links=tuple(self._semantic.no_default_collision_links) + (link,),
        )
        return self

    def add_joint_property(self, joint_name: str, property_name: str, value: str) -> Robot:
        """Add a custom property/metadata to a joint.

        Args:
            joint_name: Name of the joint.
            property_name: Name of the property.
            value: Property value as string.

        Returns:
            The robot instance for chaining.
        """
        jp = JointProperty(joint_name=joint_name, property_name=property_name, value=value)
        self._semantic = replace(
            self._semantic,
            joint_properties=tuple(self._semantic.joint_properties) + (jp,),
        )
        return self

    def approximate_link_collision(self, link: str, spheres: list[SrdfSphere]) -> Robot:
        """Add sphere-based collision approximation for a link.

        Args:
            link: Name of the link.
            spheres: List of SrdfSphere objects.

        Returns:
            The robot instance for chaining.
        """
        lsa = LinkSphereApproximation(link=link, spheres=tuple(spheres))
        self._semantic = replace(
            self._semantic,
            link_sphere_approximations=tuple(self._semantic.link_sphere_approximations) + (lsa,),
        )
        return self

    def export_urdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Export the assembled robot to URDF XML.

        Args:
            validate: Whether to run full kinematic validation (default: True).
            pretty_print: Whether to indent the XML (default: True).

        Returns:
            URDF XML string.
        """
        from ..generators.urdf_generator import URDFGenerator

        generator = URDFGenerator(pretty_print=pretty_print)
        return generator.generate(self, validate=validate)

    def export_srdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Export the assembled semantic description to SRDF XML.

        Args:
            validate: Whether to validate (default: True).
            pretty_print: Whether to indent the XML (default: True).

        Returns:
            SRDF XML string.
        """
        from ..generators.srdf_generator import SRDFGenerator

        generator = SRDFGenerator(pretty_print=pretty_print)
        return generator.generate(self, validate=validate)

    def __str__(self) -> str:
        """Return a human-readable summary of the robot structure."""
        parts = [
            f"Robot(name={self.name}",
            f"links={len(self.links)}",
            f"joints={len(self.joints)}",
            f"dof={self.degrees_of_freedom}",
        ]
        if self.sensors:
            parts.append(f"sensors={len(self.sensors)}")
        if self.transmissions:
            parts.append(f"transmissions={len(self.transmissions)}")
        if self.ros2_controls:
            parts.append(f"ros2_controls={len(self.ros2_controls)}")
        if self.gazebo_elements:
            parts.append(f"gazebo_elements={len(self.gazebo_elements)}")
        return ", ".join(parts) + ")"
