"""Central Robot model representing the LinkForge Intermediate Representation (IR).

This module provides the core `Robot` class, which serves as the central
hub for all kinematic, physical, and sensor data within the LinkForge ecosystem.
"""

from __future__ import annotations

import copy
from collections.abc import Sequence
from dataclasses import InitVar, dataclass, field
from pathlib import Path
from typing import Any

from ..base import FileSystemResolver, IResourceResolver
from ..exceptions import RobotModelError, RobotValidationError
from ..utils.string_utils import is_valid_urdf_name
from .gazebo import GazeboElement
from .graph import KinematicGraph
from .joint import Joint
from .link import Link
from .material import Material
from .ros2_control import Ros2Control
from .sensor import Sensor
from .srdf import SemanticRobotDescription
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
        `graph` property.
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
    _semantic: SemanticRobotDescription | None = field(default=None, init=False)

    # Fast lookup indices (name -> object)
    _link_index: dict[str, Link] = field(default_factory=dict, init=False)
    _joint_index: dict[str, Joint] = field(default_factory=dict, init=False)
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
        """Initialize and index the robot structure."""
        if not self.name:
            raise RobotModelError()

        # Validate naming convention
        if not is_valid_urdf_name(self.name):
            raise RobotValidationError("RobotName", self.name, "Invalid characters")

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
            self._semantic = initial_semantic

        self._reindex()

    def _reindex(self) -> None:
        """Rebuild internal lookup indices and clear cache."""
        # Validate link names and build index
        self._link_index = {}
        for link in self._links:
            if link.name in self._link_index:
                raise RobotValidationError("LinkName", link.name, "Duplicate found in index")
            self._link_index[link.name] = link

        # Validate joint names and build index
        self._joint_index = {}
        for joint in self._joints:
            if joint.name in self._joint_index:
                raise RobotValidationError("JointName", joint.name, "Duplicate found in index")
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
        """Add a prefix to all links, joints, sensors, and related elements.

        This is used when merging robots into an assembly to prevent
        name collisions.

        Args:
            prefix: The string to prepend to all names.
        """
        if not prefix:
            return

        from dataclasses import replace

        # 1. Update Links (Mutable)
        for link in self._links:
            link.name = f"{prefix}{link.name}"

        # 2. Update Joints (Frozen)
        new_joints = []
        for joint in self._joints:
            new_joint = replace(
                joint,
                name=f"{prefix}{joint.name}",
                parent=f"{prefix}{joint.parent}",
                child=f"{prefix}{joint.child}",
            )
            if joint.mimic:
                new_joint = replace(
                    new_joint,
                    mimic=replace(joint.mimic, joint=f"{prefix}{joint.mimic.joint}"),
                )
            new_joints.append(new_joint)
        self._joints = new_joints

        # 3. Update Sensors (Frozen)
        new_sensors = []
        for sensor in self._sensors:
            new_sensor = replace(
                sensor,
                name=f"{prefix}{sensor.name}",
                link_name=f"{prefix}{sensor.link_name}",
            )
            if sensor.contact_info:
                new_sensor = replace(
                    new_sensor,
                    contact_info=replace(
                        sensor.contact_info,
                        collision=f"{prefix}{sensor.contact_info.collision}",
                    ),
                )
            new_sensors.append(new_sensor)
        self._sensors = new_sensors

        # 4. Update Transmissions (Frozen)
        new_transmissions = []
        for trans in self._transmissions:
            new_trans = replace(
                trans,
                name=f"{prefix}{trans.name}",
                joints=[replace(tj, name=f"{prefix}{tj.name}") for tj in trans.joints],
                actuators=[replace(ta, name=f"{prefix}{ta.name}") for ta in trans.actuators],
            )
            new_transmissions.append(new_trans)
        self._transmissions = new_transmissions

        # 5. Update ROS2 Controls (Mutable)
        for rc in self._ros2_controls:
            rc.name = f"{prefix}{rc.name}"
            for rc_joint in rc.joints:
                rc_joint.name = f"{prefix}{rc_joint.name}"

        # 6. Update Gazebo Elements (Frozen)
        new_gazebo_elements = []
        for ge in self._gazebo_elements:
            new_ge = replace(ge, reference=f"{prefix}{ge.reference}" if ge.reference else None)
            new_gazebo_elements.append(new_ge)
        self._gazebo_elements = new_gazebo_elements

        # 7. Update Semantic Description (Frozen)
        if self._semantic:
            s = self._semantic
            self._semantic = replace(
                s,
                virtual_joints=[
                    replace(
                        vj,
                        name=f"{prefix}{vj.name}",
                        child_link=f"{prefix}{vj.child_link}",
                    )
                    for vj in s.virtual_joints
                ],
                groups=[
                    replace(
                        g,
                        name=f"{prefix}{g.name}",
                        links=[f"{prefix}{link_name}" for link_name in g.links],
                        joints=[f"{prefix}{joint_name}" for joint_name in g.joints],
                        chains=[
                            (f"{prefix}{base_name}", f"{prefix}{tip_name}")
                            for base_name, tip_name in g.chains
                        ],
                        subgroups=[f"{prefix}{subgroup_name}" for subgroup_name in g.subgroups],
                    )
                    for g in s.groups
                ],
                group_states=[
                    replace(
                        gs,
                        name=f"{prefix}{gs.name}",
                        group=f"{prefix}{gs.group}",
                        joint_values={f"{prefix}{k}": v for k, v in gs.joint_values.items()},
                    )
                    for gs in s.group_states
                ],
                end_effectors=[
                    replace(
                        ee,
                        name=f"{prefix}{ee.name}",
                        group=f"{prefix}{ee.group}",
                        parent_link=f"{prefix}{ee.parent_link}",
                        parent_group=f"{prefix}{ee.parent_group}" if ee.parent_group else None,
                    )
                    for ee in s.end_effectors
                ],
                passive_joints=[replace(pj, name=f"{prefix}{pj.name}") for pj in s.passive_joints],
                disabled_collisions=[
                    replace(dc, link1=f"{prefix}{dc.link1}", link2=f"{prefix}{dc.link2}")
                    for dc in s.disabled_collisions
                ],
            )

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
            raise RobotValidationError("LinkName", link.name, "Already exists")
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
            raise RobotValidationError("JointName", joint.name, "Already exists")

        # Validate parent and child links exist
        if joint.parent not in self._link_index:
            raise RobotValidationError("ParentLink", joint.parent, "Not found")
        if joint.child not in self._link_index:
            raise RobotValidationError("ChildLink", joint.child, "Not found")

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
        """Get link by name - O(1) lookup."""
        return self._link_index.get(name)

    def get_joint(self, name: str) -> Joint | None:
        """Get joint by name - O(1) lookup."""
        return self._joint_index.get(name)

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
        """Add a sensor to the robot and update indices."""
        if sensor.name in self._sensor_index:
            raise RobotValidationError(
                check_name="DuplicateSensor", value=sensor.name, reason="Already exists"
            )

        # Validate that the link exists
        if sensor.link_name not in self._link_index:
            raise RobotValidationError(
                check_name="LinkName", value=sensor.link_name, reason=sensor.name
            )

        self._sensors.append(sensor)
        self._sensor_index[sensor.name] = sensor

    def add_transmission(self, transmission: Transmission) -> None:
        """Add a transmission to the robot."""
        if any(t.name == transmission.name for t in self._transmissions):
            raise RobotValidationError(
                check_name="DuplicateTransmission", value=transmission.name, reason="Already exists"
            )

        # Validate that all referenced joints exist
        for trans_joint in transmission.joints:
            if trans_joint.name not in self._joint_index:
                raise RobotValidationError(
                    check_name="JointName", value=trans_joint.name, reason=transmission.name
                )

        self._transmissions.append(transmission)

    def add_gazebo_element(self, element: GazeboElement) -> None:
        """Add a Gazebo element to the robot."""
        # Validate reference if specified
        if (
            element.reference is not None
            and self.get_link(element.reference) is None
            and self.get_joint(element.reference) is None
        ):
            raise RobotValidationError(
                check_name="GazeboReference",
                value=element.reference,
                reason="No matching link or joint",
            )

        self._gazebo_elements.append(element)

    def add_ros2_control(self, ros2_control: Ros2Control) -> None:
        """Add a ROS2 Control configuration to the robot."""
        # Check for duplicate names
        if any(rc.name == ros2_control.name for rc in self._ros2_controls):
            raise RobotValidationError("Ros2ControlName", ros2_control.name, "Already exists")

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

    def get_root_link(self) -> Link | None:
        """Get the root link of the kinematic tree.

        The root link is the one that is never a child in any joint.
        """
        if not self.links:
            return None

        roots = self.graph.get_root_links()
        if not roots:
            raise RobotValidationError(check_name="Roots", value=0, reason="No root link found")
        if len(roots) > 1:
            raise RobotValidationError(
                check_name="Roots", value=len(roots), reason="Multiple root links found"
            )

        return self.get_link(roots[0])

    def _has_cycle(self) -> bool:
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

        Use `add_link()` to modify the robot structure.
        """
        return tuple(self._links)

    @property
    def joints(self) -> tuple[Joint, ...]:
        """Get read-only view of joints.

        Use `add_joint()` to modify the robot structure.
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
    def semantic(self) -> SemanticRobotDescription | None:
        """Get semantic description (SRDF metadata) of the robot."""
        return self._semantic

    @semantic.setter
    def semantic(self, value: SemanticRobotDescription | None) -> None:
        """Set semantic description of the robot."""
        self._semantic = value

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
