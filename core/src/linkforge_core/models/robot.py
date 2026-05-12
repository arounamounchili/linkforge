"""Central Robot model representing the LinkForge Intermediate Representation (IR).

This module provides the core `Robot` class, which serves as the central
hub for all kinematic, physical, and sensor data within the LinkForge ecosystem.
It acts as a unified data structure that bridges high-fidelity design tools
(like Blender or FreeCAD) with robotics description formats (URDF, XACRO, SRDF).

The LinkForge IR is designed for:
1.  **Format Agnosticism**: Supporting lossless translation between different description formats.
2.  **Physical Integrity**: Ensuring all mass properties and kinematic constraints are validated.
3.  **Extensibility**: Allowing format-specific metadata to be preserved via internal dictionaries.
"""

from __future__ import annotations

import copy
import itertools
from collections import defaultdict
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
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
        links: Read-only access to the collection of rigid bodies.
        joints: Read-only access to kinematic constraints connecting links.
        sensors: Read-only access to attached sensors.
        transmissions: Read-only access to mechanical transmissions.
        ros2_controls: Read-only access to hardware interface configurations.
        gazebo_elements: Read-only access to simulation-specific metadata.
        semantic: MoveIt/SRDF semantic description of the robot.
        graph: Formally verified kinematic structure (rebuilt on demand).

    Note:
        - Uses O(1) hash map lookups for links and joints via internal indices.
    """

    name: str
    version: str = "1.1"  # LinkForge IR Version
    materials: dict[str, Material] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)
    resource_resolver: IResourceResolver = field(default_factory=FileSystemResolver, compare=False)

    # Core collections
    links: Sequence[Link] = field(default_factory=tuple)
    joints: Sequence[Joint] = field(default_factory=tuple)
    sensors: Sequence[Sensor] = field(default_factory=tuple)
    transmissions: Sequence[Transmission] = field(default_factory=tuple)
    ros2_controls: Sequence[Ros2Control] = field(default_factory=tuple)
    gazebo_elements: Sequence[GazeboElement] = field(default_factory=tuple)
    semantic: SemanticRobotDescription = field(default_factory=SemanticRobotDescription)

    # Fast lookup indices (name -> object)
    _link_index: dict[str, Link] = field(
        default_factory=dict, init=False, repr=False, compare=False
    )
    _joint_index: dict[str, Joint] = field(
        default_factory=dict, init=False, repr=False, compare=False
    )
    _sensor_index: dict[str, Sensor] = field(
        default_factory=dict, init=False, repr=False, compare=False
    )
    _transmission_index: dict[str, Transmission] = field(
        default_factory=dict, init=False, repr=False, compare=False
    )
    _ros2_control_index: dict[str, Ros2Control] = field(
        default_factory=dict, init=False, repr=False, compare=False
    )

    # Adjacency maps for kinematic traversal
    _link_as_parent_index: dict[str, list[Joint]] = field(
        default_factory=lambda: defaultdict(list), init=False, repr=False, compare=False
    )
    _link_as_child_index: dict[str, list[Joint]] = field(
        default_factory=lambda: defaultdict(list), init=False, repr=False, compare=False
    )

    _graph_cache: KinematicGraph | None = field(default=None, init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        """Initialize and index the robot structure."""
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

        self.semantic = replace(self.semantic, robot_name=self.name)
        self._reindex()

    def clone(self) -> Robot:
        """Create a deep copy of the robot model."""

        return copy.deepcopy(self)

    def normalized(self) -> Robot:
        """Return a new Robot with all components sorted by name.

        This ensures that structural equality checks are order-independent.
        """
        robot = self.clone()
        robot.links = tuple(sorted(robot.links, key=lambda x: x.name))
        robot.joints = tuple(sorted(robot.joints, key=lambda x: x.name))
        robot.sensors = tuple(sorted(robot.sensors, key=lambda x: x.name))
        robot.transmissions = tuple(
            sorted([t.normalized() for t in robot.transmissions], key=lambda x: x.name)
        )
        robot.ros2_controls = tuple(
            sorted([rc.normalized() for rc in robot.ros2_controls], key=lambda x: x.name)
        )
        robot.gazebo_elements = tuple(
            sorted(robot.gazebo_elements, key=lambda x: x.reference or "")
        )
        robot.semantic = robot.semantic.normalized()
        robot._reindex()
        return robot

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

        # Update Robot Name
        self.name = f"{prefix}{self.name}"

        # Update Materials (Global)
        self.materials = {f"{prefix}{k}": v.with_prefix(prefix) for k, v in self.materials.items()}

        # Update Components
        self.links = tuple(link.with_prefix(prefix) for link in self.links)
        self.joints = tuple(joint.with_prefix(prefix) for joint in self.joints)
        self.sensors = tuple(sensor.with_prefix(prefix) for sensor in self.sensors)
        self.transmissions = tuple(trans.with_prefix(prefix) for trans in self.transmissions)
        self.ros2_controls = tuple(rc.with_prefix(prefix) for rc in self.ros2_controls)
        self.gazebo_elements = tuple(ge.with_prefix(prefix) for ge in self.gazebo_elements)
        self.semantic = self.semantic.with_prefix(prefix)

        self._reindex()

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
        """Merge another robot model into this one at a specific link.

        This operation combines the kinematic tree, sensors, transmissions,
        hardware interfaces, and semantic metadata. A new joint is created
        to connect this robot's `at_link` to the sub-robot's root link.

        Args:
            component: The sub-robot model to attach.
            at_link: The link in the current robot to attach to.
            joint_name: Unique name for the new connecting joint.
            prefix: Optional namespace prefix for all elements in the sub-robot.
            joint_type: Type of the connecting joint (default: FIXED).
            origin: Optional relative transform for the connection.
            axis: Optional joint axis (required for non-fixed types).
            limits: Optional joint limits.

        Returns:
            The current robot instance (self) for method chaining.

        Raises:
            RobotValidationError: If the attachment link is not found or
                if merging results in name collisions or kinematic cycles.
        """
        # Validation of attachment point
        if not self.get_link(at_link):
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Attachment link '{at_link}' not found in assembly",
                target="Attach",
                value=at_link,
            )

        # Prepare component (Namespace Isolation)
        # Deep copy ensures we don't mutate the original template robot
        sub_robot = component.clone()
        if prefix:
            sub_robot.prefix_all(prefix)

        # Identify the connection point (root) of the incoming component
        root_link = sub_robot.get_root_link()

        # Migrate Physical & Functional Collections
        self.links = (*self.links, *sub_robot.links)
        self.joints = (*self.joints, *sub_robot.joints)
        self.sensors = (*self.sensors, *sub_robot.sensors)
        self.transmissions = (*self.transmissions, *sub_robot.transmissions)
        self.ros2_controls = (*self.ros2_controls, *sub_robot.ros2_controls)
        self.gazebo_elements = (*self.gazebo_elements, *sub_robot.gazebo_elements)

        # Re-index to include merged elements
        self._reindex()

        # Integrate Global Resources & Metadata
        self.materials.update(sub_robot.materials)
        self.semantic = self.semantic.merge_with(sub_robot.semantic)

        # Bridge the Kinematic Trees
        # This joint connects our existing structure to the sub-robot's root
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

        # Final Integrity Validation
        # Re-triggering the graph property ensures no cycles or islands were created
        _ = self.graph

        return self

    def add_link(self, link: Link, overwrite: bool = False) -> None:
        """Add a link to the robot and update indices.

        Args:
            link: The Link object to add.
            overwrite: If True, replaces existing link with same name.

        Raises:
            RobotValidationError: If a link with the same name already exists
                and overwrite is False, or if naming conventions are violated.
        """
        if link.name in self._link_index and not overwrite:
            raise RobotValidationError(
                ValidationErrorCode.DUPLICATE_NAME,
                f"Duplicate: Link '{link.name}'",
                target="Link",
                value=link.name,
            )

        if overwrite and link.name in self._link_index:
            self.links = tuple(link if lnk.name == link.name else lnk for lnk in self.links)
        else:
            self.links = (*self.links, link)

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
                f"Duplicate joint name: '{joint.name}'",
                target="Joint",
                value=joint.name,
            )

        # Validate parent and child links exist
        if joint.parent not in self._link_index:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Parent link '{joint.parent}' not found",
                target="ParentLink",
                value=joint.parent,
            )
        if joint.child not in self._link_index:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Child link '{joint.child}' not found",
                target="ChildLink",
                value=joint.child,
            )

        self.joints = (*self.joints, joint)
        self._joint_index[joint.name] = joint
        self._link_as_parent_index[joint.parent].append(joint)
        self._link_as_child_index[joint.child].append(joint)
        self._graph_cache = None

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
                f"Link '{name}' not found",
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
                f"Joint '{name}' not found",
                target="Joint",
                value=name,
            )
        return joint_obj

    def has_link(self, name: str) -> bool:
        """Check if a link with the given name exists in the robot.

        Args:
            name: The name of the link to check.

        Returns:
            True if the link exists, False otherwise.
        """
        return name in self._link_index

    def has_joint(self, name: str) -> bool:
        """Check if a joint with the given name exists in the robot.

        Args:
            name: The name of the joint to check.

        Returns:
            True if the joint exists, False otherwise.
        """
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
            return list(self._link_as_parent_index.get(link_name, []))
        else:
            return list(self._link_as_child_index.get(link_name, []))

    def get_parent_joint(self, link_name: str) -> Joint | None:
        """Get the joint that has this link as its child.

        In a tree structure, a link has at most one parent joint.

        Args:
            link_name: Name of the link.

        Returns:
            The parent Joint if found, otherwise None.
        """
        joints = self._link_as_child_index.get(link_name, [])
        return joints[0] if joints else None

    def get_child_joints(self, link_name: str) -> list[Joint]:
        """Get all joints that have this link as their parent.

        Args:
            link_name: Name of the parent link.

        Returns:
            List of child Joint objects.
        """
        return list(self._link_as_parent_index.get(link_name, []))

    def get_parent_link(self, link_name: str) -> Link | None:
        """Get the parent link of the specified link.

        Args:
            link_name: Name of the child link.

        Returns:
            The parent Link object if found, otherwise None.
        """
        joint = self.get_parent_joint(link_name)
        return self.get_link(joint.parent) if joint else None

    def get_child_links(self, link_name: str) -> list[Link]:
        """Get all immediate child links of the specified link.

        Args:
            link_name: Name of the parent link.

        Returns:
            List of child Link objects.
        """
        return [self.link(j.child) for j in self.get_child_joints(link_name)]

    def get_root_link(self) -> Link:
        """Get the root link of the kinematic tree.

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
                f"Duplicate sensor name: '{sensor.name}'",
                target="Sensor",
                value=sensor.name,
            )

        # Validate that the link exists
        if sensor.link_name not in self._link_index:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Link '{sensor.link_name}' not found",
                target="LinkName",
                value=sensor.link_name,
            )

        self.sensors = (*self.sensors, sensor)
        self._sensor_index[sensor.name] = sensor

    def get_sensor(self, name: str) -> Sensor | None:
        """Retrieve a sensor by name.

        Args:
            name: The name of the sensor to find.

        Returns:
            The Sensor object if found, otherwise None.
        """
        return self._sensor_index.get(name)

    def sensor(self, name: str) -> Sensor:
        """Retrieve a sensor by name, raising an error if it does not exist.

        Args:
            name: The name of the sensor to find.

        Returns:
            The Sensor object.

        Raises:
            RobotValidationError: If the sensor is not found.
        """
        obj = self.get_sensor(name)
        if obj is None:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Sensor '{name}' not found in robot '{self.name}'",
                target="Sensor",
                value=name,
            )
        return obj

    def has_sensor(self, name: str) -> bool:
        """Check if a sensor with the given name exists.

        Args:
            name: The name of the sensor to check.

        Returns:
            True if the sensor exists, False otherwise.
        """
        return name in self._sensor_index

    def add_transmission(self, transmission: Transmission) -> None:
        """Define a mechanical transmission for one or more joints.

        Args:
            transmission: The Transmission definition to add.

        Raises:
            RobotValidationError: If the transmission name is a duplicate
                or referenced joints do not exist.
        """
        if transmission.name in self._transmission_index:
            raise RobotValidationError(
                ValidationErrorCode.DUPLICATE_NAME,
                f"Duplicate transmission name: '{transmission.name}'",
                target="Transmission",
                value=transmission.name,
            )

        # Validate that all referenced joints exist
        for trans_joint in transmission.joints:
            if trans_joint.name not in self._joint_index:
                raise RobotValidationError(
                    ValidationErrorCode.NOT_FOUND,
                    f"Joint '{trans_joint.name}' not found",
                    target="JointName",
                    value=trans_joint.name,
                )

        self.transmissions = (*self.transmissions, transmission)
        self._transmission_index[transmission.name] = transmission

    def get_transmission(self, name: str) -> Transmission | None:
        """Retrieve a transmission by name.

        Args:
            name: The name of the transmission to find.

        Returns:
            The Transmission object if found, otherwise None.
        """
        return self._transmission_index.get(name)

    def transmission(self, name: str) -> Transmission:
        """Retrieve a transmission by name, raising an error if it does not exist.

        Args:
            name: The name of the transmission to find.

        Returns:
            The Transmission object.

        Raises:
            RobotValidationError: If the transmission is not found.
        """
        obj = self.get_transmission(name)
        if obj is None:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"Transmission '{name}' not found in robot '{self.name}'",
                target="Transmission",
                value=name,
            )
        return obj

    def has_transmission(self, name: str) -> bool:
        """Check if a transmission with the given name exists.

        Args:
            name: The name of the transmission to check.

        Returns:
            True if the transmission exists, False otherwise.
        """
        return name in self._transmission_index

    def add_ros2_control(self, ros2_control: Ros2Control) -> None:
        """Register a ros2_control hardware system.

        Args:
            ros2_control: The hardware configuration to add.

        Raises:
            RobotValidationError: If the configuration name is a duplicate.
        """
        # Check for duplicate names
        if ros2_control.name in self._ros2_control_index:
            raise RobotValidationError(
                ValidationErrorCode.DUPLICATE_NAME,
                f"Duplicate ros2_control name: '{ros2_control.name}'",
                target="Ros2Control",
                value=ros2_control.name,
            )

        # Validate that all referenced joints exist
        for ctrl_joint in ros2_control.joints:
            if ctrl_joint.name not in self._joint_index:
                raise RobotValidationError(
                    ValidationErrorCode.NOT_FOUND,
                    f"Not found Joint '{ctrl_joint.name}'",
                    target="JointName",
                    value=ctrl_joint.name,
                )

        self.ros2_controls = (*self.ros2_controls, ros2_control)
        self._ros2_control_index[ros2_control.name] = ros2_control

    def update_ros2_control(self, ros2_control: Ros2Control) -> None:
        """Update an existing ros2_control configuration.

        Args:
            ros2_control: The updated Ros2Control configuration.
        """
        if ros2_control.name not in self._ros2_control_index:
            self.add_ros2_control(ros2_control)
            return

        self.ros2_controls = tuple(
            ros2_control if ctrl.name == ros2_control.name else ctrl for ctrl in self.ros2_controls
        )
        self._ros2_control_index[ros2_control.name] = ros2_control

    def get_ros2_control(self, name: str) -> Ros2Control | None:
        """Retrieve a ROS2 Control configuration by name.

        Args:
            name: The name of the configuration to find.

        Returns:
            The Ros2Control object if found, otherwise None.
        """
        return self._ros2_control_index.get(name)

    def ros2_control(self, name: str) -> Ros2Control:
        """Retrieve a ROS2 Control configuration by name, raising an error if it does not exist.

        Args:
            name: The name of the configuration to find.

        Returns:
            The Ros2Control object.

        Raises:
            RobotValidationError: If the configuration is not found.
        """
        obj = self.get_ros2_control(name)
        if obj is None:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                f"ROS2 control '{name}' not found in robot '{self.name}'",
                target="Ros2Control",
                value=name,
            )
        return obj

    def has_ros2_control(self, name: str) -> bool:
        """Check if a ROS2 Control configuration with the given name exists.

        Args:
            name: The name of the configuration to check.

        Returns:
            True if it exists, False otherwise.
        """
        return name in self._ros2_control_index

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
                f"Not found Gazebo reference '{element.reference}'",
                target="GazeboReference",
                value=element.reference,
            )

        self.gazebo_elements = (*self.gazebo_elements, element)

    def get_gazebo_elements(self, reference: str | None = None) -> list[GazeboElement]:
        """Get Gazebo elements, optionally filtered by reference.

        Args:
            reference: Optional name of the link/joint to filter by.

        Returns:
            List of matching GazeboElement objects.
        """
        if reference is None:
            return list(self.gazebo_elements)
        return [ge for ge in self.gazebo_elements if ge.reference == reference]

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
            chains: List of kinematic `Chain` objects.
            subgroups: List of other planning group names to include.
            base_link: Shorthand for a single chain base link.
            tip_link: Shorthand for a single chain tip link.

        Returns:
            The robot instance for chaining.

        Raises:
            RobotValidationError: If any referenced link, joint, or subgroup does not exist.
        """
        # Validate that all referenced elements exist
        if links:
            for link_name in links:
                self.link(link_name)
        if joints:
            for j in joints:
                self.joint(j)
        if chains:
            for c in chains:
                self.link(c.base_link)
                self.link(c.tip_link)
        if base_link and tip_link:
            self.link(base_link)
            self.link(tip_link)

        # Note: subgroups validation is deferred to final export or graph check
        # as the subgroups might not have been added yet in the chain.

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
        self.semantic = replace(self.semantic, groups=tuple(self.semantic.groups) + (group,))
        return self

    def disable_collisions(self, link1: str, link2: str, reason: str = "Adjacent") -> Robot:
        """Disable collision checking between two links.

        Args:
            link1: First link name.
            link2: Second link name.
            reason: Reason for disabling (default: 'Adjacent').

        Returns:
            The robot instance for chaining.

        Raises:
            RobotValidationError: If link1 or link2 is not found.
        """
        self.link(link1)
        self.link(link2)

        # Disable collisions
        dc = CollisionPair(link1=link1, link2=link2, reason=reason)
        self.semantic = replace(
            self.semantic,
            disabled_collisions=tuple(self.semantic.disabled_collisions) + (dc,),
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

        Raises:
            RobotValidationError: If link1 or link2 is not found.
        """
        self.link(link1)
        self.link(link2)

        ec = CollisionPair(link1=link1, link2=link2, reason=reason)
        self.semantic = replace(
            self.semantic,
            enabled_collisions=tuple(self.semantic.enabled_collisions) + (ec,),
        )
        return self

    def disable_default_collisions(self, link: str) -> Robot:
        """Disable all default collisions for a specific link.

        Args:
            link: Link name.

        Returns:
            The robot instance for chaining.

        Raises:
            RobotValidationError: If the link is not found.
        """
        self.link(link)

        self.semantic = replace(
            self.semantic,
            no_default_collision_links=tuple(self.semantic.no_default_collision_links) + (link,),
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

        Raises:
            RobotValidationError: If the joint is not found.
        """
        self.joint(joint_name)

        jp = JointProperty(joint_name=joint_name, property_name=property_name, value=value)
        self.semantic = replace(
            self.semantic,
            joint_properties=tuple(self.semantic.joint_properties) + (jp,),
        )
        return self

    def approximate_link_collision(self, link: str, spheres: list[SrdfSphere]) -> Robot:
        """Add sphere-based collision approximation for a link.

        Args:
            link: Name of the link.
            spheres: List of SrdfSphere objects.

        Returns:
            The robot instance for chaining.

        Raises:
            RobotValidationError: If the link is not found.
        """
        self.link(link)

        lsa = LinkSphereApproximation(link=link, spheres=tuple(spheres))
        self.semantic = replace(
            self.semantic,
            link_sphere_approximations=tuple(self.semantic.link_sphere_approximations) + (lsa,),
        )
        return self

    @property
    def graph(self) -> KinematicGraph:
        """Get the formal kinematic graph representing the robot's structure.

        This is built on demand (and cached) to ensure it reflects the current state
        of links and joints with optimal performance.
        """
        if self._graph_cache is None:
            self._graph_cache = KinematicGraph(self.links, self.joints)
        return self._graph_cache

    @property
    def root_link(self) -> Link:
        """Get the root link of the kinematic tree.

        The root link is the one that is never a child in any joint.
        """
        return self.get_root_link()

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
        """Return a lightweight human-readable summary of the robot."""
        return f"Robot(name={self.name}, links={len(self.links)}, joints={len(self.joints)}, dof={self.degrees_of_freedom})"

    def summary(self) -> str:
        """Return a detailed architectural summary and validity status.

        This method performs a deep kinematic analysis, which may be
        computationally expensive on first call as it builds the graph.
        """
        try:
            roots = self.graph.get_root_links()
            root_name = roots[0] if len(roots) == 1 else f"{len(roots)} roots"
            is_valid = len(roots) == 1 and not self.has_cycle
        except Exception:
            root_name = "Unknown"
            is_valid = False

        parts = [
            f"Robot Summary: {self.name}",
            f"  Status: {'VALID' if is_valid else 'INVALID'}",
            f"  Root: {root_name}",
            f"  Topology: {len(self.links)} links, {len(self.joints)} joints ({self.degrees_of_freedom} DOF)",
        ]

        if self.sensors:
            parts.append(
                f"  Functional: {len(self.sensors)} sensors, {len(self.transmissions)} transmissions"
            )
        if self.ros2_controls:
            parts.append(f"  Hardware: {len(self.ros2_controls)} ros2_control blocks")
        if self.gazebo_elements:
            parts.append(f"  Simulation: {len(self.gazebo_elements)} gazebo tags")

        return "\n".join(parts)

    def resolve_resource(self, uri: str, relative_to: Path | None = None) -> Path:
        """Resolve a resource URI using the robot's configured resolver.

        Args:
            uri: The resource URI to resolve (e.g. mesh path, package://).
            relative_to: Optional base directory for relative path resolution.

        Returns:
            The resolved absolute Path.
        """
        return self.resource_resolver.resolve(uri, relative_to=relative_to)

    def _reindex(self) -> None:
        """Rebuild internal lookup indices and clear cache.

        This is an internal maintenance method that ensures the O(1)
        lookup maps stay in sync with the field-based storage.
        """
        # Enforce tuple immutability for core collections
        self.links = tuple(self.links)
        self.joints = tuple(self.joints)
        self.sensors = tuple(self.sensors)
        self.transmissions = tuple(self.transmissions)
        self.ros2_controls = tuple(self.ros2_controls)
        self.gazebo_elements = tuple(self.gazebo_elements)
        # Reset indices with duplicate validation
        self._link_index = {}
        for link in self.links:
            if link.name in self._link_index:
                raise RobotValidationError(
                    ValidationErrorCode.DUPLICATE_NAME,
                    f"Duplicate link name: '{link.name}'",
                    target="Link",
                    value=link.name,
                )
            self._link_index[link.name] = link

        self._joint_index = {}
        for joint in self.joints:
            if joint.name in self._joint_index:
                raise RobotValidationError(
                    ValidationErrorCode.DUPLICATE_NAME,
                    f"Duplicate joint name: '{joint.name}'",
                    target="Joint",
                    value=joint.name,
                )
            self._joint_index[joint.name] = joint

        self._sensor_index = {s.name: s for s in self.sensors}
        self._transmission_index = {t.name: t for t in self.transmissions}
        self._ros2_control_index = {rc.name: rc for rc in self.ros2_controls}

        # Rebuild adjacency maps for fast traversal
        self._link_as_parent_index = defaultdict(list)
        self._link_as_child_index = defaultdict(list)
        for joint in self.joints:
            self._link_as_parent_index[joint.parent].append(joint)
            self._link_as_child_index[joint.child].append(joint)

        # Clear caches
        self._graph_cache = None
