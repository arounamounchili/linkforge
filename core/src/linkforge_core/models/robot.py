"""Robot model representing a complete robot description.

This module provides the central `Robot` class and related data structures
that form the internal representation of a robot in LinkForge. It acts
as the source of truth for all generators and parsers, encapsulating:

- **Kinematic Tree**: Hierarchy of links connected by various joint types.
- **Physical Properties**: Aggregate mass, inertia tensors, and center of mass data.
- **Visual & Collision Geometry**: Support for multiple primitives and mesh assets.
- **Actuation & Sensing**: Transmissions, hardware interfaces, and sensor configurations.
- **Simulator Metadata**: Gazebo-specific properties and plugin definitions.

The models are designed to be simulator-agnostic while providing enough
fidelity to generate highly optimized URDF/XACRO for ROS-ready environments.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Sequence
from dataclasses import InitVar, dataclass, field
from typing import Any

from ..utils.string_utils import is_valid_urdf_name
from .gazebo import GazeboElement
from .joint import Joint
from .link import Link
from .ros2_control import Ros2Control
from .sensor import Sensor
from .transmission import Transmission


@dataclass
class Robot:
    """Complete robot description.

    A robot consists of links connected by joints, forming a kinematic tree.
    May also include sensors, transmissions, ros2_control, and Gazebo-specific elements.

    Performance Note:
        Link and joint lookups use O(1) hash map indices for fast access,
        which is critical for large robots with 100+ components.
    """

    name: str
    version: str = "1.1"  # LinkForge IR Version
    sensors: list[Sensor] = field(default_factory=list)
    transmissions: list[Transmission] = field(default_factory=list)
    ros2_controls: list[Ros2Control] = field(default_factory=list)
    gazebo_elements: list[GazeboElement] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)

    # Internal storage
    _links: list[Link] = field(default_factory=list, init=False)
    _joints: list[Joint] = field(default_factory=list, init=False)

    # Fast lookup indices (name -> object)
    _link_index: dict[str, Link] = field(default_factory=dict, init=False)
    _joint_index: dict[str, Joint] = field(default_factory=dict, init=False)
    _sensor_index: dict[str, Sensor] = field(default_factory=dict, init=False, repr=False)

    # Init args
    initial_links: InitVar[Sequence[Link] | None] = None
    initial_joints: InitVar[Sequence[Joint] | None] = None

    def __post_init__(
        self,
        initial_links: Sequence[Link] | None,
        initial_joints: Sequence[Joint] | None,
    ) -> None:
        """Initialize indices and validate structure."""
        if not self.name:
            raise ValueError("Robot name cannot be empty")

        # Validate naming convention
        if not is_valid_urdf_name(self.name):
            raise ValueError(
                f"Robot name '{self.name}' contains invalid characters. "
                "Use only alphanumeric, underscore, or hyphen."
            )

        # Initialize storage
        if initial_links:
            self._links.extend(initial_links)
        if initial_joints:
            self._joints.extend(initial_joints)

        # Build indices
        self._link_index = {}
        for link in self._links:
            if link.name in self._link_index:
                raise ValueError(f"Duplicate link name: {link.name}")
            self._link_index[link.name] = link

        self._joint_index = {}
        for joint in self._joints:
            if joint.name in self._joint_index:
                raise ValueError(f"Duplicate joint name: {joint.name}")
            self._joint_index[joint.name] = joint

        self._sensor_index = {sensor.name: sensor for sensor in self.sensors}

    def add_link(self, link: Link) -> None:
        """Add a link to the robot and update indices."""
        if link.name in self._link_index:
            raise ValueError(f"Link '{link.name}' already exists")
        self._links.append(link)
        self._link_index[link.name] = link

    def add_joint(self, joint: Joint) -> None:
        """Add a joint to the robot and update indices."""
        if joint.name in self._joint_index:
            raise ValueError(f"Joint '{joint.name}' already exists")

        # Validate parent and child links exist
        if joint.parent not in self._link_index:
            raise ValueError(f"Parent link '{joint.parent}' not found")
        if joint.child not in self._link_index:
            raise ValueError(f"Child link '{joint.child}' not found")

        self._joints.append(joint)
        self._joint_index[joint.name] = joint

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
            List of matching joints

        """
        if as_parent:
            return [joint for joint in self.joints if joint.parent == link_name]
        else:
            return [joint for joint in self.joints if joint.child == link_name]

    def add_sensor(self, sensor: Sensor) -> None:
        """Add a sensor to the robot and update indices."""
        if sensor.name in self._sensor_index:
            raise ValueError(f"Sensor '{sensor.name}' already exists")

        # Validate that the link exists
        if sensor.link_name not in self._link_index:
            raise ValueError(f"Sensor '{sensor.name}': link '{sensor.link_name}' not found")

        self.sensors.append(sensor)
        self._sensor_index[sensor.name] = sensor

    def add_transmission(self, transmission: Transmission) -> None:
        """Add a transmission to the robot."""
        if any(t.name == transmission.name for t in self.transmissions):
            raise ValueError(f"Transmission '{transmission.name}' already exists")

        # Validate that all referenced joints exist
        for trans_joint in transmission.joints:
            if trans_joint.name not in self._joint_index:
                raise ValueError(
                    f"Transmission '{transmission.name}': joint '{trans_joint.name}' not found"
                )

        self.transmissions.append(transmission)

    def add_gazebo_element(self, element: GazeboElement) -> None:
        """Add a Gazebo element to the robot."""
        # Validate reference if specified
        if (
            element.reference is not None
            and self.get_link(element.reference) is None
            and self.get_joint(element.reference) is None
        ):
            raise ValueError(
                f"Gazebo element reference '{element.reference}' does not match any link or joint"
            )

        self.gazebo_elements.append(element)

    def add_ros2_control(self, ros2_control: Ros2Control) -> None:
        """Add a ROS2 Control configuration to the robot."""
        # Check for duplicate names
        if any(rc.name == ros2_control.name for rc in self.ros2_controls):
            raise ValueError(f"ROS2 Control '{ros2_control.name}' already exists")

        self.ros2_controls.append(ros2_control)

    def get_root_link(self) -> Link | None:
        """Get the root link of the kinematic tree.

        The root link is the one that is never a child in any joint.
        """
        if not self.links:
            return None

        child_links = {joint.child for joint in self.joints}
        root_links = [link for link in self.links if link.name not in child_links]

        if len(root_links) == 0:
            raise ValueError("No root link found (circular dependency detected)")
        if len(root_links) > 1:
            raise ValueError(f"Multiple root links found: {[link.name for link in root_links]}")

        return root_links[0]

    def validate_tree_structure(self) -> list[str]:
        """Validate the kinematic tree structure.

        Returns:
            List of error messages (empty if valid)

        """
        errors = []

        # Must have at least one link
        if not self.links:
            errors.append("Robot must have at least one link")
            return errors

        # Check for duplicate link names
        link_names = [link.name for link in self.links]
        duplicates = {name for name, count in Counter(link_names).items() if count > 1}
        if duplicates:
            errors.append(f"Duplicate link names: {duplicates}")

        # Check for duplicate joint names
        joint_names = [joint.name for joint in self.joints]
        duplicates = {name for name, count in Counter(joint_names).items() if count > 1}
        if duplicates:
            errors.append(f"Duplicate joint names: {duplicates}")

        # Check that all joint parent/child links exist
        link_name_set = set(link_names)
        for joint in self.joints:
            if joint.parent not in link_name_set:
                errors.append(f"Joint '{joint.name}': parent link '{joint.parent}' not found")
            if joint.child not in link_name_set:
                errors.append(f"Joint '{joint.name}': child link '{joint.child}' not found")

        # Check for cycles in the kinematic tree
        if self._has_cycle():
            errors.append("Kinematic tree contains a cycle")

        # Check for circular mimic dependencies
        mimic_errors = self._validate_mimic_chains()
        errors.extend(mimic_errors)

        # Check that there's exactly one root link
        root = None
        try:
            root = self.get_root_link()
            if root is None:
                errors.append("No root link found")
        except ValueError as e:
            errors.append(str(e))

        # Check that each link (except root) is a child in exactly one joint
        child_counts: dict[str, int] = {}
        for joint in self.joints:
            child_counts[joint.child] = child_counts.get(joint.child, 0) + 1

        for link in self.links:
            count = child_counts.get(link.name, 0)
            # Only check connectivity if we have a valid root
            if root is not None and link != root and count == 0:
                errors.append(f"Link '{link.name}' is not connected to the tree")
            elif count > 1:
                errors.append(f"Link '{link.name}' has {count} parent joints (should have 1)")

        return errors

    def _has_cycle(self) -> bool:
        """Check if the kinematic tree has a cycle using iterative DFS.

        Uses an iterative approach to avoid stack overflow on large robot models.
        This prevents RecursionError on robots with 1000+ joints.

        Returns:
            True if a cycle is detected, False otherwise
        """
        if not self.joints:
            return False

        # Build adjacency list
        graph: dict[str, list[str]] = {}
        for joint in self.joints:
            if joint.parent not in graph:
                graph[joint.parent] = []
            graph[joint.parent].append(joint.child)

        visited: set[str] = set()
        rec_stack: set[str] = set()

        def iterative_dfs(start: str) -> bool:
            """Iterative DFS with explicit stack to detect cycles."""
            # Stack contains: (node, children_iterator, is_backtracking)
            stack: list[tuple[str, int, bool]] = [(start, 0, False)]

            while stack:
                node, child_idx, backtracking = stack.pop()

                if backtracking:
                    # Backtracking: remove from recursion stack
                    rec_stack.discard(node)
                    continue

                # First visit to this node
                if child_idx == 0:
                    visited.add(node)
                    rec_stack.add(node)
                    # Push backtrack marker
                    stack.append((node, 0, True))

                # Process children
                if node in graph:
                    children = graph[node]
                    if child_idx < len(children):
                        neighbor = children[child_idx]
                        # Push next sibling
                        stack.append((node, child_idx + 1, False))
                        # Push child if not visited
                        if neighbor not in visited:
                            stack.append((neighbor, 0, False))
                        elif neighbor in rec_stack:
                            # Back edge detected - cycle!
                            return True

            return False

        # Check from all unvisited nodes
        return any(link.name not in visited and iterative_dfs(link.name) for link in self.links)

    def _validate_mimic_chains(self) -> list[str]:
        """Validate that mimic joints don't form circular dependencies.

        Returns:
            List of error messages (empty if valid)

        Example circular dependency:
            Joint A mimics Joint B
            Joint B mimics Joint C
            Joint C mimics Joint A  <- Circular!
        """
        errors = []

        # Use existing joint index for O(1) lookup
        joint_map = self._joint_index

        for joint in self.joints:
            if joint.mimic is None:
                continue

            # Follow the mimic chain to detect cycles
            visited = {joint.name}
            current = joint.mimic.joint

            # Traverse the mimic chain
            while current:
                # Check if mimic target exists
                if current not in joint_map:
                    errors.append(f"Joint '{joint.name}' mimics non-existent joint '{current}'")
                    break

                # Check for circular dependency
                if current in visited:
                    chain = " -> ".join(visited) + f" -> {current}"
                    errors.append(f"Circular mimic dependency detected: {chain}")
                    break

                visited.add(current)

                # Move to next joint in chain
                next_joint = joint_map[current]
                if next_joint.mimic is None:
                    break
                current = next_joint.mimic.joint

        return errors

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

    def __str__(self) -> str:
        """String representation."""
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
