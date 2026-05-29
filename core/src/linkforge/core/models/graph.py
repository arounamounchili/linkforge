"""Kinematic graph management for robot structures.

This module provides formal graph theory logic for validating and traversing
the link-joint structure of a robot. It ensures the model forms a valid tree
or forest (collection of trees) suitable for simulation and control.

Core Functionality:
- **Topology**: Detects cycles and identifies root links.
- **Traversal**: Provides topological sorting for links and joints.
- **Isolation**: Detects disconnected kinematic islands.
"""

from __future__ import annotations

import collections
import heapq
from collections.abc import Iterable
from typing import TYPE_CHECKING

from ..exceptions import RobotValidationError, ValidationErrorCode

if TYPE_CHECKING:
    from .joint import Joint
    from .link import Link


class KinematicGraph:
    """Robot connectivity model for cycle detection and topological sorting.

    This class decouples the graph-theoretical concerns (islands, cycles, roots)
    from the main Robot data model, enabling efficient structural validation.
    """

    def __init__(self, links: Iterable[Link], joints: Iterable[Joint]) -> None:
        """Initialize the kinematic graph.

        Args:
            links: Collection of Link objects forming the nodes of the graph.
            joints: Collection of Joint objects forming the edges of the graph.

        Raises:
            RobotValidationError: If a joint references a link not present in the links collection.
        """
        self.link_names = {link.name for link in links}
        self.joints = list(joints)

        # Build adjacency list: parent -> list of (child, joint_name)
        self.adj: dict[str, list[tuple[str, str]]] = collections.defaultdict(list)
        # Inverse adjacency: child -> list of (parent, joint_name)
        self.inv_adj: dict[str, list[tuple[str, str]]] = collections.defaultdict(list)

        for joint in self.joints:
            if joint.parent not in self.link_names:
                raise RobotValidationError(
                    ValidationErrorCode.NOT_FOUND,
                    f"Parent link '{joint.parent}' unknown",
                    target="ParentLink",
                    value=joint.parent,
                )
            if joint.child not in self.link_names:
                raise RobotValidationError(
                    ValidationErrorCode.NOT_FOUND,
                    f"Child link '{joint.child}' unknown",
                    target="ChildLink",
                    value=joint.child,
                )

            self.adj[joint.parent].append((joint.child, joint.name))
            self.inv_adj[joint.child].append((joint.parent, joint.name))

    def has_cycle(self) -> bool:
        """Detect kinematic loops using iterative DFS with WHITE/GREY/BLACK colouring.

        Kinematic cycles are generally illegal in URDF and many physics solvers
        unless explicitly handled by parallel linkage plugins.

        Uses three-colour marking to identify back-edges in the directed graph:
        - WHITE (0): not yet visited
        - GREY  (1): currently in the DFS recursion stack
        - BLACK (2): fully processed, all descendants explored

        A back-edge (node pointing to a GREY ancestor) indicates a cycle.
        The iterative stack avoids Python's default recursion limit.
        """
        if not self.joints:
            return False

        white, grey, black = 0, 1, 2
        colour: dict[str, int] = dict.fromkeys(self.link_names, white)

        for start in self.link_names:
            if colour[start] != white:
                continue

            # Each stack entry: (node, child_iterator, entered_grey)
            # We push a sentinel "backtrack" entry alongside the children.
            stack: list[tuple[str, int]] = [(start, 0)]
            colour[start] = grey

            while stack:
                node, child_idx = stack[-1]
                children = self.adj.get(node, [])

                if child_idx < len(children):
                    # Advance to next child
                    stack[-1] = (node, child_idx + 1)
                    child_name, _ = children[child_idx]

                    if colour[child_name] == grey:
                        # Back-edge found → cycle exists
                        return True
                    if colour[child_name] == white:
                        colour[child_name] = grey
                        stack.append((child_name, 0))
                else:
                    # All children processed → mark BLACK and backtrack
                    colour[node] = black
                    stack.pop()

        return False

    def get_root_links(self) -> list[str]:
        """Identify all potential root links in the robot structure.

        A root link is defined as a link that has outgoing joint edges (is a parent)
        but no incoming joint edges (is never a child).

        Returns:
            Alphabetically sorted list of root link names.
        """
        # Ensure we are not dealing with None
        if not self.link_names:
            return []

        # Root links have no incoming joint edges (empty inv_adj)
        roots = [name for name in self.link_names if not self.inv_adj.get(name)]
        return sorted(roots)

    def find_islands(self) -> list[set[str]]:
        """Identify disconnected components (islands) in the robot structure.

        Uses BFS traversal to find all linked components. Returns isolated links
        as single-node islands.
        """
        remaining = set(self.link_names)
        islands: list[set[str]] = []

        while remaining:
            start = next(iter(remaining))
            island: set[str] = set()
            queue = collections.deque([start])
            visited = {start}

            while queue:
                curr = queue.popleft()
                island.add(curr)
                # Traverse both directions (undirected connectivity)
                neighbors = [c for c, _ in self.adj.get(curr, [])] + [
                    p for p, _ in self.inv_adj.get(curr, [])
                ]
                for n in neighbors:
                    if n not in visited:
                        visited.add(n)
                        queue.append(n)

            islands.append(island)
            remaining -= island

        return islands

    def get_topological_link_names(self) -> list[str]:
        """Return links in topological order (parents before children).

        Returns:
            List of link names

        Raises:
            RobotValidationError: If a cycle is detected
        """
        if self.has_cycle():
            raise RobotValidationError(
                ValidationErrorCode.HAS_CYCLE,
                "Kinematic graph contains cycles",
                target="CyclicGraph",
            )

        order: list[str] = []
        # Implement Kahn's algorithm for topological sorting using a min-heap.
        # heapq gives O(n log n) overall instead of O(n² log n) from list.sort() + pop(0).
        # This provides a level-by-level ordering useful for recursive solvers,
        # and the heap's natural ordering guarantees alphabetical determinism.
        in_degree = {name: len(self.inv_adj.get(name, [])) for name in self.link_names}
        heap = [name for name, deg in in_degree.items() if deg == 0]
        heapq.heapify(heap)

        while heap:
            curr = heapq.heappop(heap)
            order.append(curr)

            for child, _ in self.adj.get(curr, []):
                in_degree[child] -= 1
                if in_degree[child] == 0:
                    heapq.heappush(heap, child)

        return order

    def get_topological_joints(self) -> list[Joint]:
        """Return joints in topological order (parents before children).

        This ensures that when building a hierarchy, the parent structure
        always exists before the child is attached.

        Returns:
            Sorted list of Joint models

        Raises:
            RobotValidationError: If a cycle is detected
        """
        if self.has_cycle():
            raise RobotValidationError(
                ValidationErrorCode.HAS_CYCLE,
                "Kinematic graph contains cycles",
                target="CyclicGraph",
            )

        # Build a map for O(1) joint lookup during traversal
        joint_registry = {j.name: j for j in self.joints}

        root_links = self.get_root_links()
        sorted_joints: list[Joint] = []
        visited: set[str] = set()

        def visit(link_name: str) -> None:
            if link_name in visited:
                return
            visited.add(link_name)

            # Traverse child joints
            if link_name in self.adj:
                # Iterate through outgoing edges (child_name, joint_name)
                for child_name, joint_name in self.adj[link_name]:
                    joint = joint_registry.get(joint_name)
                    if joint:
                        sorted_joints.append(joint)
                        visit(child_name)

        for root in root_links:
            visit(root)

        return sorted_joints
