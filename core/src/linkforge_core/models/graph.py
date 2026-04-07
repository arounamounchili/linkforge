"""Kinematic graph management for robot structures.

This module provides formal graph theory logic for validating and
traversing the link-joint structure of a robot.
"""

from __future__ import annotations

import collections
from collections.abc import Iterable
from typing import TYPE_CHECKING

from ..exceptions import RobotValidationError, ValidationErrorCode

if TYPE_CHECKING:
    from .joint import Joint
    from .link import Link


class KinematicGraph:
    """Robot connectivity model for cycle detection and topological sorting.

    Decouples graph logic from the main Robot model.
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
        """Detect kinematic loops using iterative DFS stability."""
        if not self.joints:
            return False

        visited: set[str] = set()
        rec_stack: set[str] = set()

        for start_node in self.link_names:
            if start_node in visited:
                continue

            # Stack contains: (node, child_idx, is_backtracking)
            stack: list[tuple[str, int, bool]] = [(start_node, 0, False)]

            while stack:
                node, child_idx, backtracking = stack.pop()

                if backtracking:
                    rec_stack.discard(node)
                    continue

                if child_idx == 0:
                    visited.add(node)
                    rec_stack.add(node)
                    stack.append((node, 0, True))

                children = self.adj.get(node, [])
                if child_idx < len(children):
                    child_name, _ = children[child_idx]
                    # Push next sibling
                    stack.append((node, child_idx + 1, False))

                    if child_name not in visited:
                        stack.append((child_name, 0, False))
                    elif child_name in rec_stack:
                        return True
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

    def get_topological_order(self) -> list[str]:
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
        # Implement Kahn's algorithm for topological sorting.
        # This provides a level-by-level ordering useful for recursive solvers.
        in_degree = {name: len(self.inv_adj.get(name, [])) for name in self.link_names}
        queue = [name for name, deg in in_degree.items() if deg == 0]

        while queue:
            # Sort for determinism
            queue.sort()
            curr = queue.pop(0)
            order.append(curr)

            for child, _ in self.adj.get(curr, []):
                in_degree[child] -= 1
                if in_degree[child] == 0:
                    queue.append(child)

        return order
