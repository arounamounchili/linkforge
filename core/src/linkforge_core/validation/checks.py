"""Modular validation checks for robot models.

Each class implements a single, focused validation rule following the
Single Responsibility Principle. Checks are designed to be composable:
they write errors and warnings directly into a shared ``ValidationResult``,
allowing the caller to run any subset of checks independently.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from ..exceptions import RobotModelError, RobotValidationError, ValidationErrorCode
from .result import ValidationResult

if TYPE_CHECKING:
    from ..models.link import Link
    from ..models.robot import Robot


class ValidationCheck(ABC):
    """Abstract base class for a single, focused validation rule.

    All concrete checks must implement :meth:`run`. Checks are stateless
    by design — all output is written into the provided ``ValidationResult``.
    """

    @abstractmethod
    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Apply this check, writing errors and warnings into ``result``.

        Args:
            robot: The robot model to validate.
            result: The shared result object to append errors/warnings to.
        """
        ...  # pragma: no cover


class HasLinksCheck(ValidationCheck):
    """Check that the robot has at least one link."""

    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Check that robot has at least one link."""
        if not robot.links:
            result.add_error(
                title="No links",
                message="Robot must have at least one link",
                suggestion="Add a link by marking an object as a robot link in the Link panel",
            )


class DuplicateNameCheck(ValidationCheck):
    """Check for duplicate link and joint names."""

    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Check for duplicate link and joint names."""
        self._check_duplicates(
            names=[link.name for link in robot.links],
            kind="link",
            result=result,
        )
        self._check_duplicates(
            names=[joint.name for joint in robot.joints],
            kind="joint",
            result=result,
        )

    @staticmethod
    def _check_duplicates(names: list[str], kind: str, result: ValidationResult) -> None:
        seen: set[str] = set()
        for name in names:
            if name in seen:
                result.add_error(
                    title=f"Duplicate {kind} name",
                    message=(
                        f"{kind.capitalize()} name '{name}' is used by "
                        f"{names.count(name)} {kind}s. Each {kind} must have a unique name"
                    ),
                    affected_objects=[n for n in names if n == name],
                    suggestion=f"Rename duplicate {kind}s to unique names (e.g., '{name}_1', '{name}_2')",
                )
                return  # Report once per kind
            seen.add(name)


class JointReferenceCheck(ValidationCheck):
    """Check that all joints reference existing links."""

    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Check that all joints reference valid links."""
        link_names = {link.name for link in robot.links}

        for joint in robot.joints:
            if joint.parent not in link_names:
                result.add_error(
                    title="Missing parent link",
                    message=(
                        f"Joint '{joint.name}' references parent link "
                        f"'{joint.parent}' which does not exist"
                    ),
                    affected_objects=[joint.name],
                    suggestion=f"Create a link named '{joint.parent}' or update the joint's parent reference",
                )

            if joint.child not in link_names:
                result.add_error(
                    title="Missing child link",
                    message=(
                        f"Joint '{joint.name}' references child link "
                        f"'{joint.child}' which does not exist"
                    ),
                    affected_objects=[joint.name],
                    suggestion=f"Create a link named '{joint.child}' or update the joint's child reference",
                )


class TreeStructureCheck(ValidationCheck):
    """Check kinematic tree integrity: cycles, root link, and connectivity."""

    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Check kinematic tree structure."""
        if not robot.links:
            return  # Already reported by HasLinksCheck

        self._check_cycles(robot, result)
        root = self._check_root(robot, result)
        if root is not None:
            self._check_connectivity(robot, root, result)

    @staticmethod
    def _check_cycles(robot: Robot, result: ValidationResult) -> None:
        try:
            if robot._has_cycle():
                result.add_error(
                    title="Circular dependency",
                    message=(
                        "Kinematic tree contains a cycle. "
                        "Links must form a tree structure, not a loop."
                    ),
                    suggestion="Review joint connections to ensure they form a tree (no loops)",
                )
        except RobotModelError as e:
            has_ref_errors = any(
                err.title in ("Missing parent link", "Missing child link") for err in result.errors
            )
            if not has_ref_errors:
                result.add_error(
                    title="Kinematic graph error",
                    message=str(e),
                    suggestion="Check joint and link consistency",
                )

    @staticmethod
    def _check_root(robot: Robot, result: ValidationResult) -> Link | None:
        """Return the root link, or None if it cannot be determined."""
        try:
            root = robot.get_root_link()
            if root is None:
                result.add_error(
                    title="No root link",
                    message=(
                        "No root link found. A robot must have exactly one link "
                        "that is not a child in any joint."
                    ),
                    suggestion="Ensure exactly one link has no parent joint (the base/root link)",
                )
            return root
        except RobotModelError as e:
            if isinstance(e, RobotValidationError):
                if e.code == ValidationErrorCode.MULTIPLE_ROOTS:
                    result.add_error(
                        title="Multiple root links",
                        message=str(e),
                        suggestion="Ensure only one link has no parent joint. Connect other root links to the tree with joints",
                    )
                elif e.code == ValidationErrorCode.NO_ROOT:
                    result.add_error(
                        title="No root link",
                        message=str(e),
                        suggestion="Ensure exactly one link has no parent joint (the base/root link)",
                    )
                else:
                    result.add_error(
                        title="Root link error",
                        message=str(e),
                        suggestion="Check the joint connections in your robot tree",
                    )
            else:
                result.add_error(
                    title="Root link error",
                    message=str(e),
                    suggestion="Check the joint connections in your robot tree",
                )
            return None

    @staticmethod
    def _check_connectivity(robot: Robot, root: object, result: ValidationResult) -> None:
        child_counts: dict[str, int] = {}
        for joint in robot.joints:
            child_counts[joint.child] = child_counts.get(joint.child, 0) + 1

        for link in robot.links:
            count = child_counts.get(link.name, 0)
            if link != root and count == 0:
                result.add_error(
                    title="Disconnected link",
                    message=f"Link '{link.name}' is not connected to the kinematic tree",
                    affected_objects=[link.name],
                    suggestion=f"Create a joint connecting '{link.name}' to another link in the tree",
                )
            elif count > 1:
                result.add_error(
                    title="Multiple parent joints",
                    message=(
                        f"Link '{link.name}' has {count} parent joints (should have exactly 1)"
                    ),
                    affected_objects=[link.name],
                    suggestion="Remove extra joints. Each link can only have one parent",
                )


class MassPropertiesCheck(ValidationCheck):
    """Check for mass and inertia issues (warnings)."""

    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Check for mass property warnings."""
        for link in robot.links:
            if link.mass < 0.01:
                result.add_warning(
                    title="Very low mass",
                    message=f"Link '{link.name}' has very low mass ({link.mass:.6f} kg).",
                    affected_objects=[link.name],
                    suggestion="Consider providing a more realistic mass to avoid simulation instability",
                )

            if link.inertial is None:
                result.add_warning(
                    title="Missing inertia",
                    message=f"Link '{link.name}' has no inertia tensor defined",
                    affected_objects=[link.name],
                    suggestion="Add an inertial element or use automatic inertia calculation",
                )


class GeometryCheck(ValidationCheck):
    """Check for missing visual and collision geometry (warnings)."""

    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Check for geometry warnings."""
        for link in robot.links:
            if not link.visuals:
                result.add_warning(
                    title="No visual geometry",
                    message=f"Link '{link.name}' has no visual geometry",
                    affected_objects=[link.name],
                    suggestion="Add visual geometry for better visualization in simulators",
                )

            if not link.collisions:
                result.add_warning(
                    title="No collision geometry",
                    message=f"Link '{link.name}' has no collision geometry",
                    affected_objects=[link.name],
                    suggestion="Add collision geometry for physics simulation",
                )


class Ros2ControlCheck(ValidationCheck):
    """Check that ros2_control joints reference existing robot joints."""

    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Check ros2_control joint existence."""
        if not robot.ros2_controls:
            return

        joint_names = {joint.name for joint in robot.joints}
        for control in robot.ros2_controls:
            for rc_joint in control.joints:
                if rc_joint.name not in joint_names:
                    result.add_error(
                        title="Invalid ros2_control joint",
                        message=(
                            f"ros2_control joint '{rc_joint.name}' "
                            "does not exist in the kinematic tree"
                        ),
                        affected_objects=[rc_joint.name],
                        suggestion="Ensure joint name in control matches a robot joint",
                    )


class MimicChainCheck(ValidationCheck):
    """Check for invalid or circular mimic joint configurations."""

    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Check for invalid mimic joint configurations."""
        joint_names = {joint.name for joint in robot.joints}
        joint_map = {joint.name: joint for joint in robot.joints}

        for joint in robot.joints:
            if joint.mimic is None:
                continue

            visited: set[str] = {joint.name}
            current: str | None = joint.mimic.joint

            while current:
                if current not in joint_names:
                    result.add_error(
                        title="Invalid mimic target",
                        message=(f"Joint '{joint.name}' mimics non-existent joint '{current}'"),
                        affected_objects=[joint.name],
                        suggestion=(f"Ensure joint '{current}' exists or update mimic reference"),
                    )
                    break

                if current in visited:
                    chain = " -> ".join(visited) + f" -> {current}"
                    result.add_error(
                        title="Circular mimic dependency",
                        message=f"Circular mimic dependency detected: {chain}",
                        affected_objects=list(visited),
                        suggestion="Break the circular mimic chain by changing mimic targets",
                    )
                    break

                visited.add(current)
                next_joint = joint_map[current]
                if next_joint.mimic is None:
                    break
                current = next_joint.mimic.joint


__all__ = [
    "ValidationCheck",
    "HasLinksCheck",
    "DuplicateNameCheck",
    "JointReferenceCheck",
    "TreeStructureCheck",
    "MassPropertiesCheck",
    "GeometryCheck",
    "Ros2ControlCheck",
    "MimicChainCheck",
]
