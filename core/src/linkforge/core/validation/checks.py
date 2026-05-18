"""Modular validation checks for robot models.

This module provides focused validation rules following the Single
Responsibility Principle. Each check is independent and contributes to
a shared :class:`~result.ValidationResult`.

Check Categories:
- **Topology**: Links, joints, and kinematic tree integrity.
- **Physics**: Mass, inertia, and numerical stability.
- **Interfaces**: ros2_control and mimic joint chains.
- **Semantic**: SRDF planning groups and collision filters.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import TYPE_CHECKING

from ..constants import (
    MIN_MASS_STABILITY_THRESHOLD,
    MIN_REASONABLE_INERTIA,
    MIN_REASONABLE_MASS,
)
from ..exceptions import RobotModelError, RobotValidationError, ValidationErrorCode
from .result import ValidationResult

if TYPE_CHECKING:
    from ..models.link import Link
    from ..models.robot import Robot
    from ..models.srdf import PlanningGroup


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
        ...


class HasLinksCheck(ValidationCheck):
    """Check that the robot has at least one link."""

    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Check that robot has at least one link."""
        if not robot.links:
            result.add_error(
                title="No links",
                message="Robot must have at least one link",
                code=ValidationErrorCode.VALUE_EMPTY,
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
                    code=ValidationErrorCode.DUPLICATE_NAME,
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
                    code=ValidationErrorCode.NOT_FOUND,
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
                    code=ValidationErrorCode.NOT_FOUND,
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
            if robot.has_cycle:
                result.add_error(
                    title="Circular dependency",
                    message=(
                        "Kinematic tree contains a cycle. "
                        "Links must form a tree structure, not a loop."
                    ),
                    code=ValidationErrorCode.HAS_CYCLE,
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
                    code=ValidationErrorCode.INVALID_VALUE,
                    suggestion="Check joint and link consistency",
                )

    @staticmethod
    def _check_root(robot: Robot, result: ValidationResult) -> Link | None:
        """Return the root link, or None if it cannot be determined."""
        try:
            return robot.root_link
        except RobotValidationError as e:
            if e.code == ValidationErrorCode.NO_ROOT:
                result.add_error(
                    title="No root link",
                    message=(
                        "No root link found. A robot must have exactly one link "
                        "that is not a child in any joint."
                    ),
                    code=ValidationErrorCode.NO_ROOT,
                    suggestion="Ensure exactly one link has no parent joint (the base/root link)",
                )
            elif e.code == ValidationErrorCode.MULTIPLE_ROOTS:
                result.add_error(
                    title="Multiple root links",
                    message=str(e),
                    code=ValidationErrorCode.MULTIPLE_ROOTS,
                    suggestion="Ensure only one link has no parent joint. Connect other root links to the tree with joints",
                )
            else:
                result.add_error(
                    title="Root link error",
                    message=str(e),
                    suggestion="Check the joint connections in your robot tree",
                )
            return None
        except RobotModelError as e:
            result.add_error(
                title="Kinematic error",
                message=str(e),
                code=ValidationErrorCode.INVALID_VALUE,
            )
            return None

    @staticmethod
    def _check_connectivity(robot: Robot, root: Link, result: ValidationResult) -> None:
        child_counts: dict[str, int] = {}
        for joint in robot.joints:
            child_counts[joint.child] = child_counts.get(joint.child, 0) + 1

        for link in robot.links:
            count = child_counts.get(link.name, 0)
            if count > 1:
                result.add_error(
                    title="Multiple parent joints",
                    message=(
                        f"Link '{link.name}' has {count} parent joints (should have exactly 1)"
                    ),
                    affected_objects=[link.name],
                    code=ValidationErrorCode.MULTIPLE_ROOTS,
                    suggestion="Remove extra joints. Each link can only have one parent",
                )
            elif count == 0 and link.name != root.name:
                result.add_error(
                    title="Disconnected link",
                    message=f"Link '{link.name}' is not connected to the kinematic tree",
                    affected_objects=[link.name],
                    code=ValidationErrorCode.MULTIPLE_ROOTS,
                    suggestion="Add a joint to connect this link to the kinematic tree",
                )


class MassPropertiesCheck(ValidationCheck):
    """Check for mass and inertia issues (warnings)."""

    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Check for mass property health."""
        for link in robot.links:
            # 1. Mass Checks
            if link.mass < MIN_REASONABLE_MASS:
                result.add_error(
                    title="Critical low mass",
                    message=(
                        f"Link '{link.name}' has near-zero mass ({link.mass:.9f} kg). "
                        "This will crash most physics solvers."
                    ),
                    affected_objects=[link.name],
                    code=ValidationErrorCode.PHYSICS_VIOLATION,
                    suggestion=f"Increase mass to at least {MIN_REASONABLE_MASS} kg",
                )
            elif link.mass < MIN_MASS_STABILITY_THRESHOLD:
                result.add_warning(
                    title="Very low mass",
                    message=f"Link '{link.name}' has low mass ({link.mass:.6f} kg).",
                    affected_objects=[link.name],
                    code=ValidationErrorCode.INVALID_VALUE,
                    suggestion="Consider providing a more realistic mass for better simulation stability",
                )

            # 2. Inertia Checks
            if link.inertial is None:
                result.add_warning(
                    title="Missing inertia",
                    message=f"Link '{link.name}' has no inertia tensor defined",
                    affected_objects=[link.name],
                    code=ValidationErrorCode.NOT_FOUND,
                    suggestion="Add an inertial element or use automatic inertia calculation",
                )
            else:
                tensor = link.inertial.inertia
                if any(v < MIN_REASONABLE_INERTIA for v in [tensor.ixx, tensor.iyy, tensor.izz]):
                    result.add_error(
                        title="Critical low inertia",
                        message=(
                            f"Link '{link.name}' has near-zero inertia diagonals. "
                            "This will lead to numerical instability."
                        ),
                        affected_objects=[link.name],
                        code=ValidationErrorCode.PHYSICS_VIOLATION,
                        suggestion=f"Increase inertia diagonals to at least {MIN_REASONABLE_INERTIA}",
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
                    code=ValidationErrorCode.NOT_FOUND,
                    suggestion="Add visual geometry for better visualization in simulators",
                )

            if not link.collisions:
                result.add_warning(
                    title="No collision geometry",
                    message=f"Link '{link.name}' has no collision geometry",
                    affected_objects=[link.name],
                    code=ValidationErrorCode.NOT_FOUND,
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
                        code=ValidationErrorCode.NOT_FOUND,
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

            while True:
                if current not in joint_names:
                    result.add_error(
                        title="Invalid mimic target",
                        message=(f"Joint '{joint.name}' mimics non-existent joint '{current}'"),
                        affected_objects=[joint.name],
                        code=ValidationErrorCode.NOT_FOUND,
                        suggestion=(f"Ensure joint '{current}' exists or update mimic reference"),
                    )
                    break

                if current in visited:
                    chain = " -> ".join(visited) + f" -> {current}"
                    result.add_error(
                        title="Circular mimic dependency",
                        message=f"Circular mimic dependency detected: {chain}",
                        affected_objects=list(visited),
                        code=ValidationErrorCode.HAS_CYCLE,
                        suggestion="Break the circular mimic chain by changing mimic targets",
                    )
                    break

                visited.add(current)
                next_joint = joint_map[current]
                if next_joint.mimic is None:
                    break
                current = next_joint.mimic.joint


class SemanticCheck(ValidationCheck):
    """Check for semantic robot description (SRDF) invariants."""

    def run(self, robot: Robot, result: ValidationResult) -> None:
        """Check SRDF invariants."""
        if not robot.semantic:
            return

        semantic = robot.semantic
        group_names = {g.name for g in semantic.groups}
        link_names = {link.name for link in robot.links}
        joint_names = {joint.name for joint in robot.joints}

        # Check planning groups
        for group in semantic.groups:
            # Check links
            for link_name in group.links:
                if link_name not in link_names:
                    result.add_error(
                        title="Invalid planning group link",
                        message=f"Group '{group.name}' references non-existent link '{link_name}'",
                        affected_objects=[group.name],
                        code=ValidationErrorCode.NOT_FOUND,
                    )
            # Check joints
            for joint_name in group.joints:
                if joint_name not in joint_names:
                    result.add_error(
                        title="Invalid planning group joint",
                        message=f"Group '{group.name}' references non-existent joint '{joint_name}'",
                        affected_objects=[group.name],
                        code=ValidationErrorCode.NOT_FOUND,
                    )
            # Check subgroups and cycles
            self._check_subgroup_cycles(group, semantic.groups, result)

        # Check group states
        for state in semantic.group_states:
            if state.group not in group_names:
                result.add_error(
                    title="Invalid group state reference",
                    message=f"State '{state.name}' references non-existent group '{state.group}'",
                    affected_objects=[state.name],
                    code=ValidationErrorCode.NOT_FOUND,
                )

        # Check end effectors
        for ee in semantic.end_effectors:
            if ee.group not in group_names:
                result.add_error(
                    title="Invalid end effector group",
                    message=f"End effector '{ee.name}' references non-existent group '{ee.group}'",
                    affected_objects=[ee.name],
                    code=ValidationErrorCode.NOT_FOUND,
                )
            if ee.parent_link not in link_names:
                result.add_error(
                    title="Invalid end effector parent link",
                    message=f"End effector '{ee.name}' references non-existent parent link '{ee.parent_link}'",
                    affected_objects=[ee.name],
                    code=ValidationErrorCode.NOT_FOUND,
                )
            if ee.parent_group and ee.parent_group not in group_names:
                result.add_error(
                    title="Invalid end effector parent group",
                    message=f"End effector '{ee.name}' references non-existent parent group '{ee.parent_group}'",
                    affected_objects=[ee.name],
                    code=ValidationErrorCode.NOT_FOUND,
                )

        # Check passive joints
        for pj in semantic.passive_joints:
            if pj.name not in joint_names:
                result.add_error(
                    title="Invalid passive joint",
                    message=f"Passive joint '{pj.name}' does not exist",
                    affected_objects=[pj.name],
                    code=ValidationErrorCode.NOT_FOUND,
                )

    def _check_subgroup_cycles(
        self, group: PlanningGroup, all_groups: Sequence[PlanningGroup], result: ValidationResult
    ) -> None:
        """Check for circular subgroup dependencies."""
        group_map = {g.name for g in all_groups}
        group_obj_map = {g.name: g for g in all_groups}

        def _dfs(current_name: str, path: list[str]) -> bool:
            current_group = group_obj_map.get(current_name)
            if not current_group:
                return False

            for sg_name in current_group.subgroups:
                if sg_name not in group_map:
                    result.add_error(
                        title="Invalid subgroup reference",
                        message=f"Group '{current_name}' references non-existent subgroup '{sg_name}'",
                        affected_objects=[current_name],
                        code=ValidationErrorCode.NOT_FOUND,
                    )
                    continue

                if sg_name in path:
                    cycle = " -> ".join(path[path.index(sg_name) :]) + f" -> {sg_name}"
                    result.add_error(
                        title="Circular subgroup dependency",
                        message=f"Circular subgroup dependency detected: {cycle}",
                        affected_objects=path[path.index(sg_name) :],
                        code=ValidationErrorCode.HAS_CYCLE,
                    )
                    return True

                if _dfs(sg_name, path + [sg_name]):
                    return True
            return False

        _dfs(group.name, [group.name])


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
    "SemanticCheck",
]
