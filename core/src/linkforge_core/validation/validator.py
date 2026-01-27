"""Robot validator with enhanced error reporting."""

from __future__ import annotations

from typing import TYPE_CHECKING

from .result import ValidationResult

if TYPE_CHECKING:
    from ..models.robot import Robot


class RobotValidator:
    """Validates robot structure with detailed error and warning reporting.

    This validator performs comprehensive checks on robot models to ensure they
    are valid for URDF export and simulation. It checks for common issues like:
    - Missing or duplicate names
    - Invalid joint references
    - Disconnected links (not part of kinematic tree)
    - Missing mass/inertia properties
    - Invalid geometry definitions

    The validator distinguishes between errors (must be fixed) and warnings
    (should be reviewed but not critical).

    Example:
        >>> from linkforge.core.models import Robot, Link
        >>> from linkforge.core.validation import RobotValidator
        >>> robot = Robot(name="test_robot")
        >>> robot.add_link(Link(name="base_link"))
        >>> validator = RobotValidator(robot)
        >>> result = validator.validate()
        >>> if result.is_valid:
        ...     print("Robot is valid!")
        ... else:
        ...     print(f"Found {result.error_count} errors")
    """

    def __init__(self, robot: Robot):
        """Initialize validator.

        Args:
            robot: Robot model to validate

        """
        self.robot = robot
        self.result = ValidationResult(robot_name=robot.name)

    def validate(self) -> ValidationResult:
        """Run all validation checks on the robot model.

        Performs comprehensive validation including:
        1. Basic structure checks (has links, no duplicate names)
        2. Joint reference validation (parent/child links exist)
        3. Kinematic tree structure (no cycles, all links connected)
        4. Mass properties (inertia for dynamic links)
        5. Geometry validation (valid dimensions)

        Returns:
            ValidationResult containing:
            - is_valid: True if no errors found (warnings are allowed)
            - error_count: Number of critical errors
            - warning_count: Number of non-critical warnings
            - errors: List of ValidationIssue objects with details
            - warnings: List of ValidationIssue objects with details

        Example:
            >>> validator = RobotValidator(robot)
            >>> result = validator.validate()
            >>> print(f"Valid: {result.is_valid}")
            >>> print(f"Errors: {result.error_count}, Warnings: {result.warning_count}")
            >>> for error in result.errors:
            ...     print(f"  - {error.title}: {error.message}")

        Note:
            Each call to validate() resets the result, so you can call this
            multiple times after modifying the robot.
        """
        self.result = ValidationResult(robot_name=self.robot.name)

        # Run all validation checks
        self._check_has_links()
        self._check_duplicate_names()
        self._check_joint_references()
        self._check_tree_structure()
        self._check_mass_properties()
        self._check_geometry()

        return self.result

    def _check_has_links(self) -> None:
        """Check that robot has at least one link."""
        if not self.robot.links:
            self.result.add_error(
                title="No links",
                message="Robot must have at least one link",
                suggestion="Add a link by marking an object as a robot link in the Link panel",
            )

    def _check_duplicate_names(self) -> None:
        """Check for duplicate link and joint names."""
        # Check duplicate link names
        link_names = [link.name for link in self.robot.links]
        seen = set()
        for name in link_names:
            if name in seen:
                # Find all objects with this name
                duplicates = [link.name for link in self.robot.links if link.name == name]
                self.result.add_error(
                    title="Duplicate link name",
                    message=f"Link name '{name}' is used by {link_names.count(name)} links. Each link must have a unique name.",
                    affected_objects=duplicates,
                    suggestion=f"Rename duplicate links to unique names (e.g., '{name}_1', '{name}_2')",
                )
                break  # Only report once
            seen.add(name)

        # Check duplicate joint names
        joint_names = [joint.name for joint in self.robot.joints]
        seen = set()
        for name in joint_names:
            if name in seen:
                duplicates = [joint.name for joint in self.robot.joints if joint.name == name]
                self.result.add_error(
                    title="Duplicate joint name",
                    message=f"Joint name '{name}' is used by {joint_names.count(name)} joints. Each joint must have a unique name.",
                    affected_objects=duplicates,
                    suggestion=f"Rename duplicate joints to unique names (e.g., '{name}_1', '{name}_2')",
                )
                break  # Only report once
            seen.add(name)

    def _check_joint_references(self) -> None:
        """Check that all joints reference valid links."""
        link_name_set = {link.name for link in self.robot.links}

        for joint in self.robot.joints:
            if joint.parent not in link_name_set:
                self.result.add_error(
                    title="Missing parent link",
                    message=f"Joint '{joint.name}' references parent link '{joint.parent}' which does not exist",
                    affected_objects=[joint.name],
                    suggestion=f"Create a link named '{joint.parent}' or update the joint's parent reference",
                )

            if joint.child not in link_name_set:
                self.result.add_error(
                    title="Missing child link",
                    message=f"Joint '{joint.name}' references child link '{joint.child}' which does not exist",
                    affected_objects=[joint.name],
                    suggestion=f"Create a link named '{joint.child}' or update the joint's child reference",
                )

    def _check_tree_structure(self) -> None:
        """Check kinematic tree structure."""
        if not self.robot.links:
            return  # Already reported in _check_has_links

        # Check for cycles
        if self.robot._has_cycle():
            self.result.add_error(
                title="Circular dependency",
                message="Kinematic tree contains a cycle. Links must form a tree structure, not a loop.",
                suggestion="Review joint connections to ensure they form a tree (no loops)",
            )

        # Check for root link
        try:
            root = self.robot.get_root_link()
            if root is None:
                self.result.add_error(
                    title="No root link",
                    message="No root link found. A robot must have exactly one link that is not a child in any joint.",
                    suggestion="Ensure exactly one link has no parent joint (the base/root link)",
                )
        except ValueError as e:
            error_msg = str(e)
            if "Multiple root links" in error_msg:
                # Extract root link names from error message
                self.result.add_error(
                    title="Multiple root links",
                    message=error_msg,
                    suggestion="Ensure only one link has no parent joint. Connect other root links to the tree with joints.",
                )
            else:
                self.result.add_error(
                    title="Root link error",
                    message=error_msg,
                    suggestion="Check the joint connections in your robot tree",
                )
            return

        # Check connectivity - each link (except root) should be a child in exactly one joint
        child_counts: dict[str, int] = {}
        for joint in self.robot.joints:
            child_counts[joint.child] = child_counts.get(joint.child, 0) + 1

        for link in self.robot.links:
            count = child_counts.get(link.name, 0)

            if root and link != root and count == 0:
                self.result.add_error(
                    title="Disconnected link",
                    message=f"Link '{link.name}' is not connected to the kinematic tree",
                    affected_objects=[link.name],
                    suggestion=f"Create a joint connecting '{link.name}' to another link in the tree",
                )
            elif count > 1:
                self.result.add_error(
                    title="Multiple parent joints",
                    message=f"Link '{link.name}' has {count} parent joints (should have exactly 1)",
                    affected_objects=[link.name],
                    suggestion="Remove extra joints. Each link can only have one parent.",
                )

    def _check_mass_properties(self) -> None:
        """Check for mass property issues (warnings)."""
        for link in self.robot.links:
            # Warn about very low mass
            if link.mass < 0.01:
                self.result.add_warning(
                    title="Very low mass",
                    message=f"Link '{link.name}' has very low mass ({link.mass:.6f} kg)",
                    affected_objects=[link.name],
                    suggestion="Consider if this mass is realistic. Very low masses can cause simulation instability.",
                )

            # Warn about missing inertia
            if link.inertial is None:
                self.result.add_warning(
                    title="Missing inertia",
                    message=f"Link '{link.name}' has no inertia tensor defined",
                    affected_objects=[link.name],
                    suggestion="Add an inertial element or use automatic inertia calculation",
                )

    def _check_geometry(self) -> None:
        """Check for geometry issues (warnings)."""
        for link in self.robot.links:
            # Warn about missing visual geometry
            if not link.visuals:
                self.result.add_warning(
                    title="No visual geometry",
                    message=f"Link '{link.name}' has no visual geometry",
                    affected_objects=[link.name],
                    suggestion="Add visual geometry for better visualization in simulators",
                )

            # Warn about missing collision geometry
            if not link.collisions:
                self.result.add_warning(
                    title="No collision geometry",
                    message=f"Link '{link.name}' has no collision geometry",
                    affected_objects=[link.name],
                    suggestion="Add collision geometry for physics simulation",
                )
