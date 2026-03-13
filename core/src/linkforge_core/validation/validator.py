"""Robot validator — thin orchestrator over modular validation checks.

This module provides :class:`RobotValidator`, which runs a configurable
registry of :class:`~linkforge_core.validation.checks.ValidationCheck`
instances against a robot model and returns a unified
:class:`~linkforge_core.validation.result.ValidationResult`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Type  # noqa: UP035

from .checks import (
    DuplicateNameCheck,
    GeometryCheck,
    HasLinksCheck,
    JointReferenceCheck,
    MassPropertiesCheck,
    MimicChainCheck,
    Ros2ControlCheck,
    TreeStructureCheck,
    ValidationCheck,
)
from .result import ValidationResult

if TYPE_CHECKING:
    from ..models.robot import Robot


class RobotValidator:
    """Validates robot structure for URDF export and simulation.

    Runs a configurable registry of :class:`ValidationCheck` instances.
    By default, all standard checks run in dependency order. Callers can
    pass a custom list to run only a specific subset of validation rules.

    Example:
        >>> from linkforge_core.models import Robot, Link
        >>> from linkforge_core.validation import RobotValidator
        >>> robot = Robot(name="test_robot")
        >>> robot.add_link(Link(name="base_link"))
        >>> # Validating a robot
        >>> result = RobotValidator().validate(robot)
        >>> if result.is_valid:
        ...     print("Robot is valid!")
        ... else:
        ...     print(f"Found {result.error_count} errors")

    """

    DEFAULT_CHECKS: list[Type[ValidationCheck]] = [  # noqa: UP006
        HasLinksCheck,
        DuplicateNameCheck,
        JointReferenceCheck,
        TreeStructureCheck,
        MassPropertiesCheck,
        GeometryCheck,
        Ros2ControlCheck,
        MimicChainCheck,
    ]

    def __init__(
        self,
        checks: list[ValidationCheck] | None = None,
    ) -> None:
        """Initialize validator.

        Args:
            checks: Optional custom list of check instances to run.
                Defaults to :attr:`DEFAULT_CHECKS` (all standard checks).
        """
        self._checks: list[ValidationCheck] = checks or [cls() for cls in self.DEFAULT_CHECKS]

    def validate(self, robot: Robot) -> ValidationResult:
        """Run all registered validation checks on a robot model.

        Args:
            robot: The Robot model instance to validate.

        Returns:
            ValidationResult containing all errors and warnings.

        Example:
            >>> validator = RobotValidator()
            >>> result = validator.validate(robot)
            >>> print(f"Valid: {result.is_valid}")
            >>> print(f"Errors: {result.error_count}, Warnings: {result.warning_count}")
            >>> for error in result.errors:
            ...     print(f"  - {error.title}: {error.message}")

        Note:
            The validator itself is stateless (aside from its check registry),
            so a single instance can be reused across multiple robot models.
        """
        result = ValidationResult(robot_name=robot.name)
        for check in self._checks:
            check.run(robot, result)
        return result
