"""Robot validation orchestrator.

This module provides the :class:`RobotValidator`, which coordinates a suite
of modular :class:`ValidationCheck` instances to verify the structural,
kinematic, and physical integrity of a robot model.

Core Responsibilities:
- **Orchestration**: Running multiple checks in a specific order.
- **Reporting**: Aggregating issues into a unified :class:`ValidationResult`.
- **Consistency**: Ensuring internal model indices are fresh before validation.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from ..exceptions import RobotValidationError
from .checks import (
    DuplicateNameCheck,
    GeometryCheck,
    HasLinksCheck,
    JointReferenceCheck,
    MassPropertiesCheck,
    MimicChainCheck,
    Ros2ControlCheck,
    SemanticCheck,
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
        >>> from linkforge.core.models import Robot, Link
        >>> from linkforge.core import RobotValidator
        >>> robot = Robot(name="test_robot")
        >>> robot.add_link(Link(name="base_link"))
        >>> # Validating a robot
        >>> result = RobotValidator().validate(robot)
        >>> if result.is_valid:
        ...     print("Robot is valid!")
        ... else:
        ...     print(f"Found {result.error_count} errors")

    """

    DEFAULT_CHECKS: list[type[ValidationCheck]] = [
        HasLinksCheck,
        DuplicateNameCheck,
        JointReferenceCheck,
        TreeStructureCheck,
        MassPropertiesCheck,
        GeometryCheck,
        Ros2ControlCheck,
        MimicChainCheck,
        SemanticCheck,
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
        """
        result = ValidationResult(robot_name=robot.name)

        # Ensure internal indices are fresh before validation
        try:
            robot._reindex()
        except RobotValidationError as e:
            # Report indexing errors (like duplicates) as validation errors
            result.add_error(
                title=str(e),
                message=str(e),
                code=e.code,
                affected_objects=[str(e.value)] if e.value is not None else [],
            )

        for check in self._checks:
            try:
                check.run(robot, result)
            except Exception as e:
                result.add_error(
                    title="Check failure",
                    message=f"Validation check {check.__class__.__name__} failed: {str(e)}",
                )

        return result
