"""Validation result data structures."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum


class Severity(Enum):
    """Severity level of a validation issue."""

    ERROR = "error"  # Blocking - will prevent export/simulation
    WARNING = "warning"  # Non-blocking - user should review


@dataclass
class ValidationIssue:
    """A single validation issue (error or warning).

    Attributes:
        severity: Error or warning
        title: Short description (e.g., "Duplicate link name")
        message: Detailed message with context
        affected_objects: Names of objects involved
        suggestion: How to fix this issue
        auto_fix: Optional function to automatically fix the issue

    """

    severity: Severity
    title: str
    message: str
    affected_objects: list[str] = field(default_factory=list)
    suggestion: str | None = None
    auto_fix: Callable[[], None] | None = None

    @property
    def is_error(self) -> bool:
        """Check if this is an error (blocking)."""
        return self.severity == Severity.ERROR

    @property
    def is_warning(self) -> bool:
        """Check if this is a warning (non-blocking)."""
        return self.severity == Severity.WARNING

    def __str__(self) -> str:
        """String representation."""
        prefix = "ERROR" if self.is_error else "WARNING"
        objects = f" [{', '.join(self.affected_objects)}]" if self.affected_objects else ""
        return f"{prefix}: {self.title}{objects} - {self.message}"


@dataclass
class ValidationResult:
    """Result of robot validation."""

    issues: list[ValidationIssue] = field(default_factory=list)
    robot_name: str = ""

    @property
    def errors(self) -> list[ValidationIssue]:
        """Get only errors."""
        return [issue for issue in self.issues if issue.is_error]

    @property
    def warnings(self) -> list[ValidationIssue]:
        """Get only warnings."""
        return [issue for issue in self.issues if issue.is_warning]

    @property
    def is_valid(self) -> bool:
        """Check if validation passed (no errors).

        Note: Warnings don't block validity.
        """
        return len(self.errors) == 0

    @property
    def has_warnings(self) -> bool:
        """Check if there are any warnings."""
        return len(self.warnings) > 0

    @property
    def error_count(self) -> int:
        """Number of errors."""
        return len(self.errors)

    @property
    def warning_count(self) -> int:
        """Number of warnings."""
        return len(self.warnings)

    def add_error(
        self,
        title: str,
        message: str,
        affected_objects: list[str] | None = None,
        suggestion: str | None = None,
        auto_fix: Callable[[], None] | None = None,
    ) -> None:
        """Add an error to the validation result."""
        self.issues.append(
            ValidationIssue(
                severity=Severity.ERROR,
                title=title,
                message=message,
                affected_objects=affected_objects or [],
                suggestion=suggestion,
                auto_fix=auto_fix,
            )
        )

    def add_warning(
        self,
        title: str,
        message: str,
        affected_objects: list[str] | None = None,
        suggestion: str | None = None,
        auto_fix: Callable[[], None] | None = None,
    ) -> None:
        """Add a warning to the validation result."""
        self.issues.append(
            ValidationIssue(
                severity=Severity.WARNING,
                title=title,
                message=message,
                affected_objects=affected_objects or [],
                suggestion=suggestion,
                auto_fix=auto_fix,
            )
        )

    def __str__(self) -> str:
        """String representation."""
        if self.is_valid and not self.has_warnings:
            return f"Robot '{self.robot_name}' is valid"
        elif self.is_valid:
            return f"Robot '{self.robot_name}' is valid with {self.warning_count} warning(s)"
        else:
            return f"Robot '{self.robot_name}' has {self.error_count} error(s) and {self.warning_count} warning(s)"
