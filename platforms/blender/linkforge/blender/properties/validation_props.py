"""Blender Property Groups for validation results.

These properties store the last validation result for display in the UI.
"""

from __future__ import annotations

import contextlib

import bpy
from bpy.props import BoolProperty, CollectionProperty, IntProperty, StringProperty
from bpy.types import PropertyGroup


class ValidationIssueProperty(PropertyGroup):
    """A single validation issue (error or warning)."""

    title: StringProperty(  # type: ignore
        name="Title",
        description="Short title of the issue",
        default="",
    )

    message: StringProperty(  # type: ignore
        name="Message",
        description="Detailed message",
        default="",
    )

    suggestion: StringProperty(  # type: ignore
        name="Suggestion",
        description="How to fix this issue",
        default="",
    )

    affected_objects: StringProperty(  # type: ignore
        name="Affected Objects",
        description="Comma-separated list of affected object names",
        default="",
    )

    @property
    def has_suggestion(self) -> bool:
        """Check if this issue has a suggestion."""
        return bool(self.suggestion)

    @property
    def has_objects(self) -> bool:
        """Check if this issue has affected objects."""
        return bool(self.affected_objects)

    @property
    def objects_str(self) -> str:
        """Get affected objects as a formatted string."""
        import typing

        return typing.cast(str, self.affected_objects)

    @property
    def message_lines(self) -> list[str]:
        """Split message into lines for display (max 60 chars per line)."""
        max_width = 60
        words = self.message.split()
        lines: list[str] = []
        current_line: list[str] = []
        current_length = 0

        for word in words:
            word_length = len(word) + (1 if current_line else 0)
            if current_length + word_length > max_width and current_line:
                lines.append(" ".join(current_line))
                current_line = [word]
                current_length = len(word)
            else:
                current_line.append(word)
                current_length += word_length

        if current_line:
            lines.append(" ".join(current_line))

        return lines

    @property
    def suggestion_lines(self) -> list[str]:
        """Split suggestion into lines for display (max 58 chars per line)."""
        if not self.suggestion:
            return []

        max_width = 58  # Account for "  " prefix
        words = self.suggestion.split()
        lines: list[str] = []
        current_line: list[str] = []
        current_length = 0

        for word in words:
            word_length = len(word) + (1 if current_line else 0)
            if current_length + word_length > max_width and current_line:
                lines.append(" ".join(current_line))
                current_line = [word]
                current_length = len(word)
            else:
                current_line.append(word)
                current_length += word_length

        if current_line:
            lines.append(" ".join(current_line))

        return lines


class ValidationResultProperty(PropertyGroup):
    """Validation result stored in window manager."""

    has_results: BoolProperty(  # type: ignore
        name="Has Results",
        description="Whether validation has been run",
        default=False,
    )

    is_valid: BoolProperty(  # type: ignore
        name="Is Valid",
        description="Whether robot passed validation (no errors)",
        default=False,
    )

    error_count: IntProperty(  # type: ignore
        name="Error Count",
        description="Number of errors",
        default=0,
    )

    warning_count: IntProperty(  # type: ignore
        name="Warning Count",
        description="Number of warnings",
        default=0,
    )

    link_count: IntProperty(  # type: ignore
        name="Link Count",
        description="Number of links in robot",
        default=0,
    )

    joint_count: IntProperty(  # type: ignore
        name="Joint Count",
        description="Number of joints in robot",
        default=0,
    )

    dof_count: IntProperty(  # type: ignore
        name="DOF Count",
        description="Degrees of freedom",
        default=0,
    )

    errors: CollectionProperty(  # type: ignore
        type=ValidationIssueProperty,
        name="Errors",
        description="List of validation errors",
    )

    warnings: CollectionProperty(  # type: ignore
        type=ValidationIssueProperty,
        name="Warnings",
        description="List of validation warnings",
    )

    show_errors: BoolProperty(  # type: ignore
        name="Show Errors",
        description="Expand errors section",
        default=True,
    )

    show_warnings: BoolProperty(  # type: ignore
        name="Show Warnings",
        description="Expand warnings section",
        default=False,
    )

    def clear(self) -> None:
        """Clear all validation results."""
        self.has_results = False
        self.is_valid = False
        self.error_count = 0
        self.warning_count = 0
        self.link_count = 0
        self.joint_count = 0
        self.dof_count = 0
        self.errors.clear()
        self.warnings.clear()

    def get_error(self, index: int) -> ValidationIssueProperty:
        """Get error by index."""
        import typing

        return typing.cast(ValidationIssueProperty, self.errors[index])

    def get_warning(self, index: int) -> ValidationIssueProperty:
        """Get warning by index."""
        import typing

        return typing.cast(ValidationIssueProperty, self.warnings[index])


# Registration
def register() -> None:
    """Register property groups."""
    # Register ValidationIssueProperty
    try:
        bpy.utils.register_class(ValidationIssueProperty)
    except ValueError:
        bpy.utils.unregister_class(ValidationIssueProperty)
        bpy.utils.register_class(ValidationIssueProperty)

    # Register ValidationResultProperty
    try:
        bpy.utils.register_class(ValidationResultProperty)
    except ValueError:
        bpy.utils.unregister_class(ValidationResultProperty)
        bpy.utils.register_class(ValidationResultProperty)

    # Register window manager property
    prop = bpy.props.PointerProperty(type=ValidationResultProperty)  # type: ignore[func-returns-value]
    bpy.types.WindowManager.linkforge_validation = prop  # type: ignore[attr-defined]


def unregister() -> None:
    """Unregister property groups."""

    with contextlib.suppress(AttributeError):
        delattr(bpy.types.WindowManager, "linkforge_validation")

    with contextlib.suppress(RuntimeError):
        bpy.utils.unregister_class(ValidationResultProperty)

        bpy.utils.unregister_class(ValidationIssueProperty)


if __name__ == "__main__":
    register()
