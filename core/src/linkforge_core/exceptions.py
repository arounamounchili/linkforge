"""Custom exceptions for the LinkForge ecosystem.

This module defines the exception hierarchy used across models, parsers,
and generators to provide granular error handling.
"""

from enum import Enum
from pathlib import Path
from typing import Any


class ValidationErrorCode(Enum):
    """Categorized error codes for robot validation failures."""

    # Naming and Identity
    INVALID_NAME = "invalid_name"
    DUPLICATE_NAME = "duplicate_name"
    NAME_EMPTY = "name_empty"

    # Kinematic Structure
    NOT_FOUND = "not_found"
    HAS_CYCLE = "has_cycle"
    NO_ROOT = "no_root"
    MULTIPLE_ROOTS = "multiple_roots"
    DISCONNECTED = "disconnected"

    # Physics and Values
    OUT_OF_RANGE = "out_of_range"
    VALUE_EMPTY = "value_empty"
    INVALID_VALUE = "invalid_value"
    PHYSICS_VIOLATION = "physics_violation"
    MATH_ERROR = "math_error"
    INERTIA_TRIANGLE_INEQUALITY = "inertia_triangle_inequality"

    # Configuration and Misc
    MISMATCH = "mismatch"
    GENERIC_FAILURE = "generic_failure"


class LinkForgeError(Exception):
    """Base category for all LinkForge-related exceptions."""

    pass


class RobotModelError(LinkForgeError):
    """Exception raised for structural or logic errors in the Robot model."""

    pass


class RobotGeneratorError(LinkForgeError):
    """Exception raised during robot generation or export."""

    pass


class RobotParserError(LinkForgeError):
    """Exception raised during robot parsing or import."""

    pass


class RobotParserIOError(RobotParserError):
    """Exception raised for file-level or I/O errors during parsing."""

    def __init__(self, filepath: Path | str = "unknown", reason: str = "error"):
        super().__init__(f"Parser IO error: {reason} (file: {filepath})")


class RobotParserXMLRootError(RobotParserError):
    """Exception raised when the XML root element is invalid."""

    def __init__(self, actual_tag: str = "unknown", expected_tag: str = "robot"):
        super().__init__(f"Invalid XML root: <{actual_tag}> (expected <{expected_tag}>)")


class RobotParserUnexpectedError(RobotParserError):
    """General wrapper for unexpected parsing failures."""

    def __init__(self, source_area: str = "unknown", original_error: Any = None):
        msg = f"Unexpected error in {source_area}"
        if original_error:
            msg += f": {original_error}"
        super().__init__(msg)


class RobotPhysicsError(RobotModelError):
    """Exception raised for unphysical properties (e.g. negative mass or volume)."""

    def __init__(
        self,
        code: ValidationErrorCode,
        message: str,
        target: str | None = None,
        value: Any = None,
    ):
        self.code = code
        self.target = target
        self.value = value
        self.raw_message = message

        full_msg = f"[PHYSICS_{code.name}] {message}"
        if target:
            full_msg += f" (target: {target})"
        if value is not None:
            full_msg += f" (value: {value})"

        super().__init__(full_msg)


class RobotValidationError(RobotModelError):
    """Exception raised for structural or logic validation failures.

    Now structured using ValidationErrorCode for robust error handling.
    """

    def __init__(
        self,
        code: ValidationErrorCode,
        message: str,
        target: str | None = None,
        value: Any = None,
    ):
        self.code = code
        self.target = target
        self.value = value
        self.raw_message = message

        full_msg = f"[{code.name}] {message}"
        if target:
            full_msg += f" (target: {target})"
        if value is not None:
            full_msg += f" (value: {value})"

        super().__init__(full_msg)


class RobotSecurityError(RobotModelError):
    """Exception raised for security-related errors (e.g. sandbox escapes)."""

    def __init__(self, path: str = "unknown", reason: str = "violation"):
        super().__init__(f"Security Violation: {reason} (path: {path})")


class RobotMathError(RobotModelError):
    """Exception raised for invalid numerical values (NaN, Inf, or Out of Range)."""

    def __init__(
        self,
        code: ValidationErrorCode,
        message: str,
        target: str | None = None,
        value: Any = None,
    ):
        self.code = code
        self.target = target
        self.value = value
        self.raw_message = message

        full_msg = f"[MATH_{code.name}] {message}"
        if target:
            full_msg += f" (target: {target})"
        if value is not None:
            full_msg += f" (value: {value})"

        super().__init__(full_msg)


class RobotXacroError(RobotParserError):
    """General exception for XACRO resolution failures."""

    def __init__(self, message: str = "failure", context: str | None = None):
        ctx = f" (at {context})" if context else ""
        super().__init__(f"XACRO error: {message}{ctx}")


class RobotXacroRecursionError(RobotXacroError):
    """Exception raised for circular dependencies or max depth in XACRO."""

    def __init__(self, depth: int | str = "unknown", reason: str | None = None):
        msg = f"Recursion depth exceeded: {depth}"
        if reason:
            msg += f" ({reason})"
        super().__init__(msg)


class RobotXacroExpressionError(RobotXacroError):
    """Exception raised for failures in XACRO math or property evaluation."""

    def __init__(self, expression: str = "unknown", reason: str = "error"):
        super().__init__(f"Expression evaluation failed: ${{{expression}}} -> {reason}")


class XacroDetectedError(RobotParserError):
    """Raised when XACRO content is detected in a URDF parser."""

    def __init__(self, message: str = "XACRO detected"):
        super().__init__(
            f"XACRO file detected: {message}. Please convert to URDF or use XACROParser."
        )
