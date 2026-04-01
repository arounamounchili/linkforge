"""Custom exceptions for the LinkForge ecosystem.

This module defines the exception hierarchy used across models, parsers,
and generators to provide granular error handling.
"""

from pathlib import Path
from typing import Any


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
        self, property_name: str = "unknown", value: Any = None, reason: str | None = None
    ):
        msg = f"Invalid physics property '{property_name}': {value}"
        if reason:
            msg += f" ({reason})"
        super().__init__(msg)


class RobotValidationError(RobotModelError):
    """Exception raised for structural or logic validation failures."""

    def __init__(self, check_name: str = "unknown", value: Any = None, reason: str | None = None):
        msg = f"Validation failed [{check_name}]: {value}"
        if reason:
            msg += f" ({reason})"
        super().__init__(msg)


class RobotSecurityError(RobotModelError):
    """Exception raised for security-related errors (e.g. sandbox escapes)."""

    def __init__(self, path: str = "unknown", reason: str = "violation"):
        super().__init__(f"Security Violation: {reason} (path: {path})")


class RobotMathError(RobotModelError):
    """Exception raised for invalid numerical values (NaN, Inf, or Out of Range)."""

    def __init__(
        self, value: float | str | None = None, check_name: str = "value", reason: str | None = None
    ):
        msg = f"Invalid value '{value}' in {check_name}: must be a finite number"
        if reason:
            msg += f" ({reason})"
        super().__init__(msg)


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
