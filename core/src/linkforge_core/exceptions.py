"""Custom exceptions for the LinkForge ecosystem.

This module defines the exception hierarchy used across models, parsers,
and generators to provide granular error handling.
"""


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


class XacroDetectedError(RobotParserError):
    """Raised when XACRO content is detected in a URDF parser."""

    pass
