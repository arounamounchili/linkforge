"""Base classes for Robot Generators and Parsers.

This module defines the abstract base classes that all specific format generators
(URDF, XACRO, MJCF, etc.) and parsers should inherit from. This ensures a consistent
API for the LinkForge ecosystem.
"""

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Generic, TypeVar

from .models.robot import Robot

# Generic type for the output format (e.g., str for XML, dict for JSON)
T = TypeVar("T")


class LinkForgeError(Exception):
    """Base category for all LinkForge-related exceptions."""

    pass


class RobotGeneratorError(LinkForgeError):
    """Exception raised during robot generation or export."""

    pass


class RobotParserError(LinkForgeError):
    """Exception raised during robot parsing or import."""

    pass


class XacroDetectedError(RobotParserError):
    """Raised when XACRO content is detected in a URDF parser.

    This allows the platform layer to catch a specific exception and
    automatically switch to the XACRO parser.
    """

    pass


class RobotGenerator(ABC, Generic[T]):  # noqa: UP046
    """Abstract base class for all Robot Generators."""

    @abstractmethod
    def generate(self, robot: Robot, **kwargs: Any) -> T:
        """Generate the output representation from the Robot model.

        Args:
            robot: The generic Robot model (Intermediate Representation)
            **kwargs: Format-specific generation options

        Returns:
            The generated output (e.g. XML string, JSON dict)
        """
        pass

    def write(self, robot: Robot, filepath: Path, **kwargs: Any) -> None:
        """Write the generated output to a file.

        This is a template method that handles directory creation and
        delegates the actual writing to the _save_to_file hook.

        Args:
            robot: Robot model to export
            filepath: Destination file path
            **kwargs: Options passed to generate() and _save_to_file()
        """
        try:
            # Ensure parent directory exists
            filepath.parent.mkdir(parents=True, exist_ok=True)

            content = self.generate(robot, **kwargs)
            self._save_to_file(content, filepath, **kwargs)
        except Exception as e:
            if isinstance(e, LinkForgeError):
                raise
            raise RobotGeneratorError(f"Failed to write robot to {filepath}: {e}") from e

    def _save_to_file(self, content: T, filepath: Path, **kwargs: Any) -> None:
        """Default I/O hook for saving content.

        Supports string-based content by default. Binary generators or formats
        requiring special handling should override this.

        Args:
            content: Generated content from generate()
            filepath: Target file path
            **kwargs: Additional options
        """
        if isinstance(content, str):
            filepath.write_text(content, encoding="utf-8")
        elif isinstance(content, bytes):
            filepath.write_bytes(content)
        else:
            raise RobotGeneratorError(
                f"Default _save_to_file does not support {type(content)}. "
                f"Generator {self.__class__.__name__} must override this method."
            )


class RobotParser(ABC):
    """Abstract base class for all Robot Parsers."""

    @abstractmethod
    def parse(self, filepath: Path, **kwargs: Any) -> Robot:
        """Parse a file into a Robot model.

        Args:
            filepath: Path to the input file
            **kwargs: Format-specific parsing options

        Returns:
            The generic Robot model (Intermediate Representation)
        """
        pass
