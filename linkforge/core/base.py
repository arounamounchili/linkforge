"""Base classes for Robot Generators and Parsers.

This module defines the abstract base classes that all specific format generators
(URDF, XACRO, MJCF, etc.) and parsers should inherit from. This ensures a consistent
API for the LinkForge ecosystem.
"""

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Generic, TypeVar

from .models.robot import Robot

# Generic type for the output format (e.g., str for XML, dict for JSON)
T = TypeVar("T")


class RobotGenerator(ABC, Generic[T]):  # noqa: UP046
    """Abstract base class for all Robot Generators."""

    @abstractmethod
    def generate(self, robot: Robot) -> T:
        """Generate the output representation from the Robot model.

        Args:
            robot: The generic Robot model (Intermediate Representation)

        Returns:
            The generated output (e.g. XML string, JSON dict)
        """
        pass

    def write(self, robot: Robot, filepath: Path) -> None:
        """Write the generated output to a file.

        Args:
            robot: Robot model to export
            filepath: Destination file path
        """
        content = self.generate(robot)

        # Default implementation for string-based content (XML/JSON)
        # Binary formats should override this
        if isinstance(content, str):
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(content)
        else:
            raise NotImplementedError(
                f"Default write method only supports strings. "
                f"Generator {self.__class__.__name__} must override write()."
            )


class RobotParser(ABC):
    """Abstract base class for all Robot Parsers."""

    @abstractmethod
    def parse(self, filepath: Path) -> Robot:
        """Parse a file into a Robot model.

        Args:
            filepath: Path to the input file

        Returns:
            The generic Robot model (Intermediate Representation)
        """
        pass
