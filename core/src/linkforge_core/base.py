"""Base classes for Robot Generators and Parsers.

This module defines the abstract base classes that all specific format generators
(URDF, XACRO, MJCF, etc.) and parsers should inherit from. This ensures a consistent
API for the LinkForge ecosystem.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
from typing import TYPE_CHECKING, Any, Generic, Protocol, TypeVar, runtime_checkable

from .exceptions import (  # noqa: F401
    LinkForgeError,
    RobotGeneratorError,
    RobotModelError,
    RobotParserError,
    XacroDetectedError,
)

if TYPE_CHECKING:
    from .models.robot import Robot

# Generic type for the output format (e.g., str for XML, dict for JSON)
T = TypeVar("T")

__all__ = [
    "RobotParser",
    "IResourceResolver",
    "FileSystemResolver",
    "NetworkResolver",
    "LinkForgeError",
    "RobotGeneratorError",
    "RobotModelError",
    "RobotParserError",
    "XacroDetectedError",
]


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
        pass  # pragma: no cover

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
        pass  # pragma: no cover


@runtime_checkable
class IResourceResolver(Protocol):
    """Protocol for resolving resource URIs (e.g. package://, file://, https://)."""

    def resolve(self, uri: str) -> Path:
        """Resolve a URI to a local filesystem Path.

        Args:
            uri: The resource URI to resolve.

        Returns:
            The resolved absolute Path.

        Raises:
            FileNotFoundError: If the resource cannot be located.
        """
        ...


class FileSystemResolver:
    """Default resolver that handles standard file paths and package:// URIs."""

    def resolve(self, uri: str) -> Path:
        """Resolve standard file paths.

        Note: package:// resolution is currently handled by converters, but
        this resolver will eventually absorb that logic.
        """
        path = Path(uri)
        if path.exists():
            return path.absolute()

        # Fallback for relative paths if needed, though usually URIs are absolute
        # or resolved relative to the URDF file during parsing.
        raise FileNotFoundError(f"Could not resolve resource: {uri}")


class NetworkResolver:
    """Mock network resolver for URL-based meshes.

    This is a placeholder for future cloud integrations (e.g. AWS S3, HTTP).
    Currently raises a NotImplementedError if a network URI is detected.
    """

    def resolve(self, uri: str) -> Path:
        """Simulate network resolution."""
        if any(uri.startswith(p) for p in ("http://", "https://", "s3://")):
            # In a real implementation, this would download to a /tmp cache
            raise NotImplementedError(
                f"Network translation for '{uri}' is not yet implemented. "
                "The core IR supports the model, but the NetworkResolver requires a "
                "caching backend (planned for v1.5.0)."
            )

        # Fallback to standard filesystem if it's a local path
        return FileSystemResolver().resolve(uri)
