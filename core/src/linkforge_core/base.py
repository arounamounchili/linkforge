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

    def resolve(self, uri: str, relative_to: Path | None = None) -> Path:
        """Resolve a URI to a local filesystem Path.

        Args:
            uri: The resource URI to resolve.
            relative_to: Optional base directory for relative path resolution.

        Returns:
            The resolved absolute Path.

        Raises:
            FileNotFoundError: If the resource cannot be located.
        """
        ...


class FileSystemResolver:
    """Default resolver that handles standard file paths and package:// URIs."""

    def resolve(self, uri: str, relative_to: Path | None = None) -> Path:
        """Resolve standard file paths, file:// URIs, and package:// URIs."""
        from .utils.path_utils import resolve_package_path

        # 1. Handle package:// URIs
        if "package://" in uri or "package:/" in uri:
            # We use an empty Path if relative_to is not provided,
            # though package resolution usually doesn't strictly need it if ROS_PACKAGE_PATH is set.
            resolved = resolve_package_path(uri, relative_to or Path.cwd())
            if resolved and resolved.exists():
                return resolved.absolute()
            raise FileNotFoundError(f"Could not resolve package resource: {uri}")

        # 2. Handle file:// URIs
        if uri.startswith("file://"):
            from .utils.path_utils import normalize_uri_to_path

            path = normalize_uri_to_path(uri)
            if path.exists():
                return path.absolute()
            raise FileNotFoundError(f"Could not resolve file URI: {uri}")

        # 3. Handle standard paths (absolute or relative)
        path = Path(uri)
        if path.is_absolute():
            if path.exists():
                return path.absolute()
        elif relative_to is not None:
            # Try relative to the provided directory
            rel_path = (relative_to / path).resolve()
            if rel_path.exists():
                return rel_path

        # Final fallback: current working directory if it exists there
        if path.exists():
            return path.absolute()

        raise FileNotFoundError(f"Could not resolve resource: {uri} (relative_to={relative_to})")


class NetworkResolver:
    """Mock network resolver for URL-based meshes.

    This is a placeholder for future cloud integrations (e.g. AWS S3, HTTP).
    Currently raises a NotImplementedError if a network URI is detected.
    """

    def resolve(self, uri: str, relative_to: Path | None = None) -> Path:
        """Simulate network resolution."""
        if any(uri.startswith(p) for p in ("http://", "https://", "s3://")):
            # In a real implementation, this would download to a /tmp cache
            raise NotImplementedError(
                f"Network translation for '{uri}' is not yet implemented. "
                "The core IR supports the model, but the NetworkResolver requires a "
                "caching backend (planned for v1.5.0)."
            )

        # Fallback to standard filesystem if it's a local path
        return FileSystemResolver().resolve(uri, relative_to=relative_to)
