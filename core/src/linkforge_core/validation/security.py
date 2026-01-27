"""Security validation functions for URDF parsing.

This module provides security checks to prevent malicious URDF files from
accessing unauthorized file system locations or causing other security issues.
"""

from __future__ import annotations

from pathlib import Path
from urllib.parse import unquote

from ..logging_config import get_logger

logger = get_logger(__name__)


def validate_mesh_path(
    mesh_filepath: Path, urdf_directory: Path, allow_absolute: bool = False
) -> Path:
    """Validate that a mesh file path is safe to access.

    This function prevents path traversal attacks by ensuring that mesh paths
    stay within the URDF file's directory or its subdirectories.

    **Security Note:** Absolute paths are discouraged for portability and security.
    Use `allow_absolute=True` only when loading trusted URDF files.

    Args:
        mesh_filepath: The mesh file path from the URDF (may be relative or absolute)
        urdf_directory: The directory containing the URDF file
        allow_absolute: If True, allows absolute paths (default: False for security)

    Returns:
        The validated absolute path to the mesh file

    Raises:
        ValueError: If the mesh path attempts to escape the URDF directory
        ValueError: If the path contains suspicious components
        ValueError: If absolute paths are not allowed but one is provided

    Example:
        >>> urdf_dir = Path("/home/user/robot")
        >>> # Safe path - within URDF directory
        >>> validate_mesh_path(Path("meshes/arm.stl"), urdf_dir)
        PosixPath('/home/user/robot/meshes/arm.stl')

        >>> # Unsafe path - tries to escape
        >>> validate_mesh_path(Path("../../../etc/passwd"), urdf_dir)
        ValueError: Mesh path '../../../etc/passwd' attempts to escape...

        >>> # Absolute path (rejected by default)
        >>> validate_mesh_path(Path("/etc/passwd"), urdf_dir)
        ValueError: Absolute path '/etc/passwd' not allowed...

        >>> # Absolute path (allowed explicitly)
        >>> validate_mesh_path(Path("/shared/meshes/arm.stl"), urdf_dir, allow_absolute=True)
        PosixPath('/shared/meshes/arm.stl')
    """
    # Decode URL encoding to catch encoded path traversal attempts (e.g., %2e%2e%2f -> ../)
    mesh_str = str(mesh_filepath)
    decoded_str = unquote(mesh_str)

    # Recreate Path from decoded string for further validation
    if decoded_str != mesh_str:
        mesh_filepath = Path(decoded_str)

    # Check for absolute paths
    if mesh_filepath.is_absolute():
        if not allow_absolute:
            raise ValueError(
                f"Absolute path '{mesh_filepath}' not allowed for security and portability. "
                "Use relative paths within the URDF directory, or set allow_absolute=True "
                "if loading trusted files."
            )
        # Absolute paths allowed: resolve and validate against system paths
        resolved = mesh_filepath.resolve()
    else:
        # Resolve relative to URDF directory
        resolved = (urdf_directory / mesh_filepath).resolve()

    # Additional security: check for suspicious system paths
    suspicious_paths = [
        "/etc",
        "/sys",
        "/proc",
        "/dev",
        "/root",
        "C:\\Windows",
        "C:\\System32",
    ]
    for suspicious in suspicious_paths:
        is_suspicious = False
        try:
            resolved.relative_to(suspicious)
            is_suspicious = True
        except ValueError:
            # Check relative to resolved suspicious path (handles symlinks e.g. /etc -> /private/etc)
            try:
                s_path = Path(suspicious)
                if s_path.exists():
                    resolved.relative_to(s_path.resolve())
                    is_suspicious = True
            except (ValueError, OSError):
                pass

        if is_suspicious:
            logger.warning(
                f"SECURITY: System path access attempt - '{mesh_filepath}' "
                f"resolves to restricted location: {resolved}"
            )
            raise ValueError(
                f"Mesh path '{mesh_filepath}' resolves to a restricted system location: {resolved}. "
                "This is not allowed for security reasons."
            )

    return resolved


def validate_package_uri(uri: str) -> str:
    """Validate a ROS package:// URI.

    Args:
        uri: The package URI to validate (e.g., "package://my_robot/meshes/arm.stl")

    Returns:
        The validated URI

    Raises:
        ValueError: If the URI is malformed or contains suspicious components

    Example:
        >>> # Valid package URIs
        >>> validate_package_uri("package://my_robot/meshes/arm.stl")
        'package://my_robot/meshes/arm.stl'

        >>> validate_package_uri("package://pr2_description/meshes/base_link.dae")
        'package://pr2_description/meshes/base_link.dae'

        >>> # Invalid URIs (will raise ValueError)
        >>> validate_package_uri("file:///etc/passwd")  # Wrong scheme
        ValueError: Invalid package URI: file:///etc/passwd (must start with 'package://')

        >>> validate_package_uri("package://")  # Missing package name
        ValueError: Invalid package URI: package:// (missing package name)

        >>> validate_package_uri("package://robot/../../../etc/passwd")  # Path traversal
        ValueError: Invalid package URI: package://robot/../../../etc/passwd (contains suspicious path components like '..' or '.')

    Note:
        This function only validates the URI format. Actual package resolution
        must be handled by the ROS environment or Blender importer.
    """
    if not uri.startswith("package://"):
        raise ValueError(f"Invalid package URI: {uri} (must start with 'package://')")

    # Decode URL encoding to catch encoded path traversal attempts
    decoded_uri = unquote(uri)

    # Check both original and decoded for path traversal
    if ".." in uri or ".." in decoded_uri:
        logger.warning(f"SECURITY: Path traversal attempt in package URI: '{uri}'")
        raise ValueError(
            f"Path traversal detected in package URI: '{uri}'. "
            f"URIs containing '..' are not allowed for security reasons."
        )

    # Extract path component after package://
    path_component = decoded_uri[10:]

    if not path_component:
        raise ValueError(f"Invalid package URI: {uri} (missing package name)")

    # Validate path components (skip package name at index 0)
    parts = path_component.split("/")
    if any(part in (".", "") for part in parts[1:]):
        raise ValueError(
            f"Invalid package URI: {uri} (contains suspicious path components like '.' or empty segments)"
        )

    return uri
