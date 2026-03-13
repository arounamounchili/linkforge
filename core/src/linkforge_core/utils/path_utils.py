"""Path and resource resolution utilities for LinkForge."""

import os
from pathlib import Path


def resolve_package_path(
    uri: str, start_dir: Path, additional_search_paths: list[Path] | None = None
) -> Path | None:
    """Resolve package:// URI by searching ROS_PACKAGE_PATH or upward in the tree.

    This enables LinkForge to work both in standard ROS environments and in
    standalone Blender workspaces without ROS installed.

    Args:
        uri: URI starting with package:// or package:/
        start_dir: Starting directory for upward search (usually URDF/XACRO directory)
        additional_search_paths: Optional list of fallback directories to check first.

    Returns:
        Path to the resolved resource or None
    """
    if "package://" in uri:
        path_remainder = uri.replace("package://", "")
    elif "package:/" in uri:
        path_remainder = uri.replace("package:/", "")
    elif uri.startswith("package:"):
        path_remainder = uri.replace("package:", "")
    else:
        return None

    if not path_remainder:
        return None

    parts = path_remainder.split("/")
    package_name = parts[0]
    relative_path = "/".join(parts[1:])

    # Check provided additional search paths first (highest priority for overrides)
    if additional_search_paths:
        for p in additional_search_paths:
            pkg_base = Path(p)
            pkg_path = pkg_base / package_name
            if pkg_path.exists():
                return pkg_path / relative_path

            # Also check if we ARE inside the package base already
            if pkg_base.name == package_name:
                return pkg_base / relative_path

    # Check ROS_PACKAGE_PATH (ROS Mode)
    ros_path = os.environ.get("ROS_PACKAGE_PATH")
    if ros_path:
        for path in ros_path.split(os.pathsep):
            pkg_base = Path(path)
            # Some paths in ROS_PACKAGE_PATH might be empty or invalid
            if not path.strip():
                continue

            pkg_path = pkg_base / package_name
            if pkg_path.exists():
                return pkg_path / relative_path

            # Also check if we ARE inside the package base already (e.g. if path is the pkg itself)
            if pkg_base.name == package_name:
                return pkg_base / relative_path

    # Fallback to upward search (Standalone Mode)
    # This searches for a folder matching the package name or a package.xml
    curr = start_dir.resolve()
    if curr.is_file():
        curr = curr.parent

    # Search up to 10 levels or root
    for _ in range(10):
        # Case A: Current folder name matches package name
        if curr.name == package_name:
            return curr / relative_path

        # Case B: Current folder contains package.xml
        # In standalone mode, we treat any folder with package.xml as a potential package root
        # if Case A missed it (e.g. folder was renamed to 'franka_description-main')
        if (curr / "package.xml").exists():
            # Heuristic: If we are looking for a package and find A package.xml,
            # it's very likely the root we want in a standalone directory structure.
            # (Ideally we'd parse the name from XML, but this is a robust pro fallback)
            return curr / relative_path

        if curr.parent == curr:
            break
        curr = curr.parent

    return None


def normalize_uri_to_path(uri: str) -> Path:
    """Normalize a resource URI (specifically file://) to a local filesystem Path.

    Handles Windows-style file:// URIs (e.g. file:///C:/) by stripping the
    leading slash if a drive letter is detected. This ensures portability across
    Posix and Windows environments.

    Args:
        uri: The URI to normalize (e.g. file:///path/mesh.stl).

    Returns:
        A Path object representing the local path.
    """
    if uri.startswith("file://"):
        # Strip scheme (file://)
        scheme = "file://"
        path_str = uri[len(scheme) :]
        # Windows handling: /C:/ -> C:/ (strip leading slash before drive letter)
        if path_str.startswith("/") and len(path_str) > 2 and path_str[2] == ":":
            path_str = path_str[1:]
        return Path(path_str)

    return Path(uri)


def get_export_path(resource: str, relative_to: Path | None = None) -> str:
    """Prepare a resource string for export in URDF/XACRO.

    Ensures that URIs (package://, file://) are preserved correctly and not
    mangled by standard Path normalization. If a base directory is provided,
    local paths and file:// URIs are converted to relative paths where possible.

    Args:
        resource: The resource URI or path string.
        relative_to: Optional base directory to make paths relative to.

    Returns:
        The string to be used in the 'filename' attribute.
    """
    # 1. Preserve package:// URIs (never make relative)
    if resource.startswith("package://") or resource.startswith("package:/"):
        return resource

    # 2. Handle file:// URIs
    if resource.startswith("file://"):
        path = normalize_uri_to_path(resource)
        if relative_to and path.is_absolute():
            try:
                # Use .absolute() on relative_to just in case it's not
                rel = path.relative_to(relative_to.absolute())
                return str(rel)
            except ValueError:
                pass
        return resource

    # 3. Handle standard paths
    path = Path(resource)
    if relative_to and path.is_absolute():
        try:
            rel = path.relative_to(relative_to.absolute())
            return str(rel)
        except ValueError:
            pass

    return resource
