"""Path and resource resolution utilities for LinkForge."""

import os
from pathlib import Path


def resolve_package_path(uri: str, start_dir: Path) -> Path | None:
    """Resolve package:// URI by searching ROS_PACKAGE_PATH or upward in the tree.

    This enables LinkForge to work both in standard ROS environments and in
    standalone Blender workspaces without ROS installed.

    Args:
        uri: URI starting with package:// or package:/
        start_dir: Starting directory for upward search (usually URDF/XACRO directory)

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

    # Step 1: Check ROS_PACKAGE_PATH (ROS Mode)
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

    # Step 2: Fallback to upward search (Standalone Mode)
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
