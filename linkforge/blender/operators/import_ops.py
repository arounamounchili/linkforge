"""Blender Operators for importing robot models from URDF/XACRO.

This module implements the user-facing operators that handle the import of
robot descriptions into the Blender environment.
"""

from __future__ import annotations

import os
from contextlib import contextmanager
from pathlib import Path

import bpy
from bpy.props import StringProperty
from bpy.types import Context, Operator
from bpy_extras.io_utils import ImportHelper

from ...core.logging_config import get_logger
from ..utils.decorators import safe_execute

logger = get_logger(__name__)


@contextmanager
def working_directory(path: Path):
    """Context manager for temporarily changing the working directory."""
    old_cwd = os.getcwd()
    try:
        os.chdir(path)
        yield path
    finally:
        os.chdir(old_cwd)


class LINKFORGE_OT_import_urdf(Operator, ImportHelper):
    """Import robot from URDF or XACRO file"""

    bl_idname = "linkforge.import_urdf"
    bl_label = "Import Robot"
    bl_description = "Import robot from URDF or XACRO file (auto-detects format)"

    # ImportHelper properties
    filename_ext = ".urdf"
    filter_glob: StringProperty(  # type: ignore
        default="*.urdf;*.xacro;*.urdf.xacro",
        options={"HIDDEN"},
    )

    @safe_execute
    def execute(self, context: Context):
        """Execute the import."""
        from ...core.parsers.urdf_parser import parse_urdf, parse_urdf_string
        from ..scene_builder import import_robot_to_scene

        # Parse URDF/XACRO file
        urdf_path = Path(self.filepath)

        # Validate that the path is a file, not a directory
        if not urdf_path.is_file():
            if urdf_path.is_dir():
                self.report({"ERROR"}, f"Selected path is a directory, not a file: {urdf_path}")
            else:
                self.report({"ERROR"}, f"File not found: {urdf_path}")
            return {"CANCELLED"}

        # Detect if this is explicitly a XACRO file by extension
        is_xacro = urdf_path.suffix == ".xacro" or urdf_path.name.endswith(".urdf.xacro")

        # Smart Import Logic:
        # 1. If it looks like URDF, try parsing as URDF.
        # 2. If parsing fails because of Xacro tags, catch the error and switch to Xacro mode.
        if not is_xacro:
            try:
                # Attempt standard URDF import
                robot = parse_urdf(urdf_path)
            except ValueError as e:
                # Check if our parser detected hidden Xacro content
                if "XACRO file detected" in str(e):
                    self.report(
                        {"WARNING"},
                        "Detected XACRO content in .urdf file. Switching to XACRO parser...",
                    )
                    is_xacro = True  # Enable Xacro mode and fall through to the block below
                else:
                    # Real validation error, re-raise to be caught by outer try/except
                    raise

        # XACRO PROCESSING (Triggered by extension OR fallback detection)
        if is_xacro:
            # Convert XACRO to URDF using xacrodoc (bundled dependency)
            from xacrodoc import XacroDoc

            self.report({"INFO"}, f"Processing XACRO file: {urdf_path.name}")

            # Change to XACRO file's directory to resolve relative includes
            # xacrodoc resolves <xacro:include> paths relative to CWD
            urdf_string = None

            try:
                # Use context manager to safely change working directory
                with working_directory(urdf_path.parent):
                    doc = XacroDoc.from_file(urdf_path.name)
                    urdf_string = doc.to_urdf_string()
            except Exception as e:
                # Check if this is a PackageNotFoundError or XacroException wrapping it
                exception_name = type(e).__name__
                exception_str = str(e)

                is_package_error = exception_name == "PackageNotFoundError" or (
                    exception_name == "XacroException" and "PackageNotFoundError" in exception_str
                )

                if is_package_error:
                    # Extract package name from error message
                    package_name = "unknown"
                    if "PackageNotFoundError" in exception_str:
                        parts = exception_str.split(":")
                        if len(parts) > 1:
                            package_name = parts[-1].strip().split()[0]
                        else:
                            package_name = exception_str.split()[-1]

                    # Show professional error message with actionable guidance
                    error_msg = (
                        f"XACRO processing failed: Missing ROS package '{package_name}'.\n\n"
                        f"Solutions:\n"
                        f"1. Use the URDF version of this file instead.\n"
                        f"2. Install the required ROS package on your system.\n"
                        f"3. Edit the XACRO file to use relative paths instead of $(find ...)."
                    )
                    self.report({"ERROR"}, error_msg)
                    return {"CANCELLED"}
                else:
                    # Not a package error, re-raise
                    raise

            # Parse URDF string with directory for mesh path validation
            self.report({"INFO"}, "Parsing URDF...")
            robot = parse_urdf_string(urdf_string, urdf_directory=urdf_path.parent)

        # Import to scene
        success = import_robot_to_scene(robot, urdf_path, context)
        if success:
            # Sync collision visibility with current scene settings
            if hasattr(context.scene, "linkforge"):
                robot_props = context.scene.linkforge
                # Replicate simple visibility logic
                if not robot_props.show_collisions:
                    # Hide all collision meshes if toggle is off
                    for obj in context.scene.objects:
                        if (
                            obj.parent
                            and hasattr(obj.parent, "linkforge")
                            and obj.parent.linkforge.is_robot_link
                            and "_collision" in obj.name.lower()
                        ):
                            obj.hide_viewport = True

            file_type = "XACRO" if is_xacro else "URDF"
            self.report(
                {"INFO"},
                f"Imported {file_type}: '{robot.name}' "
                f"({len(robot.links)} links, {len(robot.joints)} joints)",
            )
            return {"FINISHED"}
        else:
            self.report({"ERROR"}, "Failed to import robot to scene")
            return {"CANCELLED"}


# Registration
classes = [
    LINKFORGE_OT_import_urdf,
]


def register():
    """Register operators."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister():
    """Unregister operators."""
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except RuntimeError:
            pass


if __name__ == "__main__":
    register()
