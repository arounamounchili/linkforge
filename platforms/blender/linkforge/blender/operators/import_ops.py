"""Blender Operators for importing robot models from URDF/XACRO.

This module implements the user-facing operators that handle the import of
robot descriptions into the Blender environment.
"""

from __future__ import annotations

import os
from contextlib import contextmanager, suppress
from pathlib import Path

import bpy
from bpy.props import StringProperty
from bpy.types import Context, Operator
from bpy_extras.io_utils import ImportHelper

from ...linkforge_core.logging_config import get_logger
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
        from ...linkforge_core.parsers import URDFParser

        # Parse URDF/XACRO file
        urdf_path = Path(self.filepath)

        # Validate that the path is a file, not a directory
        if not urdf_path.is_file():
            if urdf_path.is_dir():
                self.report({"ERROR"}, f"Selected path is a directory, not a file: {urdf_path}")
            else:
                self.report({"ERROR"}, f"File not found: {urdf_path}")
            return {"CANCELLED"}

        is_xacro = urdf_path.suffix == ".xacro" or urdf_path.name.endswith(".urdf.xacro")

        # Detect Sandbox Root for security (allows sibling folders like meshes/)
        from ...linkforge_core.validation.security import find_sandbox_root

        sandbox_root = find_sandbox_root(urdf_path)

        # Smart Import Logic:
        # 1. If it looks like URDF, try parsing as URDF.
        # 2. If parsing fails because of Xacro tags, catch the error and switch to Xacro mode.
        if not is_xacro:
            try:
                # Attempt standard URDF import
                robot = URDFParser(sandbox_root=sandbox_root).parse(urdf_path)
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
            # Convert XACRO to URDF using native XacroResolver
            from ...linkforge_core.parsers import XacroResolver

            self.report({"INFO"}, f"Processing XACRO file: {urdf_path.name}")

            urdf_string = None
            try:
                resolver = XacroResolver()
                urdf_string = resolver.resolve_file(urdf_path)
            except Exception as e:
                # Provide a professional error message for XACRO failures
                error_msg = f"XACRO processing failed: {e}\n\n"
                if "PackageNotFoundError" in str(e) or "$(find" in str(e):
                    error_msg += (
                        "Solutions:\n"
                        "1. Use the URDF version of this file instead.\n"
                        "2. Ensure all required ROS packages are in ROS_PACKAGE_PATH.\n"
                        "3. Edit the XACRO file to use relative paths instead of $(find ...)."
                    )
                self.report({"ERROR"}, error_msg)
                return {"CANCELLED"}

            # Parse URDF string with directory for mesh path validation
            self.report({"INFO"}, "Parsing URDF...")
            robot = URDFParser(sandbox_root=sandbox_root).parse_string(
                urdf_string, urdf_directory=urdf_path.parent
            )

        # Import to scene (Asynchronous)
        from ..logic.asynchronous_builder import AsynchronousRobotBuilder

        builder = AsynchronousRobotBuilder(robot, urdf_path, context)
        builder.start()

        # We return FINISHED here, but the builder continues in the background via timers.
        # This is standard for long-running non-blocking tasks in Blender.
        file_type = "XACRO" if is_xacro else "URDF"
        self.report(
            {"INFO"},
            f"Started background import of {file_type}: '{robot.name}'...",
        )
        return {"FINISHED"}


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
        with suppress(RuntimeError):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
