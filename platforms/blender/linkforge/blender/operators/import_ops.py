"""Blender Operators for importing robot models from URDF/XACRO.

This module implements the user-facing operators that handle the import of
robot descriptions into the Blender environment.
"""

from __future__ import annotations

from contextlib import suppress
from pathlib import Path

import bpy
from bpy.types import Context, Operator
from bpy_extras.io_utils import ImportHelper

from ...linkforge_core.logging_config import get_logger
from ..utils.decorators import safe_execute

logger = get_logger(__name__)


class LINKFORGE_OT_import_urdf(Operator, ImportHelper):  # type: ignore[misc]
    """Import robot from URDF or XACRO file.

    This operator opens a file browser to select a robot description file,
    auto-detects the format (URDF or XACRO), validates the model structure,
    and initiates an asynchronous import process into the Blender scene.
    """

    bl_idname = "linkforge.import_urdf"
    bl_label = "Import Robot"
    bl_description = "Import robot from URDF or XACRO file (auto-detects format)"

    # Operator properties for ExportHelper/ImportHelper
    filepath: bpy.props.StringProperty(subtype="FILE_PATH")  # type: ignore
    filter_glob: bpy.props.StringProperty(  # type: ignore
        default="*.urdf;*.xacro;*.xml",
        options={"HIDDEN"},
        maxlen=255,
    )

    # Type ignore to resolve 'misc' definition collision with Operator.check
    def check(self, context: Context) -> bool:  # type: ignore
        """Check if the operator can update its properties.

        Args:
            context: The current Blender context.

        Returns:
            True to indicate the properties have changed and the UI needs update.
        """
        return True

    @safe_execute
    def execute(self, context: Context) -> set[str]:
        """Execute the robot import process.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        from ...linkforge_core.parsers import URDFParser

        # Parse URDF/XACRO file
        urdf_path = Path(self.filepath)

        # Smart Directory Handling
        # If user selects a folder, try to find the main robot file automatically.
        if urdf_path.is_dir():
            candidates = [
                urdf_path / f"{urdf_path.name}.urdf",
                urdf_path / f"{urdf_path.name}.xacro",
                urdf_path / f"{urdf_path.name}.urdf.xacro",
                urdf_path / "robot.urdf",
                urdf_path / "robot.xacro",
                urdf_path / "robot.urdf.xacro",
            ]

            found = [f for f in candidates if f.is_file()]
            valid_files = list(urdf_path.glob("*.urdf")) + list(urdf_path.glob("*.xacro"))

            if found:
                # Pick the first "best guess" match
                urdf_path = found[0]
                self.report({"INFO"}, f"Auto-detected robot description: {urdf_path.name}")
            elif len(valid_files) == 1:
                # If there's only one valid file in the folder, use it
                urdf_path = valid_files[0]
                self.report({"INFO"}, f"Auto-detected single robot file: {urdf_path.name}")
            else:
                self.report(
                    {"ERROR"},
                    "Directory selected but no obvious robot file found. Please select a .urdf or .xacro file directly.",
                )
                return {"CANCELLED"}

        # Validate that the path is now a file
        if not urdf_path.is_file():
            self.report({"ERROR"}, f"File not found: {urdf_path}")
            return {"CANCELLED"}

        is_xacro = urdf_path.suffix == ".xacro" or urdf_path.name.endswith(".urdf.xacro")

        # Detect Sandbox Root for security (allows sibling folders like meshes/)
        from ...linkforge_core.validation.security import find_sandbox_root

        sandbox_root = find_sandbox_root(urdf_path)
        logger.info(f"Importing robot from: {urdf_path}")
        logger.debug(f"Detected sandbox root: {sandbox_root}")

        # Smart Import Logic:
        # 1. If it looks like URDF, try parsing as URDF.
        # 2. If parsing fails because of Xacro tags, catch the error and switch to Xacro mode.
        from ...linkforge_core import RobotParserError, XacroDetectedError

        try:
            if not is_xacro:
                try:
                    # Attempt standard URDF import
                    robot = URDFParser(sandbox_root=sandbox_root).parse(urdf_path)
                except XacroDetectedError:
                    # Explicitly detected Xacro, enable fallback
                    self.report(
                        {"WARNING"},
                        "Detected XACRO content in .urdf file. Switching to XACRO parser...",
                    )
                    is_xacro = True
                except RobotParserError as e:
                    # Real validation error
                    self.report({"ERROR"}, f"URDF Parsing failed: {e}")
                    return {"CANCELLED"}

            # XACRO PROCESSING (Triggered by extension OR fallback detection)
            if is_xacro:
                # Convert XACRO to URDF using native XacroResolver
                from ...linkforge_core.parsers import XacroResolver

                self.report({"INFO"}, f"Processing XACRO file: {urdf_path.name}")

                resolver = XacroResolver()
                urdf_string = resolver.resolve_file(urdf_path)

                # Parse URDF string with directory for mesh path validation
                self.report({"INFO"}, "Parsing URDF...")
                robot = URDFParser(sandbox_root=sandbox_root).parse_string(
                    urdf_string,
                    urdf_directory=urdf_path.parent,
                    default_name=urdf_path.stem,
                )
        except RobotParserError as e:
            self.report({"ERROR"}, f"Import failed: {e}")
            return {"CANCELLED"}
        except Exception as e:
            self.report({"ERROR"}, f"Unexpected internal error: {e}")
            logger.exception("Import process crashed")
            return {"CANCELLED"}

        # Validate robot structure
        from ...linkforge_core.validation import RobotValidator

        validator = RobotValidator(robot)
        result = validator.validate()

        if not result.is_valid:
            # Report the most critical errors via popups/info bar
            for issue in result.errors[:2]:
                self.report({"WARNING"}, f"Validation Error: {issue.message}")

            self.report(
                {"WARNING"},
                f"Imported robot '{robot.name}' has {result.error_count} structural errors. "
                "Check the Validation panel for details.",
            )
        elif result.has_warnings:
            self.report(
                {"INFO"},
                f"Imported robot '{robot.name}' with {result.warning_count} warnings.",
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


def register() -> None:
    """Register operators."""
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister operators."""
    for cls in reversed(classes):
        with suppress(RuntimeError):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
