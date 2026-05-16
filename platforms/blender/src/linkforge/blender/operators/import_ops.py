"""Blender Operators for importing robot models.

This module implements the user-facing operators that handle the import of
robot descriptions into the Blender environment.
"""

from __future__ import annotations

import typing
from contextlib import suppress
from pathlib import Path

import bpy
from bpy_extras.io_utils import ImportHelper
from linkforge.core import get_logger

from ..utils.decorators import OperatorReturn, safe_execute
from ..utils.scene_utils import clear_stats_cache

if typing.TYPE_CHECKING:
    from bpy.types import Context, Operator
else:
    # Runtime fallback for mock environments where bpy.types might be partially loaded.
    Context = typing.Any
    Operator = getattr(getattr(bpy, "types", object), "Operator", object)

logger = get_logger(__name__)


class LINKFORGE_OT_import_robot_model(Operator, ImportHelper):  # type: ignore[misc]
    """Import robot from URDF or XACRO file.

    This operator opens a file browser to select a robot description file,
    auto-detects the format (URDF or XACRO), validates the model structure,
    and initiates an asynchronous import process into the Blender scene.
    """

    bl_idname = "linkforge.import_robot_model"
    bl_label = "Import Robot Model"
    bl_description = "Import robot from supported formats (URDF, XACRO, etc.)"

    # Operator properties for ExportHelper/ImportHelper
    filepath: bpy.props.StringProperty(subtype="FILE_PATH")  # type: ignore
    filter_glob: bpy.props.StringProperty(  # type: ignore
        default="*.urdf;*.xacro;*.xml",
        options={"HIDDEN"},
        maxlen=255,
    )

    # Type ignore to resolve 'misc' definition collision with Operator.check
    def check(self, _context: Context) -> typing.Any:
        """Check if the operator can update its properties.

        Args:
            _context: The current Blender context (unused, required by API).

        Returns:
            True to indicate the properties have changed and the UI needs update.
        """
        return True

    @safe_execute
    def execute(self, context: Context) -> OperatorReturn:
        """Execute the robot import process.

        Args:
            context: The execution context.

        Returns:
            Set containing the execution state (e.g., {'FINISHED'} or {'CANCELLED'}).
        """
        from linkforge.core import URDFParser, clear_xacro_cache

        # Clear XACRO cache to ensure changes on disk are picked up
        clear_xacro_cache()

        # Parse URDF/XACRO file
        source_path = Path(self.filepath)

        # Smart Directory Handling
        # If user selects a folder, try to find the main robot file automatically.
        if source_path.is_dir():
            candidates = [
                source_path / f"{source_path.name}.urdf",
                source_path / f"{source_path.name}.xacro",
                source_path / f"{source_path.name}.urdf.xacro",
                source_path / "robot.urdf",
                source_path / "robot.xacro",
                source_path / "robot.urdf.xacro",
            ]

            found = [f for f in candidates if f.is_file()]
            valid_files = list(source_path.glob("*.urdf")) + list(source_path.glob("*.xacro"))

            if found:
                # Pick the first "best guess" match
                source_path = found[0]
                self.report({"INFO"}, f"Auto-detected robot description: {source_path.name}")
            elif len(valid_files) == 1:
                # If there's only one valid file in the folder, use it
                source_path = valid_files[0]
                self.report({"INFO"}, f"Auto-detected single robot file: {source_path.name}")
            else:
                self.report(
                    {"ERROR"},
                    "Directory selected but no obvious robot file found. Please select a .urdf or .xacro file directly.",
                )
                return {"CANCELLED"}

        # Validate that the path is now a file
        if not source_path.is_file():
            self.report({"ERROR"}, f"File not found: {source_path}")
            return {"CANCELLED"}

        is_xacro = source_path.suffix == ".xacro" or source_path.name.endswith(".urdf.xacro")

        # Detect Sandbox Root for security (allows sibling folders like meshes/)
        from linkforge.core.validation import find_sandbox_root

        sandbox_root = find_sandbox_root(source_path)
        logger.info(f"Importing robot from: {source_path}")
        logger.debug(f"Detected sandbox root: {sandbox_root}")

        # Smart Import Logic:
        # 1. If it looks like URDF, try parsing as URDF.
        # 2. If parsing fails because of Xacro tags, catch the error and switch to Xacro mode.
        from linkforge.core import FileSystemResolver, RobotParserError, XacroDetectedError

        # Read additional package paths from preferences
        from ..preferences import get_addon_prefs

        prefs = get_addon_prefs(context)
        additional_paths = []
        if prefs and hasattr(prefs, "additional_search_paths") and prefs.additional_search_paths:
            import os

            # Split by comma or os.pathsep (collapsing spaces)
            raw_paths = prefs.additional_search_paths.replace(",", os.pathsep).split(os.pathsep)
            additional_paths = [Path(p.strip()) for p in raw_paths if p.strip()]

        resolver = FileSystemResolver(additional_search_paths=additional_paths)

        try:
            if not is_xacro:
                try:
                    # Attempt standard URDF import
                    robot = URDFParser(sandbox_root=sandbox_root, resource_resolver=resolver).parse(
                        source_path
                    )
                except XacroDetectedError:
                    # Explicitly detected Xacro, enable fallback
                    self.report(
                        {"WARNING"},
                        "Detected XACRO content in robot model file. Switching to XACRO parser...",
                    )
                    is_xacro = True
                except RobotParserError as e:
                    # Real validation error
                    self.report({"ERROR"}, f"URDF Parsing failed: {e}")
                    return {"CANCELLED"}

            # XACRO PROCESSING (Triggered by extension OR fallback detection)
            if is_xacro:
                # Convert XACRO to URDF using native XacroResolver
                from linkforge.core import XacroResolver

                self.report({"INFO"}, f"Processing XACRO file: {source_path.name}")

                # Pass the additional paths so XACRO includes can find package:// references
                xacro_resolver = XacroResolver(search_paths=additional_paths)
                urdf_string = xacro_resolver.resolve_file(source_path)

                # Parse URDF string with directory for mesh path validation
                self.report({"INFO"}, "Parsing URDF...")
                robot = URDFParser(
                    sandbox_root=sandbox_root, resource_resolver=resolver
                ).parse_string(
                    urdf_string,
                    source_directory=source_path.parent,
                    default_name=source_path.stem,
                )
        except RobotParserError as e:
            self.report({"ERROR"}, f"Import failed: {e}")
            return {"CANCELLED"}
        except Exception as e:
            self.report({"ERROR"}, f"Unexpected internal error: {e}")
            logger.exception("Import process crashed")
            return {"CANCELLED"}

        if not robot.links and not robot.joints:
            self.report(
                {"ERROR"},
                f"The file '{source_path.name}' contains no links or joints. "
                "It may be a macro-only XACRO file. Please import the top-level robot description instead.",
            )
            return {"CANCELLED"}

        # Validate robot structure
        from linkforge.core import RobotValidator

        validator = RobotValidator()
        result = validator.validate(robot)

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
        from ..adapters.context import BlenderContext
        from ..logic.asynchronous_builder import AsynchronousRobotBuilder

        builder = AsynchronousRobotBuilder(robot, source_path, BlenderContext(context))
        builder.start()

        # We return FINISHED here, but the builder continues in the background via timers.
        # This is standard for long-running non-blocking tasks in Blender.
        file_type = "XACRO" if is_xacro else "URDF"
        self.report(
            {"INFO"},
            f"Started background import of {file_type}: '{robot.name}'...",
        )
        clear_stats_cache()
        return {"FINISHED"}


# Registration
classes = [
    LINKFORGE_OT_import_robot_model,
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
