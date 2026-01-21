"""Blender Operators for exporting robot models to URDF/XACRO.

This module implements the user-facing operators that handle the export of
robot models from Blender to URDF or XACRO formats.
"""

from __future__ import annotations

import os
from contextlib import contextmanager
from pathlib import Path

import bpy
from bpy.props import StringProperty
from bpy.types import Context, Event, Operator
from bpy_extras.io_utils import ExportHelper

from ...core.logging_config import get_logger

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


class LINKFORGE_OT_export_urdf(Operator, ExportHelper):
    """Export robot to URDF file"""

    bl_idname = "linkforge.export_urdf"
    bl_label = "Export URDF"
    bl_description = "Export robot to URDF/XACRO file"

    # ExportHelper properties
    filename_ext = ".urdf"
    filter_glob: StringProperty(  # type: ignore
        default="*.urdf;*.xacro",
        options={"HIDDEN"},
    )

    def invoke(self, context: Context, event: Event):
        """Invoked before the file browser opens."""
        # Update file extension based on export format
        robot_props = context.scene.linkforge
        if robot_props.export_format == "XACRO":
            self.filename_ext = ".xacro"
        else:
            self.filename_ext = ".urdf"

        # Call parent invoke to open file browser
        return ExportHelper.invoke(self, context, event)

    def execute(self, context: Context):
        """Execute the export."""
        # Import here to avoid circular dependencies
        from ...core import URDFGenerator, XACROGenerator
        from ..converters import scene_to_robot

        scene = context.scene
        robot_props = scene.linkforge

        # Prepare meshes directory if exporting meshes
        output_path = Path(self.filepath)

        # Ensure correct file extension matches format
        if robot_props.export_format == "XACRO" and output_path.suffix != ".xacro":
            output_path = output_path.with_suffix(".xacro")
        elif robot_props.export_format == "URDF" and output_path.suffix != ".urdf":
            output_path = output_path.with_suffix(".urdf")

        # Update self.filepath to reflect the corrected path
        self.filepath = str(output_path)

        # Always define meshes_dir so URDF can reference it
        meshes_dir = output_path.parent / robot_props.mesh_directory_name

        # Validate if requested
        if robot_props.validate_before_export:
            # First pass: Dry run to generate robot model without exporting meshes
            try:
                robot_dry_run, _ = scene_to_robot(context, meshes_dir=meshes_dir, dry_run=True)
            except Exception as e:
                self.report({"ERROR"}, f"Failed to build robot model: {e}")
                return {"CANCELLED"}

            from ...core.validation import RobotValidator

            validator = RobotValidator(robot_dry_run)
            result = validator.validate()

            if not result.is_valid:
                self.report(
                    {"ERROR"},
                    f"Cannot export: {result.error_count} validation error(s). "
                    f"Run validation to see details.",
                )
                return {"CANCELLED"}

        # Second pass: Actual export
        # If export_meshes is False, we run in dry_run mode (generate paths but don't write files)
        should_write_meshes = robot_props.export_meshes
        try:
            robot, _ = scene_to_robot(
                context,
                meshes_dir=meshes_dir,
                dry_run=not should_write_meshes,
            )
        except Exception as e:
            msg = str(e)
            # Make error more user friendly
            if "Unable to build robot model" in msg:
                parts = msg.split("\n", 1)
                if len(parts) > 1:
                    # Show just the first error summary line to keep popup clean
                    # The full log is usually too big for a toast notification
                    msg = "Configuration errors found. Check console or run Validation."

            self.report({"ERROR"}, f"Build failed: {msg}")
            return {"CANCELLED"}

        # Generate URDF/XACRO
        try:
            if robot_props.export_format == "URDF":
                urdf_generator = URDFGenerator(
                    pretty_print=True,
                    urdf_path=output_path,
                    use_ros2_control=robot_props.use_ros2_control,
                )
                urdf_generator.write(robot, output_path, validate=False)
                msg = f"Exported URDF to {output_path}"
                if meshes_dir:
                    msg += f" (meshes in {meshes_dir})"
                self.report({"INFO"}, msg)
                logger.info(msg)
            else:  # XACRO
                xacro_generator = XACROGenerator(
                    pretty_print=True,
                    advanced_mode=True,
                    extract_materials=robot_props.xacro_extract_materials,
                    extract_dimensions=robot_props.xacro_extract_dimensions,
                    generate_macros=robot_props.xacro_generate_macros,
                    split_files=robot_props.xacro_split_files,
                    urdf_path=output_path,
                    use_ros2_control=robot_props.use_ros2_control,
                )
                xacro_generator.write(robot, output_path, validate=False)
                msg = f"Exported XACRO to {output_path}"
                if meshes_dir:
                    msg += f" (meshes in {meshes_dir})"
                self.report({"INFO"}, msg)
                logger.info(msg)

            return {"FINISHED"}

        except Exception as e:
            self.report({"ERROR"}, f"Export failed: {e}")
            return {"CANCELLED"}


class LINKFORGE_OT_validate_robot(Operator):
    """Validate robot structure"""

    bl_idname = "linkforge.validate_robot"
    bl_label = "Validate Robot"
    bl_description = "Validate the robot structure for errors"

    def execute(self, context: Context):
        """Execute validation."""
        from ...core.validation import RobotValidator
        from ..converters import scene_to_robot

        # Clear previous results
        validation_props = context.window_manager.linkforge_validation
        validation_props.clear()

        # Convert scene to robot
        try:
            robot, _ = scene_to_robot(context)
        except Exception as e:
            # Catch all build errors and report them in UI
            validation_props.has_results = True
            validation_props.is_valid = False
            validation_props.error_count = 1
            validation_props.warning_count = 0

            # Create error entry
            error_prop = validation_props.errors.add()
            error_prop.title = "Configuration Error"

            # Clean up the error message
            msg = str(e)
            prefix = "Unable to build robot model."
            if prefix in msg:
                # Extract the part after the first line (which is the header)
                parts = msg.split("\n", 1)
                if len(parts) > 1:
                    msg = parts[1].strip()

            error_prop.message = msg
            error_prop.suggestion = "Review the errors above and check the object properties."

            self.report(
                {"ERROR"}, "Validation failed. Detailed errors are listed in the Validation Panel."
            )
            return {"FINISHED"}

        # Validate using new validator
        validator = RobotValidator(robot)
        result = validator.validate()

        # Store results in window manager
        validation_props.has_results = True
        validation_props.is_valid = result.is_valid
        validation_props.error_count = result.error_count
        validation_props.warning_count = result.warning_count
        validation_props.link_count = len(robot.links)
        validation_props.joint_count = len(robot.joints)
        validation_props.dof_count = robot.degrees_of_freedom

        # Store errors
        for error in result.errors:
            error_prop = validation_props.errors.add()
            error_prop.title = error.title
            error_prop.message = error.message
            error_prop.suggestion = error.suggestion or ""
            error_prop.affected_objects = ", ".join(error.affected_objects)

        # Store warnings
        for warning in result.warnings:
            warning_prop = validation_props.warnings.add()
            warning_prop.title = warning.title
            warning_prop.message = warning.message
            warning_prop.suggestion = warning.suggestion or ""
            warning_prop.affected_objects = ", ".join(warning.affected_objects)

        # Report result
        if result.is_valid and not result.has_warnings:
            self.report(
                {"INFO"},
                f"Robot '{robot.name}' is valid! "
                f"({len(robot.links)} links, {len(robot.joints)} joints, "
                f"{robot.degrees_of_freedom} DOF)",
            )
            return {"FINISHED"}
        elif result.is_valid:
            self.report(
                {"WARNING"},
                f"Robot valid with {result.warning_count} warning(s). Check Validation panel.",
            )
            return {"FINISHED"}
        else:
            self.report(
                {"ERROR"},
                f"Validation failed. Found {result.error_count} error(s). Please check the Validation Panel.",
            )
            return {"CANCELLED"}


# Registration
classes = [
    LINKFORGE_OT_export_urdf,
    LINKFORGE_OT_validate_robot,
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
