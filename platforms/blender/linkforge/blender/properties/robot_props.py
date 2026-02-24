"""Blender Property Groups for robot-level configuration.

This module defines the property groups used to store robot-level metadata
and export settings directly within the Blender Scene. These properties
drive the UI panels and provide the configuration parameters for the
URDF and XACRO generators, including:

- **Robot Metadata**: Name and global settings.
- **Export Configuration**: Target formats (URDF/XACRO) and validation toggles.
- **Advanced XACRO Settings**: Toggles for property extraction, macro generation,
  and modular file splitting.
"""

from __future__ import annotations

import contextlib

import bpy
from bpy.props import BoolProperty, CollectionProperty, EnumProperty, IntProperty, StringProperty
from bpy.types import PropertyGroup

from .control_props import Ros2ControlJointProperty, Ros2ControlParameterProperty


class RobotPropertyGroup(PropertyGroup):
    """Global robot properties stored on the Scene."""

    # Robot identification
    robot_name: StringProperty(  # type: ignore
        name="Robot Name",
        description="Name of your robot (used in URDF file)",
        default="my_robot",
        maxlen=64,
    )

    # Export settings
    export_format: EnumProperty(  # type: ignore
        name="Format",
        description="File format for export",
        items=[
            ("URDF", "URDF", "Unified Robot Description Format (XML)"),
            ("XACRO", "XACRO", "XACRO (XML Macros for URDF)"),
        ],
        default="URDF",
    )

    use_ros2_control: BoolProperty(  # type: ignore
        name="Generate ROS2 Control",
        description="Generate ros2_control tags from centralized system configuration",
        default=True,
    )

    ros2_control_name: StringProperty(  # type: ignore
        name="System Name",
        description="Name of the ros2_control system",
        default="GazeboSimSystem",
    )

    ros2_control_type: EnumProperty(  # type: ignore
        name="System Type",
        description="Type of the hardware system",
        items=[
            ("system", "System", "Full robot system with multiple joints"),
            ("actuator", "Actuator", "Single actuator system"),
            ("sensor", "Sensor", "Sensor-only system"),
        ],
        default="system",
    )

    hardware_plugin: StringProperty(  # type: ignore
        name="Hardware Plugin",
        description="ROS 2 Hardware Interface plugin name",
        default="gz_ros2_control/GazeboSimSystem",
    )

    gazebo_plugin_name: StringProperty(  # type: ignore
        name="Gazebo Plugin",
        description="Gazebo ros2_control plugin name",
        default="gz_ros2_control::GazeboSimROS2ControlPlugin",
    )

    controllers_yaml_path: StringProperty(  # type: ignore
        name="Controllers YAML",
        description="Path to controllers.yaml configuration file",
        default="$(find robot_description)/config/controllers.yaml",
        subtype="FILE_PATH",
    )

    # Centralized collection of controlled joints
    ros2_control_joints: CollectionProperty(type=Ros2ControlJointProperty)  # type: ignore
    ros2_control_active_joint_index: IntProperty()  # type: ignore

    # Global hardware parameters
    ros2_control_parameters: CollectionProperty(type=Ros2ControlParameterProperty)  # type: ignore
    show_ros2_control_parameters: BoolProperty(name="Show Parameters", default=True)  # type: ignore

    export_meshes: BoolProperty(  # type: ignore
        name="Export Meshes",
        description="Save 3D mesh files alongside URDF (needed for visualization and simulation)",
        default=True,
    )

    mesh_format: EnumProperty(  # type: ignore
        name="Mesh Format",
        description="3D file format for visual meshes (collision meshes always use STL)",
        items=[
            (
                "OBJ",
                "OBJ",
                "Wavefront OBJ with materials - recommended for Gazebo/RViz visualization",
            ),
            (
                "STL",
                "STL",
                "STereoLithography without materials - for simple geometry or 3D printing",
            ),
            (
                "GLB",
                "glTF Binary (.glb)",
                "Modern, efficient standard - best for web/Foxglove/Isaac Sim",
            ),
        ],
        default="OBJ",
    )

    mesh_directory_name: StringProperty(  # type: ignore
        name="Mesh Directory",
        description="Folder name where mesh files will be saved",
        default="meshes",
    )

    # Validation
    validate_before_export: BoolProperty(  # type: ignore
        name="Validate Before Export",
        description="Check for errors before exporting (recommended)",
        default=True,
    )

    strict_mode: BoolProperty(  # type: ignore
        name="Strict Mode",
        description=(
            "Fail immediately on first error instead of collecting all errors. "
            "Useful for debugging and automated pipelines"
        ),
        default=False,
    )

    # XACRO advanced settings
    xacro_advanced_mode: BoolProperty(  # type: ignore
        name="Show Advanced XACRO Settings",
        description="Show detailed options for XACRO generation",
        default=True,
    )

    xacro_extract_materials: BoolProperty(  # type: ignore
        name="Extract Materials",
        description="Convert materials to XACRO properties",
        default=True,
    )

    xacro_extract_dimensions: BoolProperty(  # type: ignore
        name="Extract Dimensions",
        description="Identify repeated dimensions (radius, length, size) and extract as properties",
        default=True,
    )

    xacro_generate_macros: BoolProperty(  # type: ignore
        name="Generate Macros",
        description="Identify similar links and group them into macros",
        default=False,
    )

    xacro_split_files: BoolProperty(  # type: ignore
        name="Split Files",
        description="Split output into _robot.xacro, _properties.xacro, and _macros.xacro",
        default=False,
    )

    # Visual helpers (kinematic tree display)
    show_kinematic_tree: BoolProperty(  # type: ignore
        name="Show Component Browser",
        description="Show list of all robot components in panel",
        default=False,
    )
    component_browser_search: StringProperty(  # type: ignore
        name="Component Browser Search",
        description="Filter components by name",
        default="",
        options={"TEXTEDIT_UPDATE"},
    )

    # Collision Visibility
    show_collisions: BoolProperty(  # type: ignore
        name="Show Collisions",
        description="Show/Hide all collision meshes in the viewport",
        default=False,
        update=update_collision_visibility,
    )

    # Background Import State (Pro UX)
    is_importing: BoolProperty(  # type: ignore
        name="Is Importing",
        description="True if a background import is currently active",
        default=False,
    )

    abort_import: BoolProperty(  # type: ignore
        name="Abort Import",
        description="Request cancellation of the current background import",
        default=False,
    )

    import_status: StringProperty(  # type: ignore
        name="Import Status",
        description="Current status message from the background importer",
        default="",
    )


def update_collision_visibility(self: RobotPropertyGroup, context: bpy.types.Context) -> None:
    """Update visibility of all collision meshes in the scene."""
    if not context or not context.scene:
        return

    show = self.show_collisions
    scene = context.scene

    for obj in scene.objects:
        # Check if object is a collision mesh
        # Criteria: Parent is a robot link AND name contains "_collision"
        if (
            obj.parent
            and hasattr(obj.parent, "linkforge")
            and obj.parent.linkforge.is_robot_link
            and "_collision" in obj.name.lower()
        ):
            obj.hide_viewport = not show


# Registration
def register() -> None:
    """Register property group."""
    try:
        bpy.utils.register_class(RobotPropertyGroup)
    except ValueError:
        # If already registered (e.g. from reload), unregister first to ensure clean state
        bpy.utils.unregister_class(RobotPropertyGroup)
        bpy.utils.register_class(RobotPropertyGroup)

    import typing

    typing.cast(typing.Any, bpy.types.Scene).linkforge = bpy.props.PointerProperty(
        type=RobotPropertyGroup
    )  # type: ignore[func-returns-value]


def unregister() -> None:
    """Unregister property group."""
    import typing

    with contextlib.suppress(AttributeError):
        del typing.cast(typing.Any, bpy.types.Scene).linkforge

    with contextlib.suppress(RuntimeError):
        bpy.utils.unregister_class(RobotPropertyGroup)


if __name__ == "__main__":
    register()
