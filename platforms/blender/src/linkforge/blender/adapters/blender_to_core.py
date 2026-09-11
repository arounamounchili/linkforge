"""Converters between Blender properties and Core models.

These functions bridge the gap between Blender's property system
and LinkForge's core data models.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import bpy
from mathutils import Matrix

from ..constants import SUFFIX_VISUAL
from ..core import (
    GazeboElement,
    GazeboPlugin,
    Robot,
    RobotBuilder,
    RobotValidationError,
    ValidationErrorCode,
    ValidationResult,
    get_logger,
)
from ..core._utils.string_utils import sanitize_name as sanitize_name
from ..core.constants import DEFAULT_MATERIAL_RGBA
from ..utils.property_helpers import (
    get_joint_props,
    get_link_props,
    get_robot_props,
)
from . import translator
from .context import IBlenderContext
from .geometry_extractor import (
    detect_primitive_type,
    extract_mesh_triangles,
    get_object_geometry,
    get_object_material,
)

logger = get_logger(__name__)

__all__ = [
    "SceneToRobotTranslator",
    "detect_primitive_type",
    "extract_mesh_triangles",
    "get_object_geometry",
    "get_object_material",
    "sanitize_name",
    "scene_to_robot",
]


def _categorize_scene_objects(
    scene: Any,
) -> tuple[
    dict[str, Any],
    list[Any],
    list[Any],
    dict[str, tuple[str, Any]],
    tuple[str, Any] | None,
]:
    """Extract and categorize objects from Blender scene using scene_utils.

    Args:
        scene: Blender scene object

    Returns:
        Tuple of (link_objects, joint_objects, sensor_objects,
                 joints_map, root_link)
    """
    from ..utils.scene_utils import get_robot_statistics

    stats = get_robot_statistics(scene, force_refresh=True)
    return (
        stats.link_objects,
        stats.joint_objects,
        stats.sensor_objects,
        stats.joints_map,
        stats.root_link,
    )


def _calculate_link_frames(
    link_objects: dict[str, Any],
    joints_map: dict[str, tuple[str, Any]],
    root_link: tuple[str, Any] | None,
) -> dict[str, Any]:
    """Calculate coordinate frames for all links in the kinematic tree.

    Args:
        link_objects: Dictionary of link names to Blender objects
        joints_map: Mapping of child links to (parent, joint_object) tuples
        root_link: Tuple of (root_link_name, root_link_object)

    Returns:
        Dictionary mapping link names to their world transformation matrices
    """
    link_frames = {}  # link_name -> world matrix where link frame is

    if root_link is not None and Matrix is not None:
        root_name, root_obj = root_link
        link_frames[root_name] = Matrix.Identity(4)

        root_world = root_obj.matrix_world.copy()
        root_translation = root_world.to_translation()
        root_rotation = root_world.to_quaternion()
        root_transform = Matrix.Translation(root_translation) @ root_rotation.to_matrix().to_4x4()
        root_world_transform_inv = root_transform.inverted()

        def calc_child_frames(parent_name: str) -> None:
            """Recursively calculate child link coordinate frames."""
            for child_name, (parent, _joint_obj) in joints_map.items():
                if parent == parent_name and child_name not in link_frames:
                    child_obj = link_objects.get(child_name)
                    if child_obj:
                        child_world = child_obj.matrix_world.copy()
                        child_translation = child_world.to_translation()
                        child_rotation = child_world.to_quaternion()
                        child_transform = (
                            Matrix.Translation(child_translation)
                            @ child_rotation.to_matrix().to_4x4()
                        )
                        child_frame = root_world_transform_inv @ child_transform
                        link_frames[child_name] = child_frame
                        calc_child_frames(child_name)

        calc_child_frames(root_name)

        # Calculate frames for any disconnected links/islands in the scene relative to the primary root
        for name, obj in link_objects.items():
            if name not in link_frames:
                obj_world = obj.matrix_world.copy()
                obj_translation = obj_world.to_translation()
                obj_rotation = obj_world.to_quaternion()
                obj_transform = (
                    Matrix.Translation(obj_translation) @ obj_rotation.to_matrix().to_4x4()
                )
                link_frames[name] = root_world_transform_inv @ obj_transform
                calc_child_frames(name)

    return link_frames


class SceneToRobotTranslator:
    """Orchestrates the conversion of a Blender scene to a Core Robot model.

    This class follows the SOLID principles by encapsulating the translation logic
    and leveraging the RobotBuilder (Composer) API for structural integrity.
    """

    def __init__(
        self,
        context: IBlenderContext,
        meshes_dir: Path | None = None,
        dry_run: bool = False,
        depsgraph: Any | None = None,
    ):
        self.context = context
        self.meshes_dir = meshes_dir
        self.dry_run = dry_run
        self.depsgraph = depsgraph

        # Get robot properties from scene
        self.robot_props = get_robot_props(context.scene)
        if not self.robot_props:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND, "Scene has no LinkForge properties"
            )

        self.robot_name = self.robot_props.robot_name if self.robot_props.robot_name else "robot"
        self.builder = RobotBuilder(self.robot_name)
        self.validation_result = ValidationResult(robot_name=self.robot_name)

    def translate(self, raise_on_error: bool = True) -> tuple[Robot, ValidationResult]:
        """Perform the translation and return the built Robot model."""
        # Categorize scene objects
        link_objects, joint_objects, sensor_objects, joints_map, root = _categorize_scene_objects(
            self.context.scene
        )

        # Validate joint definitions (parent/child references, self-loops, duplicates)
        self._validate_joint_definitions(joint_objects)

        # Calculate coordinate frames (needed for joint relative origins)
        link_frames = _calculate_link_frames(link_objects, joints_map, root)

        # Translate Materials globally (Centralized management)
        self._translate_global_materials(link_objects)

        # Build Kinematic Tree recursively (The "Composer" way)
        if root:
            root_name, _ = root
            self._build_link_recursive(root_name, None, link_objects, joints_map, link_frames)

            # Translate any orphaned links/islands that are not connected to the main root's tree
            for orphaned_link_name in link_objects:
                if not self.builder.robot.has_link(orphaned_link_name):
                    self._build_link_recursive(
                        orphaned_link_name, None, link_objects, joints_map, link_frames
                    )
        else:
            self.validation_result.add_error(
                title="No root link",
                message="No root link found in scene. Ensure at least one link has no parent joint.",
                code=ValidationErrorCode.NO_ROOT,
            )

        # Translate orphaned components (Sensors)
        self._translate_sensors(sensor_objects, link_frames)
        self._translate_ros2_control()
        self._translate_scene_gazebo_plugins()

        # Finalize and return
        try:
            robot = self.builder.build(validate=False)
        except Exception as e:
            self.validation_result.add_error(
                title="Build failed", message=str(e), code=ValidationErrorCode.INVALID_VALUE
            )
            robot = Robot(name=self.robot_name)

        if raise_on_error and self.validation_result.errors:
            first_err = self.validation_result.errors[0]
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Multiple configuration errors found ({len(self.validation_result.errors)}). First: {first_err.title} - {first_err.message}",
            )

        return robot, self.validation_result

    def _validate_joint_definitions(self, joint_objects: list[Any]) -> None:
        """Validate that all robot joints in the scene have valid parent and child links."""
        seen_children: dict[str, str] = {}

        for joint_obj in joint_objects:
            props = get_joint_props(joint_obj)
            if not props or not getattr(props, "is_robot_joint", False):
                continue

            joint_name = props.joint_name if props.joint_name else joint_obj.name
            parent_obj = props.parent_link
            child_obj = props.child_link

            # Validate parent link reference
            if not parent_obj:
                self.validation_result.add_error(
                    title="Missing Parent Link",
                    message=f"Joint '{joint_name}' has no parent link assigned.",
                    code=ValidationErrorCode.NOT_FOUND,
                    affected_objects=[joint_name],
                    suggestion=f"Assign a parent link to joint '{joint_name}' in the Joint panel.",
                )
            else:
                parent_props = get_link_props(parent_obj)
                parent_is_link = bool(
                    parent_props and getattr(parent_props, "is_robot_link", False)
                )
                if not parent_is_link:
                    self.validation_result.add_error(
                        title="Invalid Parent Link",
                        message=(
                            f"Joint '{joint_name}' references parent object '{parent_obj.name}', "
                            "which is not configured as a robot link."
                        ),
                        code=ValidationErrorCode.INVALID_VALUE,
                        affected_objects=[joint_name, parent_obj.name],
                        suggestion=f"Configure '{parent_obj.name}' as a robot link or select a valid parent link.",
                    )

            # Validate child link reference
            if not child_obj:
                self.validation_result.add_error(
                    title="Missing Child Link",
                    message=f"Joint '{joint_name}' has no child link assigned.",
                    code=ValidationErrorCode.NOT_FOUND,
                    affected_objects=[joint_name],
                    suggestion=f"Assign a child link to joint '{joint_name}' in the Joint panel.",
                )
            else:
                child_props = get_link_props(child_obj)
                child_is_link = bool(child_props and getattr(child_props, "is_robot_link", False))
                if not child_is_link:
                    self.validation_result.add_error(
                        title="Invalid Child Link",
                        message=(
                            f"Joint '{joint_name}' references child object '{child_obj.name}', "
                            "which is not configured as a robot link."
                        ),
                        code=ValidationErrorCode.INVALID_VALUE,
                        affected_objects=[joint_name, child_obj.name],
                        suggestion=f"Configure '{child_obj.name}' as a robot link or select a valid child link.",
                    )

            # Validate self-referencing connection
            if parent_obj and child_obj and parent_obj == child_obj:
                self.validation_result.add_error(
                    title="Self-Referencing Joint",
                    message=f"Joint '{joint_name}' connects link '{parent_obj.name}' to itself.",
                    code=ValidationErrorCode.HAS_CYCLE,
                    affected_objects=[joint_name, parent_obj.name],
                    suggestion=f"Select different links for parent and child on joint '{joint_name}'.",
                )

            # Validate duplicate child link assignment
            if child_obj:
                child_key = child_obj.name
                if child_key in seen_children:
                    self.validation_result.add_error(
                        title="Duplicate Child Link Assignment",
                        message=(
                            f"Child link '{child_key}' is assigned to multiple joints: "
                            f"'{seen_children[child_key]}' and '{joint_name}'."
                        ),
                        code=ValidationErrorCode.INVALID_VALUE,
                        affected_objects=[joint_name, seen_children[child_key], child_key],
                        suggestion="Ensure each link is the child of at most one joint.",
                    )
                else:
                    seen_children[child_key] = joint_name

    def _translate_global_materials(self, link_objects: dict[str, Any]) -> None:
        """Collect and register all unique materials used in the robot."""
        processed_mats = set()
        for link_obj in link_objects.values():
            props = get_link_props(link_obj)
            if props and props.use_material:
                for child in link_obj.children:
                    if SUFFIX_VISUAL in child.name and child.type == "MESH":
                        mat = get_object_material(child, props)
                        if mat and mat.name not in processed_mats:
                            # Register material in the robot model to satisfy LinkBuilder validation
                            if mat.name not in self.builder.robot.materials:
                                self.builder.robot.materials[mat.name] = mat
                            # Register with builder
                            color_tuple = (
                                (mat.color.r, mat.color.g, mat.color.b, mat.color.a)
                                if mat.color
                                else DEFAULT_MATERIAL_RGBA
                            )
                            self.builder.material(mat.name, color=color_tuple)
                            processed_mats.add(mat.name)

    def _build_link_recursive(
        self,
        link_name: str,
        parent_lb: Any,
        link_objects: dict[str, Any],
        joints_map: dict[str, tuple[str, Any]],
        link_frames: dict[str, Any],
    ) -> None:
        """Recursively build links and joints using specialized translators."""
        if link_name not in link_objects:
            return

        obj = link_objects[link_name]

        try:
            # Start link in composer
            if parent_lb is None:
                lb = self.builder.link(link_name)
            else:
                joint_info = joints_map.get(link_name)
                if not joint_info:
                    return
                _parent_name, joint_obj = joint_info
                joint_props = get_joint_props(joint_obj)
                joint_name = joint_props.joint_name if joint_props else joint_obj.name
                lb = parent_lb.child(link_name, joint_name=joint_name)

                # Configure Joint
                joint_translator = translator.JointTranslator()
                joint_translator.translate(
                    obj=joint_obj,
                    lb=lb,
                    link_frames=link_frames,
                )

            # Configure Link
            link_translator = translator.LinkTranslator()
            link_translator.translate(
                obj=obj,
                builder=self.builder,
                context=self.context,
                meshes_dir=self.meshes_dir,
                dry_run=self.dry_run,
                depsgraph=self.depsgraph,
                validation_result=self.validation_result,
                lb=lb,
            )

            # Recurse to children
            for child_name, (p_name, _j_obj) in joints_map.items():
                if p_name == link_name:
                    self._build_link_recursive(
                        child_name, lb, link_objects, joints_map, link_frames
                    )

            # Commit link
            lb.commit()

        except Exception as e:
            if self.robot_props and getattr(self.robot_props, "strict_mode", False):
                raise
            self.validation_result.add_error(
                title=f"Link translation failed: {link_name}",
                message=str(e),
                code=ValidationErrorCode.INVALID_VALUE,
                affected_objects=[link_name],
            )

    def _translate_sensors(self, sensor_objects: list[Any], link_frames: dict[str, Any]) -> None:
        """Translate sensors using specialized SensorTranslator."""

        sensor_translator = translator.SensorTranslator()
        for obj in sensor_objects:
            sensor_translator.translate(
                obj=obj,
                builder=self.builder,
                validation_result=self.validation_result,
                link_frames=link_frames,
            )

    def _translate_ros2_control(self) -> None:
        """Translate ROS2 Control settings from robot properties."""
        if self.robot_props and getattr(self.robot_props, "use_ros2_control", False):
            ros2_translator = translator.Ros2ControlTranslator()
            ros2_translator.translate(
                obj=self.robot_props,
                builder=self.builder,
                validation_result=self.validation_result,
            )

    def _translate_scene_gazebo_plugins(self) -> None:
        """Translate scene-level Gazebo plugins (e.g. ros2_control or custom)."""
        if not self.robot_props:
            return

        plugin_filename = getattr(self.robot_props, "gazebo_plugin_name", "")
        if not plugin_filename:
            return

        params = {}
        is_standard_control = (
            "gz_ros2_control" in plugin_filename or "gazebo_ros2_control" in plugin_filename
        )

        # Add controllers YAML if ros2_control is active
        if getattr(self.robot_props, "use_ros2_control", False):
            # Special case for standard gz_ros2_control: only add if we actually have joints to control
            if is_standard_control and not self.builder.robot.ros2_controls:
                return

            yaml_path = getattr(self.robot_props, "controllers_yaml_path", "")
            if yaml_path:
                params["parameters"] = yaml_path
        elif is_standard_control:
            # If standard control is NOT used, don't add the plugin at all
            return

        # Determine plugin name
        # For standard gz_ros2_control, we use 'gazebo_ros2_control' for compatibility
        if "gz_ros2_control" in plugin_filename or "gazebo_ros2_control" in plugin_filename:
            name = "gazebo_ros2_control"
        else:
            # Custom plugin: use filename as name (matches test expectation)
            name = plugin_filename

        gazebo_plugin = GazeboPlugin(
            name=name,
            filename=plugin_filename,
            parameters=params,
        )

        self.builder.robot.add_gazebo_element(GazeboElement(plugins=[gazebo_plugin]))


def scene_to_robot(
    context: IBlenderContext | bpy.types.Context,
    meshes_dir: Path | None = None,
    dry_run: bool = False,
    raise_on_error: bool = True,
) -> tuple[Robot, ValidationResult]:
    """Convert entire Blender scene to Core Robot using the Translator orchestrator."""
    from .context import BlenderContext

    # Auto-wrap raw context if passed directly
    if not isinstance(context, IBlenderContext):
        import bpy

        context = BlenderContext(bpy)

    translator = SceneToRobotTranslator(context, meshes_dir, dry_run)
    return translator.translate(raise_on_error=raise_on_error)
