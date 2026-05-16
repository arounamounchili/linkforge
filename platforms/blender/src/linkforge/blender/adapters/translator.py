"""Protocols and base classes for scene translation.

This module defines the interfaces for mapping Blender scene data
to LinkForge core models using the Composer API.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

from linkforge.core import (
    CameraInfo,
    ContactInfo,
    ForceTorqueInfo,
    GazeboPlugin,
    GPSInfo,
    IMUInfo,
    InertiaTensor,
    JointType,
    LidarInfo,
    LinkBuilder,
    RobotBuilder,
    RobotValidationError,
    Ros2Control,
    Ros2ControlJoint,
    Sensor,
    SensorNoise,
    SensorType,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    TransmissionType,
    ValidationErrorCode,
    ValidationResult,
    get_logger,
    validate_mesh_topology,
)
from linkforge.core._utils.string_utils import sanitize_name
from linkforge.core.constants import (
    CONTROL_TYPE_ACTUATOR,
    CONTROL_TYPE_SENSOR,
    CONTROL_TYPE_SYSTEM,
    DEFAULT_AXIS_XYZ,
    HW_IF_EFFORT,
    HW_IF_POSITION,
    HW_IF_VELOCITY,
    SYLVESTER_TOLERANCE_EPSILON,
    TRANS_CUSTOM,
    TRANS_DIFFERENTIAL,
    TRANS_FOUR_BAR,
    TRANS_SIMPLE,
)

from ..constants import (
    FORMAT_STL,
    GEOM_AUTO,
    SUFFIX_COLLISION,
    SUFFIX_VISUAL,
    TAG_IMPORTED_SOURCE,
    TAG_SOURCE_NAME,
)

if TYPE_CHECKING:
    from .context import IBlenderContext

from ..utils.property_helpers import (
    get_joint_props,
    get_link_props,
    get_robot_props,
    get_sensor_props,
    get_transmission_props,
)

logger = logging.getLogger(__name__)


@runtime_checkable
class ITranslator(Protocol):
    """Base protocol for translating Blender objects to Core models."""

    def translate(
        self,
        obj: Any,
        builder: RobotBuilder,
        context: IBlenderContext,
        meshes_dir: Path | None = None,
        dry_run: bool = False,
        depsgraph: Any | None = None,
        validation_result: ValidationResult | None = None,
    ) -> Any:
        """Translate a Blender object using the provided builder."""
        ...


class TranslationRegistry:
    """Registry for managing specialized translators for different component types."""

    def __init__(self) -> None:
        self._translators: dict[str, ITranslator] = {}

    def register(self, component_type: str, translator: ITranslator) -> None:
        """Register a translator for a specific component type."""
        self._translators[component_type] = translator

    def get(self, component_type: str) -> ITranslator | None:
        """Retrieve a translator for a component type."""
        return self._translators.get(component_type)


class LinkTranslator(ITranslator):
    """Translates Blender objects marked as robot links."""

    def translate(
        self,
        obj: Any,
        builder: RobotBuilder,
        context: IBlenderContext,
        meshes_dir: Path | None = None,
        dry_run: bool = False,
        depsgraph: Any | None = None,
        validation_result: ValidationResult | None = None,
        lb: LinkBuilder | None = None,
        **_kwargs: Any,
    ) -> LinkBuilder | None:
        """Translate a Blender link to a Core Link using RobotBuilder."""
        from .blender_to_core import (
            get_object_geometry,
            get_object_material,
            matrix_to_transform,
        )

        props = get_link_props(obj)
        if not props:
            return None

        link_name = props.link_name if props.link_name else obj.name
        robot_props = get_robot_props(context.scene)
        mesh_format = robot_props.mesh_format if robot_props else FORMAT_STL

        # Use provided LinkBuilder or create a new one
        active_lb = lb if lb else builder.link(link_name)

        # 1. Translate visuals
        for child in obj.children:
            if SUFFIX_VISUAL in child.name:
                mat = get_object_material(child, props)
                suffix = self._get_geom_suffix(child, obj, SUFFIX_VISUAL, sanitize_name)

                geom, world_mat = get_object_geometry(
                    child,
                    GEOM_AUTO,
                    link_name,
                    "visual",
                    meshes_dir,
                    mesh_format,
                    dry_run=dry_run,
                    suffix=suffix,
                    depsgraph=depsgraph,
                )
                if mat and mat.name not in builder.robot.materials:
                    # Register material in the robot model to satisfy LinkBuilder validation
                    builder.robot.materials[mat.name] = mat

                if geom:
                    rel_mat = obj.matrix_world.inverted() @ world_mat
                    origin = matrix_to_transform(rel_mat)
                    active_lb.visual(
                        geom,
                        xyz=origin.xyz.to_tuple(),
                        rpy=origin.rpy.to_tuple(),
                        material=mat.name if mat else None,
                        name=child.get(TAG_SOURCE_NAME),
                    )
                    # Mesh Topology Validation
                    self._validate_mesh(
                        child, link_name, "visual", validation_result, depsgraph=depsgraph
                    )

        # 2. Translate collisions
        for child in obj.children:
            if SUFFIX_COLLISION in child.name:
                suffix = self._get_geom_suffix(child, obj, SUFFIX_COLLISION, sanitize_name)
                quality = props.collision_quality / 100.0
                is_imported = child.get(TAG_IMPORTED_SOURCE, False)

                geom, world_mat = get_object_geometry(
                    child,
                    GEOM_AUTO,
                    link_name,
                    "collision",
                    meshes_dir,
                    FORMAT_STL,  # Collisions always use STL for maximum physics compatibility
                    simplify=(quality < 1.0) and not is_imported,
                    decimation_ratio=quality,
                    dry_run=dry_run,
                    suffix=suffix,
                    depsgraph=depsgraph,
                )
                if geom:
                    rel_mat = obj.matrix_world.inverted() @ world_mat
                    origin = matrix_to_transform(rel_mat)
                    active_lb.collision(
                        geom,
                        xyz=origin.xyz.to_tuple(),
                        rpy=origin.rpy.to_tuple(),
                        name=child.get(TAG_SOURCE_NAME),
                    )
                    # Mesh Topology Validation
                    self._validate_mesh(
                        child, link_name, "collision", validation_result, depsgraph=depsgraph
                    )

        # 3. Translate Physics (Inertia & Mass)
        if (lp := get_link_props(obj)) and lp.use_auto_inertia:
            active_lb.mass(lp.mass)
        else:
            inertia = InertiaTensor(
                ixx=props.inertia_ixx,
                ixy=props.inertia_ixy,
                ixz=props.inertia_ixz,
                iyy=props.inertia_iyy,
                iyz=props.inertia_iyz,
                izz=props.inertia_izz,
            )
            active_lb.mass(
                props.mass,
                origin_xyz=tuple(props.inertia_origin_xyz),
                origin_rpy=tuple(props.inertia_origin_rpy),
                inertia=inertia,
            )

        # 4. Translate Gazebo Physics
        if props.use_simulation_props:
            active_lb.physics(
                self_collide=props.self_collide,
                gravity=props.gravity,
                mu=props.mu,
                mu2=props.mu2,
                kp=props.kp,
                kd=props.kd,
            )

        return active_lb

    def _get_geom_suffix(
        self, child: Any, parent_obj: Any, type_tag: str, sanitize_func: Any
    ) -> str:
        visual_count = sum(1 for c in parent_obj.children if type_tag in c.name)
        source_name = child.get(TAG_SOURCE_NAME, None)
        if source_name:
            return f"_{sanitize_func(source_name)}"
        elif visual_count > 1:
            idx = [c for c in parent_obj.children if type_tag in c.name].index(child)
            return f"_{idx}"
        return ""

    def _validate_mesh(
        self,
        obj: Any,
        link_name: str,
        purpose: str,
        result: ValidationResult | None,
        depsgraph: Any | None = None,
    ) -> None:
        if not result or obj.type != "MESH":
            return

        from .blender_to_core import extract_mesh_triangles

        try:
            # Use the robust triangle extraction from blender_to_core
            # This handles triangulation and applies modifiers via depsgraph
            mesh_data = extract_mesh_triangles(obj, depsgraph=depsgraph)
            if not mesh_data:
                return

            verts, tris = mesh_data

            issues = validate_mesh_topology(
                vertices=verts, triangles=tris, name=f"{link_name} ({purpose})", level=2
            )

            for issue in issues:
                # Mesh issues are advisory for physics stability, but not fatal for the model IR.
                # We report them as warnings to avoid breaking the build pipeline (especially in tests).
                result.add_warning(
                    title=issue.title,
                    message=issue.message,
                    code=issue.code,
                    affected_objects=[link_name, obj.name],
                    suggestion=issue.suggestion,
                )
        except Exception as e:
            logger.debug(f"Mesh validation failed for {obj.name}: {e}")


class JointTranslator(ITranslator):
    """Translates Blender objects marked as robot joints."""

    def translate(
        self,
        obj: Any,
        builder: RobotBuilder,  # noqa: ARG002
        context: IBlenderContext,  # noqa: ARG002
        meshes_dir: Path | None = None,  # noqa: ARG002
        dry_run: bool = False,  # noqa: ARG002
        depsgraph: Any | None = None,  # noqa: ARG002
        validation_result: ValidationResult | None = None,  # noqa: ARG002
        lb: LinkBuilder | None = None,
        link_frames: dict[str, Any] | None = None,
        **_kwargs: Any,
    ) -> None:
        """Translate a Blender joint to a Core Joint using the LinkBuilder."""
        from .blender_to_core import matrix_to_transform

        props = get_joint_props(obj)
        if not props or not props.is_robot_joint:
            return

        if not props.parent_link:
            raise RobotValidationError(ValidationErrorCode.NOT_FOUND, "Joint has no parent link")
        if not props.child_link:
            raise RobotValidationError(ValidationErrorCode.NOT_FOUND, "Joint has no child link")

        if not lb:
            return

        # Calculate joint origin
        if link_frames:
            parent_props = get_link_props(props.parent_link)
            parent_name = parent_props.link_name if parent_props else ""
            child_props = get_link_props(props.child_link)
            child_name = child_props.link_name if child_props else ""

            if parent_name in link_frames and child_name in link_frames:
                parent_frame = link_frames[parent_name]
                child_frame = link_frames[child_name]
                joint_relative = parent_frame.inverted() @ child_frame
                origin = matrix_to_transform(joint_relative)
            else:
                origin = matrix_to_transform(obj.matrix_world)
        else:
            origin = matrix_to_transform(obj.matrix_world)

        # Joint Axis
        axis: tuple[float, float, float]
        if props.axis == "X":
            axis = (1.0, 0.0, 0.0)
        elif props.axis == "Y":
            axis = (0.0, 1.0, 0.0)
        elif props.axis == "Z":
            axis = (0.0, 0.0, 1.0)
        elif props.axis == "CUSTOM":
            axis = (
                float(props.custom_axis_x),
                float(props.custom_axis_y),
                float(props.custom_axis_z),
            )
            # Fallback for zero axis
            if all(abs(v) < SYLVESTER_TOLERANCE_EPSILON for v in axis):
                axis = DEFAULT_AXIS_XYZ
        else:
            axis = DEFAULT_AXIS_XYZ

        # Select joint type and configure
        joint_type = JointType(props.joint_type.lower())
        j_name = props.joint_name if props.joint_name else obj.name

        if joint_type == JointType.REVOLUTE:
            lb.revolute(
                name=j_name,
                axis=axis,
                limits=(props.limit_lower, props.limit_upper),
                effort=props.limit_effort,
                velocity=props.limit_velocity,
                xyz=origin.xyz.to_tuple(),
                rpy=origin.rpy.to_tuple(),
            )
        elif joint_type == JointType.CONTINUOUS:
            lb.continuous(
                name=j_name,
                axis=axis,
                effort=props.limit_effort,
                velocity=props.limit_velocity,
                xyz=origin.xyz.to_tuple(),
                rpy=origin.rpy.to_tuple(),
            )
        elif joint_type == JointType.PRISMATIC:
            lb.prismatic(
                name=j_name,
                axis=axis,
                limits=(props.limit_lower, props.limit_upper),
                effort=props.limit_effort,
                velocity=props.limit_velocity,
                xyz=origin.xyz.to_tuple(),
                rpy=origin.rpy.to_tuple(),
            )
        elif joint_type == JointType.FIXED:
            lb.fixed(name=j_name, xyz=origin.xyz.to_tuple(), rpy=origin.rpy.to_tuple())
        elif joint_type == JointType.FLOATING:
            lb.floating(name=j_name, xyz=origin.xyz.to_tuple(), rpy=origin.rpy.to_tuple())
        elif joint_type == JointType.PLANAR:
            lb.planar(name=j_name, axis=axis, xyz=origin.xyz.to_tuple(), rpy=origin.rpy.to_tuple())

        # Dynamics
        if props.use_dynamics:
            lb.dynamics(damping=props.dynamics_damping, friction=props.dynamics_friction)

        # Mimic
        if props.use_mimic and props.mimic_joint:
            mimic_props = get_joint_props(props.mimic_joint)
            mimic_name = mimic_props.joint_name if mimic_props else props.mimic_joint.name
            lb.mimic(mimic_name, multiplier=props.mimic_multiplier, offset=props.mimic_offset)

        # Safety & Calibration
        if props.use_safety_controller:
            lb.safety(
                soft_lower=props.safety_soft_lower_limit,
                soft_upper=props.safety_soft_upper_limit,
                k_position=props.safety_k_position,
                k_velocity=props.safety_k_velocity,
            )

        if props.use_calibration:
            lb.calibration(
                rising=props.calibration_rising if props.use_calibration_rising else None,
                falling=props.calibration_falling if props.use_calibration_falling else None,
            )


class SensorTranslator(ITranslator):
    """Translates Blender objects marked as robot sensors."""

    def translate(
        self,
        obj: Any,
        builder: RobotBuilder,
        context: IBlenderContext,  # noqa: ARG002
        meshes_dir: Path | None = None,  # noqa: ARG002
        dry_run: bool = False,  # noqa: ARG002
        depsgraph: Any | None = None,  # noqa: ARG002
        validation_result: ValidationResult | None = None,
        link_frames: dict[str, Any] | None = None,
    ) -> None:
        """Translate a Blender sensor to a Core Sensor and add it to the robot."""
        from dataclasses import replace

        try:
            sensor = self._blender_sensor_to_core(obj)
            if sensor:
                # Calculate origin relative to link
                link_name = sensor.link_name
                if link_frames and link_name in link_frames:
                    link_frame_inv = link_frames[link_name].inverted()
                    sensor_relative = link_frame_inv @ obj.matrix_world
                    corrected_origin = self._matrix_to_transform(sensor_relative)
                    sensor = replace(sensor, origin=corrected_origin)

                builder.robot.add_sensor(sensor)
        except Exception as e:
            if validation_result:
                validation_result.add_error(
                    title=f"Sensor translation failed: {obj.name}",
                    message=str(e),
                    code=ValidationErrorCode.INVALID_VALUE,
                    affected_objects=[obj.name],
                )
            else:
                # If no validation result is provided, let the exception bubble up
                # to avoid silent failures in tests or scripts.
                raise

    def _matrix_to_transform(self, matrix: Any) -> Any:
        """Helper to convert matrix to transform without circular import."""
        from .blender_to_core import matrix_to_transform

        return matrix_to_transform(matrix)

    def _blender_sensor_to_core(self, obj: Any) -> Sensor | None:
        """Convert a Blender sensor Empty and its properties to a Core Sensor model."""
        if obj is None:
            return None
        props = get_sensor_props(obj)
        if not props or not props.is_robot_sensor:
            return None

        potential_sensor_name = getattr(props, "sensor_name", "")
        sensor_name = (
            potential_sensor_name
            if isinstance(potential_sensor_name, str) and potential_sensor_name
            else obj.name
        )
        sensor_type = SensorType(props.sensor_type.lower())
        link_obj = props.attached_link
        link_props = get_link_props(link_obj)
        link_name = (
            (link_props.link_name if link_props and link_props.link_name else link_obj.name)
            if link_obj
            else ""
        )

        if not link_name:
            raise RobotValidationError(
                ValidationErrorCode.NOT_FOUND,
                "Sensor is not attached to any link. Please select a parent link.",
                target="SensorAttachment",
                value=sensor_name,
            )

        # Build sensor origin from object transform
        origin = self._matrix_to_transform(obj.matrix_world)

        # Type-specific info
        camera_info = None
        lidar_info = None
        imu_info = None
        gps_info = None
        contact_info = None
        force_torque_info = None

        # Noise model
        noise = None
        if props.use_noise:
            noise = SensorNoise(
                type=props.noise_type,
                mean=props.noise_mean,
                stddev=props.noise_stddev,
            )

        # Camera info
        if sensor_type in (SensorType.CAMERA, SensorType.DEPTH_CAMERA):
            camera_info = CameraInfo(
                horizontal_fov=props.camera_horizontal_fov,
                width=props.camera_width,
                height=props.camera_height,
                format=props.camera_format,
                near_clip=props.camera_near_clip,
                far_clip=props.camera_far_clip,
                noise=noise,
            )

        # LIDAR info
        elif sensor_type in (SensorType.LIDAR, SensorType.GPU_LIDAR):
            lidar_info = LidarInfo(
                horizontal_samples=int(props.lidar_horizontal_samples),
                horizontal_min_angle=float(props.lidar_horizontal_min_angle),
                horizontal_max_angle=float(props.lidar_horizontal_max_angle),
                vertical_samples=int(props.lidar_vertical_samples),
                vertical_min_angle=float(props.lidar_vertical_min_angle),
                vertical_max_angle=float(props.lidar_vertical_max_angle),
                range_min=float(props.lidar_range_min),
                range_max=float(props.lidar_range_max),
                range_resolution=float(props.lidar_range_resolution),
                noise=noise,
            )

        # IMU info
        elif sensor_type == SensorType.IMU:
            imu_info = IMUInfo(
                angular_velocity_noise=noise,
                linear_acceleration_noise=noise,
            )

        # GPS info
        elif sensor_type == SensorType.GPS:
            gps_info = GPSInfo(
                position_sensing_horizontal_noise=noise,
                velocity_sensing_horizontal_noise=noise,
            )

        # Contact info
        elif sensor_type == SensorType.CONTACT:
            collision_name = props.contact_collision
            if not collision_name:
                # Fallback: try to guess standard name
                collision_name = f"{link_name}{SUFFIX_COLLISION}"
            contact_info = ContactInfo(collision=collision_name, noise=noise)

        # Force/Torque info
        elif sensor_type == SensorType.FORCE_TORQUE:
            force_torque_info = ForceTorqueInfo(noise=noise)

        # Gazebo plugin
        plugin = None
        if props.use_gazebo_plugin and props.plugin_filename:
            plugin = GazeboPlugin(
                name=f"{sensor_name}_plugin",
                filename=props.plugin_filename,
            )

        # Topic name
        topic = props.topic_name if props.topic_name else None

        return Sensor(
            name=sensor_name,
            type=sensor_type,
            link_name=link_name,
            origin=origin,
            update_rate=props.update_rate,
            always_on=props.always_on,
            visualize=props.visualize,
            camera_info=camera_info,
            lidar_info=lidar_info,
            imu_info=imu_info,
            gps_info=gps_info,
            contact_info=contact_info,
            force_torque_info=force_torque_info,
            plugin=plugin,
            topic=topic,
        )


class Ros2ControlTranslator(ITranslator):
    """Translates centralized Blender ros2_control properties."""

    def translate(
        self,
        obj: Any,
        builder: RobotBuilder,
        context: IBlenderContext,  # noqa: ARG002
        meshes_dir: Path | None = None,  # noqa: ARG002
        dry_run: bool = False,  # noqa: ARG002
        depsgraph: Any | None = None,  # noqa: ARG002
        validation_result: ValidationResult | None = None,
        **_kwargs: Any,
    ) -> None:
        """Translate centralized ros2_control properties and add to robot."""
        try:
            control = self._blender_ros2_control_to_core(obj)
            if control:
                builder.robot.add_ros2_control(control)
        except Exception as e:
            if validation_result:
                validation_result.add_error(
                    title="ROS2 Control translation failed",
                    message=str(e),
                    code=ValidationErrorCode.INVALID_VALUE,
                )

    def _blender_ros2_control_to_core(self, props: Any) -> Ros2Control | None:
        """Convert centralized Blender ros2_control properties to Core model."""
        logger = get_logger(__name__)

        if props is None or not getattr(props, "use_ros2_control", False):
            return None

        ros2_control_type = getattr(props, "ros2_control_type", CONTROL_TYPE_SYSTEM)
        joints: list[Ros2ControlJoint] = []
        for item in getattr(props, "ros2_control_joints", []):
            cmd_ifs = []
            if getattr(item, "cmd_position", False):
                cmd_ifs.append(HW_IF_POSITION)
            if getattr(item, "cmd_velocity", False):
                cmd_ifs.append(HW_IF_VELOCITY)
            if getattr(item, "cmd_effort", False):
                cmd_ifs.append(HW_IF_EFFORT)

            state_ifs = []
            if getattr(item, "state_position", False):
                state_ifs.append(HW_IF_POSITION)
            if getattr(item, "state_velocity", False):
                state_ifs.append(HW_IF_VELOCITY)
            if getattr(item, "state_effort", False):
                state_ifs.append(HW_IF_EFFORT)

            # Intelligent defaults
            if ros2_control_type == CONTROL_TYPE_SENSOR:
                if cmd_ifs:
                    logger.warning(
                        f"ROS2 Control: Hardware type 'sensor' cannot have command interfaces. "
                        f"Stripping {cmd_ifs} from joint '{getattr(item, 'name', 'unknown')}'."
                    )
                    cmd_ifs = []
                if not state_ifs:
                    state_ifs.append(HW_IF_POSITION)
            else:
                if state_ifs and not cmd_ifs:
                    cmd_ifs.append(HW_IF_POSITION)
                elif cmd_ifs and not state_ifs:
                    state_ifs.append(HW_IF_POSITION)

            # Extract joint-level parameters
            parameters = {p.name: p.value for p in getattr(item, "parameters", []) if p.name}

            # Determine the correct joint name
            joint_obj = getattr(item, "joint_obj", None)
            joint_props = get_joint_props(joint_obj)
            joint_name = ""
            if joint_props:
                potential_name = joint_props.joint_name
                if isinstance(potential_name, str):
                    joint_name = potential_name

            if not joint_name:
                item_name = getattr(item, "name", "joint")
                joint_name = item_name if isinstance(item_name, str) else "joint"

            if cmd_ifs or state_ifs:
                joints.append(
                    Ros2ControlJoint(
                        name=joint_name,
                        command_interfaces=cmd_ifs,
                        state_interfaces=state_ifs,
                        parameters=parameters,
                    )
                )

        # ROS 2 Specification: 'actuator' types must have exactly one joint.
        if ros2_control_type == CONTROL_TYPE_ACTUATOR and len(joints) > 1:
            logger.warning(
                f"ROS2 Control: Hardware type 'actuator' is limited to exactly one joint by ROS 2 "
                f"specification. Truncating {len(joints)} joints to only include '{joints[0].name}'."
            )
            joints = joints[:1]

        if not joints:
            return None

        core_type = ros2_control_type

        return Ros2Control(
            name=props.ros2_control_name if props.ros2_control_name else "RobotControl",
            type=core_type,
            hardware_plugin=props.hardware_plugin,
            joints=joints,
        )


class TransmissionTranslator(ITranslator):
    """Translates Blender objects marked as robot transmissions."""

    def translate(
        self,
        obj: Any,
        builder: RobotBuilder,
        context: IBlenderContext,  # noqa: ARG002
        meshes_dir: Path | None = None,  # noqa: ARG002
        dry_run: bool = False,  # noqa: ARG002
        depsgraph: Any | None = None,  # noqa: ARG002
        validation_result: ValidationResult | None = None,
    ) -> None:
        """Translate a Blender transmission to a Core Transmission and add it to the robot."""
        try:
            transmission = self._blender_transmission_to_core(obj)
            if transmission:
                builder.robot.add_transmission(transmission)
        except Exception as e:
            if validation_result:
                validation_result.add_error(
                    title=f"Transmission translation failed: {obj.name}",
                    message=str(e),
                    code=ValidationErrorCode.INVALID_VALUE,
                    affected_objects=[obj.name],
                )

    def _blender_transmission_to_core(self, obj: Any) -> Transmission | None:
        """Convert Blender Empty with TransmissionPropertyGroup to Core Transmission."""
        if obj is None:
            return None

        props = get_transmission_props(obj)
        if not props or not props.is_robot_transmission:
            return None

        trans_name = props.transmission_name if props.transmission_name else obj.name

        # Transmission type normalization (handle both 'simple' and 'SIMPLE')
        raw_type = str(props.transmission_type).lower()

        # Transmission type mapping to URDF plugin names
        trans_type_map = {
            TRANS_SIMPLE: TransmissionType.SIMPLE.value,
            TRANS_DIFFERENTIAL: TransmissionType.DIFFERENTIAL.value,
            TRANS_FOUR_BAR: TransmissionType.FOUR_BAR_LINKAGE.value,
            TRANS_CUSTOM: props.custom_type if props.custom_type else TransmissionType.CUSTOM.value,
        }
        trans_type = trans_type_map.get(raw_type, TransmissionType.SIMPLE.value)

        # Hardware interface mapping
        hw_if = props.hardware_interface

        joints = []
        actuators = []

        if raw_type in (TRANS_SIMPLE, TRANS_CUSTOM, TRANS_FOUR_BAR):
            joint_obj = props.joint_name
            if joint_obj:
                joint_props = get_joint_props(joint_obj)
                joint_name = ""
                if joint_props:
                    potential_name = joint_props.joint_name
                    if isinstance(potential_name, str):
                        joint_name = potential_name

                if not joint_name:
                    joint_name = joint_obj.name

                joints.append(
                    TransmissionJoint(
                        name=joint_name,
                        hardware_interfaces=[hw_if],
                        mechanical_reduction=props.mechanical_reduction,
                        offset=props.offset,
                    )
                )

                act_name = (
                    props.actuator_name
                    if props.use_custom_actuator_name and props.actuator_name
                    else f"{joint_name}_motor"
                )
                actuators.append(TransmissionActuator(name=act_name, hardware_interfaces=[hw_if]))
        elif raw_type == TRANS_DIFFERENTIAL:
            j1_obj = props.joint1_name
            j2_obj = props.joint2_name
            if j1_obj and j2_obj:
                j1_props = get_joint_props(j1_obj)
                j1_name = (
                    j1_props.joint_name if j1_props and j1_props.joint_name else ""
                ) or j1_obj.name
                j2_props = get_joint_props(j2_obj)
                j2_name = (
                    j2_props.joint_name if j2_props and j2_props.joint_name else ""
                ) or j2_obj.name

                joints.append(
                    TransmissionJoint(
                        name=j1_name,
                        hardware_interfaces=[hw_if],
                        mechanical_reduction=props.mechanical_reduction,
                    )
                )
                joints.append(
                    TransmissionJoint(
                        name=j2_name,
                        hardware_interfaces=[hw_if],
                        mechanical_reduction=props.mechanical_reduction,
                    )
                )

                a1_name = props.actuator1_name if props.actuator1_name else f"{j1_name}_motor"
                a2_name = props.actuator2_name if props.actuator2_name else f"{j2_name}_motor"

                actuators.append(TransmissionActuator(name=a1_name, hardware_interfaces=[hw_if]))
                actuators.append(TransmissionActuator(name=a2_name, hardware_interfaces=[hw_if]))

        if not joints:
            return None

        return Transmission(name=trans_name, type=trans_type, joints=joints, actuators=actuators)
