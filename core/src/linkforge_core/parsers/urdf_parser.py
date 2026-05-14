"""URDF XML parser to import robot models.

This module implements a robust URDF/XACRO parser that transforms XML robot
descriptions into the internal LinkForge `Robot` model. It is designed for
high-fidelity round-tripping and includes:

- **Comprehensive Parsing**: Full support for all URDF joint types, multiple
  visual/collision elements, and complex nested structures.
- **Security & Validation**: Built-in protection against XML attacks (depth
  limits), strict validation of mesh paths, numeric values, and rejection of
  unresolved XACRO files to maintain model integrity.
- **Error Resilience**: Informative error messages and fallback mechanisms
  for malformed or incomplete XML configurations.
- **Fidelity Preservation**: Captures metadata, sensor noise, and ROS 2
  control parameters to ensure nothing is lost during the Blender import process.
"""

from __future__ import annotations

import io
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path
from typing import Any, cast

from ..base import IResourceResolver
from ..constants import XACRO_URIS
from ..exceptions import (
    RobotModelError,
    RobotParserIOError,
    RobotParserUnexpectedError,
    RobotParserXMLRootError,
    RobotValidationError,
    ValidationErrorCode,
    XacroDetectedError,
)
from ..logging_config import get_logger
from ..models import (
    CameraInfo,
    Collision,
    ContactInfo,
    ForceTorqueInfo,
    GazeboElement,
    GazeboPlugin,
    GPSInfo,
    IMUInfo,
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafetyController,
    JointType,
    LidarInfo,
    Link,
    Material,
    Robot,
    Ros2Control,
    Ros2ControlJoint,
    Ros2ControlSensor,
    Sensor,
    SensorNoise,
    SensorType,
    Transform,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    Vector3,
    Visual,
)
from ..utils.math_utils import normalize_vector
from ..utils.xml_utils import (
    MAX_XML_DEPTH,
    get_xml_namespace,
    parse_float,
    parse_int,
    parse_optional_bool,
    parse_optional_float,
    parse_vector3,
    strip_xml_namespace,
)
from .xml_base import MAX_FILE_SIZE, RobotXMLParser

logger = get_logger(__name__)


class URDFParser(RobotXMLParser[Robot]):
    """Refined URDF Parser using a class-based interface."""

    def __init__(
        self,
        max_file_size: int = MAX_FILE_SIZE,
        sandbox_root: Path | None = None,
        resource_resolver: IResourceResolver | None = None,
    ) -> None:
        """Initialize parser.

        Args:
            max_file_size: Maximum allowed file size in bytes (default: 100MB)
            sandbox_root: Optional root directory for security sandbox
            resource_resolver: Optional resolver for URIs

        """
        super().__init__(
            max_file_size=max_file_size,
            sandbox_root=sandbox_root,
            resource_resolver=resource_resolver,
        )

    def _parse_link(
        self,
        link_elem: ET.Element,
        materials: dict[str, Material],
        source_directory: Path | None = None,
    ) -> Link:
        """Parse a <link> element into a Link model.

        Args:
            link_elem: The XML element for the link.
            materials: Dictionary of global materials for resolution.
            source_directory: Base directory for relative mesh paths.

        Returns:
            A populated Link model.

        """
        name = link_elem.get("name", "unnamed_link")

        # Parse Visuals
        visuals = [
            v
            for visual_elem in link_elem.findall("{*}visual")
            if (v := self._parse_visual_element(visual_elem, materials, source_directory))
        ]

        # Parse Collisions
        collisions = [
            c
            for collision_elem in link_elem.findall("{*}collision")
            if (c := self._parse_collision_element(collision_elem, source_directory))
        ]

        inertial = self._parse_inertial_element(link_elem.find("{*}inertial"))

        return Link(
            name=name,
            visuals=visuals,
            collisions=collisions,
            inertial=inertial,
        )

    def _parse_visual_element(
        self,
        visual_elem: ET.Element,
        materials: dict[str, Material],
        source_directory: Path | None = None,
    ) -> Visual | None:
        """Parse a single <visual> element.

        Args:
            visual_elem: The visual XML element.
            materials: Global materials dictionary.
            source_directory: Path for mesh resolution.

        Returns:
            A Visual model or None if geometry is missing.

        """
        origin = self._parse_origin_element(visual_elem.find("{*}origin"))
        geom_elem = visual_elem.find("{*}geometry")
        if geom_elem is None:
            return None

        geometry = self._parse_geometry_element(geom_elem, source_directory)
        if not geometry:
            return None

        material = self._parse_material_element(visual_elem.find("{*}material"), materials)
        return Visual(
            geometry=geometry,
            origin=origin,
            material=material,
            name=visual_elem.get("name"),
        )

    def _parse_collision_element(
        self, collision_elem: ET.Element, source_directory: Path | None = None
    ) -> Collision | None:
        """Parse a single <collision> element.

        Args:
            collision_elem: The collision XML element.
            source_directory: Path for mesh resolution.

        Returns:
            A Collision model or None if geometry is missing.

        """
        origin = self._parse_origin_element(collision_elem.find("{*}origin"))
        geom_elem = collision_elem.find("{*}geometry")
        if geom_elem is None:
            return None

        geometry = self._parse_geometry_element(geom_elem, source_directory)
        if not geometry:
            return None

        return Collision(
            geometry=geometry,
            origin=origin,
            name=collision_elem.get("name"),
        )

    def _parse_joint(self, joint_elem: ET.Element) -> Joint:
        """Parse a <joint> element into a Joint model.

        Args:
            joint_elem: The XML element for the joint.

        Returns:
            A populated Joint model.

        """
        name = joint_elem.get("name", "unnamed_joint")
        joint_type_str = joint_elem.get("type", "fixed")

        type_map = {
            "revolute": JointType.REVOLUTE,
            "continuous": JointType.CONTINUOUS,
            "prismatic": JointType.PRISMATIC,
            "fixed": JointType.FIXED,
            "floating": JointType.FLOATING,
            "planar": JointType.PLANAR,
        }
        joint_type = type_map.get(joint_type_str.lower(), JointType.FIXED)

        parent_elem = joint_elem.find("{*}parent")
        child_elem = joint_elem.find("{*}child")
        parent = parent_elem.get("link", "") if parent_elem is not None else ""
        child = child_elem.get("link", "") if child_elem is not None else ""

        origin = self._parse_origin_element(joint_elem.find("{*}origin"))

        # Parse Axis
        axis = self._parse_joint_axis(joint_elem, joint_type)

        # Parse Sub-components
        limits = self._parse_joint_limits(joint_elem, joint_type, name)
        dynamics = self._parse_joint_dynamics(joint_elem.find("{*}dynamics"))
        mimic = self._parse_joint_mimic(joint_elem.find("{*}mimic"))
        safety = self._parse_joint_safety(joint_elem.find("{*}safety_controller"))
        calibration = self._parse_joint_calibration(joint_elem.find("{*}calibration"))

        return Joint(
            name=name,
            type=joint_type,
            parent=parent,
            child=child,
            origin=origin,
            axis=axis,
            limits=limits,
            dynamics=dynamics,
            mimic=mimic,
            safety_controller=safety,
            calibration=calibration,
        )

    def _parse_joint_axis(self, joint_elem: ET.Element, joint_type: JointType) -> Vector3 | None:
        """Parse the <axis> element for a joint.

        Args:
            joint_elem: The joint XML element.
            joint_type: The type of the joint.

        Returns:
            A normalized Vector3 for the axis, or None if not applicable.

        """
        if joint_type not in (
            JointType.REVOLUTE,
            JointType.CONTINUOUS,
            JointType.PRISMATIC,
            JointType.PLANAR,
        ):
            return None

        axis_elem = joint_elem.find("{*}axis")
        if axis_elem is not None:
            axis = parse_vector3(axis_elem.get("xyz", "1 0 0"))
            nx, ny, nz = normalize_vector(axis.x, axis.y, axis.z)
            if nx == 0.0 and ny == 0.0 and nz == 0.0:
                return Vector3(1.0, 0.0, 0.0)
            return Vector3(nx, ny, nz)

        return Vector3(1.0, 0.0, 0.0)

    def _parse_joint_limits(
        self, joint_elem: ET.Element, joint_type: JointType, joint_name: str
    ) -> JointLimits | None:
        """Parse the <limit> element for a joint.

        Args:
            joint_elem: The joint XML element.
            joint_type: The type of the joint.
            joint_name: The name of the joint for logging.

        Returns:
            A JointLimits model or None.

        """
        if joint_type not in (JointType.REVOLUTE, JointType.PRISMATIC, JointType.CONTINUOUS):
            return None

        limits_elem = joint_elem.find("{*}limit")
        if limits_elem is not None:
            lower_str = limits_elem.get("lower")
            upper_str = limits_elem.get("upper")
            effort = parse_float(limits_elem.get("effort"), check_name="effort", default=0.0)
            velocity = parse_float(limits_elem.get("velocity"), check_name="velocity", default=0.0)

            # Validation: effort and velocity must be non-negative
            if effort < 0:
                logger.warning(f"Joint '{joint_name}' effort limit is negative ({effort})")
                effort = 0.0
            if velocity < 0:
                logger.warning(f"Joint '{joint_name}' velocity limit is negative ({velocity})")
                velocity = 0.0

            return JointLimits(
                lower=float(lower_str) if lower_str is not None else None,
                upper=float(upper_str) if upper_str is not None else None,
                effort=effort,
                velocity=velocity,
            )

        if joint_type in (JointType.REVOLUTE, JointType.PRISMATIC):
            return JointLimits(lower=0.0, upper=0.0, effort=0.0, velocity=0.0)

        return None

    def _parse_joint_dynamics(self, elem: ET.Element | None) -> JointDynamics | None:
        """Parse the <dynamics> element into a JointDynamics model.

        Args:
            elem: The dynamics XML element.

        Returns:
            A populated JointDynamics instance or None.

        """
        if elem is None:
            return None
        return JointDynamics(
            damping=parse_float(elem.get("damping"), check_name="damping", default=0.0),
            friction=parse_float(elem.get("friction"), check_name="friction", default=0.0),
        )

    def _parse_joint_mimic(self, elem: ET.Element | None) -> JointMimic | None:
        """Parse the <mimic> element into a JointMimic model.

        Args:
            elem: The mimic XML element.

        Returns:
            A populated JointMimic instance or None.

        """
        if elem is None:
            return None
        return JointMimic(
            joint=elem.get("joint", ""),
            multiplier=parse_float(elem.get("multiplier"), check_name="multiplier", default=1.0),
            offset=parse_float(elem.get("offset"), check_name="offset", default=0.0),
        )

    def _parse_joint_safety(self, elem: ET.Element | None) -> JointSafetyController | None:
        """Parse the <safety_controller> element into a JointSafetyController model.

        Args:
            elem: The safety_controller XML element.

        Returns:
            A populated JointSafetyController instance or None.

        """
        if elem is None:
            return None
        return JointSafetyController(
            soft_lower_limit=parse_float(
                elem.get("soft_lower_limit"), check_name="soft_lower_limit"
            ),
            soft_upper_limit=parse_float(
                elem.get("soft_upper_limit"), check_name="soft_upper_limit"
            ),
            k_position=parse_float(elem.get("k_position"), check_name="k_position"),
            k_velocity=parse_float(elem.get("k_velocity"), check_name="k_velocity", default=0.0),
        )

    def _parse_joint_calibration(self, elem: ET.Element | None) -> JointCalibration | None:
        """Parse the <calibration> element into a JointCalibration model.

        Args:
            elem: The calibration XML element.

        Returns:
            A populated JointCalibration instance or None.

        """
        if elem is None:
            return None
        rising_str = elem.get("rising")
        falling_str = elem.get("falling")
        return JointCalibration(
            rising=parse_float(rising_str, check_name="rising") if rising_str is not None else None,
            falling=parse_float(falling_str, check_name="falling")
            if falling_str is not None
            else None,
        )

    def _normalize_hardware_interface(self, interface: str) -> str:
        """Normalize hardware interface string (e.g., 'PositionJointInterface' -> 'position').

        Args:
            interface: The raw hardware interface string from the URDF.

        Returns:
            The normalized ROS 2 compatible hardware interface string.

        """
        clean = interface.lower().replace("hardware_interface/", "").replace("jointinterface", "")
        return clean.replace("interface", "")

    def _parse_ros2_control(self, rc_elem: ET.Element) -> Ros2Control | None:
        """Parse a <ros2_control> element.

        Args:
            rc_elem: The XML element for ros2_control.

        Returns:
            A Ros2Control model or None if parsing fails.

        """
        name = rc_elem.get("name", "")
        rc_type = rc_elem.get("type", "system")

        hw_elem = rc_elem.find("{*}hardware")
        plugin_elem = hw_elem.find("{*}plugin") if hw_elem is not None else None
        hardware_plugin = (
            plugin_elem.text.strip() if plugin_elem is not None and plugin_elem.text else ""
        )

        parameters: dict[str, str] = {}
        if hw_elem is not None:
            for param_elem in hw_elem.findall("{*}param"):
                p_name = param_elem.get("name")
                if p_name and param_elem.text:
                    parameters[p_name] = param_elem.text.strip()

        joints: list[Ros2ControlJoint] = []
        for joint_elem in rc_elem.findall("{*}joint"):
            joint_name = joint_elem.get("name", "")
            command_interfaces = [
                self._normalize_hardware_interface(cmd.get("name", "position"))
                for cmd in joint_elem.findall("{*}command_interface")
            ]
            state_interfaces = [
                self._normalize_hardware_interface(state.get("name", "position"))
                for state in joint_elem.findall("{*}state_interface")
            ]
            joint_params: dict[str, str] = {
                str(param.get("name")): param.text.strip()
                for param in joint_elem.findall("{*}param")
                if param.get("name") and param.text
            }

            if command_interfaces or state_interfaces:
                joints.append(
                    Ros2ControlJoint(
                        name=joint_name,
                        command_interfaces=command_interfaces,
                        state_interfaces=state_interfaces,
                        parameters=joint_params,
                    )
                )

        sensors: list[Ros2ControlSensor] = []
        for sensor_elem in rc_elem.findall("{*}sensor"):
            sensor_name = sensor_elem.get("name", "")
            state_interfaces = [
                self._normalize_hardware_interface(state.get("name", "position"))
                for state in sensor_elem.findall("{*}state_interface")
            ]
            sensor_params: dict[str, str] = {
                str(param.get("name")): param.text.strip()
                for param in sensor_elem.findall("{*}param")
                if param.get("name") and param.text
            }

            if state_interfaces or sensor_params:
                sensors.append(
                    Ros2ControlSensor(
                        name=sensor_name,
                        state_interfaces=state_interfaces,
                        parameters=sensor_params,
                    )
                )

        # Catch-all for top-level parameters not inside <hardware>
        for child in rc_elem:
            if strip_xml_namespace(child.tag) not in ("hardware", "joint", "sensor") and child.text:
                parameters[strip_xml_namespace(child.tag)] = child.text.strip()

        try:
            return Ros2Control(
                name=name,
                type=rc_type,
                hardware_plugin=hardware_plugin,
                joints=joints,
                sensors=sensors,
                parameters=parameters,
            )
        except RobotModelError as e:
            logger.warning(f"Invalid ros2_control '{name}' ignored during strict parsing: {e}")
            return None

    def _parse_transmission_component(
        self, elem: ET.Element, tag: str
    ) -> TransmissionJoint | TransmissionActuator | None:
        """Parse a joint or actuator component within a transmission.

        Args:
            elem: The XML element (joint or actuator).
            tag: The component type ('joint' or 'actuator').

        Returns:
            The parsed component model or None.

        """
        name = elem.get("name", "")
        if not name:
            return None

        # Handle both camelCase and snake_case for hardware interfaces
        hw_interfaces = [
            self._normalize_hardware_interface(hw.text.strip())
            for hw in (elem.findall("{*}hardwareInterface") + elem.findall("{*}hardware_interface"))
            if hw.text
        ]

        reduction = parse_float(
            elem.findtext("{*}mechanicalReduction") or elem.findtext("{*}mechanical_reduction"),
            check_name="mechanicalReduction",
            default=1.0,
        )
        if reduction == 0:
            logger.warning(f"Transmission component '{name}' has zero reduction, defaulting to 1.0")
            reduction = 1.0

        offset = parse_float(elem.findtext("{*}offset"), check_name="offset", default=0.0)

        if tag == "joint":
            return TransmissionJoint(
                name=name,
                hardware_interfaces=hw_interfaces or ["position"],
                mechanical_reduction=reduction,
                offset=offset,
            )
        return TransmissionActuator(
            name=name,
            hardware_interfaces=hw_interfaces or ["position"],
            mechanical_reduction=reduction,
            offset=offset,
        )

    def _parse_transmission(self, trans_elem: ET.Element) -> Transmission | None:
        """Parse a <transmission> element.

        Args:
            trans_elem: The transmission XML element.

        Returns:
            A Transmission model or None if parsing fails.

        """
        name = trans_elem.get("name", "unnamed_transmission")
        trans_type = trans_elem.findtext("{*}type", "")

        joints: list[TransmissionJoint] = []
        for j_elem in trans_elem.findall("{*}joint"):
            comp = self._parse_transmission_component(j_elem, "joint")
            if isinstance(comp, TransmissionJoint):
                joints.append(comp)

        actuators: list[TransmissionActuator] = []
        for a_elem in trans_elem.findall("{*}actuator"):
            comp = self._parse_transmission_component(a_elem, "actuator")
            if isinstance(comp, TransmissionActuator):
                actuators.append(comp)

        try:
            return Transmission(name=name, type=trans_type, joints=joints, actuators=actuators)
        except RobotModelError as e:
            logger.warning(f"Invalid transmission '{name}' ignored: {e}")
            return None

    def _parse_sensor_noise(self, noise_elem: ET.Element | None) -> SensorNoise | None:
        """Parse a <noise> element.

        Args:
            noise_elem: The XML element containing noise parameters.

        Returns:
            A SensorNoise model or None.

        """
        if noise_elem is None:
            return None
        actual_noise_elem = noise_elem.find("{*}noise") if noise_elem.tag != "noise" else noise_elem
        if actual_noise_elem is None:
            return None
        return SensorNoise(
            type=actual_noise_elem.findtext("{*}type", "gaussian"),
            mean=parse_float(
                actual_noise_elem.findtext("{*}mean", "0.0"), check_name="mean", default=0.0
            ),
            stddev=parse_float(
                actual_noise_elem.findtext("{*}stddev", "0.0"), check_name="stddev", default=0.0
            ),
        )

    def _parse_sensor_from_gazebo(self, gazebo_elem: ET.Element) -> Sensor | None:
        """Parse a <sensor> from within a <gazebo> extension element.

        Args:
            gazebo_elem: The parent Gazebo extension element.

        Returns:
            A populated Sensor model or None if no sensor is defined.

        """
        link_name = gazebo_elem.get("reference", "")
        sensor_elem = gazebo_elem.find("{*}sensor")
        if not link_name or sensor_elem is None:
            return None

        sensor_name = sensor_elem.get("name", "")
        sensor_type_str = sensor_elem.get("type", "camera")

        type_map = {
            "camera": SensorType.CAMERA,
            "depth": SensorType.DEPTH_CAMERA,
            "depth_camera": SensorType.DEPTH_CAMERA,
            "multicamera": SensorType.CAMERA,
            "ray": SensorType.LIDAR,
            "lidar": SensorType.LIDAR,
            "gpu_ray": SensorType.LIDAR,
            "gpu_lidar": SensorType.LIDAR,
            "imu": SensorType.IMU,
            "gps": SensorType.GPS,
            "navsat": SensorType.GPS,
            "contact": SensorType.CONTACT,
            "force_torque": SensorType.FORCE_TORQUE,
        }
        sensor_type = type_map.get(sensor_type_str.lower(), SensorType.CAMERA)
        update_rate = parse_float(
            sensor_elem.findtext("{*}update_rate", "30.0"), check_name="updateRate", default=30.0
        )

        origin = Transform.identity()
        pose_elem = sensor_elem.find("{*}pose")
        if pose_elem is not None and pose_elem.text:
            parts = pose_elem.text.strip().split()
            if len(parts) >= 6:
                try:
                    xyz = parse_vector3(" ".join(parts[0:3]))
                    rpy = parse_vector3(" ".join(parts[3:6]))
                    origin = Transform(xyz=xyz, rpy=rpy)
                except RobotModelError as e:
                    logger.warning(f"Invalid sensor pose in Gazebo element: {e}")

        # Initialize info objects based on sensor type
        camera_info = None
        lidar_info = None
        imu_info = None
        gps_info = None
        contact_info = None
        force_torque_info = None

        if sensor_type in (SensorType.CAMERA, SensorType.DEPTH_CAMERA):
            camera_elem = sensor_elem.find("{*}camera")
            if camera_elem is not None:
                camera_info = CameraInfo(
                    horizontal_fov=parse_float(
                        camera_elem.findtext("{*}horizontal_fov"),
                        check_name="horizontal_fov",
                        default=1.047,
                    ),
                    width=parse_int(camera_elem.findtext("{*}image/{*}width"), default=640),
                    height=parse_int(camera_elem.findtext("{*}image/{*}height"), default=480),
                    format=camera_elem.findtext("{*}image/{*}format", "R8G8B8"),
                    near_clip=parse_float(
                        camera_elem.findtext("{*}clip/{*}near"), check_name="near", default=0.1
                    ),
                    far_clip=parse_float(
                        camera_elem.findtext("{*}clip/{*}far"), check_name="far", default=100.0
                    ),
                    noise=self._parse_sensor_noise(camera_elem),
                )
            else:
                camera_info = CameraInfo()

        elif sensor_type == SensorType.LIDAR:
            ray_elem = sensor_elem.find("{*}ray")
            if ray_elem is not None:
                lidar_info = LidarInfo(
                    horizontal_samples=parse_int(
                        ray_elem.findtext("{*}scan/{*}horizontal/{*}samples"), default=640
                    ),
                    horizontal_min_angle=parse_float(
                        ray_elem.findtext("{*}scan/{*}horizontal/{*}min_angle"),
                        check_name="min_angle",
                        default=-1.570796,
                    ),
                    horizontal_max_angle=parse_float(
                        ray_elem.findtext("{*}scan/{*}horizontal/{*}max_angle"),
                        check_name="max_angle",
                        default=1.570796,
                    ),
                    range_min=parse_float(
                        ray_elem.findtext("{*}range/{*}min"), check_name="range_min", default=0.1
                    ),
                    range_max=parse_float(
                        ray_elem.findtext("{*}range/{*}max"), check_name="range_max", default=10.0
                    ),
                    noise=self._parse_sensor_noise(ray_elem),
                )
            else:
                lidar_info = LidarInfo()

        elif sensor_type == SensorType.GPS:
            gps_elem = sensor_elem.find("{*}gps")
            if gps_elem is not None:
                gps_info = GPSInfo(
                    position_sensing_horizontal_noise=self._parse_sensor_noise(
                        gps_elem.find("{*}position_sensing/{*}horizontal")
                    ),
                    position_sensing_vertical_noise=self._parse_sensor_noise(
                        gps_elem.find("{*}position_sensing/{*}vertical")
                    ),
                    velocity_sensing_horizontal_noise=self._parse_sensor_noise(
                        gps_elem.find("{*}velocity_sensing/{*}horizontal")
                    ),
                    velocity_sensing_vertical_noise=self._parse_sensor_noise(
                        gps_elem.find("{*}velocity_sensing/{*}vertical")
                    ),
                )
            else:
                gps_info = GPSInfo()

        elif sensor_type == SensorType.IMU:
            imu_elem = sensor_elem.find("{*}imu")
            if imu_elem is not None:
                imu_info = IMUInfo(
                    angular_velocity_noise=self._parse_sensor_noise(
                        imu_elem.find("{*}angular_velocity/{*}x")
                    ),
                    linear_acceleration_noise=self._parse_sensor_noise(
                        imu_elem.find("{*}linear_acceleration/{*}x")
                    ),
                )
            else:
                imu_info = IMUInfo()

        elif sensor_type == SensorType.CONTACT:
            contact_elem = sensor_elem.find("{*}contact")
            if contact_elem is not None:
                collision = contact_elem.findtext("{*}collision")
                if not collision:
                    raise RobotValidationError(
                        ValidationErrorCode.VALUE_EMPTY,
                        f"Sensor '{sensor_name}' is missing collision reference",
                        target="SensorCollision",
                        value=sensor_name,
                    )
                contact_info = ContactInfo(
                    collision=collision, noise=self._parse_sensor_noise(contact_elem)
                )
            else:
                raise RobotValidationError(
                    ValidationErrorCode.VALUE_EMPTY,
                    f"Sensor '{sensor_name}' expects contact info but none found",
                    target="SensorCollision",
                    value=sensor_name,
                )

        elif sensor_type == SensorType.FORCE_TORQUE:
            ft_elem = sensor_elem.find("{*}force_torque")
            force_torque_info = ForceTorqueInfo(
                frame=ft_elem.findtext("{*}frame", "child") if ft_elem is not None else "child",
                measure_direction=ft_elem.findtext("{*}measure_direction", "child_to_parent")
                if ft_elem is not None
                else "child_to_parent",
                noise=self._parse_sensor_noise(ft_elem),
            )

        # Safety Fallbacks: Ensure info objects are initialized for known types
        if sensor_type in (SensorType.CAMERA, SensorType.DEPTH_CAMERA) and camera_info is None:
            camera_info = CameraInfo()
        elif sensor_type == SensorType.LIDAR and lidar_info is None:
            lidar_info = LidarInfo()
        elif sensor_type == SensorType.IMU and imu_info is None:
            imu_info = IMUInfo()
        elif sensor_type == SensorType.GPS and gps_info is None:
            gps_info = GPSInfo()

        topic = sensor_elem.findtext("{*}topic") or f"/{sensor_name}"
        return Sensor(
            name=sensor_name,
            type=sensor_type,
            link_name=link_name,
            origin=origin,
            update_rate=update_rate,
            camera_info=camera_info,
            lidar_info=lidar_info,
            imu_info=imu_info,
            gps_info=gps_info,
            contact_info=contact_info,
            force_torque_info=force_torque_info,
            plugin=self._parse_gazebo_plugin(sensor_elem.find("{*}plugin")),
            topic=topic,
        )

    def _parse_gazebo_plugin(self, plugin_elem: ET.Element | None) -> GazeboPlugin | None:
        """Parse a Gazebo plugin element.

        Args:
            plugin_elem: The plugin XML element.

        Returns:
            A populated GazeboPlugin model or None.

        """
        if plugin_elem is None:
            return None
        name = plugin_elem.get("name", "")
        filename = plugin_elem.get("filename", "")
        parameters = {
            child.tag: child.text.strip() for child in plugin_elem if child.text and not len(child)
        }
        raw_xml = (
            "".join(ET.tostring(child, encoding="unicode") for child in plugin_elem)
            if len(plugin_elem)
            else None
        )
        return GazeboPlugin(name=name, filename=filename, parameters=parameters, raw_xml=raw_xml)

    def _parse_gazebo_element(self, gazebo_elem: ET.Element) -> GazeboElement:
        """Parse a <gazebo> extension element.

        Args:
            gazebo_elem: The XML element for the Gazebo extension.

        Returns:
            A populated GazeboElement model.

        """
        reference = gazebo_elem.get("reference")
        plugins = [
            self._parse_gazebo_plugin(p) for p in gazebo_elem.findall("{*}plugin") if p is not None
        ]

        # Filter out None values
        valid_plugins = [p for p in plugins if p is not None]

        properties = {}
        excluded_tags = [
            "plugin",
            "sensor",
            "material",
            "selfCollide",
            "static",
            "gravity",
            "provideFeedback",
            "implicitSpringDamper",
            "mu1",
            "mu2",
            "kp",
            "kd",
            "stopCfm",
            "stopErp",
        ]
        for child in gazebo_elem:
            if child.tag not in excluded_tags and child.text:
                properties[child.tag] = child.text.strip()

        return GazeboElement(
            reference=reference,
            properties=properties,
            plugins=valid_plugins,
            material=gazebo_elem.findtext("material") or gazebo_elem.findtext("{*}material"),
            static=parse_optional_bool(gazebo_elem, "static"),
            stop_cfm=parse_optional_float(gazebo_elem, "stopCfm"),
            stop_erp=parse_optional_float(gazebo_elem, "stopErp"),
            provide_feedback=parse_optional_bool(gazebo_elem, "provideFeedback"),
            implicit_spring_damper=parse_optional_bool(gazebo_elem, "implicitSpringDamper"),
        )

    def _detect_xacro_file(self, root: ET.Element, filepath: Path | None = None) -> None:
        """Detect if file is XACRO and raise helpful error."""
        is_xacro = False
        if filepath:
            is_xacro = filepath.suffix.lower() in [".xacro", ".urdf.xacro"]
            if not is_xacro:
                try:
                    content = filepath.read_text(encoding="utf-8")
                    is_xacro = "xmlns:xacro" in content
                except (OSError, UnicodeDecodeError):
                    pass

        if not is_xacro:
            for child in root:
                if get_xml_namespace(child.tag) in XACRO_URIS:
                    is_xacro = True
                    break

        if not is_xacro:
            for elem in root.iter():
                if any("${" in v or "$(" in v for v in elem.attrib.values() if isinstance(v, str)):
                    is_xacro = True
                    break

        if is_xacro:
            filename = filepath.name if filepath else "URDF String"
            raise XacroDetectedError(filename)

    def _parse_from_context(
        self, context: Any, root: ET.Element, filepath: Path | None = None
    ) -> Robot:
        """Process iterative XML parsing internally.

        This method orchestrates the element-by-element construction of the
        Robot model. It handles depth tracking for security and manages the
        processing order of dependent elements (links must exist before joints).

        Args:
            context: The iterparse context.
            root: The root <robot> element.
            filepath: Path to the source file for resolution.

        Returns:
            A fully populated Robot model.

        Raises:
            RobotParserUnexpectedError: If XML nesting exceeds MAX_XML_DEPTH.
            XacroDetectedError: If the parser detects unexpanded XACRO macros.

        """
        self._detect_xacro_file(root, filepath)

        default_name = filepath.stem if filepath else "unnamed_robot"
        kwargs: dict[str, Any] = {"name": root.get("name", default_name)}
        if self.resource_resolver is not None:
            kwargs["resource_resolver"] = self.resource_resolver

        robot = Robot(**kwargs)
        materials: dict[str, Material] = {}
        depth = 0

        # Elements that depend on links (joints, transmissions, etc.) are processed
        # in a second pass to ensure the robot's kinematic graph is complete.
        delayed_joints: list[Joint] = []
        delayed_transmissions: list[Transmission] = []
        delayed_ros2_controls: list[Ros2Control] = []
        delayed_sensors: list[Sensor] = []
        delayed_gazebo_elements: list[tuple[GazeboElement, dict[str, Any]]] = []

        # Determine base directory for resolving relative mesh paths.
        source_directory = (
            filepath.parent if filepath and filepath.is_file() else filepath
        ) or Path(".")

        for event, elem in context:
            if event == "start":
                depth += 1
                if depth > MAX_XML_DEPTH:
                    raise RobotParserUnexpectedError(
                        source_area="XML nesting", original_error=depth
                    )
            elif event == "end":
                if depth == 1:
                    tag = strip_xml_namespace(elem.tag)

                    if tag == "material":
                        mat = self._parse_material_element(elem, materials)
                        if mat:
                            materials[mat.name] = mat
                            robot.materials[mat.name] = mat

                    elif tag == "link":
                        try:
                            link = self._parse_link(elem, materials, source_directory)
                            robot.add_link(link)
                        except (RobotModelError, ValueError, Exception) as e:
                            logger.warning(f"Skipping invalid link '{elem.get('name')}': {e}")

                    elif tag == "joint":
                        try:
                            delayed_joints.append(self._parse_joint(elem))
                        except (RobotModelError, ValueError, Exception) as e:
                            logger.warning(f"Skipping invalid joint '{elem.get('name')}': {e}")

                    elif tag == "transmission":
                        try:
                            trans = self._parse_transmission(elem)
                            if trans:
                                delayed_transmissions.append(trans)
                        except (RobotModelError, ValueError, Exception) as e:
                            logger.warning(
                                f"Skipping invalid transmission '{elem.get('name')}': {e}"
                            )

                    elif tag == "ros2_control":
                        try:
                            ros2_ctrl = self._parse_ros2_control(elem)
                            if ros2_ctrl:
                                delayed_ros2_controls.append(ros2_ctrl)
                        except (RobotModelError, ValueError, Exception) as e:
                            logger.warning(
                                f"Skipping invalid ros2_control '{elem.get('name')}': {e}"
                            )

                    elif tag == "gazebo":
                        try:
                            # 1. Try to extract a sensor if present
                            sensor = self._parse_sensor_from_gazebo(elem)
                            if sensor:
                                delayed_sensors.append(sensor)

                            # 2. Extract physics fields (regardless of sensor presence)
                            physics_data = {
                                "mu": parse_optional_float(elem, "mu1", default=None),
                                "mu2": parse_optional_float(elem, "mu2", default=None),
                                "kp": parse_optional_float(elem, "kp", default=None),
                                "kd": parse_optional_float(elem, "kd", default=None),
                                "self_collide": parse_optional_bool(elem, "selfCollide"),
                                "gravity": parse_optional_bool(elem, "gravity"),
                            }
                            # Filter out None values
                            physics_data = {k: v for k, v in physics_data.items() if v is not None}

                            # 3. Extract other Gazebo metadata (plugins, material, properties)
                            gazebo_elem = self._parse_gazebo_element(elem)
                            delayed_gazebo_elements.append((gazebo_elem, physics_data))
                        except (RobotModelError, ValueError, Exception) as e:
                            logger.warning(
                                f"Skipping invalid gazebo element '{elem.get('name') or elem.get('reference')}': {e}"
                            )

                    elem.clear()

                depth -= 1

        # Second Pass: Process dependent elements
        for joint in delayed_joints:
            try:
                robot.add_joint(joint)
            except Exception as e:
                logger.warning(f"Skipping invalid joint '{joint.name}': {e}")

        for trans in delayed_transmissions:
            try:
                robot.add_transmission(trans)
            except Exception as e:
                logger.warning(f"Skipping invalid transmission '{trans.name}': {e}")

        for ros2_ctrl in delayed_ros2_controls:
            try:
                robot.add_ros2_control(ros2_ctrl)
            except Exception as e:
                logger.warning(f"Skipping invalid ros2_control '{ros2_ctrl.name}': {e}")

        for sensor in delayed_sensors:
            try:
                robot.add_sensor(sensor)
            except Exception as e:
                logger.warning(f"Skipping invalid sensor '{sensor.name}': {e}")

        for gazebo_elem, physics_data in delayed_gazebo_elements:
            try:
                # 1. Apply physics data to link if reference is a link
                if gazebo_elem.reference and robot.has_link(gazebo_elem.reference):
                    link = robot.link(gazebo_elem.reference)
                    # Create updated physics object (cast for replace compatibility)
                    new_physics = replace(link.physics, **cast(dict[str, Any], physics_data))
                    # Replace link with updated physics
                    robot.add_link(replace(link, physics=new_physics), overwrite=True)

                # 2. Add gazebo element if it has unique data (plugins, material, etc.)
                # We skip "empty" gazebo elements that only contained physics
                if (
                    gazebo_elem.plugins
                    or gazebo_elem.material
                    or gazebo_elem.properties
                    or gazebo_elem.static is not None
                    or gazebo_elem.stop_cfm is not None
                    or gazebo_elem.stop_erp is not None
                    or gazebo_elem.provide_feedback is not None
                    or gazebo_elem.implicit_spring_damper is not None
                ):
                    robot.add_gazebo_element(gazebo_elem)
            except Exception as e:
                logger.warning(f"Skipping invalid gazebo element '{gazebo_elem.reference}': {e}")

        return robot

    def parse(self, filepath: Path, **_kwargs: Any) -> Robot:
        """Parse URDF file into a Robot model using iterative parsing.

        This implementation uses iterparse to maintain O(1) memory complexity
        even for massive URDF files.

        Args:
            filepath: Path to the input file
            **kwargs: Additional parsing options

        Returns:
            The generic Robot model (Intermediate Representation)

        Raises:
            RobotParserIOError: If file cannot be read or exceeds size limit
            XacroDetectedError: If a XACRO file is passed instead of URDF
            RobotParserXMLRootError: If root tag is not <robot>
            RobotParserUnexpectedError: If XML is malformed or internal error occurs

        """
        self._validate_file(filepath)

        # Check if this is a XACRO file by extension (proactive check)
        if filepath.suffix == ".xacro" or filepath.name.endswith(".urdf.xacro"):
            raise XacroDetectedError(filepath.name)

        try:
            # We use iterparse to process elements as they are closed
            context = ET.iterparse(str(filepath), events=("start", "end"))
            event, root = next(context)

            if strip_xml_namespace(root.tag) != "robot":
                raise RobotParserXMLRootError(actual_tag=root.tag)

            return self._parse_from_context(context, root, filepath)

        except ET.ParseError as e:
            raise RobotParserUnexpectedError(source_area="URDF XML", original_error=e) from e
        except Exception as e:
            if isinstance(
                e,
                (
                    RobotParserUnexpectedError,
                    RobotParserIOError,
                    RobotParserXMLRootError,
                    XacroDetectedError,
                ),
            ):
                raise
            raise RobotParserUnexpectedError(
                source_area="Unexpected URDF parse", original_error=e
            ) from e

    def parse_string(
        self,
        content: str,
        source_directory: Path | None = None,
        **_kwargs: Any,
    ) -> Robot:
        """Parse URDF from string.

        Args:
            content: URDF XML content as string
            source_directory: Base directory for relative mesh path resolution
            **kwargs: Additional parsing options

        Returns:
            The generic Robot model (Intermediate Representation)

        Raises:
            RobotParserIOError: If string exceeds size limit
            XacroDetectedError: If string contains XACRO markers
            RobotParserXMLRootError: If root tag is not <robot>
            RobotParserUnexpectedError: If XML parsing fails

        """
        self._validate_content(content)

        if "<xacro:" in content:
            raise XacroDetectedError(message="URDF String contains XACRO")

        try:
            stream = io.BytesIO(content.encode("utf-8"))
            context = ET.iterparse(stream, events=("start", "end"))
            event, root = next(context)

            if strip_xml_namespace(root.tag) != "robot":
                raise RobotParserXMLRootError(actual_tag=root.tag)

            return self._parse_from_context(context, root, source_directory)

        except ET.ParseError as e:
            raise RobotParserUnexpectedError(source_area="URDF XML", original_error=e) from e
        except Exception as e:
            if isinstance(
                e,
                (
                    RobotParserUnexpectedError,
                    RobotParserIOError,
                    RobotParserXMLRootError,
                    XacroDetectedError,
                ),
            ):
                raise
            raise RobotParserUnexpectedError(
                source_area="Unexpected URDF parse", original_error=e
            ) from e
