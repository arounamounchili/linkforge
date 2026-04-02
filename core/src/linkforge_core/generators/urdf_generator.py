"""URDF XML generation from robot models.

Transforms internal LinkForge models into Unified Robot Description Format (URDF)
XML files, with support for physics, visuals, sensor modeling, and ROS 2.
"""

from __future__ import annotations

__all__ = [
    "URDFGenerator",
    "format_float",
    "format_vector",
]

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from .. import __version__
from ..base import RobotGeneratorError
from ..logging_config import get_logger
from ..models.gazebo import GazeboElement, GazeboPlugin
from ..models.geometry import Transform
from ..models.joint import Joint, JointType
from ..models.link import Collision, Link, Visual
from ..models.material import Material
from ..models.robot import Robot
from ..models.ros2_control import Ros2Control
from ..models.sensor import (
    CameraInfo,
    ContactInfo,
    ForceTorqueInfo,
    GPSInfo,
    IMUInfo,
    LidarInfo,
    Sensor,
    SensorNoise,
    SensorType,
)
from ..models.transmission import Transmission
from ..utils.math_utils import format_float, format_vector
from ..utils.xml_utils import serialize_xml
from ..validation import RobotValidator
from .xml_base import RobotXMLGenerator

logger = get_logger(__name__)


class URDFGenerator(RobotXMLGenerator):
    """Unified Robot Description Format (URDF) generator."""

    def __init__(
        self,
        pretty_print: bool = True,
        urdf_path: Path | None = None,
        use_ros2_control: bool = True,
    ) -> None:
        """Initialize URDF generator.

        Args:
            pretty_print: If True, format XML with indentation for readability (default: True)
            urdf_path: Path where URDF will be saved. Used to calculate relative mesh paths.
                      If None, mesh paths will be absolute or package:// URIs.
            use_ros2_control: Whether to generate ros2_control blocks from transmissions.
                             Set to False if you don't use ROS2 Control or prefer
                             manual configuration. (default: True)

        Example:
            >>> from pathlib import Path
            >>> # Basic generator with defaults
            >>> generator = URDFGenerator()
            >>>
            >>> # Generator with relative mesh paths
            >>> generator = URDFGenerator(urdf_path=Path("/workspace/robot.urdf"))
        """
        super().__init__(pretty_print=pretty_print, output_path=urdf_path)
        self.urdf_path = urdf_path
        self.use_ros2_control = use_ros2_control

    def generate(self, robot: Robot, validate: bool = True, **kwargs: Any) -> str:
        """Generate URDF XML string from robot.

        Args:
            robot: Robot model with links, joints, sensors, etc.
            validate: Whether to validate robot structure before generation (default: True)
            **kwargs: Additional generation options (e.g. 'compact' to avoid comments)

        Returns:
            URDF XML as formatted string with proper indentation

        Raises:
            RobotGeneratorError: If robot validation fails (checks for cycles, missing links, etc.)
        """
        if kwargs:
            logger.debug(f"URDFGenerator: unused generation options: {list(kwargs.keys())}")

        root = self.generate_robot_element(robot, validate=validate)
        return serialize_xml(root, pretty_print=self.pretty_print, version=__version__)

    def generate_robot_element(self, robot: Robot, validate: bool = True) -> ET.Element:
        """Generate URDF XML Element tree from robot.

        Args:
            robot: Robot model with links, joints, sensors, etc.
            validate: Whether to validate robot structure before generation (default: True)

        Returns:
            URDF XML Element tree
        """
        # Validate robot structure
        if validate:
            validator = RobotValidator()
            result = validator.validate(robot)
            if not result.is_valid:
                error_msgs = [str(issue) for issue in result.errors]
                raise RobotGeneratorError("Robot validation failed:\n" + "\n".join(error_msgs))

        # Create root element
        root = ET.Element("robot", name=robot.name)

        # Add materials (collect unique materials first)
        self.global_materials = self._collect_materials(robot)
        if self.global_materials:
            root.append(ET.Comment(" Materials "))
        # Sort materials by name for deterministic output
        for mat_name in sorted(self.global_materials.keys()):
            self._add_material_element(root, self.global_materials[mat_name])

        # Add links
        self.add_links_section(root, robot)

        # Add joints
        self.add_joints_section(root, robot)

        # Add transmissions
        self.add_transmissions(root, robot)

        # Add ROS2 Control
        if self.use_ros2_control:
            self.add_ros2_control(root, robot)

        # Add Gazebo elements (including sensors - all gazebo tags together)
        # Note: Sensors create <gazebo reference="link"> tags, so we add them first
        # to keep all gazebo elements together at the end
        self.add_gazebo(root, robot)

        # Add sensors (Gazebo format) - these also create <gazebo> tags
        self.add_sensors(root, robot)

        return root

    def add_links_section(self, parent: ET.Element, robot: Robot) -> None:
        """Add Links section to parent element.

        This follows the Template Method pattern. It handles the section header (comment)
        and iteration, delegating the specific element creation to _add_link_to_xml.
        """
        if robot.links:
            parent.append(ET.Comment(" Links "))
        # Sort links by name for deterministic output
        for link in sorted(robot.links, key=lambda link_item: link_item.name):
            self._add_link_to_xml(parent, link)

    def add_joints_section(self, parent: ET.Element, robot: Robot) -> None:
        """Add Joints section to parent element."""
        if robot.joints:
            parent.append(ET.Comment(" Joints "))
        # Sort joints by name for deterministic output
        for joint in sorted(robot.joints, key=lambda joint_item: joint_item.name):
            self._add_joint_to_xml(parent, joint)

    def _add_link_to_xml(self, parent: ET.Element, link: Link) -> None:
        """Hook for adding a single link. Can be overridden (e.g. for XACRO macros)."""
        self._add_link_element(parent, link)

    def _add_joint_to_xml(self, parent: ET.Element, joint: Joint) -> None:
        """Hook for adding a single joint. Can be overridden."""
        self._add_joint_element(parent, joint)

    def _collect_materials(self, robot: Robot) -> dict[str, Material]:
        """Collect unique materials from all links for global material definitions.

        This method scans all visual elements in the robot and identifies materials
        that can be defined globally (at the top of the URDF) versus inline (within
        each visual element).

        Materials with the same name but different colors will be kept as inline
        materials (not in global definitions) to preserve the original URDF semantics.
        Only materials with the same name AND same color are deduplicated.

        Args:
            robot: Robot model to scan for materials

        Returns:
            Dictionary mapping material names to Material objects for global definitions.
            Materials with name conflicts are excluded and will be defined inline instead.

        Note:
            URDF allows both global and inline material definitions:
            - Global: <material name="blue"><color rgba="0 0 1 1"/></material>
            - Inline: <visual><material name="red"><color rgba="1 0 0 1"/></material></visual>
            This method ensures no conflicts by only globalizing consistent materials.

        Example:
            If two links both use Material(name="metal", color=Color(0.5, 0.5, 0.5, 1)),
            the material will be defined globally once.

            If one link uses Material(name="metal", color=Color(0.5, 0.5, 0.5, 1))
            and another uses Material(name="metal", color=Color(0.7, 0.7, 0.7, 1)),
            both will be defined inline to preserve their different colors.
        """
        # Track all materials by name
        materials_by_name: dict[str, list[Material]] = {}

        for link in robot.links:
            # Collect materials from all visual elements
            for visual in link.visuals:
                if visual.material and visual.material.name:
                    mat = visual.material
                    if mat.name not in materials_by_name:
                        materials_by_name[mat.name] = []
                    materials_by_name[mat.name].append(mat)

        # Only export materials that are consistent (same color/texture for all uses)
        global_materials: dict[str, Material] = {}
        for mat_name, mat_list in materials_by_name.items():
            # Check if all materials with this name have the same color/texture
            first_mat = mat_list[0]
            if all(m.color == first_mat.color and m.texture == first_mat.texture for m in mat_list):
                # Consistent material, add to global definitions
                global_materials[mat_name] = first_mat
            # Otherwise, keep as inline materials (don't add to global)

        return global_materials

    def _add_material_element(self, parent: ET.Element, material: Material) -> None:
        """Add material element to parent."""
        mat_elem = ET.SubElement(parent, "material", name=material.name)

        if material.color:
            rgba_str = (
                f"{format_float(material.color.r)} "
                f"{format_float(material.color.g)} "
                f"{format_float(material.color.b)} "
                f"{format_float(material.color.a)}"
            )
            ET.SubElement(mat_elem, "color", rgba=rgba_str)

        if material.texture:
            ET.SubElement(mat_elem, "texture", filename=material.texture)

    def _add_link_element(self, parent: ET.Element, link: Link) -> None:
        """Add link element to parent."""
        link_elem = ET.SubElement(parent, "link", name=link.name)
        self._add_link_contents(link_elem, link)

    def _add_link_contents(self, link_elem: ET.Element, link: Link) -> None:
        """Populate link element with inertial, visual, and collision data.

        This is shared between standard links and XACRO macros.
        """
        # Inertial
        if link.inertial:
            self._add_inertial_element(link_elem, link.inertial)

        # Visuals
        for visual in link.visuals:
            self._add_visual_element(link_elem, visual)

        # Collisions
        for collision in link.collisions:
            self._add_collision_element(link_elem, collision)

    def _add_visual_element(self, parent: ET.Element, visual: Visual) -> None:
        """Add visual element to parent."""
        visual_elem = ET.SubElement(parent, "visual")

        if visual.name:
            visual_elem.set("name", visual.name)

        # Origin
        self._add_origin_element(visual_elem, visual.origin)

        # Geometry
        self._add_geometry_element(visual.geometry, visual_elem)

        # Material (reference if in global materials, inline otherwise)
        if visual.material:
            # Check if this material is in global materials
            is_global = (
                visual.material.name
                and visual.material.name in self.global_materials
                and self.global_materials[visual.material.name] == visual.material
            )

            if is_global:
                # Reference to global material
                ET.SubElement(visual_elem, "material", name=visual.material.name)
            else:
                # Inline material (either unnamed or has different color than global)
                mat_elem = ET.SubElement(
                    visual_elem,
                    "material",
                    name=visual.material.name if visual.material.name else "",
                )
                if visual.material.color:
                    rgba_str = (
                        f"{format_float(visual.material.color.r)} "
                        f"{format_float(visual.material.color.g)} "
                        f"{format_float(visual.material.color.b)} "
                        f"{format_float(visual.material.color.a)}"
                    )
                    ET.SubElement(mat_elem, "color", rgba=rgba_str)

                if visual.material.texture:
                    ET.SubElement(mat_elem, "texture", filename=visual.material.texture)

    def _add_collision_element(self, parent: ET.Element, collision: Collision) -> None:
        """Add collision element to parent."""
        collision_elem = ET.SubElement(parent, "collision")

        if collision.name:
            collision_elem.set("name", collision.name)

        # Origin
        self._add_origin_element(collision_elem, collision.origin)

        # Geometry
        self._add_geometry_element(collision.geometry, collision_elem)

    def _add_joint_element(self, parent: ET.Element, joint: Joint) -> None:
        """Add joint element to parent."""
        joint_elem = ET.SubElement(parent, "joint", name=joint.name, type=joint.type.value)

        # Origin
        self._add_origin_element(joint_elem, joint.origin)

        # Parent and child
        ET.SubElement(joint_elem, "parent", link=joint.parent)
        ET.SubElement(joint_elem, "child", link=joint.child)

        # Properties (Axis, Limits, Mimic, etc.)
        self._add_joint_properties(joint_elem, joint)

    def _add_joint_properties(self, joint_elem: ET.Element, joint: Joint) -> None:
        """Add functional properties to joint element."""
        # Axis (only for revolute, continuous, prismatic, planar)
        self._add_joint_axis(joint_elem, joint)
        self._add_joint_limits(joint_elem, joint)
        self._add_joint_dynamics(joint_elem, joint)
        self._add_joint_mimic(joint_elem, joint)
        self._add_joint_safety(joint_elem, joint)
        self._add_joint_calibration(joint_elem, joint)

    def _add_joint_axis(self, joint_elem: ET.Element, joint: Joint) -> None:
        """Add joint axis if applicable."""
        if (
            joint.type
            in (
                JointType.REVOLUTE,
                JointType.CONTINUOUS,
                JointType.PRISMATIC,
                JointType.PLANAR,
            )
            and joint.axis
        ):
            axis_str = format_vector(joint.axis.x, joint.axis.y, joint.axis.z)
            ET.SubElement(joint_elem, "axis", xyz=axis_str)

    def _add_joint_limits(self, joint_elem: ET.Element, joint: Joint) -> None:
        """Add joint mechanical limits."""
        if joint.limits:
            attrib: dict[str, str] = {
                "effort": format_float(joint.limits.effort),
                "velocity": format_float(joint.limits.velocity),
            }
            # Only add lower/upper if they are specified (not for CONTINUOUS joints)
            if joint.limits.lower is not None:
                attrib["lower"] = format_float(joint.limits.lower)
            if joint.limits.upper is not None:
                attrib["upper"] = format_float(joint.limits.upper)
            self._create_element(joint_elem, "limit", **attrib)

    def _add_joint_dynamics(self, joint_elem: ET.Element, joint: Joint) -> None:
        """Add joint dynamics like damping and friction."""
        if joint.dynamics:
            ET.SubElement(
                joint_elem,
                "dynamics",
                damping=format_float(joint.dynamics.damping),
                friction=format_float(joint.dynamics.friction),
            )

    def _add_joint_mimic(self, joint_elem: ET.Element, joint: Joint) -> None:
        """Add joint mimic information if applicable."""
        if joint.mimic:
            mimic_attrib: dict[str, str] = {"joint": joint.mimic.joint}
            if joint.mimic.multiplier != 1.0:
                mimic_attrib["multiplier"] = format_float(joint.mimic.multiplier)
            if joint.mimic.offset != 0.0:
                mimic_attrib["offset"] = format_float(joint.mimic.offset)
            self._create_element(joint_elem, "mimic", **mimic_attrib)

    def _add_joint_safety(self, joint_elem: ET.Element, joint: Joint) -> None:
        """Add safety controller limits."""
        if joint.safety_controller:
            safety_attrib: dict[str, str] = {
                "soft_lower_limit": format_float(joint.safety_controller.soft_lower_limit),
                "soft_upper_limit": format_float(joint.safety_controller.soft_upper_limit),
                "k_position": format_float(joint.safety_controller.k_position),
                "k_velocity": format_float(joint.safety_controller.k_velocity),
            }
            self._create_element(joint_elem, "safety_controller", **safety_attrib)

    def _add_joint_calibration(self, joint_elem: ET.Element, joint: Joint) -> None:
        """Add joint calibration offsets."""
        if joint.calibration:
            calib_attrib: dict[str, str] = {}
            if joint.calibration.rising is not None:
                calib_attrib["rising"] = format_float(joint.calibration.rising)
            if joint.calibration.falling is not None:
                calib_attrib["falling"] = format_float(joint.calibration.falling)

            if calib_attrib:
                self._create_element(joint_elem, "calibration", **calib_attrib)

    def _add_transmission_element(self, parent: ET.Element, transmission: Transmission) -> None:
        """Add transmission element to parent.

        Args:
            parent: Parent XML element
            transmission: Transmission model

        """
        trans_elem = ET.SubElement(parent, "transmission", name=transmission.name)

        # Add type
        type_elem = ET.SubElement(trans_elem, "type")
        type_elem.text = transmission.type

        # Add joints
        for trans_joint in transmission.joints:
            joint_elem = ET.SubElement(trans_elem, "joint", name=trans_joint.name)

            # Add hardware interfaces (ROS2 short names)
            for interface in trans_joint.hardware_interfaces:
                iface_elem = ET.SubElement(joint_elem, "hardwareInterface")
                # Normalize to short name (position, velocity, effort)
                iface_elem.text = self._normalize_interface_name(interface)

            # Add mechanical reduction if not default
            if trans_joint.mechanical_reduction != 1.0:
                reduction_elem = ET.SubElement(joint_elem, "mechanicalReduction")
                reduction_elem.text = format_float(trans_joint.mechanical_reduction)

            # Add offset if not default
            if trans_joint.offset != 0.0:
                offset_elem = ET.SubElement(joint_elem, "offset")
                offset_elem.text = format_float(trans_joint.offset)

        # Add actuators
        for actuator in transmission.actuators:
            actuator_elem = ET.SubElement(trans_elem, "actuator", name=actuator.name)

            # Add hardware interfaces (ROS2 short names)
            for interface in actuator.hardware_interfaces:
                iface_elem = ET.SubElement(actuator_elem, "hardwareInterface")
                # Normalize to short name (position, velocity, effort)
                iface_elem.text = self._normalize_interface_name(interface)

            # Always export mechanical reduction for model fidelity and deterministic round-trips,
            # even if it is 1.0. This ensures consistency across various URDF configurations.
            # Add mechanical reduction (always include for round-trip fidelity)
            if actuator.mechanical_reduction is not None:
                reduction_elem = ET.SubElement(actuator_elem, "mechanicalReduction")
                reduction_elem.text = format_float(actuator.mechanical_reduction)

            # Add offset if not default
            if actuator.offset != 0.0:
                offset_elem = ET.SubElement(actuator_elem, "offset")
                offset_elem.text = format_float(actuator.offset)

    def _add_sensor_element(self, parent: ET.Element, sensor: Sensor) -> None:
        """Add Gazebo sensor element to parent (within gazebo tag).

        Args:
            parent: Parent XML element (robot)
            sensor: Sensor model

        Note:
            Sensors in URDF/Gazebo are typically wrapped in a <gazebo reference="link_name"> tag.
            This creates a Gazebo-specific sensor element.

        """

        # Create gazebo element referencing the link
        gazebo_elem = ET.SubElement(parent, "gazebo", reference=sensor.link_name)

        # Map internal types to Gazebo Sim modern types
        # "lidar" -> "gpu_lidar" for performance
        # "gps" -> "navsat"
        type_mapping = {
            SensorType.LIDAR.value: "gpu_lidar",
            SensorType.GPS.value: "navsat",
            SensorType.CAMERA.value: "camera",
            SensorType.DEPTH_CAMERA.value: "depth_camera",
            SensorType.IMU.value: "imu",
            SensorType.CONTACT.value: "contact",
            SensorType.FORCE_TORQUE.value: "force_torque",
        }

        sim_type = type_mapping.get(sensor.type.value, sensor.type.value)
        sensor_elem = ET.SubElement(gazebo_elem, "sensor", name=sensor.name, type=sim_type)

        # Standard sensor attributes
        ET.SubElement(sensor_elem, "always_on").text = "true"
        ET.SubElement(sensor_elem, "update_rate").text = format_float(sensor.update_rate)
        ET.SubElement(sensor_elem, "visualize").text = "true" if sensor.visualize else "false"

        # Topic and Pose
        if sensor.topic:
            ET.SubElement(sensor_elem, "topic").text = sensor.topic

        # Export sensor pose (transform relative to parent link)
        # Gazebo uses <pose>x y z roll pitch yaw</pose> format
        if sensor.origin != Transform.identity():
            pose_str = (
                f"{format_float(sensor.origin.xyz.x)} "
                f"{format_float(sensor.origin.xyz.y)} "
                f"{format_float(sensor.origin.xyz.z)} "
                f"{format_float(sensor.origin.rpy.x)} "
                f"{format_float(sensor.origin.rpy.y)} "
                f"{format_float(sensor.origin.rpy.z)}"
            )
            pose_elem = ET.SubElement(sensor_elem, "pose")
            pose_elem.text = pose_str

        # Add sensor specific info
        if sensor.camera_info:
            self._add_camera_sensor_info(sensor_elem, sensor.camera_info, sim_type)
        elif sensor.lidar_info:
            self._add_lidar_sensor_info(sensor_elem, sensor.lidar_info)
        elif sensor.imu_info:
            self._add_imu_sensor_info(sensor_elem, sensor.imu_info)
        elif sensor.gps_info:
            self._add_gps_sensor_info(sensor_elem, sensor.gps_info)
        elif sensor.contact_info:
            self._add_contact_sensor_info(sensor_elem, sensor.contact_info)
        elif sensor.force_torque_info:
            self._add_force_torque_sensor_info(sensor_elem, sensor.force_torque_info)

        # No plugins added here - Gazebo Sim bridge handles data transport

        # Add plugin if specified
        if sensor.plugin:
            self._add_gazebo_plugin_element(sensor_elem, sensor.plugin)

    def _add_camera_sensor_info(
        self, parent: ET.Element, camera_info: CameraInfo, sensor_type: str
    ) -> None:
        """Add camera sensor information."""

        camera_elem = ET.SubElement(parent, "camera")

        # Horizontal FOV
        hfov_elem = ET.SubElement(camera_elem, "horizontal_fov")
        hfov_elem.text = format_float(camera_info.horizontal_fov)

        # Image
        image_elem = ET.SubElement(camera_elem, "image")
        width_elem = ET.SubElement(image_elem, "width")
        width_elem.text = str(camera_info.width)
        height_elem = ET.SubElement(image_elem, "height")
        height_elem.text = str(camera_info.height)
        format_elem = ET.SubElement(image_elem, "format")
        format_elem.text = camera_info.format

        # Clip
        clip_elem = ET.SubElement(camera_elem, "clip")
        near_elem = ET.SubElement(clip_elem, "near")
        near_elem.text = format_float(camera_info.near_clip)
        far_elem = ET.SubElement(clip_elem, "far")
        far_elem.text = format_float(camera_info.far_clip)

        # For depth camera, add depth camera element
        if sensor_type == "depth_camera":
            depth_cam_elem = ET.SubElement(parent, "depth_camera")
            output_elem = ET.SubElement(depth_cam_elem, "output")
            type_elem = ET.SubElement(output_elem, "type")
            type_elem.text = "depth"

        # Noise
        if camera_info.noise:
            self._add_sensor_noise(camera_elem, camera_info.noise)

    def _add_lidar_sensor_info(self, parent: ET.Element, lidar_info: LidarInfo) -> None:
        """Add LIDAR/ray sensor information."""
        ray_elem = ET.SubElement(parent, "ray")

        # Horizontal scan configuration
        scan_elem = ET.SubElement(ray_elem, "scan")
        horizontal_elem = ET.SubElement(scan_elem, "horizontal")

        samples_elem = ET.SubElement(horizontal_elem, "samples")
        samples_elem.text = str(lidar_info.horizontal_samples)

        resolution_elem = ET.SubElement(horizontal_elem, "resolution")
        resolution_elem.text = format_float(lidar_info.horizontal_resolution)

        min_angle_elem = ET.SubElement(horizontal_elem, "min_angle")
        min_angle_elem.text = format_float(lidar_info.horizontal_min_angle)

        max_angle_elem = ET.SubElement(horizontal_elem, "max_angle")
        max_angle_elem.text = format_float(lidar_info.horizontal_max_angle)

        # Vertical scan (if 3D LIDAR)
        if lidar_info.vertical_samples > 1:
            vertical_elem = ET.SubElement(scan_elem, "vertical")
            v_samples_elem = ET.SubElement(vertical_elem, "samples")
            v_samples_elem.text = str(lidar_info.vertical_samples)
            v_resolution_elem = ET.SubElement(vertical_elem, "resolution")
            v_resolution_elem.text = format_float(lidar_info.vertical_resolution)
            v_min_angle_elem = ET.SubElement(vertical_elem, "min_angle")
            v_min_angle_elem.text = format_float(lidar_info.vertical_min_angle)
            v_max_angle_elem = ET.SubElement(vertical_elem, "max_angle")
            v_max_angle_elem.text = format_float(lidar_info.vertical_max_angle)

        # Range
        range_elem = ET.SubElement(ray_elem, "range")
        min_elem = ET.SubElement(range_elem, "min")
        min_elem.text = format_float(lidar_info.range_min)
        max_elem = ET.SubElement(range_elem, "max")
        max_elem.text = format_float(lidar_info.range_max)
        resolution_elem = ET.SubElement(range_elem, "resolution")
        resolution_elem.text = format_float(lidar_info.range_resolution)

        # Noise (if specified)
        if lidar_info.noise:
            self._add_sensor_noise(ray_elem, lidar_info.noise)

    def _add_imu_sensor_info(self, parent: ET.Element, imu_info: IMUInfo) -> None:
        """Add IMU sensor information."""
        imu_elem = ET.SubElement(parent, "imu")

        # Add noise for different measurements
        if imu_info.angular_velocity_noise:
            ang_vel_elem = ET.SubElement(imu_elem, "angular_velocity")
            # Add noise for x, y, z
            for axis in ["x", "y", "z"]:
                axis_elem = ET.SubElement(ang_vel_elem, axis)
                self._add_sensor_noise(axis_elem, imu_info.angular_velocity_noise)

        if imu_info.linear_acceleration_noise:
            lin_acc_elem = ET.SubElement(imu_elem, "linear_acceleration")
            # Add noise for x, y, z
            for axis in ["x", "y", "z"]:
                axis_elem = ET.SubElement(lin_acc_elem, axis)
                self._add_sensor_noise(axis_elem, imu_info.linear_acceleration_noise)

        # Gravity is handled by World settings in Gazebo
        # (No sensor-specific gravity magnitude setting)

    def _add_gps_sensor_info(self, parent: ET.Element, gps_info: GPSInfo) -> None:
        """Add GPS sensor information."""
        gps_elem = ET.SubElement(parent, "navsat")

        # Position noise
        if gps_info.position_sensing_horizontal_noise:
            pos_elem = ET.SubElement(gps_elem, "position_sensing")
            horiz_elem = ET.SubElement(pos_elem, "horizontal")
            self._add_sensor_noise(
                horiz_elem, gps_info.position_sensing_horizontal_noise, prefix=""
            )

        if gps_info.position_sensing_vertical_noise:
            if not gps_info.position_sensing_horizontal_noise:
                pos_elem = ET.SubElement(gps_elem, "position_sensing")
            vert_elem = ET.SubElement(pos_elem, "vertical")
            self._add_sensor_noise(vert_elem, gps_info.position_sensing_vertical_noise, prefix="")

        # Velocity noise
        if gps_info.velocity_sensing_horizontal_noise or gps_info.velocity_sensing_vertical_noise:
            vel_sensing_elem = ET.SubElement(gps_elem, "velocity_sensing")
            if gps_info.velocity_sensing_horizontal_noise:
                vel_horiz_elem = ET.SubElement(vel_sensing_elem, "horizontal")
                self._add_sensor_noise(vel_horiz_elem, gps_info.velocity_sensing_horizontal_noise)
            if gps_info.velocity_sensing_vertical_noise:
                vel_vert_elem = ET.SubElement(vel_sensing_elem, "vertical")
                self._add_sensor_noise(vel_vert_elem, gps_info.velocity_sensing_vertical_noise)

    def _add_contact_sensor_info(self, parent: ET.Element, contact_info: ContactInfo) -> None:
        """Add contact sensor information."""
        contact_elem = ET.SubElement(parent, "contact")

        # Add collision element
        if contact_info.collision:
            coll_elem = ET.SubElement(contact_elem, "collision")
            coll_elem.text = contact_info.collision

        # Add noise if specified
        if contact_info.noise:
            self._add_sensor_noise(contact_elem, contact_info.noise)

    def _add_force_torque_sensor_info(self, parent: ET.Element, ft_info: ForceTorqueInfo) -> None:
        """Add force/torque sensor information."""
        ft_elem = ET.SubElement(parent, "force_torque")

        # Frame
        frame_elem = ET.SubElement(ft_elem, "frame")
        frame_elem.text = ft_info.frame

        # Measure direction
        measure_elem = ET.SubElement(ft_elem, "measure_direction")
        measure_elem.text = ft_info.measure_direction

        # Add noise if specified
        if ft_info.noise:
            self._add_sensor_noise(ft_elem, ft_info.noise, prefix="")

    def _add_sensor_noise(
        self, parent: ET.Element, noise: SensorNoise, prefix: str = "noise"
    ) -> None:
        """Add noise model to sensor element."""
        noise_elem = ET.SubElement(parent, prefix) if prefix else parent

        type_elem = ET.SubElement(noise_elem, "type")
        type_elem.text = noise.type

        if noise.mean != 0.0:
            mean_elem = ET.SubElement(noise_elem, "mean")
            mean_elem.text = format_float(noise.mean)

        if noise.stddev != 0.0:
            stddev_elem = ET.SubElement(noise_elem, "stddev")
            stddev_elem.text = format_float(noise.stddev)

        if noise.bias_mean != 0.0:
            bias_mean_elem = ET.SubElement(noise_elem, "bias_mean")
            bias_mean_elem.text = format_float(noise.bias_mean)

        if noise.bias_stddev != 0.0:
            bias_stddev_elem = ET.SubElement(noise_elem, "bias_stddev")
            bias_stddev_elem.text = format_float(noise.bias_stddev)

    @staticmethod
    def _add_optional_bool_element(parent: ET.Element, tag: str, value: bool | None) -> None:
        """Add optional boolean XML element if value is not None."""
        if value is not None:
            elem = ET.SubElement(parent, tag)
            elem.text = "true" if value else "false"

    @staticmethod
    def _add_optional_numeric_element(
        parent: ET.Element, tag: str, value: float | int | None
    ) -> None:
        """Add optional numeric XML element if value is not None."""
        if value is not None:
            elem = ET.SubElement(parent, tag)
            elem.text = format_float(float(value))

    def _add_gazebo_element(self, parent: ET.Element, gazebo_elem: GazeboElement) -> None:
        """Add Gazebo extension element to parent.

        Args:
            parent: Parent XML element
            gazebo_elem: GazeboElement model

        """
        # Create gazebo element with optional reference attribute
        attrib: dict[str, str] = {}
        if gazebo_elem.reference is not None:
            attrib["reference"] = gazebo_elem.reference

        gz_elem = self._create_element(parent, "gazebo", **attrib)

        # Add material if specified
        if gazebo_elem.material is not None:
            material_elem = ET.SubElement(gz_elem, "material")
            material_elem.text = gazebo_elem.material

        # Add boolean properties
        self._add_optional_bool_element(gz_elem, "selfCollide", gazebo_elem.self_collide)
        self._add_optional_bool_element(gz_elem, "static", gazebo_elem.static)
        self._add_optional_bool_element(gz_elem, "gravity", gazebo_elem.gravity)
        self._add_optional_bool_element(gz_elem, "provideFeedback", gazebo_elem.provide_feedback)
        self._add_optional_bool_element(
            gz_elem, "implicitSpringDamper", gazebo_elem.implicit_spring_damper
        )

        # Add numeric properties
        self._add_optional_numeric_element(gz_elem, "mu1", gazebo_elem.mu1)
        self._add_optional_numeric_element(gz_elem, "mu2", gazebo_elem.mu2)
        self._add_optional_numeric_element(gz_elem, "kp", gazebo_elem.kp)
        self._add_optional_numeric_element(gz_elem, "kd", gazebo_elem.kd)
        self._add_optional_numeric_element(gz_elem, "stopCfm", gazebo_elem.stop_cfm)
        self._add_optional_numeric_element(gz_elem, "stopErp", gazebo_elem.stop_erp)

        # Add custom properties
        for key, value in gazebo_elem.properties.items():
            prop_elem = ET.SubElement(gz_elem, key)
            prop_elem.text = value

        # Add plugins
        for plugin in gazebo_elem.plugins:
            self._add_gazebo_plugin_element(gz_elem, plugin)

    def _add_gazebo_plugin_element(self, parent: ET.Element, plugin: GazeboPlugin) -> None:
        """Add Gazebo plugin element to parent.

        Args:
            parent: Parent XML element
            plugin: GazeboPlugin model

        """
        plugin_elem = ET.SubElement(parent, "plugin", name=plugin.name, filename=plugin.filename)

        # If raw XML is available (from import), use it for perfect round-trip
        if plugin.raw_xml:
            # Parse and append the raw XML content
            try:
                # Wrap in temporary root to parse multiple elements
                temp_xml = f"<temp>{plugin.raw_xml}</temp>"
                temp_root = ET.fromstring(temp_xml)

                # Strip whitespace-only text nodes to prevent empty lines
                def strip_whitespace_nodes(elem: ET.Element) -> None:
                    """Recursively remove whitespace-only text nodes."""
                    # Remove whitespace-only tail
                    if elem.tail and not elem.tail.strip():
                        elem.tail = None
                    # Remove whitespace-only text
                    if elem.text and not elem.text.strip():
                        elem.text = None
                    # Recurse to children
                    for child in elem:
                        strip_whitespace_nodes(child)

                for child in temp_root:
                    strip_whitespace_nodes(child)
                    plugin_elem.append(child)
            except ET.ParseError:
                # Fall back to parameters if raw XML is malformed
                for key, value in plugin.parameters.items():
                    param_elem = ET.SubElement(plugin_elem, key)
                    param_elem.text = value
        else:
            # Add all parameters as sub-elements (for manually created plugins)
            for key, value in plugin.parameters.items():
                param_elem = ET.SubElement(plugin_elem, key)
                param_elem.text = value

    def add_ros2_control(self, parent: ET.Element, robot: Robot) -> None:
        """Add ros2_control to parent element.

        This method centralizes the logic for choosing between parsed (centralized)
        ROS 2 Control configuration and standard transmission-based generation.
        It ensures consistent XML structure and comments across URDF and XACRO export.

        Args:
            parent: Parent XML element (robot)
            robot: Robot model
        """
        # Priority: use parsed ros2_control if available, otherwise generate from transmissions
        if robot.ros2_controls:
            # Use parsed ros2_control data (Preferred)
            parent.append(ET.Comment(" ROS2 Control "))
            for rc in robot.ros2_controls:
                self._add_parsed_ros2_control_element(parent, rc)
        elif robot.transmissions:
            # Generate ros2_control from transmissions if enabled
            parent.append(ET.Comment(" ROS2 Control "))
            self._add_ros2_control_element(parent, robot)

    def add_transmissions(self, parent: ET.Element, robot: Robot) -> None:
        """Add transmissions section to parent element.

        Args:
            parent: Parent XML element (robot)
            robot: Robot model
        """
        if robot.transmissions:
            parent.append(ET.Comment(" Transmissions "))
        for transmission in robot.transmissions:
            self._add_transmission_element(parent, transmission)

    def add_gazebo(self, parent: ET.Element, robot: Robot) -> None:
        """Add Gazebo section to parent element.

        Args:
            parent: Parent XML element (robot)
            robot: Robot model
        """
        if robot.gazebo_elements:
            parent.append(ET.Comment(" Gazebo "))
        for gazebo_elem in robot.gazebo_elements:
            self._add_gazebo_element(parent, gazebo_elem)

    def add_sensors(self, parent: ET.Element, robot: Robot) -> None:
        """Add Sensors section to parent element.

        Args:
            parent: Parent XML element (robot)
            robot: Robot model
        """
        if robot.sensors:
            parent.append(ET.Comment(" Sensors "))
        for sensor in robot.sensors:
            self._add_sensor_element(parent, sensor)

    def _add_ros2_control_element(self, parent: ET.Element, robot: Robot) -> None:
        """Add ros2_control block for ROS2 standard with Gazebo Sim (Ignition) plugin.

        Args:
            parent: Parent XML element
            robot: Robot model

        """
        # Create ros2_control element with Gazebo Sim system
        # Using "gz_ros2_control/GazeboSimSystem" which is the modern standard
        rc_elem = ET.SubElement(parent, "ros2_control", name="GazeboSimSystem", type="system")

        # Hardware plugin for Gazebo Sim
        hw_elem = ET.SubElement(rc_elem, "hardware")
        plugin_elem = ET.SubElement(hw_elem, "plugin")
        plugin_elem.text = "gz_ros2_control/GazeboSimSystem"

        # Process transmissions to extract joint interfaces
        for trans in robot.transmissions:
            for trans_joint in trans.joints:
                joint_elem = ET.SubElement(rc_elem, "joint", name=trans_joint.name)

                # Collect all command interfaces
                command_interfaces = []
                for hw_interface in trans_joint.hardware_interfaces:
                    interface_name = self._normalize_interface_name(hw_interface)
                    command_interfaces.append(interface_name)
                    # Add as command interface
                    ET.SubElement(joint_elem, "command_interface", name=interface_name)

                # Always add standard state interfaces
                ET.SubElement(joint_elem, "state_interface", name="position")
                ET.SubElement(joint_elem, "state_interface", name="velocity")

                # Add effort state interface if using effort control
                if "effort" in command_interfaces:
                    ET.SubElement(joint_elem, "state_interface", name="effort")

        # Add Gazebo Sim ros2_control plugin (gz_ros2_control)
        # Check if plugin already exists in robot model
        plugin_exists = False
        for gz in robot.gazebo_elements:
            for p in gz.plugins:
                if "ros2_control" in p.name:
                    plugin_exists = True
                    break
            if plugin_exists:
                break

        if not plugin_exists:
            gz_elem = ET.SubElement(parent, "gazebo")
            # Use libgz_ros2_control-system.so for modern Gazebo
            gz_plugin = ET.SubElement(
                gz_elem,
                "plugin",
                filename="libgz_ros2_control-system.so",
                name="gz_ros2_control::GazeboSimROS2ControlPlugin",
            )
            # Add robot description parameter
            robot_param = ET.SubElement(gz_plugin, "parameters")
            robot_param.text = "$(find robot_description)/config/controllers.yaml"

    def _add_parsed_ros2_control_element(self, parent: ET.Element, rc: Ros2Control) -> None:
        """Add ros2_control element from parsed data.

        Args:
            parent: Parent XML element
            rc: Ros2Control model from parsed URDF
        """
        rc_elem = ET.SubElement(parent, "ros2_control", name=rc.name, type=rc.type)

        # Hardware plugin
        hw_elem = ET.SubElement(rc_elem, "hardware")
        plugin_elem = ET.SubElement(hw_elem, "plugin")
        plugin_elem.text = rc.hardware_plugin

        # Hardware-level parameters
        for key, value in rc.parameters.items():
            param_elem = ET.SubElement(hw_elem, "param", name=key)
            param_elem.text = value

        # Joints
        for joint in rc.joints:
            joint_elem = ET.SubElement(rc_elem, "joint", name=joint.name)

            # Command interfaces
            for cmd_iface in joint.command_interfaces:
                ET.SubElement(joint_elem, "command_interface", name=cmd_iface)

            # State interfaces
            for state_iface in joint.state_interfaces:
                ET.SubElement(joint_elem, "state_interface", name=state_iface)

            # Joint parameters
            for key, value in joint.parameters.items():
                param_elem = ET.SubElement(joint_elem, "param", name=key)
                param_elem.text = value

    def _normalize_interface_name(self, hw_interface: str) -> str:
        """Normalize hardware interface name to ROS2 standard short name.

        Accepts both short names (position, velocity, effort) and full interface names
        (e.g. hardware_interface/PositionJointInterface) and returns the short form.

        Args:
            hw_interface: Hardware interface string

        Returns:
            ROS2 standard short name (position, velocity, or effort)

        """
        hw_lower = hw_interface.lower()
        if "position" in hw_lower:
            return "position"
        if "velocity" in hw_lower:
            return "velocity"
        if "effort" in hw_lower:
            return "effort"
        return "position"  # Default fallback
