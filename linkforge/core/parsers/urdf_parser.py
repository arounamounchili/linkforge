"""URDF XML parser to import robot models.

This module implements a robust URDF/XACRO parser that transforms XML robot
descriptions into the internal LinkForge `Robot` model. It is designed for
high-fidelity round-tripping and includes:

- **Comprehensive Parsing**: Full support for all URDF joint types, multiple
  visual/collision elements, and complex nested structures.
- **XACRO Documentation Support**: Integration with `xacrodoc` for seamless
  processing of parameterized models and relative includes.
- **Security & Validation**: Built-in protection against XML attacks (depth
  limits) and strict validation of mesh paths and numeric values.
- **Error Resilience**: Informative error messages and fallback mechanisms
  for malformed or incomplete XML configurations.
- **Fidelity Preservation**: Captures metadata, sensor noise, and ROS 2
  control parameters to ensure nothing is lost during the Blender import process.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

from ..logging_config import get_logger
from ..models import (
    Box,
    CameraInfo,
    Collision,
    Color,
    ContactInfo,
    Cylinder,
    ForceTorqueInfo,
    GazeboElement,
    GazeboPlugin,
    GPSInfo,
    IMUInfo,
    Inertial,
    InertiaTensor,
    Joint,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointType,
    LidarInfo,
    Link,
    Material,
    Mesh,
    Robot,
    Ros2Control,
    Ros2ControlJoint,
    Sensor,
    SensorNoise,
    SensorType,
    Sphere,
    Transform,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    Vector3,
    Visual,
)
from ..validation import validate_mesh_path

logger = get_logger(__name__)

# Resource limits to prevent DoS attacks
MAX_FILE_SIZE = 100 * 1024 * 1024  # 100 MB (generous for robot models with many links)
MAX_XML_DEPTH = 100  # Maximum nesting depth (typical URDF: <robot><link><visual><geometry>...</geometry></visual></link></robot> = ~5 levels)


def _validate_xml_depth(element: ET.Element, depth: int) -> None:
    """Validate XML depth to prevent billion laughs attack.

    Args:
        element: XML element to validate
        depth: Current nesting depth

    Raises:
        ValueError: If depth exceeds MAX_XML_DEPTH
    """
    if depth > MAX_XML_DEPTH:
        raise ValueError(
            f"XML nesting too deep: {depth} levels (maximum {MAX_XML_DEPTH}). "
            "This may indicate a malicious or corrupted URDF file."
        )

    for child in element:
        _validate_xml_depth(child, depth + 1)


def parse_float(
    text: str | None, attribute_name: str = "value", default: float | None = None
) -> float:
    """Parse float value from URDF XML with comprehensive validation.

    This function ensures safe parsing of numeric values from URDF files by:
    - Rejecting NaN and Inf values (can crash physics engines)
    - Validating reasonable value ranges
    - Providing clear error messages with context

    Args:
        text: String to parse as float (e.g., "1.5", "-3.14")
        attribute_name: Name of the attribute being parsed, used in error messages
                       (e.g., "mass", "radius", "xyz")
        default: Default value if text is None. Note: malformed text will still
                raise ValueError even if default is provided.

    Returns:
        Parsed and validated float value

    Raises:
        ValueError: If text is None and no default is provided
        ValueError: If text is not a valid float
        ValueError: If value is NaN, Inf, or outside reasonable range (-1e10 to 1e10)

    Example:
        >>> parse_float("1.5", "mass")
        1.5
        >>> parse_float(None, "mass", default=1.0)
        1.0
        >>> parse_float("NaN", "mass")
        ValueError: Invalid mass value 'NaN': NaN (Not a Number) values are not allowed
        >>> parse_float(None, "mass")
        ValueError: Missing required attribute 'mass'

    Note:
        The reasonable range check prevents values like 1e308 that could cause
        numerical issues in physics simulations. Most robot dimensions are in
        the range of 0.001 (1mm) to 100 (100m).
    """
    # Treat empty or whitespace-only strings as None
    if text is not None and not text.strip():
        text = None

    if text is None:
        if default is not None:
            return default
        raise ValueError(f"Missing required attribute '{attribute_name}'")

    # If text is provided, it must be valid (don't fall back to default for malformed input)
    try:
        import math

        value = float(text)

        # Check for NaN and Inf which can cause issues in physics calculations
        if math.isnan(value):
            raise ValueError("NaN (Not a Number) values are not allowed")
        if math.isinf(value):
            raise ValueError("Infinite values are not allowed")

        # Sanity check for reasonable values
        if not (-1e10 < value < 1e10):
            raise ValueError(f"Value {value} is outside reasonable range (-1e10 to 1e10)")
        return value
    except ValueError as e:
        raise ValueError(f"Invalid {attribute_name} value '{text}': {e}") from e


def parse_int(text: str | None, attribute_name: str = "value", default: int | None = None) -> int:
    """Parse integer value from URDF XML with comprehensive validation.

    Similar to parse_float but for integer values.

    Args:
        text: String to parse as int (e.g., "640", "480")
        attribute_name: Name of the attribute being parsed
        default: Default value if text is None or empty

    Returns:
        Parsed and validated integer value

    Raises:
        ValueError: If text is invalid or out of range
    """
    # Treat empty or whitespace-only strings as None
    if text is not None and not text.strip():
        text = None

    if text is None:
        if default is not None:
            return default
        raise ValueError(f"Missing required attribute '{attribute_name}'")

    try:
        value = int(text)
        # Sanity check for reasonable values
        if not (-1000000 < value < 1000000):
            raise ValueError(f"Value {value} is outside reasonable range")
        return value
    except ValueError as e:
        raise ValueError(f"Invalid {attribute_name} value '{text}': {e}") from e


def parse_vector3(text: str) -> Vector3:
    """Parse space-separated vector3 string from URDF XML.

    URDF uses space-separated strings for 3D vectors in attributes like
    xyz (position), rpy (rotation), size (box dimensions), etc.

    Args:
        text: Space-separated string with 3 numeric values
              Examples: "1.0 2.0 3.0", "0 0 0", "-1.5 3.2 0.1"

    Returns:
        Vector3 object with x, y, z components

    Raises:
        ValueError: If text doesn't contain exactly 3 numeric values
        ValueError: If any component contains NaN, Inf, or invalid numbers

    Example:
        >>> parse_vector3("1.0 2.0 3.0")
        Vector3(x=1.0, y=2.0, z=3.0)
        >>> parse_vector3("0 0 0")
        Vector3(x=0.0, y=0.0, z=0.0)
        >>> parse_vector3("1.0 2.0")
        ValueError: Expected 3 values, got 2: 1.0 2.0

    Note:
        URDF coordinate system: X-forward, Y-left, Z-up (right-handed)
        Rotations use roll-pitch-yaw (RPY) in radians
    """
    parts = text.strip().split()
    if len(parts) != 3:
        raise ValueError(f"Expected 3 values for Vector3, got {len(parts)}: '{text}'")
    try:
        # Parse and validate each component
        x = parse_float(parts[0], "x")
        y = parse_float(parts[1], "y")
        z = parse_float(parts[2], "z")
        return Vector3(x, y, z)
    except (ValueError, IndexError) as e:
        raise ValueError(f"Invalid Vector3 format '{text}': {e}") from e


def parse_origin(elem: ET.Element | None) -> Transform:
    """Parse origin element to Transform."""
    if elem is None:
        return Transform.identity()

    xyz_text = elem.get("xyz", "0 0 0")
    rpy_text = elem.get("rpy", "0 0 0")

    xyz = parse_vector3(xyz_text)
    rpy = parse_vector3(rpy_text)

    return Transform(xyz=xyz, rpy=rpy)


def parse_geometry(
    geom_elem: ET.Element, urdf_directory: Path | None = None
) -> Box | Cylinder | Sphere | Mesh | None:
    """Parse geometry element with security validation for mesh paths.

    Args:
        geom_elem: XML element containing geometry definition
        urdf_directory: Directory containing the URDF file (required for mesh path validation)

    Returns:
        Geometry object (Box, Cylinder, Sphere, or Mesh) or None if no valid geometry

    Raises:
        ValueError: If geometry attributes are invalid or mesh path is unsafe

    """
    # Check for box
    box = geom_elem.find("box")
    if box is not None:
        try:
            size_text = box.get("size")
            if size_text is None:
                raise ValueError("Box geometry missing required 'size' attribute")
            size = parse_vector3(size_text)
            # Validate positive dimensions
            if size.x <= 0 or size.y <= 0 or size.z <= 0:
                raise ValueError(f"Box dimensions must be positive, got: {size}")
            return Box(size=size)
        except ValueError as e:
            logger.warning(f"Invalid box geometry ignored: {e}")
            return None

    # Check for cylinder
    cylinder = geom_elem.find("cylinder")
    if cylinder is not None:
        try:
            radius = parse_float(cylinder.get("radius"), "cylinder radius", default=0.5)
            length = parse_float(cylinder.get("length"), "cylinder length", default=1.0)
            if radius <= 0:
                raise ValueError(f"Cylinder radius must be positive, got: {radius}")
            if length <= 0:
                raise ValueError(f"Cylinder length must be positive, got: {length}")
            return Cylinder(radius=radius, length=length)
        except ValueError as e:
            logger.warning(f"Invalid cylinder geometry ignored: {e}")
            return None

    # Check for sphere
    sphere = geom_elem.find("sphere")
    if sphere is not None:
        try:
            radius = parse_float(sphere.get("radius"), "sphere radius", default=0.5)
            if radius <= 0:
                raise ValueError(f"Sphere radius must be positive, got: {radius}")
            return Sphere(radius=radius)
        except ValueError as e:
            logger.warning(f"Invalid sphere geometry ignored: {e}")
            return None

    # Check for mesh
    mesh = geom_elem.find("mesh")
    if mesh is not None:
        try:
            filename = mesh.get("filename", "")
            if not filename:
                raise ValueError("Mesh geometry missing required 'filename' attribute")

            # Handle package:// URIs (common in ROS URDFs)
            if filename.startswith("package://"):
                # Validate package URI format for security (prevent path traversal)
                from ..validation import validate_package_uri

                try:
                    validate_package_uri(filename)
                except ValueError as e:
                    # Re-raise with more context
                    raise ValueError(f"Package URI validation failed: {e}") from e

                # Package URIs are resolved by ROS/Blender environment
                # Store as-is for later resolution
                mesh_path = Path(filename)
            elif filename.startswith("file://"):
                # Handle file:// URIs (sometimes generated by xacro)
                # Strip scheme to get file path
                filename = filename[7:]
                mesh_path = Path(filename)

                # If path is absolute, it might be outside URDF directory
                # If relative, it will be resolved relative to URDF directory
                if urdf_directory is not None and not mesh_path.is_absolute():
                    # Validate that mesh path doesn't escape URDF directory
                    try:
                        validate_mesh_path(mesh_path, urdf_directory)
                    except ValueError as e:
                        # Re-raise with more context
                        raise ValueError(f"Mesh path validation failed: {e}") from e
            else:
                # Regular file path - validate for security
                mesh_path = Path(filename)
                if urdf_directory is not None:
                    # Validate that mesh path doesn't escape URDF directory
                    try:
                        validate_mesh_path(mesh_path, urdf_directory)
                    except ValueError as e:
                        # Re-raise with more context
                        raise ValueError(f"Mesh path validation failed: {e}") from e

            scale_text = mesh.get("scale", "1 1 1")
            scale = parse_vector3(scale_text)
            # Validate positive scale
            if scale.x <= 0 or scale.y <= 0 or scale.z <= 0:
                raise ValueError(f"Mesh scale must be positive, got: {scale}")
            return Mesh(filepath=mesh_path, scale=scale)
        except ValueError as e:
            logger.warning(f"Invalid mesh geometry ignored: {e}")
            return None

    return None


def parse_material(mat_elem: ET.Element | None, materials: dict[str, Material]) -> Material | None:
    """Parse material element or reference."""
    if mat_elem is None:
        return None

    # Check if it's a reference
    mat_name = mat_elem.get("name", "")
    if mat_name and mat_name in materials:
        return materials[mat_name]

    # Parse color
    color = None
    color_elem = mat_elem.find("color")
    if color_elem is not None:
        rgba_text = color_elem.get("rgba", "0.8 0.8 0.8 1.0")
        parts = rgba_text.strip().split()

        # Validate RGBA array bounds
        if len(parts) < 3:
            raise ValueError(
                f"Invalid RGBA color format: expected at least 3 components (R G B), got {len(parts)}"
            )
        if len(parts) > 4:
            raise ValueError(
                f"Invalid RGBA color format: expected at most 4 components (R G B A), got {len(parts)}"
            )

        # Parse with bounds checking (Color validates 0-1 range)
        color = Color(
            r=float(parts[0]),
            g=float(parts[1]),
            b=float(parts[2]),
            a=float(parts[3]) if len(parts) > 3 else 1.0,
        )

    # Parse texture
    texture = None
    texture_elem = mat_elem.find("texture")
    if texture_elem is not None:
        texture = texture_elem.get("filename")

    if color or texture:
        return Material(name=mat_name if mat_name else "default", color=color, texture=texture)

    return None


def parse_link(
    link_elem: ET.Element, materials: dict[str, Material], urdf_directory: Path | None = None
) -> Link:
    """Parse link element with support for multiple visual/collision elements.

    Args:
        link_elem: XML element containing link definition
        materials: Dictionary of global materials
        urdf_directory: Directory containing the URDF file (for mesh path validation)

    Returns:
        Link object with visuals, collisions, and inertial properties
    """
    name = link_elem.get("name", "unnamed_link")

    # Parse all visual elements (URDF allows multiple <visual> per link)
    visuals: list[Visual] = []
    for visual_elem in link_elem.findall("visual"):
        origin = parse_origin(visual_elem.find("origin"))
        geom_elem = visual_elem.find("geometry")
        geometry = parse_geometry(geom_elem, urdf_directory) if geom_elem is not None else None
        material = parse_material(visual_elem.find("material"), materials)
        visual_name = visual_elem.get("name")  # Optional name attribute

        if geometry:
            visuals.append(
                Visual(geometry=geometry, origin=origin, material=material, name=visual_name)
            )

    # Parse all collision elements (URDF allows multiple <collision> per link)
    collisions: list[Collision] = []
    for collision_elem in link_elem.findall("collision"):
        origin = parse_origin(collision_elem.find("origin"))
        geom_elem = collision_elem.find("geometry")
        geometry = parse_geometry(geom_elem, urdf_directory) if geom_elem is not None else None
        collision_name = collision_elem.get("name")  # Optional name attribute

        if geometry:
            collisions.append(Collision(geometry=geometry, origin=origin, name=collision_name))

    # Parse inertial
    inertial = None
    inertial_elem = link_elem.find("inertial")
    if inertial_elem is not None:
        origin = parse_origin(inertial_elem.find("origin"))

        mass_elem = inertial_elem.find("mass")
        mass = (
            parse_float(mass_elem.get("value"), "mass", default=1.0)
            if mass_elem is not None
            else 1.0
        )

        inertia_elem = inertial_elem.find("inertia")
        if inertia_elem is not None:
            inertia = InertiaTensor(
                ixx=parse_float(inertia_elem.get("ixx"), "ixx", default=1.0),
                ixy=parse_float(inertia_elem.get("ixy"), "ixy", default=0.0),
                ixz=parse_float(inertia_elem.get("ixz"), "ixz", default=0.0),
                iyy=parse_float(inertia_elem.get("iyy"), "iyy", default=1.0),
                iyz=parse_float(inertia_elem.get("iyz"), "iyz", default=0.0),
                izz=parse_float(inertia_elem.get("izz"), "izz", default=1.0),
            )
            inertial = Inertial(mass=mass, origin=origin, inertia=inertia)

    return Link(name=name, visuals=visuals, collisions=collisions, inertial=inertial)


def parse_joint(joint_elem: ET.Element) -> Joint:
    """Parse joint element."""
    name = joint_elem.get("name", "unnamed_joint")
    joint_type_str = joint_elem.get("type", "fixed")

    # Map URDF type string to JointType enum
    type_map = {
        "revolute": JointType.REVOLUTE,
        "continuous": JointType.CONTINUOUS,
        "prismatic": JointType.PRISMATIC,
        "fixed": JointType.FIXED,
        "floating": JointType.FLOATING,
        "planar": JointType.PLANAR,
    }
    joint_type = type_map.get(joint_type_str.lower(), JointType.FIXED)

    # Parse parent and child
    parent_elem = joint_elem.find("parent")
    child_elem = joint_elem.find("child")
    parent = parent_elem.get("link", "") if parent_elem is not None else ""
    child = child_elem.get("link", "") if child_elem is not None else ""

    # Parse origin
    origin = parse_origin(joint_elem.find("origin"))

    # Parse axis (only set default for joints that use axis)
    # Parse axis (only relevant for revolute, continuous, prismatic, planar)
    axis_elem = joint_elem.find("axis")
    axis: Vector3 | None = None

    if axis_elem is not None:
        # Explicit axis provided
        axis = parse_vector3(axis_elem.get("xyz", "1 0 0"))
    elif joint_type in (
        JointType.REVOLUTE,
        JointType.CONTINUOUS,
        JointType.PRISMATIC,
        JointType.PLANAR,
    ):
        # Default axis per URDF spec is (1, 0, 0) for these types provided none is specified
        axis = Vector3(1, 0, 0)
    else:
        # FIXED and FLOATING joints must not have an axis
        axis = None

    # Parse limits
    limits = None
    limits_elem = joint_elem.find("limit")
    if limits_elem is not None:
        # Get lower/upper (optional for CONTINUOUS joints)
        lower_str = limits_elem.get("lower")
        upper_str = limits_elem.get("upper")

        # Parse limits with optional lower/upper
        limits = JointLimits(
            lower=float(lower_str) if lower_str is not None else None,
            upper=float(upper_str) if upper_str is not None else None,
            effort=parse_float(limits_elem.get("effort"), "effort", default=0.0),
            velocity=parse_float(limits_elem.get("velocity"), "velocity", default=0.0),
        )

    # Parse dynamics
    dynamics = None
    dynamics_elem = joint_elem.find("dynamics")
    if dynamics_elem is not None:
        dynamics = JointDynamics(
            damping=parse_float(dynamics_elem.get("damping"), "damping", default=0.0),
            friction=parse_float(dynamics_elem.get("friction"), "friction", default=0.0),
        )

    # Parse mimic
    mimic = None
    mimic_elem = joint_elem.find("mimic")
    if mimic_elem is not None:
        mimic = JointMimic(
            joint=mimic_elem.get("joint", ""),
            multiplier=parse_float(mimic_elem.get("multiplier"), "multiplier", default=1.0),
            offset=parse_float(mimic_elem.get("offset"), "offset", default=0.0),
        )

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
    )


def _normalize_hardware_interface(interface: str) -> str:
    """Normalize hardware interface name to short form.

    Accepts both short form ("position") and full ROS2 form
    ("hardware_interface/PositionJointInterface") and returns short form.

    Args:
        interface: Hardware interface string

    Returns:
        Normalized short form (e.g., "position", "velocity", "effort")
    """
    # Mapping from full ROS2 interface names to short form
    full_to_short = {
        "hardware_interface/positionjointinterface": "position",
        "hardware_interface/velocityjointinterface": "velocity",
        "hardware_interface/effortjointinterface": "effort",
    }

    # Try case-insensitive match for full form
    normalized = full_to_short.get(interface.lower())
    if normalized:
        return normalized

    # Already in short form or custom interface - return as-is
    return interface


def parse_transmission(trans_elem: ET.Element) -> Transmission:
    """Parse transmission element.

    Args:
        trans_elem: <transmission> XML element

    Returns:
        Transmission model

    """
    name = trans_elem.get("name", "")
    trans_type = trans_elem.findtext("type", "")

    # Parse joints
    joints: list[TransmissionJoint] = []
    for joint_elem in trans_elem.findall("joint"):
        joint_name = joint_elem.get("name", "")

        # Parse hardware interfaces
        interfaces = []
        for iface_elem in joint_elem.findall("hardwareInterface"):
            raw_interface = iface_elem.text or "position"
            interfaces.append(_normalize_hardware_interface(raw_interface))

        if not interfaces:
            interfaces = ["position"]  # Default

        # Parse mechanical reduction (optional)
        reduction = 1.0
        reduction_elem = joint_elem.find("mechanicalReduction")
        if reduction_elem is not None and reduction_elem.text:
            reduction = float(reduction_elem.text)

        # Parse offset (optional)
        offset = 0.0
        offset_elem = joint_elem.find("offset")
        if offset_elem is not None and offset_elem.text:
            offset = float(offset_elem.text)

        joints.append(
            TransmissionJoint(
                name=joint_name,
                hardware_interfaces=interfaces,
                mechanical_reduction=reduction,
                offset=offset,
            )
        )

    # Parse actuators
    actuators: list[TransmissionActuator] = []
    for actuator_elem in trans_elem.findall("actuator"):
        actuator_name = actuator_elem.get("name", "")

        # Parse hardware interfaces
        interfaces = []
        for iface_elem in actuator_elem.findall("hardwareInterface"):
            raw_interface = iface_elem.text or "position"
            interfaces.append(_normalize_hardware_interface(raw_interface))

        if not interfaces:
            interfaces = ["position"]  # Default

        # Parse mechanical reduction (optional)
        reduction = 1.0
        reduction_elem = actuator_elem.find("mechanicalReduction")
        if reduction_elem is not None and reduction_elem.text:
            reduction = float(reduction_elem.text)

        # Parse offset (optional)
        offset = 0.0
        offset_elem = actuator_elem.find("offset")
        if offset_elem is not None and offset_elem.text:
            offset = float(offset_elem.text)

        actuators.append(
            TransmissionActuator(
                name=actuator_name,
                hardware_interfaces=interfaces,
                mechanical_reduction=reduction,
                offset=offset,
            )
        )

    return Transmission(
        name=name,
        type=trans_type,
        joints=joints,
        actuators=actuators,
    )


def parse_ros2_control(rc_elem: ET.Element) -> Ros2Control:
    """Parse ros2_control element.

    Args:
        rc_elem: <ros2_control> XML element

    Returns:
        Ros2Control model
    """
    name = rc_elem.get("name", "")
    rc_type = rc_elem.get("type", "system")

    # Parse hardware plugin
    hw_elem = rc_elem.find("hardware")
    plugin_elem = hw_elem.find("plugin") if hw_elem is not None else None
    hardware_plugin = (
        plugin_elem.text.strip() if plugin_elem is not None and plugin_elem.text else ""
    )

    # Parse joints
    joints: list[Ros2ControlJoint] = []
    for joint_elem in rc_elem.findall("joint"):
        joint_name = joint_elem.get("name", "")

        # Parse command interfaces
        command_interfaces = []
        for cmd_elem in joint_elem.findall("command_interface"):
            iface_name = cmd_elem.get("name", "position")
            command_interfaces.append(iface_name)

        # Parse state interfaces
        state_interfaces = []
        for state_elem in joint_elem.findall("state_interface"):
            iface_name = state_elem.get("name", "position")
            state_interfaces.append(iface_name)

        # Only add joint if it has at least one command and state interface
        if command_interfaces and state_interfaces:
            joints.append(
                Ros2ControlJoint(
                    name=joint_name,
                    command_interfaces=command_interfaces,
                    state_interfaces=state_interfaces,
                )
            )

    # Parse extra parameters (optional)
    parameters: dict[str, str] = {}
    for child in rc_elem:
        if child.tag not in ("hardware", "joint"):
            if child.text:
                parameters[child.tag] = child.text.strip()

    return Ros2Control(
        name=name,
        type=rc_type,
        hardware_plugin=hardware_plugin,
        joints=joints,
        parameters=parameters,
    )


def parse_sensor_noise(noise_elem: ET.Element | None) -> SensorNoise | None:
    """Parse sensor noise element.

    Args:
        noise_elem: <noise> XML element or element containing noise sub-elements

    Returns:
        SensorNoise model or None

    """
    if noise_elem is None:
        return None

    # Check if we have a <noise> child or if this IS the noise element
    actual_noise_elem = noise_elem.find("noise") if noise_elem.tag != "noise" else noise_elem

    if actual_noise_elem is None:
        return None

    noise_type = actual_noise_elem.findtext("type", "gaussian")
    mean = parse_float(actual_noise_elem.findtext("mean", "0.0"), "noise mean", default=0.0)
    stddev = parse_float(actual_noise_elem.findtext("stddev", "0.0"), "noise stddev", default=0.0)

    return SensorNoise(type=noise_type, mean=mean, stddev=stddev)


def parse_sensor_from_gazebo(gazebo_elem: ET.Element) -> Sensor | None:
    """Parse sensor from Gazebo element.

    Args:
        gazebo_elem: <gazebo reference="link"> element containing <sensor>

    Returns:
        Sensor model or None

    """
    # Get reference link
    link_name = gazebo_elem.get("reference", "")
    if not link_name:
        return None

    # Find sensor element
    sensor_elem = gazebo_elem.find("sensor")
    if sensor_elem is None:
        return None

    sensor_name = sensor_elem.get("name", "")
    sensor_type_str = sensor_elem.get("type", "camera")

    # Map Gazebo sensor type to SensorType enum
    type_map = {
        "camera": SensorType.CAMERA,
        "depth": SensorType.DEPTH_CAMERA,
        "depth_camera": SensorType.DEPTH_CAMERA,  # Standard Gazebo depth camera
        "multicamera": SensorType.CAMERA,
        "ray": SensorType.LIDAR,
        "lidar": SensorType.LIDAR,  # Modern Gazebo uses "lidar" instead of "ray"
        "gpu_ray": SensorType.LIDAR,  # Normalize GPU lidar to internal LIDAR type
        "gpu_lidar": SensorType.LIDAR,  # Normalize GPU lidar to internal LIDAR type
        "imu": SensorType.IMU,
        "gps": SensorType.GPS,
        "navsat": SensorType.GPS,  # Gazebo Sim uses navsat for GPS
        "contact": SensorType.CONTACT,
        "force_torque": SensorType.FORCE_TORQUE,
    }
    sensor_type = type_map.get(sensor_type_str.lower(), SensorType.CAMERA)

    # Parse update rate
    update_rate = parse_float(
        sensor_elem.findtext("update_rate", "30.0"), "update_rate", default=30.0
    )

    # Parse origin from pose element (Gazebo uses <pose> instead of <origin>)
    origin = Transform.identity()
    pose_elem = sensor_elem.find("pose")
    if pose_elem is not None and pose_elem.text:
        parts = pose_elem.text.strip().split()
        if len(parts) >= 6:
            origin = Transform(
                xyz=Vector3(float(parts[0]), float(parts[1]), float(parts[2])),
                rpy=Vector3(float(parts[3]), float(parts[4]), float(parts[5])),
            )

    # Parse sensor-specific info
    camera_info = None
    lidar_info = None
    imu_info = None
    gps_info = None
    contact_info = None
    force_torque_info = None

    # Parse camera
    if sensor_type in (SensorType.CAMERA, SensorType.DEPTH_CAMERA):
        camera_elem = sensor_elem.find("camera")
        if camera_elem is not None:
            horizontal_fov = parse_float(
                camera_elem.findtext("horizontal_fov", "1.047"), "horizontal_fov", default=1.047
            )

            image_elem = camera_elem.find("image")
            width = 640
            height = 480
            format_str = "R8G8B8"
            if image_elem is not None:
                width = parse_int(image_elem.findtext("width", "640"), "image width", default=640)
                height = parse_int(
                    image_elem.findtext("height", "480"), "image height", default=480
                )
                format_str = image_elem.findtext("format", "R8G8B8")

            clip_elem = camera_elem.find("clip")
            near_clip = 0.1
            far_clip = 100.0
            if clip_elem is not None:
                near_clip = parse_float(clip_elem.findtext("near", "0.1"), "near_clip", default=0.1)
                far_clip = parse_float(
                    clip_elem.findtext("far", "100.0"), "far_clip", default=100.0
                )

            # Parse noise
            noise = parse_sensor_noise(camera_elem)

            camera_info = CameraInfo(
                horizontal_fov=horizontal_fov,
                width=width,
                height=height,
                format=format_str,
                near_clip=near_clip,
                far_clip=far_clip,
                noise=noise,
            )
        else:
            # Create default camera info if <camera> element is missing but type is camera
            camera_info = CameraInfo()

    # Parse LIDAR (ray sensor)
    elif sensor_type == SensorType.LIDAR:
        ray_elem = sensor_elem.find("ray")
        if ray_elem is not None:
            scan_elem = ray_elem.find("scan")
            horizontal_elem = scan_elem.find("horizontal") if scan_elem is not None else None

            horizontal_samples = 640
            horizontal_min_angle = -1.570796
            horizontal_max_angle = 1.570796
            vertical_samples = 1

            if horizontal_elem is not None:
                horizontal_samples = parse_int(
                    horizontal_elem.findtext("samples", "640"), "horizontal_samples", default=640
                )
                horizontal_min_angle = parse_float(
                    horizontal_elem.findtext("min_angle", "-1.570796"),
                    "horizontal_min_angle",
                    default=-1.570796,
                )
                horizontal_max_angle = parse_float(
                    horizontal_elem.findtext("max_angle", "1.570796"),
                    "horizontal_max_angle",
                    default=1.570796,
                )

            range_elem = ray_elem.find("range")
            range_min = 0.1
            range_max = 10.0
            if range_elem is not None:
                range_min = parse_float(range_elem.findtext("min", "0.1"), "range_min", default=0.1)
                range_max = parse_float(
                    range_elem.findtext("max", "10.0"), "range_max", default=10.0
                )

            # Parse noise
            noise = parse_sensor_noise(ray_elem)

            lidar_info = LidarInfo(
                horizontal_samples=horizontal_samples,
                horizontal_min_angle=horizontal_min_angle,
                horizontal_max_angle=horizontal_max_angle,
                vertical_samples=vertical_samples,
                range_min=range_min,
                range_max=range_max,
                noise=noise,
            )
        else:
            # Create default LIDAR info if <ray> element is missing but type is LIDAR
            lidar_info = LidarInfo()

    # Parse GPS
    elif sensor_type == SensorType.GPS:
        gps_elem = sensor_elem.find("gps")
        if gps_elem is not None:
            # Helper to parse specific noise path
            def _parse_gps_noise(
                parent: ET.Element, category: str, axis: str
            ) -> SensorNoise | None:
                cat_elem = parent.find(category)
                if cat_elem is not None:
                    axis_elem = cat_elem.find(axis)
                    if axis_elem is not None:
                        return parse_sensor_noise(axis_elem)
                return None

            pos_horiz = _parse_gps_noise(gps_elem, "position_sensing", "horizontal")
            pos_vert = _parse_gps_noise(gps_elem, "position_sensing", "vertical")
            vel_horiz = _parse_gps_noise(gps_elem, "velocity_sensing", "horizontal")
            vel_vert = _parse_gps_noise(gps_elem, "velocity_sensing", "vertical")

            gps_info = GPSInfo(
                position_sensing_horizontal_noise=pos_horiz,
                position_sensing_vertical_noise=pos_vert,
                velocity_sensing_horizontal_noise=vel_horiz,
                velocity_sensing_vertical_noise=vel_vert,
            )
        else:
            # Create default GPS info if <gps> element is missing but type is GPS
            gps_info = GPSInfo()

    # Parse IMU
    elif sensor_type == SensorType.IMU:
        imu_elem = sensor_elem.find("imu")
        if imu_elem is not None:
            # Parse angular velocity noise
            ang_vel_noise = None
            ang_vel_elem = imu_elem.find("angular_velocity")
            if ang_vel_elem is not None:
                ang_vel_noise = parse_sensor_noise(ang_vel_elem.find("x"))

            # Parse linear acceleration noise
            lin_acc_noise = None
            lin_acc_elem = imu_elem.find("linear_acceleration")
            if lin_acc_elem is not None:
                lin_acc_noise = parse_sensor_noise(lin_acc_elem.find("x"))

            imu_info = IMUInfo(
                angular_velocity_noise=ang_vel_noise,
                linear_acceleration_noise=lin_acc_noise,
            )
        else:
            # Create default IMU info if <imu> element is missing but type is IMU
            imu_info = IMUInfo()

    # Parse Contact
    elif sensor_type == SensorType.CONTACT:
        contact_elem = sensor_elem.find("contact")
        if contact_elem is not None:
            # Parse collision (required)
            collision = contact_elem.findtext("collision")
            if not collision:
                raise ValueError(
                    f"Contact sensor '{sensor_name}' missing required <collision> element"
                )

            # Parse noise
            noise = parse_sensor_noise(contact_elem)
            contact_info = ContactInfo(collision=collision, noise=noise)
        else:
            raise ValueError(f"Contact sensor '{sensor_name}' missing required <contact> element")

    # Parse Force/Torque
    elif sensor_type == SensorType.FORCE_TORQUE:
        ft_elem = sensor_elem.find("force_torque")
        frame = "child"
        measure_direction = "child_to_parent"
        noise = None

        if ft_elem is not None:
            frame = ft_elem.findtext("frame", "child")
            measure_direction = ft_elem.findtext("measure_direction", "child_to_parent")
            noise = parse_sensor_noise(ft_elem)

        force_torque_info = ForceTorqueInfo(
            frame=frame,
            measure_direction=measure_direction,
            noise=noise,
        )

    # Parse plugin
    plugin = None
    plugin_elem = sensor_elem.find("plugin")
    if plugin_elem is not None:
        plugin = parse_gazebo_plugin(plugin_elem)

    # Get topic from sensor element or default
    topic = sensor_elem.findtext("topic")
    if not topic:
        topic = f"/{sensor_name}"

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
        plugin=plugin,
        topic=topic,
    )


def parse_gazebo_plugin(plugin_elem: ET.Element) -> GazeboPlugin:
    """Parse Gazebo plugin element.

    Args:
        plugin_elem: <plugin> XML element

    Returns:
        GazeboPlugin model

    """
    name = plugin_elem.get("name", "")
    filename = plugin_elem.get("filename", "")

    # Parse all sub-elements as parameters (for simple cases)
    parameters: dict[str, str] = {}
    for child in plugin_elem:
        if child.text and not len(child):  # Only if no nested children
            parameters[child.tag] = child.text.strip()

    # Store raw XML content for round-trip fidelity (preserves nested elements)
    raw_xml = None
    if len(plugin_elem):  # If plugin has sub-elements
        raw_xml = "".join(ET.tostring(child, encoding="unicode") for child in plugin_elem)

    return GazeboPlugin(name=name, filename=filename, parameters=parameters, raw_xml=raw_xml)


def parse_gazebo_element(gazebo_elem: ET.Element) -> GazeboElement:
    """Parse Gazebo extension element.

    Args:
        gazebo_elem: <gazebo> XML element

    Returns:
        GazeboElement model

    """
    reference = gazebo_elem.get("reference", None)

    # Parse properties as dict
    properties: dict[str, str] = {}

    # Parse plugins
    plugins: list[GazeboPlugin] = []
    for plugin_elem in gazebo_elem.findall("plugin"):
        plugins.append(parse_gazebo_plugin(plugin_elem))

    # Helper functions for parsing optional elements
    def _parse_optional_bool(elem: ET.Element, tag: str, default: str = "false") -> bool | None:
        """Parse optional boolean element."""
        if elem.find(tag) is not None:
            return elem.findtext(tag, default).lower() == "true"
        return None

    def _parse_optional_float(elem: ET.Element, tag: str, default: str = "0") -> float | None:
        """Parse optional float element."""
        if elem.find(tag) is not None:
            text = elem.findtext(tag, default)
            # Use centralized parse_float to handle empty strings
            return parse_float(text, tag, default=float(default) if default else None)
        return None

    # Parse common properties
    material = gazebo_elem.findtext("material")

    # Parse boolean properties
    self_collide = _parse_optional_bool(gazebo_elem, "selfCollide")
    static = _parse_optional_bool(gazebo_elem, "static")
    gravity = _parse_optional_bool(gazebo_elem, "gravity", "true")
    provide_feedback = _parse_optional_bool(gazebo_elem, "provideFeedback")
    implicit_spring_damper = _parse_optional_bool(gazebo_elem, "implicitSpringDamper")

    # Parse numeric properties
    mu1 = _parse_optional_float(gazebo_elem, "mu1")
    mu2 = _parse_optional_float(gazebo_elem, "mu2")
    kp = _parse_optional_float(gazebo_elem, "kp")
    kd = _parse_optional_float(gazebo_elem, "kd")
    stop_cfm = _parse_optional_float(gazebo_elem, "stopCfm")
    stop_erp = _parse_optional_float(gazebo_elem, "stopErp")

    # Store any other elements as properties
    for child in gazebo_elem:
        if child.tag not in [
            "plugin",
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
        ]:
            if child.text:
                properties[child.tag] = child.text.strip()

    return GazeboElement(
        reference=reference,
        properties=properties,
        plugins=plugins,
        material=material,
        self_collide=self_collide,
        static=static,
        gravity=gravity,
        stop_cfm=stop_cfm,
        stop_erp=stop_erp,
        provide_feedback=provide_feedback,
        implicit_spring_damper=implicit_spring_damper,
        mu1=mu1,
        mu2=mu2,
        kp=kp,
        kd=kd,
    )


def _detect_xacro_file(root: ET.Element, filepath: Path) -> None:
    """Detect if file is XACRO and raise helpful error.

    This function is called by parse_urdf() to prevent attempting to parse
    raw XACRO files. XACRO files should be handled by the import operator
    which converts them to URDF first using xacrodoc.

    Args:
        root: XML root element
        filepath: Path to file being parsed

    Raises:
        ValueError: If XACRO features are detected in this parser
            (XACRO files should use the "Import Robot" operator instead)

    """
    # Check for .xacro extension
    is_xacro_extension = filepath.suffix.lower() in [".xacro", ".urdf.xacro"]

    # Check file content for xmlns:xacro (ElementTree may not preserve it)
    has_xacro_namespace = False
    try:
        content = filepath.read_text(encoding="utf-8")
        has_xacro_namespace = "xmlns:xacro" in content
    except (OSError, UnicodeDecodeError):
        # If we can't read the file, assume no XACRO namespace
        # (file may be binary, corrupted, or have permission issues)
        pass

    # Check for xacro elements in root
    xacro_elements = []
    for child in root:
        tag = child.tag
        # Handle both namespaced and non-namespaced tags
        if "xacro:" in tag or (isinstance(tag, str) and tag.startswith("{") and "xacro" in tag):
            element_name = tag.split("}")[-1] if "}" in tag else tag.split("xacro:")[-1]
            xacro_elements.append(element_name)

    # Check for xacro substitutions in attributes
    has_substitutions = False
    for elem in root.iter():
        for attr_value in elem.attrib.values():
            if isinstance(attr_value, str) and ("${" in attr_value or "$(" in attr_value):
                has_substitutions = True
                break
        if has_substitutions:
            break

    # Raise error if XACRO features detected
    if is_xacro_extension or has_xacro_namespace or xacro_elements or has_substitutions:
        error_msg = (
            f"XACRO file detected: {filepath.name}\n\n"
            "This parser handles URDF files only. For XACRO files, use the 'Import Robot' "
            "operator in Blender which automatically converts XACRO to URDF.\n\n"
            "If using this parser programmatically, convert XACRO to URDF first:\n"
            "   from xacrodoc import XacroDoc\n"
            f"   doc = XacroDoc.from_file('{filepath.name}')\n"
            "   robot = parse_urdf_string(doc.to_urdf_string())\n"
        )

        if xacro_elements:
            error_msg += f"\n\nDetected XACRO features: {', '.join(set(xacro_elements))}"

        raise ValueError(error_msg)


def parse_urdf(filepath: Path) -> Robot:
    """Parse URDF file and return Robot model.

    Note: This function parses URDF files only. For XACRO files,
    use parse_urdf_string() with xacrodoc, or use the Blender
    "Import Robot" operator which handles XACRO automatically.

    Args:
        filepath: Path to URDF file

    Returns:
        Robot model

    Raises:
        FileNotFoundError: If URDF file doesn't exist
        ET.ParseError: If XML is malformed
        ValueError: If XACRO file is detected (use "Import Robot" operator instead)

    """
    if not filepath.exists():
        raise FileNotFoundError(f"URDF file not found: {filepath}")

    # Check file size to prevent DoS
    file_size = filepath.stat().st_size
    if file_size > MAX_FILE_SIZE:
        logger.warning(
            f"URDF file '{filepath.name}' rejected: size {file_size / (1024 * 1024):.1f} MB "
            f"exceeds limit of {MAX_FILE_SIZE / (1024 * 1024):.0f} MB"
        )
        raise ValueError(
            f"URDF file too large: {file_size / (1024 * 1024):.1f} MB "
            f"(maximum {MAX_FILE_SIZE / (1024 * 1024):.0f} MB)"
        )

    logger.info(f"Parsing URDF file: {filepath.name} ({file_size / 1024:.1f} KB)")

    # Parse XML with default parser
    # Note: Python 3.8+ has XXE protection enabled by default in xml.etree.ElementTree
    # External entity expansion is disabled by default since Python 3.7.1
    tree = ET.parse(filepath)
    root = tree.getroot()

    # Validate XML depth to prevent billion laughs attack
    _validate_xml_depth(root, 0)

    if root.tag != "robot":
        raise ValueError("Root element must be <robot>")

    # Detect XACRO files and provide helpful error
    _detect_xacro_file(root, filepath)

    robot_name = root.get("name", "imported_robot")
    robot = Robot(name=robot_name)

    # Parse global materials first
    materials: dict[str, Material] = {}
    for mat_elem in root.findall("material"):
        mat = parse_material(mat_elem, materials)
        if mat:
            materials[mat.name] = mat

    # Get URDF directory for mesh path validation
    urdf_directory = filepath.parent

    # Parse all links
    for link_elem in root.findall("link"):
        link = parse_link(link_elem, materials, urdf_directory)
        try:
            robot.add_link(link)
        except ValueError:
            # Handle duplicate link name by renaming
            original_name = link.name
            counter = 1
            while True:
                new_name = f"{original_name}_duplicate_{counter}"
                if new_name not in robot._link_index:
                    link = replace(link, name=new_name)
                    try:
                        robot.add_link(link)
                        logger.warning(f"Renamed duplicate link '{original_name}' to '{new_name}'")
                        break
                    except ValueError:
                        counter += 1  # Should not happen given check above, but safe
                else:
                    counter += 1

    # Parse all joints
    for joint_elem in root.findall("joint"):
        joint = parse_joint(joint_elem)
        try:
            robot.add_joint(joint)
        except ValueError as e:
            if "already exists" in str(e):
                # Handle duplicate joint name by renaming
                original_name = joint.name
                counter = 1
                while True:
                    new_name = f"{original_name}_duplicate_{counter}"
                    if new_name not in robot._joint_index:
                        joint = replace(joint, name=new_name)
                        try:
                            robot.add_joint(joint)
                            logger.warning(
                                f"Renamed duplicate joint '{original_name}' to '{new_name}'"
                            )
                            break
                        except ValueError as inner_e:
                            if "not found" in str(inner_e):
                                # Also handle missing parent/child during rename attempt
                                logger.warning(
                                    f"Skipping duplicate joint '{original_name}' (renamed '{new_name}') "
                                    f"due to broken reference: {inner_e}"
                                )
                                break
                            counter += 1
                    else:
                        counter += 1
            else:
                # Handle other ValueErrors (missing parent/child links)
                logger.warning(f"Skipping invalid joint '{joint.name}': {e}")

    # Parse all transmissions
    for trans_elem in root.findall("transmission"):
        transmission = parse_transmission(trans_elem)
        robot.transmissions.append(transmission)  # Direct append, validation done by Robot model

    # Parse ros2_control blocks
    for rc_elem in root.findall("ros2_control"):
        ros2_control = parse_ros2_control(rc_elem)
        robot.ros2_controls.append(ros2_control)

    # Parse all Gazebo elements (including sensors)
    for gazebo_elem in root.findall("gazebo"):
        # Try to parse as sensor first
        sensor = parse_sensor_from_gazebo(gazebo_elem)
        if sensor:
            robot.sensors.append(sensor)
        else:
            # Parse as regular Gazebo element
            gazebo_element = parse_gazebo_element(gazebo_elem)
            robot.gazebo_elements.append(gazebo_element)

    logger.info(
        f"Successfully parsed robot '{robot.name}': {len(robot.links)} links, "
        f"{len(robot.joints)} joints, {len(robot.sensors)} sensors"
    )

    return robot


def parse_urdf_string(urdf_string: str, urdf_directory: Path | None = None) -> Robot:
    """Parse URDF from XML string instead of file.

    This function is used for parsing URDF content that has been generated
    or converted from other formats (e.g., XACRO) in memory.

    Args:
        urdf_string: URDF XML content as string
        urdf_directory: Optional directory for mesh path validation.
                       If provided, mesh paths will be validated for security.
                       If None, mesh path validation is skipped (less secure).

    Returns:
        Robot model

    Raises:
        ET.ParseError: If XML is malformed
        ValueError: If root element is not <robot> or URDF is invalid

    Example:
        >>> urdf_xml = '<robot name="test">...</robot>'
        >>> robot = parse_urdf_string(urdf_xml)
        >>> print(robot.name)
        test

        >>> # With mesh path validation (recommended for XACRO imports)
        >>> from pathlib import Path
        >>> robot = parse_urdf_string(urdf_xml, urdf_directory=Path("/path/to/robot"))

    Note:
        For XACRO-converted URDFs, it's recommended to pass urdf_directory
        to enable mesh path validation and prevent path traversal attacks.

    """
    # Check string size to prevent DoS
    string_size = len(urdf_string.encode("utf-8"))
    if string_size > MAX_FILE_SIZE:
        raise ValueError(
            f"URDF string too large: {string_size / (1024 * 1024):.1f} MB "
            f"(maximum {MAX_FILE_SIZE / (1024 * 1024):.0f} MB)"
        )

    # Parse XML string with default parser
    # Note: Python 3.8+ has XXE protection enabled by default in xml.etree.ElementTree
    # External entity expansion is disabled by default since Python 3.7.1
    root = ET.fromstring(urdf_string)

    # Validate XML depth to prevent billion laughs attack
    _validate_xml_depth(root, 0)

    if root.tag != "robot":
        raise ValueError("Root element must be <robot>")

    robot_name = root.get("name", "imported_robot")
    robot = Robot(name=robot_name)

    # Parse global materials first
    materials: dict[str, Material] = {}
    for mat_elem in root.findall("material"):
        mat = parse_material(mat_elem, materials)
        if mat:
            materials[mat.name] = mat

    # Parse all links with optional directory for mesh path validation
    for link_elem in root.findall("link"):
        link = parse_link(link_elem, materials, urdf_directory=urdf_directory)
        try:
            robot.add_link(link)
        except ValueError:
            # Handle duplicate link name by renaming
            original_name = link.name
            counter = 1
            while True:
                new_name = f"{original_name}_duplicate_{counter}"
                if new_name not in robot._link_index:
                    link = replace(link, name=new_name)
                    try:
                        robot.add_link(link)
                        logger.warning(f"Renamed duplicate link '{original_name}' to '{new_name}'")
                        break
                    except ValueError:
                        counter += 1
                else:
                    counter += 1

    # Parse all joints
    for joint_elem in root.findall("joint"):
        joint = parse_joint(joint_elem)
        try:
            robot.add_joint(joint)
        except ValueError as e:
            if "already exists" in str(e):
                # Handle duplicate joint name by renaming
                original_name = joint.name
                counter = 1
                while True:
                    new_name = f"{original_name}_duplicate_{counter}"
                    if new_name not in robot._joint_index:
                        joint = replace(joint, name=new_name)
                        try:
                            robot.add_joint(joint)
                            logger.warning(
                                f"Renamed duplicate joint '{original_name}' to '{new_name}'"
                            )
                            break
                        except ValueError as inner_e:
                            if "not found" in str(inner_e):
                                # Also handle missing parent/child during rename attempt
                                logger.warning(
                                    f"Skipping duplicate joint '{original_name}' (renamed '{new_name}') "
                                    f"due to broken reference: {inner_e}"
                                )
                                break
                            counter += 1
                    else:
                        counter += 1
            else:
                # Handle other ValueErrors (missing parent/child links)
                logger.warning(f"Skipping invalid joint '{joint.name}': {e}")

    # Parse all transmissions
    for trans_elem in root.findall("transmission"):
        transmission = parse_transmission(trans_elem)
        robot.transmissions.append(transmission)

    # Parse ros2_control blocks
    for rc_elem in root.findall("ros2_control"):
        ros2_control = parse_ros2_control(rc_elem)
        robot.ros2_controls.append(ros2_control)

    # Parse all Gazebo elements (including sensors)
    for gazebo_elem in root.findall("gazebo"):
        # Try to parse as sensor first
        sensor = parse_sensor_from_gazebo(gazebo_elem)
        if sensor:
            robot.sensors.append(sensor)
        else:
            # Parse as regular Gazebo element
            gazebo_element = parse_gazebo_element(gazebo_elem)
            robot.gazebo_elements.append(gazebo_element)

    return robot
