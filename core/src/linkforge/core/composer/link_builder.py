"""Link physics and kinematics builder for LinkForge Composer.

Provides a staged fluent interface for constructing robot links, including
visuals, collisions, mass properties, and their parent joint connections.

Core Components:
    - LinkBuilder: Staged builder for programmatic link/joint construction.
    - _JointState: Internal container for staged joint properties.
    - _LinkState: Internal container for staged link properties.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any

from .._utils.math_utils import normalize_vector
from ..constants import (
    DEFAULT_CAMERA_FOV,
    DEFAULT_CAMERA_HEIGHT,
    DEFAULT_CAMERA_WIDTH,
    DEFAULT_JOINT_DAMPING,
    DEFAULT_JOINT_EFFORT,
    DEFAULT_JOINT_FRICTION,
    DEFAULT_JOINT_VELOCITY,
    DEFAULT_LIDAR_RANGE_MAX,
    DEFAULT_LIDAR_RANGE_MIN,
    DEFAULT_LIDAR_SAMPLES,
    DEFAULT_UPDATE_RATE_CONTACT,
    DEFAULT_UPDATE_RATE_FORCE_TORQUE,
    DEFAULT_UPDATE_RATE_GPS,
    DEFAULT_UPDATE_RATE_IMU,
    HW_IF_EFFORT,
)
from ..exceptions import RobotValidationError, ValidationErrorCode
from ..models.gazebo import GazeboElement
from ..models.geometry import Geometry, Transform, Vector3
from ..models.joint import (
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafetyController,
    JointType,
)
from ..models.link import Collision, Inertial, InertiaTensor, Link, LinkPhysics, Visual
from ..models.material import Material
from ..models.robot import Robot
from ..models.ros2_control import Ros2ControlJoint
from ..models.sensor import (
    CameraInfo,
    ContactInfo,
    ForceTorqueInfo,
    GPSInfo,
    IMUInfo,
    LidarInfo,
    Sensor,
    SensorType,
)
from ..models.transmission import Transmission
from ..physics.inertia import calculate_inertia

if TYPE_CHECKING:
    from .interfaces import IComposer

logger = logging.getLogger(__name__)


@dataclass
class _JointState:
    """Internal container for staged joint properties."""

    type: JointType = JointType.FIXED
    origin: Transform = field(default_factory=Transform.identity)
    axis: Vector3 | None = None
    limits: JointLimits | None = None
    dynamics: JointDynamics | None = None
    mimic: JointMimic | None = None
    safety: JointSafetyController | None = None
    calibration: JointCalibration | None = None


@dataclass
class _LinkState:
    """Internal container for staged link properties."""

    mass: float | None = None
    inertia: InertiaTensor | None = None
    inertial_origin: Transform | None = None
    visuals: list[Visual] = field(default_factory=list)
    collisions: list[Collision] = field(default_factory=list)
    sensors: list[Sensor] = field(default_factory=list)
    physics: LinkPhysics = field(default_factory=LinkPhysics)
    gazebo_params: dict[str, Any] = field(default_factory=dict)


class LinkBuilder:
    """Staged fluent builder for programmatic link and joint construction.

    This builder accumulates link and joint properties in stages. It is usually
    returned by builder.link() or link_builder.child().
    """

    def __init__(
        self,
        builder: IComposer,
        name: str,
        parent: str | None = None,
        joint_name: str | None = None,
    ) -> None:
        """Initialize a new LinkBuilder. Internal use only."""
        self._builder = builder
        self._link_name = name
        self._parent = parent
        self._joint_name = joint_name

        # Staged state containers
        self._joint = _JointState()
        self._link = _LinkState()

        self._transmission_params: dict[str, Any] | None = None
        self._control_interfaces: tuple[list[str], list[str], dict[str, Any]] | None = None
        self._control_system_name: str | None = None
        self._committed = False
        self._in_context = False

        self._builder._active_link_builders.append(self)

    def __enter__(self) -> LinkBuilder:
        """Enter the context of this link.

        Pushes this link's name onto the parent stack, making it the default
        parent for any links created within this block.
        """
        self._in_context = True
        # Create a skeletal Link so that child links/joints created inside the block
        # can refer to this link as a valid parent/child in the robot's indices.
        skeletal_link = Link(name=self._link_name)
        self._builder.robot.add_link(skeletal_link, overwrite=True)

        self._builder._parent_stack.append(self._link_name)
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: Any,
    ) -> None:
        """Exit the context of this link.

        Pops this link's name from the parent stack and automatically commits the
        link to flush its configured properties if no exception occurred.
        """
        if self._builder._parent_stack and self._builder._parent_stack[-1] == self._link_name:
            self._builder._parent_stack.pop()
        elif self._link_name in self._builder._parent_stack:
            self._builder._parent_stack.remove(self._link_name)

        if exc_type is None:
            self._commit()

    def _check_not_committed(self) -> None:
        """Helper to ensure the builder hasn't been committed yet."""
        if self._committed:
            raise RuntimeError(f"LinkBuilder '{self._link_name}' already committed")  # noqa: TRY003

    def visual(
        self,
        geometry: Geometry,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
        material: str | Material | None = None,
        name: str | None = None,
    ) -> LinkBuilder:
        """Add a visual representation to the link.

        Args:
            geometry: Shape of the visual (e.g., box(), cylinder()).
            xyz: Translation relative to the link frame.
            rpy: Rotation (roll-pitch-yaw) in radians.
            material: Material name or Material object.
            name: Optional name for this visual element.

        Returns:
            The LinkBuilder instance for chaining.
        """
        self._check_not_committed()
        origin = Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy))

        if isinstance(material, str):
            # Resolve global material
            mat = self._builder.robot.materials.get(material)
            if mat is None:
                raise RobotValidationError(
                    ValidationErrorCode.NOT_FOUND,
                    f"Material '{material}' not found. Did you call builder.material('{material}', ...) first?",
                    target="LinkBuilder",
                )
        else:
            mat = material

        self._link.visuals.append(Visual(geometry=geometry, origin=origin, material=mat, name=name))
        return self

    def collision(
        self,
        geometry: Geometry | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
        name: str | None = None,
    ) -> LinkBuilder:
        """Add a collision geometry to the link.

        If no arguments are provided, it automatically clones the last added
        visual element's geometry and origin.

        Args:
            geometry: Shape of the collision element.
            xyz: Translation relative to the link frame.
            rpy: Rotation relative to the link frame.
            name: Optional name for this collision element.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        if geometry is None:
            if not self._link.visuals:
                raise RobotValidationError(
                    ValidationErrorCode.GENERIC_FAILURE,
                    "Cannot infer collision geometry: no visuals defined",
                    target="LinkBuilder",
                    value=self._link_name,
                )
            last_visual = self._link.visuals[-1]
            geometry = last_visual.geometry
            if xyz is not None or rpy is not None:
                origin = Transform(
                    xyz=Vector3(*(xyz or (0, 0, 0))),
                    rpy=Vector3(*(rpy or (0, 0, 0))),
                )
            else:
                origin = last_visual.origin
        else:
            origin = Transform(
                xyz=Vector3(*(xyz or (0, 0, 0))),
                rpy=Vector3(*(rpy or (0, 0, 0))),
            )

        self._link.collisions.append(Collision(geometry=geometry, origin=origin, name=name))
        return self

    def mass(
        self,
        value: float,
        origin_xyz: tuple[float, float, float] | None = None,
        origin_rpy: tuple[float, float, float] | None = None,
        inertia: InertiaTensor | None = None,
    ) -> LinkBuilder:
        """Define the mass and center of gravity for the link.

        If no inertia is provided, LinkForge will automatically calculate the
        inertia tensor based on the link's geometry and mass during commit().

        Args:
            value: Mass in kilograms.
            origin_xyz: Position of the center of mass.
            origin_rpy: Orientation of the principal axes of inertia.
            inertia: Optional manual InertiaTensor.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._link.mass = value
        if inertia:
            self._link.inertia = inertia
        if origin_xyz or origin_rpy:
            self._link.inertial_origin = Transform(
                xyz=Vector3(*(origin_xyz or (0, 0, 0))),
                rpy=Vector3(*(origin_rpy or (0, 0, 0))),
            )
        return self

    def inertia(
        self, ixx: float, iyy: float, izz: float, ixy: float = 0, ixz: float = 0, iyz: float = 0
    ) -> LinkBuilder:
        """Manually specify the inertia tensor components.

        Args:
            ixx, iyy, izz: Moments of inertia.
            ixy, ixz, iyz: Products of inertia.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._link.inertia = InertiaTensor(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz)
        return self

    def at_origin(
        self,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
    ) -> LinkBuilder:
        """Set the transform from the parent link to this link's frame.

        Args:
            xyz: Translation as (x, y, z).
            rpy: Rotation as (roll, pitch, yaw).

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._joint.origin = Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy))
        return self

    def fixed(
        self,
        name: str | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
    ) -> LinkBuilder:
        """Configure the connection as a FIXED joint.

        Args:
            name: Unique joint name.
            xyz: Joint origin translation.
            rpy: Joint origin rotation.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._joint.type = JointType.FIXED
        return self._configure_joint(name, xyz, rpy)

    def revolute(
        self,
        axis: tuple[float, float, float],
        limits: tuple[float, float],
        name: str | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
        effort: float = DEFAULT_JOINT_EFFORT,
        velocity: float = DEFAULT_JOINT_VELOCITY,
    ) -> LinkBuilder:
        """Configure the connection as a REVOLUTE (limited rotation) joint.

        Args:
            axis: Rotation axis unit vector.
            limits: (lower, upper) joint limits in radians.
            name: Unique joint name.
            xyz: Joint origin translation.
            rpy: Joint origin rotation.
            effort: Maximum joint effort.
            velocity: Maximum joint velocity.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._joint.type = JointType.REVOLUTE
        self._joint.axis = self._normalize_axis(axis)
        self._joint.limits = JointLimits(
            lower=limits[0], upper=limits[1], effort=effort, velocity=velocity
        )
        return self._configure_joint(name, xyz, rpy)

    def continuous(
        self,
        axis: tuple[float, float, float],
        name: str | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
        effort: float = DEFAULT_JOINT_EFFORT,
        velocity: float = DEFAULT_JOINT_VELOCITY,
    ) -> LinkBuilder:
        """Configure the connection as a CONTINUOUS (unlimited rotation) joint.

        Args:
            axis: Rotation axis unit vector.
            name: Unique joint name.
            xyz: Joint origin translation.
            rpy: Joint origin rotation.
            effort: Maximum joint effort (optional).
            velocity: Maximum joint velocity (optional).

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._joint.type = JointType.CONTINUOUS
        self._joint.axis = self._normalize_axis(axis)
        if effort is not None or velocity is not None:
            self._joint.limits = JointLimits(
                lower=0.0, upper=0.0, effort=effort or 0.0, velocity=velocity or 0.0
            )
        return self._configure_joint(name, xyz, rpy)

    def prismatic(
        self,
        axis: tuple[float, float, float],
        limits: tuple[float, float],
        name: str | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
        effort: float = DEFAULT_JOINT_EFFORT,
        velocity: float = DEFAULT_JOINT_VELOCITY,
    ) -> LinkBuilder:
        """Configure the connection as a PRISMATIC (linear sliding) joint.

        Args:
            axis: Translation axis unit vector.
            limits: (lower, upper) joint limits in meters.
            name: Unique joint name.
            xyz: Joint origin translation.
            rpy: Joint origin rotation.
            effort: Maximum joint effort.
            velocity: Maximum joint velocity.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._joint.type = JointType.PRISMATIC
        self._joint.axis = self._normalize_axis(axis)
        self._joint.limits = JointLimits(
            lower=limits[0], upper=limits[1], effort=effort, velocity=velocity
        )
        return self._configure_joint(name, xyz, rpy)

    def floating(
        self,
        name: str | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
    ) -> LinkBuilder:
        """Configure the connection as a FLOATING (6 DOF) joint.

        Args:
            name: Unique joint name.
            xyz: Joint origin translation.
            rpy: Joint origin rotation.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._joint.type = JointType.FLOATING
        return self._configure_joint(name, xyz, rpy)

    def planar(
        self,
        axis: tuple[float, float, float],
        name: str | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
    ) -> LinkBuilder:
        """Configure the connection as a PLANAR joint.

        Args:
            axis: Plane normal unit vector.
            name: Unique joint name.
            xyz: Joint origin translation.
            rpy: Joint origin rotation.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._joint.type = JointType.PLANAR
        self._joint.axis = self._normalize_axis(axis)
        return self._configure_joint(name, xyz, rpy)

    def dynamics(
        self, damping: float = DEFAULT_JOINT_DAMPING, friction: float = DEFAULT_JOINT_FRICTION
    ) -> LinkBuilder:
        """Set the physical dynamics for the joint.

        Args:
            damping: Damping coefficient.
            friction: Friction coefficient.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._joint.dynamics = JointDynamics(damping=damping, friction=friction)
        return self

    def mimic(self, joint: str, multiplier: float = 1.0, offset: float = 0.0) -> LinkBuilder:
        """Set this joint to mimic another joint's movement.

        Args:
            joint: Name of the joint to mimic.
            multiplier: Scaling factor for the movement.
            offset: Offset in radians/meters.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._joint.mimic = JointMimic(joint=joint, multiplier=multiplier, offset=offset)
        return self

    def safety(
        self,
        soft_lower: float | None = None,
        soft_upper: float | None = None,
        k_position: float | None = None,
        k_velocity: float = 0.0,
    ) -> LinkBuilder:
        """Define a safety controller for the joint.

        Args:
            soft_lower, soft_upper: Software limits.
            k_position, k_velocity: Controller gains.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._joint.safety = JointSafetyController(
            soft_lower_limit=soft_lower,
            soft_upper_limit=soft_upper,
            k_position=k_position,
            k_velocity=k_velocity,
        )
        return self

    def calibration(self, rising: float | None = None, falling: float | None = None) -> LinkBuilder:
        """Set calibration offsets for the joint.

        Args:
            rising: Rising edge offset.
            falling: Falling edge offset.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._joint.calibration = JointCalibration(rising=rising, falling=falling)
        return self

    def physics(self, **kwargs: Any) -> LinkBuilder:
        """Set surface and contact physics properties for this link.

        Supports both typed LinkPhysics fields and raw engine-specific parameters.

        Common arguments:
            self_collide (bool): Enable self-collision.
            gravity (bool): Enable gravity.
            mu, mu2 (float): Friction coefficients.
            kp, kd (float): Contact stiffness and damping.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()

        phys_fields = {f.name for f in LinkPhysics.__dataclass_fields__.values()}
        phys_updates = {k: v for k, v in kwargs.items() if k in phys_fields}

        if phys_updates:
            self._link.physics = replace(self._link.physics, **phys_updates)

        # Store non-physics fields in gazebo_params
        remaining_kwargs = {k: v for k, v in kwargs.items() if k not in phys_fields}
        if remaining_kwargs:
            self._link.gazebo_params.update(remaining_kwargs)

        return self

    def _configure_joint(
        self,
        name: str | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
    ) -> LinkBuilder:
        """Helper to set common joint properties."""
        if name:
            self._joint_name = name
        if xyz or rpy:
            self._joint.origin = Transform(
                xyz=Vector3(*(xyz or (0, 0, 0))), rpy=Vector3(*(rpy or (0, 0, 0)))
            )
        return self

    def _normalize_axis(self, axis: tuple[float, float, float]) -> Vector3:
        """Validate and normalize a joint axis vector.

        Args:
            axis: The raw (x, y, z) axis.

        Returns:
            A normalized Vector3.

        Raises:
            RobotValidationError: If the axis magnitude is too small.
        """
        nx, ny, nz = normalize_vector(*axis)
        if nx == 0.0 and ny == 0.0 and nz == 0.0:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Joint axis magnitude is too small",
                target="LinkBuilder",
                value=0.0,
            )
        return Vector3(nx, ny, nz)

    def transmission(
        self,
        reduction: float = 1.0,
        interface: str = HW_IF_EFFORT,
        actuator: str | None = None,
        name: str | None = None,
    ) -> LinkBuilder:
        """Define a transmission (mechanical reduction) for the current joint.

        Args:
            reduction: Mechanical reduction ratio.
            interface: Hardware interface (effort, position, velocity).
            actuator: Optional name for the actuator.
            name: Optional transmission name.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._transmission_params = {
            "reduction": reduction,
            "interface": interface,
            "actuator": actuator or f"actuator_{self._link_name}",
            "name": name,
        }
        return self

    def ros2_control(
        self,
        command_interfaces: list[str],
        state_interfaces: list[str],
        parameters: dict[str, Any] | None = None,
        system_name: str | None = None,
    ) -> LinkBuilder:
        """Configure ros2_control interfaces for the current joint.

        Args:
            command_interfaces: List of allowed commands (e.g. ['position']).
            state_interfaces: List of exposed states (e.g. ['position', 'velocity']).
            parameters: Key-value parameters for the joint control.
            system_name: Optional name of the specific ros2_control system to attach to.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        params = {k: str(v) for k, v in (parameters or {}).items()}
        self._control_interfaces = (command_interfaces, state_interfaces, params)
        self._control_system_name = system_name
        return self

    def camera(
        self,
        name: str,
        fov: float = DEFAULT_CAMERA_FOV,
        width: int = DEFAULT_CAMERA_WIDTH,
        height: int = DEFAULT_CAMERA_HEIGHT,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
    ) -> LinkBuilder:
        """Attach a camera sensor to this link.

        Args:
            name: Unique sensor name.
            fov: Horizontal field of view in radians.
            width, height: Resolution in pixels.
            xyz, rpy: Position/Orientation relative to link frame.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        info = CameraInfo(horizontal_fov=fov, width=width, height=height)
        sensor = Sensor(
            name=name,
            type=SensorType.CAMERA,
            link_name=self._link_name,
            camera_info=info,
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        self._link.sensors.append(sensor)
        return self

    def lidar(
        self,
        name: str,
        range_min: float = DEFAULT_LIDAR_RANGE_MIN,
        range_max: float = DEFAULT_LIDAR_RANGE_MAX,
        samples: int = DEFAULT_LIDAR_SAMPLES,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
    ) -> LinkBuilder:
        """Attach a 1D/2D lidar sensor to this link.

        Args:
            name: Unique sensor name.
            range_min, range_max: Distance limits in meters.
            samples: Number of rays per scan.
            xyz, rpy: Position/Orientation relative to link frame.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        info = LidarInfo(range_min=range_min, range_max=range_max, horizontal_samples=samples)
        sensor = Sensor(
            name=name,
            type=SensorType.LIDAR,
            link_name=self._link_name,
            lidar_info=info,
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        self._link.sensors.append(sensor)
        return self

    def gpu_lidar(
        self,
        name: str,
        range_min: float = DEFAULT_LIDAR_RANGE_MIN,
        range_max: float = DEFAULT_LIDAR_RANGE_MAX,
        samples: int = DEFAULT_LIDAR_SAMPLES,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
    ) -> LinkBuilder:
        """Attach a high-performance GPU-accelerated lidar sensor to this link.

        Args:
            name: Unique sensor name.
            range_min, range_max: Distance limits in meters.
            samples: Number of rays per scan.
            xyz, rpy: Position/Orientation relative to link frame.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        info = LidarInfo(range_min=range_min, range_max=range_max, horizontal_samples=samples)
        sensor = Sensor(
            name=name,
            type=SensorType.GPU_LIDAR,
            link_name=self._link_name,
            lidar_info=info,
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        self._link.sensors.append(sensor)
        return self

    def imu(
        self,
        name: str,
        update_rate: float = DEFAULT_UPDATE_RATE_IMU,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
    ) -> LinkBuilder:
        """Attach an IMU (Inertial Measurement Unit) to this link.

        Args:
            name: Unique sensor name.
            update_rate: Sampling rate in Hz.
            xyz, rpy: Position/Orientation relative to link frame.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        sensor = Sensor(
            name=name,
            type=SensorType.IMU,
            link_name=self._link_name,
            update_rate=update_rate,
            imu_info=IMUInfo(),
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        self._link.sensors.append(sensor)
        return self

    def gps(
        self,
        name: str,
        update_rate: float = DEFAULT_UPDATE_RATE_GPS,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
    ) -> LinkBuilder:
        """Attach a GPS sensor to this link.

        Args:
            name: Unique sensor name.
            update_rate: Sampling rate in Hz.
            xyz, rpy: Position/Orientation relative to link frame.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        sensor = Sensor(
            name=name,
            type=SensorType.GPS,
            link_name=self._link_name,
            update_rate=update_rate,
            gps_info=GPSInfo(),
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        self._link.sensors.append(sensor)
        return self

    def force_torque(
        self,
        name: str,
        update_rate: float = DEFAULT_UPDATE_RATE_FORCE_TORQUE,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
    ) -> LinkBuilder:
        """Attach a force-torque sensor to this link.

        Args:
            name: Unique sensor name.
            update_rate: Sampling rate in Hz.
            xyz, rpy: Position/Orientation relative to link frame.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()

        sensor = Sensor(
            name=name,
            type=SensorType.FORCE_TORQUE,
            link_name=self._link_name,
            update_rate=update_rate,
            force_torque_info=ForceTorqueInfo(),
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        self._link.sensors.append(sensor)
        return self

    def contact(
        self,
        name: str,
        collision: str,
        update_rate: float = DEFAULT_UPDATE_RATE_CONTACT,
    ) -> LinkBuilder:
        """Attach a contact sensor to this link.

        Args:
            name: Unique sensor name.
            collision: The name of the collision element to monitor.
            update_rate: Sampling rate in Hz.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        sensor = Sensor(
            name=name,
            type=SensorType.CONTACT,
            link_name=self._link_name,
            update_rate=update_rate,
            contact_info=ContactInfo(collision=collision),
        )
        self._link.sensors.append(sensor)
        return self

    def sensor(self, sensor: Sensor) -> LinkBuilder:
        """Attach a pre-configured Sensor object to this link.

        Args:
            sensor: Pre-configured Sensor model.

        Returns:
            The LinkBuilder instance.
        """
        self._check_not_committed()
        self._link.sensors.append(replace(sensor, link_name=self._link_name))
        return self

    def child(self, name: str, joint_name: str | None = None) -> LinkBuilder:
        """Finalize this link and start building a new child link attached to it.

        Args:
            name: Name of the new child link.
            joint_name: Optional explicit name for the connecting joint.

        Returns:
            A new LinkBuilder instance for the child link.
        """
        self._commit()
        return LinkBuilder(self._builder, name, parent=self._link_name, joint_name=joint_name)

    def commit(self) -> IComposer:
        """Finalize this link and return to the main RobotBuilder.

        Returns:
            The parent RobotBuilder instance.
        """
        self._commit()
        return self._builder

    def root(self) -> IComposer:
        """Finalize this link as the robot's root link (no joint).

        Raises:
            RobotValidationError: If the link already has a parent assigned.

        Returns:
            The parent RobotBuilder instance.
        """
        if self._parent:
            raise RobotValidationError(
                ValidationErrorCode.GENERIC_FAILURE,
                f"Link '{self._link_name}' has a parent '{self._parent}' and cannot be root",
                target="LinkBuilder",
            )
        return self.commit()

    def build(self) -> Robot:
        """Finalize this link and return the completed Robot model.

        Returns:
            The completed Robot model.
        """
        self._commit()
        return self._builder.build()

    def _commit(self) -> None:
        """Internal method to flush staged properties to the Robot model."""
        if self._committed:
            return

        inertial = self._finalize_inertial()
        self._finalize_link(inertial)

        if self._parent:
            joint = self._finalize_joint()
            self._finalize_transmission(joint)
            self._finalize_ros2_control(joint)

        self._committed = True

    def _finalize_inertial(self) -> Inertial | None:
        """Calculate and return the final Inertial properties for the link."""
        l_state = self._link
        if l_state.mass is None:
            return None

        if l_state.inertia is None:
            source_geometry = None
            source_origin = Transform.identity()

            if l_state.collisions:
                if len(l_state.collisions) > 1:
                    logger.warning(
                        "Auto-calculating inertia for link '%s' with multiple collisions. Only the first geometry is used.",
                        self._link_name,
                    )
                source_geometry = l_state.collisions[0].geometry
                source_origin = l_state.collisions[0].origin
            elif l_state.visuals:
                if len(l_state.visuals) > 1:
                    logger.warning(
                        "Auto-calculating inertia for link '%s' with multiple visuals. Only the first geometry is used.",
                        self._link_name,
                    )
                source_geometry = l_state.visuals[0].geometry
                source_origin = l_state.visuals[0].origin

            if source_geometry:
                l_state.inertia = calculate_inertia(source_geometry, l_state.mass)
                if l_state.inertial_origin is None:
                    l_state.inertial_origin = source_origin
            else:
                l_state.inertia = InertiaTensor.zero()

        return Inertial(
            mass=l_state.mass,
            inertia=l_state.inertia,
            origin=l_state.inertial_origin or Transform.identity(),
        )

    def _finalize_link(self, inertial: Inertial | None) -> None:
        """Create and add the final Link model to the robot."""
        l_state = self._link
        # Check for duplicates, unless it was a skeletal link registered in context
        if self._builder.robot.has_link(self._link_name) and not self._in_context:
            raise RobotValidationError(
                ValidationErrorCode.DUPLICATE_NAME,
                f"Duplicate: Link '{self._link_name}'",
                target="Link",
                value=self._link_name,
            )

        link = Link(
            name=self._link_name,
            visuals=l_state.visuals,
            collisions=l_state.collisions,
            inertial=inertial,
            physics=l_state.physics,
        )
        self._builder.robot.add_link(link, overwrite=True)

        for sensor in l_state.sensors:
            self._builder.robot.add_sensor(sensor)

        if l_state.gazebo_params:
            gz = GazeboElement(reference=self._link_name, **l_state.gazebo_params)
            self._builder.robot.add_gazebo_element(gz)

    def _finalize_joint(self) -> Joint:
        """Create and add the final Joint model to the robot."""
        j_state = self._joint
        joint_name = self._joint_name or f"{self._parent}_to_{self._link_name}"
        is_fixed = j_state.type == JointType.FIXED

        assert self._parent is not None
        joint = Joint(
            name=joint_name,
            type=j_state.type,
            parent=self._parent,
            child=self._link_name,
            origin=j_state.origin,
            axis=j_state.axis if not is_fixed else None,
            limits=j_state.limits if not is_fixed else None,
            dynamics=j_state.dynamics if not is_fixed else None,
            mimic=j_state.mimic if not is_fixed else None,
            safety_controller=j_state.safety if not is_fixed else None,
            calibration=j_state.calibration if not is_fixed else None,
        )
        self._builder.robot.add_joint(joint)
        return joint

    def _finalize_transmission(self, joint: Joint) -> None:
        """Create and add the final Transmission model to the robot."""
        if not self._transmission_params:
            return
        t_name = self._transmission_params["name"] or f"trans_{joint.name}"
        trans = Transmission.create_simple(
            name=t_name,
            joint_name=joint.name,
            actuator_name=self._transmission_params["actuator"],
            mechanical_reduction=self._transmission_params["reduction"],
            hardware_interface=self._transmission_params["interface"],
        )
        self._builder.robot.add_transmission(trans)

    def _finalize_ros2_control(self, joint: Joint) -> None:
        """Configure and add ros2_control interfaces to the robot."""
        if not self._control_interfaces:
            return

        if not self._builder.robot.ros2_controls:
            raise RobotValidationError(
                ValidationErrorCode.VALUE_EMPTY,
                f"Joint '{joint.name}' requested ros2_control interfaces, but no global system exists.",
                target="Ros2Control",
            )

        target_system = None
        if self._control_system_name:
            for ctrl in self._builder.robot.ros2_controls:
                if ctrl.name == self._control_system_name:
                    target_system = ctrl
                    break
            if not target_system:
                raise RobotValidationError(
                    ValidationErrorCode.GENERIC_FAILURE,
                    f"Joint '{joint.name}' requested ros2_control system '{self._control_system_name}', but it was not found.",
                    target="Ros2Control",
                )
        else:
            target_system = self._builder.robot.ros2_controls[0]

        new_system = replace(
            target_system,
            joints=(
                *target_system.joints,
                Ros2ControlJoint(
                    name=joint.name,
                    command_interfaces=self._control_interfaces[0],
                    state_interfaces=self._control_interfaces[1],
                    parameters=self._control_interfaces[2],
                ),
            ),
        )
        self._builder.robot.update_ros2_control(new_system)
