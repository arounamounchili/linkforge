"""RobotBuilder API for LinkForge.

This module implements the 'Composer' layer—the primary interface for
programmatically defining, assembling, and merging robots.

The API uses a **Fluent Builder Pattern** that allows for intuitive,
hierarchical construction of robot trees (links and joints) as well as
high-level assembly of pre-existing sub-components.

Examples:
    >>> builder = RobotBuilder("my_robot")
    >>> (
    ...     builder.link("base_link")
    ...         .visual(box(0.5, 0.5, 0.2), material="blue")
    ...         .collision()  # Auto-clones visual geometry
    ...         .mass(10.0)   # Auto-calculates inertia
    ...     .child("arm_link", xyz=(0, 0, 0.1))
    ...         .revolute(axis=(0, 0, 1), limits=(-1.57, 1.57))
    ...         .dynamics(damping=0.5)
    ...         .commit()
    ... )
    >>> # Define semantic properties
    >>> builder.semantic.group("arm", links=["arm_link"])
    >>>
    >>> # Finalize with validation
    >>> robot = builder.build(validate=True)
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any

from ..exceptions import RobotModelError, RobotValidationError, ValidationErrorCode
from ..models.gazebo import GazeboElement
from ..models.geometry import Box, Cylinder, Geometry, Mesh, Sphere, Transform, Vector3
from ..models.joint import (
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafetyController,
    JointType,
)
from ..models.link import Collision, Inertial, InertiaTensor, Link, Visual
from ..models.material import Color, Material
from ..models.robot import Robot
from ..models.ros2_control import Ros2Control, Ros2ControlJoint
from ..models.sensor import CameraInfo, GPSInfo, IMUInfo, LidarInfo, Sensor, SensorType
from ..models.srdf import (
    DisabledCollision,
    EndEffector,
    GroupState,
    PassiveJoint,
    PlanningGroup,
    VirtualJoint,
)
from ..models.transmission import Transmission
from ..physics.inertia import calculate_inertia


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
    gazebo_params: dict[str, Any] = field(default_factory=dict)


def box(x: float, y: float, z: float) -> Box:
    """Helper to create Box geometry."""
    return Box(size=Vector3(x, y, z))


def cylinder(radius: float, length: float) -> Cylinder:
    """Helper to create Cylinder geometry."""
    return Cylinder(radius=radius, length=length)


def sphere(radius: float) -> Sphere:
    """Helper to create Sphere geometry."""
    return Sphere(radius=radius)


def mesh(resource: str, scale: tuple[float, float, float] = (1.0, 1.0, 1.0)) -> Mesh:
    """Helper to create Mesh geometry."""
    return Mesh(resource=resource, scale=Vector3(*scale))


class RobotBuilder:
    """A high-level API to compose robots programmatically.

    This class serves as the entry point for building robots. It can create
    new robots from scratch or modify existing ones by adding links or
    attaching sub-components.
    """

    def __init__(self, name: str | None = None, robot: Robot | None = None) -> None:
        """Initialize a new robot builder.

        Args:
            name: Name of the new robot (required if robot is None).
            robot: Existing robot model to build upon.

        Raises:
            RobotModelError: If neither name nor robot is provided.
        """
        if robot is not None:
            self.robot = robot
        elif name is not None:
            self.robot = Robot(name=name)
        else:
            msg = "Either name or robot must be provided"
            raise RobotModelError(msg)

    def link(
        self, name: str, parent: str | None = None, joint_name: str | None = None
    ) -> LinkBuilder:
        """Start building a new link programmatically.

        Args:
            name: Unique name for the link.
            parent: Optional parent link name to connect to immediately.
            joint_name: Optional explicit name for the connecting joint.

        Returns:
            A LinkBuilder instance for fluent construction.
        """
        return LinkBuilder(self, name, parent=parent, joint_name=joint_name)

    def attach(
        self,
        component: Robot | RobotBuilder,
        at_link: str,
        joint_name: str | None = None,
        prefix: str = "",
        joint_type: JointType = JointType.FIXED,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
    ) -> RobotBuilder:
        """Merge another robot or assembly into the current one.

        Args:
            component: The Robot or RobotBuilder to attach.
            at_link: The link in the current robot to attach to.
            joint_name: Optional name for the connecting joint.
            prefix: Optional prefix for all links/joints in the component.
            joint_type: Type of connecting joint.
            xyz: Joint origin translation.
            rpy: Joint origin rotation.

        Returns:
            The RobotBuilder instance.
        """
        sub_robot = component.robot if isinstance(component, RobotBuilder) else component
        self.robot.merge(
            component=sub_robot,
            at_link=at_link,
            joint_name=joint_name or f"{at_link}_to_{prefix}{sub_robot.get_root_link().name}",
            prefix=prefix,
            joint_type=joint_type,
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        return self

    def material(
        self, name: str, color: tuple[float, float, float, float] | None = None
    ) -> RobotBuilder:
        """Define a global material that can be reused by multiple links.

        Args:
            name: Unique material name.
            color: RGBA color as (r, g, b, a).

        Returns:
            The RobotBuilder instance.
        """
        color_obj = Color(*color) if color else None
        self.robot.materials[name] = Material(name=name, color=color_obj)
        return self

    def ros2_control(
        self,
        name: str,
        hardware_plugin: str,
        control_type: str = "system",
        parameters: dict[str, Any] | None = None,
    ) -> RobotBuilder:
        """Add a global ros2_control system configuration.

        Args:
            name: Unique system name.
            hardware_plugin: The hardware interface plugin (e.g. 'fake_components/GenericSystem').
            control_type: Type of control (usually 'system').
            parameters: Key-value parameters for the hardware interface.

        Returns:
            The RobotBuilder instance.
        """
        params = {k: str(v) for k, v in (parameters or {}).items()}
        control = Ros2Control(
            name=name,
            type=control_type,
            hardware_plugin=hardware_plugin,
            parameters=params,
        )
        self.robot.add_ros2_control(control)
        return self

    @property
    def semantic(self) -> SemanticBuilder:
        """Access the semantic (SRDF/MoveIt) construction API.

        Example:
            >>> builder.semantic.group("arm", links=["link1", "link2"])
        """
        return SemanticBuilder(self)

    def build(self, validate: bool = True) -> Robot:
        """Finalize the assembly and return the completed Robot model.

        Args:
            validate: If True, performs a kinematic check for disconnected links or cycles.

        Returns:
            The completed Robot object.

        Raises:
            RobotValidationError: If validation is requested and the robot is invalid.
        """
        if validate:
            # Trigger root search to verify connectivity (raises error if no root)
            self.robot.get_root_link()

            if self.robot.has_cycle:
                raise RobotValidationError(
                    ValidationErrorCode.HAS_CYCLE,
                    "Robot kinematic chain contains a cycle (not supported in URDF)",
                    target="KinematicTree",
                )

        return self.robot

    def export_urdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Generate the URDF XML representation of the robot.

        Args:
            validate: Whether to run internal LinkForge validation.
            pretty_print: Whether to format the XML with indentation.

        Returns:
            A URDF XML string.
        """
        if validate:
            self.build(validate=True)
        return self.robot.export_urdf(validate=validate, pretty_print=pretty_print)

    def export_srdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Generate the SRDF XML representation of the robot.

        Args:
            validate: Whether to validate the semantic description.
            pretty_print: Whether to format the XML with indentation.

        Returns:
            An SRDF XML string.
        """
        if validate:
            self.build(validate=True)
        return self.robot.export_srdf(validate=validate, pretty_print=pretty_print)


class SemanticBuilder:
    """Namespace for SRDF and MoveIt-specific semantic properties.

    Accessed via RobotBuilder.semantic.
    """

    def __init__(self, builder: RobotBuilder) -> None:
        """Initialize semantic builder."""
        self._builder = builder

    def group(
        self,
        name: str,
        links: list[str] | None = None,
        joints: list[str] | None = None,
        chains: list[tuple[str, str]] | None = None,
    ) -> RobotBuilder:
        """Define a planning group for MoveIt.

        Args:
            name: Unique name for the group.
            links: List of link names to include.
            joints: List of joint names to include.
            chains: List of (base, tip) tuples for kinematic chains.

        Returns:
            The parent RobotBuilder instance.
        """
        # Define planning group

        group = PlanningGroup(
            name=name,
            links=links or [],
            joints=joints or [],
            chains=chains or [],
        )
        self._builder.robot.semantic.groups.append(group)
        return self._builder

    def group_state(self, name: str, group: str, values: dict[str, float]) -> RobotBuilder:
        """Define a named state (e.g. 'home') for a planning group.

        Args:
            name: Unique name for the state.
            group: The group this state belongs to.
            values: Dictionary of joint names and their positions.

        Returns:
            The parent RobotBuilder instance.
        """
        state = GroupState(name=name, group=group, joint_values=values)
        self._builder.robot.semantic.group_states.append(state)
        return self._builder

    def end_effector(
        self, name: str, group: str, parent_link: str, parent_group: str | None = None
    ) -> RobotBuilder:
        """Define an end effector for MoveIt.

        Args:
            name: Unique name for the end effector.
            group: The planning group representing the end effector.
            parent_link: The link it is attached to.
            parent_group: Optional parent group (e.g. 'arm').

        Returns:
            The parent RobotBuilder instance.
        """
        ee = EndEffector(name=name, group=group, parent_link=parent_link, parent_group=parent_group)
        self._builder.robot.semantic.end_effectors.append(ee)
        return self._builder

    def passive_joint(self, name: str) -> RobotBuilder:
        """Mark a joint as passive (not actuated) for MoveIt.

        Args:
            name: Name of the joint to mark as passive.

        Returns:
            The parent RobotBuilder instance.
        """
        self._builder.robot.semantic.passive_joints.append(PassiveJoint(name=name))
        return self._builder

    def virtual_joint(
        self, name: str, child_link: str, parent_frame: str = "world", joint_type: str = "fixed"
    ) -> RobotBuilder:
        """Define a virtual joint connecting the robot to the world frame.

        Args:
            name: Unique joint name.
            child_link: The root link of the robot.
            parent_frame: The external frame (e.g., 'world', 'map').
            joint_type: Joint type (fixed, floating, planar).

        Returns:
            The parent RobotBuilder instance.
        """
        vj = VirtualJoint(
            name=name, type=joint_type, parent_frame=parent_frame, child_link=child_link
        )
        self._builder.robot.semantic.virtual_joints.append(vj)
        return self._builder

    def disable_collisions(self, link1: str, link2: str, reason: str = "Adjacent") -> RobotBuilder:
        """Instruct MoveIt to ignore collisions between two specific links.

        Args:
            link1, link2: Names of the links.
            reason: Explanation for disabling (e.g. 'Adjacent', 'Never').

        Returns:
            The parent RobotBuilder instance.
        """
        self._builder.robot.semantic.disabled_collisions.append(
            DisabledCollision(link1=link1, link2=link2, reason=reason)
        )
        return self._builder


class LinkBuilder:
    """Staged fluent builder for programmatic link and joint construction.

    This builder accumulates link and joint properties in stages. It is usually
    returned by builder.link() or link_builder.child().
    """

    def __init__(
        self,
        builder: RobotBuilder,
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
        self._committed = False

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
        origin = Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy))
        mat = self._builder.robot.materials.get(material) if isinstance(material, str) else material
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
        self._joint.type = JointType.FIXED
        return self._configure_joint(name, xyz, rpy)

    def revolute(
        self,
        axis: tuple[float, float, float],
        limits: tuple[float, float],
        name: str | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
    ) -> LinkBuilder:
        """Configure the connection as a REVOLUTE (limited rotation) joint.

        Args:
            axis: Rotation axis unit vector.
            limits: (lower, upper) joint limits in radians.
            name: Unique joint name.
            xyz: Joint origin translation.
            rpy: Joint origin rotation.

        Returns:
            The LinkBuilder instance.
        """
        self._joint.type = JointType.REVOLUTE
        self._joint.axis = Vector3(*axis)
        self._joint.limits = JointLimits(lower=limits[0], upper=limits[1], effort=0, velocity=0)
        return self._configure_joint(name, xyz, rpy)

    def continuous(
        self,
        axis: tuple[float, float, float],
        name: str | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
    ) -> LinkBuilder:
        """Configure the connection as a CONTINUOUS (unlimited rotation) joint.

        Args:
            axis: Rotation axis unit vector.
            name: Unique joint name.
            xyz: Joint origin translation.
            rpy: Joint origin rotation.

        Returns:
            The LinkBuilder instance.
        """
        self._joint.type = JointType.CONTINUOUS
        self._joint.axis = Vector3(*axis)
        return self._configure_joint(name, xyz, rpy)

    def prismatic(
        self,
        axis: tuple[float, float, float],
        limits: tuple[float, float],
        name: str | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
    ) -> LinkBuilder:
        """Configure the connection as a PRISMATIC (linear sliding) joint.

        Args:
            axis: Translation axis unit vector.
            limits: (lower, upper) joint limits in meters.
            name: Unique joint name.
            xyz: Joint origin translation.
            rpy: Joint origin rotation.

        Returns:
            The LinkBuilder instance.
        """
        self._joint.type = JointType.PRISMATIC
        self._joint.axis = Vector3(*axis)
        self._joint.limits = JointLimits(lower=limits[0], upper=limits[1], effort=0, velocity=0)
        return self._configure_joint(name, xyz, rpy)

    def dynamics(self, damping: float = 0.0, friction: float = 0.0) -> LinkBuilder:
        """Set the physical dynamics for the joint.

        Args:
            damping: Damping coefficient.
            friction: Friction coefficient.

        Returns:
            The LinkBuilder instance.
        """
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
        self._joint.mimic = JointMimic(joint=joint, multiplier=multiplier, offset=offset)
        return self

    def safety(
        self,
        soft_lower: float | None = None,
        soft_upper: float | None = None,
        k_position: float | None = None,
        k_velocity: float | None = None,
    ) -> LinkBuilder:
        """Define a safety controller for the joint.

        Args:
            soft_lower, soft_upper: Software limits.
            k_position, k_velocity: Controller gains.

        Returns:
            The LinkBuilder instance.
        """
        self._joint.safety = JointSafetyController(
            soft_lower_limit=soft_lower or 0.0,
            soft_upper_limit=soft_upper or 0.0,
            k_position=k_position or 0.0,
            k_velocity=k_velocity or 0.0,
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
        self._joint.calibration = JointCalibration(rising=rising, falling=falling)
        return self

    def simulation(self, **kwargs: Any) -> LinkBuilder:
        """Set Gazebo-specific simulation properties for this link.

        Common arguments:
            self_collide (bool): Enable self-collision.
            gravity (bool): Enable gravity.
            static (bool): Mark link as static.
            mu1, mu2 (float): Friction coefficients.
            kp, kd (float): Contact stiffness and damping.

        Returns:
            The LinkBuilder instance.
        """
        self._link.gazebo_params.update(kwargs)
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

    def transmission(
        self,
        reduction: float = 1.0,
        interface: str = "effort",
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
    ) -> LinkBuilder:
        """Configure ros2_control interfaces for the current joint.

        Args:
            command_interfaces: List of allowed commands (e.g. ['position']).
            state_interfaces: List of exposed states (e.g. ['position', 'velocity']).
            parameters: Key-value parameters for the joint control.

        Returns:
            The LinkBuilder instance.
        """
        params = {k: str(v) for k, v in (parameters or {}).items()}
        self._control_interfaces = (command_interfaces, state_interfaces, params)
        return self

    def camera(
        self,
        name: str,
        fov: float = 1.047,
        width: int = 640,
        height: int = 480,
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
        range_min: float = 0.1,
        range_max: float = 10.0,
        samples: int = 640,
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

    def imu(
        self,
        name: str,
        update_rate: float = 100.0,
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
        update_rate: float = 5.0,
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
        update_rate: float = 100.0,
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
        sensor = Sensor(
            name=name,
            type=SensorType.FORCE_TORQUE,
            link_name=self._link_name,
            update_rate=update_rate,
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        self._link.sensors.append(sensor)
        return self

    def contact(
        self,
        name: str,
        collision: str,
        update_rate: float = 50.0,
    ) -> LinkBuilder:
        """Attach a contact sensor to this link.

        Args:
            name: Unique sensor name.
            collision: The name of the collision element to monitor.
            update_rate: Sampling rate in Hz.

        Returns:
            The LinkBuilder instance.
        """
        from linkforge_core.models.sensor import ContactInfo

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

    def commit(self) -> RobotBuilder:
        """Finalize this link and return to the main RobotBuilder.

        Returns:
            The parent RobotBuilder instance.
        """
        self._commit()
        return self._builder

    def root(self) -> RobotBuilder:
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

        # 1. Handle Inertial properties
        l_state = self._link
        if l_state.mass is not None:
            if l_state.inertia is None:
                # Auto-calculate inertia if mass is provided but tensor isn't
                source_geometry = None
                source_origin = Transform.identity()

                if l_state.collisions:
                    # Prioritize collision geometry for inertia
                    source_geometry = l_state.collisions[0].geometry
                    source_origin = l_state.collisions[0].origin
                elif l_state.visuals:
                    # Fallback to visual geometry
                    source_geometry = l_state.visuals[0].geometry
                    source_origin = l_state.visuals[0].origin

                if source_geometry:
                    l_state.inertia = calculate_inertia(source_geometry, l_state.mass)
                    if l_state.inertial_origin is None:
                        l_state.inertial_origin = source_origin
                else:
                    l_state.inertia = InertiaTensor.zero()

            inertial = Inertial(
                mass=l_state.mass,
                inertia=l_state.inertia,
                origin=l_state.inertial_origin or Transform.identity(),
            )
        else:
            inertial = None

        # 2. Finalize Link
        link = Link(
            name=self._link_name,
            initial_visuals=l_state.visuals,
            initial_collisions=l_state.collisions,
            inertial=inertial,
        )
        self._builder.robot.add_link(link)

        # 3. Add Sensors to Robot
        for sensor in l_state.sensors:
            self._builder.robot.add_sensor(sensor)

        # 4. Handle Simulation (Gazebo) Properties
        if l_state.gazebo_params:
            # Add gazebo element
            gz = GazeboElement(reference=self._link_name, **l_state.gazebo_params)
            self._builder.robot.add_gazebo_element(gz)

        # 5. Finalize Joint (if parent exists)
        if self._parent:
            j_state = self._joint
            joint_name = self._joint_name or f"{self._link_name}_joint"
            # For FIXED joints, we surgically ensure no motion-related properties are passed
            is_fixed = j_state.type == JointType.FIXED

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

            # 6. Finalize Transmission
            if self._transmission_params:
                t_name = self._transmission_params["name"] or f"trans_{joint.name}"
                trans = Transmission.create_simple(
                    name=t_name,
                    joint_name=joint.name,
                    actuator_name=self._transmission_params["actuator"],
                    mechanical_reduction=self._transmission_params["reduction"],
                    hardware_interface=self._transmission_params["interface"],
                )
                self._builder.robot.add_transmission(trans)

            # 7. Finalize ROS2 Control
            if self._control_interfaces:
                if not self._builder.robot.ros2_controls:
                    raise RobotValidationError(
                        ValidationErrorCode.VALUE_EMPTY,
                        f"Joint '{joint.name}' requested ros2_control interfaces, but no global system exists.",
                        target="Ros2Control",
                    )
                ctrl = self._builder.robot.ros2_controls[0]
                ctrl.joints.append(
                    Ros2ControlJoint(
                        name=joint.name,
                        command_interfaces=self._control_interfaces[0],
                        state_interfaces=self._control_interfaces[1],
                        parameters=self._control_interfaces[2],
                    )
                )

        self._committed = True
