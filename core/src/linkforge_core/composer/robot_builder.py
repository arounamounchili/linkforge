"""RobotBuilder API for LinkForge.

This module implements the 'Composer' layer—the primary interface for
programmatically defining, assembling, and merging robots.

The API uses a **Fluent Builder Pattern** that allows for intuitive,
hierarchical construction of robot trees (links and joints) as well as
high-level assembly of pre-existing sub-components.

Example:
    >>> builder = RobotBuilder("my_robot")
    >>> (
    ...     builder.link("base_link")
    ...     .visual(box(1, 1, 1))
    ...     .child("arm")
    ...         .revolute(axis=(0, 0, 1), limits=(0, 3.14))
    ...         .commit()
    ... )
    >>> robot = builder.build()
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from ..exceptions import RobotModelError, RobotValidationError, ValidationErrorCode
from ..models.geometry import Box, Cylinder, Geometry, Mesh, Sphere, Transform, Vector3
from ..models.joint import Joint, JointLimits, JointType
from ..models.link import Collision, Inertial, InertiaTensor, Link, Visual
from ..models.material import Color, Material
from ..models.robot import Robot
from ..models.ros2_control import Ros2Control, Ros2ControlJoint
from ..models.sensor import CameraInfo, GPSInfo, IMUInfo, LidarInfo, Sensor, SensorType
from ..models.srdf import EndEffector, GroupState, VirtualJoint
from ..models.transmission import Transmission
from ..physics.inertia import calculate_inertia

# --- Geometry Helpers ---


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
            raise RobotModelError("Either name or robot must be provided")  # noqa: TRY003

    # --- Core Construction ---

    def link(self, name: str, parent: str | None = None) -> LinkBuilder:
        """Start building a new link programmatically.

        Args:
            name: Unique name for the link.
            parent: Optional parent link name to connect to immediately.

        Returns:
            A LinkBuilder instance for fluent construction.
        """
        return LinkBuilder(self, name, parent=parent)

    def attach(
        self,
        component: Robot,
        at_link: str,
        joint_name: str,
        prefix: str = "",
        joint_type: JointType = JointType.FIXED,
        origin: Transform | None = None,
        axis: Vector3 | None = None,
        limits: JointLimits | None = None,
    ) -> RobotBuilder:
        """Merge a sub-robot component into the current assembly.

        Args:
            component: The robot model to attach.
            at_link: The link in the current assembly to attach to.
            joint_name: Name of the joint connecting the assembly to the component.
            prefix: Optional prefix for all elements in the component.
            joint_type: Type of the connecting joint (default: FIXED).
            origin: Optional transform for the joint.
            axis: Optional joint axis.
            limits: Optional joint limits.

        Returns:
            The RobotBuilder instance for chaining.
        """
        self.robot.merge(
            component=component,
            at_link=at_link,
            joint_name=joint_name,
            prefix=prefix,
            joint_type=joint_type,
            origin=origin,
            axis=axis,
            limits=limits,
        )
        return self

    # --- System Setup ---

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

    # --- Semantic Setup (SRDF) ---

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
            The RobotBuilder instance.
        """
        vj = VirtualJoint(
            name=name, type=joint_type, parent_frame=parent_frame, child_link=child_link
        )
        self.robot.semantic.virtual_joints.append(vj)
        return self

    def add_group(
        self,
        name: str,
        links: list[str] | None = None,
        joints: list[str] | None = None,
        chains: list[tuple[str, str]] | None = None,
    ) -> RobotBuilder:
        """Add a planning group for MoveIt motion planning.

        Args:
            name: Unique group name (e.g., 'arm', 'gripper').
            links: List of link names in the group.
            joints: List of joint names in the group.
            chains: List of (base_link, tip_link) tuples.

        Returns:
            The RobotBuilder instance.
        """
        self.robot.add_group(name=name, links=links, joints=joints, chains=chains)
        return self

    def group_state(self, name: str, group: str, values: dict[str, float]) -> RobotBuilder:
        """Define a named pose (group state) for a planning group.

        Args:
            name: Unique pose name (e.g., 'home', 'ready').
            group: The target planning group.
            values: Map of joint names to their target positions.

        Returns:
            The RobotBuilder instance.
        """
        gs = GroupState(name=name, group=group, joint_values=values)
        self.robot.semantic.group_states.append(gs)
        return self

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
            The RobotBuilder instance.
        """
        ee = EndEffector(name=name, group=group, parent_link=parent_link, parent_group=parent_group)
        self.robot.semantic.end_effectors.append(ee)
        return self

    def disable_collisions(self, link1: str, link2: str, reason: str = "Adjacent") -> RobotBuilder:
        """Instruct MoveIt to ignore collisions between two specific links.

        Args:
            link1: First link name.
            link2: Second link name.
            reason: Why collisions are disabled (default: 'Adjacent').

        Returns:
            The RobotBuilder instance.
        """
        self.robot.disable_collisions(link1=link1, link2=link2, reason=reason)
        return self

    # --- Finalization & Export ---

    def build(self) -> Robot:
        """Finalize the building process and return the Robot model.

        Returns:
            The completed Robot model.
        """
        return self.robot

    def export_urdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Generate the URDF XML representation of the robot.

        Args:
            validate: Whether to run kinematic and physical validation.
            pretty_print: Whether to format the XML with indentation.

        Returns:
            A URDF XML string.
        """
        return self.robot.export_urdf(validate=validate, pretty_print=pretty_print)

    def export_srdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Generate the SRDF XML representation of the robot.

        Args:
            validate: Whether to validate the semantic description.
            pretty_print: Whether to format the XML with indentation.

        Returns:
            An SRDF XML string.
        """
        return self.robot.export_srdf(validate=validate, pretty_print=pretty_print)


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
        self._link = Link(name=name)
        self._name = name
        self._parent = parent
        self._joint_name = joint_name

        # Joint configuration (if we have a parent)
        self._joint_type: JointType = JointType.FIXED
        self._joint_origin: Transform = Transform.identity()
        self._joint_axis: Vector3 = Vector3(0, 0, 1)
        self._joint_limits: JointLimits | None = None
        self._transmission_params: dict[str, Any] | None = None
        self._control_interfaces: tuple[list[str], list[str], dict[str, Any]] | None = None
        self._committed = False

        # Link physical state
        self._mass: float | None = None
        self._inertia: InertiaTensor | None = None
        self._inertial_origin: Transform | None = None
        self._visuals: list[Visual] = []
        self._collisions: list[Collision] = []
        self._sensors: list[Sensor] = []

    # --- Physical Body ---

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
        self._link.add_visual(Visual(geometry=geometry, origin=origin, material=mat, name=name))
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
                    value=self._link.name,
                )
            last_visual = self._link.visuals[-1]
            geometry = last_visual.geometry
            origin = last_visual.origin
        else:
            origin = Transform(
                xyz=Vector3(*(xyz or (0, 0, 0))),
                rpy=Vector3(*(rpy or (0, 0, 0))),
            )

        self._link.add_collision(Collision(geometry=geometry, origin=origin, name=name))
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
        self._mass = value
        if inertia:
            self._inertia = inertia
        if origin_xyz or origin_rpy:
            self._inertial_origin = Transform(
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
        self._inertia = InertiaTensor(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz)
        return self

    # --- Joint Configuration ---

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
        self._joint_origin = Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy))
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
        self._joint_type = JointType.FIXED
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
        self._joint_type = JointType.REVOLUTE
        self._joint_axis = Vector3(*axis)
        self._joint_limits = JointLimits(lower=limits[0], upper=limits[1], effort=0, velocity=0)
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
        self._joint_type = JointType.CONTINUOUS
        self._joint_axis = Vector3(*axis)
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
        self._joint_type = JointType.PRISMATIC
        self._joint_axis = Vector3(*axis)
        self._joint_limits = JointLimits(lower=limits[0], upper=limits[1], effort=0, velocity=0)
        return self._configure_joint(name, xyz, rpy)

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
            self._joint_origin = Transform(
                xyz=Vector3(*(xyz or (0, 0, 0))), rpy=Vector3(*(rpy or (0, 0, 0)))
            )
        return self

    # --- Control & Actuation ---

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
            "actuator": actuator or f"actuator_{self._name}",
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

    # --- Sensors ---

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
            link_name=self._name,
            camera_info=info,
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        self._sensors.append(sensor)
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
            link_name=self._name,
            lidar_info=info,
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        self._sensors.append(sensor)
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
            link_name=self._name,
            update_rate=update_rate,
            imu_info=IMUInfo(),
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        self._sensors.append(sensor)
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
            link_name=self._name,
            update_rate=update_rate,
            gps_info=GPSInfo(),
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
        )
        self._sensors.append(sensor)
        return self

    def sensor(self, sensor: Sensor) -> LinkBuilder:
        """Attach a pre-configured Sensor object to this link.

        Returns:
            The LinkBuilder instance.
        """
        self._sensors.append(replace(sensor, link_name=self._name))
        return self

    # --- Navigation & Lifecycle ---

    def child(self, name: str, joint_name: str | None = None) -> LinkBuilder:
        """Finalize this link and start building a new child link attached to it.

        Returns:
            A new LinkBuilder instance for the child link.
        """
        self._commit()
        return LinkBuilder(self._builder, name, parent=self._link.name, joint_name=joint_name)

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
                f"Link '{self._link.name}' has a parent '{self._parent}' and cannot be root",
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
        if self._mass is not None:
            if self._inertia is None:
                # Auto-calculate inertia if mass is provided but tensor isn't
                source_geometry = None
                source_origin = Transform.identity()

                if self._link.visuals:
                    source_geometry = self._link.visuals[0].geometry
                    source_origin = self._link.visuals[0].origin
                elif self._link.collisions:
                    source_geometry = self._link.collisions[0].geometry
                    source_origin = self._link.collisions[0].origin

                if source_geometry:
                    self._inertia = calculate_inertia(source_geometry, self._mass)
                    if self._inertial_origin is None:
                        self._inertial_origin = source_origin
                else:
                    self._inertia = InertiaTensor.zero()

            self._link.inertial = Inertial(
                mass=self._mass,
                inertia=self._inertia,
                origin=self._inertial_origin or Transform.identity(),
            )

        # 2. Add Link
        self._builder.robot.add_link(self._link)

        # 3. Add Joint (if parent exists)
        if self._parent:
            joint_name = self._joint_name or f"{self._parent}_to_{self._link.name}"
            joint = Joint(
                name=joint_name,
                type=self._joint_type,
                parent=self._parent,
                child=self._link.name,
                origin=self._joint_origin,
                axis=self._joint_axis if self._joint_type != JointType.FIXED else None,
                limits=self._joint_limits,
            )
            self._builder.robot.add_joint(joint)

            # 4. Add Transmission (if configured)
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

            # 5. Add ROS2 Control (if configured)
            if self._control_interfaces:
                if not self._builder.robot.ros2_controls:
                    raise RobotValidationError(
                        ValidationErrorCode.VALUE_EMPTY,
                        f"Joint '{joint.name}' requested ros2_control interfaces, but no global ros2_control system was defined. Call builder.ros2_control() first.",
                        target="Ros2Control",
                    )

                # Add joint to the first control system
                ctrl = self._builder.robot.ros2_controls[0]
                ctrl.joints.append(
                    Ros2ControlJoint(
                        name=joint.name,
                        command_interfaces=self._control_interfaces[0],
                        state_interfaces=self._control_interfaces[1],
                        parameters=self._control_interfaces[2],
                    )
                )

        # 6. Add Sensors
        for sensor in self._sensors:
            self._builder.robot.add_sensor(sensor)

        self._committed = True
