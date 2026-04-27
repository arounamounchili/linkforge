"""RobotBuilder API for LinkForge.

This module implements the 'Composer' which allows for both macro-assembly
(merging sub-robots) and micro-construction (programmatic link/joint building).
"""

from __future__ import annotations

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
from ..physics.inertia import calculate_inertia


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

    Attributes:
        robot: The underlying robot model being composed.
    """

    def __init__(self, name: str | None = None, robot: Robot | None = None) -> None:
        """Initialize a new robot builder.

        Args:
            name: Name of the new robot (if creating from scratch).
            robot: Existing robot model to build upon.
        """
        if robot is not None:
            self.robot = robot
        elif name is not None:
            self.robot = Robot(name=name)
        else:
            raise RobotModelError("Either name or robot must be provided")  # noqa: TRY003

    def material(
        self, name: str, color: tuple[float, float, float, float] | None = None
    ) -> RobotBuilder:
        """Add a global material to the robot."""
        color_obj = Color(*color) if color else None
        self.robot.materials[name] = Material(name=name, color=color_obj)
        return self

    def link(self, name: str, parent: str | None = None) -> LinkBuilder:
        """Start building a new link.

        Args:
            name: Unique name for the link.
            parent: Optional parent link name to connect to.
        """
        return LinkBuilder(self, name, parent=parent)

    def ros2_control(
        self,
        name: str,
        hardware_plugin: str,
        control_type: str = "system",
        parameters: dict[str, Any] | None = None,
    ) -> RobotBuilder:
        """Add a ros2_control system configuration."""
        params = {k: str(v) for k, v in (parameters or {}).items()}
        control = Ros2Control(
            name=name,
            type=control_type,
            hardware_plugin=hardware_plugin,
            parameters=params,
        )
        self.robot.add_ros2_control(control)
        return self

    def virtual_joint(
        self, name: str, child_link: str, parent_frame: str = "world", joint_type: str = "fixed"
    ) -> RobotBuilder:
        """Add a virtual joint (SRDF)."""
        vj = VirtualJoint(
            name=name, type=joint_type, parent_frame=parent_frame, child_link=child_link
        )
        self.robot.semantic.virtual_joints.append(vj)
        return self

    def group_state(self, name: str, group: str, values: dict[str, float]) -> RobotBuilder:
        """Add a group state (pose) for a MoveIt group."""
        gs = GroupState(name=name, group=group, joint_values=values)
        self.robot.semantic.group_states.append(gs)
        return self

    def end_effector(
        self, name: str, group: str, parent_link: str, parent_group: str | None = None
    ) -> RobotBuilder:
        """Add an end effector definition (SRDF)."""
        ee = EndEffector(name=name, group=group, parent_link=parent_link, parent_group=parent_group)
        self.robot.semantic.end_effectors.append(ee)
        return self

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
        """Attach a sub-robot component to the current assembly."""
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

    def add_group(
        self,
        name: str,
        links: list[str] | None = None,
        joints: list[str] | None = None,
        chains: list[tuple[str, str]] | None = None,
    ) -> RobotBuilder:
        """Add a planning group for MoveIt."""
        self.robot.add_group(name=name, links=links, joints=joints, chains=chains)
        return self

    def disable_collisions(self, link1: str, link2: str, reason: str = "Adjacent") -> RobotBuilder:
        """Disable collision checking between two links."""
        self.robot.disable_collisions(link1=link1, link2=link2, reason=reason)
        return self

    def export_urdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Export the assembled robot to URDF XML."""
        return self.robot.export_urdf(validate=validate, pretty_print=pretty_print)

    def export_srdf(self, validate: bool = True, pretty_print: bool = True) -> str:
        """Export the assembled semantic description to SRDF XML."""
        return self.robot.export_srdf(validate=validate, pretty_print=pretty_print)

    def build(self) -> Robot:
        """Return the completed Robot model."""
        return self.robot


class LinkBuilder:
    """Staged fluent builder for programmatic link and joint construction."""

    def __init__(
        self,
        builder: RobotBuilder,
        name: str,
        parent: str | None = None,
        joint_name: str | None = None,
    ) -> None:
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

        # Link accumulation
        self._mass: float | None = None
        self._inertia: InertiaTensor | None = None
        self._inertial_origin: Transform | None = None
        self._visuals: list[Visual] = []
        self._collisions: list[Collision] = []
        self._sensors: list[Sensor] = []

    def visual(
        self,
        geometry: Geometry,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
        material: str | Material | None = None,
        name: str | None = None,
    ) -> LinkBuilder:
        """Add a visual element to the link."""
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
        """Add a collision element to the link.

        If no geometry is provided, it clones the last visual element's geometry and origin.
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
        """Set the mass and optional inertial properties."""
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
        """Manually set the inertia tensor."""
        self._inertia = InertiaTensor(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz)
        return self

    # Joint Configuration (only relevant if it has a parent)

    def at_origin(
        self,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
    ) -> LinkBuilder:
        """Set the joint origin (transform from parent to child)."""
        self._joint_origin = Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy))
        return self

    def fixed(
        self,
        name: str | None = None,
        xyz: tuple[float, float, float] | None = None,
        rpy: tuple[float, float, float] | None = None,
    ) -> LinkBuilder:
        """Set joint type to FIXED."""
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
        """Set joint type to REVOLUTE."""
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
        """Set joint type to CONTINUOUS."""
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
        """Set joint type to PRISMATIC."""
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

    def transmission(
        self,
        reduction: float = 1.0,
        interface: str = "effort",
        actuator: str | None = None,
        name: str | None = None,
    ) -> LinkBuilder:
        """Configure a transmission for the current joint."""
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
        """Configure ros2_control interfaces for the current joint."""
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
        """Attach a camera sensor to this link."""
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
        """Attach a lidar sensor to this link."""
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
        """Attach an IMU sensor to this link."""
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
        """Attach a GPS sensor to this link."""
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
        """Attach a pre-configured sensor to this link."""
        self._sensors.append(sensor)
        return self

    # Kinematic Branching

    def child(self, name: str, joint_name: str | None = None) -> LinkBuilder:
        """Finalize this link and start a new child link."""
        self._commit()
        return LinkBuilder(self._builder, name, parent=self._link.name, joint_name=joint_name)

    def commit(self) -> RobotBuilder:
        """Finalize this link and return to the RobotBuilder."""
        self._commit()
        return self._builder

    def root(self) -> RobotBuilder:
        """Finalize this link as the root (no joint) and return to RobotBuilder."""
        if self._parent:
            raise RobotValidationError(
                ValidationErrorCode.GENERIC_FAILURE,
                f"Link '{self._link.name}' has a parent '{self._parent}' and cannot be root",
                target="LinkBuilder",
            )
        return self.commit()

    def build(self) -> Robot:
        """Finalize this link and return the completed Robot model."""
        self._commit()
        return self._builder.build()

    def _commit(self) -> None:
        """Internal method to flush staged link/joint to the Robot model."""
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
                from ..models.transmission import Transmission

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
                if not self._builder.robot._ros2_controls:
                    raise RobotValidationError(
                        ValidationErrorCode.VALUE_EMPTY,
                        f"Joint '{joint.name}' requested ros2_control interfaces, but no global ros2_control system was defined. Call builder.ros2_control() first.",
                        target="Ros2Control",
                    )

                # Add joint to the first control system
                ctrl = self._builder.robot._ros2_controls[0]
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
