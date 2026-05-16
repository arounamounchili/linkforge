"""RobotBuilder API for LinkForge.

Provides a **Fluent Builder Pattern** for intuitive, hierarchical construction
of robot kinematic trees (links and joints).

Core Components:
    - RobotBuilder: Entry point for assembly and global material/control config.
    - LinkBuilder: Fluent API for defining link visuals, collisions, and mass.
    - SemanticBuilder: Interface for SRDF/MoveIt semantic groups.
"""

from __future__ import annotations

from typing import Any

from .._utils.math_utils import normalize_vector
from ..constants import (
    CONTROL_TYPE_SYSTEM,
    SRDF_REASON_ADJACENT,
)
from ..exceptions import RobotModelError, RobotValidationError, ValidationErrorCode
from ..models.geometry import Transform, Vector3
from ..models.joint import JointLimits, JointType
from ..models.material import Color, Material
from ..models.robot import Robot
from ..models.ros2_control import Ros2Control
from .interfaces import IComposer
from .link_builder import LinkBuilder
from .semantic_builder import SemanticBuilder


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

        self._active_link_builders: list[LinkBuilder] = []

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
        component: Robot | IComposer,
        at_link: str,
        joint_name: str | None = None,
        prefix: str = "",
        joint_type: JointType = JointType.FIXED,
        xyz: tuple[float, float, float] = (0, 0, 0),
        rpy: tuple[float, float, float] = (0, 0, 0),
        axis: tuple[float, float, float] | None = None,
        limits: tuple[float, float] | None = None,
        disable_collision: bool = False,
        reason: str = SRDF_REASON_ADJACENT,
    ) -> RobotBuilder:
        """Merge another robot or assembly into the current one.

        Args:
            component: The Robot or IComposer (e.g. RobotBuilder) to attach.
            at_link: The link in the current robot to attach to.
            joint_name: Optional name for the connecting joint.
            prefix: Optional prefix for all links/joints in the component.
            joint_type: Type of connecting joint.
            xyz: Joint origin translation.
            rpy: Joint origin rotation.
            axis: Optional joint axis (automatically normalized).
            limits: Optional (lower, upper) joint limits.
            disable_collision: Whether to disable collision checking at the interface.
            reason: Semantic reason for disabling collisions (e.g., "Adjacent").

        Returns:
            The RobotBuilder instance.
        """
        sub_robot = component if isinstance(component, Robot) else component.robot
        root_link = sub_robot.root_link

        # Normalize axis if provided
        axis_vec = None
        if axis:
            nx, ny, nz = normalize_vector(*axis)
            if nx == 0.0 and ny == 0.0 and nz == 0.0:
                raise RobotValidationError(
                    ValidationErrorCode.OUT_OF_RANGE,
                    "Joint axis magnitude is too small",
                    target="RobotBuilder.attach",
                    value=0.0,
                )
            axis_vec = Vector3(nx, ny, nz)

        # Prepare limits if provided
        limits_obj = None
        if limits:
            limits_obj = JointLimits(lower=limits[0], upper=limits[1])

        self.robot.merge(
            component=sub_robot,
            at_link=at_link,
            joint_name=joint_name or f"{at_link}_to_{prefix}{root_link.name}",
            prefix=prefix,
            joint_type=joint_type,
            origin=Transform(xyz=Vector3(*xyz), rpy=Vector3(*rpy)),
            axis=axis_vec,
            limits=limits_obj,
        )

        if disable_collision:
            self.robot.disable_collisions(at_link, f"{prefix}{root_link.name}", reason=reason)

        return self

    def material(
        self, name: str, color: tuple[float, float, float, float] | None = None
    ) -> RobotBuilder:
        """Define a global material that can be reused by multiple links.

        Args:
            name: Unique material name.
            color: RGBA color as (r, g, b, a), each in range [0.0, 1.0].

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
        control_type: str = CONTROL_TYPE_SYSTEM,
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
        for lb in list(self._active_link_builders):
            lb._commit()
        self._active_link_builders.clear()

        if validate:
            # Trigger root search to verify connectivity (raises error if no root)
            _ = self.robot.root_link

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
